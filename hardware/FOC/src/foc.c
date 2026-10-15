/**
 * @file foc.c
 * @brief FOC 纯算法层 (Clarke/Park/SVPWM/PID/三环状态机)
 *
 * 零硬件依赖: 不include任何MCU头文件, 只做浮点运算
 * bldc.c的ADC中断调用FOC_Update(), 取回占空比后写TIM1
 *
 * 数据流 (20kHz电流环):
 *   ia,ib,θm → Clarke → Park(θe) → [位置环→速度环→]电流PI → 反Park → SVPWM → da,db,dc
 *
 * 速度环 2kHz (每10个电流环), 位置环 1kHz (每20个电流环)
 *
 * @note CH32V303 (RV32IMAC) 无FPU, 全部soft-float; sinf/cosf约1µs/次,
 *       全流程约3~5µs, 50µs周期(20kHz)内安全
 */
#include "foc.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif
#define SQ3   1.7320508f
#define _2PI  (2.0f * M_PI)
#define DEG2RAD (M_PI / 180.0f)
#define RAD2DEG (180.0f / M_PI)

// ==================== 共享变量定义 ====================
volatile FOC_Mode foc_mode = FOC_MODE_IDLE;
volatile float foc_target = 0.0f;
volatile float foc_actual_position = 0.0f;
volatile float foc_actual_speed = 0.0f;
volatile float foc_actual_iq = 0.0f;
volatile float foc_actual_id = 0.0f;

// ==================== 内部状态 ====================
static FOC_PID pid_iq, pid_id, pid_spd;
static float   iq_ref, id_ref;       // 电流参考
static float   spd_ref;              // 速度参考 (内环目标)
static float   spd_target_ramped;    // 斜坡后的速度目标
static float   theta_m_prev;        // 上一周期机械角 (速度微分用)
static float   spd_filt;             // 滤波后速度 RPM
static uint16_t cycle_cnt;           // 电流环周期计数
static float   ol_angle;             // 开环角度积分 rad

// 电流环周期
#define DT_I   (1.0f / FOC_PWM_FREQ)           // 50µs
#define DT_S   (10.0f / FOC_PWM_FREQ)          // 500µs = 2kHz
#define DT_P   (20.0f / FOC_PWM_FREQ)          // 1ms = 1kHz

// ==================== PID ====================

void FOC_PID_Init(FOC_PID *p, float kp, float ki, float kd, float out_max, float int_max)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->out_max = out_max; p->int_max = int_max;
    p->integral = 0; p->prev_err = 0;
}

void FOC_PID_Reset(FOC_PID *p)
{
    p->integral = 0; p->prev_err = 0;
}

float FOC_PID_Calc(FOC_PID *p, float err, float dt)
{
    float out;
    p->integral += err * dt;
    // 积分限幅 (clamping抗饱和)
    if (p->integral >  p->int_max) p->integral =  p->int_max;
    if (p->integral < -p->int_max) p->integral = -p->int_max;

    out = p->kp * err + p->ki * p->integral + p->kd * (err - p->prev_err) / dt;
    p->prev_err = err;

    // 输出限幅
    if (out >  p->out_max) out =  p->out_max;
    if (out < -p->out_max) out = -p->out_max;
    return out;
}

// ==================== 数学变换 ====================

void foc_clarke(float ia, float ib, float *alpha, float *beta)
{
    // ic = -(ia + ib)
    *alpha = ia;
    *beta  = (ia + 2.0f * ib) / SQ3;  // = (ia + 2ib)/√3, 等价于 √3/2·(ia+2ib)/... 标准Clarke
}

void foc_park(float alpha, float beta, float sin_e, float cos_e, float *d, float *q)
{
    *d =  alpha * cos_e + beta * sin_e;
    *q = -alpha * sin_e + beta * cos_e;
}

void foc_inv_park(float d, float q, float sin_e, float cos_e, float *alpha, float *beta)
{
    *alpha = d * cos_e - q * sin_e;
    *beta  = d * sin_e + q * cos_e;
}

/**
 * @brief 七段式SVPWM (X/Y/Z扇区判定法, 无需atan2)
 * @param valpha, vbeta  电压 V
 * @param vbus            母线电压 V
 * @param da, db, dc     输出: 三相高侧占空比 0~1
 */
void foc_svpwm(float valpha, float vbeta, float vbus, float *da, float *db, float *dc)
{
    float X, Y, Z, t1, t2, t0;
    int N, sector;

    X = SQ3 * vbeta / vbus;
    Y = (SQ3 * vbeta + 3.0f * valpha) / (2.0f * vbus);
    Z = (SQ3 * vbeta - 3.0f * valpha) / (2.0f * vbus);

    // 扇区判定 (符号编码)
    N = 0;
    if (X > 0) N += 1;
    if (Y > 0) N += 2;
    if (Z > 0) N += 4;

    // N→扇区号映射
    switch (N) {
    case 3:  sector = 1; t1 = -Z; t2 =  X; break;
    case 1:  sector = 2; t1 =  Z; t2 =  Y; break;
    case 5:  sector = 3; t1 = -X; t2 = -Z; break;  // 注意: Y>0,X<0,Z>0→N=5→sector3
    case 4:  sector = 4; t1 = -Y; t2 =  Z; break;
    case 6:  sector = 5; t1 =  Y; t2 = -X; break;
    case 2:  sector = 6; t1 =  X; t2 = -Y; break;
    default: sector = 1; t1 = 0;   t2 = 0;   break; // 零矢量
    }

    // 过调制钳位: t1+t2>1时等比缩放
    if (t1 + t2 > 1.0f) {
        float k = 1.0f / (t1 + t2);
        t1 *= k; t2 *= k;
    }
    if (t1 < 0) t1 = 0;
    if (t2 < 0) t2 = 0;

    t0 = 1.0f - t1 - t2;
    if (t0 < 0) t0 = 0;

    // 七段式对称排列: 各相高侧占空比
    float ta = t0 * 0.5f;
    float tb = ta + t1;
    float tc = tb + t2;

    // 按扇区分配到ABC三相
    switch (sector) {
    case 1: *da = tc; *db = tb; *dc = ta; break;  // A=C, B=B, C=A (最高→最低)
    case 2: *da = tb; *db = tc; *dc = ta; break;
    case 3: *da = ta; *db = tc; *dc = tb; break;
    case 4: *da = ta; *db = tb; *dc = tc; break;
    case 5: *da = tb; *db = ta; *dc = tc; break;
    case 6: *da = tc; *db = ta; *dc = tb; break;
    default:*da = 0.5f; *db = 0.5f; *dc = 0.5f; break;
    }
}

// ==================== 工具 ====================

/** @brief 角度环绕到 [0, 360) */
static float wrap360(float deg)
{
    while (deg < 0)    deg += 360.0f;
    while (deg >= 360) deg -= 360.0f;
    return deg;
}

/** @brief 角度误差环绕到 [-180, 180] */
static float wrap180(float err)
{
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

// ==================== FOC 核心 ====================

void FOC_Init(void)
{
    FOC_PID_Init(&pid_iq, FOC_IQ_KP, FOC_IQ_KI, 0, FOC_IQ_MAX, FOC_IQ_MAX);
    FOC_PID_Init(&pid_id, FOC_ID_KP, FOC_ID_KI, 0, FOC_ID_MAX, FOC_ID_MAX);
    FOC_PID_Init(&pid_spd, FOC_SPD_KP, FOC_SPD_KI, 0, FOC_SPD_MAX, FOC_SPD_MAX);

    foc_mode = FOC_MODE_IDLE;
    foc_target = 0;
    foc_actual_position = 0;
    foc_actual_speed = 0;
    foc_actual_iq = 0;
    foc_actual_id = 0;

    iq_ref = 0; id_ref = 0;
    spd_ref = 0; spd_target_ramped = 0;
    theta_m_prev = 0;
    spd_filt = 0;
    cycle_cnt = 0;
    ol_angle = 0;
}

/**
 * @brief FOC电流环主函数 (20kHz, bldc.c ADC JEOC中断调用)
 * @return 1=使能PWM输出, 0=关闭输出
 */
uint8_t FOC_Update(float ia, float ib, float theta_mech,
                   float *da, float *db, float *dc)
{
    float alpha, beta;
    float sin_e, cos_e;
    float id, iq;
    float theta_e_rad;
    float dtheta;
    FOC_Mode mode = foc_mode;  // volatile快照

    // ---- 过流保护 ----
    if (fabsf(ia) > FOC_I_MAX || fabsf(ib) > FOC_I_MAX) {
        foc_mode = FOC_MODE_FAULT;
        *da = *db = *dc = 0.5f;
        return 0;  // 关输出
    }
    if (mode == FOC_MODE_FAULT) {
        *da = *db = *dc = 0.5f;
        return 0;
    }

    // ---- 电气角 ----
    // theta_e = theta_m * PP + zero_offset, 转rad
    theta_e_rad = wrap360(theta_mech * FOC_POLE_PAIRS + FOC_ENC_ZERO_OFFSET) * DEG2RAD;
    sin_e = sinf(theta_e_rad);
    cos_e = cosf(theta_e_rad);

    // ---- Clarke + Park ----
    foc_clarke(ia, ib, &alpha, &beta);
    foc_park(alpha, beta, sin_e, cos_e, &id, &iq);

    // ---- 速度计算 (机械角微分, 一阶LPF) ----
    dtheta = wrap180(theta_mech - theta_m_prev);  // deg, 环绕处理
    theta_m_prev = theta_mech;
    // dθ/dt: deg/s = dtheta / DT_I; RPM = deg/s / 6
    {
        float spd_raw = (dtheta / DT_I) / 6.0f;  // RPM
        // 一阶低通: fc≈50Hz, α=DT_I/(RC+DT_I), RC=1/(2π·50)≈3.18ms
        float a = DT_I / (0.00318f + DT_I);
        spd_filt = spd_filt * (1.0f - a) + spd_raw * a;
    }

    // ---- 外环 (分频运行) ----
    cycle_cnt++;
    // 速度环 2kHz
    if ((cycle_cnt % 10) == 0)
    {
        switch (mode) {
        case FOC_MODE_POSITION: {
            // 位置环 → 速度参考 (每20个周期=1kHz)
            if ((cycle_cnt % 20) == 0) {
                float pos_err = wrap180(foc_target - theta_mech);
                spd_ref = FOC_PID_Calc(&pid_spd, pos_err * FOC_POS_KP, DT_P);
                // 上面用P增益简化: 实际是 P输出限幅
                // spd_ref 已被PID的out_max限幅(FOC_POS_MAX_OUT)
            }
            break;
        }
        case FOC_MODE_SPEED:
            spd_ref = foc_target;
            break;
        case FOC_MODE_OPENLOOP:
            // 开环: 不走PID, 直接积分角度
            ol_angle += FOC_OL_SPEED * _2PI * DT_I;  // rad
            if (ol_angle > _2PI) ol_angle -= _2PI;
            break;
        default:
            spd_ref = 0;
            break;
        }

        // 速度斜坡 (速度模式 + 位置模式)
        if (mode == FOC_MODE_SPEED || mode == FOC_MODE_POSITION) {
            float max_step = FOC_SPEED_RAMP * DT_S;  // 每个速度环周期允许的变化
            float diff = spd_ref - spd_target_ramped;
            if (diff >  max_step) diff =  max_step;
            if (diff < -max_step) diff = -max_step;
            spd_target_ramped += diff;
        }
    }

    // ---- 电流环 (每次都跑, 20kHz) ----
    if (mode == FOC_MODE_IDLE) {
        *da = *db = *dc = 0.5f;
        // 复位积分, 防止重新启动时积分饱和
        FOC_PID_Reset(&pid_iq);
        FOC_PID_Reset(&pid_id);
        FOC_PID_Reset(&pid_spd);
        foc_actual_position = theta_mech;
        foc_actual_speed = spd_filt;
        foc_actual_iq = iq;
        foc_actual_id = id;
        return 0;  // 关输出
    }

    if (mode == FOC_MODE_OPENLOOP) {
        // 开环: 固定电压, 角度由ol_angle驱动
        float v = FOC_OL_VOLTAGE;
        foc_inv_park(0, v, sinf(ol_angle), cosf(ol_angle), &alpha, &beta);
        foc_svpwm(alpha, beta, FOC_VBUS, da, db, dc);
        foc_actual_position = theta_mech;
        foc_actual_speed = spd_filt;
        foc_actual_iq = iq;
        foc_actual_id = id;
        return 1;
    }

    // 速度环 → Iq参考 (2kHz)
    if ((cycle_cnt % 10) == 0) {
        iq_ref = FOC_PID_Calc(&pid_spd, spd_target_ramped - spd_filt, DT_S);
    }
    id_ref = 0;  // Id参考恒为0 (表贴式PMSM)

    // 电流PI (20kHz)
    float vd = FOC_PID_Calc(&pid_id, id_ref - id, DT_I);
    float vq = FOC_PID_Calc(&pid_iq, iq_ref - iq, DT_I);

    // 反Park + SVPWM
    foc_inv_park(vd, vq, sin_e, cos_e, &alpha, &beta);
    foc_svpwm(alpha, beta, FOC_VBUS, da, db, dc);

    // ---- 快照供菜单/VOFA ----
    foc_actual_position = theta_mech;
    foc_actual_speed = spd_filt;
    foc_actual_iq = iq;
    foc_actual_id = id;

    return 1;
}
