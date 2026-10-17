#ifndef __FOC_H__
#define __FOC_H__

#include <stdint.h>

// ====================================================================
//  FOC 纯算法层 —— 零硬件依赖, 可独立移植/单元测试
//  bldc.c 的ADC中断调用 FOC_Update(), 取回占空比后写TIM1 CCR
// ====================================================================

// ==================== 可调参数  ====================
#define FOC_PWM_FREQ        35000.0f  // PWM开关频率 Hz
#define FOC_POLE_PAIRS      7         // 电机极对数
#define FOC_VBUS            11.3f     // 母线电压 V (无母线采样, 固定值)
#define FOC_R_SHUNT         0.01f     // 采样电阻 Ω (10mΩ)
#define FOC_INA_GAIN        50.0f     // INA240A2 增益
#define FOC_ADC_VREF        3.3f      // ADC参考电压
#define FOC_ADC_RES         4096.0f   // 12位
#define FOC_I_MAX           3.0f      // 过流保护阈值 A
#define FOC_DEADTIME_NS     500       // 死区 ns

#define FOC_POS_MAX         360.0f    // 位置上限 deg
#define FOC_SPEED_MAX       1000.0f   // 速度上限 RPM
#define FOC_SPEED_RAMP      2000.0f   // 速度斜坡 RPM/s

// 编码器电气零点 (deg, 标定后填; 参见FOC_Update内的手动标定注释)
#define FOC_ENC_ZERO_OFFSET  0.0f

// ---------- PID 初始值 (上板后用VOFA调) ----------
#define FOC_IQ_KP   0.5f
#define FOC_IQ_KI   0.01f
#define FOC_IQ_MAX  12.0f
#define FOC_ID_KP   0.5f
#define FOC_ID_KI   0.01f
#define FOC_ID_MAX  12.0f
#define FOC_SPD_KP   0.02f
#define FOC_SPD_KI   0.0005f
#define FOC_SPD_MAX  5.0f           // → Iq参考上限 A
#define FOC_POS_KP   10.0f          // deg误差→RPM参考
#define FOC_POS_MAX_OUT  3000.0f    // RPM

#define FOC_OL_VOLTAGE   7.0f       // 开环电压 V
#define FOC_OL_SPEED     10.0f       // 开环电频率 Hz

// ==================== 工作模式 ====================
typedef enum {
    FOC_MODE_IDLE = 0,
    FOC_MODE_POSITION,
    FOC_MODE_SPEED,
    FOC_MODE_OPENLOOP,
    FOC_MODE_FAULT,
} FOC_Mode;

// ==================== 菜单 <-> FOC 共享变量 ====================
extern volatile FOC_Mode foc_mode;
extern volatile float foc_target;
extern volatile float foc_actual_position;
extern volatile float foc_actual_speed;
extern volatile float foc_actual_iq;
extern volatile float foc_actual_id;

// ==================== PID 控制器 ====================
typedef struct {
    float kp, ki, kd;
    float integral;
    float out_max;
    float int_max;
    float prev_err;
} FOC_PID;

void  FOC_PID_Init(FOC_PID *p, float kp, float ki, float kd, float out_max, float int_max);
void  FOC_PID_Reset(FOC_PID *p);
float FOC_PID_Calc(FOC_PID *p, float err, float dt);

// ==================== 数学变换 ====================
void  foc_clarke(float ia, float ib, float *alpha, float *beta);
void  foc_park(float alpha, float beta, float sin_e, float cos_e, float *d, float *q);
void  foc_inv_park(float d, float q, float sin_e, float cos_e, float *alpha, float *beta);
void  foc_svpwm(float valpha, float vbeta, float vbus, float *da, float *db, float *dc);

// ==================== FOC 核心 ====================

/** @brief 初始化算法层 (复位PID/变量, 不触碰硬件) */
void FOC_Init(void);

/**
 * @brief FOC电流环 (20kHz, 由bldc.c ADC中断调用)
 * @param ia, ib     A/B相电流 A (已减偏置)
 * @param theta_mech 机械角度 deg (0~360, 来自MT6701)
 * @param da, db, dc 输出: 三相高侧占空比 0~1
 * @return 1=使能PWM输出, 0=关闭输出(IDLE/FAULT)
 */
uint8_t FOC_Update(float ia, float ib, float theta_mech,float *da, float *db, float *dc);

float wrap360(float deg);

#endif // __FOC_H__
