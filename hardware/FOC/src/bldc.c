/**
 * @file bldc.c
 * @brief BLDC硬件驱动层: TIM1互补PWM + ADC1注入采样 + FOC中断回调
 *
 * 硬件接线:
 *   PWM:  PA8=TIM1_CH1 PA9=CH2 PA10=CH3 (→FD6288 LIN, 下管)
 *         PB13=CH1N PB14=CH2N PB15=CH3N (→FD6288 HIN, 上管)
 *   ADC:  PA0=ADC_CH0(相A电流) PA1=ADC_CH1(相B电流)  INA240A2 G=20 R=10mΩ
 *   ENC:  MT6701 SSI via SPI1 (PA4~PA7)
 *
 * FD6288 HIN/LIN接法与常规相反:
 *   CHxN→HIN(上管) CHx→LIN(下管)
 *   PWM1模式下 CHx高电平=下管导通, 占空比d_下=CCR/ARR
 *   高侧占空比 d_上 = 1 - CCR/ARR → CCR = (1 - d_上) × ARR (软件取反)
 *
 * ADC采样点: TIM1中心对齐模式1, 下溢(CNT=0)产生TRGO触发ADC注入组
 *   此时所有下管导通(零矢量000), 低边电阻上有相电流, 为最佳采样窗口
 *
 * 中断: ADC1 JEOC(优先级0最高) → 读电流 → 读编码器 → FOC_Update → 写CCR
 */
 #include<math.h>
#include "bldc.h"
#include "foc.h"
#include "mt6701.h"
#include "ch32v30x.h"

// ==================== 模块变量 ====================
static uint16_t s_arr;          // TIM1 ARR (周期值)
static uint16_t s_offset_a;    // A相电流零点(ADC原始值)
static uint16_t s_offset_b;    // B相电流零点
static float    s_current_lsb; // ADC 1 LSB 对应的电流 A

// ==================== TIM1 三相互补PWM ====================

/**
 * @brief 初始化TIM1: 20kHz中心对齐, CH1/2/3+CH1N/2N/3N互补, 死区500ns
 */
static void BLDC_PWM_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
    TIM_OCInitTypeDef       TIM_OCStruct;
    TIM_BDTRInitTypeDef     TIM_BDTRStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_TIM1, ENABLE);

    // PA8/9/10 = CH1/2/3 复用推挽
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PB13/14/15 = CH1N/2N/3N 复用推挽
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 时基: 中心对齐模式1, ARR = SysClk / (2 × PWM_FREQ)
    s_arr = (uint16_t)(SystemCoreClock / (2 * (uint32_t)FOC_PWM_FREQ)); // 3600 @144MHz

    TIM_BaseStruct.TIM_Prescaler     = 0;
    TIM_BaseStruct.TIM_Period        = s_arr;
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_CounterMode   = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);

    // TRGO=Update(下溢), 用于触发ADC注入采样
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);

    // OC1/2/3 + 互补N, PWM1, 初始50%占空比
    TIM_OCStruct.TIM_OCMode        = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState   = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OutputNState  = TIM_OutputNState_Enable;
    TIM_OCStruct.TIM_Pulse         = s_arr / 2;          // 50% → CCR=1800
    TIM_OCStruct.TIM_OCPolarity    = TIM_OCPolarity_High;
    TIM_OCStruct.TIM_OCNPolarity   = TIM_OCNPolarity_High;
    TIM_OCStruct.TIM_OCIdleState   = TIM_OCIdleState_Reset; // MOE关时下管断
    TIM_OCStruct.TIM_OCNIdleState  = TIM_OCNIdleState_Reset; // MOE关时上管断
    TIM_OC1Init(TIM1, &TIM_OCStruct);
    TIM_OC2Init(TIM1, &TIM_OCStruct);
    TIM_OC3Init(TIM1, &TIM_OCStruct);

    // 预装载: CCR写后在下个更新事件生效, 防止毛刺
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    // BDTR: 死区 + OSSR/OSSI + MOE初始关
    TIM_BDTRStruct.TIM_OSSRState        = TIM_OSSRState_Enable;
    TIM_BDTRStruct.TIM_OSSIState        = TIM_OSSIState_Enable;
    TIM_BDTRStruct.TIM_LOCKLevel        = TIM_LOCKLevel_OFF;
    TIM_BDTRStruct.TIM_DeadTime         = (uint16_t)((uint64_t)FOC_DEADTIME_NS * SystemCoreClock / 1000000000u); // 72 @144MHz
    TIM_BDTRStruct.TIM_Break            = TIM_Break_Disable;
    TIM_BDTRStruct.TIM_BreakPolarity    = TIM_BreakPolarity_High;
    TIM_BDTRStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRStruct);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, DISABLE); // MOE初始关, 等FOC启动
}

/** @brief 写三相高侧占空比(0~1)到CCR (内部做HIN/LIN取反) */
static void BLDC_PWM_SetDuty(float da, float db, float dc)
{
    if(da > 1.0f)da = 1.0f;
    if(db > 1.0f)db = 1.0f;
    if(dc > 1.0f)dc = 1.0f;
    if(da < 0)da = 0;
    if(db < 0)db = 0;
    if(dc < 0)dc = 0;

    // HIN接CHxN(互补), 高侧占空比=1-CCR/ARR → CCR=(1-d)×ARR
    TIM_SetCompare1(TIM1, (uint16_t)((1.0f - da) * s_arr));
    TIM_SetCompare2(TIM1, (uint16_t)((1.0f - db) * s_arr));
    TIM_SetCompare3(TIM1, (uint16_t)((1.0f - dc) * s_arr));
}

// ==================== ADC1 注入采样 ====================

/**
 * @brief 初始化ADC1: 注入组2通道(CH0=PA0 CH1=PA1), TIM1 TRGO触发, JEOC中断
 */
static void BLDC_ADC_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStruct;
    ADC_InitTypeDef     ADC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8); // 144/8=18MHz (WCH实测可用)

    // PA0/PA1 模拟输入
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ADC1 基础配置
    ADC_InitStruct.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStruct.ADC_ScanConvMode       = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None; // 常规组不用
    ADC_InitStruct.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &ADC_InitStruct);

    // 注入组: rank1=CH0(相A) rank2=CH1(相B), 最快采样时间
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_1Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_1Cycles5);
    ADC_InjectedSequencerLengthConfig(ADC1, 2);

    // 注入外部触发 = TIM1 TRGO (下溢点)
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);

    // JEOC中断
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);

    ADC_Cmd(ADC1, ENABLE);

    // ADC自校准
    {
        uint32_t to;
        ADC_ResetCalibration(ADC1);
        to = 100000;
        while (ADC_GetResetCalibrationStatus(ADC1) == SET && --to);
        ADC_StartCalibration(ADC1);
        to = 100000;
        while (ADC_GetCalibrationStatus(ADC1) == SET && --to);
    }

    // NVIC在BLDC_Init()最后使能, 避免初始化阶段被20kHz中断打扰
}

/**
 * @brief 上电电流零点校准 (PWM关断时软件触发注入采样, 取256次平均)
 * @note  必须在MOE关闭状态下调用
 */
static void BLDC_ADC_CalibrateOffset(void)
{
    uint32_t sum_a = 0, sum_b = 0;
    uint32_t timeout;
    int i;
    int valid = 0;

    // 切到软件触发模式: 禁用外部触发, 确保软件触发能工作
    ADC_ExternalTrigInjectedConvCmd(ADC1, DISABLE);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);

    for (i = 0; i < 256; i++) {
        ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
        // 超时保护: 防止ADC异常时死循环卡死整个系统
        timeout = 100000;
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET && --timeout);
        if (timeout == 0)
            break;  // ADC注入转换超时, 跳出校准
        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
        sum_a += ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        sum_b += ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
        valid++;
    }

    if (valid > 0) {
        s_offset_a = sum_a / valid;
        s_offset_b = sum_b / valid;
    } else {
        // ADC完全不可用, 用默认中点值兜底
        s_offset_a = 2048;
        s_offset_b = 2048;
    }

    // 恢复TIM1 TRGO触发
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
}

// ==================== ADC中断 (FOC电流环 20kHz) ====================

/* ISR主体逻辑(纯C, 可被mingw64语法检查) */
static void ADC1_2_ISR_Body(void)
{
    if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) != RESET)
    {
        float ia, ib, theta;
        float da, db, dc;
        uint8_t en;
        static uint8_t moeState = 0;

        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);

        // 读注入结果并换算电流 (减零点偏置 × LSB系数)
        ia = (float)((int16_t)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1)
                     - (int16_t)s_offset_a) * s_current_lsb;
        ib = (float)((int16_t)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2)
                     - (int16_t)s_offset_b) * s_current_lsb;

        // 读编码器机械角度
        theta = MT6701_ReadAngle(); // 0~360 deg

        // FOC核心算法
        en = FOC_Update(ia, ib, theta, &da, &db, &dc);

        // 写PWM占空比
        BLDC_PWM_SetDuty(da, db, dc);

        // MOE状态管理 (仅在变化时操作寄存器)
        if (en != moeState) {
            moeState = en;
            TIM_CtrlPWMOutputs(TIM1, en ? ENABLE : DISABLE);
        }
    }
}

/**
 * @brief ADC1注入组转换完成中断
 *        TIM1下溢→TRGO→ADC注入→JEOC→本函数
 *        流程: 读电流→读编码器→FOC_Update→写CCR
 *
 * @note 软浮点ABI(-mabi=ilp32, 不启用F扩展)下:
 *       1. 浮点运算走libgcc软浮点库调用, 不使用f0-f31浮点寄存器
 *       2. ISR无需手动保存/恢复浮点寄存器上下文
 *       3. ADC ISR优先级=0走软件压栈 → 用标准interrupt()属性
 *          (此工具链不支持"WCH-Interrupt"字符串, 只支持"WCH-Interrupt-fast"
 *           和标准interrupt(); 优先级0不在硬件压栈范围, 必须用标准属性
 *           让gcc自己保存callee-saved通用寄存器 + 生成mret返回)
 *       4. 性能代价: FOC_Update大量浮点运算走软浮点库, ISR耗时上升
 *          (约30~50µs), 接近50µs周期上限, 后续可考虑重新启用硬浮点
 */
#ifdef __riscv
__attribute__((interrupt()))
void ADC1_2_IRQHandler(void)
{
    ADC1_2_ISR_Body();
}
#else
/* 非RISC-V平台: 占位实现, 仅用于mingw64语法检查 */
void ADC1_2_IRQHandler(void)
{
    ADC1_2_ISR_Body();
}
#endif

// ==================== 对外接口 ====================

void BLDC_Init(void)
{
    // 计算电流换算系数: I = (raw - offset) × Vref / (ADCres × INA增益 × R分流)
    s_current_lsb = FOC_ADC_VREF / (FOC_ADC_RES * FOC_INA_GAIN * FOC_R_SHUNT);

    MT6701_Init();     // 编码器SPI1
    BLDC_PWM_Init();   // TIM1互补PWM (MOE关)
    BLDC_ADC_Init();   // ADC1注入组+JEOC中断

    // 上电零点校准 (此时MOE关, 电机无电流, ADC读到的是INA240零电流偏置)
    BLDC_ADC_CalibrateOffset();

    FOC_Init();        // 初始化算法层(PID/变量复位)

    // 使能ADC1 JEOC中断 (优先级0最高, 软件压栈, 用标准interrupt()属性)
    // 此后TIM1每周期(下溢)产生TRGO→ADC注入→JEOC中断→FOC_Update
    // 放在最后使能, 避免初始化阶段被20kHz高频中断打扰
    {
        NVIC_InitTypeDef NVIC_InitStruct;
        NVIC_InitStruct.NVIC_IRQChannel                   = ADC_IRQn;
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
        NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStruct);
    }

    // foc_mode=IDLE → FOC_Update返回0 → MOE保持关, 电机不动
    // 菜单设foc_mode=OPENLOOP/POSITION/SPEED后, FOC_Update返回1 → MOE自动开启
    //   OPENLOOP : FOC_Update内开环分支(ol_angle积分 + 固定电压SVPWM)
    //   POSITION : 位置环→速度环→电流环 闭环
    //   SPEED    : 速度环→电流环 闭环
}