/**
 * @file key.c
 * @brief 五向开关按键驱动 —— EXTI双边沿中断 + TIM3(1ms)时基 (中断驱动, 无轮询)
 *
 * @note 硬件: 3V3经10K上拉, 按键接地, 按下为低电平
 * @note 引脚: PB0=UP PB1=DOWN PB10=LEFT PB11=RIGHT PB12=OK
 *
 * 架构:
 *   EXTI(双边沿)  →  ISR: 消抖过滤 + 按下/释放事件  →  环形缓冲
 *   TIM3(1ms)     →  ISR: 递增sys_ms + 长按/连发判定    →  环形缓冲
 *   主循环        →  KEY_GetEvent() 非阻塞取事件
 *
 * 事件模型:
 *   - 方向键: 消抖确认按下时立即触发KEY_EVT_x, 长按500ms后每120ms连发KEY_EVT_x_R
 *   - OK键:   松开且未长按触发KEY_EVT_OK(短按确认); 按住500ms触发KEY_EVT_OK_L(长按)
 *
 * @note EXTI和TIM3中断优先级设为最低(Preemption=3), 不影响FOC定时器中断
 * @note 主循环不再需要KEY_Scan()+Delay_Ms(10)轮询, FOC控制环获得完整CPU时间
 */
#include "key.h"

// WCH 青稞V4F内核: 所有中断函数必须用attribute声明, 否则mret不执行导致跑飞
// WCH定制GCC只认自己的专有字符串, 不支持标准interrupt属性
// 按键ISR优先级=3(4级嵌套最低)走硬件压栈 → 用WCH-Interrupt-fast
#ifdef __riscv
#define KEY_ISR __attribute__((interrupt("WCH-Interrupt-fast")))
#else
#define KEY_ISR
#endif

// ==================== 内部类型 ====================
typedef enum {
    KEY_ID_UP = 0,
    KEY_ID_DOWN,
    KEY_ID_LEFT,
    KEY_ID_RIGHT,
    KEY_ID_OK,
    KEY_ID_NUM,
} KEY_ID;

typedef struct {
    uint8_t pressed;      // 当前是否按住(消抖后)
    uint8_t longFired;    // 本次按下的长按已触发标志
    uint32_t pressTime;   // 按下时刻(sys_ms)
    uint32_t nextRepeat;  // 下次连发时刻(sys_ms)
    uint32_t lastEdgeMs;  // 上次边沿时刻(消抖用)
} KeyState;

// ==================== 模块变量 ====================
static volatile KeyState s_keys[KEY_ID_NUM];
static volatile uint32_t s_sysMs = 0;

// 环形缓冲 (单生产者=ISR, 单消费者=主循环, 无锁安全)
#define EVT_BUF_SIZE 8
static volatile KEY_Event s_evtBuf[EVT_BUF_SIZE];
static volatile uint8_t s_evtHead = 0;  // ISR写
static volatile uint8_t s_evtTail = 0;  // 主循环读

// 按键→引脚映射表 (供EXTI ISR查用)
typedef struct { GPIO_TypeDef *port; uint16_t pin; uint32_t extiLine; } KeyPin;
static const KeyPin s_keyPins[KEY_ID_NUM] = {
    {KEY_UP_PORT,    KEY_UP_PIN,    EXTI_Line0},
    {KEY_DOWN_PORT,  KEY_DOWN_PIN,  EXTI_Line1},
    {KEY_LEFT_PORT,  KEY_LEFT_PIN,  EXTI_Line10},
    {KEY_RIGHT_PORT, KEY_RIGHT_PIN, EXTI_Line11},
    {KEY_OK_PORT,    KEY_OK_PIN,    EXTI_Line12},
};

// 按下事件查表 (OK键无按下事件, 释放/长按时才触发)
static const KEY_Event s_pressEvt[KEY_ID_NUM] = {
    KEY_EVT_UP, KEY_EVT_DOWN, KEY_EVT_LEFT, KEY_EVT_RIGHT, KEY_EVT_NONE,
};
// 连发事件查表 (仅方向键)
static const KEY_Event s_repeatEvt[KEY_ID_NUM] = {
    KEY_EVT_UP_R, KEY_EVT_DOWN_R, KEY_EVT_LEFT_R, KEY_EVT_RIGHT_R, KEY_EVT_NONE,
};

// ==================== 环形缓冲操作 ====================

/** @brief 向事件缓冲推入一个事件 (仅ISR调用) */
static void pushEvent(KEY_Event evt)
{
    uint8_t next = (s_evtHead + 1) % EVT_BUF_SIZE;
    if (next != s_evtTail)  // 非满
    {
        s_evtBuf[s_evtHead] = evt;
        s_evtHead = next;
    }
    // 满则丢弃 — 方向键连发每120ms会重发, 丢一帧不影响
}

// ==================== EXTI中断处理 ====================

/**
 * @brief 处理某个按键的边沿中断
 * @param id  按键ID
 * @note  低电平有效: 下降沿=按下, 上升沿=释放
 * @note  消抖: 若距上次边沿<KEY_DEBOUNCE_MS则忽略(过滤机械抖动)
 * @note 中断处理的逻辑，irq负责判断中断的来源然后清空标志位交给边沿处理函数处理，边沿处理函数负责更新按键状态和环形缓冲区
 */
static void handleEdge(KEY_ID id)
{
    uint32_t now = s_sysMs;//获取当前时刻
    KeyState *k = (KeyState *)&s_keys[id];//结构体数组的位置找对应按键--->取地址、转指针--->获取按键的状态
    const KeyPin *pin = &s_keyPins[id];
    uint8_t level = GPIO_ReadInputDataBit(pin->port, pin->pin);
    uint8_t pressed = (level == Bit_RESET);  // 低电平=按下

    // 消抖: 同一键两次边沿间隔太近则忽略
    if (now - k->lastEdgeMs < KEY_DEBOUNCE_MS)
        return;
    k->lastEdgeMs = now;

    if (pressed && !k->pressed)
    {
        //前面没有按下，刚刚按下了--->刚按下
        k->pressed = 1;
        k->longFired = 0;
        k->pressTime = now;
        k->nextRepeat = now + KEY_LONG_PRESS_MS;
        // 方向键立即触发 (OK键等释放再判定, 避免长按误触短按)
        if (id != KEY_ID_OK)
            pushEvent(s_pressEvt[id]);
    }
    else if (!pressed && k->pressed)
    {
        //前面有按下，刚刚没按下了--->刚释放
        k->pressed = 0;
        // OK键: 若长按未触发则判定为短按
        if (id == KEY_ID_OK && !k->longFired)
            pushEvent(KEY_EVT_OK);
    }
    // pressed==k->pressed: 同方向跳变, 忽略
}

KEY_ISR void EXTI0_IRQHandler(void)       // PB0 = UP
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);
        handleEdge(KEY_ID_UP);
    }
}

KEY_ISR void EXTI1_IRQHandler(void)       // PB1 = DOWN
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line1);
        handleEdge(KEY_ID_DOWN);
    }
}

KEY_ISR void EXTI15_10_IRQHandler(void)   // PB10/PB11/PB12 共用
{
    if (EXTI_GetITStatus(EXTI_Line10) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line10);
        handleEdge(KEY_ID_LEFT);
    }
    if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line11);
        handleEdge(KEY_ID_RIGHT);
    }
    if (EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line12);
        handleEdge(KEY_ID_OK);
    }
}

// ==================== TIM3 1ms时基中断（就是为了该死的长按逻辑） ====================

KEY_ISR void TIM3_IRQHandler(void)
{
    uint8_t i;
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//确认是TIM3的时基中断
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        s_sysMs++;

        // 检查所有按住的键: 长按/连发判定
        for (i = 0; i < KEY_ID_NUM; i++)
        {
            KeyState *k = (KeyState *)&s_keys[i];
            if (!k->pressed)
                continue;//按键没有按下，直接退出

            if (!k->longFired && (s_sysMs - k->pressTime) >= KEY_LONG_PRESS_MS)
            {
                k->longFired = 1;
                if (i == KEY_ID_OK)
                    pushEvent(KEY_EVT_OK_L);      // OK长按, 只触发一次
                else
                {
                    pushEvent(s_repeatEvt[i]);    // 方向键首次连发
                    k->nextRepeat = s_sysMs + KEY_REPEAT_MS;
                }
            }
            else if (k->longFired && i != KEY_ID_OK && s_sysMs >= k->nextRepeat)
            {
                pushEvent(s_repeatEvt[i]);        // 方向键周期连发
                k->nextRepeat += KEY_REPEAT_MS;
            }
        }
    }
}

// ==================== 对外接口 ====================

KEY_Event KEY_GetEvent(void)
{
    if (s_evtHead == s_evtTail)
        return KEY_EVT_NONE;//环形缓冲区为空--->没有新的时间
    //读取事件，更新缓冲区指针，返回对应事件
    KEY_Event evt = s_evtBuf[s_evtTail];
    s_evtTail = (s_evtTail + 1) % EVT_BUF_SIZE;
    return evt;
}

uint32_t KEY_GetSysMs(void)
{
    return s_sysMs;
}

// ==================== 初始化 ====================

/** @brief 初始化TIM3: 1ms周期中断 (最低优先级) */
static void KEY_TIM3_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // TIM3时钟 = SystemCoreClock (APB1预分频≠1时定时器时钟×2)
    // Prescaler = SystemCoreClock/1000000 - 1 → 计数器1MHz (1µs/tick)
    // Period = 1000-1 → 1ms更新中断
    TIM_TimeBaseStruct.TIM_Prescaler = SystemCoreClock / 1000000 - 1;
    TIM_TimeBaseStruct.TIM_Period = 1000 - 1;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

/** @brief 初始化EXTI: 5个按键双边沿中断 */
static void KEY_EXTI_Init(void)
{
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    uint8_t i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // EXTI线映射: 全部映射到GPIOB
    // PB0→EXTI0, PB1→EXTI1, PB10→EXTI10, PB11→EXTI11, PB12→EXTI12
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);

    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  //双边沿，单独下降沿会导致长按后松手永远没有办法停止
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;

    // EXTI0 (PB0=UP) 和 EXTI1 (PB1=DOWN)
    EXTI_InitStruct.EXTI_Line = EXTI_Line0 | EXTI_Line1;
    EXTI_Init(&EXTI_InitStruct);

    // EXTI10/11/12 (PB10/PB11/PB12)
    EXTI_InitStruct.EXTI_Line = EXTI_Line10 | EXTI_Line11 | EXTI_Line12;
    EXTI_Init(&EXTI_InitStruct);

    // NVIC: 最低优先级, 不抢FOC定时器中断
    // PriorityGroup_2: 抢占0~3, 子优先级0~1
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;  // 最低
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    // 清除可能残留的挂起标志
    for (i = 0; i < KEY_ID_NUM; i++)
        EXTI_ClearITPendingBit(s_keyPins[i].extiLine);
}

void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    uint8_t i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // GPIO: 内部上拉输入 (外部已有10K上拉, 内部上拉增强抗扰)
    GPIO_InitStruct.GPIO_Pin = KEY_UP_PIN | KEY_DOWN_PIN | KEY_LEFT_PIN |KEY_RIGHT_PIN | KEY_OK_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 清零状态
    for (i = 0; i < KEY_ID_NUM; i++)
    {
        s_keys[i].pressed = 0;
        s_keys[i].longFired = 0;
        s_keys[i].pressTime = 0;
        s_keys[i].nextRepeat = 0;
        s_keys[i].lastEdgeMs = 0;
    }
    s_evtHead = 0;
    s_evtTail = 0;
    s_sysMs = 0;

    KEY_TIM3_Init();
    KEY_EXTI_Init();
}
