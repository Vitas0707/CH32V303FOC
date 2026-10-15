#ifndef __KEY_H__
#define __KEY_H__

#include "ch32v30x.h"

// ==================== 五向开关引脚定义 (低电平有效) ====================
// 注意: 引脚均在GPIOB, 若改用其他端口需在key.c的KEY_Init中修改EXTI线映射
#define KEY_UP_PORT     GPIOB
#define KEY_UP_PIN      GPIO_Pin_0
#define KEY_DOWN_PORT   GPIOB
#define KEY_DOWN_PIN    GPIO_Pin_1
#define KEY_LEFT_PORT   GPIOB
#define KEY_LEFT_PIN    GPIO_Pin_10
#define KEY_RIGHT_PORT  GPIOB
#define KEY_RIGHT_PIN   GPIO_Pin_11
#define KEY_OK_PORT     GPIOB
#define KEY_OK_PIN      GPIO_Pin_12

// ==================== 时序参数 (单位: ms, 由TIM3 1ms时基驱动) ====================
#define KEY_DEBOUNCE_MS     20    // 消抖: 同一按键两次边沿间隔<此值则忽略
#define KEY_LONG_PRESS_MS   500   // 长按判定阈值
#define KEY_REPEAT_MS       120   // 长按后连发间隔

// ==================== 按键事件 ====================
typedef enum {
    KEY_EVT_NONE = 0,
    // 方向键: 消抖确认按下时立即触发
    KEY_EVT_UP,
    KEY_EVT_DOWN,
    KEY_EVT_LEFT,
    KEY_EVT_RIGHT,
    // 方向键: 长按达到阈值后的连发 (首次触发于长按判定时刻, 之后周期连发)
    KEY_EVT_UP_R,
    KEY_EVT_DOWN_R,
    KEY_EVT_LEFT_R,
    KEY_EVT_RIGHT_R,
    // OK键: 松开且未长按 = 短按确认
    KEY_EVT_OK,
    // OK键: 按住达到500ms = 长按
    KEY_EVT_OK_L,
} KEY_Event;

/**
 * @brief 初始化五向开关: GPIO + EXTI双边沿中断 + TIM3(1ms)时基
 * @note  初始化后按键事件自动放入环形缓冲, 主循环调用KEY_GetEvent()取走
 * @note  需在NVIC_PriorityGroupConfig()之后调用
 */
void KEY_Init(void);

/**
 * @brief 从环形缓冲取一个按键事件 (非阻塞)
 * @return 事件, KEY_EVT_NONE表示缓冲空
 * @note   ISR写(head), 主循环读(tail), 单生产者单消费者, 无需关中断
 */
KEY_Event KEY_GetEvent(void);

/**
 * @brief 获取系统毫秒计数 (由TIM3 1ms中断递增)
 * @return 自KEY_Init以来的毫秒数 (约49天溢出)
 * @note   供菜单做闪烁/刷新计时
 */
uint32_t KEY_GetSysMs(void);

#endif // __KEY_H__
