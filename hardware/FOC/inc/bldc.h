#ifndef __BLDC_H__
#define __BLDC_H__

#include <stdint.h>

/**
 * @brief 初始化BLDC全套硬件 + FOC算法
 *        内部完成: TIM1三相互补PWM(20kHz) + ADC1注入采样(TIM1 TRGO触发)
 *        + 上电电流零点校准 + MT6701初始化 + FOC_Init
 *        初始化后MOE关闭, 等菜单设foc_mode!=IDLE后自动开启输出
 * @note  需在 Delay_Init() / NVIC_PriorityGroupConfig() 之后调用
 */
void BLDC_Init(void);

#endif // __BLDC_H__
