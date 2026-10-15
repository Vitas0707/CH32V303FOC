/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "debug.h"
#include "menu.h"
#include "bldc.h"
#include "vofa.h"

/* Global typedef */

/* Global define */

/* Global Variable */


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n", SystemCoreClock);

	Menu_Init();   // 五向开关(EXTI+TIM3) + OLED + 菜单界面
	BLDC_Init();   // TIM1互补PWM + ADC1注入 + 零点校准 + MT6701 + FOC算法
	VOFA_Init();   // USART2 DMA JustFloat调试串口

	printf("FOC ready. foc_mode=%d\r\n", foc_mode);

	while(1)
    {
		/* 主循环: 菜单状态机 + VOFA发送
		 * FOC电流环(20kHz)在ADC JEOC中断中运行, 不受主循环阻塞影响
		 * 按键事件由EXTI中断异步入队, OLED刷新期间的按键不会丢失 */
		Menu_Run(KEY_GetEvent());
		VOFA_Send();
    }
}
