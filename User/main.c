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

/*
 * ============================================================================
 *  CH32V303 FOC 无刷电机驱动 — 主程序
 * ============================================================================
 *  硬件: CH32V303CBT6 (青稞V4F, 144MHz, 软浮点)
 *  功能: FOC闭环/开环驱动 + OLED多级菜单 + 五向按键 + MT6701编码器 + VOFA调试
 *
 *  主循环架构 (约100Hz节拍, 受OLED刷帧阻塞实际约25~30Hz):
 *    ┌─ KEY_GetEvent()   取按键事件 (EXTI+TIM3异步入队, 非阻塞)
 *    ├─ Menu_Run(evt)    三屏状态机: 模式选择→目标设定→运行监视
 *    │                   确认后设置 foc_mode / foc_target 启动输出
 *    └─ VOFA_Send()      JustFloat 5通道非阻塞DMA发送 (目标/位置/速度/Iq/Id)
 *
 *  FOC控制环 (ADC1 JEOC中断, 20kHz, 自动运行, 不占主循环):
 *    IDLE      → MOE关, 电机不动, 复位PID积分
 *    OPENLOOP  → 开环: ol_angle积分 + 固定电压SVPWM (FOC_Update内置)
 *    POSITION  → 位置环→速度环→电流环 闭环
 *    SPEED     → 速度环→电流环 闭环
 *    FAULT     → 过流保护 (ia/ib > FOC_I_MAX), 关输出
 *
 *  安全设计:
 *    - 上电默认 foc_mode = IDLE, MOE关闭, 电机不转, 等待菜单选择模式
 *    - 运行屏 OK长按 = 紧急停机 (foc_mode→IDLE)
 * ============================================================================
 */

#include "debug.h"
#include "key.h"
#include "oled.h"
#include "menu.h"
#include "foc.h"
#include "bldc.h"
#include "vofa.h"

/* Global typedef */

/* Global define */

/* Global Variable */

/*********************************************************************
 * @fn      main
 *
 * @brief   主程序: 初始化全部外设模块, 进入菜单+FOC主循环
 *
 * @return  none
 */
int main(void)
{
    /* ---- 系统基础初始化 ---- */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 2位抢占 / 2位子优先级
    SystemCoreClockUpdate();
    Delay_Init();

    /* ---- 应用模块初始化 (顺序: 显示/输入 → 调试 → 驱动) ---- */
    Menu_Init();    // OLED(I2C1) + 五向按键(EXTI+TIM3) + 菜单界面, 内部含KEY_Init
    VOFA_Init();    // VOFA+ JustFloat 调试串口(USART2 + DMA1_CH7, 115200)
    BLDC_Init();    // TIM1互补PWM + ADC1注入采样 + MT6701 + 零点校准 + FOC_Init
                    //   + ADC1 JEOC中断使能 (20kHz FOC电流环自动运行)
                    //   初始化后 foc_mode=IDLE → FOC_Update返回0 → MOE关, 电机不动

    /* 上电默认空闲, 等待菜单选择模式后启动 (IDLE时电机不动) */
    foc_mode = FOC_MODE_IDLE;

    while (1)
    {
        KEY_Event evt;
        float vofa_buf[5];

        /* 1. 菜单交互: 按键事件驱动三屏状态机, 确认后设置 foc_mode/foc_target
         *    FOC控制环由 ADC1 JEOC中断(20kHz)自动运行, 不占主循环 */
        evt = KEY_GetEvent();
        Menu_Run(evt);

        /* 2. VOFA+ JustFloat 在线调试 (非阻塞DMA, 上一帧未发完则跳过)
         *    通道: [0]目标 [1]位置deg [2]速度RPM [3]Iq A [4]Id A */
        vofa_buf[0] = foc_target;
        vofa_buf[1] = foc_actual_position;
        vofa_buf[2] = foc_actual_speed;
        vofa_buf[3] = foc_actual_iq;
        vofa_buf[4] = foc_actual_id;
        VOFA_Send(vofa_buf, sizeof(vofa_buf) / sizeof(vofa_buf[0]));

        /* 3. 主循环节拍 ~10ms (OLED刷帧阻塞约30ms, 实际约25~30Hz) */
        Delay_Ms(10);
    }
}
