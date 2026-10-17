#ifndef __VOFA_H__
#define __VOFA_H__

#include<stdint.h>

/**
 * @brief 初始化VOFA串口 (USART2: PA2=TX PA3=RX, 115200, DMA1_CH7发送)
 */
void VOFA_Init(void);

/**
 * @brief 发送一帧JustFloat数据 (非阻塞, DMA发送)
 *        若上一帧仍在发送则跳过, 主循环中可放心每次调用
 *        通道: [0]目标 [1]位置deg [2]速度RPM [3]Iq A [4]Id A
 */
void VOFA_Send(const float *data, uint8_t size);

#endif // __VOFA_H__
