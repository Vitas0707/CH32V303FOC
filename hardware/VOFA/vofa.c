/**
 * @file vofa.c
 * @brief VOFA+ 上位机调试串口 (USART2 + DMA1_CH7 + JustFloat协议)
 *
 * 接线: PA2=TX PA3=RX, 115200-8-N-1
 * 协议: JustFloat = N×float(小端) + 尾帧 0x00 0x00 0x80 0x7F
 *
 * 数据流: 主循环调用VOFA_Send() → 快照foc_actual_* → 打包 → DMA发送(非阻塞)
 *         上一帧未发完则跳过, 不阻塞FOC主循环
 *
 * @note VOFA+ 设置: 协议=JustFloat, 串口=对应COM, 波特=115200
 */
#include "vofa.h"
#include "foc.h"
#include "ch32v30x.h"

// JustFloat帧: 5个float数据 + 4字节尾帧 = 24字节
#define VOFA_CH_COUNT  5
#define VOFA_FRAME_LEN (VOFA_CH_COUNT * 4 + 4)

// 4字节对齐(float访问要求)
static uint8_t s_buf[VOFA_FRAME_LEN] __attribute__((aligned(4)));
static uint8_t s_sending = 0;

// JustFloat尾帧
static const uint8_t s_tail[4] = {0x00, 0x00, 0x80, 0x7F};

void VOFA_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef   DMA_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // PA2=TX 复用推挽, PA3=RX 上拉输入
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // USART2: 115200 8N1 TX
    USART_InitStruct.USART_BaudRate            = 115200;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode                = USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);

    // DMA1_CH7: 内存→USART2_DATAR, 单次模式
    DMA_DeInit(DMA1_Channel7);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(uintptr_t)&USART2->DATAR;
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)(uintptr_t)s_buf;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize         = VOFA_FRAME_LEN;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_Low;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel7, &DMA_InitStruct);

    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART2, ENABLE);

    // DMA传输完成中断 (可选, 也可轮询flag, 这里用轮询更简单)
    // 不开中断, VOFA_Send中轮询TC7 flag
}

void VOFA_Send(void)
{
    float *fp;

    // 上一帧仍在发送?
    if (s_sending) {
        if (DMA_GetFlagStatus(DMA1_FLAG_TC7) != RESET) {
            DMA_ClearFlag(DMA1_FLAG_TC7);
            DMA_Cmd(DMA1_Channel7, DISABLE);
            s_sending = 0;
        } else {
            return; // 还没发完, 跳过本帧
        }
    }

    // 快照volatile变量 (FOC ISR可能在写, 但float原子读, 无撕裂)
    fp = (float *)s_buf;
    fp[0] = foc_target;
    fp[1] = foc_actual_position;
    fp[2] = foc_actual_speed;
    fp[3] = foc_actual_iq;
    fp[4] = foc_actual_id;

    // 尾帧
    s_buf[20] = s_tail[0];
    s_buf[21] = s_tail[1];
    s_buf[22] = s_tail[2];
    s_buf[23] = s_tail[3];

    // 启动DMA传输
    DMA_SetCurrDataCounter(DMA1_Channel7, VOFA_FRAME_LEN);
    DMA_Cmd(DMA1_Channel7, ENABLE);
    s_sending = 1;
}
