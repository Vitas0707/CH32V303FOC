#ifndef __MT6701_H__
#define __MT6701_H__

#include "ch32v30x.h"

// ==================== SPI1 引脚定义 (按原理图接线, 调试时在此修改) ====================
// SPI1_CS   -> PA4 (软件控制片选)
// SPI1_SCK  -> PA5
// SPI1_MISO -> PA6  (MT6701 DO, 数据输出)
// SPI1_MOSI -> PA7  (MT6701 SSI为只读接口, MOSI悬空不接传感器, 仅按SPI常规配置)
#define MT6701_CS_PORT    GPIOA
#define MT6701_CS_PIN     GPIO_Pin_4

// ==================== SPI 时序参数 ====================
// MT6701 SSI: TCLK>=64ns(最高约15.6MHz), 数据在CLK上升沿变化, 下降沿采样 -> SPI Mode 1
// APB2=144MHz时, 16分频=9MHz, 满足时序且留有余量; FOC中断中读取一次约3~4us
#define MT6701_SPI_PRESCALER  SPI_BaudRatePrescaler_16

// ==================== 帧格式 ====================
// 24bit: D[13:0]角度 + Mg[3:0]状态 + CRC[5:0]
// CRC范围: D[13:0]+Mg[3:0]共18位, 多项式X^6+X+1, MSB先移入
#define MT6701_FRAME_BITS   24

// ==================== 读取结果 ====================
typedef struct {
    float angle_deg;   // 绝对角度 0~360deg
    uint16_t raw;      // 14位原始角度值 0~16383
    uint8_t mg_status; // Mg[3:0]状态位
    uint8_t crc_ok;    // 1=CRC校验通过
} MT6701_Data;

// Mg状态位解析 (mt6701_data.mg_status)
#define MT6701_STS_FIELD_MASK   0x03  // Mg[1:0]: 0正常 1磁场过强 2磁场过弱 3保留
#define MT6701_STS_PUSH         0x04  // Mg[2]:   1=旋钮被按压
#define MT6701_STS_OVERSPEED    0x08  // Mg[3]:   1=超速标志

void MT6701_Init(void);
void MT6701_Read(MT6701_Data *data);
float MT6701_ReadAngle(void);

#endif // __MT6701_H__
