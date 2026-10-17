/**
 * @file mt6701.c
 * @brief MT6701 磁编码器驱动 (SSI接口, 硬件SPI1读取)
 *
 * @note 硬件连接 (见原理图):
 *       SPI1_CS  -> PA4 (软件片选, MT6701 CSN)
 *       SPI1_SCK -> PA5 (MT6701 CLK)
 *       SPI1_MISO-> PA6 (MT6701 DO)
 *       SPI1_MOSI-> PA7 (SSI为只读接口, 传感器无数据输入脚, 此线仅走线)
 *
 * @note SSI协议 (MT6701数据手册 7.8节):
 *       - CSN下降沿启动通信, CSN拉高后停止
 *       - DO数据在CLK上升沿变化, MCU在CLK下降沿采样 -> SPI Mode 1 (CPOL=0, CPHA=2Edge)
 *       - 24位帧: D[13:0]角度 + Mg[3:0]磁场状态 + CRC[5:0]
 *       - CRC覆盖D[13:0]+Mg[3:0]共18位, 多项式X^6+X+1, MSB先移入
 *       - 角度: angle = D * 360 / 16384
 *
 * @note 读取时序: CS拉低 -> 硬件SPI连发3个哑字节产生24个时钟并收3字节 -> CS拉高
 *       MOSI发送内容无意义, 填0xFF保持总线电平稳定
 *
 * @note 可在FOC定时器中断中调用MT6701_Read(), 全程约3~4us(9MHz), 无浮点除法
 */
#include "mt6701.h"

// ==================== SPI1 初始化 ====================

/**
 * @brief 初始化SPI1与CS引脚 (PA4=CS软件控制, PA5/6/7=SPI复用)
 * @note  MT6701_Init()内部自动调用
 */
static void MT6701_SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    // PA4 = CS, 软件控制通用推挽输出, 默认拉高(未选中)
    GPIO_InitStructure.GPIO_Pin = MT6701_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MT6701_CS_PORT, &GPIO_InitStructure);
    GPIO_SetBits(MT6701_CS_PORT, MT6701_CS_PIN);

    // PA5=SCK, PA7=MOSI 复用推挽; PA6=MISO 上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // SPI1 主模式, Mode1(CPOL=0, CPHA=2Edge: 下降沿采样), 8bit, 软件NSS
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = MT6701_SPI_PRESCALER;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);
}

// ==================== 底层收发 ====================

/**
 * @brief SPI1收发一个字节 (阻塞)
 * @param tx 发送的字节 (读操作时无意义, 任意值)
 * @return 接收到的字节
 */
static uint8_t MT6701_SPI_TransferByte(uint8_t tx)
{
    SPI_I2S_SendData(SPI1, tx);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI1);
}

/**
 * @brief CRC6校验 (多项式X^6+X+1, MSB先移入, 初值0)
 * @param data18 18位数据: 高14位=D[13:0], 低4位=Mg[3:0]
 * @return 6位CRC
 * @note 原理: 寄存器MSB移出时相当于乘X^6, 模X^6+X+1即异或(X+1)=0x03
 */
static uint8_t MT6701_CRC6(uint32_t data18)
{
    uint8_t crc = 0;
    int8_t i;
    uint8_t bit, msb;

    for (i = 17; i >= 0; i--)
    {
        bit = (data18 >> i) & 0x01;
        msb = (crc >> 5) & 0x01;
        crc = ((crc << 1) | bit) & 0x3F;
        if (msb)
            crc ^= 0x03;
    }
    return crc;
}

// ==================== 对外接口 ====================

/**
 * @brief 初始化MT6701 (SPI1: PA4/5/6/7)
 * @note  MT6701上电即工作于SSI模式(MODE引脚浮空/上拉时为I2C/SSI选择,
 *        具体接线见板卡, SSI模式下无需任何配置命令, 直接读时序即可)
 */
void MT6701_Init(void)
{
    MT6701_SPI_Init();
}

/**
 * @brief 读取一帧MT6701数据 (阻塞, 约3~4us)
 * @param data 输出: 角度/状态/CRC校验结果
 * @note  可在FOC电流环中断中安全调用
 */
void MT6701_Read(MT6701_Data *data)
{
    uint32_t raw;
    uint16_t angle14;
    uint8_t mg, crcRx, crcCalc;

    // CS拉低, 启动SSI帧
    MT6701_CS_PORT->BCR = MT6701_CS_PIN;

    // 连发24个时钟, 收3字节 (发送内容无意义)
    raw = ((uint32_t)MT6701_SPI_TransferByte(0xFF) << 16) |
          ((uint32_t)MT6701_SPI_TransferByte(0xFF) << 8) |
          (uint32_t)MT6701_SPI_TransferByte(0xFF);

    // CS拉高, 帧结束 (最后一次RXNE等待已保证CLK下降沿后TL间隔足够)
    MT6701_CS_PORT->BSHR = MT6701_CS_PIN;

    // 解析: bit23~10角度, bit9~6状态, bit5~0 CRC
    angle14 = (uint16_t)(raw >> 10);
    mg = (uint8_t)((raw >> 6) & 0x0F);
    crcRx = (uint8_t)(raw & 0x3F);
    crcCalc = MT6701_CRC6(((uint32_t)angle14 << 4) | mg);

    data->raw = angle14;
    data->mg_status = mg;
    data->crc_ok = (crcRx == crcCalc);
    // 定点转角度: *360/16384 = *45/2048, 先乘后除保精度, float单精度足够
    data->angle_deg = (float)angle14 * (360.0f / 16384.0f);
}

/**
 * @brief 快速读取角度 (仅角度场景的便捷接口)
 * @return 绝对角度 0~360deg
 * @note  不做CRC校验(省时间), 供对速度敏感的场合; 需要校验请用MT6701_Read()
 */
float MT6701_ReadAngle(void)
{
    uint32_t raw;
    float angle;

    MT6701_CS_PORT->BCR = MT6701_CS_PIN;

    raw = ((uint32_t)MT6701_SPI_TransferByte(0xFF) << 16) |
          ((uint32_t)MT6701_SPI_TransferByte(0xFF) << 8) |
          (uint32_t)MT6701_SPI_TransferByte(0xFF);

    MT6701_CS_PORT->BSHR = MT6701_CS_PIN;

    angle = (float)(raw >> 10) * (360.0f / 16384.0f);
    return angle;
}
