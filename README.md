# CH32V303FOC — 基于 CH32V303 的无刷电机 FOC 驱动

本方案用于车队一轮招新全栈组的考核

基于 WCH CH32V303CBT6（青稞 V4F RISC-V 内核，144 MHz）的磁场定向控制（FOC）无刷直流电机驱动工程，包含 OLED 显示、五向按键多级菜单、MT6701 磁编码器、双电阻相电流采样、SVPWM 互补 PWM 输出及 VOFA+ JustFloat 在线调试。

## 硬件平台

| 项 | 规格 |
|---|---|
| MCU | CH32V303CBT6（RISC-V，48 脚） |
| 内核 | 青稞 V4F，144 MHz |
| Flash / SRAM | 128 KB / 32 KB |
| 浮点 | 当前为**软浮点**（`-march=rv32imacxw -mabi=ilp32`，硬件浮点已关闭） |
| 调试器 | WCH-Link（1-wire serial） |
| 栅极驱动 | FD6288（CHxN → 上管 HIN，CHx → 下管 LIN，占空比取反） |
| 电流采样 | INA240A2 + 10 mΩ 采样电阻，增益 50 V/V，ADC REF 3.3 V |
| 编码器 | MT6701（SSI 接口，14 位绝对角度 + 4 位状态 + 6 位 CRC） |

## 功能模块

| 模块 | 路径 | 说明 |
|---|---|---|
| OLED | `hardware/OLED/` | I2C1（PB6/PB7）SSD1306 驱动，标准外设库实现，含总线恢复与超时保护 |
| 菜单/按键 | `hardware/menu/` | 五向开关（GPIOB），EXTI 双边沿 + TIM3(1 ms) 消抖/长按/连发，三屏状态机 |
| MT6701 | `hardware/MT6701/` | SPI1（Mode 1）SSI 读取 24 位帧，CRC6（x⁶+x+1）校验 |
| FOC 算法 | `hardware/FOC/inc/foc.h`、`hardware/FOC/src/foc.c` | 纯算法层，零硬件依赖：Clarke / Park / 反 Park / 七段式 SVPWM / 位置式 PID（clamping 抗饱和） |
| BLDC 驱动 | `hardware/FOC/inc/bldc.h`、`hardware/FOC/src/bldc.c` | TIM1 三相互补 PWM + ADC1 注入组（TIM1 TRGO 触发）+ 零点校准 + MT6701 + FOC_Update 调度 |
| VOFA 调试 | `hardware/VOFA/` | USART2（PA2/PA3，115200）+ DMA1_CH7，JustFloat 5 通道（目标/位置/速度/Iq/Id） |

## 引脚分配

| 功能 | 引脚 | 备注 |
|---|---|---|
| OLED I2C1 | PB6/PB7 | SCL/SDA |
| 五向按键 | PB0/PB1/PB10/PB11/PB12 | UP/DOWN/LEFT/RIGHT/OK，低电平有效 |
| MT6701 SPI1 | PA4/PA5/PA6/PA7 | CS/SCK/MISO/MOSI（MOSI 悬空，SSI 只读） |
| TIM1 PWM | PA8/PA9/PA10 | CH1/CH2/CH3 高侧（FD6288 LIN） |
| TIM1 互补 | PB13/PB14/PB15 | CH1N/CH2N/CH3N 低侧（FD6288 HIN） |
| ADC1 注入 | PA0/PA1 | IA/IB 双电阻采样 |
| VOFA USART2 | PA2/PA3 | TX/RX |

## 软件架构

```
main()
  ├─ NVIC_PriorityGroupConfig(Group2)   // 2 位抢占 / 2 位子优先级
  ├─ SystemCoreClockUpdate() / Delay_Init()
  ├─ Menu_Init()    // OLED + 五向按键(EXTI+TIM3) + 菜单界面 (内部含 KEY_Init)
  ├─ VOFA_Init()    // VOFA+ JustFloat 调试串口 (USART2 + DMA1_CH7, 115200)
  ├─ BLDC_Init()    // TIM1 互补PWM + ADC1 注入 + MT6701 + 零点校准 + FOC_Init
  │                 + ADC1 JEOC 中断使能 (20kHz FOC 电流环自动运行)
  │     └─ 初始化后 foc_mode=IDLE → FOC_Update 返回 0 → MOE 关, 电机不动
  └─ while(1) {
       Menu_Run(KEY_GetEvent());   // 三屏状态机, 确认后设 foc_mode/foc_target
       VOFA_Send(vofa_buf, 5);     // JustFloat 5 通道非阻塞 DMA (目标/位置/速度/Iq/Id)
       Delay_Ms(10);
  }
```

**FOC 控制环**（ADC1 JEOC 中断，20kHz，自动运行，不占主循环）：
- `IDLE`：MOE 关闭，电机不动，复位 PID 积分
- `OPENLOOP`：`FOC_Update` 内置开环分支（`ol_angle` 积分 + 固定电压 SVPWM）
- `POSITION` / `SPEED`：位置环→速度环→电流环 / 速度环→电流环 闭环
- `FAULT`：过流触发（`ia/ib > FOC_I_MAX`），关输出

**菜单交互**（三屏状态机）：模式选择 → 目标值设定 → 运行监视。运行屏 OK 长按 = 紧急停机。

**中断属性约定**（WCH GCC）：
- `__attribute__((interrupt("WCH-Interrupt-fast")))` — 硬件压栈，用于高优先级高频 ISR
- `__attribute__((interrupt()))` — 软件压栈，用于低优先级 ISR
- 不支持 `"WCH-Interrupt"` 字串形式

**电流环数据流**（闭环）：ADC1 注入中断（TIM1 TRGO 触发）→ 读 IA/IB → `FOC_Update()`（Clarke→Park→PID→反 Park→SVPWM）→ 写 TIM1 CCR。

## 关键参数（`hardware/FOC/inc/foc.h`）

| 宏 | 值 | 说明 |
|---|---|---|
| `FOC_PWM_FREQ` | 35000 Hz | PWM 开关频率 |
| `FOC_POLE_PAIRS` | 7 | 电机极对数 |
| `FOC_VBUS` | 11.3 V | 母线电压（固定值，无采样） |
| `FOC_R_SHUNT` | 0.01 Ω | 采样电阻 |
| `FOC_INA_GAIN` | 50 | INA240A2 增益 |
| `FOC_I_MAX` | 3.0 A | 过流保护阈值 |
| `FOC_DEADTIME_NS` | 500 ns | 死区 |
| `FOC_OL_VOLTAGE` | 7.0 V | 开环电压 |
| `FOC_OL_SPEED` | 10 Hz | 开环电频率 |

PID 初始值（Iq/Id/速度/位置）在 `foc.h` 中，上板后用 VOFA 在线整定。

## 编译与调试

**IDE**：MounRiver Studio  
**工具链**：WCH RISC-V GCC8  
**优化**：`-Os`（size），`gnu99`，`-ffunction-sections -fdata-sections`，`--gc-sections`  
**链接**：`Ld/Link.ld`，newlib nano，链接 libm  
**架构**：`rv32imacxw` / `ilp32`（软浮点）

编译产物：`obj/CH32V303FOC.elf`、`obj/CH32V303FOC.hex`  
烧录配置见 `flashConfig`（WCH-Link，地址 0x08000000）。  
调试使用 OpenOCD（`wch-riscv.cfg`）+ GDB，端口 3333。

## 目录结构

```
CH32V303FOC/
├── Core/            系统时钟/中断
├── Debug/           printf 重定向
├── Ld/              链接脚本
├── Peripheral/      CH32V30x 标准外设库
├── Startup/          启动汇编
├── User/            main.c
├── hardware/
│   ├── OLED/        OLED + 字模
│   ├── menu/        五向按键 + 菜单
│   ├── MT6701/      磁编码器
│   ├── FOC/         FOC 算法 + BLDC 驱动
│   └── VOFA/        调试串口
├── obj/             编译产物
├── CH32V303FOC.wvproj
└── CH32V303FOC.launch
```

## VOFA+ 调试

`VOFA_Send()` 在主循环调用，DMA 非阻塞发送。JustFloat 协议帧：`N × float(小端) + 0x00 0x00 0x80 0x7F`。

通道定义：
1. 目标值
2. 位置（deg）
3. 速度（RPM）
4. Iq（A）
5. Id（A）

## 已知问题与注意

- **PID 待整定**：ADC1 JEOC 中断（20kHz）已启用，`FOC_Update()` 自动运行。`OPENLOOP` 开环可直接使用；`POSITION`/`SPEED` 闭环的 PID 初始值在 `foc.h` 中，需上板用 VOFA 在线整定后方能稳定运行。
- **软浮点**：硬件浮点关闭后，ADC ISR 中已删除浮点寄存器保存/恢复汇编代码。`FOC_Update` 大量浮点运算走软浮点库，ISR 耗时约 30~50µs，接近周期上限，后续可考虑重新启用硬浮点（需恢复寄存器保存/恢复汇编）。
- **ADC 校准**：校准 `while` 循环已加超时（100000 次）+ 默认值 2048 兜底。
- **I2C 鲁棒性**：OLED 驱动采用分步 TXE/BTF 标志检查 + 9 时钟脉冲总线恢复 + 软件复位 + 超时，避免 `I2C_CheckEvent` 卡死。

## 许可

基于 WCH 官方 SDK 与标准外设库开发，遵循原厂许可条款。
