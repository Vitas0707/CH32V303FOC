/**
 * @file menu.c
 * @brief FOC多级菜单 (三屏状态机: 模式选择 -> 目标值设定 -> 运行监视)
 *
 * 交互设计:
 *  屏1 模式选择: 上/下切换模式, OK进入设定; 反色条显示当前选中项
 *  屏2 目标设定: 左/右移动数位, 上/下当前位加减(长按连发), OK确认启动
 *                位置XXX.X deg, 速度XXXX RPM; OK长按取消返回
 *  屏3 运行监视: 显示目标/位置/速度/电流实际量, 10Hz刷新
 *                上/下在线微调目标(位置1deg/速度10RPM), OK长按紧急停机
 *
 * @note 本模块只操作OLED显存和交互变量, 不做FOC控制
 * @note OLED_ShowFrame为阻塞I2C传输(约30ms/帧), 置于主循环, 不影响FOC中断
 * @note 按键事件由EXTI中断+TIM3(1ms)异步推入环形缓冲, 主循环KEY_GetEvent()取走
 *       OLED刷新期间(25ms)的按键不会丢失(EXTI捕获后入队, 刷新完再处理)
 */
#include "menu.h"
#include "oled.h"
#include "debug.h"

// ==================== 界面状态 ====================
typedef enum {
    SCR_MODE = 0, // 屏1: 模式选择
    SCR_SET,      // 屏2: 目标值设定
    SCR_RUN,      // 屏3: 运行监视
} MenuScreen;

static MenuScreen screen;   // 当前界面
static uint8_t selMode;     // 0=位置模式 1=速度模式
static uint8_t digits[4];   // 屏2的四位数字 (位置: XXX.X)
static uint8_t cursor;      // 屏2当前编辑位 0-3
static uint8_t blinkOn;     // 屏2光标闪烁状态
static uint8_t blinkShown;  // 上次绘制时的闪烁状态
static uint32_t lastBlinkMs; // 上次闪烁翻转时刻
static uint32_t lastRefreshMs; // 上次运行屏刷新时刻

// 每种模式的上次设定值 (上电默认: 90.0deg / 500RPM)
static uint16_t lastPos = 900; // 0.1deg单位
static uint16_t lastSpeed = 500;

// FOC交互变量定义已移至 foc.c (由foc.h声明)

// ==================== 工具函数 ====================

// 前置声明 (绘制函数)
static void Draw_ModeScreen(void);
static void Draw_SetScreen(void);
static void Draw_RunScreen(void);

/**
 * @brief 浮点数转字符串 (不使用%f, 避免拉入浮点版printf增大代码体积)
 * @param buf 输出缓冲(需>=12字节)
 * @param v 数值
 * @param dec 小数位数(0-3)
 */
static void fmtFloat(char *buf, float v, uint8_t dec)
{
    uint16_t i = 0;
    uint32_t scale = 1;
    uint32_t u, ip, fp;
    char tmp[10];
    uint8_t t = 0;
    uint8_t k;

    for (k = 0; k < dec; k++)
        scale *= 10;

    if (v < 0)
    {
        buf[i++] = '-';
        v = -v;
    }
    u = (uint32_t)(v * scale + 0.5f); // 四舍五入到最小位
    ip = u / scale;
    fp = u % scale;

    if (ip == 0)
        buf[i++] = '0';
    while (ip)
    {
        tmp[t++] = '0' + ip % 10;
        ip /= 10;
    }
    while (t)
        buf[i++] = tmp[--t];

    if (dec)
    {
        buf[i++] = '.';
        while (scale > 1)
        {
            scale /= 10;
            buf[i++] = '0' + (fp / scale) % 10;
        }
    }
    buf[i] = '\0';
}

/** @brief 屏2载入上次设定值 */
static void LoadDigits(void)
{
    uint16_t v = (selMode == 0) ? lastPos : lastSpeed;
    digits[0] = (v / 1000) % 10;
    digits[1] = (v / 100) % 10;
    digits[2] = (v / 10) % 10;
    digits[3] = v % 10;
}

/** @brief 屏2合成数值(0.1deg或RPM), 限幅后返回 */
static uint16_t ComposeDigits(void)
{
    uint16_t v = digits[0] * 1000 + digits[1] * 100 + digits[2] * 10 + digits[3];
    uint16_t max = (selMode == 0) ? (uint16_t)(FOC_POS_MAX * 10)
                                  : (uint16_t)FOC_SPEED_MAX;
    if (v > max)
        v = max;
    return v;
}

// ==================== 屏1: 模式选择 ====================

/** @brief 绘制模式选项 (选中项反色) */
static void Draw_Item(uint8_t x, uint8_t y, char *text, const Font *font, uint8_t selected)
{
    if (selected)
    {
        OLED_DrawFilledRectangle(x - 2, y - 2, 67, 19, 0); // 反色底
        OLED_PrintString(x, y, text, font, OLED_COLOR_REVERSED);
    }
    else
    {
        OLED_PrintString(x, y, text, font, OLED_COLOR_NORMAL);
    }
}

static void Draw_ModeScreen(void)
{
    OLED_NewFrame();
    OLED_DrawFilledRectangle(0, 0, 127, 15, 0); // 标题条
    OLED_PrintString(16, 0, "输出模式选择", &select_modefont16x16, OLED_COLOR_REVERSED);
    Draw_Item(8, 22, "位置模式", &angle_modefont16x16, selMode == 0);
    Draw_Item(8, 44, "速度模式", &speed_modefont16x16, selMode == 1);
    OLED_ShowFrame();
}

static void Handle_Mode(KEY_Event evt)
{
    switch (evt)
    {
    case KEY_EVT_UP:
    case KEY_EVT_DOWN:
    case KEY_EVT_UP_R:
    case KEY_EVT_DOWN_R:
        selMode ^= 1;
        Draw_ModeScreen();
        break;

    case KEY_EVT_OK: // 进入目标值设定
        LoadDigits();
        cursor = 3; // 从最低位开始编辑
        screen = SCR_SET;
        blinkShown = 0xFF;
        lastBlinkMs = KEY_GetSysMs();
        Draw_SetScreen();
        break;

    default:
        break;
    }
}

// ==================== 屏2: 目标值设定 ====================

static void Draw_SetScreen(void)
{
    uint8_t i;
    uint8_t xBase = (selMode == 0) ? 34 : 40; // "123.4"60px / "1234"48px 居中

    OLED_NewFrame();

    // 标题条: 设置目标值
    OLED_DrawFilledRectangle(0, 0, 127, 15, 0);
    OLED_PrintString(4, 0, "设置目标值", &set_targetfont16x16, OLED_COLOR_REVERSED);

    // 大数字(24x12) + 光标位闪烁反显
    for (i = 0; i < 4; i++)
    {
        uint8_t dispIdx = (selMode == 0 && i == 3) ? 4 : i; // 位置模式: 跳过小数点
        uint8_t x = xBase + dispIdx * 12;
        uint8_t inv = (i == cursor) && blinkOn;
        char ch = '0' + digits[i];
        if (inv)
            OLED_DrawFilledRectangle(x - 1, 19, 13, 25, 0);
        OLED_PrintASCIIChar(x, 20, ch, &afont24x12,
                            inv ? OLED_COLOR_REVERSED : OLED_COLOR_NORMAL);
    }
    if (selMode == 0)
        OLED_PrintASCIIChar(xBase + 36, 20, '.', &afont24x12, OLED_COLOR_NORMAL);

    // 提示行
    OLED_PrintASCIIString(1, 48, "OK:Start LR:Move UD:+-", &afont8x6, OLED_COLOR_NORMAL);
    if (selMode == 0)
        OLED_PrintASCIIString(1, 57, "Range 0~360.0 deg", &afont8x6, OLED_COLOR_NORMAL);
    else
        OLED_PrintASCIIString(1, 57, "Range 0~9999 RPM", &afont8x6, OLED_COLOR_NORMAL);

    OLED_ShowFrame();
}

static void Handle_Set(KEY_Event evt)
{
    uint16_t v;
    uint8_t needDraw = 0;

    switch (evt)
    {
    case KEY_EVT_UP:
    case KEY_EVT_UP_R:
        digits[cursor] = (digits[cursor] + 1) % 10;
        needDraw = 1;
        break;

    case KEY_EVT_DOWN:
    case KEY_EVT_DOWN_R:
        digits[cursor] = (digits[cursor] + 9) % 10;
        needDraw = 1;
        break;

    case KEY_EVT_LEFT:
    case KEY_EVT_LEFT_R:
        cursor = (cursor + 3) % 4;
        needDraw = 1;
        break;

    case KEY_EVT_RIGHT:
    case KEY_EVT_RIGHT_R:
        cursor = (cursor + 1) % 4;
        needDraw = 1;
        break;

    case KEY_EVT_OK: // 确认: 限幅后启动输出, 进入运行监视
        v = ComposeDigits();
        if (selMode == 0)
        {
            lastPos = v;
            foc_target = v * 0.1f;
            foc_mode = FOC_MODE_POSITION;
        }
        else
        {
            lastSpeed = v;
            foc_target = (float)v;
            foc_mode = FOC_MODE_SPEED;
        }
        screen = SCR_RUN;
        Draw_RunScreen();
        return;

    case KEY_EVT_OK_L: // 长按取消, 返回模式选择
        screen = SCR_MODE;
        Draw_ModeScreen();
        return;

    default:
        break;
    }

    // 光标闪烁重绘 (250ms翻转一次, 基于TIM3真实毫秒时基)
    {
        uint32_t now = KEY_GetSysMs();
        if (now - lastBlinkMs >= 250)
        {
            lastBlinkMs = now;
            blinkOn = !blinkOn;
            needDraw = 1;
        }
        if (blinkOn != blinkShown || needDraw)
        {
            blinkShown = blinkOn;
            Draw_SetScreen();
        }
    }
}

// ==================== 屏3: 运行监视 ====================

static void Draw_RunScreen(void)
{
    char buf[12];
    float tgt = foc_target; // volatile快照
    float pos = foc_actual_position;
    float spd = foc_actual_speed;
    float iq = foc_actual_iq;

    OLED_NewFrame();

    // 行0(反色): 目标值, 上/下可在线调整
    OLED_DrawFilledRectangle(0, 0, 127, 15, 0);
    OLED_PrintString(0, 0, "目标", &set_targetfont16x16, OLED_COLOR_REVERSED);
    if (selMode == 0)
    {
        fmtFloat(buf, tgt, 1);
        OLED_PrintASCIIString(40, 0, buf, &afont16x8, OLED_COLOR_REVERSED);
        OLED_PrintASCIIString(40 + 8 * strlen(buf), 0, "deg", &afont16x8, OLED_COLOR_REVERSED);
    }
    else
    {
        fmtFloat(buf, tgt, 0);
        OLED_PrintASCIIString(40, 0, buf, &afont16x8, OLED_COLOR_REVERSED);
        OLED_PrintASCIIString(40 + 8 * strlen(buf), 0, "RPM", &afont16x8, OLED_COLOR_REVERSED);
    }

    // 行1: 实际位置
    OLED_PrintString(0, 16, "位置", &angle_modefont16x16, OLED_COLOR_NORMAL);
    fmtFloat(buf, pos, 1);
    OLED_PrintASCIIString(40, 16, buf, &afont16x8, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(40 + 8 * strlen(buf), 16, "deg", &afont16x8, OLED_COLOR_NORMAL);

    // 行2: 实际角速度
    OLED_PrintString(0, 32, "速度", &speed_modefont16x16, OLED_COLOR_NORMAL);
    fmtFloat(buf, spd, 0);
    OLED_PrintASCIIString(40, 32, buf, &afont16x8, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(40 + 8 * strlen(buf), 32, "RPM", &afont16x8, OLED_COLOR_NORMAL);

    // 行3: 实际电流(Iq)
    OLED_PrintString(0, 48, "电流", &currentfont16x16, OLED_COLOR_NORMAL);
    fmtFloat(buf, iq, 2);
    OLED_PrintASCIIString(40, 48, buf, &afont16x8, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(40 + 8 * strlen(buf), 48, "A", &afont16x8, OLED_COLOR_NORMAL);

    OLED_ShowFrame();
}

static void Handle_Run(KEY_Event evt)
{
    float t;

    switch (evt)
    {
    case KEY_EVT_UP: // 在线微调目标: 位置1deg / 速度10RPM
    case KEY_EVT_UP_R:
        t = foc_target + ((selMode == 0) ? 1.0f : 10.0f);
        if (t > ((selMode == 0) ? FOC_POS_MAX : FOC_SPEED_MAX))
            t = (selMode == 0) ? FOC_POS_MAX : FOC_SPEED_MAX;
        foc_target = t;
        Draw_RunScreen();
        break;

    case KEY_EVT_DOWN:
    case KEY_EVT_DOWN_R:
        t = foc_target - ((selMode == 0) ? 1.0f : 10.0f);
        if (t < 0.0f)
            t = 0.0f;
        foc_target = t;
        Draw_RunScreen();
        break;

    case KEY_EVT_OK_L: // 长按紧急停机, 返回模式选择
        foc_mode = FOC_MODE_IDLE;
        screen = SCR_MODE;
        Draw_ModeScreen();
        return;

    default:
        break;
    }

    // 实际量10Hz刷新 (100ms一帧, 基于TIM3真实毫秒时基)
    {
        uint32_t now = KEY_GetSysMs();
        if (now - lastRefreshMs >= 100)
        {
            lastRefreshMs = now;
            Draw_RunScreen();
        }
    }
}

// ==================== 对外接口 ====================

/**
 * @brief 菜单初始化: 按键+OLED, 并显示屏1
 * @note 需在Delay_Init()之后调用
 */
void Menu_Init(void)
{
    KEY_Init();
    Delay_Ms(50); // 等待OLED上电稳定 (MCU启动快于OLED)
    OLED_Init();  // 内部完成I2C1(PB6/PB7)初始化

    screen = SCR_MODE;
    selMode = 0;
    lastBlinkMs = 0;
    lastRefreshMs = 0;
    Draw_ModeScreen();
}

/**
 * @brief 菜单状态机, 主循环中调用(无需固定周期)
 * @param evt KEY_GetEvent()返回的按键事件, KEY_EVT_NONE时仍需调用以驱动闪烁/刷新
 */
void Menu_Run(KEY_Event evt)
{
    switch (screen)
    {
    case SCR_MODE:
        Handle_Mode(evt);
        break;
    case SCR_SET:
        Handle_Set(evt);
        break;
    case SCR_RUN:
        Handle_Run(evt);
        break;
    }
}
