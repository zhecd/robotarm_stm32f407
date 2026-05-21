/**
 * @file    bsp_ps2.c
 * @brief   PS2 controller bit-bang SPI implementation / PS2 手柄位打 SPI 实现
 * @ingroup bsp
 */

#include "bsp/bsp_ps2.h"

#define PS2_DELAY_CYCLES            120U
#define PS2_FRAME_HEADER_OK         0x5AU
#define PS2_MODE_DIGITAL            0x41U
#define PS2_MODE_ANALOG_RED         0x73U
#define PS2_MODE_ANALOG_PRESSURE    0x79U
#define PS2_REINIT_RETRY_THRESHOLD  3U

static bool    s_analog_mode        = false;
static uint8_t s_invalid_mode_count = 0U;

/* ── Low-level GPIO helpers (BSRR for atomic access) / 底层 GPIO (BSRR 原子操作) ── */

static inline void CMD_Write(GPIO_PinState s)
{
    if (s == GPIO_PIN_SET)
        PS2_CMD_GPIO_Port->BSRR = PS2_CMD_Pin;
    else
        PS2_CMD_GPIO_Port->BSRR = ((uint32_t)PS2_CMD_Pin << 16U);
}

static inline void CLK_Write(GPIO_PinState s)
{
    if (s == GPIO_PIN_SET)
        PS2_CLK_GPIO_Port->BSRR = PS2_CLK_Pin;
    else
        PS2_CLK_GPIO_Port->BSRR = ((uint32_t)PS2_CLK_Pin << 16U);
}

static inline void CS_Write(GPIO_PinState s)
{
    if (s == GPIO_PIN_SET)
        PS2_CS_GPIO_Port->BSRR = PS2_CS_Pin;
    else
        PS2_CS_GPIO_Port->BSRR = ((uint32_t)PS2_CS_Pin << 16U);
}

static inline GPIO_PinState DAT_Read(void)
{
    return (PS2_DAT_GPIO_Port->IDR & PS2_DAT_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static inline void PS2_Delay(void)
{
    for (volatile uint32_t d = 0; d < PS2_DELAY_CYCLES; d++) { __NOP(); }
}

static bool IsSupportedAnalogMode(uint8_t mode)
{
    return (mode == PS2_MODE_ANALOG_RED) || (mode == PS2_MODE_ANALOG_PRESSURE);
}

/* ── Core SPI-like byte transfer / 核心 SPI 字节传输 ── */

static uint8_t TransferByte(uint8_t tx)
{
    uint8_t rx = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        CMD_Write((tx & (1U << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        CLK_Write(GPIO_PIN_RESET);  PS2_Delay();
        if (DAT_Read() == GPIO_PIN_SET) rx |= (1U << i);
        CLK_Write(GPIO_PIN_SET);    PS2_Delay();
    }
    PS2_Delay();
    return rx;
}

static void SendCommand(const uint8_t *cmd, uint8_t len)
{
    CS_Write(GPIO_PIN_RESET);
    PS2_Delay();
    for (uint8_t i = 0U; i < len; i++) { TransferByte(cmd[i]); }
    CS_Write(GPIO_PIN_SET);
    HAL_Delay(16U);
}

/* ── Public API / 公开接口 ── */

void BSP_PS2_Init(void)
{
    GPIO_InitTypeDef gi = {0};
    gi.Pin  = PS2_DAT_Pin;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PS2_DAT_GPIO_Port, &gi);

    CLK_Write(GPIO_PIN_SET);
    CS_Write(GPIO_PIN_SET);
    CMD_Write(GPIO_PIN_SET);
    HAL_Delay(50U);

    /* Enter config mode -> set analog mode -> exit config / 进入配置模式->设模拟模式->退出配置 */
    {
        const uint8_t enter_cfg[] = {0x01U, 0x43U, 0x00U, 0x01U, 0x00U};
        const uint8_t set_mode[]  = {0x01U, 0x44U, 0x00U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U};
        const uint8_t exit_cfg[]  = {0x01U, 0x43U, 0x00U, 0x00U, 0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU};
        SendCommand(enter_cfg, (uint8_t)sizeof(enter_cfg));
        SendCommand(set_mode,  (uint8_t)sizeof(set_mode));
        SendCommand(exit_cfg,  (uint8_t)sizeof(exit_cfg));
    }

    s_analog_mode        = false;
    s_invalid_mode_count = 0U;
}

bool BSP_PS2_ReadData(PS2_Data_t *data)
{
    if (!data) return false;

    uint8_t raw[9] = {0};
    CS_Write(GPIO_PIN_RESET);
    PS2_Delay();

    raw[0] = TransferByte(0x01U);
    raw[1] = TransferByte(0x42U);
    raw[2] = TransferByte(0x00U);

    /* Frame header check / 帧头校验 */
    if (raw[2] != PS2_FRAME_HEADER_OK) {
        CS_Write(GPIO_PIN_SET);
        s_analog_mode = false;
        return false;
    }

    /* Mode check + auto reinit / 模式检查 + 自动重新初始化 */
    if (!IsSupportedAnalogMode(raw[1])) {
        CS_Write(GPIO_PIN_SET);
        s_analog_mode = false;
        if (raw[1] == PS2_MODE_DIGITAL) {
            s_invalid_mode_count++;
            if (s_invalid_mode_count >= PS2_REINIT_RETRY_THRESHOLD)
                BSP_PS2_Init();
        } else {
            s_invalid_mode_count = 0U;
        }
        return false;
    }

    s_analog_mode        = true;
    s_invalid_mode_count = 0U;

    /* Read remaining data bytes / 读取剩余数据字节 */
    raw[3] = TransferByte(0x00U);
    raw[4] = TransferByte(0x00U);
    raw[5] = TransferByte(0x00U);
    raw[6] = TransferByte(0x00U);
    raw[7] = TransferByte(0x00U);
    raw[8] = TransferByte(0x00U);

    CS_Write(GPIO_PIN_SET);

    data->buttons = (uint16_t)((raw[3] << 8) | raw[4]);
    data->RX = raw[5];
    data->RY = raw[6];
    data->LX = raw[7];
    data->LY = raw[8];
    return true;
}

bool BSP_PS2_IsAnalogMode(void)
{
    return s_analog_mode;
}
