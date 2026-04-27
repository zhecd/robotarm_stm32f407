#include "bsp_ps2.h"
#include "main.h"

#define PS2_DELAY_CYCLES          120U
#define PS2_FRAME_HEADER_OK       0x5AU
#define PS2_MODE_DIGITAL          0x41U
#define PS2_MODE_ANALOG_RED       0x73U
#define PS2_MODE_ANALOG_PRESSURE  0x79U
#define PS2_REINIT_RETRY_THRESHOLD 3U

static bool s_ps2_is_analog_mode = false;
static uint8_t s_ps2_invalid_mode_count = 0U;

static inline void PS2_CMD_Write(GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) {
        PS2_CMD_GPIO_Port->BSRR = PS2_CMD_Pin;
    } else {
        PS2_CMD_GPIO_Port->BSRR = ((uint32_t)PS2_CMD_Pin << 16U);
    }
}

static inline void PS2_CLK_Write(GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) {
        PS2_CLK_GPIO_Port->BSRR = PS2_CLK_Pin;
    } else {
        PS2_CLK_GPIO_Port->BSRR = ((uint32_t)PS2_CLK_Pin << 16U);
    }
}

static inline void PS2_CS_Write(GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) {
        PS2_CS_GPIO_Port->BSRR = PS2_CS_Pin;
    } else {
        PS2_CS_GPIO_Port->BSRR = ((uint32_t)PS2_CS_Pin << 16U);
    }
}

static inline GPIO_PinState PS2_DAT_Read(void)
{
    return ((PS2_DAT_GPIO_Port->IDR & PS2_DAT_Pin) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

static inline void PS2_Delay(void)
{
    for (volatile uint32_t delay = 0; delay < PS2_DELAY_CYCLES; delay++) {
        __NOP();
    }
}

static bool PS2_IsSupportedAnalogMode(uint8_t mode_byte)
{
    return (mode_byte == PS2_MODE_ANALOG_RED) || (mode_byte == PS2_MODE_ANALOG_PRESSURE);
}

static uint8_t PS2_TransferByte(uint8_t tx_data)
{
    uint8_t rx_data = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        PS2_CMD_Write((tx_data & (1U << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        PS2_CLK_Write(GPIO_PIN_RESET);
        PS2_Delay();

        if (PS2_DAT_Read() == GPIO_PIN_SET) {
            rx_data |= (1U << i);
        }

        PS2_CLK_Write(GPIO_PIN_SET);
        PS2_Delay();
    }

    PS2_Delay();
    return rx_data;
}

static void PS2_SendCommand(const uint8_t *cmd, uint8_t len)
{
    PS2_CS_Write(GPIO_PIN_RESET);
    PS2_Delay();

    for (uint8_t i = 0U; i < len; i++) {
        PS2_TransferByte(cmd[i]);
    }

    PS2_CS_Write(GPIO_PIN_SET);
    HAL_Delay(16U);
}

void BSP_PS2_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = PS2_DAT_Pin;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PS2_DAT_GPIO_Port, &gpio_init);

    PS2_CLK_Write(GPIO_PIN_SET);
    PS2_CS_Write(GPIO_PIN_SET);
    PS2_CMD_Write(GPIO_PIN_SET);

    HAL_Delay(50U);

    {
        const uint8_t enter_config[] = {0x01U, 0x43U, 0x00U, 0x01U, 0x00U};
        const uint8_t set_mode[] = {0x01U, 0x44U, 0x00U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U};
        const uint8_t exit_config[] = {0x01U, 0x43U, 0x00U, 0x00U, 0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU};

        PS2_SendCommand(enter_config, (uint8_t)sizeof(enter_config));
        PS2_SendCommand(set_mode, (uint8_t)sizeof(set_mode));
        PS2_SendCommand(exit_config, (uint8_t)sizeof(exit_config));
    }

    s_ps2_is_analog_mode = false;
    s_ps2_invalid_mode_count = 0U;
}

bool BSP_PS2_ReadData(PS2_Data_t *ps2_data)
{
    if (ps2_data == NULL) {
        return false;
    }

    uint8_t raw[9] = {0};

    PS2_CS_Write(GPIO_PIN_RESET);
    PS2_Delay();

    raw[0] = PS2_TransferByte(0x01U);
    raw[1] = PS2_TransferByte(0x42U);
    raw[2] = PS2_TransferByte(0x00U);

    if (raw[2] != PS2_FRAME_HEADER_OK) {
        PS2_CS_Write(GPIO_PIN_SET);
        s_ps2_is_analog_mode = false;
        return false;
    }

    if (!PS2_IsSupportedAnalogMode(raw[1])) {
        PS2_CS_Write(GPIO_PIN_SET);
        s_ps2_is_analog_mode = false;

        if (raw[1] == PS2_MODE_DIGITAL) {
            s_ps2_invalid_mode_count++;
            if (s_ps2_invalid_mode_count >= PS2_REINIT_RETRY_THRESHOLD) {
                BSP_PS2_Init();
            }
        } else {
            s_ps2_invalid_mode_count = 0U;
        }

        return false;
    }

    s_ps2_is_analog_mode = true;
    s_ps2_invalid_mode_count = 0U;

    raw[3] = PS2_TransferByte(0x00U);
    raw[4] = PS2_TransferByte(0x00U);
    raw[5] = PS2_TransferByte(0x00U);
    raw[6] = PS2_TransferByte(0x00U);
    raw[7] = PS2_TransferByte(0x00U);
    raw[8] = PS2_TransferByte(0x00U);

    PS2_CS_Write(GPIO_PIN_SET);

    ps2_data->buttons = (uint16_t)((raw[3] << 8) | raw[4]);
    ps2_data->RX = raw[5];
    ps2_data->RY = raw[6];
    ps2_data->LX = raw[7];
    ps2_data->LY = raw[8];

    return true;
}

bool BSP_PS2_IsAnalogMode(void)
{
    return s_ps2_is_analog_mode;
}
