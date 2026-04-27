#include "bsp_ps2.h"
#include "main.h"

#define PS2_DELAY_CYCLES 120U

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

static uint8_t PS2_TransferByte(uint8_t tx_data)
{
    uint8_t rx_data = 0;

    for (uint8_t i = 0; i < 8U; i++) {
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

    for (uint8_t i = 0; i < len; i++) {
        PS2_TransferByte(cmd[i]);
    }

    PS2_CS_Write(GPIO_PIN_SET);
    HAL_Delay(16);
}

void BSP_PS2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = PS2_DAT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PS2_DAT_GPIO_Port, &GPIO_InitStruct);

    PS2_CLK_Write(GPIO_PIN_SET);
    PS2_CS_Write(GPIO_PIN_SET);
    PS2_CMD_Write(GPIO_PIN_SET);

    HAL_Delay(50);

    const uint8_t enter_config[] = {0x01, 0x43, 0x00, 0x01, 0x00};
    const uint8_t set_mode[] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    const uint8_t exit_config[] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

    PS2_SendCommand(enter_config, 5);
    PS2_SendCommand(set_mode, 9);
    PS2_SendCommand(exit_config, 9);
}

bool BSP_PS2_ReadData(PS2_Data_t *ps2_data)
{
    if (ps2_data == NULL) {
        return false;
    }

    uint8_t raw[9] = {0};

    PS2_CS_Write(GPIO_PIN_RESET);
    PS2_Delay();

    raw[0] = PS2_TransferByte(0x01);
    raw[1] = PS2_TransferByte(0x42);
    raw[2] = PS2_TransferByte(0x00);

    if (raw[2] != 0x5A) {
        PS2_CS_Write(GPIO_PIN_SET);
        return false;
    }

    raw[3] = PS2_TransferByte(0x00);
    raw[4] = PS2_TransferByte(0x00);
    raw[5] = PS2_TransferByte(0x00);
    raw[6] = PS2_TransferByte(0x00);
    raw[7] = PS2_TransferByte(0x00);
    raw[8] = PS2_TransferByte(0x00);

    PS2_CS_Write(GPIO_PIN_SET);

    ps2_data->buttons = (uint16_t)((raw[3] << 8) | raw[4]);
    ps2_data->RX = raw[5];
    ps2_data->RY = raw[6];
    ps2_data->LX = raw[7];
    ps2_data->LY = raw[8];

    return true;
}
