#include "bsp_as5600.h"
#include "i2c.h"

static I2C_HandleTypeDef *s_i2c_handles[ENCODER_COUNT];
static uint16_t s_zero_offsets[ENCODER_COUNT];

void BSP_AS5600_Init(void)
{
    s_i2c_handles[ENCODER_M1] = &hi2c1;
    s_i2c_handles[ENCODER_M2] = &hi2c2;
    s_i2c_handles[ENCODER_M3] = &hi2c3;

    BSP_AS5600_CalibrateZero();
}

bool BSP_AS5600_ReadAngle(EncoderIndex_t encoder, uint16_t *raw_angle)
{
    if (encoder >= ENCODER_COUNT || raw_angle == NULL)
        return false;

    uint8_t data[2];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        s_i2c_handles[encoder],
        AS5600_ADDR << 1,
        AS5600_REG_RAW_ANGLE,
        I2C_MEMADD_SIZE_8BIT,
        data, 2, 100
    );

    if (ret != HAL_OK)
        return false;

    *raw_angle = ((uint16_t)data[0] << 8) | data[1];
    return true;
}

uint16_t BSP_AS5600_GetAngle(EncoderIndex_t encoder)
{
    uint16_t raw;
    if (!BSP_AS5600_ReadAngle(encoder, &raw))
        return 0;

    uint16_t offset = s_zero_offsets[encoder];
    if (raw >= offset)
        return raw - offset;
    else
        return raw + 4096 - offset;
}

void BSP_AS5600_CalibrateZero(void)
{
    for (int i = 0; i < ENCODER_COUNT; i++) {
        uint16_t raw;
        if (BSP_AS5600_ReadAngle((EncoderIndex_t)i, &raw))
            s_zero_offsets[i] = raw;
        else
            s_zero_offsets[i] = 0;
    }
}
