#include "bsp_as5600.h"
#include "i2c.h"

AS5600_t Encoder_M1;
AS5600_t Encoder_M2;
AS5600_t Encoder_M3;

/* 单次读取原始物理角度 (0~360), I2C 失败返回负值 */
static float AS5600_ReadPhysicalDeg(AS5600_t *enc)
{
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(enc->hi2c, AS5600_ADDR_WRITE, AS5600_REG_RAW_ANGLE,
                          I2C_MEMADD_SIZE_8BIT, data, 2, 100) != HAL_OK) {
        return -1.0f;
    }
    /* 低 4 bit 是 AGC, 必须用 0x0FFF 屏蔽, 否则角度会随 AGC 跳动 ±1.4° */
    uint16_t raw_angle = (((uint16_t)data[0] << 8) | data[1]) & 0x0FFFU;
    return (float)raw_angle * 360.0f / 4096.0f;
}

/* 多次采样取平均，滤除磁编瞬时噪声 */
static float AS5600_ReadPhysicalDegAvg(AS5600_t *enc)
{
    float sum = 0.0f;
    int valid = 0;
    for (int i = 0; i < AS5600_AVG_SAMPLES; i++) {
        float val = AS5600_ReadPhysicalDeg(enc);
        if (val >= 0.0f) {
            sum += val;
            valid++;
        }
    }
    return (valid > 0) ? (sum / (float)valid) : -1.0f;
}

void BSP_AS5600_Init(void)
{
    Encoder_M1.hi2c = &hi2c1;
    Encoder_M1.zero_offset = 0.0f;
    Encoder_M1.angle_deg = 0.0f;

    Encoder_M2.hi2c = &hi2c2;
    Encoder_M2.zero_offset = 0.0f;
    Encoder_M2.angle_deg = 0.0f;

    Encoder_M3.hi2c = &hi2c3;
    Encoder_M3.zero_offset = 0.0f;
    Encoder_M3.angle_deg = 0.0f;
}

void BSP_AS5600_Update(AS5600_t *enc)
{
    if (enc == NULL || enc->hi2c == NULL) return;

    float physical_deg = AS5600_ReadPhysicalDegAvg(enc);
    if (physical_deg < 0.0f) return;

    /* 归一化到 (-180, 180], 避免 0/360 边界跳变 */
    float adjusted = physical_deg - enc->zero_offset;
    while (adjusted >  180.0f) adjusted -= 360.0f;
    while (adjusted <= -180.0f) adjusted += 360.0f;

    enc->angle_deg = adjusted;
}

void BSP_AS5600_SetZero(AS5600_t *enc)
{
    if (enc == NULL || enc->hi2c == NULL) return;

    float physical_deg = AS5600_ReadPhysicalDegAvg(enc);
    if (physical_deg < 0.0f) return;

    enc->zero_offset = physical_deg;
    enc->angle_deg = 0.0f;
}
