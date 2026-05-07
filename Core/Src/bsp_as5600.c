#include "bsp_as5600.h"
#include "i2c.h"
#include <stdio.h>
#include <stdlib.h>

AS5600_t Encoder_M1;
AS5600_t Encoder_M2;
AS5600_t Encoder_M3;

/* ── 内部函数 ── */

static float ReadRawDeg(AS5600_t *enc)
{
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(enc->hi2c, AS5600_ADDR_WRITE, AS5600_REG_RAW_ANGLE,
                          I2C_MEMADD_SIZE_8BIT, data, 2, 100) != HAL_OK)
        return -1.0f;
    uint16_t raw = (((uint16_t)data[0] << 8) | data[1]) & 0x0FFFU;
    return (float)raw * 360.0f / 4096.0f;
}

static float ReadAvgDeg(AS5600_t *enc)
{
    float sum = 0.0f;
    int valid = 0;
    for (int i = 0; i < AS5600_AVG_SAMPLES; i++) {
        float val = ReadRawDeg(enc);
        if (val >= 0.0f) { sum += val; valid++; }
    }
    return (valid > 0) ? (sum / (float)valid) : -1.0f;
}

/* ── 公共 API ── */

void BSP_AS5600_Init(void)
{
    AS5600_t *list[] = {&Encoder_M1, &Encoder_M2, &Encoder_M3};
    I2C_HandleTypeDef *i2c[] = {&hi2c1, &hi2c2, &hi2c3};
    for (int i = 0; i < 3; i++) {
        list[i]->hi2c = i2c[i];
        list[i]->zero_offset = 0.0f;
        list[i]->angle_deg = 0.0f;
    }
}

bool BSP_AS5600_Update(AS5600_t *enc)
{
    if (!enc || !enc->hi2c) return false;
    float phys = ReadAvgDeg(enc);
    if (phys < 0.0f) return false;
    enc->angle_deg = AngleWrap180(phys - enc->zero_offset);
    return true;
}

bool BSP_AS5600_SetZero(AS5600_t *enc)
{
    if (!enc || !enc->hi2c) return false;
    float phys = ReadAvgDeg(enc);
    if (phys < 0.0f) return false;
    enc->zero_offset = phys;
    enc->angle_deg = 0.0f;
    return true;
}

void BSP_AS5600_PrintStatus(void)
{
    AS5600_t *enc[3] = {&Encoder_M1, &Encoder_M2, &Encoder_M3};
    BSP_AS5600_Update(enc[0]);
    BSP_AS5600_Update(enc[1]);
    BSP_AS5600_Update(enc[2]);

    int d[3];
    for (int i = 0; i < 3; i++)
        d[i] = (int)(enc[i]->angle_deg * 10.0f);

    printf("ENC M1:%s%d.%d M2:%s%d.%d M3:%s%d.%d\r\n",
           d[0] < 0 ? "-" : "", abs(d[0]) / 10, abs(d[0]) % 10,
           d[1] < 0 ? "-" : "", abs(d[1]) / 10, abs(d[1]) % 10,
           d[2] < 0 ? "-" : "", abs(d[2]) / 10, abs(d[2]) % 10);
}

int32_t BSP_AS5600_GetSteps(AS5600_t *enc)
{
    return DegToSteps(enc->angle_deg);
}
