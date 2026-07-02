/**
 * @file    bsp_as5600.c
 * @brief   AS5600 magnetic encoder driver (multi-turn tracking) / AS5600 磁编码器驱动 (多圈跟踪)
 * @ingroup bsp
 */

#include "bsp/bsp_as5600.h"
#include "i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static AS5600_Dev_t s_enc_m1;
static AS5600_Dev_t s_enc_m2;
static AS5600_Dev_t s_enc_m3;

static float ReadRawDeg(AS5600_Dev_t *enc)
{
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(enc->hi2c, AS5600_ADDR_WRITE, AS5600_REG_RAW_ANGLE,
                         I2C_MEMADD_SIZE_8BIT, data, 2, 100) != HAL_OK)
        return -1.0f;
    uint16_t raw = (((uint16_t)data[0] << 8) | data[1]) & 0x0FFFU;
    return (float)raw * 360.0f / 4096.0f;
}

static float ReadAvgDeg(AS5600_Dev_t *enc)
{
    float sum = 0.0f;
    int   valid = 0;
    for (int i = 0; i < AS5600_AVG_SAMPLES; i++) {
        float val = ReadRawDeg(enc);
        if (val >= 0.0f) { sum += val; valid++; }
    }
    return (valid > 0) ? (sum / (float)valid) : -1.0f;
}

/* ── Public API ── */

ErrorCode_t BSP_AS5600_Init(void)
{
    AS5600_Dev_t *list[] = {&s_enc_m1, &s_enc_m2, &s_enc_m3};
    I2C_HandleTypeDef *i2c[] = {&hi2c1, &hi2c2, &hi2c3};
    for (int i = 0; i < 3; i++) {
        list[i]->hi2c          = i2c[i];
        list[i]->zero_offset   = 0.0f;
        list[i]->angle_deg     = 0.0f;
        list[i]->raw_unwrapped = 0.0f;
        list[i]->turn_count    = 0;
    }
    return ERR_OK;
}

ErrorCode_t BSP_AS5600_Update(AS5600_Dev_t *enc)
{
    if (!enc || !enc->hi2c) return ERR_NULL_PARAM;

    float phys = ReadAvgDeg(enc);
    if (phys < 0.0f) return ERR_ENCODER_FAIL;

    /* Init on first call or after SetZero */
    if (enc->turn_count == 0 && enc->raw_unwrapped == 0.0f) {
        enc->raw_unwrapped = phys;
    } else {
        float prev = fmodf(enc->raw_unwrapped, 360.0f);
        if (prev < 0.0f) prev += 360.0f;
        float delta = phys - prev;
        if (delta > 180.0f) {
            enc->turn_count--;                    /* wrapped backward (360→0) */
        } else if (delta < -180.0f) {
            enc->turn_count++;                    /* wrapped forward (0→360) */
        }
        enc->raw_unwrapped = (float)enc->turn_count * 360.0f + phys;
    }

    enc->angle_deg = enc->raw_unwrapped - enc->zero_offset;
    return ERR_OK;
}

void BSP_AS5600_SyncTurn(AS5600_Dev_t *enc, float expected_deg)
{
    if (!enc) return;

    int32_t turns_off = (int32_t)roundf((expected_deg - enc->angle_deg) / 360.0f);
    if (turns_off == 0) return;

    float phys = enc->raw_unwrapped - (float)enc->turn_count * 360.0f;
    enc->turn_count    += turns_off;
    enc->raw_unwrapped  = (float)enc->turn_count * 360.0f + phys;
    enc->angle_deg      = enc->raw_unwrapped - enc->zero_offset;
}

ErrorCode_t BSP_AS5600_SetZero(AS5600_Dev_t *enc)
{
    if (!enc || !enc->hi2c) return ERR_NULL_PARAM;

    float phys = ReadAvgDeg(enc);
    if (phys < 0.0f) return ERR_ENCODER_FAIL;

    enc->zero_offset   = phys;
    enc->angle_deg     = 0.0f;
    enc->raw_unwrapped = phys;
    enc->turn_count    = 0;
    return ERR_OK;
}

void BSP_AS5600_PrintStatus(void)
{
    AS5600_Dev_t *enc[3] = {&s_enc_m1, &s_enc_m2, &s_enc_m3};
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

int32_t BSP_AS5600_GetSteps(AS5600_Dev_t *enc)
{
    return DegToSteps(enc->angle_deg);
}

AS5600_Dev_t *BSP_AS5600_GetM1(void) { return &s_enc_m1; }
AS5600_Dev_t *BSP_AS5600_GetM2(void) { return &s_enc_m2; }
AS5600_Dev_t *BSP_AS5600_GetM3(void) { return &s_enc_m3; }
