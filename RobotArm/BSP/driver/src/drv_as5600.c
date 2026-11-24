/**
 * @file drv_as5600.c
 * @brief AS5600 驱动：I2C 读取、圆周均值、软件多圈跟踪与置零。
 * @details 软件多圈依赖连续采样且相邻样本转角小于 180 deg；断电后必须重新回零建立参考。
 */

#include "driver/drv_as5600.h"
#include "i2c.h"
#include "bsp/bsp_i2c_recovery.h"
#include "platform_time.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static As5600Device_t s_enc_m1;
static As5600Device_t s_enc_m2;
static As5600Device_t s_enc_m3;

static float ReadRawDeg(As5600Device_t *enc)
{
    uint8_t data[2];
    HAL_StatusTypeDef status=HAL_I2C_Mem_Read(enc->hi2c,AS5600_ADDR_WRITE,AS5600_REG_RAW_ANGLE,
                                            I2C_MEMADD_SIZE_8BIT,data,2,AS5600_I2C_TIMEOUT_MS);
    if (status!=HAL_OK) {
        uint32_t error=enc->hi2c->ErrorCode;
        enc->last_hal_error=error;
        enc->last_hal_status=(uint32_t)status;
        uint32_t now=PlatformTime_NowMs();
        bool due=!enc->recovery_attempted || (uint32_t)(now-enc->last_recovery_ms)>=AS5600_RECOVERY_INTERVAL_MS;
        bool available=enc->hi2c->State==HAL_I2C_STATE_READY && enc->hi2c->Lock==HAL_UNLOCKED;
        /* F4 HAL等BUSY标志超时会返回HAL_BUSY，但ErrorCode含TIMEOUT且句柄已解锁。
         * 因此不能仅按HAL_BUSY返回值判断“有人正在使用总线”。 */
        bool recoverable=available && (status==HAL_BUSY ? (error&HAL_I2C_ERROR_TIMEOUT)!=0U :
            ((error&(HAL_I2C_ERROR_TIMEOUT|HAL_I2C_ERROR_BERR|HAL_I2C_ERROR_ARLO))!=0U || status==HAL_TIMEOUT));
        /* NACK不触发GPIO时钟；State/Lock表明有真实传输时也不得强行重置。 */
        if (!recoverable || !due) return -1.0f;
        enc->recovery_attempted=true;enc->last_recovery_ms=now;++enc->recovery_attempts;
        if (!BSP_I2C_RecoverBus(enc->hi2c)) return -1.0f;
        status=HAL_I2C_Mem_Read(enc->hi2c,AS5600_ADDR_WRITE,AS5600_REG_RAW_ANGLE,
                              I2C_MEMADD_SIZE_8BIT,data,2,AS5600_I2C_TIMEOUT_MS);
        if (status!=HAL_OK) {
            enc->last_hal_error=enc->hi2c->ErrorCode;
            enc->last_hal_status=(uint32_t)status;
            return -1.0f;
        }
        ++enc->recovery_successes;
    }
    uint16_t raw = (((uint16_t)data[0] << 8) | data[1]) & 0x0FFFU;
    return (float)raw * 360.0f / 4096.0f;
}

static float ReadAvgDeg(As5600Device_t *enc)
{
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    int   valid = 0;
    for (int i = 0; i < AS5600_AVG_SAMPLES; i++) {
        float val = ReadRawDeg(enc);
        if (val < 0.0f) return -1.0f;
        float rad = val * M_PI / 180.0f;
        sum_sin += sinf(rad);
        sum_cos += cosf(rad);
        valid++;
    }
    if (valid == 0) return -1.0f;
    float avg = atan2f(sum_sin, sum_cos) * 180.0f / M_PI;
    return (avg < 0.0f) ? (avg + 360.0f) : avg;
}

/* 公开接口。 */

ErrorCode_t Drv_AS5600_Init(void)
{
    As5600Device_t *list[] = {&s_enc_m1, &s_enc_m2, &s_enc_m3};
    I2C_HandleTypeDef *i2c[] = {&hi2c1, &hi2c2, &hi2c3};
    for (int i = 0; i < 3; i++) {
        *list[i]=(As5600Device_t){.hi2c=i2c[i]};
    }
    return ERR_OK;
}

/** 完成一次圆周均值采样和软件解圈；通信失败不得当作零角度使用。 */
ErrorCode_t Drv_AS5600_Update(As5600Device_t *enc)
{
    if (!enc || !enc->hi2c) return ERR_NULL_PARAM;

    uint32_t now=PlatformTime_NowMs();
    if (enc->retry_wait && (uint32_t)(now-enc->last_failure_ms)<AS5600_FAILURE_BACKOFF_MS)
        return ERR_ENCODER_FAIL;

    float phys = ReadAvgDeg(enc);
    if (phys < 0.0f) {
        enc->retry_wait=true;enc->last_failure_ms=PlatformTime_NowMs();
        return ERR_ENCODER_FAIL;
    }
    enc->retry_wait=false;

    /* Init on first call or after SetZero */
    if (!enc->initialized) {
        enc->raw_unwrapped = phys;
        enc->initialized = true;
    } else {
        float prev = fmodf(enc->raw_unwrapped, 360.0f);
        if (prev < 0.0f) prev += 360.0f;
        float delta = phys - prev;
        if (delta > 180.0f) {
            enc->turn_count--;                    /* 原始角度向反方向跨过一圈。 */
        } else if (delta < -180.0f) {
            enc->turn_count++;                    /* 原始角度向正方向跨过一圈。 */
        }
        enc->raw_unwrapped = (float)enc->turn_count * 360.0f + phys;
    }

    enc->angle_deg = enc->raw_unwrapped - enc->zero_offset;
    return ERR_OK;
}

void Drv_AS5600_SyncTurn(As5600Device_t *enc, float expected_deg)
{
    if (!enc) return;

    int32_t turns_off = (int32_t)roundf((expected_deg - enc->angle_deg) / 360.0f);
    if (turns_off == 0) return;

    float phys = enc->raw_unwrapped - (float)enc->turn_count * 360.0f;
    enc->turn_count    += turns_off;
    enc->raw_unwrapped  = (float)enc->turn_count * 360.0f + phys;
    enc->angle_deg      = enc->raw_unwrapped - enc->zero_offset;
}

/** 以当前电机姿态建立编码器零偏，不修改机构几何零点定义。 */
ErrorCode_t Drv_AS5600_SetZero(As5600Device_t *enc)
{
    if (!enc || !enc->hi2c) return ERR_NULL_PARAM;

    float phys = ReadAvgDeg(enc);
    if (phys < 0.0f) return ERR_ENCODER_FAIL;

    enc->zero_offset   = phys;
    enc->angle_deg     = 0.0f;
    enc->raw_unwrapped = phys;
    enc->turn_count    = 0;
    enc->initialized   = true;
    return ERR_OK;
}

void Drv_AS5600_PrintStatus(void)
{
    As5600Device_t *enc[3] = {&s_enc_m1, &s_enc_m2, &s_enc_m3};
    Drv_AS5600_Update(enc[0]);
    Drv_AS5600_Update(enc[1]);
    Drv_AS5600_Update(enc[2]);

    int d[3];
    for (int i = 0; i < 3; i++)
        d[i] = (int)(enc[i]->angle_deg * 10.0f);

    printf("ENC M1:%s%d.%d M2:%s%d.%d M3:%s%d.%d\r\n",
           d[0] < 0 ? "-" : "", abs(d[0]) / 10, abs(d[0]) % 10,
           d[1] < 0 ? "-" : "", abs(d[1]) / 10, abs(d[1]) % 10,
           d[2] < 0 ? "-" : "", abs(d[2]) / 10, abs(d[2]) % 10);
}

int32_t Drv_AS5600_GetSteps(As5600Device_t *enc)
{
    return DegToSteps(enc->angle_deg);
}

As5600Device_t *Drv_AS5600_GetM1(void) { return &s_enc_m1; }
As5600Device_t *Drv_AS5600_GetM2(void) { return &s_enc_m2; }
As5600Device_t *Drv_AS5600_GetM3(void) { return &s_enc_m3; }
