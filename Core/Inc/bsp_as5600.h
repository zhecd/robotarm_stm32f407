#ifndef __BSP_AS5600_H__
#define __BSP_AS5600_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_ADDR          0x36
#define AS5600_REG_RAW_ANGLE 0x0C

typedef enum {
    ENCODER_M1 = 0,
    ENCODER_M2 = 1,
    ENCODER_M3 = 2,
    ENCODER_COUNT = 3
} EncoderIndex_t;

void BSP_AS5600_Init(void);
bool BSP_AS5600_ReadAngle(EncoderIndex_t encoder, uint16_t *raw_angle);
uint16_t BSP_AS5600_GetAngle(EncoderIndex_t encoder);
void BSP_AS5600_CalibrateZero(void);

#ifdef __cplusplus
}
#endif

#endif
