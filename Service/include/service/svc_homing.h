/**
 * @file    svc_homing.h
 * @brief   Non-blocking limit-switch homing state machine.
 * @ingroup service
 */

#ifndef __BSP_HOMING_H__
#define __BSP_HOMING_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SVC_HOMING_IDLE = 0,
    SVC_HOMING_SEEK,
    SVC_HOMING_BACKOFF,
    SVC_HOMING_COMPLETE,
    SVC_HOMING_FAILED
} SvcHomingState_t;

bool Svc_Homing_Start(void);
void Svc_Homing_Step(void);
SvcHomingState_t Svc_Homing_GetState(void);
bool Svc_Homing_IsFinished(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HOMING_H__ */
