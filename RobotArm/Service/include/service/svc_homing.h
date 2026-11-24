/**
 * @file svc_homing.h
 * @brief 回零服务：寻找限位、逐轴停步、退回与失败处理的非阻塞状态机。
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

/** 启动寻限位；重复启动活动回零返回 false，不在函数内等待到位。 */
bool Svc_Homing_Start(void);
/** 主循环周期调用；时间未到就返回，每次最多给每个待运动轴一个 STEP。 */
void Svc_Homing_Step(void);
/** 取消回零；之后 Step 不再输出脉冲，重新开始必须显式调用 Start。 */
void Svc_Homing_Cancel(void);
SvcHomingState_t Svc_Homing_GetState(void);
bool Svc_Homing_IsFinished(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HOMING_H__ */
