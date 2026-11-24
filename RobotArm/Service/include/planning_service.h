/**
 * @file planning_service.h
 * @brief 规划公开入口：增量校验路径并生成运动帧，不暴露私有规划器。
 */
#ifndef ROBOTARM_PLANNING_SERVICE_H
#define ROBOTARM_PLANNING_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "common/error_code.h"

/* Public facade for Cartesian path planning.  Callers must not access the
 * planning implementation in Service/planning/internal directly. */
ErrorCode_t PlanningService_Init(float start_x, float start_y, float start_z);
/** 输入目标 mm 和总时长 ms；非零路径先返回 ERR_PENDING，由 Service 推进。 */
ErrorCode_t PlanningService_StartLine(float target_x, float target_y,
                                      float target_z, uint32_t duration_ms);
ErrorCode_t PlanningService_StartTeleopStep(float dx_mm, float dy_mm, float dz_mm);
void PlanningService_Service(void);
/** 故障/上位机停止时撤销校验与帧生成；不代表机械臂已回零。 */
void PlanningService_Cancel(void);
bool PlanningService_IsBusy(void);
bool PlanningService_TakeStartResult(ErrorCode_t *out_result);

#endif /* ROBOTARM_PLANNING_SERVICE_H */
