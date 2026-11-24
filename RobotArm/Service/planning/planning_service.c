/**
 * @file planning_service.c
 * @brief 规划公开入口：增量校验路径并生成运动帧，不暴露私有规划器。
 */
#include "planning_service.h"

#include "ctrl_planner.h"

void PlanningService_Cancel(void) { Ctrl_Planner_Cancel(); }

ErrorCode_t PlanningService_Init(float start_x, float start_y, float start_z)
{
    return Ctrl_Planner_Init(start_x, start_y, start_z);
}

/** 输入目标 mm 和总时长 ms；非零路径先返回 ERR_PENDING，由 Service 推进。 */
ErrorCode_t PlanningService_StartLine(float target_x, float target_y,
                                      float target_z, uint32_t duration_ms)
{
    return Ctrl_Planner_MoveLine(target_x, target_y, target_z, duration_ms);
}

ErrorCode_t PlanningService_StartTeleopStep(float dx_mm, float dy_mm, float dz_mm)
{
    return Ctrl_Planner_TeleopStep(dx_mm, dy_mm, dz_mm);
}

void PlanningService_Service(void)
{
    Ctrl_Planner_Service();
}

bool PlanningService_IsBusy(void)
{
    return Ctrl_Planner_IsBusy();
}

bool PlanningService_TakeStartResult(ErrorCode_t *out_result)
{
    return Ctrl_Planner_TakeStartResult(out_result);
}
