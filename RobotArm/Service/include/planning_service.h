#ifndef ROBOTARM_PLANNING_SERVICE_H
#define ROBOTARM_PLANNING_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "common/error_code.h"

/* Public facade for Cartesian path planning.  Callers must not access the
 * planning implementation in Service/planning/internal directly. */
ErrorCode_t PlanningService_Init(float start_x, float start_y, float start_z);
ErrorCode_t PlanningService_StartLine(float target_x, float target_y,
                                      float target_z, uint32_t duration_ms);
ErrorCode_t PlanningService_StartTeleopStep(float dx_mm, float dy_mm, float dz_mm);
void PlanningService_Service(void);
bool PlanningService_IsBusy(void);
bool PlanningService_TakeStartResult(ErrorCode_t *out_result);

#endif /* ROBOTARM_PLANNING_SERVICE_H */
