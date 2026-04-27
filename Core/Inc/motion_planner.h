#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void Motion_Planner_Init(float start_x, float start_y, float start_z);
bool Motion_Planner_MoveLine(float target_x, float target_y, float target_z, uint32_t duration_ms);
bool Motion_Planner_TeleopStep(float dx, float dy, float dz);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_PLANNER_H */
