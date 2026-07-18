#include "app/app_gcode_exec.h"

#include "command_service.h"

ErrorCode_t App_GCodeExec_Init(float start_x, float start_y, float start_z)
{
    return CommandService_Init(start_x, start_y, start_z);
}

void App_GCodeExec_GetPlannedPosition(float *x, float *y, float *z)
{
    CommandServiceStatus_t status;
    CommandService_GetStatus(&status);
    if (x) *x = status.planned_x_mm;
    if (y) *y = status.planned_y_mm;
    if (z) *z = status.planned_z_mm;
}

ErrorCode_t App_GCodeExec_Run(const GCodeFrame_t *frame)
{
    return CommandService_RunGCode(frame);
}

void App_GCodeExec_Service(void)
{
    CommandService_Service();
}

bool App_GCodeExec_TakeMoveResult(ErrorCode_t *out_result)
{
    return CommandService_TakeMoveResult(out_result);
}

bool App_GCodeExec_TakeQueuedMoveResult(ErrorCode_t *out_result)
{
    return CommandService_TakeQueuedMoveResult(out_result);
}

bool App_GCodeExec_IsMotionPending(void)
{
    return CommandService_IsMotionPending();
}
