#ifndef ROBOTARM_COMMAND_SERVICE_H
#define ROBOTARM_COMMAND_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "command_types.h"
#include "common/error_code.h"

typedef enum {
    COMMAND_SOURCE_GCODE = 0,
    COMMAND_SOURCE_PS2
} CommandSource_t;

typedef struct {
    uint32_t generation;
    CommandSource_t active_source;
    float planned_x_mm;
    float planned_y_mm;
    float planned_z_mm;
    float feedrate_mm_min;
} CommandServiceStatus_t;

ErrorCode_t CommandService_Init(float start_x, float start_y, float start_z);
ErrorCode_t CommandService_RunGCode(const GCodeFrame_t *frame);
ErrorCode_t CommandService_RunTeleopStep(float dx_mm, float dy_mm, float dz_mm);
void CommandService_Service(void);
bool CommandService_TakeMoveResult(ErrorCode_t *out_result);
bool CommandService_TakeQueuedMoveResult(ErrorCode_t *out_result);
bool CommandService_IsMotionPending(void);
uint16_t CommandService_GetQueuedMoveCount(void);
void CommandService_GetStatus(CommandServiceStatus_t *out_status);

#endif
