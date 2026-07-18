#ifndef ROBOTARM_COMMAND_TYPES_H
#define ROBOTARM_COMMAND_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* Syntax-independent command representation passed from App to Service. */
typedef enum {
    GCMD_UNKNOWN = -1,
    GCMD_G0 = 0,
    GCMD_G1 = 1,
    GCMD_M3 = 3,
    GCMD_M5 = 5,
    GCMD_M114 = 114,
    GCMD_M119 = 119,
    GCMD_M400 = 400,
    GCMD_M999 = 999
} GCodeType_t;

typedef struct {
    GCodeType_t type;
    float x;
    float y;
    float z;
    uint32_t f;
    bool has_x;
    bool has_y;
    bool has_z;
    bool has_f;
} GCodeFrame_t;

#endif /* ROBOTARM_COMMAND_TYPES_H */
