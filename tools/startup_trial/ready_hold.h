/** @file ready_hold.h
 * @brief READY保持验收的有界纯状态机，不操作执行器。
 * @details 只供显式试验构建使用；普通生产固件保持原使能策略。
 */
#ifndef ROBOTARM_READY_HOLD_H
#define ROBOTARM_READY_HOLD_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { READY_HOLD_IDLE,READY_HOLD_RUNNING,READY_HOLD_DONE,READY_HOLD_FAILED } ReadyHoldState;
typedef struct {ReadyHoldState state;uint32_t began_ms,duration_ms;} ReadyHold;
bool ReadyHold_Arm(ReadyHold *s,uint32_t now,uint32_t duration_ms);
ReadyHoldState ReadyHold_Tick(ReadyHold *s,uint32_t now,bool homed,bool allowed,bool fault);
#endif
