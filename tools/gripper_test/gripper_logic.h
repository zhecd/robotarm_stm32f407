/** @file gripper_logic.h @brief 夹爪单次PWM窗口；纯C，不访问HAL，不表示实际到位。 */
#ifndef GRIPPER_TEST_LOGIC_H
#define GRIPPER_TEST_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#define GRIP_TEST_WINDOW_MS 600U
#define GRIP_TEST_LEASE_MS 250U
#define GRIP_TEST_MAIN_MS 200U
typedef enum { GRIP_IDLE=0,GRIP_ACTIVE,GRIP_DONE,GRIP_STOPPED,GRIP_FAILED } GripState;
typedef enum { GRIP_NONE=0,GRIP_OPEN,GRIP_CLOSE } GripTarget;
typedef enum { GRIP_OK=0,GRIP_HOST_STOP,GRIP_HOST_LOST,GRIP_MAIN_STALE,
    GRIP_LIMIT_CHANGED,GRIP_INPUT_FAULT,GRIP_PWM_ERROR,GRIP_SERIAL_ERROR } GripReason;
typedef struct {
    GripState state;GripTarget target;GripReason reason;
    bool used;uint8_t initial_limits;
    uint32_t started_ms,lease_ms;
} GripTest;
void GripTest_Init(GripTest *t);
/** 静态限位允许作为起始快照；与正常三轴App的安全许可完全独立。
 * 这里不控制关节，固件端必须永久禁用EN/STEP。窗口中任何限位变化均停止。 */
bool GripTest_Start(GripTest *t,uint32_t now,GripTarget target,bool compile_permit,
                    bool port_ready,uint8_t limits,bool input_config_ok);
bool GripTest_Keep(GripTest *t,uint32_t now);
void GripTest_Tick(GripTest *t,uint32_t now,uint32_t main_seen,uint8_t limits,bool config_ok);
void GripTest_Stop(GripTest *t,GripReason reason);
float GripTest_TargetAngle(GripTarget target);
uint32_t GripTest_TargetPulse(GripTarget target);
#endif
