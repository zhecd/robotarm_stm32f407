/**
 * @file offline_plan_check.c
 * @brief 仅电脑构建的分拣预演：真实解析/命令/规划/STEP引擎，设备与时钟使用测试替身。
 * @details 不链接HAL/串口。合成参考只存在于本进程，不调用正常App初始化或真实回零。
 * 输入参数为起点XYZ(mm)，stdin每行是command<TAB>G-code或wait_seconds<TAB>秒。
 * 成功表示本模型按当前配置能完成，不表示避障、闭环响应或真实机构验收通过。
 */
#include "app/app_gcode_parser.h"
#include "command_service.h"
#include "motion_service.h"
#include "safety_service.h"
#include "state_service.h"
#include "service/svc_gripper.h"
#include "platform_time.h"
#include "robot_config.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MODEL_MAX_MS 120000U
#define MODEL_MAX_ITEMS 256U
_Static_assert(TICKS_PER_MS == 50U, "Update model clock when STEP timer frequency changes");

static unsigned s_item;
static unsigned s_completed;
static bool Number(const char *text, double *out)
{
    char *end;
    errno = 0;
    *out = strtod(text, &end);
    return end != text && *end == '\0' && errno != ERANGE && isfinite(*out);
}

static int Finish(bool success, const char *reason)
{
    CommandServiceStatus_t status;
    CommandService_GetStatus(&status);
    if (!success) {
        MotionService_AbortForFault(MOTION_FAULT_HOST_STOP);
        CommandService_AbortPending();
    }
    printf("{\"simulation_only\":true,\"hardware_connected\":false,\"synthetic_reference\":true,"
           "\"pass\":%s,\"reason\":\"%s\",\"failed_item\":%u,\"completed_items\":%u,"
           "\"simulated_ms\":%lu,\"fake_steps\":[%lu,%lu,%lu],"
           "\"fake_gripper_open\":%u,\"fake_gripper_close\":%u,"
           "\"position_mm\":[%.6f,%.6f,%.6f]}\n",
           success ? "true" : "false", reason, success ? 0U : s_item, s_completed,
           (unsigned long)PlatformTime_NowMs(), (unsigned long)fake_steps[0],
           (unsigned long)fake_steps[1], (unsigned long)fake_steps[2],
           fake_gripper_open_count, fake_gripper_close_count,
           status.planned_x_mm, status.planned_y_mm, status.planned_z_mm);
    return success ? 0 : 2;
}

static const char *RunMove(const GCodeFrame_t *frame)
{
    ErrorCode_t result = CommandService_RunGCode(frame);
    if (result != ERR_PENDING && result != ERR_OK) return "MOVE_REJECTED";
    bool validated = result == ERR_OK;
    /* 每2ms推进一次主循环，逐个调用真实STEP调度器；时间不依赖电脑运行速度。 */
    while (CommandService_IsMotionPending() || !MotionService_IsIdle()) {
        CommandService_Service();
        if (CommandService_TakeMoveResult(&result)) {
            if (result != ERR_OK) return "PATH_REJECTED";
            validated = true;
        }
        for (unsigned tick = 0; tick < 100U; ++tick) {
            if (PlatformTime_NowMs() >= MODEL_MAX_MS) return "MODEL_DEADLINE";
            MotionService_OnStepTickFromISR();
            FakeRobot_AdvanceUs(20U);
        }
        Svc_Gripper_IdleStop();
        if (MotionService_HasFault()) return "MOTION_FAULT";
    }
    return validated ? NULL : "MISSING_MOVE_RESULT";
}

int main(int argc, char **argv)
{
    FakeRobot_Reset(); FakeHardware_Reset();
    MotionService_Init(); StateService_Init(); SafetyService_Init();
    MotionService_InitClosedLoop();
    double start[3];
    if (argc != 4) return Finish(false, "START_XYZ_REQUIRED");
    for (unsigned axis=0; axis<3U; ++axis)
        if (!Number(argv[axis+1], &start[axis]) || fabs(start[axis]) > 1e6)
            return Finish(false, "INVALID_START");
    if (CommandService_Init((float)start[0],(float)start[1],(float)start[2]) != ERR_OK)
        return Finish(false, "START_UNREACHABLE");
    /* 仅测试替身建立合成许可。没有真实回零/标定，也不会保存到板卡。 */
    SafetyService_MarkHomed(); SafetyService_ClearAfterSuccessfulHoming();
    enum { EXPECT_COMMAND, EXPECT_M400, EXPECT_WAIT } next = EXPECT_COMMAND;
    char line[512];
    while (fgets(line, sizeof(line), stdin)) {
        ++s_item;
        if (s_item > MODEL_MAX_ITEMS) return Finish(false, "TOO_MANY_ITEMS");
        size_t length = strlen(line);
        if (!length || line[length-1] != '\n') return Finish(false, "INCOMPLETE_OR_LONG_LINE");
        line[--length] = '\0';
        if (length && line[length-1] == '\r') line[--length]='\0';
        char *value = strchr(line, '\t');
        if (!value) return Finish(false, "INVALID_ITEM");
        *value++ = '\0';
        if (strcmp(line, "wait_seconds") == 0) {
            double seconds;
            if (next != EXPECT_WAIT || !Number(value, &seconds) || seconds < 0.6 || seconds > 60.0)
                return Finish(false, "INVALID_GRIPPER_WAIT");
            uint32_t us=(uint32_t)ceil(seconds * 1000000.0);
            if ((double)PlatformTime_NowMs() + us/1000.0 >= MODEL_MAX_MS)
                return Finish(false, "MODEL_DEADLINE");
            FakeRobot_AdvanceUs(us); Svc_Gripper_IdleStop(); next=EXPECT_COMMAND;
        } else if (strcmp(line, "command") == 0) {
            GCodeFrame_t frame;
            if (strlen(value)>255U || !App_GCodeParser_ParseLine(value, &frame))
                return Finish(false, "INVALID_GCODE");
            if (next == EXPECT_WAIT) return Finish(false, "MISSING_GRIPPER_WAIT");
            if (next == EXPECT_M400) {
                if (frame.type != GCMD_M400) return Finish(false, "MISSING_M400");
                next=EXPECT_COMMAND;
            } else if (frame.type == GCMD_G1 && (frame.has_x || frame.has_y || frame.has_z)) {
                const char *error = RunMove(&frame);
                if (error) return Finish(false, error);
                next=EXPECT_M400;
            } else if (frame.type == GCMD_M3 || frame.type == GCMD_M5) {
                if (CommandService_RunGCode(&frame) != ERR_OK) return Finish(false, "GRIPPER_REJECTED");
                next=EXPECT_WAIT;
            } else return Finish(false, "COMMAND_NOT_IN_PREVIEW_CONTRACT");
        } else return Finish(false, "INVALID_ITEM_KIND");
        ++s_completed;
    }
    if (ferror(stdin)) return Finish(false, "INPUT_ERROR");
    if (!s_item || next != EXPECT_COMMAND) return Finish(false, "INCOMPLETE_PLAN");
    return Finish(true, "MODEL_COMPLETED");
}
