/**
 * @file    ctrl_compensation.c
 * @brief   Static position error compensation implementation. / 闈欐€佷綅缃宸ˉ鍋垮疄鐜般€? * @ingroup control
 *
 * Architecture guarantee / 鏋舵瀯淇濊瘉:
 *   theory_steps accumulates only planner-pushed frames (not compensation
 *   frames), so it always represents the "commanded" position. This function
 *   takes a snapshot of theory_steps as a fixed target and drives the
 *   encoders toward it.
 *   theory_steps 浠呯疮绉鍒掑櫒鎺ㄩ€佺殑�?涓嶅惈琛ュ伩�?, 鍥犳濮嬬粓浠ｈ�?鎸囦�?浣嶇疆銆? *   鏈嚱鏁板皢 theory_steps 鐨勫揩鐓т綔涓哄浐瀹氱洰鏍? 椹卞姩缂栫爜鍣ㄨ秼杩戣鐩爣銆? */

#include "ctrl_compensation.h"
#include "ctrl_motion_engine.h"
#include "ctrl_closed_loop.h"
#include "os/os_adapter.h"
#include "robot_math.h"
#include <math.h>
#include <stdio.h>

static bool WaitForMotionIdle(uint32_t timeout_ms)
{
    uint32_t start = Os_GetTickMs();
    while (Ctrl_MotionEngine_IsRunning() || Ctrl_MotionEngine_GetQueueCount() > 0U) {
        if (Ctrl_MotionEngine_HasFault() ||
            (Os_GetTickMs() - start) >= timeout_ms)
            return false;
    }
    return true;
}

void Ctrl_Compensation_Execute(void)
{
    /* Wait for planner queue to drain while tracking multi-turn wraps.
       Sampling encoders during the wait prevents Nyquist violations
       from long motion gaps. / 绛夊緟杩愬姩闃熷垪娓呯┖, 鍚屾椂璺熻釜澶氬湀�?       鏈熼棿鎸佺画閲囨牱缂栫爜�? 闃叉闀胯繍鍔ㄩ棿闅欏鑷磋法鍦堜涪澶便�?*/
    uint32_t wait_start = Os_GetTickMs();
    while (Ctrl_MotionEngine_IsRunning() || Ctrl_MotionEngine_GetQueueCount() > 0) {
        Ctrl_ClosedLoop_GetAxisAngle(0, &(float){0});
        Ctrl_ClosedLoop_GetAxisAngle(1, &(float){0});
        Ctrl_ClosedLoop_GetAxisAngle(2, &(float){0});
        if (Ctrl_MotionEngine_HasFault() ||
            (Os_GetTickMs() - wait_start) >= COMP_WAIT_TIMEOUT_MS)
            return;
    }
    Os_DelayMs(50U);

    /* Snapshot theory step target / 蹇収鐞嗚姝ユ暟鐩爣 */
    int32_t tm1, tm2, tm3;
    Ctrl_MotionEngine_GetTheorySteps(&tm1, &tm2, &tm3);

    float tdeg1 = StepsToDeg(tm1);
    float tdeg2 = StepsToDeg(tm2);
    float tdeg3 = StepsToDeg(tm3);

    /* Read current encoder values / 璇诲彇褰撳墠缂栫爜鍣ㄥ€?*/
    float e1, e2, e3;
    Ctrl_ClosedLoop_GetAxisAngle(0, &e1);
    Ctrl_ClosedLoop_GetAxisAngle(1, &e2);
    Ctrl_ClosedLoop_GetAxisAngle(2, &e3);

    /* Persist stuck flags across calls; reset when theory target changes / 鎸佷箙鍖栧崱姝绘爣璁? 鐩爣鍙樺寲鏃堕噸缃?*/
    static int32_t s_last_tm1 = -1, s_last_tm2 = -1, s_last_tm3 = -1;
    static bool    s_stuck1 = false, s_stuck2 = false, s_stuck3 = false;

    if (tm1 != s_last_tm1 || tm2 != s_last_tm2 || tm3 != s_last_tm3) {
        s_stuck1 = s_stuck2 = s_stuck3 = false;
        s_last_tm1 = tm1; s_last_tm2 = tm2; s_last_tm3 = tm3;
    }

    bool skip1 = s_stuck1, skip2 = s_stuck2, skip3 = s_stuck3;
    float prev1 = 1e9f, prev2 = 1e9f, prev3 = 1e9f;

    for (int iter = 0; ; iter++) {
        if (!skip1 && !Ctrl_ClosedLoop_GetAxisAngle(0, &e1)) { skip1 = s_stuck1 = true; }
        if (!skip2 && !Ctrl_ClosedLoop_GetAxisAngle(1, &e2)) { skip2 = s_stuck2 = true; }
        if (!skip3 && !Ctrl_ClosedLoop_GetAxisAngle(2, &e3)) { skip3 = s_stuck3 = true; }

        float err1 = AngleWrap180(tdeg1 - e1);
        float err2 = AngleWrap180(tdeg2 - e2);
        float err3 = AngleWrap180(tdeg3 - e3);

        float ae1 = fabsf(err1), ae2 = fabsf(err2), ae3 = fabsf(err3);

        bool ok1 = skip1 || ae1 <= COMP_DEADBAND_DEG;
        bool ok2 = skip2 || ae2 <= COMP_DEADBAND_DEG;
        bool ok3 = skip3 || ae3 <= COMP_DEADBAND_DEG;
        if (ok1 && ok2 && ok3) return;

        if (!skip1 && iter > 0 && ae1 >= prev1) { skip1 = s_stuck1 = true; }
        if (!skip2 && iter > 0 && ae2 >= prev2) { skip2 = s_stuck2 = true; }
        if (!skip3 && iter > 0 && ae3 >= prev3) { skip3 = s_stuck3 = true; }

        if ((tm1 == 0 || skip1) && (tm2 == 0 || skip2) && (tm3 == 0 || skip3))
            return;

        prev1 = ae1; prev2 = ae2; prev3 = ae3;

        /* Global divergence check / 鍏ㄥ眬鍙戞暎妫€�?*/
        float sum = (skip1 ? 0.0f : ae1) + (skip2 ? 0.0f : ae2) + (skip3 ? 0.0f : ae3);
        if (iter > 0 && sum == 0.0f) return;
        if (iter >= COMP_WATCHDOG_ROUNDS) return;

        int32_t cm1 = 0, cm2 = 0, cm3 = 0;
        if (!skip1 && ae1 > COMP_DEADBAND_DEG) cm1 = DegToSteps(err1);
        if (!skip2 && ae2 > COMP_DEADBAND_DEG) cm2 = DegToSteps(err2);
        if (!skip3 && ae3 > COMP_DEADBAND_DEG) cm3 = DegToSteps(err3);

        if (cm1 == 0 && cm2 == 0 && cm3 == 0) continue;

        MotionFrame_t cf = {
            .delta_m1    = cm1,
            .delta_m2    = cm2,
            .delta_m3    = cm3,
            .total_ticks = RobotMath_MaxAbs3(cm1, cm2, cm3) * COMP_SPEED_DIV
        };
        if (cf.total_ticks < COMP_MIN_TICKS)
            cf.total_ticks = COMP_MIN_TICKS;

        if (!Ctrl_MotionEngine_PushFrame(&cf)) return;
        if (!WaitForMotionIdle(COMP_WAIT_TIMEOUT_MS)) return;
        Os_DelayMs(30U);
    }
}
