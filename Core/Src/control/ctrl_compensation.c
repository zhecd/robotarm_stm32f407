/**
 * @file    ctrl_compensation.c
 * @brief   Static position error compensation implementation. / 静态位置误差补偿实现。
 * @ingroup control
 *
 * Architecture guarantee / 架构保证:
 *   theory_steps accumulates only planner-pushed frames (not compensation
 *   frames), so it always represents the "commanded" position. This function
 *   takes a snapshot of theory_steps as a fixed target and drives the
 *   encoders toward it.
 *   theory_steps 仅累积规划器推送的帧(不含补偿帧), 因此始终代表"指令"位置。
 *   本函数将 theory_steps 的快照作为固定目标, 驱动编码器趋近该目标。
 */

#include "control/ctrl_compensation.h"
#include "control/ctrl_motion_engine.h"
#include "bsp/bsp_as5600.h"
#include "common.h"
#include <math.h>
#include <stdio.h>

void Ctrl_Compensation_Execute(void)
{
    AS5600_Dev_t *em1 = BSP_AS5600_GetM1();
    AS5600_Dev_t *em2 = BSP_AS5600_GetM2();
    AS5600_Dev_t *em3 = BSP_AS5600_GetM3();

    /* Wait for planner queue to drain while tracking multi-turn wraps.
       Sampling encoders during the wait prevents Nyquist violations
       from long motion gaps. / 等待运动队列清空, 同时跟踪多圈。
       期间持续采样编码器, 防止长运动间隙导致跨圈丢失。 */
    while (Ctrl_MotionEngine_IsRunning() || Ctrl_MotionEngine_GetQueueCount() > 0) {
        BSP_AS5600_Update(em1);
        BSP_AS5600_Update(em2);
        BSP_AS5600_Update(em3);
    }
    HAL_Delay(50);

    /* Snapshot theory step target / 快照理论步数目标 */
    int32_t tm1, tm2, tm3;
    Ctrl_MotionEngine_GetTheorySteps(&tm1, &tm2, &tm3);

    float tdeg1 = StepsToDeg(tm1);
    float tdeg2 = StepsToDeg(tm2);
    float tdeg3 = StepsToDeg(tm3);

    /* Read current encoder values / 读取当前编码器值 */
    BSP_AS5600_Update(em1);
    BSP_AS5600_Update(em2);
    BSP_AS5600_Update(em3);

    /* Persist stuck flags across calls; reset when theory target changes / 持久化卡死标记; 目标变化时重置 */
    static int32_t s_last_tm1 = -1, s_last_tm2 = -1, s_last_tm3 = -1;
    static bool    s_stuck1 = false, s_stuck2 = false, s_stuck3 = false;

    if (tm1 != s_last_tm1 || tm2 != s_last_tm2 || tm3 != s_last_tm3) {
        s_stuck1 = s_stuck2 = s_stuck3 = false;
        s_last_tm1 = tm1; s_last_tm2 = tm2; s_last_tm3 = tm3;
    }

    bool skip1 = s_stuck1, skip2 = s_stuck2, skip3 = s_stuck3;
    float prev1 = 1e9f, prev2 = 1e9f, prev3 = 1e9f;

    for (int iter = 0; ; iter++) {
        if (!skip1 && BSP_AS5600_Update(em1) != ERR_OK) { skip1 = s_stuck1 = true; }
        if (!skip2 && BSP_AS5600_Update(em2) != ERR_OK) { skip2 = s_stuck2 = true; }
        if (!skip3 && BSP_AS5600_Update(em3) != ERR_OK) { skip3 = s_stuck3 = true; }

        float e1 = AngleWrap180(tdeg1 - em1->angle_deg);
        float e2 = AngleWrap180(tdeg2 - em2->angle_deg);
        float e3 = AngleWrap180(tdeg3 - em3->angle_deg);

        float ae1 = fabsf(e1), ae2 = fabsf(e2), ae3 = fabsf(e3);

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

        /* Global divergence check / 全局发散检查 */
        float sum = (skip1 ? 0.0f : ae1) + (skip2 ? 0.0f : ae2) + (skip3 ? 0.0f : ae3);
        if (iter > 0 && sum == 0.0f) return;
        if (iter >= COMP_WATCHDOG_ROUNDS) return;

        int32_t cm1 = 0, cm2 = 0, cm3 = 0;
        if (!skip1 && ae1 > COMP_DEADBAND_DEG) cm1 = DegToSteps(e1);
        if (!skip2 && ae2 > COMP_DEADBAND_DEG) cm2 = DegToSteps(e2);
        if (!skip3 && ae3 > COMP_DEADBAND_DEG) cm3 = DegToSteps(e3);

        if (cm1 == 0 && cm2 == 0 && cm3 == 0) continue;

        MotionFrame_t cf = {
            .delta_m1    = cm1,
            .delta_m2    = cm2,
            .delta_m3    = cm3,
            .total_ticks = Common_MaxAbs3(cm1, cm2, cm3) * COMP_SPEED_DIV
        };
        if (cf.total_ticks < COMP_MIN_TICKS)
            cf.total_ticks = COMP_MIN_TICKS;

        Ctrl_MotionEngine_PushFrame(&cf);
        while (Ctrl_MotionEngine_IsRunning() || Ctrl_MotionEngine_GetQueueCount() > 0) {}
        HAL_Delay(30);
    }
}
