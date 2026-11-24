/**
 * @file closed_loop_status.h
 * @brief 闭环只读合同：最近补偿判定和已受理补偿量，角度均为电机轴度。
 */
#ifndef ROBOTARM_CLOSED_LOOP_STATUS_H
#define ROBOTARM_CLOSED_LOOP_STATUS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool target_initialized;      /**< Init零目标或Sync目标已建立且Motion未故障；仍须查Safety参考。 */
    bool active;                  /**< 补偿轨迹尚未完成；故障后立即为false。 */
    uint8_t enabled_mask;          /**< bit0/1/2分别为M1/M2/M3的闭环使能配置。 */
    uint8_t decision_mask;         /**< 最近误差来自有效滤波判定；年龄另查，非实时到位标志。 */
    uint32_t decision_ms[3];       /**< 判定取样开始时间；未判定时须忽略。 */
    float target_motor_deg[3];     /**< 当前命令目标；未初始化或参考撤销后不能用于定位。 */
    float error_motor_deg[3];      /**< 最近判定：目标减EMA滤波反馈；不是M409原始反馈差。 */
    uint32_t accepted_rounds[3];   /**< Init后各轴已受理的非零补偿轮数，饱和不回绕。 */
    int32_t last_steps[3];         /**< 最近受理的整轮有符号微步；不是已发STEP或到位证明。 */
} ClosedLoopStatus_t;

#endif
