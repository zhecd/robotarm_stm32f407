/** @file limit_state_filter.h
 * @brief 普通GPIO的非阻塞消抖；不依赖HAL，便于单独测试。
 * 第一份低电平样本即要求停步，连续低10ms才确认按下；连续高10ms取消短脉冲。
 * 单次确认100ms仍不稳定则锁存故障。已确认按住没有“按住超时”。
 * 采样间隔超过1ms时重新累计稳定时间，不能把未观察的空档算作稳定。
 */
#ifndef ROBOTARM_LIMIT_STATE_FILTER_H
#define ROBOTARM_LIMIT_STATE_FILTER_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t began[3],stable_since[3],last_ms;
    uint8_t pending,level_low,pressed,unstable;
    bool sampled;
} LimitStateFilter;
void LimitStateFilter_Reset(LimitStateFilter *s);
void LimitStateFilter_Sample(LimitStateFilter *s,uint32_t now,uint8_t low);
bool LimitStateFilter_BlocksSteps(const LimitStateFilter *s);
#endif
