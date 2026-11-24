/** @file m1_limits.h @brief 限位电平解码、LED映射与采样历史；无HAL依赖。 */
#ifndef M1_LIMITS_H
#define M1_LIMITS_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    uint32_t idr,moder,pupdr;
    uint8_t active_mask; /* bit0/1/2分别为PC10/11/12低电平，不表示已经稳定10ms。 */
    bool config_ok;
} M1LimitSnapshot;
typedef struct {
    uint32_t samples,low_samples[3],changes[3];
    uint8_t low_seen_mask,last_mask;
    bool initialized;
} M1LimitTrace;
M1LimitSnapshot M1Limits_Decode(uint32_t idr,uint32_t moder,uint32_t pupdr);
/** 仅控制PC1/2/3（板上LED2/3/4），低有效；绝不触碰PC0或限位/串口引脚。 */
uint32_t M1Limits_LedBsrr(uint8_t active_mask);
/** 每个SysTick使用与HOME相同的那一份快照；计数饱和，不清除历史，不代替消抖。 */
void M1Limits_Observe(volatile M1LimitTrace *trace,uint8_t active_mask);
#endif
