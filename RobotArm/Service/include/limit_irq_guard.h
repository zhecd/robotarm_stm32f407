/**
 * @file limit_irq_guard.h
 * @brief 正常/波形入口的限位IRQ过载锁存；不取代原有边沿停止和GPIO消抖。
 */
#ifndef ROBOTARM_LIMIT_IRQ_GUARD_H
#define ROBOTARM_LIMIT_IRQ_GUARD_H
#include <stdbool.h>
#include <stdint.h>
#define LIMIT_IRQ_GUARD_WINDOW_MS 10U
#define LIMIT_IRQ_GUARD_BURST_LIMIT 256U
/** EXTI传逻辑轴0..2；同轴窗口达到阈值或已锁存时返回true，请求硬件屏蔽/停止。
 * 零初始化即可工作，包括App初始化之前；只有MCU重新启动才清锁存，无在线复位接口。
 */
bool LimitIrqGuard_OnEdgeFromISR(uint8_t axis);
uint8_t LimitIrqGuard_GetLatchMask(void);
#endif
