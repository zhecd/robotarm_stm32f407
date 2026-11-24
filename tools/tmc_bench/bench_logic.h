#ifndef TMC_BENCH_LOGIC_H
#define TMC_BENCH_LOGIC_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
/* 专用台架：每次启动仅一次动作，调用方负责独立硬件EN截止。 */
#define BENCH_RUN_MS 1000U
#define BENCH_VELOCITY 6000U
typedef struct { volatile bool used; volatile bool active; volatile uint32_t started; } BenchGuard;
bool Bench_Start(BenchGuard *guard, uint32_t now);
bool Bench_Expired(const BenchGuard *guard, uint32_t now);
/* STEP对照：1ms高/1ms低，1000ms截止前恰好500个上升沿。 */
bool Bench_StepLevel(const BenchGuard *guard, uint32_t now);
void Bench_Stop(BenchGuard *guard);
uint8_t Bench_Crc(const uint8_t *bytes, size_t count);
void Bench_WriteFrame(uint8_t node, uint8_t reg, uint32_t value, uint8_t frame[8]);
#endif
