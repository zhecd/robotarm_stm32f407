#ifndef TMC_LEGACY_BRIDGE_H
#define TMC_LEGACY_BRIDGE_H
#include <stdint.h>
#include <stdbool.h>
/* 只把备份驱动隔离到临时台架，正常业务和现有驱动不替换。 */
typedef struct {
    uint32_t initialized,tx_count,tx_fail,config_count;
    uint32_t high_seen,low_seen,en_low_seen,pulses;
    uint32_t regs[3],values[3];
} LegacyBenchReport;
extern volatile LegacyBenchReport g_legacy_report;
bool Legacy_Init(void);
bool Legacy_ConfigOriginal(void);
void Legacy_Enable(void);
void Legacy_Pulse(void);
uint32_t Bench_Milliseconds(void);
bool Bench_IsActive(void);
#endif
