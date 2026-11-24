#include "bench_logic.h"
bool Bench_Start(BenchGuard *g, uint32_t now)
{
    if (g->used) return false;
    g->used = true; g->active = true; g->started = now; return true;
}
bool Bench_Expired(const BenchGuard *g, uint32_t now)
{
    /* 无符号差支持毫秒计数回绕；运行时间固定为1秒。 */
    return g->active && (uint32_t)(now - g->started) >= BENCH_RUN_MS;
}
void Bench_Stop(BenchGuard *g) { g->active = false; }
bool Bench_StepLevel(const BenchGuard *g, uint32_t now)
{
    uint32_t elapsed=(uint32_t)(now-g->started);
    return g->active && elapsed<BENCH_RUN_MS && (elapsed&1U);
}
uint8_t Bench_Crc(const uint8_t *p, size_t n)
{
    uint8_t crc = 0;
    while (n--) {
        uint8_t b = *p++;
        for (unsigned bit = 0; bit < 8; ++bit) {
            bool mix = ((crc >> 7) ^ b) & 1U;
            crc = (uint8_t)(crc << 1);
            if (mix) crc ^= 7U;
            b >>= 1;
        }
    }
    return crc;
}
void Bench_WriteFrame(uint8_t node, uint8_t reg, uint32_t v, uint8_t f[8])
{
    f[0]=5U; f[1]=node; f[2]=reg|0x80U;
    f[3]=(uint8_t)(v>>24); f[4]=(uint8_t)(v>>16);
    f[5]=(uint8_t)(v>>8); f[6]=(uint8_t)v; f[7]=Bench_Crc(f,7);
}
