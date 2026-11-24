/* 仅替换CPU计数寄存器，测试编译真实platform_time.c。 */
#ifndef TEST_TIME_MAIN_H
#define TEST_TIME_MAIN_H
#include <stdint.h>
typedef struct { uint32_t DEMCR; } TestCoreDebug;
typedef struct { uint32_t CTRL, CYCCNT; } TestDwt;
extern TestCoreDebug test_debug;
extern TestDwt test_dwt;
extern uint32_t SystemCoreClock;
#define CoreDebug (&test_debug)
#define DWT (&test_dwt)
#define CoreDebug_DEMCR_TRCENA_Msk (1U<<24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
uint32_t HAL_GetTick(void);
#endif
