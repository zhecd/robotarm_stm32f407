/* 只替换寄存器，编译真实BSP记录器；不模拟真实异步边沿或W1C。 */
#ifndef TEST_LIMIT_TRACE_MAIN_H
#define TEST_LIMIT_TRACE_MAIN_H
#include <stdint.h>
#define M1_STOP_Pin (1U<<10)
#define M2_STOP_Pin (1U<<11)
#define M3_STOP_Pin (1U<<12)
#define __DMB() ((void)0)
typedef struct { uint32_t PR,IMR,RTSR,FTSR,SWIER; } TestExti;
typedef struct { uint32_t IDR; } TestGpio;
typedef struct { uint32_t CYCCNT; } TestDwt;
typedef struct { uint32_t EXTICR[4]; } TestSyscfg;
typedef struct { uint32_t ICSR; } TestScb;
extern TestExti test_exti;
extern TestGpio test_gpio;
extern TestDwt test_dwt;
extern TestSyscfg test_syscfg;
extern TestScb test_scb;
#define EXTI (&test_exti)
#define GPIOC (&test_gpio)
#define DWT (&test_dwt)
#define SYSCFG (&test_syscfg)
#define SCB (&test_scb)
uint32_t HAL_GetTick(void);
#endif
