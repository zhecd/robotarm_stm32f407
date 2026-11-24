/** 舵机驱动电脑测试端口；只记录CCR/启动/停止，不产生实际PWM。 */
#ifndef TEST_SERVO_MAIN_H
#define TEST_SERVO_MAIN_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { uint32_t compare; } TIM_HandleTypeDef;
typedef enum { HAL_OK=0,HAL_ERROR=1 } HAL_StatusTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *,uint32_t);
void Test_SetCompare(TIM_HandleTypeDef *,uint32_t,uint32_t);
#define __HAL_TIM_SET_COMPARE(h,c,v) Test_SetCompare((h),(c),(v))
#endif
