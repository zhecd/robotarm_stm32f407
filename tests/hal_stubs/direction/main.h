/* GPIO链路测试只替换HAL端口，不替换Device和Stepper生产实现。 */
#ifndef TEST_DIRECTION_MAIN_H
#define TEST_DIRECTION_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct {uint32_t ODR;} GPIO_TypeDef;
typedef struct {unsigned unused;} UART_HandleTypeDef;
typedef struct {unsigned unused;} I2C_HandleTypeDef;
typedef struct {uint16_t Pin;uint32_t Mode,Pull,Speed;} GPIO_InitTypeDef;
typedef enum {GPIO_PIN_RESET=0,GPIO_PIN_SET=1} GPIO_PinState;
extern GPIO_TypeDef test_gpio_a,test_gpio_b,test_gpio_d;
#define M1_STEP_GPIO_Port (&test_gpio_a)
#define M2_STEP_GPIO_Port (&test_gpio_a)
#define M3_STEP_GPIO_Port (&test_gpio_b)
#define M1_DIR_GPIO_Port (&test_gpio_d)
#define M2_DIR_GPIO_Port (&test_gpio_d)
#define M3_DIR_GPIO_Port (&test_gpio_d)
#define EN_GPIO_Port (&test_gpio_d)
#define M1_STEP_Pin (1U<<6)
#define M2_STEP_Pin (1U<<7)
#define M3_STEP_Pin (1U<<0)
#define M1_DIR_Pin (1U<<3)
#define M2_DIR_Pin (1U<<4)
#define M3_DIR_Pin (1U<<5)
#define EN_Pin (1U<<2)
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_HIGH 2U
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __NOP() ((void)0)
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState state);
void HAL_GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *settings);
#endif
