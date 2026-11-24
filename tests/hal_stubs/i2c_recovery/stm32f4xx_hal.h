#ifndef TEST_I2C_RECOVERY_HAL_H
#define TEST_I2C_RECOVERY_HAL_H
#include <stdint.h>
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef enum {GPIO_PIN_RESET,GPIO_PIN_SET} GPIO_PinState;
typedef struct { unsigned index;uint16_t output; } GPIO_TypeDef;
extern GPIO_TypeDef test_gpio_a,test_gpio_b,test_gpio_c;
#define GPIOA (&test_gpio_a)
#define GPIOB (&test_gpio_b)
#define GPIOC (&test_gpio_c)
#define GPIO_PIN_8 (1U<<8)
#define GPIO_PIN_9 (1U<<9)
#define GPIO_PIN_10 (1U<<10)
#define GPIO_PIN_11 (1U<<11)
#define GPIO_MODE_OUTPUT_OD 0x11U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_HIGH 3U
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
typedef struct {uint32_t State,Lock,ErrorCode;} I2C_HandleTypeDef;
#define HAL_UNLOCKED 0U
#define HAL_LOCKED 1U
#define HAL_I2C_STATE_RESET 0U
#define HAL_I2C_STATE_READY 0x20U
#define HAL_I2C_STATE_BUSY_TX 0x21U
#define HAL_I2C_ERROR_BERR 1U
#define HAL_I2C_ERROR_ARLO 2U
#define HAL_I2C_ERROR_AF 4U
#define HAL_I2C_ERROR_TIMEOUT 0x20U
#define I2C_MEMADD_SIZE_8BIT 1U
void HAL_GPIO_Init(GPIO_TypeDef *,GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *,uint16_t,GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *,uint16_t);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t,uint32_t);
#endif
