/* 编译真实AS5600驱动：HAL超时后有界恢复、重读、失败退避和角度不被失败覆盖。 */
#include "driver/drv_as5600.h"
#include "bsp/bsp_i2c_recovery.h"
#include "i2c.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
I2C_HandleTypeDef hi2c1,hi2c2,hi2c3;
static uint32_t now_ms,error;
static unsigned reads[3],recoveries,failures;
static bool recovery_ok,clear_on_recovery;
static HAL_StatusTypeDef failure_status;
uint32_t PlatformTime_NowMs(void) {return now_ms;}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus,uint16_t address,uint16_t reg,
                                uint16_t size,uint8_t *out,uint16_t count,uint32_t timeout)
{
    unsigned axis=bus==&hi2c1?0:bus==&hi2c2?1:2;++reads[axis];
    assert(address==0x6c && reg==0x0c && size==I2C_MEMADD_SIZE_8BIT && count==2 && timeout==10);
    if(axis==1 && failures) {--failures;bus->ErrorCode=error;return failure_status;}
    out[0]=8;out[1]=0;bus->ErrorCode=0;return HAL_OK; /* 180deg */
}
bool BSP_I2C_RecoverBus(I2C_HandleTypeDef *bus)
{
    assert(bus==&hi2c2);++recoveries;
    if(recovery_ok && clear_on_recovery) failures=0;
    return recovery_ok;
}
static As5600Device_t *Reset(void)
{
    now_ms=0;error=HAL_I2C_ERROR_TIMEOUT;reads[0]=reads[1]=reads[2]=recoveries=failures=0;
    failure_status=HAL_TIMEOUT;recovery_ok=clear_on_recovery=true;
    hi2c1=hi2c2=hi2c3=(I2C_HandleTypeDef){HAL_I2C_STATE_READY,HAL_UNLOCKED,0};
    assert(Drv_AS5600_Init()==ERR_OK);return Drv_AS5600_GetM2();
}
int main(void)
{
    /* 用明确退出码保留旧版本复现证据，避免Windows断言对话框拖到测试超时。 */
    As5600Device_t *busy=Reset();failure_status=HAL_BUSY;failures=1;
    if (Drv_AS5600_Update(busy)!=ERR_OK || recoveries!=1 || busy->recovery_successes!=1) {
        fputs("FAIL: unlocked READY handle with HAL_BUSY + TIMEOUT did not recover\n",stderr);
        return 1;
    }
    As5600Device_t *enc=Reset();
    assert(Drv_AS5600_Update(enc)==ERR_OK && fabsf(enc->angle_deg-180)<0.001f && reads[1]==5 && recoveries==0);
    failures=1;assert(Drv_AS5600_Update(enc)==ERR_OK && reads[1]==11 && recoveries==1);
    assert(enc->last_hal_error==HAL_I2C_ERROR_TIMEOUT && enc->recovery_attempts==1 && enc->recovery_successes==1);
    assert(Drv_AS5600_Update(Drv_AS5600_GetM1())==ERR_OK && Drv_AS5600_Update(Drv_AS5600_GetM3())==ERR_OK);
    assert(reads[0]==5 && reads[2]==5);

    enc=Reset();assert(Drv_AS5600_Update(enc)==ERR_OK);
    failures=100;clear_on_recovery=false;
    assert(Drv_AS5600_Update(enc)==ERR_ENCODER_FAIL && fabsf(enc->angle_deg-180)<0.001f);
    unsigned count=reads[1];assert(Drv_AS5600_Update(enc)==ERR_ENCODER_FAIL && reads[1]==count);
    now_ms=100;assert(Drv_AS5600_Update(enc)==ERR_ENCODER_FAIL && recoveries==1);
    now_ms=1000;assert(Drv_AS5600_Update(enc)==ERR_ENCODER_FAIL && recoveries==2);
    assert(enc->recovery_successes==0);

    enc=Reset();failure_status=HAL_ERROR;error=HAL_I2C_ERROR_AF;failures=1;
    assert(Drv_AS5600_Update(enc)==ERR_ENCODER_FAIL && recoveries==0);
    now_ms=100;assert(Drv_AS5600_Update(enc)==ERR_OK && !enc->retry_wait);
    assert(enc->last_hal_error==HAL_I2C_ERROR_AF); /* 不隐瞒此前失败。 */
    enc=Reset();failure_status=HAL_BUSY;failures=1;
    hi2c2.State=HAL_I2C_STATE_BUSY_TX;hi2c2.Lock=HAL_LOCKED;
    assert(Drv_AS5600_Update(enc)==ERR_ENCODER_FAIL && recoveries==0);
    enc=Reset();failure_status=HAL_BUSY;error=0;failures=1;
    assert(Drv_AS5600_Update(enc)==ERR_ENCODER_FAIL && recoveries==0);

    /* 真实F4 HAL语义：等BUSY标志超时也返回HAL_BUSY，但State=READY且已解锁。
     * 该场景必须恢复，不能与上面的真实占用混为一谈。 */
    enc=Reset();failure_status=HAL_BUSY;failures=1;
    assert(Drv_AS5600_Update(enc)==ERR_OK && recoveries==1 && enc->recovery_successes==1);

    enc=Reset();now_ms=UINT32_MAX-50U;failures=100;recovery_ok=false;
    assert(Drv_AS5600_Update(enc)==ERR_ENCODER_FAIL && recoveries==1);
    now_ms=49;assert(Drv_AS5600_Update(enc)==ERR_ENCODER_FAIL && recoveries==1);
    now_ms=949;assert(Drv_AS5600_Update(enc)==ERR_ENCODER_FAIL && recoveries==2);
    assert(Drv_AS5600_Update(NULL)==ERR_NULL_PARAM);
    assert(Drv_AS5600_Init()==ERR_OK && enc->recovery_attempts==0 && !enc->retry_wait);
    puts("as5600_driver_tests: passed");return 0;
}
