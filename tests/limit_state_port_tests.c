/** 编译真实Device/BSP，检查状态模式下恢复/释放握手不会重开限位EXTI。
 * W1C仍是端口替身；不把此测试当作真实引脚电气消抖验证。 */
#include "main.h"
#include "device/dev_limit_switch.h"
#include "bsp/bsp_robot_board.h"
#include <assert.h>
#include <stdio.h>
EXTI_TypeDef test_exti;
static uint8_t low;
static unsigned reads;
static bool press_on_second_read;
void Test_ClearPending(uint32_t pin){test_exti.PR&=~pin;}
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin){(void)pin;assert(!"Unexpected limit callback");}
bool BSP_RobotBoard_IsLimitTriggered(uint8_t axis)
{return (low&(1U<<axis)) || (press_on_second_read && ++reads==2U);}
int main(void)
{
    const uint32_t pins=M1_STOP_Pin|M2_STOP_Pin|M3_STOP_Pin;
    test_exti=(EXTI_TypeDef){.IMR=pins|4U,.EMR=pins|4U,.RTSR=pins|4U,.FTSR=pins|4U,.PR=pins|4U};
    Dev_LimitSwitch_ResetDiagnosticInterrupts();
    assert(test_exti.IMR==4 && test_exti.EMR==4 && test_exti.RTSR==4 && test_exti.FTSR==4 && test_exti.PR==4);
    for(unsigned a=0;a<3;a++){
        low=(uint8_t)(1U<<a);assert(!Dev_LimitSwitch_TryRearmInterrupt((DevJointId_t)a));
        low=0;assert(Dev_LimitSwitch_TryRearmInterrupt((DevJointId_t)a));
        assert(test_exti.IMR==4 && test_exti.EMR==4 && test_exti.RTSR==4 && test_exti.FTSR==4);
    }
    press_on_second_read=true;reads=0;
    assert(!Dev_LimitSwitch_TryRearmInterrupt(DEV_JOINT_M1));
    assert(test_exti.IMR==4 && test_exti.PR==4);
    assert(!Dev_LimitSwitch_TryRearmInterrupt((DevJointId_t)255));
    puts("state port: no EXTI re-enable; held/released/repressed inputs protected");
}
