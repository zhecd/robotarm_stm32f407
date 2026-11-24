/** 使用真实 AppHardware 实现，检查只读初始化不触及驱动、夹爪和 STEP 启动。 */
#include "app_hardware_adapter.h"
#include "driver/drv_ps2.h"
#include "bsp/bsp_led.h"
#include "bsp/bsp_uart1.h"
#include "bsp/bsp_robot_board.h"
#include "device/dev_joint.h"
#include "device/dev_gripper.h"
#include "device/dev_input.h"
#include "device/dev_limit_switch.h"
#include "limit_irq_guard.h"
#include "platform_critical.h"
#include <assert.h>
#include <stdio.h>
void Drv_PS2_GetDiagnostics(Ps2Diagnostics_t *out)
{assert(out);*out=(Ps2Diagnostics_t){.frames=123,.valid_frames=122,.invalid_frames=1};}
static unsigned joint_init,uart_init,input_init,gripper_init,configure,step_start,stop_count;
static bool enabled;
static unsigned step_low_mask;
static bool step_port_failure;
static unsigned driver_reads,driver_fail_at;
static unsigned irq_masks,irq_resets;
static DevJointId_t irq_axis;
static uint8_t guard_mask;
static bool prepare_ok=true,prepare_irq;
static bool gripper_init_ok=true;
static unsigned critical_depth;
static LedState_t led_states[4];
static uint8_t limit_mask=2U;
static uint32_t indicator_ms;
uint32_t PlatformTime_NowMs(void) { return indicator_ms; }
uint8_t LimitIrqGuard_GetLatchMask(void) { return guard_mask; }
PlatformCriticalState_t PlatformCritical_Enter(void) { ++critical_depth;return 0U; }
void PlatformCritical_Exit(PlatformCriticalState_t key) { (void)key;assert(critical_depth);--critical_depth; }
static uint8_t driver_regs[4],driver_nodes[4];
bool BSP_RobotBoard_ReadDriverRegister(uint8_t node,uint8_t reg,uint32_t *out)
{
    assert(driver_reads<4 && out);
    driver_regs[driver_reads]=reg;driver_nodes[driver_reads]=node;
    ++driver_reads;
    if (driver_reads==driver_fail_at) return false;
    *out=reg;return true;
}
uint8_t BSP_RobotBoard_ReadDriverRegisterResult(uint8_t node,uint8_t reg,uint32_t *out)
{
    return BSP_RobotBoard_ReadDriverRegister(node,reg,out)?0U:4U;
}
void Dev_Joint_Init(void) { joint_init++; }
void Dev_Joint_ConfigureDrivers(void) { configure++; }
bool Dev_Joint_PrepareDrivers(uint8_t *failed_node)
{
    assert(!enabled && critical_depth==0); /* UART等待期间不屏蔽中断。 */
    ++configure;
    if (failed_node) *failed_node=prepare_ok?0xFFU:2U;
    if (prepare_irq) guard_mask=4U;
    return prepare_ok;
}
void Dev_Joint_EnableAll(bool on) { enabled=on; }
bool Dev_Joint_WriteStepLevel(DevJointId_t axis,bool high)
{assert((unsigned)axis<3U && !high);step_low_mask|=1U<<axis;return !step_port_failure;}
ErrorCode_t Dev_Joint_ReadMotorAngle(DevJointId_t axis,float *out)
{ if (out && axis<DEV_JOINT_COUNT) { *out=12;return ERR_OK; }return ERR_NULL_PARAM; }
bool Dev_Gripper_TryInit(void)
{ if (!gripper_init_ok) return false;gripper_init++;return true; }
void Dev_Gripper_Init(void) { (void)Dev_Gripper_TryInit(); }
void Dev_Input_Init(void) { input_init++; }
bool Dev_Input_Read(DevInputState_t *out) { (void)out;return false; }
bool Dev_LimitSwitch_IsTriggered(DevJointId_t axis) { return (limit_mask&(1U<<axis))!=0; }
void Dev_LimitSwitch_MaskDiagnosticInterrupt(DevJointId_t axis) { ++irq_masks;irq_axis=axis; }
void Dev_LimitSwitch_MaskInterrupt(DevJointId_t axis) { ++irq_masks;irq_axis=axis; }
void Dev_LimitSwitch_ResetDiagnosticInterrupts(void) { ++irq_resets; }
void BSP_LED_Init(void) { assert(critical_depth>0);for(unsigned i=0;i<4;i++)led_states[i]=LED_OFF; }
bool BSP_LED_SetState(LedNumber_t led,LedState_t state) { assert((unsigned)led<4);led_states[led]=state;return true; }
LedState_t BSP_LED_GetState(LedNumber_t led) { assert((unsigned)led<4);return led_states[led]; }
void BSP_RobotBoard_StartStepTimer(void) { step_start++; }
void BSP_RobotBoard_StopActuators(void) { stop_count++;enabled=false; }
void BSP_UART1_Init(void) { uart_init++; }
void BSP_UART1_SendString(const char *text) { (void)text; }
bool BSP_UART1_TakeLineTimeout(void) { return false; }
bool BSP_UART1_TakeRxOverflow(void) { return false; }
bool BSP_UART1_TakeTxOverflow(void) { return false; }
bool BSP_UART1_ReadLine(char *out,uint16_t size) { (void)out;(void)size;return false; }
void BSP_UART1_DiscardRx(void) {}
int main(void)
{
    /* 穷举三路原始限位状态；所有模式/故障通知都不能覆盖限位灯。 */
    for(limit_mask=0;limit_mask<8;limit_mask++) {
        AppHardware_UpdateLimitIndicators();
        AppHardware_SetModeIndicator(true,false);AppHardware_SetModeIndicator(false,true);
        AppHardware_SetModeIndicator(false,false);
        assert(led_states[0]==LED_ON);
        for(unsigned a=0;a<3;a++)assert(led_states[a+1]==((limit_mask&(1U<<a))?LED_ON:LED_OFF));
    }
    assert(!joint_init && !configure && !step_start && !gripper_init && !enabled);
    /* 真实显示端口：相位从切入PS2开始，500ms亮/500ms灭，不覆盖任一路限位。 */
    limit_mask=5;indicator_ms=1234;AppHardware_SetModeIndicator(false,true);
    for(unsigned n=0;n<5;n++) {
        indicator_ms=1234U+n*250U;AppHardware_UpdateLimitIndicators();
        assert(led_states[0]==((n==2 || n==3)?LED_OFF:LED_ON));
        assert(led_states[1]==LED_ON && led_states[2]==LED_OFF && led_states[3]==LED_ON);
        AppHardware_SetModeIndicator(false,true); /* 重复通知不重置闪烁计时。 */
    }
    AppHardware_SetModeIndicator(true,false);AppHardware_UpdateLimitIndicators();assert(led_states[0]==LED_ON);
    indicator_ms=UINT32_MAX-100U;AppHardware_SetModeIndicator(false,true);
    indicator_ms=399U;AppHardware_UpdateLimitIndicators();assert(led_states[0]==LED_OFF); /* 恰好500ms，含回绕。 */
    AppHardware_SetModeIndicator(true,false);
    limit_mask=2;
    enabled=true;assert(AppHardware_StopJointSteps());
    assert(step_low_mask==7 && enabled && !stop_count && !gripper_init);
    enabled=true; AppHardware_InitDiagnostics();
    assert(joint_init==1 && uart_init==1 && stop_count==1 && !enabled);
    assert(input_init==0 && gripper_init==0 && configure==0 && step_start==0);
    AppHardware_MaskDiagnosticLimitInterrupt(1);assert(irq_masks==1 && irq_axis==DEV_JOINT_M2);
    AppHardware_MaskDiagnosticLimitInterrupt(3);assert(irq_masks==1);
    AppHardware_ResetDiagnosticLimitInterrupts();assert(irq_resets==1);
    float angle; assert(AppHardware_ReadMotorAngle(0,&angle) && angle==12);
    assert(!AppHardware_ReadMotorAngle(3,&angle));
    assert(AppHardware_IsLimitTriggered(1) && !AppHardware_IsLimitTriggered(3));
    enabled=true;
    AppHardware_Init(); assert(gripper_init==0 && input_init==1 && !enabled && stop_count==2);
    AppHardware_ConfigureMotionDrivers(); assert(!enabled && configure==1);
    AppHardware_InitGripper(); assert(gripper_init==1);
    AppHardware_StartStepTimer(); assert(step_start==1);
    guard_mask=2U;
    AppHardware_EnableJoints(true); assert(!enabled);
    AppHardware_InitGripper(); assert(gripper_init==1);
    AppHardware_StartStepTimer(); assert(step_start==1);
    AppHardware_MaskLimitInterrupt(2); assert(irq_axis==DEV_JOINT_M3);
    AppHardware_StopActuators(); assert(!enabled && stop_count==3);
    guard_mask=0U;
    gripper_init_ok=false;
    assert(!AppHardware_TryInitGripper() && gripper_init==1 && !critical_depth);
    gripper_init_ok=true;
    assert(AppHardware_TryInitGripper() && gripper_init==2 && !critical_depth);
    AppDriverSnapshot_t driver={99,99,99,99};
    assert(!AppHardware_ReadDriverSnapshot(4,&driver));
    assert(!AppHardware_ReadDriverSnapshot(0,NULL) && driver_reads==0);
    assert(AppHardware_ReadDriverSnapshot(2,&driver));
    assert(driver_reads==4 && driver.ioin==6 && driver.gconf==0 &&
           driver.chopconf==0x6C && driver.ifcnt==2);
    const uint8_t expected_regs[]={6,0,0x6C,2};
    for (unsigned i=0;i<4;i++) assert(driver_regs[i]==expected_regs[i] && driver_nodes[i]==2);
    for(unsigned fail=1;fail<=4;fail++) {
        driver_reads=0;driver_fail_at=fail;driver=(AppDriverSnapshot_t){99,99,99,99};
        assert(!AppHardware_ReadDriverSnapshot(0,&driver) && driver_reads==fail);
        assert(driver.ioin==99 && driver.gconf==99 && driver.chopconf==99 && driver.ifcnt==99);
    }
    driver_reads=driver_fail_at=0;
    AppDriverHealth_t health;
    assert(!AppHardware_ReadDriverHealth(0,NULL) && driver_reads==0);
    assert(!AppHardware_ReadDriverHealth(4,&health) && driver_reads==0);
    assert(health.error==1 && health.failed_register==0xFF && health.ioin==0 && health.drv_status==0);
    assert(AppHardware_ReadDriverHealth(1,&health));
    assert(driver_reads==2 && driver_nodes[0]==1 && driver_nodes[1]==1);
    assert(driver_regs[0]==6 && driver_regs[1]==0x6F && health.ioin==6 && health.drv_status==0x6F);
    assert(health.error==0 && health.failed_register==0xFF);
    for(unsigned failure=1;failure<=2;failure++) {
        driver_reads=0;driver_fail_at=failure;health=(AppDriverHealth_t){99,99,99,99};
        assert(!AppHardware_ReadDriverHealth(2,&health) && driver_reads==failure);
        assert(health.error==4 && health.failed_register==(failure==1?6:0x6F));
        assert(health.ioin==0 && health.drv_status==0); /* 前半快照不能被当成成功数据。 */
    }
    uint8_t failed=99;
    enabled=true;assert(AppHardware_PrepareMotionDrivers(&failed));
    assert(!enabled && failed==0xFF && configure==2);
    prepare_ok=false;
    assert(!AppHardware_PrepareMotionDrivers(&failed) && failed==2 && configure==3);
    guard_mask=4;
    assert(!AppHardware_PrepareMotionDrivers(&failed) && failed==0xFF && configure==3);
    guard_mask=0;prepare_ok=true;prepare_irq=true;
    assert(!AppHardware_PrepareMotionDrivers(NULL) && !enabled && configure==4);
    unsigned before_stop=stop_count;enabled=true;step_port_failure=true;
    assert(!AppHardware_StopJointSteps() && !enabled && stop_count==before_stop+1);
    AppTeleopDiagnostics_t diag;AppHardware_GetTeleopDiagnostics(&diag);
    assert(diag.frames==123 && diag.valid_frames==122 && diag.invalid_frames==1);
    puts("hardware_adapter_tests: passed");return 0;
}
