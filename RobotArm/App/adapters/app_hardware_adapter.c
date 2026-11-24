/**
 * @file app_hardware_adapter.c
 * @brief 应用硬件端口：将 BSP/Device 操作转换为应用可用的简单接口。
 */
#include "app_hardware_adapter.h"


#include "bsp/bsp_led.h"
#include "bsp/bsp_robot_board.h"
#include "bsp/bsp_uart1.h"
#include "device/dev_gripper.h"
#include "device/dev_input.h"
#include "device/dev_joint.h"
#include "device/dev_limit_switch.h"
#include "limit_irq_guard.h"
#include "platform_critical.h"
#include "driver/drv_ps2.h"
#include "platform_time.h"

/* LED1恢复可见模式反馈，LED2/3/4仍由限位独占。该记录只供显示，不决定控制权。 */
static volatile bool s_ps2_indicator;
static volatile uint32_t s_indicator_started_ms;

static void InitIndicators(void)
{
    /* SysTick也会写LED，初始化的RAM状态与GPIO电平必须一起完成。
     * 否则恰好抢占在“状态置OFF/引脚置高”之间会留下状态与实灯不一致。 */
    PlatformCriticalState_t key=PlatformCritical_Enter();
    s_ps2_indicator=false;
    s_indicator_started_ms=PlatformTime_NowMs();
    BSP_LED_Init();
    AppHardware_UpdateLimitIndicators();
    PlatformCritical_Exit(key);
}

void AppHardware_Init(void)
{
    InitIndicators();
    Dev_Joint_Init();
    BSP_RobotBoard_StopActuators();
    BSP_UART1_Init();
    Dev_Input_Init();
}

void AppHardware_InitGripper(void)
{ (void)AppHardware_TryInitGripper(); }

bool AppHardware_TryInitGripper(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    bool accepted=LimitIrqGuard_GetLatchMask()==0U && Dev_Gripper_TryInit();
    PlatformCritical_Exit(key);
    return accepted;
}

void AppHardware_ConfigureMotionDrivers(void)
{
    /* 写入电流/细分配置不需要使能输出；就绪检查完成前始终保持 EN 禁用。 */
    Dev_Joint_EnableAll(false);
    Dev_Joint_ConfigureDrivers();
}

void AppHardware_InitDiagnostics(void)
{
    /* 不调用夹爪初始化：原有 Drv_Servo_Init 会立即闭合夹爪。 */
    InitIndicators();
    Dev_Joint_Init();
    BSP_RobotBoard_StopActuators();
    BSP_UART1_Init();
}

bool AppHardware_PrepareMotionDrivers(uint8_t *failed_node)
{
    if (failed_node!=NULL) *failed_node=0xFFU;
    AppHardware_EnableJoints(false);
    if (LimitIrqGuard_GetLatchMask()!=0U) return false;
    /* 不把阻塞UART放入临界区；ISR仍可锁存过载并立即关闭执行器。 */
    bool ready=Dev_Joint_PrepareDrivers(failed_node);
    if (LimitIrqGuard_GetLatchMask()!=0U) return false;
    return ready;
}
void AppHardware_MaskDiagnosticLimitInterrupt(uint8_t axis)
{
    if (axis<DEV_JOINT_COUNT) Dev_LimitSwitch_MaskDiagnosticInterrupt((DevJointId_t)axis);
}
void AppHardware_ResetDiagnosticLimitInterrupts(void)
{
    Dev_LimitSwitch_ResetDiagnosticInterrupts();
}
void AppHardware_MaskLimitInterrupt(uint8_t axis)
{
    if (axis<DEV_JOINT_COUNT) Dev_LimitSwitch_MaskInterrupt((DevJointId_t)axis);
}
void AppHardware_StopActuators(void) { BSP_RobotBoard_StopActuators(); }
bool AppHardware_StopJointSteps(void)
{
    bool ok=true;
    for(uint8_t a=0;a<DEV_JOINT_COUNT;a++)ok=Dev_Joint_WriteStepLevel((DevJointId_t)a,false)&&ok;
    if(!ok)BSP_RobotBoard_StopActuators(); /* 端口不受理时退回直接板级停止。 */
    return ok;
}

bool AppHardware_ReadMotorAngle(uint8_t axis, float *out_deg)
{
    return axis < DEV_JOINT_COUNT &&
           Dev_Joint_ReadMotorAngle((DevJointId_t)axis, out_deg) == ERR_OK;
}

void AppHardware_EnableJoints(bool enabled)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    Dev_Joint_EnableAll(enabled && LimitIrqGuard_GetLatchMask()==0U);
    PlatformCritical_Exit(key);
}

bool AppHardware_ReadDriverSnapshot(uint8_t node, AppDriverSnapshot_t *out)
{
    if (node>3U || out==NULL) return false;
    AppDriverSnapshot_t value;
    if (!BSP_RobotBoard_ReadDriverRegister(node,0x06U,&value.ioin) ||
        !BSP_RobotBoard_ReadDriverRegister(node,0x00U,&value.gconf) ||
        !BSP_RobotBoard_ReadDriverRegister(node,0x6CU,&value.chopconf) ||
        !BSP_RobotBoard_ReadDriverRegister(node,0x02U,&value.ifcnt)) return false;
    *out=value;
    return true;
}

bool AppHardware_ReadDriverHealth(uint8_t node, AppDriverHealth_t *out)
{
    if (out==NULL) return false;
    *out=(AppDriverHealth_t){.error=1U,.failed_register=0xFFU};
    if (node>3U) return false;
    AppDriverHealth_t value={.failed_register=0xFFU};
    const uint8_t regs[]={0x06U,0x6FU}; /* IOIN、DRV_STATUS均为只读寄存器。 */
    uint32_t *values[]={&value.ioin,&value.drv_status};
    for (unsigned i=0U;i<2U;++i) {
        uint8_t error=BSP_RobotBoard_ReadDriverRegisterResult(node,regs[i],values[i]);
        if (error!=0U) {
            *out=(AppDriverHealth_t){.error=error,.failed_register=regs[i]};
            return false; /* 不发布前半个快照，也不自动重试。 */
        }
    }
    *out=value;
    return true;
}

void AppHardware_SetModeIndicator(bool gcode_on, bool ps2_on)
{
    /* 只有LED1表达模式：串口常亮，PS2每500ms翻转。模式变化从亮相位开始。
     * 此函数不写LED2/3/4，不触发运动或手柄接管，不以灯状态反推控制模式。 */
    bool next=ps2_on && !gcode_on;
    PlatformCriticalState_t key=PlatformCritical_Enter();
    if(next!=s_ps2_indicator) {
        s_indicator_started_ms=PlatformTime_NowMs();
        s_ps2_indicator=next;
    }
    PlatformCritical_Exit(key);
}

void AppHardware_UpdateLimitIndicators(void)
{
    bool run_on=!s_ps2_indicator ||
        (((uint32_t)(PlatformTime_NowMs()-s_indicator_started_ms)/500U)&1U)==0U;
    const LedState_t requested[4]={run_on?LED_ON:LED_OFF,
        Dev_LimitSwitch_IsTriggered(DEV_JOINT_M1)?LED_ON:LED_OFF,
        Dev_LimitSwitch_IsTriggered(DEV_JOINT_M2)?LED_ON:LED_OFF,
        Dev_LimitSwitch_IsTriggered(DEV_JOINT_M3)?LED_ON:LED_OFF};
    for (unsigned i=0;i<4;i++) {
        LedNumber_t led=(LedNumber_t)i;
        if (BSP_LED_GetState(led)!=requested[i]) (void)BSP_LED_SetState(led,requested[i]);
    }
}

void AppHardware_StartStepTimer(void)
{
    PlatformCriticalState_t key=PlatformCritical_Enter();
    if (LimitIrqGuard_GetLatchMask()==0U) BSP_RobotBoard_StartStepTimer();
    PlatformCritical_Exit(key);
}

bool AppHardware_IsLimitTriggered(uint8_t joint_index)
{
    if (joint_index >= DEV_JOINT_COUNT) return false;
    return Dev_LimitSwitch_IsTriggered((DevJointId_t)joint_index);
}

bool AppHardware_ReadTeleopInput(AppTeleopInput_t *out_input)
{
    if (!out_input) return false;

    DevInputState_t input = {0};
    if (!Dev_Input_Read(&input)) return false;

    out_input->left_x = input.left_x;
    out_input->left_y = input.left_y;
    out_input->right_y = input.right_y;
    out_input->cross_pressed = (input.buttons & DEV_INPUT_BTN_CROSS) == 0U;
    out_input->square_pressed = (input.buttons & DEV_INPUT_BTN_SQUARE) == 0U;
    return true;
}

void AppHardware_GetTeleopDiagnostics(AppTeleopDiagnostics_t *out)
{
    if(!out)return;
    Ps2Diagnostics_t d;Drv_PS2_GetDiagnostics(&d);
    *out=(AppTeleopDiagnostics_t){.frames=d.frames,.valid_frames=d.valid_frames,
        .invalid_frames=d.invalid_frames,.last_invalid_ms=d.last_invalid_ms,
        .first_invalid_ms=d.first_invalid_ms,.last_status=d.last_status,.config_phase=d.config_phase,
        .first_invalid_recorded=d.first_invalid_recorded,.first_invalid_status=d.first_invalid_status};
    for(unsigned i=0;i<9;i++){out->last_frame[i]=d.last_frame[i];out->first_invalid_frame[i]=d.first_invalid_frame[i];}
}

void AppHardware_SendText(const char *text) { BSP_UART1_SendString(text); }
bool AppHardware_TakeLineTimeout(void) { return BSP_UART1_TakeLineTimeout(); }
bool AppHardware_TakeRxOverflow(void) { return BSP_UART1_TakeRxOverflow(); }
bool AppHardware_TakeTxOverflow(void) { return BSP_UART1_TakeTxOverflow(); }
bool AppHardware_ReadLine(char *out_line, size_t out_size) { return BSP_UART1_ReadLine(out_line, out_size); }
void AppHardware_DiscardRx(void) { BSP_UART1_DiscardRx(); }
