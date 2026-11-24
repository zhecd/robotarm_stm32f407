#include "fake_hardware.h"
AppTeleopDiagnostics_t fake_ps2_diagnostics;
void AppHardware_GetTeleopDiagnostics(AppTeleopDiagnostics_t *out)
{if(out)*out=fake_ps2_diagnostics;}
#include "fake_robot.h"
#include "limit_irq_guard.h"
#include <assert.h>
#include <string.h>
char fake_tx[32768];
unsigned fake_normal_init_count, fake_diagnostic_init_count, fake_configure_count, fake_timer_start_count;
unsigned fake_limit_indicator_updates;
bool fake_indicator_gcode,fake_indicator_ps2;
bool fake_ps2_valid;
bool fake_rx_overflow, fake_line_timeout, fake_tx_overflow;
unsigned fake_driver_reads;
unsigned fake_encoder_reads;
unsigned fake_limit_irq_reset_count;
unsigned fake_gripper_init_count;
unsigned fake_gripper_init_attempts;
bool fake_gripper_init_failure;
uint32_t fake_encoder_read_delay_us;
uint8_t fake_limit_irq_mask;
bool fake_driver_present[4];
bool fake_driver_prepare_ok;
AppTeleopInput_t fake_ps2_input;
static char s_rx[64][256];
static unsigned s_head,s_tail;
void FakeHardware_ClearTx(void) { fake_tx[0]='\0'; }
void FakeHardware_Reset(void)
{
    fake_normal_init_count=fake_diagnostic_init_count=fake_configure_count=fake_timer_start_count=0;
    fake_limit_indicator_updates=0;
    fake_indicator_gcode=fake_indicator_ps2=false;
    fake_ps2_valid=false;
    fake_ps2_diagnostics=(AppTeleopDiagnostics_t){0};
    fake_rx_overflow=fake_line_timeout=fake_tx_overflow=false;
    fake_driver_reads=0;
    fake_driver_prepare_ok=true;
    fake_encoder_reads=0;
    fake_gripper_init_count=0;fake_encoder_read_delay_us=0;
    fake_gripper_init_attempts=0;fake_gripper_init_failure=false;
    fake_limit_irq_reset_count=0;fake_limit_irq_mask=0;
    memset(fake_driver_present,0,sizeof(fake_driver_present));
    fake_ps2_input=(AppTeleopInput_t){128,128,128,false,false};
    s_head=s_tail=0;
    FakeHardware_ClearTx();
}
void FakeHardware_SendLine(const char *line)
{
    assert(s_head-s_tail<64 && strlen(line)<256);
    strcpy(s_rx[s_head++%64],line);
}
void AppHardware_Init(void) { fake_normal_init_count++; fake_joints_enabled=false; }
void BSP_LimitTrace_CaptureVector(void) { /* 真实记录器另有BSP独立测试。 */ }
bool AppHardware_TryInitGripper(void)
{
    fake_gripper_init_attempts++;
    if (LimitIrqGuard_GetLatchMask() || fake_gripper_init_failure) return false;
    fake_gripper_init_count++;
    return true;
}
void AppHardware_InitGripper(void) { (void)AppHardware_TryInitGripper(); }
void AppHardware_InitDiagnostics(void) { fake_diagnostic_init_count++; fake_joints_enabled=false; }
void AppHardware_MaskDiagnosticLimitInterrupt(uint8_t axis)
{ assert(axis<3U);fake_limit_irq_mask|=(uint8_t)(1U<<axis); }
void AppHardware_MaskLimitInterrupt(uint8_t axis)
{ assert(axis<3U);fake_limit_irq_mask|=(uint8_t)(1U<<axis); }
void AppHardware_StopActuators(void)
{
    fake_joints_enabled=false;
    for (unsigned axis=0;axis<3;axis++) fake_wave_level[axis]=false;
}
bool AppHardware_StopJointSteps(void)
{
    bool ok=true;
    for(unsigned a=0;a<3U;a++){
        if(fake_step_failure[a])ok=false;else fake_wave_level[a]=false;
    }
    if(!ok)AppHardware_StopActuators();
    return ok;
}
void AppHardware_ResetDiagnosticLimitInterrupts(void)
{ ++fake_limit_irq_reset_count;fake_limit_irq_mask=0; }
void AppHardware_ConfigureMotionDrivers(void) { fake_configure_count++; fake_joints_enabled=false; }
bool AppHardware_PrepareMotionDrivers(uint8_t *failed_node)
{
    fake_configure_count++;fake_joints_enabled=false;
    if (failed_node) *failed_node=fake_driver_prepare_ok?0xFFU:1U;
    return fake_driver_prepare_ok;
}
void AppHardware_EnableJoints(bool enabled) { fake_joints_enabled=enabled && !LimitIrqGuard_GetLatchMask(); }
void AppHardware_SetModeIndicator(bool gcode, bool ps2) { fake_indicator_gcode=gcode;fake_indicator_ps2=ps2; }
void AppHardware_UpdateLimitIndicators(void) { ++fake_limit_indicator_updates; }
void AppHardware_StartStepTimer(void) { if (!LimitIrqGuard_GetLatchMask()) fake_timer_start_count++; }
bool AppHardware_IsLimitTriggered(uint8_t axis) { return axis<3 && fake_limits[axis]; }
bool AppHardware_ReadTeleopInput(AppTeleopInput_t *out)
{
    if (out) *out=fake_ps2_input;
    return fake_ps2_valid;
}
bool AppHardware_ReadMotorAngle(uint8_t axis,float *out)
{
    ++fake_encoder_reads;
    FakeRobot_AdvanceUs(fake_encoder_read_delay_us);
    return Dev_Joint_ReadMotorAngle((DevJointId_t)axis,out)==ERR_OK;
}
bool AppHardware_ReadDriverSnapshot(uint8_t node,AppDriverSnapshot_t *out)
{
    ++fake_driver_reads;
    if (node>3U || !out || !fake_driver_present[node]) return false;
    *out=(AppDriverSnapshot_t){0x21000001U,0x1C1U,0x14000053U,7U};
    return true;
}
bool AppHardware_ReadDriverHealth(uint8_t node,AppDriverHealth_t *out)
{
    ++fake_driver_reads;
    if (!out) return false;
    *out=(AppDriverHealth_t){.error=4U,.failed_register=0x06U};
    if (node>3U || !fake_driver_present[node]) return false;
    *out=(AppDriverHealth_t){.ioin=0x21000001U,.drv_status=0xC01F0003U,
                            .failed_register=0xFFU};
    return true;
}
void AppHardware_SendText(const char *text)
{
    assert(strlen(fake_tx)+strlen(text)<sizeof(fake_tx));
    strcat(fake_tx,text);
}
bool AppHardware_TakeLineTimeout(void) { bool v=fake_line_timeout; fake_line_timeout=false; return v; }
bool AppHardware_TakeRxOverflow(void) { bool v=fake_rx_overflow; fake_rx_overflow=false; return v; }
bool AppHardware_TakeTxOverflow(void) { bool v=fake_tx_overflow; fake_tx_overflow=false; return v; }
bool AppHardware_ReadLine(char *out,size_t size)
{
    if (s_head==s_tail) return false;
    assert(strlen(s_rx[s_tail%64])<size);
    strcpy(out,s_rx[s_tail++%64]);
    return true;
}
void AppHardware_DiscardRx(void) { s_tail=s_head; }
