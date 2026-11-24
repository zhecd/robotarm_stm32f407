/** 首坏帧查询使用显式替代设备快照；验证真实解析/序列化和故障门禁。 */
#include "app/app_serial_protocol.h"
#include "app/app_gcode_parser.h"
#include "safety_service.h"
#include "state_service.h"
#include "motion_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
int main(void)
{
    FakeRobot_Reset();FakeHardware_Reset();SafetyService_Init();StateService_Init();MotionService_Init();
    fake_ps2_diagnostics=(AppTeleopDiagnostics_t){.frames=UINT32_MAX,.valid_frames=42,.invalid_frames=1,
        .first_invalid_recorded=1,.first_invalid_status=2,.first_invalid_ms=1234,.last_status=2,.config_phase=4};
    for(unsigned i=0;i<9;i++)fake_ps2_diagnostics.first_invalid_frame[i]=fake_ps2_diagnostics.last_frame[i]=(uint8_t)(0xF0U+i);
    SafetyService_ReportInputLost();
    for(unsigned mode=0;mode<=6;mode++){
        AppSerial_Init((AppSerialMode_t)mode);FakeHardware_ClearTx();
        FakeHardware_SendLine("M412");AppSerial_Service();
        assert(strstr(fake_tx,"M412 FRAMES:4294967295 VALID:42 INVALID:1"));
        assert(strstr(fake_tx,"FIRST_RECORDED:1 FIRST_STATUS:2 FIRST_MS:1234"));
        assert(strstr(fake_tx,"FIRST:F0F1F2F3F4F5F6F7F8\r\n") && strlen(fake_tx)<384);
        assert(!fake_driver_reads && !fake_encoder_reads && !fake_enable_true_count);
        assert(!fake_gripper_open_count && !fake_gripper_close_count && SafetyService_GetFault()==SAFETY_FAULT_INPUT_LOST);
    }
    GCodeFrame_t f;assert(App_GCodeParser_ParseLine("M412",&f) && f.type==GCMD_M412);
    assert(!App_GCodeParser_ParseLine("M412 X1",&f) && !App_GCodeParser_ParseLine("M412X",&f));
    puts("PS2 diagnostic query passed");return 0;
}
