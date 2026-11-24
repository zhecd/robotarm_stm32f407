#include "app_init.h"
#include "waveform_service.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void Send(const char *line)
{
    FakeHardware_ClearTx(); FakeHardware_SendLine(line); App_RunOnce();
}
static WaveformStatus_t Status(void)
{
    WaveformStatus_t status; WaveformService_GetStatus(&status); return status;
}
static void Reset(void)
{
    FakeRobot_Reset(); FakeHardware_Reset(); App_Init(); App_RunOnce();
}
int main(void)
{
    Reset();
    assert(fake_normal_init_count==0 && fake_diagnostic_init_count==1);
    assert(fake_configure_count==0 && fake_timer_start_count==1);
    for(unsigned i=0;i<1000;i++) { WaveformService_OnTickFromISR(); App_RunOnce(); }
    assert(fake_wave_rises[0]==0 && fake_wave_rises[1]==0 && fake_wave_rises[2]==0);
    Send("M115"); assert(strstr(fake_tx,"MOTOR_ENABLE:LOCKED"));
    assert(strstr(fake_tx,"PROTOCOL:WAVE7") && strstr(fake_tx,"ENCODER_MASK,TMC_HEALTH"));
    Send("M123 N0"); assert(strstr(fake_tx,"M123 NODE:0 VALID:0"));
    assert(strstr(fake_tx,"REG:06 ERROR:4"));
    Send("M122 N0"); assert(strstr(fake_tx,"M122 NODE:0 VALID:0") && fake_driver_reads==2);
    fake_driver_present[2]=true;
    Send("M122 N2");
    assert(strstr(fake_tx,"NODE:2 VALID:1 IOIN:21000001 GCONF:000001C1 CHOPCONF:14000053 IFCNT:7 VERSION:33 ENN:1"));
    Send("M123 N2");
    assert(strstr(fake_tx,"M123 NODE:2 VALID:1 IOIN:21000001 DRV_STATUS:C01F0003 CS_CODE:31 OT:1 OTPW:1 STST:1 ENN:1"));
    unsigned reads=fake_driver_reads;
    const char *bad_queries[]={"M122 N4","M122 N-1","M122 N4294967296","M122 N0 extra","M122 N",
        "M123 N4","M123 N-1","M123 N4294967296","M123 N0 extra","M123 N","M123"};
    for(unsigned i=0;i<sizeof(bad_queries)/sizeof(bad_queries[0]);i++) {
        Send(bad_queries[i]);assert(strstr(fake_tx,"error:") && fake_driver_reads==reads);
    }
    Send("M408"); assert(strstr(fake_tx,"STATE:WAVEFORM HOMED:0 ALLOWED:0 EN:DISABLED"));
    assert(strstr(fake_tx,"LIMIT_MASK:0 LIMIT_RAW:0 LIMIT_SOURCE:0 EDGE:0,0,0"));
    Send("M114"); assert(strstr(fake_tx,"VALID:1,1,1"));
    Send("M119"); assert(strstr(fake_tx,"M1:OPEN M2:OPEN M3:OPEN"));
    const char *bad[]={"G1 X10","G28","M3","M5","M999","M112 X1",
        "WAVE I1 A1 D0 N0 F50","WAVE I1 A1 D0 N101 F50",
        "WAVE I1 A0 D0 N1 F50","WAVE I1 A4 D0 N1 F50",
        "WAVE I1 A1 D2 N1 F50","WAVE I1 A1 D0 N1 F51",
        "WAVE I4294967296 A1 D0 N1 F50","WAVE I1 A1 D0 N65537 F50",
        "WAVE I1 A1 D0 N1 F50 junk","WAVE I1 A1 D0 N1 F50;M3",
        "WAVE I1 A1 D0 N1 F50 ","WAVE I-1 A1 D0 N1 F50",
        "WAVE I1","WAVE I1 A1 D0 N1 F","WAVE"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) {
        Send(bad[i]); assert(strstr(fake_tx,"error:"));
        assert(Status().state!=WAVE_RUNNING && Status().next_id==1);
    }
    Send("WAVE I1 A2 D1 N10 F50"); assert(strstr(fake_tx,"accepted:"));
    Send("M122 N2"); assert(strstr(fake_tx,"error:") && fake_driver_reads==reads);
    Send("M123 N2"); assert(strstr(fake_tx,"error:") && fake_driver_reads==reads);
    Send("WAVE I1 A2 D1 N10 F50"); assert(strstr(fake_tx,"error:"));
    Send("M112"); assert(strstr(fake_tx,"M112OK") && Status().state==WAVE_HOST_STOP);
    Send("WAVE I1 A2 D1 N10 F50"); assert(strstr(fake_tx,"error:"));
    Send("WAVE I2 A1 D0 N100 F10"); assert(Status().state==WAVE_RUNNING);
    FakeHardware_SendLine("M112"); FakeHardware_SendLine("WAVE I3 A1 D0 N1 F50");
    App_RunOnce(); App_RunOnce(); assert(Status().state==WAVE_HOST_STOP);
    for(unsigned error=0;error<3;error++) {
        Reset(); Send("WAVE I1 A1 D0 N100 F10");
        if(error==0) fake_rx_overflow=true;
        else if(error==1) fake_line_timeout=true;
        else fake_tx_overflow=true;
        FakeHardware_SendLine("WAVE I2 A1 D0 N1 F50"); App_RunOnce(); App_RunOnce();
        assert(Status().state==WAVE_SERIAL && Status().next_id==2);
    }
    Reset(); Send("WAVE I1 A1 D0 N100 F10");
    fake_encoder_failure[1]=true; FakeRobot_AdvanceUs(20000); App_RunOnce();
    assert(Status().state==WAVE_ENCODER);
    Send("WAVE I2 A1 D0 N1 F50"); assert(strstr(fake_tx,"error:"));
    assert(fake_gripper_open_count==0 && fake_gripper_close_count==0);
    assert(fake_configure_count==0 && !fake_joints_enabled && fake_enable_true_count==0);
    puts("waveform_runtime_tests: passed"); return 0;
}
