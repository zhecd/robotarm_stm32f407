/** 实际WAVE入口和协议：暂停采样不伪造VALID，不沿用旧健康状态启动波形。 */
#include "app_init.h"
#include "app/app_waveform.h"
#include "waveform_service.h"
#include "fake_hardware.h"
#include "fake_robot.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void Send(const char *s) { FakeHardware_ClearTx();FakeHardware_SendLine(s);App_RunOnce(); }
int main(void)
{
    FakeRobot_Reset();FakeHardware_Reset();App_Init();App_RunOnce();
    assert(AppWaveform_EncoderPollMask()==7);
    const unsigned masks[]={0,1,2,4,7};
    const char *valid[]={"VALID:0,0,0","VALID:1,0,0","VALID:0,1,0","VALID:0,0,1","VALID:1,1,1"};
    for(unsigned i=0;i<5;i++) {
        unsigned before=fake_encoder_reads;
        g_waveform_encoder_poll_mask=masks[i];
        Send("M114");assert(strstr(fake_tx,valid[i]));
        unsigned expected=masks[i]==7?3:(masks[i]?1:0);
        assert(fake_encoder_reads-before==expected);
        Send("M408");char text[48];snprintf(text,sizeof(text),"ENCODER_POLL_MASK:%u",masks[i]);
        assert(strstr(fake_tx,text));
    }
    g_waveform_encoder_poll_mask=0; /* 不等20ms，立即失去旧采样许可。 */
    Send("WAVE I1 A1 D0 N1 F50");assert(strstr(fake_tx,"error:"));
    g_waveform_encoder_poll_mask=7;
    Send("WAVE I1 A1 D0 N100 F10");assert(strstr(fake_tx,"accepted:"));
    g_waveform_encoder_poll_mask=4;App_RunOnce();
    WaveformStatus_t wave;WaveformService_GetStatus(&wave);assert(wave.state==WAVE_ENCODER);
    assert(!fake_enable_true_count && !fake_gripper_init_count && !fake_configure_count);
    App_Init();assert(AppWaveform_EncoderPollMask()==7);
    puts("waveform_encoder_mask_tests: passed");return 0;
}
