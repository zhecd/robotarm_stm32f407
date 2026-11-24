/**
 * @file app_waveform.c
 * @brief 波形测试编排：编码器健康采样、串口请求与有限 STEP 服务。
 */
#include "app/app_waveform.h"
#include "app/app_serial_protocol.h"
#include "app_hardware_adapter.h"
#include "waveform_service.h"
#include "platform_time.h"
#include "state_service.h"
#include "safety_service.h"
#include "limit_irq_guard.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
static uint32_t s_sample_ms;
static bool s_healthy;
volatile uint32_t g_waveform_encoder_poll_mask=7U;
static uint8_t s_last_poll_mask;
uint8_t AppWaveform_EncoderPollMask(void) { return (uint8_t)(g_waveform_encoder_poll_mask&7U); }

/* 不用 sscanf 的无界整数转换；逐个数字检查，拒绝符号、小数、溢出和尾部垃圾。
 * 格式固定，便于课堂跟踪：WAVE I<id> A<1..3> D<0|1> N<1..100> F<hz>。
 */
static bool ReadNumber(const char **cursor, char tag, uint32_t *out)
{
    const char *p=*cursor;
    if (*p++!=tag || *p<'0' || *p>'9') return false;
    uint32_t value=0;
    while (*p>='0' && *p<='9') {
        uint32_t digit=(uint32_t)(*p++-'0');
        if (value>(UINT32_MAX-digit)/10U) return false;
        value=value*10U+digit;
    }
    *out=value; *cursor=p; return true;
}
void AppWaveform_HandleCommand(const char *line)
{
    if (strcmp(line,"M112")==0) {
        WaveformService_Stop(WAVE_HOST_STOP);
        AppHardware_DiscardRx();
        AppHardware_SendText("M112OK EN:DISABLED\r\n"); return;
    }
    if (strncmp(line,"M122 ",5)==0 || strncmp(line,"M123 ",5)==0) {
        bool health_query=line[3]=='3';
        uint32_t node;
        const char *query=line+5;
        WaveformStatus_t wave;
        WaveformService_GetStatus(&wave);
        if (!ReadNumber(&query,'N',&node) || *query!='\0' || node>3U ||
            wave.state==WAVE_RUNNING) {
            AppHardware_SendText("error: driver query requires N0..N3 and no running waveform\r\n"); return;
        }
        if (health_query) {
            AppDriverHealth_t health;
            char text[192];
            if (AppHardware_ReadDriverHealth((uint8_t)node,&health)) {
                /* 原始寄存器保留供复核；温度/短路位和电流编码不等同实测相电流。 */
                (void)snprintf(text,sizeof(text),
                    "M123 NODE:%lu VALID:1 IOIN:%08lX DRV_STATUS:%08lX CS_CODE:%lu OT:%lu OTPW:%lu STST:%lu ENN:%lu\r\n",
                    (unsigned long)node,(unsigned long)health.ioin,(unsigned long)health.drv_status,
                    (unsigned long)((health.drv_status>>16)&31U),
                    (unsigned long)((health.drv_status>>1)&1U),(unsigned long)(health.drv_status&1U),
                    (unsigned long)(health.drv_status>>31),(unsigned long)(health.ioin&1U));
            } else (void)snprintf(text,sizeof(text),
                "M123 NODE:%lu VALID:0 REG:%02X ERROR:%u\r\n",
                (unsigned long)node,(unsigned)health.failed_register,(unsigned)health.error);
            AppHardware_SendText(text); return;
        }
        AppDriverSnapshot_t driver;
        char text[192];
        if (AppHardware_ReadDriverSnapshot((uint8_t)node,&driver)) {
            (void)snprintf(text,sizeof(text),
                "M122 NODE:%lu VALID:1 IOIN:%08lX GCONF:%08lX CHOPCONF:%08lX IFCNT:%lu VERSION:%lu ENN:%lu\r\n",
                (unsigned long)node,(unsigned long)driver.ioin,(unsigned long)driver.gconf,
                (unsigned long)driver.chopconf,(unsigned long)(driver.ifcnt&0xFFU),
                (unsigned long)(driver.ioin>>24),(unsigned long)(driver.ioin&1U));
        } else (void)snprintf(text,sizeof(text),"M122 NODE:%lu VALID:0\r\n",(unsigned long)node);
        AppHardware_SendText(text); return;
    }
    uint32_t id,axis,dir,pulses,hz;
    const char *p=line;
    bool valid=strncmp(p,"WAVE ",5)==0;
    if (valid) p+=5;
    valid=valid && ReadNumber(&p,'I',&id) && *p++==' ' &&
        ReadNumber(&p,'A',&axis) && *p++==' ' &&
        ReadNumber(&p,'D',&dir) && *p++==' ' &&
        ReadNumber(&p,'N',&pulses) && *p++==' ' &&
        ReadNumber(&p,'F',&hz) && *p=='\0';
    if (valid && AppWaveform_EncoderPollMask()==7U && axis>=1 && axis<=3 && dir<=1 && pulses<=100 && hz<=100 &&
        WaveformService_Start(id,(uint8_t)(axis-1U),dir!=0,
                               (uint16_t)pulses,(uint16_t)hz))
        AppHardware_SendText("accepted: WAVE EN:DISABLED; poll M408\r\n");
    else AppHardware_SendText("error: waveform request rejected; poll M408\r\n");
}
void AppWaveform_Init(void)
{
    PlatformTime_Init();
    AppHardware_InitDiagnostics();
    StateService_Init(); SafetyService_Init();
    WaveformService_Init();
    AppSerial_Init(APP_SERIAL_WAVEFORM);
    s_healthy=false; s_sample_ms=PlatformTime_NowMs()-20U;
    g_waveform_encoder_poll_mask=7U;s_last_poll_mask=7U;
    /* 只开空闲定时中断，不产生脉冲；新入口永不调用 EnableJoints(true)。 */
    AppHardware_StartStepTimer();
    AppHardware_SendText("# WAVEFORM: EN disabled; motors/gripper must remain unplugged\r\n");
}
void AppWaveform_RunOnce(void)
{
    if (LimitIrqGuard_GetLatchMask()!=0U) {
        WaveformService_Stop(WAVE_IRQ_OVERLOAD);
        AppHardware_StopActuators();
        SafetyService_ReportLimitIrqOverload();
    }
    uint32_t now=PlatformTime_NowMs();
    uint8_t poll_mask=AppWaveform_EncoderPollMask();
    /* 控制字变更后本轮立即重采样/使未采样轴无效，不能沿用旧VALID与健康许可。 */
    if (poll_mask!=s_last_poll_mask) { s_sample_ms=now-20U;s_last_poll_mask=poll_mask; }
    if (poll_mask!=7U) s_healthy=false;
    if ((uint32_t)(now-s_sample_ms)>=20U) {
        s_sample_ms=now; s_healthy=poll_mask==7U;
        for (uint8_t axis=0; axis<3; ++axis) {
            if ((poll_mask&(1U<<axis))==0U) {
                StateService_PublishAxisReadFailure(axis,1U);
                continue;
            }
            float angle;
            uint32_t started=PlatformTime_NowMs();
            if (AppHardware_ReadMotorAngle(axis,&angle) && isfinite(angle))
                StateService_PublishAxisSampleTimed(axis,angle,0.0f,started,PlatformTime_NowMs());
            else {
                StateService_PublishAxisReadFailure(axis,1U);
                s_healthy=false;
                WaveformService_Feed(false);
            }
        }
    }
    WaveformService_Feed(s_healthy);
    if (AppHardware_TakeTxOverflow()) {
        WaveformService_Stop(WAVE_SERIAL); AppHardware_DiscardRx();
    }
    AppSerial_Service();
}
