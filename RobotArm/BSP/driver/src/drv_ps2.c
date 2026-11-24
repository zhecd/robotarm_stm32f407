/** @file drv_ps2.c
 * @brief PS2模式协商和数据解释。主循环每次最多交换一帧，不使用HAL_Delay。
 * Init只初始化端口并开始50ms等待；ReadData在配置未结束时返回false，
 * App的首次接管窗口负责等待，已经接管后的无效帧仍由App立即停机。
 */
#include "driver/drv_ps2.h"
#include "driver/ps2_bus.h"
#include "platform_time.h"
#include <stddef.h>

#define PS2_REINIT_RETRY_THRESHOLD 10U
#define PS2_REINIT_COOLDOWN_MS 500U
static bool s_analog_mode;
static uint8_t s_invalid_mode_count;
static uint32_t s_last_reinit_ms;
static uint32_t s_phase_started_ms;
static uint8_t s_phase; /* 0等50ms；1/2/3等16ms；4正常查询。 */
/* volatile保留现场，正式Release可在停机后通过SWD只读取得；不在中断打印。 */
static volatile Ps2Diagnostics_t s_diagnostics;

void Drv_PS2_Init(void)
{
    Ps2Bus_Init();s_phase=0U;s_phase_started_ms=PlatformTime_NowMs();
    s_analog_mode=false;s_invalid_mode_count=0U;
    s_diagnostics=(Ps2Diagnostics_t){0};
}
static bool ConfigurationReady(void)
{
    static const uint8_t enter[]={1U,0x43U,0U,1U,0U};
    static const uint8_t mode[]={1U,0x44U,0U,1U,3U,0U,0U,0U,0U};
    static const uint8_t leave[]={1U,0x43U,0U,0U,0x5AU,0x5AU,0x5AU,0x5AU,0x5AU};
    if(s_phase==4U)return true;
    uint32_t now=PlatformTime_NowMs();
    if((uint32_t)(now-s_phase_started_ms)<(s_phase==0U?50U:16U))return false;
    if(s_phase==0U)Ps2Bus_Exchange(enter,NULL,(uint8_t)sizeof(enter));
    else if(s_phase==1U)Ps2Bus_Exchange(mode,NULL,(uint8_t)sizeof(mode));
    else if(s_phase==2U)Ps2Bus_Exchange(leave,NULL,(uint8_t)sizeof(leave));
    s_phase++;s_phase_started_ms=now;
    s_diagnostics.config_phase=s_phase;
    return s_phase==4U; /* 第四阶段只结束等待，本次可查询一帧。 */
}
static void RecordFrame(const uint8_t raw[9],Ps2ReadStatus_t status)
{
    s_diagnostics.frames++;s_diagnostics.last_status=(uint8_t)status;
    for(uint8_t i=0;i<9U;i++)s_diagnostics.last_frame[i]=raw[i];
    if(status==PS2_READ_VALID){s_diagnostics.valid_frames++;return;}
    uint32_t now=PlatformTime_NowMs();
    s_diagnostics.invalid_frames++;s_diagnostics.last_invalid_ms=now;
    if(s_diagnostics.valid_frames && !s_diagnostics.first_invalid_recorded) {
        s_diagnostics.first_invalid_ms=now;s_diagnostics.first_invalid_status=(uint8_t)status;
        for(uint8_t i=0;i<9U;i++)s_diagnostics.first_invalid_frame[i]=raw[i];
        s_diagnostics.first_invalid_recorded=1U; /* 内容先写，标志最后发布。 */
    }
}
bool Drv_PS2_ReadData(Ps2Input_t *data)
{
    if(!data || !ConfigurationReady())return false;
    const uint8_t tx[9]={1U,0x42U,0U,0U,0U,0U,0U,0U,0U};
    uint8_t raw[9]={0};Ps2Bus_Exchange(tx,raw,(uint8_t)sizeof(tx));
    Ps2ReadStatus_t status=raw[2]!=0x5AU?PS2_READ_BAD_HEADER:
        (raw[1]==0x73U || raw[1]==0x79U)?PS2_READ_VALID:
        raw[1]==0x41U?PS2_READ_DIGITAL:PS2_READ_UNSUPPORTED_MODE;
    RecordFrame(raw,status);
    if(raw[2]!=0x5AU) {s_analog_mode=false;return false;}
    if(raw[1]!=0x73U && raw[1]!=0x79U) {
        s_analog_mode=false;
        if(raw[1]==0x41U) {
            /* 有效数字包说明传输存在，但没有可用模拟摇杆。饱和计数避免回绕。
             * 10次后允许重新协商；App首次窗口允许走到这里，运动中则已先停机。 */
            if(s_invalid_mode_count<PS2_REINIT_RETRY_THRESHOLD)s_invalid_mode_count++;
            uint32_t now=PlatformTime_NowMs();
            if(s_invalid_mode_count>=PS2_REINIT_RETRY_THRESHOLD &&
               (uint32_t)(now-s_last_reinit_ms)>=PS2_REINIT_COOLDOWN_MS) {
                s_last_reinit_ms=now;Drv_PS2_Init();
            }
        } else s_invalid_mode_count=0U;
        return false;
    }
    s_analog_mode=true;s_invalid_mode_count=0U;
    data->buttons=(uint16_t)(((uint16_t)raw[3]<<8U)|raw[4]);
    data->RX=raw[5];data->RY=raw[6];data->LX=raw[7];data->LY=raw[8];
    return true;
}
bool Drv_PS2_IsAnalogMode(void){return s_analog_mode;}
void Drv_PS2_GetDiagnostics(Ps2Diagnostics_t *out){if(out)*out=s_diagnostics;}
