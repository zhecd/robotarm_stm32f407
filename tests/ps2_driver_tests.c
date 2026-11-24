/** 测试真实PS2配置/解析实现，只替换总线与时钟，不连接设备。 */
#include "driver/drv_ps2.h"
#include "driver/ps2_bus.h"
#include "platform_time.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t now;
static unsigned init_count,commands,polls;
static uint8_t response[9]={0xff,0x73,0x5a,0xff,0xff,128,128,128,128};
uint32_t PlatformTime_NowMs(void){return now;}
void Ps2Bus_Init(void){init_count++;}
void Ps2Bus_Exchange(const uint8_t *tx,uint8_t *rx,uint8_t length)
{
    assert(tx[0]==1U);
    if(rx){assert(tx[1]==0x42U && length==9U);memcpy(rx,response,9);polls++;}
    else {
        unsigned phase=commands%3U;
        assert(tx[1]==(phase==1U?0x44U:0x43U));
        assert(length==(phase==0U?5U:9U));
        assert(tx[3]==(phase==2U?0U:1U));commands++;
    }
}
static bool ReadAfter(uint32_t ms,Ps2Input_t *p){now+=ms;return Drv_PS2_ReadData(p);}
static void Configured(Ps2Input_t *p)
{
    assert(!ReadAfter(50,p));assert(!ReadAfter(16,p));assert(!ReadAfter(16,p));
}
int main(int argc,char **argv)
{
    assert(argc==2);Ps2Input_t value={0};const char *test=argv[1];
    if(!strcmp(test,"wrap"))now=UINT32_MAX-20U;
    Drv_PS2_Init();assert(init_count==1 && !commands && !polls);
    if(!strcmp(test,"null")){assert(!Drv_PS2_ReadData(NULL));assert(!commands && !polls);}
    else if(!strcmp(test,"timing")) {
        assert(!ReadAfter(49,&value));assert(!commands);
        assert(!ReadAfter(1,&value));assert(commands==1 && !polls);
        assert(!ReadAfter(15,&value));assert(commands==1);
        assert(!ReadAfter(1,&value));assert(commands==2);
        assert(!ReadAfter(16,&value));assert(commands==3);
        assert(!ReadAfter(15,&value));assert(!polls);
        assert(ReadAfter(1,&value));assert(polls==1);
    } else {
        Configured(&value);
        if(!strncmp(test,"diag_",5)) {
            assert(ReadAfter(16,&value));Ps2Diagnostics_t d;Drv_PS2_GetDiagnostics(&d);
            assert(d.valid_frames==1 && d.frames==1 && d.config_phase==4);
            if(strcmp(test,"diag_valid")) {
                if(!strcmp(test,"diag_unknown"))response[1]=0x12;
                else response[2]=0;
                assert(!ReadAfter(10,&value));Drv_PS2_GetDiagnostics(&d);
                assert(d.invalid_frames==1 && d.first_invalid_recorded);
                assert(d.first_invalid_status==(!strcmp(test,"diag_unknown")?PS2_READ_UNSUPPORTED_MODE:PS2_READ_BAD_HEADER));
                if(!strcmp(test,"diag_latch")) {
                    uint32_t first=d.first_invalid_ms;response[2]=0x5a;
                    assert(ReadAfter(10,&value));response[1]=0x12;assert(!ReadAfter(10,&value));
                    Drv_PS2_GetDiagnostics(&d);assert(d.last_status==PS2_READ_UNSUPPORTED_MODE);
                    assert(d.first_invalid_ms==first && d.first_invalid_frame[2]==0 && d.first_invalid_status==PS2_READ_BAD_HEADER);
                } else if(!strcmp(test,"diag_reset")) {
                    Drv_PS2_Init();Drv_PS2_GetDiagnostics(&d);
                    assert(!d.frames && !d.first_invalid_recorded && !d.invalid_frames);
                }
            }
            Drv_PS2_GetDiagnostics(NULL);
        } else if(!strcmp(test,"header")) {
            response[2]=0;value.LX=7;assert(!ReadAfter(16,&value));
            assert(value.LX==7 && !Drv_PS2_IsAnalogMode());
        } else if(!strcmp(test,"pressure")) {
            response[1]=0x79;assert(ReadAfter(16,&value));assert(Drv_PS2_IsAnalogMode());
        } else if(!strcmp(test,"digital") || !strcmp(test,"cooldown")) {
            assert(ReadAfter(16,&value));response[1]=0x41;
            if(!strcmp(test,"digital"))now=600;
            for(unsigned i=0;i<10;i++)assert(!ReadAfter(10,&value));
            if(!strcmp(test,"cooldown")) {
                assert(init_count==1);now=500;assert(!ReadAfter(10,&value));
            }
            assert(init_count==2 && !Drv_PS2_IsAnalogMode());
            response[1]=0x73;Configured(&value);assert(ReadAfter(16,&value));
            assert(commands==6 && Drv_PS2_IsAnalogMode());
        } else {
            response[3]=0x12;response[4]=0x34;response[5]=1;response[6]=2;response[7]=3;response[8]=4;
            assert(ReadAfter(16,&value));assert(value.buttons==0x1234);
            assert(value.RX==1 && value.RY==2 && value.LX==3 && value.LY==4);
        }
    }
    printf("ps2_driver_%s passed\n",test);return 0;
}
