/** 真实App ISR、原始IRQ预算和接触状态机联合测试。仅替换GPIO/时间边界。
 * old模式关闭新钩子，重现首次接触后256个回调先于10ms确认造成停机。
 * 各CTest独立进程，禁止为了测试而添加清除生产IRQ锁存的接口。 */
#include "app_isr_adapter.h"
#include "contact_gate.h"
#include "boot_contact.h"
#include "limit_irq_guard.h"
#include "fake_robot.h"
#include "fake_hardware.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static volatile Seek seek;
static SeekContact gate={.seek=&seek};
static bool active=true,hook=true;
static bool boot_enabled;
static SeekBootContact boot;
static uint8_t input;
static uint32_t now;
static unsigned stops,masks,order;
void AppRuntime_RequestModeSwitch(void) {}
void SeekContactPort_StopOutputs(void)
{assert(order==0);order=1;++stops;AppHardware_StopActuators();}
void SeekContactPort_MaskAxis(uint8_t axis)
{assert(order==1 && !fake_joints_enabled);order=0;++masks;AppHardware_MaskLimitInterrupt(axis);}
void LimitSeek_ContactFromISR(uint8_t axis)
{
    if(boot_enabled)(void)SeekBoot_OnEdge(&boot,!active,axis,input,LimitIrqGuard_GetLatchMask(),now);
    if(hook)(void)SeekContact_OnEdge(&gate,active,axis,input,
                                    LimitIrqGuard_GetLatchMask(),now);
}
static void Edge(uint8_t axis)
{
    /* BSP对IMR的实际过滤由bsp_limit_dispatch测试覆盖。这里模拟同一硬件门。 */
    if(!(fake_limit_irq_mask&(1U<<axis)))App_ISR_OnGpioEdge((uint16_t)(M1_STOP_Pin<<axis));
}
static void Tick(uint32_t t,uint8_t low)
{
    now=t;input=low;
    LimitSeek_ContactFromISR(seek.axis); /* 与SysTick一致的GPIO补采样路径。 */
    Seek s=seek;Seek_Tick(&s,now,input);seek=s;
}
static void StartM2(void)
{
    Seek s={0};assert(Seek_Start(&s,0,1));
    Seek_Tick(&s,10,1);Seek_Tick(&s,110,1);
    assert(s.axis==1 && s.state==SEEK_MOVING);
    seek=s;now=111;fake_joints_enabled=true;
}
int main(int argc,char **argv)
{
    assert(argc==2);const char *mode=argv[1];
    FakeRobot_Reset();FakeHardware_Reset();StartM2();input=3;
    if(!strncmp(mode,"boot_",5)){
        active=false;boot_enabled=true;seek=(Seek){0};input=2;now=1;
        if(!strcmp(mode,"boot_high") || !strcmp(mode,"boot_threshold")){
            input=0;for(unsigned i=0;i<255;i++)Edge(1);
            if(!strcmp(mode,"boot_threshold"))input=2;
            Edge(1);assert(LimitIrqGuard_GetLatchMask()==2 && boot.masked==0);
        }else if(!strcmp(mode,"boot_pressed")){
            for(unsigned i=0;i<4096;i++)Edge(1);
            assert(!LimitIrqGuard_GetLatchMask() && boot.masked==2 && masks==1);
            assert(!SeekBoot_Ready(&boot,2));SeekBoot_Tick(&boot,2,10);
            assert(!SeekBoot_Ready(&boot,2));SeekBoot_Tick(&boot,2,11);
            assert(SeekBoot_Ready(&boot,2) && !fake_joints_enabled && seek.state==SEEK_WAIT);
            assert(!SeekBoot_Ready(&boot,0));
            assert(SeekBoot_ReleasedBeforeContact(&boot,1,0));
            assert(!SeekBoot_ReleasedBeforeContact(&boot,3,0));
        }else if(!strcmp(mode,"boot_bounce")){
            Edge(1);SeekBoot_Tick(&boot,0,2);SeekBoot_Tick(&boot,2,5);
            SeekBoot_Tick(&boot,2,14);assert(!SeekBoot_Ready(&boot,2));
            SeekBoot_Tick(&boot,2,15);assert(SeekBoot_Ready(&boot,2));
            SeekBoot_Tick(&boot,0,16);assert(!boot.failed && !SeekBoot_Ready(&boot,2));
            SeekBoot_Tick(&boot,2,17);SeekBoot_Tick(&boot,2,26);assert(!SeekBoot_Ready(&boot,2));
            SeekBoot_Tick(&boot,2,27);assert(SeekBoot_Ready(&boot,2));
            SeekBoot_Tick(&boot,0,28);SeekBoot_Tick(&boot,0,120);
            assert(boot.began[1]==28 && !SeekBoot_Ready(&boot,2));
            SeekBoot_Tick(&boot,0,128);assert(boot.failed && !SeekBoot_Ready(&boot,2));
        }else if(!strcmp(mode,"boot_timeout")){
            Edge(1);SeekBoot_Tick(&boot,0,2);SeekBoot_Tick(&boot,0,101);
            assert(boot.failed && !SeekBoot_Ready(&boot,2));
            assert(!SeekBoot_OnEdge(&boot,true,0,1,0,102));
        }else if(!strcmp(mode,"boot_all")){
            now=0xfffffff8U;input=7;for(uint8_t a=0;a<3;a++)Edge(a);
            SeekBoot_Tick(&boot,7,2);assert(SeekBoot_Ready(&boot,7));
            assert(boot.masked==7 && masks==3 && !LimitIrqGuard_GetLatchMask());
            assert(!SeekBoot_OnEdge(&boot,true,3,7,0,3));
        }else{assert(!"unknown boot mode");}
    }else if(!strcmp(mode,"old")){
        hook=false;
        for(unsigned i=0;i<256;i++)Edge(1);
        assert(LimitIrqGuard_GetLatchMask()==2 && !fake_joints_enabled);
        assert(seek.state==SEEK_MOVING && seek.completed==1 && gate.masked_mask==0);
    }else if(!strcmp(mode,"contact")){
        Edge(1);assert(!fake_joints_enabled && !seek.enable && !seek.high);
        assert(seek.state==SEEK_QUALIFY && masks==1 && stops==1);
        for(unsigned i=0;i<4096;i++)Edge(1);
        assert(!LimitIrqGuard_GetLatchMask() && fake_limit_irq_mask==2);
        for(now=112;now<=121;now++)Tick(now,3);
        assert(seek.completed==3 && seek.state==SEEK_GAP && seek.steps[1]==0);
        assert(masks==1 && stops==1 && !fake_limit_irq_reset_count);
    }else if(!strcmp(mode,"bounce")){
        Edge(1);Tick(112,1);Tick(116,3);Tick(120,1);Tick(125,3);
        Tick(134,3);assert(seek.state==SEEK_QUALIFY && !seek.enable);
        Tick(135,3);assert(seek.completed==3 && seek.state==SEEK_GAP);
        assert(seek.steps[1]==0 && masks==1);
    }else if(!strcmp(mode,"unstable")){
        Edge(1);
        for(unsigned t=112;t<=211;t++){Tick(t,(t%2)?3:1);assert(!seek.enable && !seek.high);}
        assert(seek.reason==SEEK_UNSTABLE && seek.state==SEEK_FAILED && seek.completed==1);
        Tick(250,3);assert(seek.state==SEEK_FAILED && seek.steps[1]==0);
    }else if(!strcmp(mode,"poll")){
        Tick(120,3);assert(seek.state==SEEK_QUALIFY && masks==1 && !fake_joints_enabled);
        Tick(130,3);assert(seek.completed==3 && !LimitIrqGuard_GetLatchMask());
    }else if(!strcmp(mode,"high") || !strcmp(mode,"other") || !strcmp(mode,"inactive")){
        uint8_t axis=!strcmp(mode,"other")?2:1;
        if(!strcmp(mode,"high"))input=1;
        if(!strcmp(mode,"other"))input=5;
        if(!strcmp(mode,"inactive"))active=false;
        for(unsigned i=0;i<256;i++)Edge(axis);
        assert(LimitIrqGuard_GetLatchMask()==(1U<<axis));
        assert(!fake_joints_enabled && gate.masked_mask==0 && masks==0);
    }else if(!strcmp(mode,"threshold")){
        input=1;for(unsigned i=0;i<255;i++)Edge(1);
        assert(!LimitIrqGuard_GetLatchMask());
        input=3;Edge(1); /* 第256次即使读到低，也必须先锁存过载。 */
        assert(LimitIrqGuard_GetLatchMask()==2 && !fake_joints_enabled && masks==0);
    }else if(!strcmp(mode,"latched_other")){
        input=1;for(unsigned i=0;i<256;i++)Edge(2);
        input=3;Edge(1);
        assert(LimitIrqGuard_GetLatchMask()==4 && gate.masked_mask==0);
    }else if(!strcmp(mode,"sequence")){
        seek=(Seek){0};Seek s=seek;assert(Seek_Start(&s,0,0));seek=s;
        Tick(1,1);Tick(11,1);assert(gate.masked_mask==1);
        Tick(111,1);Tick(112,3);Tick(122,3);assert(gate.masked_mask==3);
        Tick(222,3);Tick(223,7);Tick(233,7);
        assert(seek.state==SEEK_DONE && seek.completed==7 && gate.masked_mask==7);
        assert(!LimitIrqGuard_GetLatchMask() && masks==3 && stops==3);
    }else if(!strcmp(mode,"wrap")){
        Seek s={0};assert(Seek_Start(&s,0xfffffff0U,0));seek=s;
        Tick(0xfffffff1U,1);Tick(0xfffffff5U,0);Tick(0xfffffffeU,1);
        Tick(7,1);assert(seek.state==SEEK_QUALIFY);Tick(8,1);
        assert(seek.completed==1 && !seek.enable && gate.masked_mask==1);
    }else if(!strcmp(mode,"repeat_deadline")){
        Seek s=seek;assert(Seek_BeginContact(&s,111,1));
        assert(Seek_BeginContact(&s,205,1));assert(s.qualify_ms==111);
        Seek_Tick(&s,211,3);assert(s.reason==SEEK_UNSTABLE);
        assert(!Seek_BeginContact(&s,212,1));
    }else{assert(!"unknown mode");}
    printf("seek_contact_irq %s passed\n",mode);return 0;
}
