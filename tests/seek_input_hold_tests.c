#include "input_hold.h"
#include <assert.h>
#include <string.h>
static unsigned cuts;
void SeekContactPort_StopOutputs(void){++cuts;}
static bool Tick(SeekInputHold *h,Seek *s,uint32_t t,uint8_t input)
{
    bool advance=SeekHold_Service(h,s,2,input,t);
    if(advance)Seek_Tick(s,t,input);
    return advance;
}
int main(int argc,char **argv)
{
    assert(argc==2);Seek s={0};SeekInputHold h={0};
    assert(Seek_Start(&s,0,2));
    for(unsigned t=1;t<=980;t++)assert(Tick(&h,&s,t,2));
    assert(s.steps[0]==49 && cuts==0);
    assert(!Tick(&h,&s,981,0));assert(!s.enable && !s.high && cuts==1);
    if(!strcmp(argv[1],"glitch")){
        for(unsigned t=982;t<992;t++){
            assert(!Tick(&h,&s,t,2));assert(s.steps[0]==49 && !s.enable && !s.high);
        }
        assert(Tick(&h,&s,992,2));assert(h.result==HOLD_RESUMED && s.enable && !s.high);
        for(unsigned t=993;t<1012;t++){assert(Tick(&h,&s,t,2));assert(s.steps[0]==49);}
        assert(Tick(&h,&s,1012,2));assert(s.steps[0]==50 && s.high && cuts==1);
    }else if(!strcmp(argv[1],"release")){
        for(unsigned t=982;t<=991;t++)assert(!Tick(&h,&s,t,0));
        assert(h.result==HOLD_RELEASED && s.state==SEEK_FAILED && s.steps[0]==49);
        assert(!Tick(&h,&s,1002,2) && !s.enable && !s.high);
    }else if(!strcmp(argv[1],"unstable")){
        for(unsigned t=982;t<=1081;t++)assert(!Tick(&h,&s,t,(t%2)?0:2));
        assert(h.result==HOLD_UNSTABLE && s.state==SEEK_FAILED && s.steps[0]==49);
    }else if(!strcmp(argv[1],"deadline")){
        assert(!Tick(&h,&s,SEEK_AXIS_TIMEOUT_MS,2));
        assert(s.reason==SEEK_TIME && !s.enable && !s.high);
    }else if(!strcmp(argv[1],"abort")){
        Seek_Stop(&s,SEEK_IRQ);assert(!Tick(&h,&s,992,2));
        assert(s.reason==SEEK_IRQ && !s.enable && s.steps[0]==49);
    }else if(!strcmp(argv[1],"wrap")){
        h=(SeekInputHold){0};s=(Seek){0};assert(Seek_Start(&s,0xfffffff0U,2));
        assert(!Tick(&h,&s,0xfffffff1U,0));
        assert(!Tick(&h,&s,0xfffffffeU,2));
        assert(Tick(&h,&s,8,2));assert(h.result==HOLD_RESUMED && s.steps[0]==0);
    }else{assert(!"unknown test");}
    return 0;
}
