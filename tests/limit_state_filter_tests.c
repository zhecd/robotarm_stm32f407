#include "common/limit_state_filter.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(int argc,char **argv)
{
    assert(argc==2);LimitStateFilter s;LimitStateFilter_Reset(&s);
    const char *mode=argv[1];
    if(!strcmp(mode,"held")){
        for(uint32_t t=0;t<1000;t++){
            LimitStateFilter_Sample(&s,t,1);assert(LimitStateFilter_BlocksSteps(&s));
            assert(s.pressed==(t>=10?1:0) && !s.unstable);
        }
    }else if(!strcmp(mode,"glitch")){
        LimitStateFilter_Sample(&s,0,4);
        for(uint32_t t=1;t<=11;t++){
            LimitStateFilter_Sample(&s,t,0);
            assert(LimitStateFilter_BlocksSteps(&s)==(t<11));
        }
        assert(!s.pressed && !s.unstable);
    }else if(!strcmp(mode,"bounce")){
        for(uint32_t t=0;t<8;t++)LimitStateFilter_Sample(&s,t,t%2?0:2);
        for(uint32_t t=8;t<=18;t++)LimitStateFilter_Sample(&s,t,2);
        assert(s.pressed==2 && !s.unstable);
    }else if(!strcmp(mode,"unstable")){
        for(uint32_t t=0;t<=100;t++)LimitStateFilter_Sample(&s,t,t%2?0:7);
        assert(s.unstable==7 && !s.pressed);
    }else if(!strcmp(mode,"gap")){
        LimitStateFilter_Sample(&s,0,1);LimitStateFilter_Sample(&s,50,1);
        assert(!s.pressed && !s.unstable);
        for(uint32_t t=51;t<=59;t++)LimitStateFilter_Sample(&s,t,1);
        assert(!s.pressed);LimitStateFilter_Sample(&s,60,1);assert(s.pressed==1);
    }else if(!strcmp(mode,"repeat")){
        for(unsigned i=0;i<20000;i++)LimitStateFilter_Sample(&s,3,4);
        assert(s.pending==4 && !s.pressed && !s.unstable);
        for(uint32_t t=4;t<=13;t++)LimitStateFilter_Sample(&s,t,4);
        assert(s.pressed==4);
    }else if(!strcmp(mode,"wrap")){
        for(uint32_t t=0;t<=10;t++)LimitStateFilter_Sample(&s,UINT32_MAX-5U+t,3);
        assert(s.pressed==3 && !s.unstable);
    }else if(!strcmp(mode,"reset")){
        for(uint32_t t=0;t<=10;t++)LimitStateFilter_Sample(&s,t,7);
        assert(s.pressed==7);LimitStateFilter_Reset(&s);
        assert(!LimitStateFilter_BlocksSteps(&s));
        LimitStateFilter_Sample(&s,1000,8);assert(s.unstable==7);
    }else assert(!"Unknown scenario");
    printf("limit_state_filter %s passed\n",mode);return 0;
}
