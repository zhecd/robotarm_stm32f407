/** 寻位500/s与可选单轴截止，纯算法不操作板卡。 */
#include "common/home_sequence.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(int argc,char **argv){
 assert(argc==2);const char *m=argv[1];HomeSequence s;
 HomeConfig c={2,0,220000,{4000,1240,200},4,2,true};
 if(!strcmp(m,"invalid")){c.seek_timeout_ms=1;HomeSequence_Init(&s,&c);assert(s.reason==HOME_BAD_CONFIG);return 0;}
 if(!strcmp(m,"nonzero"))c.seek_timeout_ms=100;
 if(!strcmp(m,"total"))c.total_timeout_ms=100;
 uint32_t base=!strcmp(m,"wrap")?UINT32_MAX-30U:0;
 HomeSequence_Init(&s,&c);HomeSequence_Tick(&s,base,0);HomeSequence_Tick(&s,base+10,0);
 assert(HomeSequence_Start(&s,base+11,0));
 if(!strcmp(m,"nonzero") || !strcmp(m,"total")){
  for(unsigned t=12;t<=111;t++)HomeSequence_Tick(&s,base+t,0);
  assert(s.reason==(!strcmp(m,"total")?HOME_TOTAL_TIMEOUT:HOME_SEEK_TIMEOUT) && !s.enable);
 }else{
  uint32_t previous=base+11,steps=0;
  for(unsigned t=12;t<=1011;t++){
   HomeSequence_Tick(&s,base+t,0);
   if(s.high){assert((uint32_t)(base+t-previous)==2U && s.seek[0]==steps+1);previous=base+t;steps++;}
  }
  assert(s.seek[0]==500 && s.phase==HOME_SEEK && s.reason==HOME_OK);
  if(!strcmp(m,"no30s")){
   HomeSequence_Tick(&s,base+31000,0);assert(s.phase==HOME_SEEK && s.seek[0]==501);
   HomeSequence_Tick(&s,base+31000,0);assert(s.seek[0]==501); /* 不补发/不重复同毫秒发步。 */
  }
 }
 printf("seek500 %s passed\n",m);return 0;
}
