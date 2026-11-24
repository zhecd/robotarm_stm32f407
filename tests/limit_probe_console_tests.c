/** 真实最小回复器：协议标识、GPIO位图、无编码器声明及拒绝动作与截断。 */
#include "probe_console.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void)
{
    char b[256];ProbeConsoleSnapshot_t s={2,12345,{0,1,2},5};
    assert(ProbeConsole_Reply("M115",&s,b,sizeof(b)) && strstr(b,"PROTOCOL:LPUART1"));
    assert(ProbeConsole_Reply("M408",&s,b,sizeof(b)) && strstr(b,"PHASE:2 MS:12345 PENDING:0,1,2"));
    assert(ProbeConsole_Reply("M119",&s,b,sizeof(b)) && strstr(b,"M1:TRIGGERED M2:OPEN M3:TRIGGERED"));
    assert(ProbeConsole_Reply("M114",&s,b,sizeof(b)) && strstr(b,"VALID:0,0,0 ENCODER_POLL:0"));
    const char *bad[]={"M3","G1 X1","M999","M115 extra","WAVE I1 A1 D0 N1 F50"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++)
        assert(ProbeConsole_Reply(bad[i],&s,b,sizeof(b)) && !strcmp(b,"error: limit probe is read-only\r\n"));
    assert(!ProbeConsole_Reply("M115",&s,b,8) && !b[0]);
    assert(!ProbeConsole_Reply(NULL,&s,b,sizeof(b)));
    puts("limit_probe_console_tests: passed");return 0;
}
