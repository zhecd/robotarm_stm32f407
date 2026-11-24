/**
 * @file probe_console.c
 * @brief 仅格式化最小诊断快照，不读取GPIO、不访问HAL、不决定运动。
 */
#include "probe_console.h"
#include <stdio.h>
#include <string.h>
size_t ProbeConsole_Reply(const char *line,const ProbeConsoleSnapshot_t *s,char *out,size_t capacity)
{
    if(!line || !s || !out || !capacity)return 0;
    int n;
    if(!strcmp(line,"M115"))
        n=snprintf(out,capacity,"M115 FIRMWARE:LimitProbe PROTOCOL:LPUART1 UART:115200 MOTOR_ENABLE:LOCKED FEATURES:GPIO,EXTI,POLL_UART\r\n");
    else if(!strcmp(line,"M408"))
        n=snprintf(out,capacity,"M408 STATE:LIMIT_PROBE EN:DISABLED PHASE:%lu MS:%lu PENDING:%lu,%lu,%lu\r\n",
            (unsigned long)s->phase,(unsigned long)s->ms,(unsigned long)s->pending[0],
            (unsigned long)s->pending[1],(unsigned long)s->pending[2]);
    else if(!strcmp(line,"M114"))
        n=snprintf(out,capacity,"M114 RAWENC:0,0,0 VALID:0,0,0 ENCODER_POLL:0\r\n");
    else if(!strcmp(line,"M119"))
        n=snprintf(out,capacity,"M119 M1:%s M2:%s M3:%s\r\n",s->low_mask&1?"TRIGGERED":"OPEN",
            s->low_mask&2?"TRIGGERED":"OPEN",s->low_mask&4?"TRIGGERED":"OPEN");
    else n=snprintf(out,capacity,"error: limit probe is read-only\r\n");
    if(n<0 || (size_t)n>=capacity) { out[0]='\0';return 0; }
    return (size_t)n;
}
