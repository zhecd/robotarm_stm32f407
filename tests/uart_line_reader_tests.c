#include "bsp/uart_line_reader.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
static UartLineReader_t r;
static char line[256];
static unsigned Feed(const char *text, uint32_t now)
{
    unsigned count=0;
    while (*text) if (UartLineReader_Feed(&r, (uint8_t)*text++, now, line, sizeof(line))) count++;
    return count;
}
int main(void)
{
    UartLineReader_Init(&r);
    assert(Feed("G1 X", 0) == 0);
    assert(Feed("10\r", 1) == 1 && strcmp(line,"G1 X10") == 0);
    assert(Feed("\n\nM114\nM119\r\n", 2) == 2 && strcmp(line,"M119") == 0);
    for (unsigned i=0;i<255;i++) assert(!UartLineReader_Feed(&r,'A',3,line,sizeof(line)));
    assert(UartLineReader_Feed(&r,'\n',3,line,sizeof(line)) && strlen(line)==255);
    for (unsigned i=0;i<260;i++) UartLineReader_Feed(&r,'B',4,line,sizeof(line));
    assert(Feed("M3\n",4)==0 && r.overflow);
    assert(Feed("M114\n",5)==1); /* 超长行的后缀 M3 未成为新命令。 */
    assert(Feed("G1 X",UINT32_MAX-10)==0);
    UartLineReader_CheckTimeout(&r, 501, 500);
    assert(r.timed_out && Feed("20\n",502)==0);
    assert(Feed("M119\r\n",503)==1);
    assert(Feed("M",504)==0);
    assert(!UartLineReader_Feed(&r,0,504,line,sizeof(line)));
    assert(Feed("3\n",504)==0);
    UartLineReader_Invalidate(&r);
    assert(Feed("G1 X10\n",505)==0);
    assert(Feed("M114\n",506)==1);
    assert(Feed("M114",507)==0);
    char small[3];
    assert(!UartLineReader_Feed(&r,'\n',507,small,sizeof(small)));
    assert(r.overflow);
    puts("uart_line_reader_tests: passed"); return 0;
}
