/**
 * @file probe_console.h
 * @brief 最小串口对照的纯回复生成：只接受四种查询，没有动作入口。
 */
#ifndef LIMIT_PROBE_CONSOLE_H
#define LIMIT_PROBE_CONSOLE_H
#include <stddef.h>
#include <stdint.h>
typedef struct { uint32_t phase, ms, pending[3], low_mask; } ProbeConsoleSnapshot_t;
/** 返回完整回复字节数；缓冲不足/空参数返回0，不返回截断行。 */
size_t ProbeConsole_Reply(const char *line,const ProbeConsoleSnapshot_t *s,char *out,size_t capacity);
#endif
