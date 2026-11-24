/**
 * @file uart_line_reader.h
 * @brief 纯 C 文本组帧：逐字节识别行边界，超长和超时后丢弃残行。
 * @details 接受 CR/LF/CRLF；发送端统一使用 CRLF，错误行的后缀不得被解释为新命令。
 */
#ifndef ROBOTARM_UART_LINE_READER_H
#define ROBOTARM_UART_LINE_READER_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define UART_LINE_CAPACITY 256U
typedef struct {
    char text[UART_LINE_CAPACITY];
    size_t length;
    uint32_t last_byte_ms;
    bool discarding;
    bool overflow;
    bool timed_out;
} UartLineReader_t;
void UartLineReader_Init(UartLineReader_t *reader);
/** 返回 true 时 out 包含一条完整非空行，且保证以 NUL 结束。 */
bool UartLineReader_Feed(UartLineReader_t *reader, uint8_t byte, uint32_t now_ms,
                         char *out, size_t capacity);
void UartLineReader_CheckTimeout(UartLineReader_t *reader, uint32_t now_ms, uint32_t timeout_ms);
/** DMA 覆盖等错误会破坏一行的完整性，必须进入丢弃状态。 */
void UartLineReader_Invalidate(UartLineReader_t *reader);
#endif
