/**
 * @file uart_line_reader.c
 * @brief 纯 C 文本组帧：逐字节识别行边界，超长和超时后丢弃残行。
 * @details 接受 CR/LF/CRLF；发送端统一使用 CRLF，错误行的后缀不得被解释为新命令。
 */
#include "bsp/uart_line_reader.h"
#include <string.h>
void UartLineReader_Init(UartLineReader_t *r)
{
    if (r) memset(r, 0, sizeof(*r));
}
void UartLineReader_Invalidate(UartLineReader_t *r)
{
    if (!r) return;
    r->length = 0U;
    r->discarding = true;
    r->overflow = true;
}
void UartLineReader_CheckTimeout(UartLineReader_t *r, uint32_t now, uint32_t timeout)
{
    if (r && r->length > 0U && (uint32_t)(now-r->last_byte_ms) > timeout) {
        r->length = 0U;
        r->discarding = true;
        r->timed_out = true;
    }
}
bool UartLineReader_Feed(UartLineReader_t *r, uint8_t byte, uint32_t now,
                         char *out, size_t capacity)
{
    if (!r || !out || capacity < 2U) return false;
    r->last_byte_ms = now;
    if (byte == '\r' || byte == '\n') {
        if (r->discarding) {
            r->discarding = false;
            r->length = 0;
            return false;
        }
        if (r->length == 0U) return false; /* 吸收空行及 CRLF 中的第二个字节。 */
        if (r->length >= capacity) {
            r->length = 0;
            r->overflow = true;
            return false;
        }
        memcpy(out, r->text, r->length);
        out[r->length] = '\0';
        r->length = 0;
        return true;
    }
    if (r->discarding) return false;
    /* 只接受可打印 ASCII 和制表符，拒绝 NUL 等控制字符造成的截断命令。 */
    if ((byte < 32U && byte != '\t') || byte > 126U || r->length >= UART_LINE_CAPACITY-1U) {
        UartLineReader_Invalidate(r);
        return false;
    }
    r->text[r->length++] = (char)byte;
    return false;
}
