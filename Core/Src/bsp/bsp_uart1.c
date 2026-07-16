/**
 * @file    bsp_uart1.c
 * @brief   UART1 ring-buffer line reader and printf retarget. / UART1 环形缓冲区行读取与 printf 重定向。
 * @ingroup bsp
 */

#include "bsp/bsp_uart1.h"
#include "robot_config.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

typedef struct {
    uint8_t          buffer[UART1_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuf_t;

static RingBuf_t s_rx    = {0};
static uint32_t  s_last_rx_tick = 0U;
static volatile bool s_line_timeout = false;

static void StartRxDMA(void)
{
    (void)HAL_UART_Receive_DMA(&huart1, s_rx.buffer, UART1_RX_BUF_SIZE);
}

static void RefreshRxHead(void)
{
    uint16_t dma_pos = (uint16_t)(UART1_RX_BUF_SIZE -
                                  __HAL_DMA_GET_COUNTER(huart1.hdmarx));
    if (dma_pos >= UART1_RX_BUF_SIZE)
        dma_pos = 0U;

    if (dma_pos != s_rx.head) {
        s_rx.head = dma_pos;
        s_last_rx_tick = HAL_GetTick();
    }
}

void BSP_UART1_Init(void)
{
    s_rx.head = 0U;
    s_rx.tail = 0U;
    s_last_rx_tick = HAL_GetTick();
    s_line_timeout = false;
    (void)HAL_UART_AbortReceive(&huart1);
    StartRxDMA();
}

bool BSP_UART1_ReadLine(char *line, uint16_t max_len)
{
    RefreshRxHead();
    if (!line || max_len < 2U || s_rx.head == s_rx.tail)
        return false;

    /* A terminal commonly sends CRLF.  If CR is consumed before the UART
       interrupt receives LF, the LF remains in the ring buffer and would be
       parsed as a spurious empty command on the next call. */
    while (s_rx.tail != s_rx.head &&
           (s_rx.buffer[s_rx.tail] == '\r' || s_rx.buffer[s_rx.tail] == '\n')) {
        s_rx.tail = (s_rx.tail + 1U) % UART1_RX_BUF_SIZE;
    }
    if (s_rx.head == s_rx.tail)
        return false;

    uint16_t read_idx  = s_rx.tail;
    uint16_t scan_idx  = read_idx;
    bool     found_end = false;

    while (scan_idx != s_rx.head) {
        uint8_t ch = s_rx.buffer[scan_idx];
        if (ch == '\n' || ch == '\r') { found_end = true; break; }
        scan_idx = (scan_idx + 1U) % UART1_RX_BUF_SIZE;
    }

    if (!found_end) {
        /* Strict protocol: only CR/LF-terminated lines are commands. */
        if ((HAL_GetTick() - s_last_rx_tick) > UART1_LINE_TIMEOUT_MS) {
            s_rx.tail = s_rx.head;
            s_line_timeout = true;
        }
        return false;
    }

    uint16_t out_idx = 0U;
    while (read_idx != scan_idx && out_idx < (uint16_t)(max_len - 1U)) {
        line[out_idx++] = (char)s_rx.buffer[read_idx];
        read_idx = (read_idx + 1U) % UART1_RX_BUF_SIZE;
    }
    line[out_idx] = '\0';

    while (read_idx != s_rx.head &&
           (s_rx.buffer[read_idx] == '\n' || s_rx.buffer[read_idx] == '\r')) {
        read_idx = (read_idx + 1U) % UART1_RX_BUF_SIZE;
    }

    s_rx.tail = read_idx;
    return true;
}

bool BSP_UART1_TakeLineTimeout(void)
{
    bool timed_out = s_line_timeout;
    s_line_timeout = false;
    return timed_out;
}

void BSP_UART1_RxCallback(void)
{
    /* Circular DMA owns the buffer; this callback only refreshes its cursor. */
    RefreshRxHead();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        BSP_UART1_RxCallback();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)HAL_UART_AbortReceive(huart);
        StartRxDMA();
    }
}

void BSP_UART1_SendString(const char *str)
{
    if (!str) return;
    HAL_UART_Transmit(&huart1, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

/* ── printf retarget / printf 重定向 ── */

#ifdef __GNUC__
#define PUTCHAR_PROTO int __io_putchar(int ch)
#else
#define PUTCHAR_PROTO int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTO
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1U, HAL_MAX_DELAY);
    return ch;
}
