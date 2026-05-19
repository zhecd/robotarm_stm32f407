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
static uint8_t   s_rx_byte   = 0U;
static uint32_t  s_last_rx_tick = 0U;

static void StartRxIT(void)
{
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

void BSP_UART1_Init(void)
{
    s_rx.head = 0U;
    s_rx.tail = 0U;
    s_last_rx_tick = HAL_GetTick();
    StartRxIT();
}

bool BSP_UART1_ReadLine(char *line, uint16_t max_len)
{
    if (!line || max_len < 2U || s_rx.head == s_rx.tail)
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
        if ((HAL_GetTick() - s_last_rx_tick) <= UART1_LINE_TIMEOUT_MS)
            return false;
        scan_idx = s_rx.head;
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

void BSP_UART1_RxCallback(void)
{
    uint16_t next = (s_rx.head + 1U) % UART1_RX_BUF_SIZE;
    if (next != s_rx.tail) {
        s_rx.buffer[s_rx.head] = s_rx_byte;
        s_rx.head = next;
        s_last_rx_tick = HAL_GetTick();
    }
    StartRxIT();
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
        StartRxIT();
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
