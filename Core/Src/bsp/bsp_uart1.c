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

typedef struct {
    uint8_t          buffer[UART1_TX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} TxRingBuf_t;

static RingBuf_t s_rx    = {0};
static uint32_t  s_last_rx_tick = 0U;
static volatile bool s_line_timeout = false;
static volatile uint32_t s_dma_wrap_count = 0U;
static uint32_t s_observed_wrap_count = 0U;
static volatile bool s_rx_overflow = false;
static TxRingBuf_t s_tx = {0};
static volatile bool s_tx_dma_active = false;
static volatile uint16_t s_tx_dma_len = 0U;
static volatile bool s_tx_overflow = false;

static void StartRxDMA(void)
{
    (void)HAL_UART_Receive_DMA(&huart1, s_rx.buffer, UART1_RX_BUF_SIZE);
}

static void StartTxIT(void)
{
    uint8_t *data;
    uint16_t length;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_tx_dma_active || s_tx.head == s_tx.tail) {
        __set_PRIMASK(primask);
        return;
    }

    data = &s_tx.buffer[s_tx.tail];
    length = (s_tx.head > s_tx.tail) ? (uint16_t)(s_tx.head - s_tx.tail)
                                      : (uint16_t)(UART1_TX_BUF_SIZE - s_tx.tail);
    s_tx_dma_active = true;
    s_tx_dma_len = length;
    __set_PRIMASK(primask);

    if (HAL_UART_Transmit_IT(&huart1, data, length) != HAL_OK) {
        primask = __get_PRIMASK();
        __disable_irq();
        s_tx_dma_active = false;
        s_tx_dma_len = 0U;
        s_tx_overflow = true;
        __set_PRIMASK(primask);
    }
}

static void QueueTx(const uint8_t *data, size_t length)
{
    if (!data || length == 0U) return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (size_t i = 0U; i < length; i++) {
        uint16_t next = (uint16_t)((s_tx.head + 1U) % UART1_TX_BUF_SIZE);
        if (next == s_tx.tail) {
            s_tx_overflow = true;
            break;
        }
        s_tx.buffer[s_tx.head] = data[i];
        s_tx.head = next;
    }
    __set_PRIMASK(primask);
    StartTxIT();
}

static void RefreshRxHead(void)
{
    uint16_t previous_pos = s_rx.head;
    uint16_t dma_pos = (uint16_t)(UART1_RX_BUF_SIZE -
                                  __HAL_DMA_GET_COUNTER(huart1.hdmarx));
    if (dma_pos >= UART1_RX_BUF_SIZE)
        dma_pos = 0U;

    uint32_t wraps = s_dma_wrap_count;
    int32_t produced = (int32_t)((wraps - s_observed_wrap_count) * UART1_RX_BUF_SIZE)
                     + (int32_t)dma_pos - (int32_t)previous_pos;

    if (produced >= (int32_t)UART1_RX_BUF_SIZE) {
        s_rx.tail = dma_pos;
        s_rx_overflow = true;
        s_line_timeout = false;
    }
    if (produced > 0) {
        s_last_rx_tick = HAL_GetTick();
    }
    s_rx.head = dma_pos;
    s_observed_wrap_count = wraps;
}

void BSP_UART1_Init(void)
{
    s_rx.head = 0U;
    s_rx.tail = 0U;
    s_last_rx_tick = HAL_GetTick();
    s_line_timeout = false;
    s_dma_wrap_count = 0U;
    s_observed_wrap_count = 0U;
    s_rx_overflow = false;
    s_tx.head = 0U;
    s_tx.tail = 0U;
    s_tx_dma_active = false;
    s_tx_dma_len = 0U;
    s_tx_overflow = false;
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

bool BSP_UART1_TakeRxOverflow(void)
{
    bool overflow = s_rx_overflow;
    s_rx_overflow = false;
    return overflow;
}

bool BSP_UART1_TakeTxOverflow(void)
{
    bool overflow = s_tx_overflow;
    s_tx_overflow = false;
    return overflow;
}

bool BSP_UART1_FlushTx(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    do {
        StartTxIT();
        if (!s_tx_dma_active && s_tx.head == s_tx.tail)
            return true;
    } while ((HAL_GetTick() - start) < timeout_ms);
    return false;
}

void BSP_UART1_DiscardRx(void)
{
    RefreshRxHead();
    s_rx.tail = s_rx.head;
    s_line_timeout = false;
    s_rx_overflow = false;
}

void BSP_UART1_RxCallback(void)
{
    /* Circular DMA owns the buffer; this callback only refreshes its cursor. */
    RefreshRxHead();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        s_dma_wrap_count++;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_tx.tail = (uint16_t)((s_tx.tail + s_tx_dma_len) % UART1_TX_BUF_SIZE);
    s_tx_dma_len = 0U;
    s_tx_dma_active = false;
    __set_PRIMASK(primask);
    StartTxIT();
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
    QueueTx((const uint8_t *)str, strlen(str));
}

/* ── printf retarget / printf 重定向 ── */

#ifdef __GNUC__
#define PUTCHAR_PROTO int __io_putchar(int ch)
#else
#define PUTCHAR_PROTO int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTO
{
    uint8_t byte = (uint8_t)ch;
    QueueTx(&byte, 1U);
    return ch;
}
