/**
 * @file bsp_uart1.c
 * @brief UART1 传输：DMA 环形接收、中断发送、错误恢复与文本组帧适配。
 */

#include "bsp/bsp_uart1.h"
#include "bsp/bsp_uart1_config.h"
#include "bsp/uart_line_reader.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

typedef struct {
    uint8_t          buffer[BSP_UART1_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuf_t;

typedef struct {
    uint8_t          buffer[BSP_UART1_TX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} TxRingBuf_t;

static RingBuf_t s_rx    = {0};
static UartLineReader_t s_line;
static volatile bool s_rx_invalid = false;

static volatile uint32_t s_dma_wrap_count = 0U;
static uint32_t s_observed_wrap_count = 0U;
static volatile bool s_rx_overflow = false;
static bool s_discard_pending = false;
static volatile BspUartRxFault_t s_rx_fault;
static TxRingBuf_t s_tx = {0};
static volatile bool s_tx_dma_active = false;
static volatile uint16_t s_tx_dma_len = 0U;
static volatile bool s_tx_overflow = false;

static void RecordRxFault(uint32_t reason,uint32_t wraps,uint32_t observed,
                          uint32_t dma_pos,uint32_t produced,uint32_t unread)
{
    uint32_t mask=__get_PRIMASK();__disable_irq();
    if(!s_rx_fault.reason) {
        s_rx_fault.now_ms=HAL_GetTick();s_rx_fault.wraps=wraps;
        s_rx_fault.observed_wraps=observed;s_rx_fault.dma_pos=dma_pos;
        s_rx_fault.head=s_rx.head;s_rx_fault.tail=s_rx.tail;
        s_rx_fault.produced=produced;s_rx_fault.unread=unread;
        s_rx_fault.reason=reason; /* 内容先写，原因最后发布，保留首次现场。 */
    }
    __set_PRIMASK(mask);
}

static void StartRxDMA(void)
{
    (void)HAL_UART_Receive_DMA(&huart1, s_rx.buffer, BSP_UART1_RX_BUF_SIZE);
}

/* A UART error invalidates the DMA cursor and any partial line.  Reset every
   software cursor before restarting DMA so stale bytes cannot be parsed as a
   later G-code command. */
static void ResetRxAfterError(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    RecordRxFault(2U,s_dma_wrap_count,s_observed_wrap_count,0U,0U,0U);
    s_rx.head = 0U;
    s_rx.tail = 0U;
    s_dma_wrap_count = 0U;
    s_observed_wrap_count = 0U;
    s_rx_invalid = true;
    s_rx_overflow = true;
    s_discard_pending = false;
    __set_PRIMASK(primask);
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
                                      : (uint16_t)(BSP_UART1_TX_BUF_SIZE - s_tx.tail);
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
    size_t used = (s_tx.head + BSP_UART1_TX_BUF_SIZE - s_tx.tail) % BSP_UART1_TX_BUF_SIZE;
    if (length > BSP_UART1_TX_BUF_SIZE - 1U - used) {
        s_tx_overflow = true;
        __set_PRIMASK(primask);
        return;
    }
    for (size_t i = 0U; i < length; i++) {
        s_tx.buffer[s_tx.head] = data[i];
        s_tx.head = (uint16_t)((s_tx.head + 1U) % BSP_UART1_TX_BUF_SIZE);
    }
    __set_PRIMASK(primask);
    StartTxIT();
}

/* DMA 生产字节数与尚未消费字节数一起计算；仅比较本次新增字节会漏报覆盖。
   本函数只由主循环调用，DMA 完成 ISR 只增加圈数。 */
static bool RefreshRxHead(void)
{
    uint32_t wraps, observed;
    uint16_t dma_pos;
    do {
        wraps = s_dma_wrap_count;
        dma_pos = (uint16_t)(BSP_UART1_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
        observed = s_dma_wrap_count;
    } while (wraps != observed);
    if (dma_pos >= BSP_UART1_RX_BUF_SIZE) dma_pos = 0U;
    /* DMA硬件先重装NDTR，完成中断可能尚未发布圈数。此时位置变小不能直接
     * 做无符号减法，否则把少量新字节误算为约4GB覆盖。暂不推进游标，下一轮
     * 等ISR圈数一致后再读取；不自行补圈，避免ISR稍后到达时被重复计算。 */
    if(wraps==s_observed_wrap_count && dma_pos<s_rx.head)return false;
    uint32_t produced = (wraps - s_observed_wrap_count) * BSP_UART1_RX_BUF_SIZE
                      + dma_pos - s_rx.head;
    uint16_t unread = (uint16_t)((s_rx.head + BSP_UART1_RX_BUF_SIZE - s_rx.tail)
                                 % BSP_UART1_RX_BUF_SIZE);
    if (produced >= (uint32_t)(BSP_UART1_RX_BUF_SIZE - unread)) {
        RecordRxFault(1U,wraps,s_observed_wrap_count,dma_pos,produced,unread);
        s_rx.tail = dma_pos;
        s_rx_overflow = true;
        s_rx_invalid = true;
    }
    s_rx.head = dma_pos;
    s_observed_wrap_count = wraps;
    return true;
}

void BSP_UART1_Init(void)
{
    s_rx.head = 0U;
    s_rx.tail = 0U;
    UartLineReader_Init(&s_line);
    s_rx_invalid = false;
    s_dma_wrap_count = 0U;
    s_observed_wrap_count = 0U;
    s_rx_overflow = false;
    s_discard_pending=false;s_rx_fault=(BspUartRxFault_t){0};
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
    if (!line || max_len < 2U) return false;
    if(!RefreshRxHead())return false;
    if(s_discard_pending) {
        /* 回零交接曾要求丢弃旧输入，但当时快照未一致。现在统一丢到新head，
         * 防止中断稍后补圈时，把回零前残留运动命令重新解释成新命令。 */
        s_rx.tail=s_rx.head;UartLineReader_Init(&s_line);
        s_rx_invalid=false;s_rx_overflow=false;s_discard_pending=false;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool invalid = s_rx_invalid;
    s_rx_invalid = false;
    __set_PRIMASK(primask);
    if (invalid) UartLineReader_Invalidate(&s_line);
    UartLineReader_CheckTimeout(&s_line, HAL_GetTick(), BSP_UART1_LINE_TIMEOUT_MS);
    while (s_rx.tail != s_rx.head) {
        uint8_t byte = s_rx.buffer[s_rx.tail];
        s_rx.tail = (uint16_t)((s_rx.tail + 1U) % BSP_UART1_RX_BUF_SIZE);
        if (UartLineReader_Feed(&s_line, byte, HAL_GetTick(), line, max_len)) return true;
    }
    return false;
}

bool BSP_UART1_TakeLineTimeout(void)
{
    bool value = s_line.timed_out;
    s_line.timed_out = false;
    return value;
}

bool BSP_UART1_TakeRxOverflow(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool value = s_rx_overflow || s_line.overflow;
    if(s_line.overflow)RecordRxFault(3U,s_dma_wrap_count,s_observed_wrap_count,s_rx.head,0U,0U);
    s_rx_overflow = false;
    s_line.overflow = false;
    __set_PRIMASK(primask);
    return value;
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
    s_discard_pending=!RefreshRxHead();
    s_rx.tail = s_rx.head;
    UartLineReader_Init(&s_line);
    s_rx_invalid = false;
    s_rx_overflow = false;
}
void BSP_UART1_GetRxFault(BspUartRxFault_t *out)
{
    if(!out)return;
    uint32_t mask=__get_PRIMASK();__disable_irq();*out=s_rx_fault;__set_PRIMASK(mask);
}

void BSP_UART1_RxCallback(void)
{
    /* Circular DMA owns the buffer; this callback only refreshes its cursor. */
    /* DMA 环形接收由主循环读取；ISR 只维护圈数。 */
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
    s_tx.tail = (uint16_t)((s_tx.tail + s_tx_dma_len) % BSP_UART1_TX_BUF_SIZE);
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
        ResetRxAfterError();
        StartRxDMA();
    }
}

void BSP_UART1_SendString(const char *str)
{
    if (!str) return;
    QueueTx((const uint8_t *)str, strlen(str));
}

/* 标准输出重定向到 UART 发送队列。 */

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
