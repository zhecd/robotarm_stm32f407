#include "bsp_uart1.h"
#include <stdio.h>
#include <string.h>

#define UART1_LINE_TIMEOUT_MS 50U

extern UART_HandleTypeDef huart1;

typedef struct
{
    uint8_t buffer[UART1_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} UartRingBuffer_t;

static UartRingBuffer_t s_rx_ring = {0};
static uint8_t s_rx_temp_byte = 0U;
static uint32_t s_last_rx_tick = 0U;

static void UART1_StartReceiveIT(void)
{
    HAL_UART_Receive_IT(&huart1, &s_rx_temp_byte, 1U);
}

void BSP_UART1_Init(void)
{
    s_rx_ring.head = 0U;
    s_rx_ring.tail = 0U;
    s_last_rx_tick = HAL_GetTick();
    UART1_StartReceiveIT();
}

bool BSP_UART1_ReadLine(char *out_line, uint16_t max_len)
{
    if ((out_line == NULL) || (max_len < 2U) || (s_rx_ring.head == s_rx_ring.tail)) {
        return false;
    }

    uint16_t read_index = s_rx_ring.tail;
    uint16_t scan_index = read_index;
    bool found_line_end = false;

    while (scan_index != s_rx_ring.head) {
        uint8_t ch = s_rx_ring.buffer[scan_index];
        if ((ch == '\n') || (ch == '\r')) {
            found_line_end = true;
            break;
        }
        scan_index = (scan_index + 1U) % UART1_RX_BUF_SIZE;
    }

    if (!found_line_end) {
        if ((HAL_GetTick() - s_last_rx_tick) <= UART1_LINE_TIMEOUT_MS) {
            return false;
        }
        scan_index = s_rx_ring.head;
    }

    uint16_t out_index = 0U;
    while ((read_index != scan_index) && (out_index < (uint16_t)(max_len - 1U))) {
        out_line[out_index++] = (char)s_rx_ring.buffer[read_index];
        read_index = (read_index + 1U) % UART1_RX_BUF_SIZE;
    }
    out_line[out_index] = '\0';

    while ((read_index != s_rx_ring.head) &&
           ((s_rx_ring.buffer[read_index] == '\n') || (s_rx_ring.buffer[read_index] == '\r')))
    {
        read_index = (read_index + 1U) % UART1_RX_BUF_SIZE;
    }

    s_rx_ring.tail = read_index;
    return true;
}

void BSP_UART1_RxCallback(void)
{
    uint16_t next_head = (s_rx_ring.head + 1U) % UART1_RX_BUF_SIZE;
    if (next_head != s_rx_ring.tail) {
        s_rx_ring.buffer[s_rx_ring.head] = s_rx_temp_byte;
        s_rx_ring.head = next_head;
        s_last_rx_tick = HAL_GetTick();
    }

    UART1_StartReceiveIT();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        BSP_UART1_RxCallback();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        UART1_StartReceiveIT();
    }
}

void BSP_UART1_SendString(const char *str)
{
    if (str == NULL) {
        return;
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1U, HAL_MAX_DELAY);
    return ch;
}
