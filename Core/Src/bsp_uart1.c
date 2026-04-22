#include "bsp_uart1.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

typedef struct {
    uint8_t buffer[UART1_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} UartRingBuf_t;

static UartRingBuf_t rx_ring;
static uint8_t rx_temp_byte; // 单字节接收缓存

void BSP_UART1_Init(void) {
    rx_ring.head = 0;
    rx_ring.tail = 0;
    // 开启 UART1 单字节中断接收
    HAL_UART_Receive_IT(&huart1, &rx_temp_byte, 1);
}

// 提取一行完整指令（以 \n 或 \r 结尾）
bool BSP_UART1_ReadLine(char* out_line, uint16_t max_len) {
    if (rx_ring.head == rx_ring.tail) return false; // 缓冲空

    uint16_t curr_tail = rx_ring.tail;
    uint16_t scan_idx = curr_tail;
    bool found_newline = false;
    uint16_t msg_len = 0;

    // 扫描是否有换行符
    while (scan_idx != rx_ring.head) {
        if (rx_ring.buffer[scan_idx] == '\n' || rx_ring.buffer[scan_idx] == '\r') {
            found_newline = true;
            break;
        }
        scan_idx = (scan_idx + 1) % UART1_RX_BUF_SIZE;
        msg_len++;
    }

    if (!found_newline) return false; // 还没有收到完整的一行

    // 提取这一行数据
    uint16_t i = 0;
    while (curr_tail != scan_idx && i < max_len - 1) {
        out_line[i++] = (char)rx_ring.buffer[curr_tail];
        curr_tail = (curr_tail + 1) % UART1_RX_BUF_SIZE;
    }
    out_line[i] = '\0'; // 字符串结束符

    // 跳过换行符本身 (\r 或 \n)
    while (curr_tail != rx_ring.head && 
          (rx_ring.buffer[curr_tail] == '\n' || rx_ring.buffer[curr_tail] == '\r')) {
        curr_tail = (curr_tail + 1) % UART1_RX_BUF_SIZE;
    }
    
    rx_ring.tail = curr_tail; // 更新尾指针
    return true;
}

// 在 HAL_UART_RxCpltCallback 中调用此函数
void BSP_UART1_RxCallback(void) {
    uint16_t next_head = (rx_ring.head + 1) % UART1_RX_BUF_SIZE;
    if (next_head != rx_ring.tail) {
        rx_ring.buffer[rx_ring.head] = rx_temp_byte;
        rx_ring.head = next_head;
    }
    // 重新开启中断
    HAL_UART_Receive_IT(&huart1, &rx_temp_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    // 判断是不是串口 1 触发的中断
    if (huart->Instance == USART1) 
    {
        // 调用我们自己写的环形缓冲接收逻辑
        BSP_UART1_RxCallback();
    }
    
    // 如果你以后还有串口 2、串口 3 的接收，也可以在这里继续加 if
    // else if (huart->Instance == USART2) { ... }
}