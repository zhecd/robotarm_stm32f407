#include "bsp_uart1.h"
#include <string.h>
#include <stdio.h>  // 为了 printf

extern UART_HandleTypeDef huart1;

typedef struct {
    uint8_t buffer[UART1_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} UartRingBuf_t;

static UartRingBuf_t rx_ring;
static uint8_t rx_temp_byte; // 单字节接收缓存

// 【新增】：记录最后一次收到数据的时间戳
static uint32_t last_rx_tick = 0; 

void BSP_UART1_Init(void) {
    rx_ring.head = 0;
    rx_ring.tail = 0;
    last_rx_tick = HAL_GetTick();
    // 开启 UART1 单字节中断接收
    HAL_UART_Receive_IT(&huart1, &rx_temp_byte, 1);
}

// 提取一行完整指令（以 \n 或 \r 结尾，或超过 50ms 没有新数据）
bool BSP_UART1_ReadLine(char* out_line, uint16_t max_len) {
    if (rx_ring.head == rx_ring.tail) return false; // 缓冲空

    uint16_t curr_tail = rx_ring.tail;
    uint16_t scan_idx = curr_tail;
    bool found_end = false;
    uint16_t msg_len = 0;

    // 1. 先扫描是否有显式的换行符
    while (scan_idx != rx_ring.head) {
        if (rx_ring.buffer[scan_idx] == '\n' || rx_ring.buffer[scan_idx] == '\r') {
            found_end = true;
            break;
        }
        scan_idx = (scan_idx + 1) % UART1_RX_BUF_SIZE;
        msg_len++;
    }

    // 2. 【核心修改】：如果没有换行符，使用你老代码的 50ms 超时判定
    if (!found_end) {
        if ((HAL_GetTick() - last_rx_tick) > 50) {
            found_end = true;
            scan_idx = rx_ring.head; // 把目前缓冲区里所有的东西都当成一行
        } else {
            return false; // 既没换行，也没到 50ms，继续等
        }
    }

    // 3. 提取这一行数据
    uint16_t i = 0;
    while (curr_tail != scan_idx && i < max_len - 1) {
        out_line[i++] = (char)rx_ring.buffer[curr_tail];
        curr_tail = (curr_tail + 1) % UART1_RX_BUF_SIZE;
    }
    out_line[i] = '\0'; // 字符串结束符

    // 4. 跳过换行符本身 (\r 或 \n)，防止下一次进来读出空行
    while (curr_tail != rx_ring.head && 
          (rx_ring.buffer[curr_tail] == '\n' || rx_ring.buffer[curr_tail] == '\r')) {
        curr_tail = (curr_tail + 1) % UART1_RX_BUF_SIZE;
    }
    
    rx_ring.tail = curr_tail; // 更新尾指针
    return true;
}

// 串口中断回调：负责把数据塞进环形缓冲区
void BSP_UART1_RxCallback(void) {
    uint16_t next_head = (rx_ring.head + 1) % UART1_RX_BUF_SIZE;
    if (next_head != rx_ring.tail) {
        rx_ring.buffer[rx_ring.head] = rx_temp_byte;
        rx_ring.head = next_head;
        
        // 【新增】：每次收到新字符，刷新时间戳
        last_rx_tick = HAL_GetTick(); 
    }
    // 重新开启中断
    HAL_UART_Receive_IT(&huart1, &rx_temp_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    if (huart->Instance == USART1) 
    {
        BSP_UART1_RxCallback();
    }
}

// 【防死机】：串口溢出错误恢复
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart); 
        HAL_UART_Receive_IT(&huart1, &rx_temp_byte, 1);
    }
}

void BSP_UART1_SendString(char* str) {
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/* ==================================================================== */
/* printf 重定向底层代码                          */
/* ==================================================================== */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}