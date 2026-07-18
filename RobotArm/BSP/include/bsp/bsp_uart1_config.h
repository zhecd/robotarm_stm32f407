#ifndef ROBOTARM_BSP_UART1_CONFIG_H
#define ROBOTARM_BSP_UART1_CONFIG_H

/* Board communication resources, kept out of robot kinematics configuration. */
#ifndef BSP_UART1_RX_BUF_SIZE
#define BSP_UART1_RX_BUF_SIZE 256U
#endif

#ifndef BSP_UART1_TX_BUF_SIZE
#define BSP_UART1_TX_BUF_SIZE 1024U
#endif

#ifndef BSP_UART1_LINE_TIMEOUT_MS
#define BSP_UART1_LINE_TIMEOUT_MS 500U
#endif

#endif /* ROBOTARM_BSP_UART1_CONFIG_H */
