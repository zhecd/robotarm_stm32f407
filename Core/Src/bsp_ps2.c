#include "bsp_ps2.h"
#include "main.h"

// 【修改点 1：大幅增加底层延时】
// 将 delay 从 20 增加到 400，让单步延时达到约 4~5 微秒
// 这样 SPI 频率就降到了约 100kHz，能兼容所有最差的低速接收器
static void PS2_Delay(void) {
    uint32_t delay = 400; 
    while(delay--) {
        __NOP();
    }
}

// 软件模拟 SPI 收发一个字节 (LSB First)
static uint8_t PS2_TransferByte(uint8_t tx_data) {
    uint8_t rx_data = 0;
    for (int i = 0; i < 8; i++) {
        // 1. 准备数据位 (CMD)
        if (tx_data & (1 << i)) {
            HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_RESET);
        }
        
        // 2. 拉低时钟，手柄在此刻改变 DAT 数据线
        HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET);
        PS2_Delay();
        
        // 3. 读取 DAT 数据线
        if (HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port, PS2_DAT_Pin) == GPIO_PIN_SET) {
            rx_data |= (1 << i);
        }
        
        // 4. 拉高时钟，完成一个 bit
        HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
        PS2_Delay();
    }
    
    // 【修改点 2：字节间增加喘息时间】
    // 发送完一个字节后，多等一会儿，防止接收器缓存溢出
    PS2_Delay(); 
    PS2_Delay(); 
    
    return rx_data;
}

static void PS2_SendCommand(const uint8_t *cmd, uint8_t len) {
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET);
    PS2_Delay();
    for (uint8_t i = 0; i < len; i++) {
        PS2_TransferByte(cmd[i]);
    }
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(16); 
}

void BSP_PS2_Init(void) {
    // 强行开启硬件内部上拉电阻
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = PS2_DAT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; 
    HAL_GPIO_Init(PS2_DAT_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET);

    HAL_Delay(50); 

    // 强制配置为红灯（模拟）模式并锁定
    const uint8_t enter_config[] = {0x01, 0x43, 0x00, 0x01, 0x00};
    const uint8_t set_mode[]     = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    const uint8_t exit_config[]  = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

    PS2_SendCommand(enter_config, 5);
    PS2_SendCommand(set_mode, 9);
    PS2_SendCommand(exit_config, 9);
}

// 读取一帧手柄数据
bool BSP_PS2_ReadData(PS2_Data_t *ps2_data) {
    uint8_t raw[9] = {0};
    
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET);
    
    // 【修改点 3：片选拉低后多等一会儿再发时钟】
    PS2_Delay();
    PS2_Delay();
    
    raw[0] = PS2_TransferByte(0x01); 
    raw[1] = PS2_TransferByte(0x42); 
    raw[2] = PS2_TransferByte(0x00); 
    
    // 【调试探针】：如果还是连不上，把下面这行的注释取消，看串口打印的是什么！
    // printf("RAW: %02X %02X %02X\r\n", raw[0], raw[1], raw[2]);
    
    if (raw[2] == 0x5A) {
        raw[3] = PS2_TransferByte(0x00); 
        raw[4] = PS2_TransferByte(0x00); 
        raw[5] = PS2_TransferByte(0x00); 
        raw[6] = PS2_TransferByte(0x00); 
        raw[7] = PS2_TransferByte(0x00); 
        raw[8] = PS2_TransferByte(0x00); 
        
        HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
        
        ps2_data->buttons = (raw[3] << 8) | raw[4];
        ps2_data->RX = raw[5];
        ps2_data->RY = raw[6];
        ps2_data->LX = raw[7];
        ps2_data->LY = raw[8];
        
        return true; 
    }
    
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
    return false; 
}