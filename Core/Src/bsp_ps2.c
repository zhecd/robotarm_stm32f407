#include "bsp_ps2.h"

// 纳秒级微小延时，用于模拟 SPI 时序
static void PS2_Delay(void) {
    for (volatile uint32_t i = 0; i < 300; i++) {
        __NOP();
    }
}

// 软件 SPI 发送/接收一个字节
static uint8_t PS2_TransferByte(uint8_t cmd) {
    uint8_t data = 0;
    for (uint16_t i = 0x01; i < 0x100; i <<= 1) {
        // 设置 CMD (MOSI) 引脚电平
        if (i & cmd) {
            HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_RESET);
        }
        
        // 拉低时钟，PS2 接收器在下降沿读取我们的 CMD
        HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET);
        PS2_Delay();
        
        // 读取 DAT (MISO) 引脚电平
        if (HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port, PS2_DAT_Pin) == GPIO_PIN_SET) {
            data |= i;
        }
        
        // 拉高时钟
        HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
        PS2_Delay();
    }
    return data;
}

void BSP_PS2_Init(void) {
    // 初始状态拉高时钟和片选
    HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET);
}

// 读取手柄数据
bool BSP_PS2_ReadData(PS2_Data_t *ps2_data) {
    uint8_t raw[9] = {0};
    
    // 拉低片选，开始通信
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET);
    PS2_Delay();
    
    // 发送请求读取指令
    raw[0] = PS2_TransferByte(0x01); // Start 命令
    raw[1] = PS2_TransferByte(0x42); // 接收模式 ID (0x41绿灯, 0x73/0x79红灯)
    raw[2] = PS2_TransferByte(0x00); // 接收就绪码 (必须是0x5A)
    
    // 修复 Bug 1：使用标准就绪码 0x5A 校验，兼容所有手柄
    if (raw[2] == 0x5A) {
        raw[3] = PS2_TransferByte(0x00); // Buttons 1 (包含 Select)
        raw[4] = PS2_TransferByte(0x00); // Buttons 2 (包含 L2)
        raw[5] = PS2_TransferByte(0x00); // Right Joy X
        raw[6] = PS2_TransferByte(0x00); // Right Joy Y
        raw[7] = PS2_TransferByte(0x00); // Left Joy X
        raw[8] = PS2_TransferByte(0x00); // Left Joy Y
        
        // 拉高片选，结束通信
        HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
        
        // 修复 Bug 2：颠倒 raw[3] 和 raw[4] 以匹配 bsp_ps2.h 的宏定义
        ps2_data->buttons = (raw[3] << 8) | raw[4];
        
        ps2_data->RX = raw[5];
        ps2_data->RY = raw[6];
        ps2_data->LX = raw[7];
        ps2_data->LY = raw[8];
        return true;
    }
    
    // 如果没有检测到手柄
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
    return false;
}