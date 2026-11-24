/** @file ps2_bus.h
 * @brief PS2字节传输端口；配置状态机不依赖GPIO，电脑测试只替换本端口。
 * 仅供BSP驱动内部使用。Exchange同步交换一帧，长度单位字节，不等待帧间间隔。
 */
#ifndef ROBOTARM_PS2_BUS_H
#define ROBOTARM_PS2_BUS_H
#include <stdint.h>
void Ps2Bus_Init(void);
void Ps2Bus_Exchange(const uint8_t *tx, uint8_t *rx, uint8_t length);
#endif
