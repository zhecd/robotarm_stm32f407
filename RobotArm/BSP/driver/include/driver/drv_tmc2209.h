/**
 * @file drv_tmc2209.h
 * @brief TMC2209 驱动：配置发送与寄存器读取，调用者须区分发送成功和实际读回。
 */

#ifndef __Drv_TMC2209_H__
#define __Drv_TMC2209_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/** 单次读请求的结果；数值同时用于M123诊断协议，不包含实际电流判断。 */
typedef enum {
    TMC_READ_OK=0,
    TMC_READ_ARGUMENT=1,
    TMC_READ_RX_DISABLED=2,
    TMC_READ_TX_FAILED=3,
    TMC_READ_RX_TIMEOUT=4,
    TMC_READ_RX_FAILED=5,
    TMC_READ_FRAME=6,
    TMC_READ_CRC=7
} TmcReadResult_t;

/** 配置确认结果；读错误保留在报告中，不用共享的“上一次错误”变量。 */
typedef enum {
    TMC_CONFIG_OK=0,
    TMC_CONFIG_ARGUMENT,
    TMC_CONFIG_READ_FAILED,
    TMC_CONFIG_TX_FAILED,
    TMC_CONFIG_COUNTER_MISMATCH,
    TMC_CONFIG_VALUE_MISMATCH
} TmcConfigResult_t;

/** 电流参考源，不是采样电阻选择。两种模式都保留外部Rsense。
 * 默认兼容接口使用VREF；内部参考模式由IRUN/IHOLD及Rsense确定标称电流，
 * 忽略电位器VREF缩放，但不代表已测量实际绕组电流。
 */
typedef enum {
    TMC_CURRENT_EXTERNAL_VREF=0,
    TMC_CURRENT_INTERNAL_REF=1
} TmcCurrentReference_t;

typedef struct {
    TmcConfigResult_t result;
    TmcReadResult_t read_error;
    uint8_t failed_register; /* 0xFF表示参数错误、写发送失败或没有失败寄存器。 */
    uint8_t ifcnt_before, ifcnt_after;
    uint32_t expected, observed; /* 读回比较使用掩码后的值。 */
    uint32_t gconf_observed, chopconf_observed; /* 本次真实读回；未读到时为0。 */
    uint32_t pwmconf_observed, ioin_observed; /* 静音确认接口的真实读回，非自动调谐完成证明。 */
} TmcConfigReport_t;

#ifdef __cplusplus
extern "C" {
#endif

/** 主循环单次有界读；不重试、不写配置，失败保持*out不变，并恢复接收器。 */
TmcReadResult_t Drv_TMC2209_ReadRegisterResult(UART_HandleTypeDef *huart,
                                             uint8_t node, uint8_t reg, uint32_t *out);

/** 兼容原总线初始化：尝试向四个地址发送配置，不作芯片接受确认。 */
void Drv_TMC2209_InitBus(UART_HandleTypeDef *huart);

/** 兼容的无返回值配置入口；正常启动使用Verified接口。 */
void Drv_TMC2209_ConfigNode(UART_HandleTypeDef *huart, uint8_t node_addr,
                            uint16_t microsteps, uint8_t irun, uint8_t ihold);
/** 主循环阻塞调用：校验参数，依次发送NODECONF、GCONF、CHOPCONF、IHOLD_IRUN。
 * NODECONF设置SENDDELAY=2，满足共享总线的多节点回复切换要求；首个发送失败即返回false。
 * irun/ihold为0..31寄存器编码，不是安培。true仅表示UART发送成功，未证明芯片接受。
 */
bool Drv_TMC2209_ConfigNodeChecked(UART_HandleTypeDef *huart, uint8_t node_addr,
                                 uint16_t microsteps, uint8_t irun, uint8_t ihold);
/** 主循环独占总线、执行器禁用时调用；无自动重试，每次HAL访问有超时。
 * 先读IFCNT，发送四项配置，再检查模256写入计数及GCONF/CHOPCONF读回。
 * NODECONF、IHOLD_IRUN为只写寄存器，其接收通过IFCNT间接确认，不能回读。
 * true仅说明配置被芯片接受，不代表实际相电流或机械动作已经验收。
 * report可为NULL；函数不操作EN/STEP，也不写速度、OTP等寄存器。
 */
bool Drv_TMC2209_ConfigNodeVerified(UART_HandleTypeDef *huart, uint8_t node,
                                   uint16_t microsteps, uint8_t irun, uint8_t ihold,
                                   TmcConfigReport_t *report);
/** 显式选择参考源的确认接口，其余合同同Verified。非法模式零总线访问。
 * INTERNAL_REF清GCONF.i_scale_analog；internal_Rsense仍为0，继续用外部电阻。
 * 只允许调用者在EN禁用时选择，驱动不使能电机、不修改电位器/OTP。
 * 原Verified/Checked/ConfigNode入口继续使用EXTERNAL_VREF，保持正常固件行为。
 */
bool Drv_TMC2209_ConfigNodeVerifiedWithReference(UART_HandleTypeDef *huart,uint8_t node,
    uint16_t microsteps,uint8_t irun,uint8_t ihold,TmcCurrentReference_t reference,
    TmcConfigReport_t *report);
/** 正常配置成功后、EN禁用且电机静止时调用。UART显式设置StealthChop2：
 * 读取IOIN.SPREAD_EN以补偿硬件极性，TPWMTHRS=0禁止按速度切回SpreadCycle，
 * PWMCONF启用自动缩放/梯度，PWM频率选01（内部12MHz时约35kHz）。
 * 保留原电流参考/采样电阻/细分/方向/电流值，不写OTP或速度、不输出STEP。
 * 独立IFCNT模256差值必须为3；GCONF/PWMCONF读回匹配且SPREAD电平不变才成功。
 * TPWMTHRS只写，其接收由IFCNT确认，不能把0读回当作配置证明。 */
bool Drv_TMC2209_ConfigureQuietVerified(UART_HandleTypeDef *huart,uint8_t node,
                                      TmcConfigReport_t *report);
/** 主循环独占USART调用；只发4字节读请求，不写驱动寄存器、不自动重试。
 * 抑制本机发送回声，验证8字节回复的地址、寄存器及CRC；失败时不修改*out。
 * 节点地址0..3不等于机械轴编号；调用前必须禁止并发UART发送/接收。
 */
bool Drv_TMC2209_ReadRegister(UART_HandleTypeDef *huart, uint8_t node_addr,
                             uint8_t reg, uint32_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __Drv_TMC2209_H__ */
