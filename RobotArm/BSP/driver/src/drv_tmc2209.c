/**
 * @file drv_tmc2209.c
 * @brief TMC2209 驱动：带参数检查的配置发送、只读寄存器请求与回复CRC验证。
 */

#include "driver/drv_tmc2209.h"
#include "bsp_driver_config.h"
#include <stdio.h>

#define TMC2209_REG_GCONF             0x00U
#define TMC2209_REG_IFCNT             0x02U
#define TMC2209_REG_NODECONF          0x03U
#define TMC2209_REG_IHOLD_IRUN        0x10U
#define TMC2209_REG_CHOPCONF          0x6CU
#define TMC2209_REG_IOIN              0x06U
#define TMC2209_REG_TPWMTHRS          0x13U
#define TMC2209_REG_PWMCONF           0x70U
#define TMC2209_WRITE_SYNC            0x05U
#define TMC2209_UART_WRITE_FLAG       0x80U
#define TMC2209_GCONF_MSTEP_REG_SELECT (1UL << 7)
#define TMC2209_GCONF_PDN_DISABLE    (1UL << 6)
#define TMC2209_GCONF_MULTISTEP_FILT  (1UL << 8)
#define TMC2209_GCONF_I_SCALE_ANALOG  (1UL << 0)
#define TMC2209_CHOPCONF_BASE         0x10000053UL
#define TMC2209_CHOPCONF_INTPOL       (1UL << 28)
#define TMC2209_CHOPCONF_MRES_SHIFT   24U
#define TMC2209_IHOLDDELAY_SHIFT      16U
#define TMC2209_IRUN_SHIFT            8U
#define TMC2209_CURRENT_MASK          0x1FU
#define TMC2209_WRITE_PACKET_SIZE     8U
#define TMC2209_UART_TX_TIMEOUT_MS    100U
#define TMC2209_WRITE_SETTLE_MS       5U
/* 初始化/查询均为主循环的有限阻塞调用。Rev1.09 §4.1.2规定回复结束后
 * 节点还会驱动总线4个bit；HAL_UART_Receive返回不等于节点已释放总线。
 * 在每次新的请求前统一留2ms空闲，覆盖115200下约35us释放期和错误恢复。
 * 不依赖Debug/Release指令执行快慢，不以放宽IFCNT计数代替写入确认。 */
#define TMC2209_BUS_IDLE_MS           2U
/* 三个地址共用PDN_UART；Rev1.09第4.1.2节要求SENDDELAY至少为2。
 * 编码2对应24个bit时间，先给未寻址节点留出回复切换时间。
 * 这里只配置通信时序，不改变节点地址、电流或EN；上电复位后需重新发送。
 */
#define TMC2209_NODECONF_SENDDELAY     (2UL << 8)

/* CRC8 校验计算。 */

static uint8_t CalcCRC8(const uint8_t *datagram, uint8_t len)
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < len; i++) {
        uint8_t byte = datagram[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((uint8_t)((crc >> 7) ^ (byte & 0x01U)) != 0U)
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            else
                crc <<= 1;
            byte >>= 1;
        }
    }
    return crc;
}

/* 将微步数转换为 MRES 寄存器值。 */

static bool MicrostepsToMres(uint16_t microsteps, uint32_t *mres)
{
    switch (microsteps) {
    case 256U: *mres=0U; break;
    case 128U: *mres=1U; break;
    case 64U:  *mres=2U; break;
    case 32U:  *mres=3U; break;
    case 16U:  *mres=4U; break;
    case 8U:   *mres=5U; break;
    case 4U:   *mres=6U; break;
    case 2U:   *mres=7U; break;
    case 1U:   *mres=8U; break;
    default: return false;
    }
    return true;
}

/* 发送包含 CRC8 的寄存器写报文。 */

static bool WriteReg(UART_HandleTypeDef *huart, uint8_t addr, uint8_t reg, uint32_t data)
{
    uint8_t packet[TMC2209_WRITE_PACKET_SIZE];
    packet[0] = TMC2209_WRITE_SYNC;
    packet[1] = addr;
    packet[2] = (uint8_t)(reg | TMC2209_UART_WRITE_FLAG);
    packet[3] = (uint8_t)((data >> 24) & 0xFFU);
    packet[4] = (uint8_t)((data >> 16) & 0xFFU);
    packet[5] = (uint8_t)((data >>  8) & 0xFFU);
    packet[6] = (uint8_t)( data        & 0xFFU);
    packet[7] = CalcCRC8(packet, 7U);

    /* 尤其覆盖IFCNT读回复之后紧接NODECONF首写的读→写切换。 */
    HAL_Delay(TMC2209_BUS_IDLE_MS);
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, packet, TMC2209_WRITE_PACKET_SIZE, TMC2209_UART_TX_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf("# [TMC2209] TX fail: addr=%d reg=0x%02X err=%d\r\n", addr, reg, status);
    }
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)huart->Instance->DR;
    HAL_Delay(TMC2209_WRITE_SETTLE_MS);
    return status==HAL_OK;
}

/* 公开接口。 */

static bool ValidReference(TmcCurrentReference_t reference)
{
    return reference==TMC_CURRENT_EXTERNAL_VREF || reference==TMC_CURRENT_INTERNAL_REF;
}

static uint32_t ConfigGconf(TmcCurrentReference_t reference)
{
    /* i_scale_analog(bit0)与internal_Rsense(bit1)相互独立；始终清bit1，
     * 保留模块R110外部采样电阻，不能误写成内部MOSFET采样模式。 */
    return TMC2209_GCONF_MSTEP_REG_SELECT | TMC2209_GCONF_PDN_DISABLE |
        TMC2209_GCONF_MULTISTEP_FILT |
        (reference==TMC_CURRENT_EXTERNAL_VREF?TMC2209_GCONF_I_SCALE_ANALOG:0U);
}

static bool ConfigChecked(UART_HandleTypeDef *huart, uint8_t node_addr,
    uint16_t microsteps, uint8_t irun, uint8_t ihold,TmcCurrentReference_t reference)
{
    uint32_t mres;
    if (huart==NULL || huart->Instance==NULL || node_addr>3U || irun>31U ||
        ihold>31U || !ValidReference(reference) || !MicrostepsToMres(microsteps,&mres)) return false;
    /* 先建立多节点通信时序；失败即停止后续配置，不能将部分发送当作完成。 */
    if (!WriteReg(huart, node_addr, TMC2209_REG_NODECONF, TMC2209_NODECONF_SENDDELAY))
        return false;
    /* 原调用保留VREF；单轴试验可显式选内部参考。均不代表实际相电流测量。 */
    uint32_t gconf = ConfigGconf(reference);
    if (!WriteReg(huart, node_addr, TMC2209_REG_GCONF, gconf)) return false;

    /* 配置 CHOPCONF 的微步与插值选项。 */
    uint32_t chopconf = TMC2209_CHOPCONF_BASE |
                        TMC2209_CHOPCONF_INTPOL |
                        (mres << TMC2209_CHOPCONF_MRES_SHIFT);
    if (!WriteReg(huart, node_addr, TMC2209_REG_CHOPCONF, chopconf)) return false;

    /* 配置保持电流和运行电流。 */
    uint32_t ihold_irun = (6UL << TMC2209_IHOLDDELAY_SHIFT) |
                          (((uint32_t)irun & TMC2209_CURRENT_MASK) << TMC2209_IRUN_SHIFT) |
                          ((uint32_t)ihold & TMC2209_CURRENT_MASK);
    return WriteReg(huart, node_addr, TMC2209_REG_IHOLD_IRUN, ihold_irun);
}

bool Drv_TMC2209_ConfigNodeChecked(UART_HandleTypeDef *huart, uint8_t node_addr,
                                 uint16_t microsteps, uint8_t irun, uint8_t ihold)
{
    return ConfigChecked(huart,node_addr,microsteps,irun,ihold,TMC_CURRENT_EXTERNAL_VREF);
}

void Drv_TMC2209_ConfigNode(UART_HandleTypeDef *huart, uint8_t node_addr,
                            uint16_t microsteps, uint8_t irun, uint8_t ihold)
{
    /* 兼容原正常固件入口；新的调用者必须使用可检查返回值的接口。 */
    if (!Drv_TMC2209_ConfigNodeChecked(huart,node_addr,microsteps,irun,ihold))
        printf("# [TMC2209] configuration not fully transmitted: addr=%u\r\n",node_addr);
}

TmcReadResult_t Drv_TMC2209_ReadRegisterResult(UART_HandleTypeDef *huart, uint8_t node,
                                             uint8_t reg, uint32_t *out)
{
    if (huart==NULL || huart->Instance==NULL || out==NULL || node>3U || reg>0x7FU)
        return TMC_READ_ARGUMENT;
    if ((huart->Instance->CR1 & USART_CR1_RE)==0U) return TMC_READ_RX_DISABLED;
    uint8_t request[4]={0x05U,node,reg,0U};
    uint8_t reply[8];
    request[3]=CalcCRC8(request,3U);
    /* 给前次回复的总线释放及错误重同步留空闲时间；不进行自动重发。 */
    HAL_Delay(TMC2209_BUS_IDLE_MS);
    /* 板上PC6经1k接PDN_UART，PC7接同一总线，会收到本机发送的回声。
     * F407无多字节RX FIFO；发送期间暂关接收器，避免回声溢出。
     * 在TX完成后、回复开始前恢复RE；时序和模块SENDDELAY仍须上板验收。
     * 不屏蔽整个传输期间的中断，主机串口和定时ISR保持可响应。
     */
    CLEAR_BIT(huart->Instance->CR1,USART_CR1_RE);
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_StatusTypeDef sent=HAL_UART_Transmit(huart,request,4U,TMC2209_UART_TX_TIMEOUT_MS);
    __HAL_UART_CLEAR_OREFLAG(huart);
    SET_BIT(huart->Instance->CR1,USART_CR1_RE);
    if (sent!=HAL_OK) return TMC_READ_TX_FAILED;
    HAL_StatusTypeDef received=HAL_UART_Receive(huart,reply,8U,50U);
    if (received==HAL_TIMEOUT) return TMC_READ_RX_TIMEOUT;
    if (received!=HAL_OK) return TMC_READ_RX_FAILED;
    if (reply[0]!=0x05U || reply[1]!=0xFFU || reply[2]!=reg) return TMC_READ_FRAME;
    if (reply[7]!=CalcCRC8(reply,7U)) return TMC_READ_CRC;
    *out=((uint32_t)reply[3]<<24)|((uint32_t)reply[4]<<16)|
         ((uint32_t)reply[5]<<8)|(uint32_t)reply[6];
    return TMC_READ_OK;
}

bool Drv_TMC2209_ReadRegister(UART_HandleTypeDef *huart, uint8_t node,
                             uint8_t reg, uint32_t *out)
{
    /* 保留原布尔接口；诊断入口用Result接口定位失败阶段，不另存共享错误状态。 */
    return Drv_TMC2209_ReadRegisterResult(huart,node,reg,out)==TMC_READ_OK;
}

bool Drv_TMC2209_ConfigNodeVerified(UART_HandleTypeDef *huart, uint8_t node,
                                   uint16_t microsteps, uint8_t irun, uint8_t ihold,
                                   TmcConfigReport_t *report)
{
    return Drv_TMC2209_ConfigNodeVerifiedWithReference(huart,node,microsteps,irun,ihold,
        TMC_CURRENT_EXTERNAL_VREF,report);
}

bool Drv_TMC2209_ConfigNodeVerifiedWithReference(UART_HandleTypeDef *huart,uint8_t node,
    uint16_t microsteps,uint8_t irun,uint8_t ihold,TmcCurrentReference_t reference,
    TmcConfigReport_t *report)
{
    TmcConfigReport_t local={0};
    TmcConfigReport_t *r=report!=NULL?report:&local;
    *r=(TmcConfigReport_t){.result=TMC_CONFIG_ARGUMENT, .failed_register=0xFFU};
    uint32_t mres, value;
    if (huart==NULL || huart->Instance==NULL || node>3U || irun>31U ||
        ihold>31U || !ValidReference(reference) || !MicrostepsToMres(microsteps,&mres)) return false;

    /* 缺席节点即使HAL发送成功也不能放行；首次读失败时不发送任何配置。 */
    r->failed_register=TMC2209_REG_IFCNT;
    r->read_error=Drv_TMC2209_ReadRegisterResult(huart,node,TMC2209_REG_IFCNT,&value);
    if (r->read_error!=TMC_READ_OK) { r->result=TMC_CONFIG_READ_FAILED; return false; }
    r->ifcnt_before=(uint8_t)value;
    r->failed_register=0xFFU;
    if (!ConfigChecked(huart,node,microsteps,irun,ihold,reference)) {
        r->result=TMC_CONFIG_TX_FAILED;
        return false;
    }
    r->failed_register=TMC2209_REG_IFCNT;
    r->read_error=Drv_TMC2209_ReadRegisterResult(huart,node,TMC2209_REG_IFCNT,&value);
    if (r->read_error!=TMC_READ_OK) { r->result=TMC_CONFIG_READ_FAILED; return false; }
    r->ifcnt_after=(uint8_t)value;
    r->expected=4U;
    r->observed=(uint8_t)(r->ifcnt_after-r->ifcnt_before);
    if (r->observed!=r->expected) { r->result=TMC_CONFIG_COUNTER_MISMATCH; return false; }

    /* 只比较可写位；包含短路保护和STEP沿选择，防止保留错误控制位。
     * 不读取只写寄存器；IFCNT确认接收数不等于测得了实际相电流。 */
    const uint8_t regs[]={TMC2209_REG_GCONF,TMC2209_REG_CHOPCONF};
    const uint32_t masks[]={0x000003FFUL,0xFF0387FFUL};
    const uint32_t values[]={ConfigGconf(reference),TMC2209_CHOPCONF_BASE |
        TMC2209_CHOPCONF_INTPOL | (mres<<TMC2209_CHOPCONF_MRES_SHIFT)};
    for (unsigned i=0U;i<2U;i++) {
        r->failed_register=regs[i];
        r->expected=values[i]&masks[i];
        r->observed=0U;
        r->read_error=Drv_TMC2209_ReadRegisterResult(huart,node,regs[i],&value);
        if (r->read_error!=TMC_READ_OK) { r->result=TMC_CONFIG_READ_FAILED; return false; }
        if (i==0U) r->gconf_observed=value;
        else r->chopconf_observed=value;
        r->observed=value&masks[i];
        if (r->observed!=r->expected) { r->result=TMC_CONFIG_VALUE_MISMATCH; return false; }
    }
    r->failed_register=0xFFU;
    r->result=TMC_CONFIG_OK;
    return true;
}

void Drv_TMC2209_InitBus(UART_HandleTypeDef *huart)
{
    for (uint8_t addr = 0U; addr <= 3U; addr++) {
        Drv_TMC2209_ConfigNode(huart, addr, 16U, TMC_IRUN_HIGH, TMC_IHOLD_STRONG);
    }
}

/* 原四笔配置合同保持兼容；静音策略作为启动中的独立确认阶段。
 * Rev1.09 §5.1：SPREAD输入与GCONF.bit2异或，不可盲目清bit2就宣称静音。 */
bool Drv_TMC2209_ConfigureQuietVerified(UART_HandleTypeDef *huart,uint8_t node,
                                      TmcConfigReport_t *report)
{
    TmcConfigReport_t local={0};TmcConfigReport_t *r=report?report:&local;
    *r=(TmcConfigReport_t){.result=TMC_CONFIG_ARGUMENT,.failed_register=0xFFU};
    if(!huart || !huart->Instance || node>3U)return false;
    const uint8_t initial_regs[]={TMC2209_REG_IFCNT,TMC2209_REG_IOIN,TMC2209_REG_GCONF,TMC2209_REG_PWMCONF};
    uint32_t initial[4];
    for(unsigned i=0;i<4;i++) {
        r->failed_register=initial_regs[i];
        r->read_error=Drv_TMC2209_ReadRegisterResult(huart,node,initial_regs[i],&initial[i]);
        if(r->read_error!=TMC_READ_OK){r->result=TMC_CONFIG_READ_FAILED;return false;}
    }
    r->ifcnt_before=(uint8_t)initial[0];r->ioin_observed=initial[1];
    /* 只改模式位；保留VREF、外部Rsense、DIR、16细分及256插值的既有配置。 */
    uint32_t gconf=(initial[2]&0x3FFUL&~4UL)|((initial[1]&(1UL<<8))?4UL:0UL);
    /* 清保留位/自由滑行位，保持原OFS/GRAD及调节器参数，开启自动电流调节。
     * 01为手册推荐20~40kHz范围；不宣称已经完成实物AT#1/AT#2调谐。 */
    uint32_t pwmconf=(initial[3]&0xFF0CFFFFUL)|(1UL<<16)|(1UL<<18)|(1UL<<19);
    const uint8_t write_regs[]={TMC2209_REG_PWMCONF,TMC2209_REG_TPWMTHRS,TMC2209_REG_GCONF};
    const uint32_t write_values[]={pwmconf,0U,gconf};
    for(unsigned i=0;i<3;i++) {
        r->failed_register=write_regs[i];
        if(!WriteReg(huart,node,write_regs[i],write_values[i])){r->result=TMC_CONFIG_TX_FAILED;return false;}
    }
    uint32_t value;
    r->failed_register=TMC2209_REG_IFCNT;
    r->read_error=Drv_TMC2209_ReadRegisterResult(huart,node,TMC2209_REG_IFCNT,&value);
    if(r->read_error!=TMC_READ_OK){r->result=TMC_CONFIG_READ_FAILED;return false;}
    r->ifcnt_after=(uint8_t)value;r->expected=3U;
    r->observed=(uint8_t)(r->ifcnt_after-r->ifcnt_before);
    if(r->observed!=3U){r->result=TMC_CONFIG_COUNTER_MISMATCH;return false;}
    const uint8_t regs[]={TMC2209_REG_GCONF,TMC2209_REG_PWMCONF,TMC2209_REG_IOIN};
    const uint32_t masks[]={0x3FFUL,0xFF3FFFFFUL,1UL<<8};
    const uint32_t expected[]={gconf,pwmconf,initial[1]};
    for(unsigned i=0;i<3;i++) {
        r->failed_register=regs[i];r->expected=expected[i]&masks[i];r->observed=0U;
        r->read_error=Drv_TMC2209_ReadRegisterResult(huart,node,regs[i],&value);
        if(r->read_error!=TMC_READ_OK){r->result=TMC_CONFIG_READ_FAILED;return false;}
        if(i==0)r->gconf_observed=value;
        else if(i==1)r->pwmconf_observed=value;
        else r->ioin_observed=value;
        r->observed=value&masks[i];
        if(r->observed!=r->expected){r->result=TMC_CONFIG_VALUE_MISMATCH;return false;}
    }
    r->failed_register=0xFFU;r->result=TMC_CONFIG_OK;return true;
}
