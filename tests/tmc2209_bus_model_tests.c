/**
 * @file tmc2209_bus_model_tests.c
 * @brief 真实驱动与虚拟多节点总线交互：配置接受、读回、掉包和错误路径。
 *
 * 仅替换HAL通信端口；生产帧构建/CRC/读取函数均编译真实源码。
 * 模型描述协议合同，不模拟真实电平、UART采样时序或电机运动。
 */
#include "driver/drv_tmc2209.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* CI失败直接退出，避免Windows断言弹窗阻塞后续回归。 */
#undef assert
#define assert(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1); } } while (0)

static USART_TypeDef registers;
static UART_HandleTypeDef uart={&registers};
static uint32_t words[4][128];
static uint8_t counters[4],pending[8],present_mask;
static unsigned tx_count,rx_count,write_count,fail_at,drop_at;
static HAL_StatusTypeDef failure_status;
static bool pending_valid;
static unsigned corrupt; /* 1:头部，2:CRC，3:模拟只收到本机4字节回声。 */
static unsigned corrupt_at,alter_reg,alter_at;
static uint32_t alter_mask;
static uint32_t bus_us,node_release_us;
static unsigned turnaround_drops;

/* 参考CRC先反转输入位序再执行MSB多项式除法，与驱动逐LSB写法分离。 */
static uint8_t CRC(const uint8_t *data,unsigned size)
{
    uint8_t remainder=0;
    for(unsigned i=0;i<size;i++) {
        uint8_t reversed=0;
        for(unsigned bit=0;bit<8;bit++)
            reversed|=(uint8_t)(((data[i]>>bit)&1U)<<(7U-bit));
        remainder^=reversed;
        for(unsigned bit=0;bit<8;bit++)
            remainder=(uint8_t)((remainder<<1)^((remainder&128U)?7U:0U));
    }
    return remainder;
}

static void Reset(uint8_t mask)
{
    memset(words,0,sizeof(words));memset(counters,0,sizeof(counters));
    memset(pending,0,sizeof(pending));present_mask=mask;
    registers.CR1=USART_CR1_RE;registers.DR=0;
    tx_count=rx_count=write_count=fail_at=drop_at=corrupt=0;
    corrupt_at=alter_at=0;alter_reg=255;alter_mask=0;
    failure_status=HAL_ERROR;pending_valid=false;
    bus_us=node_release_us=0;turnaround_drops=0;
    for(unsigned n=0;n<4;n++) {
        words[n][6]=0x21000041U|(n<<2); /* 各节点地址脚不同，避免固定回复掩盖串地址。 */
        words[n][0x6f]=0x80000000U|n;
    }
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *p,uint16_t size,uint32_t timeout)
{
    assert(u==&uart && timeout==100 && (size==4 || size==8));
    assert(p[0]==5 && p[1]<4 && p[size-1]==CRC(p,size-1));
    ++tx_count;pending_valid=false;
    if(fail_at==tx_count)return failure_status;
    unsigned node=p[1],reg=p[2]&127U;
    if(size==8) {
        assert(p[2]&128U);
        /* 所有配置都不能暗中写OTP、速度发生器等额外寄存器。 */
        assert(reg==0 || reg==3 || reg==0x10 || reg==0x6c || reg==0x13 || reg==0x70);
        ++write_count;
        /* 官方Rev1.09 §4.1.2：回复最后一位后，节点仍可能驱动总线4个bit。
         * 115200下向上取整为35us；快速主机不能把HAL接收完成当作已释放。
         * 冲突时HAL仍可返回OK，但芯片未接受该写帧、IFCNT不增加。
         * 本模型验证协议时序约束，不宣称测得了实物电平或确切丢失帧。 */
        if(bus_us<node_release_us){++turnaround_drops;return HAL_OK;}
        if(!(present_mask&(1U<<node)) || drop_at==write_count)return HAL_OK;
        uint32_t value=((uint32_t)p[3]<<24)|((uint32_t)p[4]<<16)|((uint32_t)p[5]<<8)|p[6];
        words[node][reg]=value;counters[node]++;
    } else {
        assert(!(registers.CR1&USART_CR1_RE) && !(p[2]&128U));
        if(!(present_mask&(1U<<node)) || reg==3 || reg==0x10 || reg==0x13)return HAL_OK;
        uint32_t value=reg==2?counters[node]:words[node][reg];
        if (reg==alter_reg && (!alter_at || tx_count==alter_at)) value^=alter_mask;
        pending[0]=5;pending[1]=255;pending[2]=(uint8_t)reg;
        for(unsigned i=0;i<4;i++)pending[3+i]=(uint8_t)(value>>(24U-i*8U));
        bool inject=corrupt_at==0 || corrupt_at==tx_count;
        if(inject && corrupt==1)pending[1]=0; /* 错误帧重新计算CRC，单独检验头部。 */
        pending[7]=CRC(pending,7);
        if(inject && corrupt==2)pending[7]^=1;
        pending_valid=true;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u,uint8_t *data,uint16_t size,uint32_t timeout)
{
    assert(u==&uart && size==8 && timeout==50 && (registers.CR1&USART_CR1_RE));
    ++rx_count;
    if(!pending_valid)return HAL_TIMEOUT;
    if(corrupt==3 && (corrupt_at==0 || corrupt_at==tx_count)) {
        const uint8_t echo[]={5,0,6,0x6f};
        memcpy(data,echo,sizeof(echo));return HAL_TIMEOUT;
    }
    memcpy(data,pending,8);node_release_us=bus_us+35U;return HAL_OK;
}
void HAL_Delay(uint32_t ms){assert(ms==2 || ms==5);bus_us+=ms*1000U;}

static uint32_t Read(unsigned node,unsigned reg)
{
    uint32_t value=0xdeadbeefU;
    assert(Drv_TMC2209_ReadRegisterResult(&uart,(uint8_t)node,(uint8_t)reg,&value)==TMC_READ_OK);
    return value;
}

static void QuietSetup(unsigned node,bool spread)
{
    Reset(15);words[node][0]=0x1C1U;words[node][0x70]=0xC10D0024U;
    if(spread)words[node][6]|=1UL<<8;
    /* 故意放入旧的非零切换阈值：必须由本次写入清除，不能依赖上电值。 */
    words[node][0x13]=12345U;
}
static void QuietTest(const char *name)
{
    TmcConfigReport_t r;
    if(!strcmp(name,"nodes") || !strcmp(name,"spread_high") || !strcmp(name,"wrap")) {
        unsigned starts=!strcmp(name,"wrap")?256U:1U;
        for(unsigned node=0;node<4;node++)for(unsigned start=0;start<starts;start++) {
            bool spread=!strcmp(name,"spread_high");QuietSetup(node,spread);counters[node]=(uint8_t)start;
            assert(Drv_TMC2209_ConfigureQuietVerified(&uart,(uint8_t)node,&r));
            assert(r.result==TMC_CONFIG_OK && tx_count==11 && rx_count==8 && write_count==3);
            assert(counters[node]==(uint8_t)(start+3) && r.ifcnt_before==start);
            assert(((words[node][0]>>2)&1U)==(unsigned)spread && words[node][0x13]==0);
            assert(words[node][0x70]==0xC10D0024U && r.pwmconf_observed==words[node][0x70]);
            for(unsigned other=0;other<4;other++)if(other!=node)assert(counters[other]==0);
        }
    } else if(!strcmp(name,"preserve")) {
        QuietSetup(0,false);words[0][0]=0x1C8U; /* 内部参考+反向标志，不可被静音配置覆盖。 */
        words[0][0x70]=0xD835102AU;words[0][0x6c]=0x14000053U;words[0][0x10]=0x00061C0FU;
        assert(Drv_TMC2209_ConfigureQuietVerified(&uart,0,&r));
        assert(words[0][0]==0x1C8U && words[0][0x6c]==0x14000053U && words[0][0x10]==0x00061C0FU);
        assert((words[0][0x70]&0xFFFFU)==0x102AU && (words[0][0x70]&0xFF000000U)==0xD8000000U);
        assert((words[0][0x70]&0x00FFFFFFU)==0x000D102AU); /* 只改频率/自动调节/自由滑行字段。 */
    } else if(!strcmp(name,"dropped")) {
        for(unsigned w=1;w<=3;w++) {
            QuietSetup(0,false);drop_at=w;
            assert(!Drv_TMC2209_ConfigureQuietVerified(&uart,0,&r));
            assert(r.result==TMC_CONFIG_COUNTER_MISMATCH && r.expected==3 && r.observed==2 && tx_count==8);
        }
    } else if(!strcmp(name,"tx_failure")) {
        for(unsigned at=1;at<=11;at++) {
            QuietSetup(0,false);fail_at=at;
            assert(!Drv_TMC2209_ConfigureQuietVerified(&uart,0,&r));
            assert(tx_count==at && (registers.CR1&USART_CR1_RE));
            assert(r.result==((at>=5 && at<=7)?TMC_CONFIG_TX_FAILED:TMC_CONFIG_READ_FAILED));
        }
    } else if(!strcmp(name,"crc") || !strcmp(name,"echo")) {
        const unsigned reads[]={1,2,3,4,8,9,10,11};
        for(unsigned i=0;i<8;i++) {
            QuietSetup(0,false);corrupt=!strcmp(name,"crc")?2:3;corrupt_at=reads[i];
            assert(!Drv_TMC2209_ConfigureQuietVerified(&uart,0,&r));
            assert(tx_count==reads[i] && r.result==TMC_CONFIG_READ_FAILED);
            assert(r.read_error==(!strcmp(name,"crc")?TMC_READ_CRC:TMC_READ_RX_TIMEOUT));
        }
    } else if(!strcmp(name,"readback") || !strcmp(name,"spread_changed")) {
        const unsigned regs[]={0,0x70,6},positions[]={9,10,11};
        const uint32_t masks[]={4U,1UL<<18,1UL<<8};
        unsigned first=!strcmp(name,"spread_changed")?2U:0U;
        unsigned end=!strcmp(name,"spread_changed")?3U:2U;
        for(unsigned i=first;i<end;i++) {
            QuietSetup(0,false);alter_reg=regs[i];alter_mask=masks[i];alter_at=positions[i];
            assert(!Drv_TMC2209_ConfigureQuietVerified(&uart,0,&r));
            assert(r.result==TMC_CONFIG_VALUE_MISMATCH && r.failed_register==regs[i] && tx_count==positions[i]);
        }
    } else if(!strcmp(name,"missing")) {
        QuietSetup(0,false);present_mask=0;
        assert(!Drv_TMC2209_ConfigureQuietVerified(&uart,0,&r));
        assert(r.result==TMC_CONFIG_READ_FAILED && tx_count==1 && write_count==0);
    } else if(!strcmp(name,"arguments")) {
        Reset(15);UART_HandleTypeDef bad={NULL};
        assert(!Drv_TMC2209_ConfigureQuietVerified(NULL,0,&r));
        assert(!Drv_TMC2209_ConfigureQuietVerified(&bad,0,&r));
        assert(!Drv_TMC2209_ConfigureQuietVerified(&uart,4,&r));
        assert(r.result==TMC_CONFIG_ARGUMENT && tx_count==0);
        registers.CR1=0;assert(!Drv_TMC2209_ConfigureQuietVerified(&uart,0,&r));
        assert(r.read_error==TMC_READ_RX_DISABLED && tx_count==0);
    } else if(!strcmp(name,"null_report")) {
        QuietSetup(0,false);assert(Drv_TMC2209_ConfigureQuietVerified(&uart,0,NULL));
    } else assert(false);
    printf("quiet profile %s passed; real hardware not tested\n",name);
}

int main(int argc,char **argv)
{
    if(argc==2){QuietTest(argv[1]);return 0;}
    unsigned cases=0;
    /* 穷举4地址、9档微步及全部32x32电流编码，检查接受后的实际模型寄存器。 */
    for(unsigned node=0;node<4;node++)for(unsigned mres=0;mres<9;mres++)
    for(unsigned run=0;run<32;run++)for(unsigned hold=0;hold<32;hold++) {
        Reset(15);
        assert(Drv_TMC2209_ConfigNodeChecked(&uart,(uint8_t)node,(uint16_t)(256U>>mres),
                                           (uint8_t)run,(uint8_t)hold));
        assert(write_count==4 && words[node][3]==0x200 && counters[node]==4);
        uint32_t gconf=Read(node,0),chop=Read(node,0x6c);
        assert((gconf&0x3ffU)==0x1c1U);
        assert(((chop>>24)&15U)==mres && (chop&15U)==3 && (chop&(1UL<<28)));
        assert(!(chop&0xe0000000U)); /* 禁止关短路保护或悄悄开启双沿步进。 */
        uint32_t current=words[node][0x10];
        assert((current&31U)==hold && ((current>>8)&31U)==run && (current>>16)==6);
        assert(Read(node,2)==4);
        for(unsigned other=0;other<4;other++)if(other!=node)
            assert(counters[other]==0 && words[other][0]==0 && words[other][0x10]==0);
        cases++;
    }
    /* 三节点共享总线时，地址3无响应；读取失败必须保留调用者旧值。 */
    Reset(7);
    for(unsigned n=0;n<3;n++)assert(Read(n,6)==(0x21000041U|(n<<2)));
    uint32_t value=0x12345678U;
    assert(Drv_TMC2209_ReadRegisterResult(&uart,3,6,&value)==TMC_READ_RX_TIMEOUT);
    assert(value==0x12345678U && (registers.CR1&USART_CR1_RE));

    /* HAL发送成功与芯片接受是不同事实：缺席节点和静默丢包不能被配置返回值发现。 */
    Reset(0);assert(Drv_TMC2209_ConfigNodeChecked(&uart,0,16,28,15));
    assert(write_count==4 && counters[0]==0);
    assert(Drv_TMC2209_ReadRegisterResult(&uart,0,6,&value)==TMC_READ_RX_TIMEOUT);
    for(unsigned drop=1;drop<=4;drop++) {
        Reset(7);drop_at=drop;
        assert(Drv_TMC2209_ConfigNodeChecked(&uart,0,16,28,15));
        assert(Read(0,2)==3); /* 接受数不足，不能以HAL_OK宣布驱动配置成功。 */
    }
    /* IFCNT是8位循环计数；跨255回绕仍能用模256差值检验4次写入。 */
    Reset(7);counters[0]=254;
    uint8_t before=(uint8_t)Read(0,2);
    assert(Drv_TMC2209_ConfigNodeChecked(&uart,0,16,28,15));
    uint8_t after=(uint8_t)Read(0,2);
    if((uint8_t)(after-before)!=4U)
        fprintf(stderr,"turnaround: IFCNT delta=%u expected=4; dropped=%u\n",(unsigned)(uint8_t)(after-before),turnaround_drops);
    assert(after==2 && (uint8_t)(after-before)==4);

    /* 传输错误在每个配置阶段注入；必须首错停止，没有隐式重试和后续写入。 */
    for(unsigned status=HAL_ERROR;status<=HAL_TIMEOUT;status++)for(unsigned at=1;at<=4;at++) {
        Reset(7);fail_at=at;failure_status=(HAL_StatusTypeDef)status;
        assert(!Drv_TMC2209_ConfigNodeChecked(&uart,0,16,28,15));
        assert(tx_count==at && counters[0]==at-1);
    }
    for(unsigned mode=1;mode<=3;mode++) {
        Reset(7);corrupt=mode;value=0x12345678U;
        TmcReadResult_t expected=mode==1?TMC_READ_FRAME:mode==2?TMC_READ_CRC:TMC_READ_RX_TIMEOUT;
        assert(Drv_TMC2209_ReadRegisterResult(&uart,0,6,&value)==expected);
        assert(value==0x12345678U && tx_count==1 && rx_count==1 && (registers.CR1&USART_CR1_RE));
    }
    /* 新确认接口必须区分HAL发送完成和芯片真正接受，覆盖全部IFCNT起点。 */
    TmcConfigReport_t report;
    /* 先前模型没有节点回传释放时间，因此漏掉“首读后立即写”的缺口。
     * 新模型中真实旧驱动应失败；加发送前空闲后仍须严格接受四笔，不能改期望为3。 */
    Reset(7);
    bool guard_ok=Drv_TMC2209_ConfigNodeVerified(&uart,0,16,28,15,&report);
    if(!guard_ok)fprintf(stderr,"turnaround: IFCNT delta=%u expected=4; dropped=%u\n",report.observed,turnaround_drops);
    assert(guard_ok && turnaround_drops==0 && report.ifcnt_after-report.ifcnt_before==4);
    for (unsigned n=0;n<4;n++) for (unsigned m=0;m<9;m++) for (unsigned start=0;start<256;start++) {
        Reset(15);counters[n]=(uint8_t)start;
        assert(Drv_TMC2209_ConfigNodeVerified(&uart,(uint8_t)n,(uint16_t)(256U>>m),28,15,&report));
        assert(report.result==TMC_CONFIG_OK && report.failed_register==255);
        assert(report.ifcnt_before==start && report.ifcnt_after==(uint8_t)(start+4));
        assert(tx_count==8 && rx_count==4 && write_count==4);
    }
    /* 两种参考源均逐个地址/微步/IFCNT起点验证，不能把参考源与Rsense选择混淆。 */
    unsigned reference_cases=0;
    for(unsigned ref=0;ref<2;ref++)for(unsigned n=0;n<4;n++)for(unsigned m=0;m<9;m++)
    for(unsigned start=0;start<256;start++) {
        Reset(15);counters[n]=(uint8_t)start;
        assert(Drv_TMC2209_ConfigNodeVerifiedWithReference(&uart,(uint8_t)n,
            (uint16_t)(256U>>m),8,4,(TmcCurrentReference_t)ref,&report));
        uint32_t expected=ref==TMC_CURRENT_INTERNAL_REF?0x1c0U:0x1c1U;
        assert(words[n][0]==expected && report.gconf_observed==expected);
        assert(report.chopconf_observed==(0x10000053U|(m<<24)));
        assert(!(report.gconf_observed&2U)); /* 外部Rsense继续使用。 */
        assert(report.ifcnt_before==start && report.ifcnt_after==(uint8_t)(start+4));
        assert(words[n][0x10]==0x00060804U && tx_count==8 && rx_count==4);
        for(unsigned other=0;other<4;other++)if(other!=n)assert(counters[other]==0);
        reference_cases++;
    }
    /* 参考源位或内部采样位错误，即使IFCNT正确也不得放行。 */
    for(unsigned bit=0;bit<2;bit++) {
        Reset(7);alter_reg=0;alter_mask=1U<<bit;
        assert(!Drv_TMC2209_ConfigNodeVerifiedWithReference(&uart,0,16,8,4,
            TMC_CURRENT_INTERNAL_REF,&report));
        assert(report.result==TMC_CONFIG_VALUE_MISMATCH && report.failed_register==0);
    }
    for(unsigned drop=1;drop<=4;drop++) {
        Reset(7);drop_at=drop;
        assert(!Drv_TMC2209_ConfigNodeVerifiedWithReference(&uart,0,16,8,4,
            TMC_CURRENT_INTERNAL_REF,&report));
        assert(report.result==TMC_CONFIG_COUNTER_MISMATCH && tx_count==6);
    }
    Reset(7);
    assert(!Drv_TMC2209_ConfigNodeVerifiedWithReference(&uart,0,16,8,4,
        (TmcCurrentReference_t)2,&report));
    assert(!Drv_TMC2209_ConfigNodeVerifiedWithReference(&uart,0,16,8,4,
        (TmcCurrentReference_t)-1,&report));
    assert(tx_count==0 && report.result==TMC_CONFIG_ARGUMENT);
    Reset(0);
    assert(!Drv_TMC2209_ConfigNodeVerifiedWithReference(&uart,0,16,8,4,
        TMC_CURRENT_INTERNAL_REF,&report));
    assert(write_count==0 && report.result==TMC_CONFIG_READ_FAILED);
    Reset(7);
    assert(Drv_TMC2209_ConfigNodeVerifiedWithReference(&uart,0,16,8,4,TMC_CURRENT_INTERNAL_REF,&report));
    assert(Drv_TMC2209_ConfigNodeVerified(&uart,0,16,8,4,&report));
    assert(words[0][0]==0x1c1U); /* 兼容入口仍恢复原VREF行为。 */
    Reset(0);
    assert(!Drv_TMC2209_ConfigNodeVerified(&uart,0,16,28,15,&report));
    assert(report.result==TMC_CONFIG_READ_FAILED && report.failed_register==2);
    assert(report.read_error==TMC_READ_RX_TIMEOUT && write_count==0 && tx_count==1);
    for (unsigned drop=1;drop<=4;drop++) {
        Reset(7);drop_at=drop;
        assert(!Drv_TMC2209_ConfigNodeVerified(&uart,1,16,28,15,&report));
        assert(report.result==TMC_CONFIG_COUNTER_MISMATCH && report.failed_register==2);
        assert(report.expected==4 && report.observed==3 && tx_count==6);
    }
    for (unsigned at=1;at<=8;at++) {
        Reset(7);fail_at=at;
        assert(!Drv_TMC2209_ConfigNodeVerified(&uart,2,16,28,15,&report));
        assert(tx_count==at); /* 首错返回，不重试、不继续写或读。 */
        assert(report.result==((at>=2 && at<=5)?TMC_CONFIG_TX_FAILED:TMC_CONFIG_READ_FAILED));
        assert(registers.CR1&USART_CR1_RE);
    }
    const unsigned read_positions[]={1,6,7,8};
    for (unsigned i=0;i<4;i++) {
        Reset(7);corrupt=2;corrupt_at=read_positions[i];
        assert(!Drv_TMC2209_ConfigNodeVerified(&uart,0,16,28,15,&report));
        assert(report.result==TMC_CONFIG_READ_FAILED && report.read_error==TMC_READ_CRC);
        assert(tx_count==corrupt_at);
    }
    /* 接收计数正确但配置值异常仍不放行；保护位和双沿STEP位也必须核对。 */
    const unsigned compare_regs[]={0,0x6c,0x6c,0x6c,0x6c};
    const uint32_t compare_bits[]={1U,1U<<24,1U<<29,1U<<30,1U<<31};
    for (unsigned i=0;i<5;i++) {
        Reset(7);alter_reg=compare_regs[i];alter_mask=compare_bits[i];
        assert(!Drv_TMC2209_ConfigNodeVerified(&uart,0,16,28,15,&report));
        assert(report.result==TMC_CONFIG_VALUE_MISMATCH && report.failed_register==compare_regs[i]);
    }
    Reset(7);alter_reg=0x6c;alter_mask=1U<<20; /* 保留位不参与配置比较。 */
    assert(Drv_TMC2209_ConfigNodeVerified(&uart,0,16,28,15,NULL));
    Reset(7);
    assert(!Drv_TMC2209_ConfigNodeVerified(NULL,0,16,28,15,&report));
    assert(!Drv_TMC2209_ConfigNodeVerified(&uart,4,16,28,15,&report));
    assert(!Drv_TMC2209_ConfigNodeVerified(&uart,0,3,28,15,&report));
    assert(!Drv_TMC2209_ConfigNodeVerified(&uart,0,16,32,15,&report));
    assert(!Drv_TMC2209_ConfigNodeVerified(&uart,0,16,28,32,&report));
    assert(report.result==TMC_CONFIG_ARGUMENT && tx_count==0 && write_count==0);
    Reset(7);registers.CR1=0;
    assert(!Drv_TMC2209_ConfigNodeVerified(&uart,0,16,28,15,&report));
    assert(report.result==TMC_CONFIG_READ_FAILED && report.read_error==TMC_READ_RX_DISABLED);
    assert(tx_count==0 && write_count==0);
    printf("tmc2209_bus_model_tests: configuration_cases=%u; verified_cases=9216; reference_cases=%u; protocol_fault_controls=passed; real_hardware=not_tested\n",cases,reference_cases);
    return 0;
}
