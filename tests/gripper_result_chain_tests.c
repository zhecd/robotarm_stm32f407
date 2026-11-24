/**
 * @file gripper_result_chain_tests.c
 * @brief 真实Service→Device→PWM驱动集成测试，只替换许可、时间和HAL。
 * @details 不使用真实串口/定时器。许可拒绝、HAL失败、幂等保持和600ms停机
 * 必须通过真实分层代码；协议回复由action_gripper_start_failure_tests另测。
 */
#include "service/svc_gripper.h"
#include "device/dev_gripper.h"
#include "driver/drv_servo.h"
#include "platform_critical.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr,"gripper chain line %d: %s\n",__LINE__,#x);exit(1); \
} } while (0)

static TIM_HandleTypeDef timer;
static uint32_t now, starts, stops, writes;
static unsigned critical_depth;
/* 模拟F4 HAL真实的READY/BUSY语义：已启动的通道重复Start必须失败。
 * 不能使用永远HAL_OK的替身，否则快速交替目标的回归无法暴露。 */
static bool permit=true, fail_start, fail_stop, channel_busy;

uint32_t HAL_GetTick(void) { return now; }
PlatformCriticalState_t PlatformCritical_Enter(void) { return critical_depth++; }
void PlatformCritical_Exit(PlatformCriticalState_t previous)
{
    CHECK(critical_depth==previous+1U);
    critical_depth=previous;
}
bool MotionService_IsActionAllowed(void)
{
    CHECK(critical_depth==1U);
    return permit;
}
void Test_SetCompare(TIM_HandleTypeDef *t,uint32_t channel,uint32_t pulse)
{
    CHECK(t==&timer && channel==TIM_CHANNEL_2);
    t->compare=pulse;
    writes++;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t channel)
{
    CHECK(t==&timer && channel==TIM_CHANNEL_2 && writes>starts);
    starts++;
    if (fail_start || channel_busy) return HAL_ERROR;
    channel_busy=true;return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *t,uint32_t channel)
{
    CHECK(t==&timer && channel==TIM_CHANNEL_2);
    stops++;
    if (fail_stop) return HAL_ERROR;
    channel_busy=false;return HAL_OK;
}
/* 绑定替身不产生PWM；本测试不替换Device/Service/driver的业务函数。 */
void BSP_RobotBoard_InitGripper(void)
{
    Drv_Servo_Init(Drv_Servo_GetHandle(),&timer,TIM_CHANNEL_2);
}
bool BSP_RobotBoard_TryInitGripper(void)
{
    return Drv_Servo_TryInit(Drv_Servo_GetHandle(),&timer,TIM_CHANNEL_2);
}

int main(int argc,char **argv)
{
    CHECK(argc==2);
    const char *scenario=argv[1];
    ServoDevice_t *g=Drv_Servo_GetHandle();
    if (!strcmp(scenario,"unbound")) {
        CHECK(!Svc_Gripper_Open() && !Svc_Gripper_Close());
        GripperStatus_t s;Svc_Gripper_GetStatus(&s);Svc_Gripper_GetStatus(NULL);
        CHECK(!s.bound && !s.pwm_requested && s.target==GRIPPER_TARGET_NONE && s.hold_ms==600);
        CHECK(!starts && !writes && !stops);
    } else {
        CHECK(Drv_Servo_Bind(g,&timer,TIM_CHANNEL_2));
        if (!strcmp(scenario,"status")) {
            GripperStatus_t s;
            Svc_Gripper_GetStatus(&s);CHECK(s.bound && s.target==GRIPPER_TARGET_NONE && !s.pwm_requested);
            CHECK(Svc_Gripper_Open());now=599;
            ServoDevice_t before;memcpy(&before,g,sizeof(before));
            uint32_t writes_before=writes,starts_before=starts,stops_before=stops;
            Svc_Gripper_GetStatus(&s);
            CHECK(s.target==GRIPPER_TARGET_OPEN && s.target_angle_deg==100 && s.elapsed_ms==599);
            CHECK(memcmp(g,&before,sizeof(before))==0 && writes==writes_before && starts==starts_before && stops==stops_before);
            /* 读查询不代替周期收尾；到600ms仍显示真实软件请求，不能假装已停。 */
            now=600;Svc_Gripper_GetStatus(&s);CHECK(s.pwm_requested && s.elapsed_ms==600);
            Svc_Gripper_IdleStop();Svc_Gripper_GetStatus(&s);
            CHECK(!s.pwm_requested && s.elapsed_ms==600 && s.target==GRIPPER_TARGET_OPEN);
            CHECK(Svc_Gripper_Close());now=620;channel_busy=false;Drv_Servo_RecordStopResult(g,true);
            Svc_Gripper_GetStatus(&s);CHECK(!s.pwm_requested && s.elapsed_ms==20 && s.target==GRIPPER_TARGET_CLOSE);
            fail_start=true;CHECK(!Svc_Gripper_Open());Svc_Gripper_GetStatus(&s);
            CHECK(s.error==GRIPPER_ERROR_START && s.target==GRIPPER_TARGET_NONE && !s.pwm_requested);
            fail_stop=true;CHECK(!Svc_Gripper_Open());Svc_Gripper_GetStatus(&s);
            CHECK(s.error==GRIPPER_ERROR_START && s.pwm_requested);
            fail_start=fail_stop=false;CHECK(!Svc_Gripper_Close());
            CHECK(Drv_Servo_TryStop(g));CHECK(Svc_Gripper_Close());
            fail_stop=true;CHECK(!Drv_Servo_TryStop(g));Svc_Gripper_GetStatus(&s);
            CHECK(s.error==GRIPPER_ERROR_STOP && s.pwm_requested);
            fail_stop=false;CHECK(Drv_Servo_TryStop(g));Svc_Gripper_GetStatus(&s);
            CHECK(!s.pwm_requested && s.error==GRIPPER_ERROR_STOP);
            fail_start=true;CHECK(!Svc_Gripper_Open());Svc_Gripper_GetStatus(&s);
            CHECK(s.error==GRIPPER_ERROR_STOP); /* 失败的新尝试不覆盖旧首因。 */
            fail_start=false;
            CHECK(Svc_Gripper_Open());Svc_Gripper_GetStatus(&s);CHECK(s.error==GRIPPER_ERROR_NONE);
        } else if (!strcmp(scenario,"rapid") || !strcmp(scenario,"rapid_wrap")) {
            uint32_t base=!strcmp(scenario,"rapid_wrap")?UINT32_MAX-150U:0U;
            now=base;CHECK(Svc_Gripper_Open());
            for(unsigned i=1;i<=7;i++) {
                now=base+i*100U;
                CHECK(i%2U?Svc_Gripper_Close():Svc_Gripper_Open());
                CHECK(timer.compare==(i%2U?1300U:1611U));
                GripperStatus_t s;Svc_Gripper_GetStatus(&s);
                CHECK(s.target==(i%2U?GRIPPER_TARGET_CLOSE:GRIPPER_TARGET_OPEN));
                CHECK(s.elapsed_ms==0 && s.error==GRIPPER_ERROR_NONE && s.pwm_requested);
                Svc_Gripper_IdleStop();CHECK(g->pwm_active && channel_busy);
            }
            CHECK(starts==1 && stops==0 && writes==8);
            now=base+1299U;CHECK(Svc_Gripper_Close()); /* 同目标不续期。 */
            CHECK(g->hold_start==base+700U && starts==1 && writes==8);
            Svc_Gripper_IdleStop();CHECK(g->pwm_active);
            now=base+1300U;Svc_Gripper_IdleStop();CHECK(!g->pwm_active && stops==1);
            CHECK(Svc_Gripper_Open() && starts==2 && channel_busy);
        } else if (!strcmp(scenario,"rapid_permission")) {
            CHECK(Svc_Gripper_Open());now=100;permit=false;
            CHECK(!Svc_Gripper_Close() && timer.compare==1611 && writes==1);
            CHECK(g->hold_start==0 && starts==1 && stops==0);
            now=600;Svc_Gripper_IdleStop();CHECK(!channel_busy && !g->pwm_active);
            CHECK(!Svc_Gripper_Open() && starts==1);
        } else if (!strcmp(scenario,"rapid_stop_failure")) {
            CHECK(Svc_Gripper_Open());fail_stop=true;now=100;
            CHECK(!Drv_Servo_TryStop(g));
            CHECK(!Svc_Gripper_Close() && timer.compare==1611 && writes==1);
            CHECK(g->error==GRIPPER_ERROR_STOP && channel_busy);
            fail_stop=false;CHECK(Drv_Servo_TryStop(g));
            CHECK(Svc_Gripper_Close() && starts==2 && g->error==GRIPPER_ERROR_NONE);
        } else if (!strcmp(scenario,"external_busy")) {
            channel_busy=true;CHECK(!Svc_Gripper_Open());
            CHECK(starts==1 && stops==1 && !g->pwm_active && !channel_busy);
            CHECK(g->error==GRIPPER_ERROR_START && g->state==SERVO_STATE_UNKNOWN);
        } else if (!strcmp(scenario,"init_failure")) {
            fail_start=true;
            CHECK(!Dev_Gripper_TryInit());
            CHECK(g->state==SERVO_STATE_UNKNOWN && !g->pwm_active && starts==1);
        } else if (!strcmp(scenario,"init_success")) {
            CHECK(Dev_Gripper_TryInit() && timer.compare==1300 && starts==1);
            CHECK(g->state==SERVO_STATE_CLOSE && g->pwm_active);
            now=600;Svc_Gripper_IdleStop();CHECK(!g->pwm_active);
        } else if (!strcmp(scenario,"permission")) {
            permit=false;
            CHECK(!Svc_Gripper_Open() && !Svc_Gripper_Close());
            CHECK(!starts && !writes && !stops);
        } else if (!strcmp(scenario,"start_failure")) {
            fail_start=true;
            CHECK(!Svc_Gripper_Open());
            CHECK(starts==1 && stops==1 && !g->pwm_active);
            CHECK(!Svc_Gripper_Close());
            CHECK(starts==2 && stops==2 && g->state==SERVO_STATE_UNKNOWN);
        } else if (!strcmp(scenario,"stop_failure")) {
            fail_start=fail_stop=true;
            CHECK(!Svc_Gripper_Open());
            CHECK(g->pwm_active && g->state==SERVO_STATE_UNKNOWN);
            fail_start=fail_stop=false;
            CHECK(Drv_Servo_TryStop(g) && !g->pwm_active);
        } else if (!strcmp(scenario,"wrap")) {
            now=UINT32_MAX-100U;
            CHECK(Svc_Gripper_Close() && timer.compare==1300);
            now=498;Svc_Gripper_IdleStop();CHECK(g->pwm_active);
            now=499;Svc_Gripper_IdleStop();CHECK(!g->pwm_active && stops==1);
            GripperStatus_t s;Svc_Gripper_GetStatus(&s);
            CHECK(s.elapsed_ms==600 && s.target==GRIPPER_TARGET_CLOSE && !s.pwm_requested);
        } else if (!strcmp(scenario,"success")) {
            CHECK(Svc_Gripper_Open() && timer.compare==1611);
            CHECK(g->state==SERVO_STATE_OPEN && g->pwm_active);
            now=599;
            CHECK(Svc_Gripper_Open() && starts==1 && g->hold_start==0);
            Svc_Gripper_IdleStop();CHECK(g->pwm_active);
            now=600;Svc_Gripper_IdleStop();CHECK(!g->pwm_active && stops==1);
            CHECK(Svc_Gripper_Close() && timer.compare==1300 && starts==2);
            now=1199;CHECK(Svc_Gripper_Close() && starts==2 && g->hold_start==600);
            now=1200;Svc_Gripper_IdleStop();CHECK(!g->pwm_active && stops==2);
        } else { CHECK(false); }
    }
    CHECK(critical_depth==0);
    printf("gripper_result_chain (%s): passed\n",scenario);
    return 0;
}
