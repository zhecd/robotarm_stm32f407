/**
 * @file m1_logic.h
 * @brief M1首次低速试验的纯状态机；不依赖HAL，不建立整机零位或运动许可。
 */
#ifndef M1_HOMING_LOGIC_H
#define M1_HOMING_LOGIC_H
#include <stdbool.h>
#include <stdint.h>

#define M1_PERIOD_MS 20U       /* 固定50微步/秒；每步高电平1ms。 */
/* 默认小步边界不放宽；较大点动必须显式选择独立构建并取得现场许可。
 * 89微步×1.8/16=10.0125电机轴度，约2.225关节度，不能称关节10度。 */
#ifndef M1_JOG_MOTOR_TEN_DEG
#define M1_JOG_MOTOR_TEN_DEG 0
#endif
#ifndef M1_JOG_JOINT_TEN_DEG
#define M1_JOG_JOINT_TEN_DEG 0
#endif
#if (M1_JOG_JOINT_TEN_DEG != 0 && M1_JOG_JOINT_TEN_DEG != 1) || \
    (M1_JOG_JOINT_TEN_DEG && M1_JOG_MOTOR_TEN_DEG)
#error Invalid or conflicting M1 joint-angle profile
#endif
#if M1_JOG_JOINT_TEN_DEG == 1
/* 400微步×360/3200/4.5=10关节度；50微步/s约8s，9s硬截止。
 * 只放宽这个显式点动配置，下面回零的5500ms上限保持原值。 */
#define M1_JOG_STEPS 400U
#define M1_JOG_DEADLINE_MS 9000U
#elif M1_JOG_MOTOR_TEN_DEG == 1
#define M1_JOG_STEPS 89U
#define M1_JOG_DEADLINE_MS 2000U
#elif M1_JOG_MOTOR_TEN_DEG == 0
#define M1_JOG_STEPS 20U
#define M1_JOG_DEADLINE_MS 500U
#else
#error Invalid M1_JOG_MOTOR_TEN_DEG
#endif
#ifndef M3_RETRY_500
#define M3_RETRY_500 0
#endif
#if M3_RETRY_500
/* 单独M3重测：保留50/s寻限位，退200微步用500/s；不建立整机零位。 */
#define M1_SEEK_STEPS 1500U
#define M1_BACKOFF_STEPS 200U
#define M1_HOME_DEADLINE_MS 31000U
#define M1_BACKOFF_PERIOD_MS 2U
#else
#define M1_SEEK_STEPS 200U     /* 寻限位最多约5关节度，不进行全行程搜索。 */
#define M1_BACKOFF_STEPS 40U   /* 固定退回约1关节度，不能沿用正常100度退回。 */
#define M1_HOME_DEADLINE_MS 5500U
#define M1_BACKOFF_PERIOD_MS M1_PERIOD_MS
#endif
/* TIM5端口需接纳所选点动的较长期限；业务HOME仍独立限制在5500ms。 */
#define M1_TIMER_MAX_MS ((M1_JOG_DEADLINE_MS > M1_HOME_DEADLINE_MS) ? M1_JOG_DEADLINE_MS : M1_HOME_DEADLINE_MS)
#define M1_LEASE_MS 500U       /* 主机需每100ms续约；500ms失联停止。 */
#define M1_MAIN_STALE_MS 250U
#define M1_FEEDBACK_STALE_MS 100U
#define M1_LIMIT_STABLE_MS 10U

/* HOME: IDLE -> SEEK -> QUALIFY -> BACKOFF -> DONE。
 * JOG:  IDLE -> JOG -> DONE。任一活动状态可进入STOPPED/FAILED，不可重放。
 * DONE只证明当前有限测试结束，不等于编码器原点或整机HOMED；局部参考由
 * m1_reference在主循环取得退回后的新样本后单独提交。
 */
typedef enum { M1_IDLE=0,M1_JOG,M1_SEEK,M1_QUALIFY,M1_BACKOFF,M1_DONE,M1_STOPPED,M1_FAILED } M1State;
typedef enum { M1_OK=0,M1_HOST_STOP,M1_TRAVEL_BOUND,M1_TIME_BOUND,M1_LIMIT_UNSTABLE,
    M1_LIMIT_STUCK,M1_OTHER_LIMIT,M1_FEEDBACK_LOST,M1_HOST_LOST,M1_MAIN_STALLED,
    M1_LIMIT_DURING_JOG,M1_INPUT_FAULT } M1Reason;
typedef struct {
    M1State state;
    M1Reason reason;
    bool used,enable,step,dir_high,direction_confirmed,seek_dir_high;
    bool release_candidate,release_confirmed;
    uint32_t started,phase_started,last_rise,lease_ms,limit_since,release_since; /* HAL毫秒时间戳。 */
    uint32_t jog_steps,seek_steps,backoff_steps; /* 已发STEP上升沿计数，不是编码器实测位移。 */
} M1Test;
typedef struct {
    bool m1_limit,other_limit,feedback_valid; /* 逻辑触发为true；GPIO低有效解码由m1_limits负责。 */
    uint32_t main_seen_ms;
} M1Inputs;

void M1Test_Init(M1Test *t);
/** 只记录经现场确认的寻限位方向，不发脉冲；使用过的会话不可修改。 */
bool M1Test_ConfirmDirection(M1Test *t,bool dir_high);
/** 主循环独占/短临界区调用。permit来自独立编译开关，ready来自真实配置读回。
 * 初始限位须松开；HOME需先确认方向。每次启动仅允许一个动作，结束/停止后不复用。
 */
bool M1Test_Start(M1Test *t,uint32_t now,bool home,bool jog_dir_high,
                  bool permit,bool driver_ready,const M1Inputs *in);
/** 有效的会话续约；不启动、不解锁、不自动重试。 */
bool M1Test_KeepAlive(M1Test *t,uint32_t now);
void M1Test_Stop(M1Test *t,M1Reason reason);
/** 1ms调用；不补发错过的脉冲，只描述所选单轴，硬件轴号由axis_binding映射。 */
void M1Test_Tick(M1Test *t,uint32_t now,const M1Inputs *in);
bool M1Test_Active(const M1Test *t);
uint32_t M1Test_Deadline(const M1Test *t);
#endif
