/** @file home_sequence.h
 * @brief 回零纯状态机：支持逐轴教学对照与三轴接触屏障后的并行退回。
 * 不读GPIO、不写电机、不清故障。mask/rearm是请求，真实端口须确认后回执。
 * 时间均为无符号毫秒差，允许uint32_t回绕；中断/主循环由外层串行化。
 */
#ifndef ROBOTARM_HOME_SEQUENCE_H
#define ROBOTARM_HOME_SEQUENCE_H
#include <stdbool.h>
#include <stdint.h>
/* 退回允许的累计输入确认暂停上限；不是额外步数，也不延长整轮/寻限位截止。 */
#define HOME_BACKOFF_PAUSE_MAX_MS 2000U
typedef enum { HOME_BOOT,HOME_SEEK,HOME_CONTACT,HOME_BACKOFF,HOME_RELEASE,
               HOME_GAP,HOME_DONE,HOME_FAILED } HomePhase;
typedef enum { HOME_OK,HOME_BAD_CONFIG,HOME_UNSTABLE,HOME_UNEXPECTED,
               HOME_IRQ,HOME_SEEK_TIMEOUT,HOME_BACK_TIMEOUT,HOME_TOTAL_TIMEOUT,
               HOME_PORT,HOME_CANCELLED,HOME_MAIN_STALLED } HomeReason;
typedef struct {
    /** 寻限位STEP周期1..1000ms、单轴寻进截止、整轮截止。单轴截止0表示取消。 */
    uint32_t period_ms,seek_timeout_ms,total_timeout_ms;
    /** 每轴反向退回微步数；轴号0/1/2分别为M1/M2/M3。 */
    uint32_t backoff[3];
    /** DIR实际高电平位图，不表示通用顺/逆时针。 */
    uint8_t seek_dir;
    /** 退回STEP周期1..1000ms，1ms即1000微步/秒。0沿用寻限位周期，兼容旧配置。 */
    uint32_t backoff_period_ms;
    /** GPIO采样候选：非当前轴异常先停步再确认；旧EXTI对照保持原策略。 */
    bool qualify_unexpected;
} HomeConfig;
typedef struct {
    HomeConfig config;
    HomePhase phase;
    HomeReason reason;
    /** 软件脉冲请求计数，不代替编码器或机械到位证据。 */
    uint32_t seek[3],back[3],began,axis_began,phase_began,last_step;
    uint32_t stable_since,boot_began[3],boot_low_since[3];
    uint32_t hold_began,hold_high_since[3],hold_low_since[3];
    uint32_t back_pause_ms; /* 当前退回段已接受的暂停时间，逐轴重新累计。 */
    uint32_t hold_sample_ms,last_sample_ms,failure_ms;
    uint32_t contact_high_since,contact_cancel_count;
    uint8_t hold_expect_low,last_low,failure_low;
    bool contact_high_seen;
    /** contacts=历史接触；completed=退回/释放/布防全部完成；owned=暂由本服务屏蔽。 */
    uint8_t axis,contacts,completed,owned,rearmed,initial;
    uint8_t mask_request,rearm_request,boot_stable,boot_low_seen,boot_high_seen;
    uint8_t hold_watch,hold_high_seen,hold_low_seen;
    bool started,enable,high,dir_high,low_seen,release_seen;
    bool holding,recovery;
    uint32_t hold_count;
    /** 并行模式：各轴独立停步确认，全部稳定压下后才允许任何反向STEP。
     * step_mask/dir_mask为三个逻辑轴的输出请求；旧逐轴教学对照不使用它们。 */
    bool parallel;
    uint8_t step_mask,dir_mask,parallel_low_seen,parallel_high_seen;
    HomePhase parallel_phase[3];
    uint32_t parallel_last_step[3],parallel_began[3];
    uint32_t parallel_low_since[3],parallel_high_since[3];
} HomeSequence;
void HomeSequence_Init(HomeSequence *s,const HomeConfig *config);
/** 显式M999继承本服务拥有的屏蔽位；高/低都重新稳定确认，不恢复故障轴IRQ。 */
void HomeSequence_PrepareRecovery(HomeSequence *s,const HomeConfig *config,uint8_t owned,uint32_t now);
bool HomeSequence_BootReady(const HomeSequence *s,uint8_t low);
bool HomeSequence_Start(HomeSequence *s,uint32_t now,uint8_t low);
/** 返回true表示本事件由回零接管，端口必须先停输出再执行单路屏蔽。 */
bool HomeSequence_OnEdge(HomeSequence *s,uint8_t axis,uint32_t now,uint8_t low);
void HomeSequence_Tick(HomeSequence *s,uint32_t now,uint8_t low);
void HomeSequence_AckRearm(HomeSequence *s,uint8_t axis,bool accepted);
void HomeSequence_Fail(HomeSequence *s,HomeReason reason);
#endif
