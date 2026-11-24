/** @file trial_logic.h
 * @brief 整机启动试验的纯逻辑许可；不代替正常回零、标定或全局安全服务。
 * @details 复位后等待一次START；任何终止均耗尽本次机会，KEEP不能重启。
 */
#ifndef STARTUP_TRIAL_LOGIC_H
#define STARTUP_TRIAL_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#ifndef TRIAL_DEADLINE_MS
#define TRIAL_DEADLINE_MS 90000U
#endif
#define TRIAL_LEASE_MS 350U
#define TRIAL_HEARTBEAT_MS 250U
typedef enum { TRIAL_WAIT,TRIAL_ACTIVE,TRIAL_DONE,TRIAL_FAILED } TrialState;
typedef enum { TRIAL_NONE,TRIAL_COMPLETE,TRIAL_HOST_STOP,TRIAL_DEADLINE,
    TRIAL_LEASE_LOST,TRIAL_MAIN_STALLED,TRIAL_INPUT,TRIAL_APP_FAILED,
    TRIAL_ENCODER,TRIAL_OUTPUT,TRIAL_PWM_OVERRUN } TrialReason;
typedef struct {
    TrialState state;
    TrialReason reason;
    bool used;
    uint32_t began_ms,keep_ms;
} Trial;
bool Trial_Start(Trial *t,uint32_t now,bool compiled,bool ready);
bool Trial_Keep(Trial *t,uint32_t now);
void Trial_Stop(Trial *t,TrialReason why);
void Trial_Tick(Trial *t,uint32_t now,uint32_t main_seen);
/** 严格白名单，不放行M999、运动/模式切换或参数前后缀。 */
bool Trial_IsQuery(const char *line);
#endif
