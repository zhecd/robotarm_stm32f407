/** @file seek_logic.h
 * @brief 有界逐轴接触测试；不是整机回零，不标定、不退回、不发布HOMED。
 * STEP为50Hz、高1ms。短程版200步；扩展版以触发结束、每轴30秒故障超时。
 * completed仅记录本轮电气输入确认，不表示绝对零位或机械位置有效。
 */
#ifndef LIMIT_SEEK_LOGIC_H
#define LIMIT_SEEK_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#ifndef SEEK_MAX_STEPS
#define SEEK_MAX_STEPS 200U
#endif
#ifndef SEEK_AXIS_TIMEOUT_MS
#define SEEK_AXIS_TIMEOUT_MS 4500U
#endif
#ifndef SEEK_ALLOW_PRESSED_START
#define SEEK_ALLOW_PRESSED_START 0
#endif
typedef enum { SEEK_WAIT, SEEK_MOVING, SEEK_QUALIFY, SEEK_GAP, SEEK_DONE, SEEK_FAILED } SeekState;
typedef enum { SEEK_OK, SEEK_BUDGET, SEEK_UNSTABLE, SEEK_UNEXPECTED, SEEK_IRQ,
               SEEK_FEEDBACK, SEEK_TIME, SEEK_ABORT } SeekReason;
typedef struct {
    SeekState state;
    SeekReason reason;
    uint32_t steps[3], axis_ms, edge_ms, last_step_ms, qualify_ms;
    uint8_t axis, completed, initial_limits;
    bool enable, high, used, low_seen;
} Seek;
bool Seek_Start(Seek *s, uint32_t now, uint8_t limits);
void Seek_Stop(Seek *s, SeekReason reason);
/** 首次有效接触立即停步；再次调用不延长100ms消抖总期限。 */
bool Seek_BeginContact(Seek *s, uint32_t now, uint8_t axis);
/** 1ms上下文；调用方还须检查真实IRQ锁存、主机租约及反馈新鲜度。 */
void Seek_Tick(Seek *s, uint32_t now, uint8_t limits);
#endif
