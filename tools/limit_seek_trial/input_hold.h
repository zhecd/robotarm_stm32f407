/** 尚未测试的启动已压触点变高：先停输出，再确认，不边运动边消抖。
 * 高持续10ms判释放；低重新稳定10ms才恢复同一段。100ms不稳定则失败。
 * 不改变方向、不重置轴/整轮期限、不重发START；不是失败后的自动重试。 */
#ifndef SEEK_INPUT_HOLD_H
#define SEEK_INPUT_HOLD_H
#include "seek_logic.h"
typedef enum { HOLD_NONE,HOLD_WAIT,HOLD_RESUMED,HOLD_RELEASED,HOLD_UNSTABLE } SeekHoldResult;
typedef struct {
    SeekHoldResult result;
    bool paused;
    uint8_t watch,input,high_seen,low_seen;
    uint32_t began,high_since[3],low_since[3];
} SeekInputHold;
/** 返回true才允许本tick推进Seek_Tick；暂停/失败时保持输出关闭。 */
bool SeekHold_Service(SeekInputHold *h,Seek *s,uint8_t watch,uint8_t input,uint32_t now);
#endif
