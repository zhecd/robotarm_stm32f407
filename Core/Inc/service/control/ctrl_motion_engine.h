/**
 * @file    ctrl_motion_engine.h
 * @brief   Bresenham-based stepper motion engine with lock-free ring buffer / 鍩轰�?Bresenham 绠楁硶鐨勬杩涜繍鍔ㄥ紩�?+ 鏃犻攣鐜舰缂撳�?
 * @ingroup control
 *
 * Driven by TIM6 at 50 kHz. Each MotionFrame_t specifies delta steps for
 * M1/M2/M3 spread over total_ticks interrupts. Frames are queued via a
 * 64-entry lock-free ring buffer.
 * �?TIM6 50kHz 椹卞姩銆傛瘡甯ф寚�?M1/M2/M3 鐨勫閲忔�? 鍒嗘暎鍒?total_ticks 娆′腑鏂腑鎵ц銆?
 * 甯ч€氳繃 64 椤规棤閿佺幆褰㈢紦鍐插尯鎺掗槦銆?
 *
 * Theory-step tracking provides the "commanded position" reference for
 * the closed-loop PID controller.
 * 鐞嗚姝ユ暟杩借釜�?PID 闂幆鎺у埗鍣ㄦ彁�?鍛戒护浣嶇疆"鍙傝€冦�?
 */

#ifndef __CTRL_MOTION_ENGINE_H__
#define __CTRL_MOTION_ENGINE_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** A single motion segment: delta steps for 3 axes + duration in timer ticks / 鍗曟杩愬姩: 涓夎酱澧為噺姝ユ�?+ 瀹氭椂鍣ㄨ妭鎷嶆椂闀?*/
typedef struct {
    int32_t  delta_m1;          /* M1 axis step delta / M1 杞存鏁板�?*/
    int32_t  delta_m2;          /* M2 axis step delta / M2 杞存鏁板�?*/
    int32_t  delta_m3;          /* M3 axis step delta / M3 杞存鏁板�?*/
    uint32_t total_ticks;       /* Duration in TIM6 ticks / TIM6 鑺傛媿鏃堕暱 */
} MotionFrame_t;

typedef enum {
    MOTION_FAULT_NONE = 0,
    MOTION_FAULT_LIMIT_SWITCH,
    MOTION_FAULT_ENCODER,
    MOTION_FAULT_SOFT_LIMIT,
    MOTION_FAULT_QUEUE_TIMEOUT
} MotionFaultReason_t;

void Ctrl_MotionEngine_Init(void);

/** Push a frame into the ring buffer. Returns false if full. / 鍘嬪叆涓€甯у埌鐜舰缂撳�? 婊℃椂杩斿洖 false */
bool Ctrl_MotionEngine_PushFrame(const MotionFrame_t *frame);

/** Clear all queued frames (emergency stop) / 娓呯┖鎵€鏈夋帓闃熷�?(鎬ュ�? */
void Ctrl_MotionEngine_Clear(void);

/* Stops the active frame and discards queued frames.  A fault remains latched
 * until it is explicitly cleared after a successful homing sequence. */
void Ctrl_MotionEngine_EmergencyStop(void);
void Ctrl_MotionEngine_EmergencyStopWithReason(MotionFaultReason_t reason);
void Ctrl_MotionEngine_EnableLimitMonitoring(bool enabled);
void Ctrl_MotionEngine_NotifyLimitSwitch(uint16_t gpio_pin);
void Ctrl_MotionEngine_ServiceSafety(void);
bool Ctrl_MotionEngine_HasFault(void);
MotionFaultReason_t Ctrl_MotionEngine_GetFaultReason(void);
void Ctrl_MotionEngine_ClearFault(void);

bool     Ctrl_MotionEngine_IsRunning(void);       /* Motion in progress / 杩愬姩涓?*/
uint16_t Ctrl_MotionEngine_GetQueueCount(void);   /* Queued frame count / 鎺掗槦甯ф�?*/

/** Read the accumulated "theory" (commanded) step positions / 璇诲彇绱Н鐨勭悊�?鍛戒�?姝ユ暟浣嶇疆 */
void Ctrl_MotionEngine_GetTheorySteps(int32_t *m1, int32_t *m2, int32_t *m3);
void Ctrl_MotionEngine_ResetTheorySteps(void);
void Ctrl_MotionEngine_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3);

/** Execute one 50 kHz step tick.  Only the App ISR adapter may call this. */
void Ctrl_MotionEngine_OnStepTickFromISR(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_MOTION_ENGINE_H__ */
