/**
 * @file    ctrl_compensation.h
 * @brief   Static position error compensation after each G-code move / G-code 杩愬姩鍚庨潤鎬佷綅缃宸ˉ�?
 * @ingroup control
 *
 * Iteratively drives encoder readings toward the theory-step target using
 * small correction frames. Includes deadband, watchdog, and stuck-encoder
 * detection to prevent runaway.
 * 杩唬椹卞姩缂栫爜鍣ㄨ秼杩戠悊璁烘鏁扮洰鏍? 鍚鍖?鐪嬮棬鐙?鍗℃妫€娴嬮槻澶辨帶銆?
 */

#ifndef __CTRL_COMPENSATION_H__
#define __CTRL_COMPENSATION_H__

#ifdef __cplusplus
extern "C" {
#endif

void Ctrl_Compensation_Execute(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_COMPENSATION_H__ */
