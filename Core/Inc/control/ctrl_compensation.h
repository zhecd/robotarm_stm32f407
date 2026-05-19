/**
 * @file    ctrl_compensation.h
 * @brief   Static position error compensation after each G-code move / G-code 运动后静态位置误差补偿
 * @ingroup control
 *
 * Iteratively drives encoder readings toward the theory-step target using
 * small correction frames. Includes deadband, watchdog, and stuck-encoder
 * detection to prevent runaway.
 * 迭代驱动编码器趋近理论步数目标, 含死区/看门狗/卡死检测防失控。
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
