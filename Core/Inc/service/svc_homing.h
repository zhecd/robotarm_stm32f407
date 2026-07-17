/**
 * @file    bsp_homing.h
 * @brief   Limit-switch-based homing sequence with back-off / 閸╄桨绨梽鎰秴瀵偓閸忓磭娈戦崶鐐烘祩鎼村繐鍨?(鐟欙妇顫?閸ョ偤鈧偓)
 * @ingroup bsp
 */

#ifndef __BSP_HOMING_H__
#define __BSP_HOMING_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Execute full homing sequence / 閹笛嗩攽鐎瑰本鏆ｉ崶鐐烘祩鎼村繐鍨?*/
bool Svc_Homing_Execute(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HOMING_H__ */
