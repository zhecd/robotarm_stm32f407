/**
 * @file    bsp_homing.h
 * @brief   Limit-switch-based homing sequence with back-off / 基于限位开关的回零序列 (触碰+回退)
 * @ingroup bsp
 */

#ifndef __BSP_HOMING_H__
#define __BSP_HOMING_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Execute full homing sequence / 执行完整回零序列 */
bool BSP_Homing_Execute(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HOMING_H__ */
