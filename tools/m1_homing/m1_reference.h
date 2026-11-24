/**
 * @file m1_reference.h
 * @brief M1局部参考点：只解释编码器坐标，不产生STEP、使能或整机运动许可。
 *
 * 参考点定义为：有限HOME确认触发并退回40微步、稳定释放后，首次重新读取
 * 编码器的位置。它不是限位接触点，不是正常三轴回零姿态，也不写Flash。
 * 复位即丢失；一次反馈/状态故障即失效，本次会话不能自动重新建立。
 */
#ifndef M1_REFERENCE_H
#define M1_REFERENCE_H
#include "m1_logic.h"

typedef enum {
    M1_REF_UNSET=0,       /* 尚未完成符合条件的HOME。 */
    M1_REF_WAIT_SAMPLE,   /* HOME完成，等待退回结束后发起的新读取。 */
    M1_REF_VALID,
    M1_REF_INVALID        /* 故障锁存，只能由显式初始化的新会话清除。 */
} M1ReferenceState;

typedef enum {
    M1_REF_OK=0,M1_REF_NOT_HOME,M1_REF_WAIT_FEEDBACK,M1_REF_BAD_FEEDBACK,
    M1_REF_LIMIT_ACTIVE,M1_REF_MOTION_FAULT,M1_REF_BAD_CONFIG,M1_REF_SAMPLE_TIMEOUT
} M1ReferenceReason;

typedef struct {
    M1ReferenceState state;
    M1ReferenceReason reason;
    float gear_ratio;          /* 电机轴转角 / 关节转角；必须为有限正数。 */
    float origin_motor_deg;    /* 当前启动会话内的电机轴展开角，不取模360。 */
    float relative_motor_deg;
    float relative_joint_deg;  /* 相对角=(当前展开角-参考展开角)/减速比。 */
    uint32_t waiting_since_ms,last_sample_started_ms;
} M1Reference;

typedef struct {
    uint32_t started_ms,finished_ms; /* 读取起止时间，不能用完成时间伪装成新采样。 */
    float motor_deg;
    bool valid,input_config_ok;
    uint8_t limit_mask;             /* 三路任一路触发，都不建立/保持有效参考。 */
} M1ReferenceSample;

void M1Reference_Init(M1Reference *r,float gear_ratio);
/** 主循环调用，识别HOME完成和故障；不在1ms中断中做浮点运算。 */
void M1Reference_ObserveState(M1Reference *r,const M1Test *motion,uint32_t now);
/** 先ObserveState，再发起读取，最后提交样本；不能提交跨越DONE转换的旧读取。
 * VALID之后仍提交新样本，用于相对角更新；不自动重设原点。
 */
void M1Reference_OnSample(M1Reference *r,const M1Test *motion,const M1ReferenceSample *sample);
#endif
