/* robotGeometry.c - V7.1 平行夹爪专用版 (含底座高度补偿) */
#include "robotGeometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 辅助宏
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

void RobotGeometry_Init(void) {
    // 平行夹爪结构不需要复杂的初始化预计算
}

/**
 * @brief 逆运动学解算 (平行夹爪结构 + 桌面坐标系支持)
 * 逻辑：
 * 1. 接收目标点 (相对于桌面的指尖坐标 G代码)
 * 2. 将 Z 轴转换为相对于肩关节的坐标 (减去 BASE_HEIGHT)
 * 3. 减去固定的水平和垂直偏移，得到"手腕坐标"
 * 4. 对"手腕坐标"进行标准的 140+140 等长双臂解算
 */
void RobotGeometry_CalculateAngles(float x, float y, float z, RobotAngles* angles) {
    if (angles == NULL) return;

    // =========================================================
    // [新增] 坐标系转换：桌面 -> 机械臂肩关节
    // =========================================================
    // G代码输入的 z=0 代表桌面，而算法认为 z=0 是肩关节高度
    // 所以需要向下偏移 BASE_HEIGHT
    float z_internal = z - BASE_HEIGHT;

    // =========================================================
    // 1. θ1 底座 (Base)
    // =========================================================
    // 旋转中心轴未变，直接计算
    angles->rot = atan2f(x, y);

    // =========================================================
    // 2. 坐标变换：从[指尖]推算[手腕]
    // =========================================================
    
    // 计算目标点的水平投影半径 (指尖的 R)
    float r_target = sqrtf(x * x + y * y);

    // 计算手腕的 R 坐标
    // 因为夹爪始终平行地面向外延伸，所以手腕一定在指尖的"后面"
    float r_wrist = r_target - TOOL_OFFSET_R;

    // 计算手腕的 Z 坐标
    // 注意：这里使用的是转换后的 z_internal
    float z_wrist = z_internal - TOOL_OFFSET_Z;

    // --- 特殊情况处理 ---
    // 如果目标点太近，导致减去夹爪长度后 r_wrist 变成负数
    // (意味着目标点比夹爪长度还短，物理上可能撞击底座)
    if (r_wrist < 1.0f) r_wrist = 1.0f; 

    // =========================================================
    // 3. 几何解算 (针对手腕坐标 r_wrist, z_wrist)
    // =========================================================
    // 现在问题回归到了标准的 140mm + 140mm 等长双臂结构
    
    // 手腕到原点的直线距离
    float dist_sq = r_wrist * r_wrist + z_wrist * z_wrist;
    float dist = sqrtf(dist_sq);

    // 限制最大臂长 (防止 NaN)
    // 这里的最大臂长是两个连杆之和 (140+140=280)
    if (dist > (LINK_1_LEN + LINK_2_LEN)) {
        dist = LINK_1_LEN + LINK_2_LEN;
    }

    // --- 计算三角形内角 Alpha ---
    // 因为 L1 == L2 (都是140)，这是等腰三角形，公式简化
    // cos(alpha) = (dist / 2L)
    float val = dist / (2.0f * LINK_1_LEN);
    float alpha_rad = acosf(CLAMP(val, -1.0f, 1.0f));

    // --- 计算目标向量仰角 Phi ---
    // 注意：这里用的是手腕的坐标
    float phi_rad = atan2f(r_wrist, z_wrist);

    // =========================================================
    // 4. θ2 大臂 (Shoulder)
    // =========================================================
    // 简单公式：总仰角 - 内部张角
    angles->low = phi_rad - alpha_rad;

    // =========================================================
    // 5. θ3 小臂 (Elbow)
    // =========================================================
    // 因为是等腰三角形，且 L1=L2
    // 我们可以直接算出肘部位置，再算小臂角度
    
    // 肘部位置
    float elbow_r = LINK_1_LEN * sinf(angles->low);
    float elbow_z = LINK_1_LEN * cosf(angles->low);

    // 向量：肘部 -> 手腕 (注意：是手腕，不是指尖！)
    float dr = r_wrist - elbow_r;
    float dz = z_wrist - elbow_z;

    // 小臂绝对角度 (相对于水平)
    angles->high = atan2f(dz, dr);

    // =========================================================
    // 6. 单位转换 (Rad -> Deg)
    // =========================================================
    angles->rot  = RAD_TO_DEG(angles->rot);
    angles->low  = RAD_TO_DEG(angles->low);
    angles->high = RAD_TO_DEG(angles->high);
}

void RobotGeometry_AnglesToMotorUnits(RobotAngles* angles, RobotMotorUnits* units) {
    if (angles == NULL || units == NULL) return;

    // 纯几何映射
    units->rotUnits  = (int32_t)(angles->rot  * UNITS_PER_DEGREE);
    units->lowUnits  = -(int32_t)(angles->low  * UNITS_PER_DEGREE);
    units->highUnits = -(int32_t)(angles->high * UNITS_PER_DEGREE);
}