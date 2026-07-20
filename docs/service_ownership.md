# 服务状态所有权

每一项可变状态只由一个服务或内部控制器写入，其他模块通过公开查询接口或事件读取。这一约束避免命令、规划、闭环和中断路径同时修改同一状态。

| 模块 | 拥有的状态 | 主要写入时机 |
|---|---|---|
| CommandService | 命令来源、规划坐标、进给速度、路径点 FIFO | G-code 或 PS2 命令被接受时 |
| PlanningService / Ctrl_Planner | 当前路径校验和流式生成状态 | 规划服务周期运行时 |
| MotionService / Ctrl_MotionEngine | 运动帧队列、当前 STEP 帧、理论步数、Motion 故障事件 | 主循环服务和 TIM6 中断 |
| Ctrl_ClosedLoop | 编码器滤波、闭环目标和恢复轨迹 | 闭环周期运行时 |
| StateService | 编码器采样和关节角度快照 | 编码器读取成功或失败时 |
| SafetyService | 回零标志、运动许可和锁存故障 | AppRuntime 处理 Motion 故障或回零完成时 |
| HomingService | 回零状态机 | 回零服务周期运行时 |
| GripperService | 夹爪服务动作与空闲停止 | M3/M5 或 PS2 输入时 |

`MotionService` 的理论步数表示已被规划器接受的电机侧目标，不等同于编码器实时测量。`StateService` 保存的是测量值；两者的偏差由闭环控制器处理。
