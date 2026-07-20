# 分层依赖规则

## 允许的依赖方向

```text
App -> Service -> Device -> BSP / Driver -> HAL
             -> Algorithm
             -> Platform
```

`Algorithm` 和 `Platform` 是可复用的侧向能力。它们不承担命令仲裁、设备控制或安全状态等业务职责。

## 关键边界

| 调用方 | 可使用的公开接口 | 禁止直接使用的接口 |
|---|---|---|
| CommandService | `planning_service.h`、`motion_service.h`、`safety_service.h` | `planning/internal/ctrl_planner.h` |
| PlanningService | `planning/internal/ctrl_planner.h` | Motion internal 头文件 |
| Ctrl_Planner | `motion_service.h` | `motion/internal/ctrl_motion_engine.h` |
| MotionService | Motion internal 头文件、`safety_service.h` | BSP 或 Device 的公开细节以外的上层服务 |
| SafetyService | `platform_critical.h` | `motion_service.h` |
| 普通 App 文件 | Service、Algorithm、Platform 的公开头文件 | `internal/`、HAL、BSP、Device |
| App adapters | App、Service、BSP、Device、CubeMX/HAL | 业务状态的重复存储 |

## 故障处理规则

1. Motion internal 检测到限位、编码器、软限位、队列超时或闭环发散时，必须先停止执行器。
2. `MotionService` 通过 `MotionService_TakeFaultEvent()` 向 App 交付故障原因。
3. AppRuntime 负责将故障原因映射为 `SafetyService_Report...()` 调用。
4. `SafetyService` 只锁存或清除安全状态，不直接操作运动执行器。
5. 回零成功后，AppRuntime 依次恢复 Safety 状态和 Motion 故障状态。

这些规则由 `scripts/check_architecture_dependencies.py` 进行静态检查。
