# 命名约定

## 目录与文件

- 目录使用首字母大写的架构名：`App`、`Service`、`Domain`、`Device`、`Driver`、`BSP`、`Platform`。
- C 文件与头文件使用小写蛇形命名，例如 `motion_service.c`、`joint_device.h`。
- App 任务使用 `app_<topic>_task.c`；硬件适配使用 `app_<interface>_adapter.c`。
- 服务接口使用 `<Name>Service_<Verb>()`，例如 `MotionService_SubmitMove()`。
- 设备接口使用 `<Name>Device_<Verb>()`，例如 `JointDevice_ReadMotorAngle()`。
- Driver 接口使用 `<ChipOrProtocol>Driver_<Verb>()`，例如 `As5600Driver_ReadAngle()`。

## 类型与函数

- 公开结构体使用 `<module>_status_t`、`<module>_config_t`、`<module>_result_t`。
- 布尔函数以 `Is`、`Has`、`Can` 或 `Get` 开头。
- 来自 ISR 的入口以 `FromISR` 结尾，例如 `MotionService_OnStepTickFromISR()`。
- 不在新代码中继续增加 `Ctrl_`、`Svc_`、`Dev_`、`Drv_` 前缀；现有前缀在行为保持型迁移期间保留，避免不必要的协议和符号变更。
