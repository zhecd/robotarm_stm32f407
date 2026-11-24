# 上位机串口与视觉模块接口

本阶段暂停新增上位机、OpenCV和标定代码；相机位于机械臂末端并朝下。后续接入以[下位机串口合同](../../docs/serial-integration.md)为准。本目录已有客户端及离线计划为历史保留工具，不代表视觉功能已实现。

当前软件150：上位机保持115200，正常固件使用PROTOCOL1。recover默认等待240秒，必须收到本轮开始与完成握手，只发送一次M999；它会真实回零，不能当只读查询使用。参考撤销后M114不再提供旧空间坐标、M409标记UNAVAILABLE，成功读取原始反馈不恢复动作许可。传输对象readline须设置有限超时。

当前教学、功能和待实测范围见[软件交付清单](../../docs/software_completion.md)。下文80等阶段为历史证据；最新板卡状态由hardware_validation/latest.json确认，不根据本机源码推断已部署。


`robotarm_client.py`管理一问一答的串口会话，`feedback_snapshot.py`解析逐轴反馈，`gripper_snapshot.py`解析夹爪软件状态，`vision_sorting_plan.py`生成离线颜色分拣计划。导入这些模块和默认运行CLI均不连接设备，也不自动执行分拣。

软件90新增`client.preflight()`及纯解析/评估模块`task_preflight.py`，用于把任务前的状态整理成可解释的阻塞原因。它依次只读M115、M408、M119、M409、M410、M408；不支持的固件读完身份即返回，异常不重试、不发送M999或动作指令。使用及分阶段测试见[任务前检查](../../docs/teaching/task_preflight.md)。

软件92增加[真实C业务链的离线分拣预演](../../docs/teaching/sorting_preview.md)。`vision_sorting_preview.py`调用仅在电脑端构建的`robotarm_plan_check`，实际经过命令解析、命令服务、规划器、逆解和STEP引擎，设备与时钟使用测试替身。输入示例只是模型坐标，不能当作相机标定结果；预演成功不会连接机械臂或执行动作。

## 连接与固件身份

接口保持115200 bit/s、8N1、3.3V TTL。端口历史为COM3，最近实测为COM4；实际连接后先读M115确认固件身份。正常RobotArm、只读诊断和独立单轴固件的命令集合不同，不能根据端口号判断支持哪些命令。

正常/诊断/波形/AUDIT源码的M115声明FEEDBACK_AGE与GRIPPER_STATUS，分别表示支持M409与M410。阶段80已部署AUDIT1输出锁定版，M409/M410只读查询已验证；M409为FRAME:UNAVAILABLE，M410为未绑定状态，审计不执行夹爪初始化与回零。使用前必须核对实际M115能力，不能根据端口或本地源码判断当前板卡支持哪些命令。

## 反馈的三个层次

| 层次 | 接口/字段 | 含义 |
|---|---|---|
| 原始读取是否成功 | VALID_MASK | 最近一次设备读取结果 |
| 数据是否及时 | AGE_MS / FRESH_MASK | 逐轴从读取开始计时，默认年龄小于100ms |
| 坐标能否用于任务 | FRAME与M408 | JOINT且反馈新鲜仍需核对READY/HOMED/ALLOWED |

M409的一次回复同时包含三轴角度和各自年龄。三轴可以在不同时间采样，快照不代表同时读取。FRAME为RAW或UNAVAILABLE时，JOINT字段不可当作标定后的关节角；未采样轴年龄为4294967295。FeedbackSnapshot是不可变对象，缓存不会自动更新，读取时新鲜不代表以后始终新鲜。

在已经明确打开的传输对象上，可只读检查接口：

```python
from robotarm_client import RobotArmClient

client = RobotArmClient(opened_transport)  # 调用方提供write/readline，示例不打开串口。
identity = client.capabilities()
if 'FEEDBACK_AGE' not in identity:
    raise RuntimeError('当前固件未声明M409能力')
feedback = client.feedback()
status = client.status()
print(feedback.frame, feedback.age_ms, feedback.joint_data_fresh, status)
```

`joint_data_fresh`只表示本回复的三轴关节反馈新鲜，不是运动授权。M114继续保留原回复格式；READY模式下的PLAN是规划位置，不能冒充末端实际测量值。

## 命令会话与离线分拣

每个请求只发送一次。接收超时、错误或无法关联回复后，客户端将uncertain置为真，保留只读查询和停止路径；查询成功不会自动解除该状态。M999/recover会触发真实恢复/回零，不能当作只读查询自动调用。

视觉模块提供颜色和经过标定的机器人坐标，离线计划器负责生成抬升、平移、下降和夹爪命令序列。坐标标定、任务许可、机械范围和实际执行由后续集成验证完成；当前不会把计划器输出自动写入COM4。

电脑测试使用可替换传输对象，覆盖分片、协议错误、超时不重发，以及M409长度、数值、掩码和年龄一致性。可从工程根目录运行：

```powershell
D:\anaconda\python.exe -X utf8 -B tests/host_client_tests.py
```

## 夹爪软件状态

软件79新增M410，完整字段说明见[夹爪状态教学说明](../../docs/teaching/gripper_status.md)。在调用方已经提供的传输对象上，可只读查询：

```python
identity = client.capabilities()
if 'GRIPPER_STATUS' not in identity:
    raise RuntimeError('当前固件未声明M410能力')
gripper = client.gripper_status()
print(gripper.target, gripper.target_angle_deg, gripper.pwm_requested,
      gripper.elapsed_ms, gripper.error, gripper.hold_finished)
```

示例不创建连接、不发送动作。PWM_REQUESTED是驱动记录，hold_finished是软件保持窗口判定，POSITION始终UNMEASURED；二者都不能当夹物/到位传感器。查询不改变输出，不自动恢复通信许可。阶段80已部署含M410的AUDIT1锁定版，实测仅覆盖未绑定状态；保持中/停止错误等状态仍为PC验证。
