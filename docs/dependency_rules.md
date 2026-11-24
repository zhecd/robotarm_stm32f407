# 分层依赖规则

正常方向为 App → Service → Device → BSP/driver → HAL。Algorithm 只包含纯数学和机械臂参数；Platform 提供时间、延时和临界区。BSP 可读取算法配置，业务层不携带 STM32 句柄。

- App 普通文件通过服务、算法、平台和 AppHardware 端口工作；HAL 回调参数只在 App/adapters 解释。
- Service 内部头文件仅供相同服务域使用。例如 Command 不包含 planning/internal，Planner 不包含 motion/internal。
- Safety 不反向调用 Motion，立即停止由 Motion 负责，事件映射由 App 负责。
- Device 公共头文件及其间接包含链不得引入 Core/HAL。具体 USART6、TIM2_CH2 等资源绑定放在 bsp_robot_board.c。
- 禁止 Algorithm、Platform、BSP、Device 反向依赖上层业务。

```powershell
python -B scripts/check_architecture_dependencies.py
python -B tests/architecture_checker_tests.py
```

检查器解析源码直接 include，检查公共头文件的间接硬件引用，并处理注释和字符串。反例测试覆盖跨层、跨服务私有域、间接 HAL 泄漏和缺少源码目录。它不是完整 C 预处理器；宏展开、链接依赖和运行时行为仍由真实编译、测试及实机验收覆盖。
