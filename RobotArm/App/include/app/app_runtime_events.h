/**
 * @file app_runtime_events.h
 * @brief ISR 到应用层的事件入口，主循环消费事件并处理模式切换。
 */
#ifndef ROBOTARM_APP_RUNTIME_EVENTS_H
#define ROBOTARM_APP_RUNTIME_EVENTS_H

void AppRuntime_RequestModeSwitch(void);

#endif /* ROBOTARM_APP_RUNTIME_EVENTS_H */
