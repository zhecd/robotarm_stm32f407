#ifndef ROBOTARM_FAKE_HARDWARE_H
#define ROBOTARM_FAKE_HARDWARE_H
#include "app_hardware_adapter.h"
extern char fake_tx[32768];
extern unsigned fake_normal_init_count;
extern unsigned fake_limit_indicator_updates;
extern bool fake_indicator_gcode,fake_indicator_ps2;
extern unsigned fake_diagnostic_init_count;
extern unsigned fake_configure_count;
extern unsigned fake_timer_start_count;
extern bool fake_ps2_valid;
extern bool fake_rx_overflow, fake_line_timeout, fake_tx_overflow;
extern unsigned fake_driver_reads;
extern unsigned fake_encoder_reads;
extern unsigned fake_limit_irq_reset_count;
extern unsigned fake_gripper_init_count;
extern unsigned fake_gripper_init_attempts;
extern bool fake_gripper_init_failure;
extern uint32_t fake_encoder_read_delay_us;
extern uint8_t fake_limit_irq_mask;
extern bool fake_driver_present[4];
extern bool fake_driver_prepare_ok;
extern AppTeleopInput_t fake_ps2_input;
extern AppTeleopDiagnostics_t fake_ps2_diagnostics;
void FakeHardware_Reset(void);
void FakeHardware_SendLine(const char *line);
void FakeHardware_ClearTx(void);
#endif
