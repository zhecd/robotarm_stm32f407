#include "app_isr_adapter.h"
#include "motion_service.h"
#include "waveform_service.h"
#include "limit_observation_service.h"
#include "app_hardware_adapter.h"
#include "limit_irq_guard.h"
#include <assert.h>
#include <stdio.h>
static unsigned motion_ticks, wave_ticks, motion_limits, wave_limits, mode_keys;
static uint8_t last_mask,last_wave_axis;
static unsigned diagnostic_edges;
static uint8_t last_diagnostic_axis;
static bool fuse;
static unsigned masked_count;
static uint8_t masked_axis;
static unsigned stopped;
static unsigned vector_traces;
void BSP_LimitTrace_CaptureVector(void) { ++vector_traces; }
bool LimitIrqGuard_OnEdgeFromISR(uint8_t axis) { (void)axis;return fuse; }
void AppHardware_MaskLimitInterrupt(uint8_t axis) { ++masked_count;masked_axis=axis; }
void AppHardware_StopActuators(void) { ++stopped; }
bool LimitObservation_OnEdgeFromISR(uint8_t axis)
{ ++diagnostic_edges; last_diagnostic_axis=axis; return fuse; }
void AppHardware_MaskDiagnosticLimitInterrupt(uint8_t axis)
{ ++masked_count;masked_axis=axis; }
void MotionService_OnStepTickFromISR(void) { ++motion_ticks; }
void MotionService_NotifyLimitSwitch(uint8_t mask) { ++motion_limits; last_mask=mask; }
void WaveformService_OnTickFromISR(void) { ++wave_ticks; }
void WaveformService_OnLimitFromISR(uint8_t axis) { ++wave_limits; last_wave_axis=axis; }
void AppRuntime_RequestModeSwitch(void) { ++mode_keys; }
int main(void)
{
    App_ISR_OnLimitVectorEnter();
#if ROBOTARM_WAVEFORM_MODE
    assert(vector_traces==1);
#else
    assert(vector_traces==0);
#endif
    TIM_HandleTypeDef timer={7U};
    App_ISR_OnTimerElapsed(&timer);
    assert(motion_ticks==0 && wave_ticks==0);
    timer.Instance=TIM6; App_ISR_OnTimerElapsed(&timer);
    const uint16_t pins[]={M1_STOP_Pin,M2_STOP_Pin,M3_STOP_Pin};
    for(unsigned i=0;i<3;i++) {
        App_ISR_OnGpioEdge(pins[i]);
#if ROBOTARM_WAVEFORM_MODE
        assert(motion_limits==0 && wave_limits==i+1U && last_wave_axis==i);
#elif ROBOTARM_DIAGNOSTIC_MODE
        assert(motion_limits==0 && wave_limits==0 && diagnostic_edges==i+1U && last_diagnostic_axis==i);
#else
        assert(motion_limits==i+1U && wave_limits==0 && last_mask==(1U<<i));
#endif
    }
    App_ISR_OnGpioEdge(KEY_MODE_Pin); assert(mode_keys==1);
    App_ISR_OnGpioEdge(0); assert(mode_keys==1);
    assert(masked_count==0);
    fuse=true;App_ISR_OnGpioEdge(M2_STOP_Pin);
#if ROBOTARM_DIAGNOSTIC_MODE && !ROBOTARM_WAVEFORM_MODE
    assert(masked_count==1 && masked_axis==1);
#else
    if (masked_count!=1 || masked_axis!=1 || stopped!=1) {
        fputs("FAIL: overloaded normal/wave IRQ was not masked and stopped\n",stderr);return 1;
    }
#endif
#if ROBOTARM_WAVEFORM_MODE
    assert(wave_ticks==1 && motion_ticks==0);
#elif ROBOTARM_DIAGNOSTIC_MODE
    assert(wave_ticks==0 && motion_ticks==0);
#else
    assert(wave_ticks==0 && motion_ticks==1);
#endif
    puts("isr_routing_tests: passed"); return 0;
}
