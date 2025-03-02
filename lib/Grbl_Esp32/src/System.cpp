#include "Grbl.h"
#include "Config.h"


// Declare system global variable structure
DMA_ATTR system_t      sys;
DMA_ATTR int32_t       sys_position[MAX_N_AXIS];        // Real-time machine (aka home) position vector in steps.
int32_t                sys_probe_position[MAX_N_AXIS];  // Last probe position in machine coordinates and steps.
int32_t                sys_probe_position_final[MAX_N_AXIS];
bool                   sys_probed_axis[MAX_N_AXIS];
volatile ExecAlarm     sys_last_alarm;
volatile Probe         sys_probe_state;                 // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
DMA_ATTR volatile ExecState sys_rt_exec_state;  // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
DMA_ATTR volatile ExecAlarm sys_rt_exec_alarm;  // Global realtime executor bitflag variable for setting various alarms.
volatile void*              sys_pl_data_inflight;  // holds a plan_line_data_t while cartesian_to_motors has taken ownership of a line motion
#ifdef DEBUG
volatile bool sys_rt_exec_debug;
#endif

DMA_ATTR int64_t idle_timer = esp_timer_get_time();
IRAM_ATTR void system_flag_wco_change() {
    sys.report_wco_counter = 0;
}
/** this is not accurate used only for rough calculations! **/
IRAM_ATTR int system_convert_mm_to_steps(float mm, uint8_t idx) {
    int steps = int( mm * axis_settings[idx]->steps_per_mm->get() );
    return steps;
}
IRAM_ATTR float system_convert_axis_steps_to_mpos(int32_t steps, uint8_t idx) {
    float pos;
    float steps_per_mm = axis_settings[idx]->steps_per_mm->get();
    pos                = steps / steps_per_mm;
    return pos;
}

// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
IRAM_ATTR void system_convert_array_steps_to_mpos(float* position, int32_t* steps) {
    auto  n_axis = N_AXIS;
    float motors[n_axis];
    for (int idx = 0; idx < n_axis; idx++) {
        motors[idx] = (float)steps[idx] / axis_settings[idx]->steps_per_mm->get();
    }
    motors_to_cartesian(position, motors, n_axis);
    // mtoc is just memcpy(cartesian, motors, n_axis * sizeof(motors[0]));
}
IRAM_ATTR float* system_get_mpos() {
    static float position[MAX_N_AXIS];
    system_convert_array_steps_to_mpos(position, sys_position);
    return position;
};
IRAM_ATTR float system_get_mpos_for_axis( int axis ) {
    return (float)sys_position[axis] / axis_settings[axis]->steps_per_mm->get();
};
