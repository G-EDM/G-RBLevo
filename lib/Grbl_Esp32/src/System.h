#pragma once
/*
  System.h - Header for system level commands and real-time processes
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

	2018 -	Bart Dring This file was modifed for use on the ESP32
		CPU. Do not use this with Grbl for atMega328P

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// Execution states and alarm
#include "Exec.h"



// System states. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
enum class State : uint8_t {
    Idle = 0,    // Must be zero.
    Alarm,       // In alarm state. Locks out all g-code processes. Allows settings access.
    CheckMode,   // G-code check mode. Locks out planner and motion only.
    Homing,      // Performing homing cycle
    Cycle,       // Cycle is running or motions are being executed.
    Hold,        // Active feed hold
    Jog,         // Jogging mode.
    Sleep,       // Sleep state.
};

// System suspend flags. Used in various ways to manage suspend states and procedures.
struct SuspendBits {
    uint8_t initiateRestore : 1;  // (Safety door only) Flag to initiate resume procedures from a cycle start.
    uint8_t restoreComplete : 1;  // (Safety door only) Indicates ready to resume normal operation.
    uint8_t motionCancel    : 1;     // Indicates a canceled resume motion. Currently used by probing routine.
    uint8_t jogCancel       : 1;        // Indicates a jog cancel in process and to reset buffers when complete.
};
union Suspend {
    uint8_t     value;
    SuspendBits bit;
};

typedef uint8_t AxisMask;  // Bits indexed by axis number
typedef uint8_t Percent;   // Integer percent
typedef uint8_t Counter;   // Report interval

enum class Override : uint8_t {
#ifdef DEACTIVATE_PARKING_UPON_INIT
    Disabled      = 0,  // (Default: Must be zero)
    ParkingMotion = 1,  // M56
#else
    ParkingMotion = 0,  // M56 (Default: Must be zero)
    Disabled      = 1,  // Parking disabled.
#endif
};

// Spindle stop override control states.
struct SpindleStopBits {
    uint8_t enabled : 1;
    uint8_t initiate : 1;
    uint8_t restore : 1;
    uint8_t restoreCycle : 1;
};
union SpindleStop {
    uint8_t         value;
    SpindleStopBits bit;
};

// Global system variables
typedef struct {
    volatile State state;               // Tracks the current system state of Grbl.
    bool           abort;               // System abort flag. Forces exit back to main loop for reset.
    Suspend        suspend;             // System suspend bitflag variable that manages holds, cancels, and safety door.
    bool           soft_limit;          // Tracks soft limit errors for the state machine. (boolean)
    bool           probe_succeeded;     // Tracks if last probing cycle was successful.
    AxisMask       homing_axis_lock;    // Locks axes when limits engage. Used as an axis motion mask in the stepper ISR.
    SpindleStop    spindle_stop_ovr;    // Tracks spindle stop override states
    Counter        report_ovr_counter;  // Tracks when to add override data to status reports.
    Counter        report_wco_counter;  // Tracks when to add work coordinate offset data to status reports.
    uint32_t       spindle_speed;
    bool           limit_isr_triggered;
} system_t;
extern DMA_ATTR system_t sys;

extern DMA_ATTR int64_t idle_timer;

// Control pin states
struct ControlPinBits {
    uint8_t reset : 1;
    uint8_t macro0 : 1;
    uint8_t macro1 : 1;
    uint8_t macro2 : 1;
    uint8_t macro3 : 1;
};
union ControlPins {
    uint8_t        value;
    ControlPinBits bit;
};

// NOTE: These position variables may need to be declared as volatiles, if problems arise.
extern DMA_ATTR int32_t sys_position[MAX_N_AXIS];        // Real-time machine (aka home) position vector in steps.
extern int32_t sys_probe_position[MAX_N_AXIS];  // Last probe position in machine coordinates and steps.
extern int32_t sys_probe_position_final[MAX_N_AXIS];  // final probe position in machine coordinates and steps.
extern bool    sys_probed_axis[MAX_N_AXIS];

extern volatile Probe              sys_probe_state;    // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
extern DMA_ATTR volatile ExecState sys_rt_exec_state;  // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
extern DMA_ATTR volatile ExecAlarm sys_rt_exec_alarm;  // Global realtime executor bitflag variable for setting various alarms.
extern volatile void* sys_pl_data_inflight;  // holds a plan_line_data_t while cartesian_to_motors has taken ownership of a line motion

extern volatile ExecAlarm sys_last_alarm;

void  system_execute_startup(char* line);
//Error execute_line(char* line);
Error system_execute_line(char* line);
Error system_execute_line(char* line);
Error do_command_or_setting(const char* key, char* value);
IRAM_ATTR void   system_flag_wco_change();
IRAM_ATTR int    system_convert_mm_to_steps(float mm, uint8_t idx);
IRAM_ATTR float  system_convert_axis_steps_to_mpos(int32_t steps, uint8_t idx);
IRAM_ATTR void   system_convert_array_steps_to_mpos(float* position, int32_t* steps);
IRAM_ATTR float* system_get_mpos();
IRAM_ATTR float  system_get_mpos_for_axis( int axis );