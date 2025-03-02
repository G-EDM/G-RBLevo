/*
  Grbl.cpp - Initialization and main loop for Grbl
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


//################################################
// GRBL_ESP32
//################################################
#include "Grbl.h"

GRBL_MAIN grbl;

GRBL_MAIN::GRBL_MAIN(){}

void GRBL_MAIN::grbl_init() {
    gserial.client_init();
    disableCore0WDT();
    disableCore1WDT();
    settings_init();
    motor_manager.init();
    memset(sys_position, 0, sizeof(sys_position));  // Clear machine position.
    machine_init();                                 // weak definition in Grbl.cpp does nothing
    sys.state = State::Idle;
#ifdef HOMING_INIT_LOCK
    if (homing_enable->get()) {
        sys.state = State::Alarm;
    }
#endif
    inputBuffer.begin();
}

void GRBL_MAIN::reset_variables() {
    State prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t));  // Clear system struct variable.
    sys.state             = prior_state;
    memset(sys_probe_position, 0, sizeof(sys_probe_position));  // Clear probe position.
    sys_probe_state                      = Probe::Off;
    sys_rt_exec_state.value              = 0;
    sys_rt_exec_alarm                    = ExecAlarm::None;
    gserial.client_reset_read_buffer(CLIENT_ALL);
    gcode_core.gc_init();
    glimits.limits_init();
    gcode_core.gc_sync_position();
    sys_pl_data_inflight = NULL;
}

void GRBL_MAIN::run_once() {
    //reset_variables();
    //gproto.protocol_main_loop();
}

void __attribute__((weak)) GRBL_MAIN::machine_init() {}

void __attribute__((weak)) GRBL_MAIN::user_m30() {}

void __attribute__((weak)) GRBL_MAIN::user_tool_change(uint8_t new_tool) {}

bool __attribute__((weak)) GRBL_MAIN::user_defined_homing(uint8_t cycle_mask) {
    return false;
}