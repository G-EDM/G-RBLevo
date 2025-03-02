/*
  Protocol.cpp - controls Grbl execution protocol and procedures
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

  2023 - Roland Lautensack (G-EDM) This file was heavily edited and may no longer be compatible with the default grbl

*/
#include "Grbl.h"
#include "gedm_sensors/sensors.h"
#include "widgets/ui_interface.h"

static char line[DEFAULT_LINE_BUFFER_SIZE];

typedef struct {
    char buffer[DEFAULT_LINE_BUFFER_SIZE];
    int  len;
    int  line_number;
} client_line_t;
client_line_t client_lines;

GRBL_PROTOCOL gproto;

GRBL_PROTOCOL::GRBL_PROTOCOL(){}

IRAM_ATTR void GRBL_PROTOCOL::empty_line() {
    client_line_t* cl = &client_lines;
    cl->len           = 0;
    cl->buffer[0]     = '\0';
}

IRAM_ATTR Error GRBL_PROTOCOL::add_char_to_line( char c ) {
    client_line_t* cl = &client_lines;
    if (c == '\b') {
        if (cl->len) {
            --cl->len;
            cl->buffer[cl->len] = '\0';
        }
        return Error::Ok;
    }
    if (cl->len == (DEFAULT_LINE_BUFFER_SIZE - 1)) {
        return Error::Overflow;
    }
    if (c == '\r' || c == '\n') {
        cl->len = 0;
        cl->line_number++;
        return Error::Eol;
    }
    cl->buffer[cl->len++] = c;
    cl->buffer[cl->len]   = '\0';
    return Error::Ok;
}

IRAM_ATTR Error GRBL_PROTOCOL::execute_line(char* line) {
    //ui.emit(LOG_LINE,line);
    Error result = Error::Ok;
    if (line[0] == 0) {
        return Error::Ok;
    }
    if (line[0] == '$' || line[0] == '[') {
        return system_execute_line(line);
    }
    if (sys.state == State::Alarm || sys.state == State::Jog) {
        return Error::SystemGcLock;
    }
    return gcode_core.gc_execute_line(line);
}


IRAM_ATTR void protocol_main_loop(void *parameter){ 
    GRBL_PROTOCOL *__this = (GRBL_PROTOCOL *)parameter;
    gserial.client_reset_read_buffer(CLIENT_ALL);
    __this->empty_line();
    if (sys.state == State::Alarm || sys.state == State::Sleep) {
        sys.state = State::Alarm;
    } else {
        sys.state = State::Idle;
        system_execute_startup(line);  
    }
    protocol_ready = true;
    int c;
    int dummy = 1;
    for (;;) {

        if (sys.abort) {
            grbl.reset_variables();
            vTaskDelay(1);
            continue;
        }

        //vTaskDelay(1000);continue;
        if( gconf.gedm_retraction_motion ){
            vTaskDelay(1);
            continue;
        }

        if( sd_line_shift || gconf.gedm_planner_line_running ){
           idle_timer = esp_timer_get_time();
        }
        char* line;
        while ((c = gserial.client_read(CLIENT_INPUT)) != -1) {
            sys.state = State::Cycle;
            Error res = __this->add_char_to_line(c);
            switch (res) {
                case Error::Ok:
                    break;
                case Error::Eol:
                    __this->protocol_exec_rt_system();
                    if (sys.abort) {
                        return;
                    }
                    line = client_lines.buffer;
                    __this->execute_line(line);
                    __this->empty_line();
                    break;
                case Error::Overflow:
                    report_status_message(Error::Overflow);
                    __this->empty_line();
                    break;
                default:
                    break;
            }
            idle_timer = esp_timer_get_time();
        }  

        int64_t timestamp_us = esp_timer_get_time();
        if( timestamp_us - idle_timer > 500000 ){
            idle_timer = timestamp_us;
            if( !gconf.gedm_planner_line_running ){
                sys.state = State::Idle;
            } else{
                sys.state = State::Cycle;
            }
        }
        __this->protocol_exec_rt_system();
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

IRAM_ATTR void GRBL_PROTOCOL::protocol_exec_rt_system() {
    ExecAlarm alarm = sys_rt_exec_alarm;  // Temp variable to avoid calling volatile multiple times.
    if( alarm != ExecAlarm::None ) {      // Enter only if an alarm is pending
        sys.state = State::Alarm;         // Set system alarm state
        report_alarm_message(alarm);
        sys_rt_exec_alarm = ExecAlarm::None;
    }
    ExecState rt_exec_state;
    rt_exec_state.value = sys_rt_exec_state.value;  // Copy volatile sys_rt_exec_state.
    if (rt_exec_state.value != 0 ) {   // Test if any bits are on
        // Execute system abort.
        if (rt_exec_state.bit.reset) {
            sys.abort = true;  // Only place this is set true.
            return;            // Nothing else to do but exit.
        }
        // NOTE: Once hold is initiated, the system immediately enters a suspend state to block all
        // main program processes until either reset or resumed. This ensures a hold completes safely.
        if (rt_exec_state.bit.motionCancel || rt_exec_state.bit.sleep) {
            // State check for allowable states for hold methods.
            if (!(sys.state == State::Alarm || sys.state == State::CheckMode)) {
                // If in CYCLE or JOG states, immediately initiate a motion HOLD.
                if (sys.state == State::Cycle || sys.state == State::Jog) {
                    if (!(sys.suspend.bit.motionCancel || sys.suspend.bit.jogCancel)) {  // Block, if already holding.
                        if (sys.state == State::Jog) {        // Jog cancelled upon any hold event, except for sleeping.
                            if (!rt_exec_state.bit.sleep) {
                                sys.suspend.bit.jogCancel = true;
                            }
                        }
                    }
                }
                // If IDLE, Grbl is not in motion. Simply indicate suspend state and hold is complete.
                if (sys.state == State::Idle) {
                    sys.suspend.value = 0;
                }
                // Execute and flag a motion cancel with deceleration and return to idle. Used primarily by probing cycle
                // to halt and cancel the remainder of the motion.
                if (rt_exec_state.bit.motionCancel) {
                    // MOTION_CANCEL only occurs during a CYCLE, but a HOLD and SAFETY_DOOR may been initiated beforehand
                    // to hold the CYCLE. Motion cancel is valid for a single planner block motion only, while jog cancel
                    // will handle and clear multiple planner block motions.
                    if (sys.state != State::Jog) {
                        sys.suspend.bit.motionCancel = true;  // NOTE: State is State::Cycle.
                    }
                    //sys_rt_exec_state.bit.motionCancel = false;
                }

            }
            if (rt_exec_state.bit.sleep) {
                sys.state                   = State::Sleep;
                sys_rt_exec_state.bit.sleep = false;
            }
        }

    }

}
