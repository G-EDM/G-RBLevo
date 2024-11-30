#pragma once
/*
  Report.h - Header for system level commands and real-time processes
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

  2023 - Roland Lautensack (G-EDM) This file was heavily edited and may no longer be compatible with the default grbl

*/


// Define status reporting boolean enable bit flags in status_report_mask
enum RtStatus {
    Position = bit(0),
    Buffer   = bit(1),
};

const char* errorString(Error errorNumber);

// Define Grbl feedback message codes. Valid values (0-255).
enum class Message : uint8_t {
    CriticalEvent   = 1,
    AlarmLock       = 2,
    AlarmUnlock     = 3,
    Enabled         = 4,
    Disabled        = 5,
    CheckLimits     = 7,
    ProgramEnd      = 8,
    RestoreDefaults = 9,
    SpindleRestore  = 10,
    SleepMode       = 11,
    SdFileQuit      = 60,  // mc_reset was called during an SD job
};

#define CLIENT_SERIAL 0
#define CLIENT_INPUT  1
#define CLIENT_ALL    0xFF
#define CLIENT_COUNT  3  // total number of client types regardless if they are used

void   report_status_message( Error status_code );
void   report_alarm_message( ExecAlarm alarm_code );
void   IRAM_ATTR mpos_to_wpos( float* position );
float* get_wco( void );
