#pragma once

#include "Config.h"
#include "stdint.h"

/*

#ifndef vTaskEnterCritical
#define vTaskEnterCritical taskENTER_CRITICAL
#endif

#ifndef vTaskExitCritical
#define vTaskExitCritical taskEXIT_CRITICAL
#endif

  Serial.h - Header for system level commands and real-time processes
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

extern portMUX_TYPE myMutex;

class GRBL_SERIAL {
    public:
        GRBL_SERIAL( void );
        void client_init( void );
        void client_reset_read_buffer( uint8_t client );
        bool is_realtime_command( uint8_t data );
        IRAM_ATTR int client_read( uint8_t client );

};

extern GRBL_SERIAL gserial;