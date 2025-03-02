#pragma once

/*
  Limits.h - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

	2018 -	Bart Dring This file was modifed for use on the ESP32
					CPU. Do not use this with Grbl for atMega328P
  2018-12-29 - Wolfgang Lienbacher renamed file from limits.h to grbl_limits.h
          fixing ambiguation issues with limit.h in the esp32 Arduino Framework
          when compiling with VS-Code/PlatformIO.

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

extern uint8_t n_homing_locate_cycle;

class GRBL_LIMITS {
    public:
        GRBL_LIMITS( void );
        void  limits_init( void );
        void  limits_disable( void );
        void  limits_soft_check( float* target );
        void  limits_go_home( uint8_t cycle_mask );
        void  limitsCheckSoft( float* target );
        bool  limitsCheckTravel( float* target );
        float limitsMaxPosition( uint8_t axis );
        float limitsMinPosition( uint8_t axis );
        static IRAM_ATTR bool limits_get_state( void );
};

extern GRBL_LIMITS glimits;