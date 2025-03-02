#pragma once

/*
  Protocol.h - controls Grbl execution protocol and procedures
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

IRAM_ATTR void protocol_main_loop(void *parameter);


class GRBL_PROTOCOL {

    private:

    public:
        GRBL_PROTOCOL( void );
        //IRAM_ATTR void protocol_main_loop( void );
        IRAM_ATTR void  protocol_exec_rt_system( void );
        IRAM_ATTR Error execute_line( char* line );
        IRAM_ATTR Error add_char_to_line( char c );
        IRAM_ATTR void empty_line( void );

};

extern GRBL_PROTOCOL gproto;