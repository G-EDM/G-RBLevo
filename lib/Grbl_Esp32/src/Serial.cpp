/*
  Serial.cpp - Header for system level commands and real-time processes
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

	2018 -	Bart Dring This file was modified for use on the ESP32
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

  What is going on here?

  Original Grbl only supports communication via serial port. That is why this
  file is call serial.cpp. Grbl_ESP32 supports many "clients".

  Clients are sources of commands like the serial port or a bluetooth connection.
  Multiple clients can be active at a time. If a client asks for status, only the client will
  receive the reply to the command.

  The serial port acts as the debugging port because it is always on and does not
  need to be reconnected after reboot. Messages about the configuration and other events
  are sent to the serial port automatically, without a request command. These are in the
  [MSG: xxxxxx] format. Gcode senders are should be OK with this because Grbl has always
  send some messages like this.

  Important: It is up user that the clients play well together. Ideally, if one client
  is sending the gcode, the others should just be doing status, feedhold, etc.

  Clients send gcode, grbl commands ($$, [ESP...], etc) and realtime commands (?,!.~, etc)
  Gcode and Grbl commands are a string of printable characters followed by a '\r' or '\n'
  Realtime commands are single characters with no '\r' or '\n'

  After sending a gcode or grbl command, you must wait for an OK to send another.
  This is because only a certain number of commands can be buffered at a time.
  Grbl will tell you when it is ready for another one with the OK.

  Realtime commands can be sent at any time and will acted upon very quickly.
  Realtime commands can be anywhere in the stream.

  To allow the realtime commands to be randomly mixed in the stream of data, we
  read all clients as fast as possible. The realtime commands are acted upon and the other charcters are
  placed into a client_buffer[client].

  The main protocol loop reads from client_buffer[]

  2023 - Roland Lautensack (G-EDM) This file was heavily edited and may no longer be compatible with the default grbl

*/

#include "Grbl.h"

portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

GRBL_SERIAL gserial;

GRBL_SERIAL::GRBL_SERIAL(){

}

void GRBL_SERIAL::client_init() {
    client_reset_read_buffer(CLIENT_ALL);
}


void GRBL_SERIAL::client_reset_read_buffer(uint8_t client) {
    inputBuffer.begin();
}

IRAM_ATTR int GRBL_SERIAL::client_read(uint8_t client) {
    portENTER_CRITICAL(&myMutex);
    int data = inputBuffer.read();
    portEXIT_CRITICAL(&myMutex);
    return data;
}

bool GRBL_SERIAL::is_realtime_command(uint8_t data) {
    if (data >= 0x80) { return true; }
    auto cmd = static_cast<Cmd>(data);
    return cmd == Cmd::Reset || cmd == Cmd::StatusReport;
}