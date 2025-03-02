#pragma once

/*
  MotionControl.h - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
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
*/

#include "Grbl.h"


// Sets up valid jog motion received from g-code parser, checks for soft-limits, and executes the jog.
// cancelledInflight will be set to true if was not added to parser due to a cancelJog.
Error jog_execute(plan_line_data_t* pl_data, parser_block_t* gc_block, bool* cancelledInflight);

#define HOMING_CYCLE_ALL 0  // Must be zero.

enum class SquaringMode : uint8_t {
    Dual,  // both motors run
    A,     // A motor runs
    B,     // B motor runs
};


// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position);
bool mc_line(float* target, plan_line_data_t* pl_data);  // returns true if line was submitted to planner
void mc_arc(float*            target,
            plan_line_data_t* pl_data,
            float*            position,
            float*            offset,
            float             radius,
            uint8_t           axis_0,
            uint8_t           axis_1,
            uint8_t           axis_linear,
            uint8_t           is_clockwise_arc);

bool mc_dwell(int32_t milliseconds);
void mc_homing_cycle(uint8_t cycle_mask);
GCUpdatePos mc_probe_cycle(float* target, plan_line_data_t* pl_data, uint8_t parser_flags);
void mc_realtime_override_motion(float* parking_target, plan_line_data_t* pl_data);
void mc_reset();

bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position);
bool kinematics_pre_homing(uint8_t cycle_mask);
void kinematics_post_homing();
bool limitsCheckTravel(float* target);  // weak in Limits.cpp; true if out of range
void IRAM_ATTR motors_to_cartesian(float* cartestian, float* motors, int n_axis);  // weak definition