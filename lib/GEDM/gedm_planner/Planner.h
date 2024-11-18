#pragma once
/*
#  ██████        ███████ ██████  ███    ███  
# ██             ██      ██   ██ ████  ████  
# ██   ███ █████ █████   ██   ██ ██ ████ ██ 
# ██    ██       ██      ██   ██ ██  ██  ██ 
#  ██████        ███████ ██████  ██      ██ 
#
# This is a beta version for testing purposes.
# Only for personal use. Commercial use or redistribution without permission is prohibited. 
# Copyright (c) Roland Lautensack        
*/ 
#include "GCode.h"
#include "shared.h"

static const uint16_t POSITION_HISTORY_LENGTH = 1000;


typedef struct planner_config {
  bool hold_on_upper_setpoint      = false;
  bool reset_sense_queue           = false;
  int  line_to_line_confirm_counts = 10;
  int  early_exit_on_plan          = 3;
  int  max_reverse_depth           = 10;
  bool is_double_tap_retraction    = false;
} planner_config;


typedef struct protection_config {
  int broken_wire_steps = 0;

} protection_config;

typedef struct retraction_config {
  int soft_retract_start = 0;
  int hard_retract_start = 0;
  int steps_done         = 0;
  int steps_total        = 0;
  int disable_pwm        = 0;
  int repeat_count       = 0;
  int steps_case_3       = 0;
  int steps_case_4       = 0;
  int steps_case_5       = 0;
  int steps_case_6       = 0;
  int steps_case_0       = 0;
  bool pwm_off_case_5    = true;
  bool pwm_off_case_3    = false;
} retraction_config;


typedef struct {
  float delta   = 0;
  long  steps   = 0;
  long  counter = 0;
} Axis_Direct;


typedef struct {
  uint8_t direction_bits = 0;
  uint8_t step_bits      = 0;
  long    counter        = 0;
} micro_history;

typedef struct {
    int     tmp                     = 0; // used for random stuff
    bool    ignore_limit_switch     = 1;  // grbl has a background task running for limits, but the new planner is not async and while homing it needs a custom limits check
    bool    ignore_xy_motion        = 0;  
    bool    ignore_z_motion         = 0;
    bool    ignore_feed_limit       = 0;
    bool    enable_position_history = 0;
    bool    enable_flushing         = 0;
    bool    skip_feed               = 0; 
    bool    skip_floating           = 0;
    bool    motion_plan_enabled     = 0;
    bool    floating_enabled        = 0;
    int     step_delay              = 0;
    int     last_motion_plan        = 0;
    int     setpoint_step_position  = 0;
    int     failure_step_pos        = 0;
    long    step_event_count        = 0;
    int     line_has_first_contact  = 0;
    uint8_t direction_bits          = 0;
    uint8_t direction_bits_inverted = 0;
    uint8_t step_bits               = 0;
    int     step_count              = 0;
    int     accel_rounds            = 0;
    bool    exit_on_contact         = 0;
    bool    enable_no_work_finish   = 0;
    bool    simulation_line         = 0;
    bool    is_arc                  = 0;
    Axis_Direct line_math[N_AXIS];
} Line_Config;



typedef struct {
    int arc_counter;
    bool z_axis_is_up;
    bool wire_has_first_contact;
    bool skip_feed;
    bool is_flushing;
    int  motion_plan;
    int  total_retraction_steps;
    int  true_retraction_steps;
    int  real_short;
    int  sinker_direction;
} planner_state;

extern DMA_ATTR volatile planner_config plconfig;



class G_EDM_PLANNER{


    public:
      G_EDM_PLANNER( void );
      void     set_sinker_direction( int direction );
      void     set_retraction_pwm_off( int _case, bool off );
      bool     get_retraction_pwm_off( int _case );

      bool     IRAM_ATTR protection_logic( void );

      void     IRAM_ATTR convert_target_to_steps( float* target, int32_t* __target );
      void     IRAM_ATTR override_target_with_current( float* target );
      uint16_t IRAM_ATTR push_to_position_history( int32_t* target, bool override, int override_index );
      uint8_t  IRAM_ATTR plan_history_line( float* target, plan_line_data_t* pl_data );
      uint16_t IRAM_ATTR get_current_work_index( void );
      void     IRAM_ATTR position_history_force_sync( void );
      int      IRAM_ATTR get_used_motion_plan( void );
      void     position_history_reset( void );
      void     push_break_to_position_history( void );
      void     push_current_mpos_to_position_history( void );
      int      position_history_undo( int count );
      bool     position_history_sync( Line_Config &line );
      void     set_ignore_breakpoints( bool ignore_break );
      void     IRAM_ATTR set_retraction_steps( void );
      void init( void );
      void set_sinker_axis( int axis );
      void feed_stop( bool stop_feed );
      void reset_planner_state( void );
      void update_max_feed( void );
      bool is_ready( void );
      void configure( void );
      bool get_is_paused( void );
      void next_gcode_round( void );
      void set_cutting_depth_limit( float _mm );
      bool retraction( int stepper, float travel_mm, float offset_mm, bool disable_pwm );
      void z_axis_up();   // acts like a pen up to change the toolpath (M3 GCode) It uses the probe position + the offset
      void z_axis_down(); // acts like pen down after a change of the toolpath (M4 GCode)
      bool z_axis_move_mm( float mm );
      bool reprobing_routine( void );
      int  get_current_round( void );

    private:

      bool IRAM_ATTR is_end_of_line( Line_Config &line );
      bool IRAM_ATTR history_real_short( Line_Config &line, bool has_minimal_steps );
      bool IRAM_ATTR history_soft_short( Line_Config &line, bool has_minimal_steps );
      bool IRAM_ATTR process_wire( Line_Config &line );

      bool IRAM_ATTR history_real_short_sinker( Line_Config &line, bool has_minimal_steps );
      bool IRAM_ATTR history_soft_short_sinker( Line_Config &line, bool has_minimal_steps );
      bool IRAM_ATTR process_sinker( Line_Config &line );
      bool IRAM_ATTR pre_process_history_line( Line_Config &line );
      void IRAM_ATTR pre_process_history_line_forward( Line_Config &line );


      void IRAM_ATTR reset_rconf( void );
      void benchmark_reaction_time( void );
      int sinker_axis;
      void  add_accel( Line_Config &line );
      float get_mm_up_possible( float mm_up_wanted );
      void  stepper_step_custom( int axis, bool dir, int _step_delay );
      void  engraver_retraction( int _case, int axis, int step_delay, Line_Config &line );
      bool IRAM_ATTR short_circuit_estop( void );

      bool IRAM_ATTR work_is_at_cutting_depth_limit( void );
      bool IRAM_ATTR no_workload_finish_reached( void );
      bool IRAM_ATTR do_flush_if_needed( void );
      int  IRAM_ATTR get_motion_plan( void );
      bool IRAM_ATTR probe_check( Line_Config &line );
      bool IRAM_ATTR floating_edm_engraver( int _step_delay, Line_Config &line );
      bool IRAM_ATTR pre_process_floating( int _step_delay, Line_Config &line );
      void IRAM_ATTR unshort_hard( Line_Config &line );
      void IRAM_ATTR wire_line_end( Line_Config &line );
      bool IRAM_ATTR move_line( int32_t* target, Line_Config &line );
      bool IRAM_ATTR process_stage( int32_t* target, Line_Config &line );
      bool IRAM_ATTR position_history_create_microhistory( Line_Config &line );
      bool IRAM_ATTR history_retraction( int _case, Line_Config &line );
      uint16_t position_history_get_previous( bool peek = false ); // used by the ringbuffer internally
      uint16_t position_history_get_next( void );     // used by the ringbuffer internally
      uint16_t position_history_work_get_previous( bool peek = false ); // used by the user
      uint16_t position_history_work_get_next( bool peek = false );     // used by the user
      bool     position_history_is_break( int32_t* target );
      bool     future_position_history_is_at_final_index( bool reverse = false );
      bool     position_history_is_at_final_index( void );
      bool     position_history_move_back( void );
      bool     position_history_move_forward( bool no_motion, Line_Config &line );
      void     history_backup_recover_target( void );
      bool     current_position_is_at_target( void );
      int      get_success_direction( int last_direction );
      int      get_failure_direction( int last_direction );
      bool     process_direction( int direction, Line_Config &line );
      void     pause( bool redraw = false );
      void     resume( void );
      void     prepare( float* target );
      bool     add_step_delay( int _step_delay, bool enable_motion_plan = false );
      int32_t  position_history[ POSITION_HISTORY_LENGTH ][MAX_N_AXIS];
      //int16_t  position_history_line_type[ POSITION_HISTORY_LENGTH ];
      uint16_t position_history_recover_index;
      uint16_t position_history_index;
      uint16_t position_history_index_current;
      uint16_t position_history_final_index; 
      bool     position_history_is_between;
      bool     position_history_is_forward;
      bool     planner_is_synced;
      bool     position_history_ignore_breakpoints;
      uint16_t position_history_recover_fail_count;
      bool     history_is_recovering;
      bool was_paused;
      int pause_recover_count;
      int64_t short_circuit_start_time; // after a given amount of time within a short circuit force the process to pause
      int64_t last_step_micros[MAX_N_AXIS];
      int64_t last_step_timestamp;
      int64_t microseconds;
      bool short_circuit_retraction;
      bool short_circuit_recover;
      int max_reverse_depth;
      int has_reverse;
      int32_t* recover_target;
      uint16_t short_trigger_target_index;
      bool recovered;
      plan_line_data_t  default_plan_data;
      plan_line_data_t* default_pl_data;
      bool is_paused;
      int    last_workload_at;
      int    travel_in_first_pass; // monitor the travel of Z in the first pass in steps
      int    deepest_step_pos;         // deepest travel of z into the work piece in the first pass
      bool   enable_wire_mode;
      int    step_limit;
      int    no_workload_steps_to_finish;
      int    flushing_total_steps;
      bool   stop_on_contact;
      int    cutting_depth_step_final;
      float  z_cutting_depth;
      int    gcode_round;
      bool init_ready;

};

extern G_EDM_PLANNER planner;