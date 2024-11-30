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


/***
 * 
 * Todo: Convert all float based positions to raw int step positions
 * 
 * 
*/

#include "Grbl.h"
#include <stdlib.h>  
#include "tft_display/ili9341_tft.h"


planner_state DRAM_ATTR _state;
retraction_config DRAM_ATTR rconf;
DMA_ATTR volatile planner_config plconfig;


G_EDM_PLANNER planner = G_EDM_PLANNER();
G_EDM_PLANNER::G_EDM_PLANNER(){
}

int IRAM_ATTR G_EDM_PLANNER::get_used_motion_plan(){
    return _state.motion_plan;
}

int DRAM_ATTR tmp_counter = 0;
int DRAM_ATTR first_contact_counter       = 0;
int DRAM_ATTR last_plan                   = 0;
int DRAM_ATTR short_circuit_estop_reset_count = 0; // to prevent a single wrong ADC reading from resetting the timer

int DRAM_ATTR no_load_steps = 0;

int DRAM_ATTR plan_counter[7] = {0,0,0,0,0,0,0};

void IRAM_ATTR toggle_spindle( bool turn_off ){
    bool spindle_is_on = ui_controller.spindle_is_running();
    if( ( turn_off && !spindle_is_on ) || ( !turn_off && spindle_is_on ) ){
        return;
    }
    int data = turn_off ? 6 : 7;
    xQueueSend( sensor_queue_main, &data, 50000 );
    while( ( turn_off ? ui_controller.spindle_is_running() : !ui_controller.spindle_is_running() ) ){ 
        if( sys.abort || sys_rt_exec_state.bit.motionCancel || !edm_process_is_running ){ break; }
        delayMicroseconds(10); 
    } 
}
void IRAM_ATTR toggle_pwm( bool turn_off, int delay ){
    bool pwm_is_off = ui_controller.get_pwm_is_off();
    if( turn_off == pwm_is_off ){ delayMicroseconds( delay ); return; }

    if( turn_off ){
        ui_controller.pwm_off();
    } else {
        ui_controller.pwm_on();
    }
    delayMicroseconds( delay );
    return;
}

//###############################################
// Set/Reset short circuit timer/counter
//###############################################
bool IRAM_ATTR G_EDM_PLANNER::protection_logic(){ 
    if( _state.motion_plan >= 4 ){
        if( short_circuit_start_time == 0 ){
            short_circuit_start_time = esp_timer_get_time();
        }
        short_circuit_estop_reset_count = 0;
    } else if ( 
        // _state.motion_plan == 1 ||
        ( _state.motion_plan < 3 && ++short_circuit_estop_reset_count > 4 )
    ){
        short_circuit_start_time        = 0;
        short_circuit_estop_reset_count = 10;
    }
    
    if( _state.motion_plan == 5 ){
        _state.real_short  = 1;
    } else {
        _state.real_short = 0;
    }

    return true; 
}


//###############################
// Todo:
// New concept: not done yet
// 1 = no load
// 2 = custom hold overwrite
// 3 = load
// 4 = little too much load
// 5 = serious short condition
//###############################
int IRAM_ATTR G_EDM_PLANNER::get_motion_plan(){

    bool check_broken_wire = false;
    bool count_load        = true;

    last_plan           = _state.motion_plan;
    _state.motion_plan  = G_EDM_SENSORS::get_calculated_motion_plan();// ui_controller.get_motion_plan();
    
    //################################################################
    // Check for negative plan (edge not recovered within timeframe)
    //################################################################
    if( _state.motion_plan < 0 ){
        _state.motion_plan *= -1; // invert it back
       if( _state.motion_plan <= 2 ){
          check_broken_wire = true;
          count_load        = false;
       }
    }

    if( _state.motion_plan == 2 && count_load ){
        --no_load_steps;
    }

    //##########################################
    // Reset no-load steps and prevent overflow
    //##########################################
    if( _state.total_retraction_steps > 0 ){
        no_load_steps = 0;
    }
    if( no_load_steps > rconf.steps_case_0 ){
        no_load_steps = rconf.steps_case_0;
    }

    //####################################
    // If motion queue timed out retry
    // a timeout will return 9 as plan
    //####################################
    if( _state.motion_plan == 9 && !gconf.edm_pause_motion ){
        tmp_counter = 0;
        while( _state.motion_plan == 9 ){
            if( sys.abort || sys_rt_exec_state.bit.motionCancel || !edm_process_is_running ){ break; }
            toggle_pwm( true, 100 );
            toggle_pwm( false, 2 );
             _state.motion_plan = G_EDM_SENSORS::get_calculated_motion_plan();
             if( ++tmp_counter > 10 || gconf.edm_pause_motion ){
                gconf.process_error    = 4;
                gconf.edm_pause_motion = true;
                _state.motion_plan     = 4;
                _state.real_short      = 1;
                break;
            }
        }

    }
    
    if( _state.motion_plan == 1 || check_broken_wire ){
        //#######################
        // Free to move forward
        //#######################
        plan_counter[1] = 0;

        //##############################
        // could be a broken wire too
        //##############################
        if( 
            ( no_load_steps >= rconf.steps_case_0 )
            && gconf.gedm_wire_gcode_task           // only in wire mode
            && _state.wire_has_first_contact        // only after it has the first contact
            && _state.total_retraction_steps <= 5   // only if there are no retraction leftovers
            && position_history_is_at_final_index() // only if it is at the final index
            && !gconf.gedm_retraction_motion        // trigger wire break only in forward motion
        ){
            gconf.process_error    = 1;
            gconf.edm_pause_motion = true;
            _state.motion_plan     = 4;
            _state.real_short      = 1;
            no_load_steps          = 0;
        }
        
    } else if( _state.motion_plan > 1 ){
        //############################
        // Some kind of load going on
        //############################
        // prevent jitter from messing around
        if( 
            _state.motion_plan > 2 
            || ( _state.motion_plan == 2 && ++plan_counter[1] > 2 ) 
        ){
          no_load_steps = 0;
        }

        //#####################################################################################################
        // Short circuit protection plan 4 is just above setpoint. Plan 5 is serious and needs a fast response
        //#####################################################################################################
        last_workload_at = sys_position[sinker_axis];

    }

    protection_logic();

    return _state.motion_plan;
}



// used only to limit retraction
// retracting around edges is a problem if the wire is stuck
// it is better to not do it and try other methods
// if nothing helps to recover from a short we enter a pause which is still better then making it worse
bool IRAM_ATTR G_EDM_PLANNER::is_end_of_line( Line_Config &line ){
    return line.step_count >= line.step_event_count ? true : false;
}


bool IRAM_ATTR G_EDM_PLANNER::process_wire( Line_Config &line ){

    if( !pre_process_history_line( line ) ){

        return false;

    } else if( gconf.gedm_retraction_motion ){

        line.step_delay        = process_speeds.WIRE_RETRACT_SOFT;
        line.ignore_feed_limit = false;
        bool end_of_line       = is_end_of_line( line );

        if( ui_controller.get_pwm_is_off() ){
            toggle_pwm( false, 2 );
        }
        get_motion_plan();
        if( _state.motion_plan > plconfig.early_exit_on_plan ){
            rconf.early_exit_confirmations = 0;
        }
        if( 
            retconf.early_exit 
            && _state.motion_plan <= plconfig.early_exit_on_plan
        ){ 
            if( ++rconf.early_exit_confirmations > 1 ){
                return false;
            }
        }

        if( rconf.soft_retract_start || rconf.hard_retract_start ){
            line.ignore_feed_limit   = true;

            line.step_delay          = _state.real_short 
                                       ? process_speeds.WIRE_RETRACT_HARD 
                                       : process_speeds.WIRE_RETRACT_SOFT;

            if( --rconf.steps_total>0 ){ 
                return true; 
            } 
        }

        if( _state.motion_plan < 3 && !_state.real_short ){
            return false;
        } 
        return true;
        
    } else {

        rconf.early_exit_confirmations = 0;
        
        pre_process_history_line_forward( line );

        switch ( _state.motion_plan )
        {

            case 5: // 
                rconf.steps_total        = rconf.steps_case_5;
                rconf.hard_retract_start = 1;
                return false;
            break; // unreachable code

            case 4: // 
                rconf.steps_total        = rconf.steps_case_3;
                rconf.soft_retract_start = 1;
                return false;
            break; // unreachable code
        
            case 3: // 
                line.skip_feed  = true;
                line.step_delay = process_speeds.EDM;
            break;

            case 2: // 
                line.skip_feed  = true;
                line.step_delay = process_speeds.EDM;
            break;

            default: // 
                line.skip_feed  = false;
                line.step_delay = process_speeds.EDM;
            break;

        }

        if( ! _state.wire_has_first_contact ){
            no_load_steps          = 0;
            line.ignore_feed_limit = true;
            line.step_delay        = round(process_speeds.RAPID*2);
        } 


    }
    return true;
}

bool IRAM_ATTR G_EDM_PLANNER::floating_edm_engraver( int _step_delay, Line_Config &line ){
    int steps      = 0;
    int steps_min, steps_max;
    bool direction = false;
    switch ( _state.motion_plan )
    {
        case 5:
        case 4:
            line.skip_feed     = true;
            line.skip_floating = true; // step up
            // direction false for up and true for down
            toggle_pwm( true, 4 );
            engraver_retraction( 4, Z_AXIS, process_speeds.RETRACT, line );
            toggle_pwm( false, 2 );
            last_workload_at = sys_position[Z_AXIS]; 
        break;

        case 3:
            line.skip_feed     = true;
            line.skip_floating = true; // step up
            // direction false for up and true for down
            engraver_retraction( 3, Z_AXIS, process_speeds.EDM, line );
            last_workload_at = sys_position[Z_AXIS]; 
        break;

        case 2:
            line.skip_floating = true;
            line.setpoint_step_position = sys_position[Z_AXIS];
        break;

        default:
            if( line.last_motion_plan != _state.motion_plan ){ // skip the down movement until it is confirmed in the enxt round
                line.skip_floating = true;
            } else{
                direction  = true; // step down
            }
        break;

    }
    line.last_motion_plan = _state.motion_plan;
    /** Floating the z axis **/
    if( !line.skip_floating ){
        if( line.failure_step_pos != 0 && sys_position[Z_AXIS] <= line.failure_step_pos  ){
            // maybe remove this?
            // if a line created a short this failure position is the max depth for the rest of the line
            //return false;
        }
        microseconds = esp_timer_get_time();
        if( direction ){
            if(
                // conditions for a single step down
                !work_is_at_cutting_depth_limit() 
                && !no_workload_finish_reached()
                && ( microseconds - last_step_micros[Z_AXIS] > max_feeds_micros[ Z_AXIS ] )
            ){ stepper_step_custom( Z_AXIS, direction, _step_delay ); }
        }
    }
    return false;
}

bool IRAM_ATTR G_EDM_PLANNER::pre_process_floating( int _step_delay, Line_Config &line ){
    get_motion_plan(); 
    if( line.floating_enabled ){
        if( _state.motion_plan >= 3 ){
            unshort_hard( line );
        } else if( _state.motion_plan <= 3 ){
            floating_edm_engraver( _step_delay, line );
        }
    } 
    return true;
}


void IRAM_ATTR G_EDM_PLANNER::wire_line_end( Line_Config &line ){
    int pass_counter = 0;
    int total_counts = round(plconfig.line_to_line_confirm_counts/2);
    if( position_history_is_at_final_index() ){ 
        if( _state.arc_counter == 0 ){
            total_counts = plconfig.line_to_line_confirm_counts;
        }
    }
    // at the final position
    if( ui_controller.get_pwm_is_off() ){
        toggle_pwm( false, 2 );
    }        
    while( true ){
        get_motion_plan();
        if( _state.motion_plan <= 2 ){
            ++pass_counter;
        } else {
            pass_counter = 0;
        }
        if( 
            pass_counter >= total_counts 
            || _state.motion_plan >= 4 
            || sys.abort 
            || sys_rt_exec_state.bit.motionCancel
        ){ break; }
    }
}

/** 
  * called while stepping and if probing is active; checks the probe state and 
  * inserts a pause until a positive is confirmed 
  * see sensors.cpp for more details
  **/
bool IRAM_ATTR G_EDM_PLANNER::probe_check( Line_Config &line ){
    if (sys_probe_state == Probe::Active) {
        no_load_steps = 0; // reset wire break for probing
        line.step_delay = process_speeds.PROBING;
        get_motion_plan();
        line.motion_plan_enabled     = false;
        line.enable_position_history = false;
        ui_controller.reset_flush_retract_timer();
        toggle_pwm( false, 4 );
        if( _state.motion_plan > 1 ){
            line.skip_feed = true;
        } else{
            line.skip_feed = false;
        }
        if( gprobe.probe_state_monitor() ){
            toggle_pwm( true, 0 );
            sys_probe_state = Probe::Off;
            return true;
        }
    }
    return false;
}

/** 
  * If possible hammer the position vertically until the short circuit position is free 
  * if the max number of rounds are exceeded is switches pwm on and off for some rounds and then goes on
  * with the normale process 
  **/
void IRAM_ATTR G_EDM_PLANNER::unshort_hard( Line_Config &line ){
    line.failure_step_pos = sys_position[Z_AXIS];
    line.failure_step_pos++; // remove a step
    bool recovered = false;
    int max_rounds  = 15;
    int rounds_done = 0;
    while( !recovered && _state.motion_plan >= 3 ){
        if( sys.abort || sys_rt_exec_state.bit.motionCancel ){
            recovered = false;
            break;
        }
        get_motion_plan(); 
        floating_edm_engraver( process_speeds.REPROBE, line );
        if( sys_position[Z_AXIS ] == line.failure_step_pos ){
            recovered = true;
        } else{
            recovered = false;
        }
        if( ++rounds_done > max_rounds ){
            if( _state.motion_plan >= 3 ){
                for(int i=0; i < 10; ++i ){
                    toggle_pwm( true, 500 );
                    toggle_pwm( false, 10 );
                }
            }
            recovered = false;
            toggle_pwm( false, 0 );
            break;
        }
    }
}


void IRAM_ATTR G_EDM_PLANNER::pre_process_history_line_forward( Line_Config &line ){
    reset_rconf();
    if( ui_controller.get_pwm_is_off() ){
        toggle_pwm( false, 2 );
    }
    get_motion_plan(); 
    if( _state.motion_plan > 2 ){
        // flag first contact
        last_workload_at = sys_position[sinker_axis]; // i know... hacky
        if( !_state.wire_has_first_contact ){
            ui_controller.reset_flush_retract_timer();
            _state.total_retraction_steps = 0;
            if( ++first_contact_counter > 1 ){
            //if( ++first_contact_counter > 0 ){
                _state.wire_has_first_contact = true;
            }
        } 
    } else {
        if( !_state.wire_has_first_contact ){
            //first_contact_counter = 0;
            ui_controller.reset_flush_retract_timer();
        }
    }
}







void IRAM_ATTR G_EDM_PLANNER::set_retraction_steps(){
    rconf.steps_case_3 = MAX(2,motor_manager.convert_mm_to_steps( retconf.soft_retraction,   ( gconf.gedm_wire_gcode_task ? X_AXIS : sinker_axis ) ));
    rconf.steps_case_4 = MAX(2,motor_manager.convert_mm_to_steps( retconf.medium_retraction, ( gconf.gedm_wire_gcode_task ? X_AXIS : sinker_axis ) ));
    rconf.steps_case_5 = MAX(2,motor_manager.convert_mm_to_steps( retconf.hard_retraction,   ( gconf.gedm_wire_gcode_task ? X_AXIS : sinker_axis ) ));
    rconf.steps_case_0 = motor_manager.convert_mm_to_steps( retconf.wire_break_distance, ( gconf.gedm_wire_gcode_task ? X_AXIS : sinker_axis ) );
}





void G_EDM_PLANNER::init(){
    enable_wire_mode         = false;
    z_cutting_depth          = 0.0;
    deepest_step_pos         = 0;
    gconf.gedm_retraction_motion = false;
    default_pl_data = &default_plan_data;
    memset(default_pl_data, 0, sizeof(plan_line_data_t));
    default_pl_data->motion                 = {};
    default_pl_data->motion.systemMotion    = 1;
    default_pl_data->process_marker         = 1;
    position_history_reset();
    init_ready = true;
    reset_planner_state();
}
void G_EDM_PLANNER::set_sinker_axis( int axis ){
    sinker_axis = axis;
}

void IRAM_ATTR G_EDM_PLANNER::reset_rconf(){
        rconf.soft_retract_start = 0;
        rconf.hard_retract_start = 0;
        rconf.steps_done         = 0;
        rconf.steps_total        = 0;
        rconf.disable_pwm        = 0;
        rconf.repeat_count       = 0;
}



/**
  * Pausing and Resume 
  **/
bool G_EDM_PLANNER::get_is_paused(){
    return is_paused;
}
void G_EDM_PLANNER::pause( bool redraw ){
    if( is_paused ){ return; }
    is_paused = true;
    toggle_pwm( true, 0 );
    toggle_spindle( true );
    int data = 13;
    xQueueSend( sensor_queue_main, &data, 50000 ); // inform the other core that a pause started
    if( redraw ) { force_redraw = true; }
    while( gconf.edm_pause_motion ){
        if( sys.abort || sys_rt_exec_state.bit.motionCancel ){ break; }
        vTaskDelay(10);
    }
    gconf.process_error = 0;
    //has_reverse                   = 0; // ignore reverse limit after pause
    no_load_steps                 = 0;
    _state.total_retraction_steps = 0;
    //_state.wire_has_first_contact = false;
    set_retraction_steps();
    G_EDM_SENSORS::reset_sensor();
    //_state.wire_has_first_contact = false;
    ui_controller.reset_flush_retract_timer();
    if( ! ui_controller.start_edm_process() ){ 
        is_paused = false;
        return; 
    }
    was_paused          = true;
    pause_recover_count = 0;
    if( ! is_paused ){ return; }
    force_redraw = true;
    if( enable_spindle ){
        toggle_spindle( false );
    }
    vTaskDelay(10);
    is_paused = false;
    for(int i = 0; i < 50; ++i ){
        toggle_pwm( true, 500 );
        toggle_pwm( false, 4 );
    }
    ui_controller.reset_flush_retract_timer();
    short_circuit_start_time = 0;
    G_EDM_SENSORS::reset_sensor();
    get_motion_plan();
}


bool G_EDM_PLANNER::is_ready(){
    return init_ready ? true : false;
}
void G_EDM_PLANNER::update_max_feed(){
    G_EDM_STEPPER* _stepper;
    for( int i = 0; i < N_AXIS; ++i )
    {
        _stepper = motor_manager.get_motor( i );
        int max_feed_step_delay = _stepper->get_step_delay_for_speed( max_feeds[i] );
        max_feeds_micros[ i ] = max_feed_step_delay;
        last_step_micros[i] = esp_timer_get_time();
    }
}
int G_EDM_PLANNER::get_current_round(){
    return gcode_round;
}
void G_EDM_PLANNER::next_gcode_round(){
    // next file pass
    if( gcode_round == 1 ){
        //Serial.println( "Initial round finished:" );
        if( gconf.gedm_wire_gcode_task ){
            // wire only does one round
            // process finished
            gconf.edm_process_finished = true;
        }
    }
    ++gcode_round;
}


void G_EDM_PLANNER::feed_stop( bool stop_feed ){
    _state.skip_feed = stop_feed;
}


/** 
 * Use with care! It is for a special case and should never be used for other purposes. The step delay it out of sync 
 * with the default step delay math. 
 * For example: If Z is moved with a custom step and after that z is moved with the normal line stepping
 * It may happen that there is a zero step delay between the two steps
 **/
void G_EDM_PLANNER::stepper_step_custom( int axis, bool dir, int _step_delay ){
    G_EDM_STEPPER* _stepper = motor_manager.get_motor( axis );
    if( sys_position[axis] > (DEFAULT_HOMING_PULLOFF*-1) * axis_settings[ axis ]->steps_per_mm->get() ){
        return;
    }
    _stepper->set_direction( dir ); // added 50mikros after a dirchange!
    _stepper->step();
    if( _step_delay > 0 ){
        delayMicroseconds( _step_delay );
    }
    if ( dir ) {
        sys_position[axis]--;
    } else {
        sys_position[axis]++;
    }
    last_step_micros[axis] = esp_timer_get_time();
    last_step_timestamp    = esp_timer_get_time();
    return;
}





void G_EDM_PLANNER::engraver_retraction( int _case, int axis, int step_delay, Line_Config &line ){
    int steps = 0; int steps_min;
    steps_min = motor_manager.convert_mm_to_steps( 
        ( _case == 4 ? ENGRAVER_MOTION_PLAN_Z_CASE_3_MIN : ENGRAVER_MOTION_PLAN_Z_CASE_2_MIN ), 
        axis
    );
    if( line.setpoint_step_position != 0 ){
        int steps_max = steps_min*2;
        steps = MAX( steps_min, sys_position[axis]*-1+line.setpoint_step_position);
        if( steps > steps_max ){ steps = steps_max; }
    } 
    steps = MAX( steps_min, steps );
    for( int i = 0; i < steps; ++i ){
        stepper_step_custom( axis, false, step_delay ); // extra steps up
    }
}





bool G_EDM_PLANNER::get_retraction_pwm_off( int _case ){
    if( _case == 3 ){
        return rconf.pwm_off_case_3;
    } else if( _case == 5 ){
        return rconf.pwm_off_case_5;
    }
    return 0;
}



void G_EDM_PLANNER::set_retraction_pwm_off( int _case, bool off ){
    switch (_case)
    {
        case 5:
            rconf.pwm_off_case_5 = off;
            break;
        case 3:
            rconf.pwm_off_case_3 = off;
            break;
        default:
            break;
    }
}






// resets some stuff, checks for short circuit timeout and pause resumes..
bool IRAM_ATTR G_EDM_PLANNER::pre_process_history_line( Line_Config &line ){
    line.skip_feed         = false;
    line.ignore_feed_limit = false;
    if( pause_recover_count < 10 ){
        ++pause_recover_count;
        short_circuit_start_time = 0;
        G_EDM_SENSORS::reset_sensor();
    }
    short_circuit_estop();
    if( was_paused ){
        G_EDM_SENSORS::reset_sensor();
        short_circuit_start_time = 0;
        was_paused = false;
        if( !gconf.gedm_retraction_motion ){
            reset_rconf();
            rconf.hard_retract_start = 1;
            return false; // force backward motion after resume
        }
    } 
    return true;
}




bool IRAM_ATTR G_EDM_PLANNER::short_circuit_estop(){
    //########################################################
    // check for max retraction distance
    //########################################################
    if( gconf.gedm_wire_gcode_task && _state.total_retraction_steps >= rconf.steps_case_0 ){
        gconf.edm_pause_motion = true;
        gconf.process_error    = 5;
    }
    //########################################################
    // check for max duration
    //########################################################
    if( short_circuit_start_time != 0 ){
        if ( esp_timer_get_time() - short_circuit_start_time > adc_critical_conf.short_circuit_max_duration_ms ){ 
            gconf.edm_pause_motion = true; 
            gconf.process_error    = 3;
        }
    }
    return true;
}







void G_EDM_PLANNER::position_history_reset(){
    has_reverse                    = 0;
    was_paused                     = false;
    pause_recover_count            = 0;
    _state.total_retraction_steps  = 0;
    position_history_index_current = 1; 
    position_history_index         = 0;
    position_history_is_between    = false;
    planner_is_synced              = false;
    _state.arc_counter             = 0;
    _state.z_axis_is_up            = false;
    _state.wire_has_first_contact  = false;
    _state.skip_feed               = false;
    memset(position_history, 0, sizeof(position_history[0]));
    push_break_to_position_history();
    feed_stop( false );
    push_current_mpos_to_position_history();
    position_history_force_sync();
    ui_controller.restore_backup_speed();
}
















void G_EDM_PLANNER::configure(){
    if( ! sys_probed_axis[sinker_axis] ){
        //Serial.println("Sinker axis is not probed!");
        sys_probe_position_final[sinker_axis] = sys_position[sinker_axis];
    }
    _state.z_axis_is_up    = false;
    _state.skip_feed       = false;
    _state.arc_counter     = 0;
    is_paused              = false;
    gcode_round            = 1; // counter for the number of file repeats
    deepest_step_pos       = 0;
    last_workload_at       = sys_probe_position_final[sinker_axis];
    no_workload_steps_to_finish = round( finish_after_mm * axis_settings[ sinker_axis ]->steps_per_mm->get() );
    ui_controller.reset_flush_retract_timer();
};
/**
  * return true if z moved down a given amount of mm without having a workload 
  * this should be used with care since noise can easily generate a spike that may be interpreted as workload!
  **/
bool IRAM_ATTR G_EDM_PLANNER::no_workload_finish_reached(){
    return false;
    
    /*if( no_load_steps >= no_workload_steps_to_finish ){
        return true;
    }*/


    if( sys_position[sinker_axis] > last_workload_at ){
        return false;
    }

    if( sys_position[sinker_axis] <= ( last_workload_at - no_workload_steps_to_finish ) ){
        return true;
    }
    return false;
}
/** 
  * set the cutting depth relative to work zero in mm 
  * 0.0 = no limit 
  * this function doesn't care if the limit set is out of range! #todo
  **/
void G_EDM_PLANNER::set_cutting_depth_limit( float _mm ){
    z_cutting_depth = _mm;
    if( _mm == 0.0 ){
        cutting_depth_step_final = 0; // disabled   
        return; 
    }
    int cutting_depth_steps  = round( _mm * axis_settings[ sinker_axis ]->steps_per_mm->get() ); 
    cutting_depth_step_final = sys_probe_position_final[sinker_axis] - cutting_depth_steps;
}
bool IRAM_ATTR G_EDM_PLANNER::work_is_at_cutting_depth_limit(){
    if( cutting_depth_step_final != 0 ){
        if( sys_position[sinker_axis] <= cutting_depth_step_final ){
            //Serial.println("Cutting depth limit");
            return true;
        }
    }
    return false;
}


void G_EDM_PLANNER::reset_planner_state(){
    _state.arc_counter            = 0;
    _state.z_axis_is_up           = false;
    _state.wire_has_first_contact = false;
    was_paused                    = false;
    pause_recover_count           = 0;
    set_ignore_breakpoints( false );
    set_cutting_depth_limit( 0.0 );
    short_circuit_start_time = 0;
}
















bool G_EDM_PLANNER::position_history_move_forward( bool no_motion, Line_Config &line ){
    // get the previous position object
    if( !position_history_is_at_final_index() ){
        position_history_work_get_next( false ); 
        if( has_reverse > 0 ){ --has_reverse; }
    } else {
        has_reverse = 0;
    }
    if( position_history_is_break( position_history[position_history_index_current] ) ){
        return true;
    }
    bool _success = move_line( position_history[position_history_index_current], line );

    return _success;
}

/** this function exits if a short is canceled and also changes the z axis position **/
bool G_EDM_PLANNER::position_history_move_back(){
    // get the previous position object
    bool _success = true;
    // make sure it is not a break point
    bool previous_is_final    = future_position_history_is_at_final_index( true );
    uint16_t index_w_previous = position_history_work_get_previous( true ); // peek the previous index without changing the work index
    bool is_stop = ( 
        ( previous_is_final )
        || position_history_is_break( position_history[index_w_previous] ) // check if the previous index is a block position
    ) ? true : false;

    position_history_work_get_previous( false );
    if( !is_stop ){
        //position_history_work_get_previous( false );
    } else {
        //Serial.println("Move not possible!");
        return false;
    }

    Line_Config line;
    line.step_delay              = process_speeds.RETRACT;
    line.motion_plan_enabled     = true;
    line.ignore_z_motion         = gconf.gedm_single_axis_drill_task ? false : true;
    line.ignore_feed_limit       = true;
    line.enable_position_history = true;
    ++has_reverse;

    _success = move_line( position_history[position_history_index_current], line );

    return _success;
}

/** used only for homing to ignore breaks while seeking the limit switches in positive direction **/
void G_EDM_PLANNER::set_ignore_breakpoints( bool ignore_break ){
    position_history_ignore_breakpoints = ignore_break;
}
bool G_EDM_PLANNER::position_history_is_break( int32_t* target ){
    // positive positions are interpreted as invalid
    // only while homing they are allowed
    if( position_history_ignore_breakpoints ){
        return false;
    }
    bool is_history_break = false; 
    for( int i = 0; i < N_AXIS; ++i ){
        if( target[i] > 0 ){
            is_history_break = true;
            break;
        }
    }
    return is_history_break;
}

uint16_t G_EDM_PLANNER::position_history_get_previous( bool peek ){
    uint16_t index = position_history_index;
    if (index == 0) {
        index = POSITION_HISTORY_LENGTH;
    }
    index--;
    if( ! peek ){
        position_history_index = index;
    }
    return index;
}
uint16_t G_EDM_PLANNER::position_history_get_next(){
    position_history_index++;
    if (position_history_index == POSITION_HISTORY_LENGTH) {
        position_history_index = 0;
    }
    return position_history_index;
}
uint16_t G_EDM_PLANNER::position_history_work_get_previous( bool peek ){
    uint16_t index = position_history_index_current;
    if (index == 0) {
        index = POSITION_HISTORY_LENGTH;
    }
    index--;
    if( ! peek ){
        position_history_index_current = index;
    }
    return index;
}
uint16_t G_EDM_PLANNER::position_history_work_get_next( bool peek ){
    uint16_t index = position_history_index_current;
    index++;
    if (index == POSITION_HISTORY_LENGTH) {
        index = 0;
    }
    if( ! peek ){
        position_history_index_current = index;
    }
    return index;
}
void IRAM_ATTR G_EDM_PLANNER::position_history_force_sync(){
    position_history_index_current = position_history_index;
}
/**
  * In reverse mode this checks if the previous work index is allowed to be used
  * if the previous index is the real final index the history run a full cycle backwards
  * 
  * In forward mode it checks if the next work index is the final index
  **/
bool G_EDM_PLANNER::position_history_is_at_final_index(){
    return position_history_index == position_history_index_current ? true : false;
}
bool G_EDM_PLANNER::future_position_history_is_at_final_index( bool reverse ){
    if( reverse ){
        uint16_t previous_w_index = position_history_work_get_previous( true );
        if( previous_w_index == position_history_index ){
            return true;
        } return false;
    }
    uint16_t next_w_index = position_history_work_get_next( true );
    if( next_w_index == position_history_index ){
        return true;
    } return false;
}
uint16_t IRAM_ATTR G_EDM_PLANNER::get_current_work_index(){
    return position_history_index_current;
}

uint16_t IRAM_ATTR G_EDM_PLANNER::push_to_position_history( int32_t* target, bool override, int override_index ){
    // move one step forward
    if( ! override ){ position_history_get_next(); }
    int index = override ? override_index : position_history_index;
    //position_history[ index ] = target;
    memcpy( position_history[ index ], target, sizeof(target[0]) * N_AXIS );
    return index;
}

/** 
  * Takes the target and line config, pushes the line to the history and syncs the history
  * All positions that are passed to this function are pushed to the history object
  * Not all motions use the history object and some are calling the move line function without storing
  * the positions in the history
  **/
bool IRAM_ATTR G_EDM_PLANNER::process_stage( int32_t* target, Line_Config &line ){
    // push the target to the history buffer
    uint16_t index = push_to_position_history( target, false, 0 );
    bool _success  = position_history_sync( line );
    if( ! _success || sys_rt_exec_state.bit.motionCancel ){
        // if the position could not be reached 
        // it is necessary to update the history element for this position
        target = sys_position;
        //override_target_with_current( target );
        push_to_position_history( target, true, index );
    }
    return _success;
}

//sys_position
void IRAM_ATTR G_EDM_PLANNER::override_target_with_current( float* target ){
    //int32_t* __target = sys_position;
    float* current_position = system_get_mpos();
    memcpy(target, current_position, sizeof(current_position[0])*MAX_N_AXIS );
}

void IRAM_ATTR G_EDM_PLANNER::convert_target_to_steps( float* target, int32_t* __target ){
    for( int axis=0; axis<N_AXIS; ++axis ) {
        __target[axis] = lround( target[axis] * axis_settings[axis]->steps_per_mm->get() );
    }
}

















void G_EDM_PLANNER::push_current_mpos_to_position_history(){
    int32_t target[MAX_N_AXIS]; // work target
    memcpy(target, sys_position, sizeof(sys_position));
    push_to_position_history( target, false, 0 );
}

/** 
  * breaks are ignored in forward direction but prevent the history from moving back 
  * This is just a cheap and easy way to prevent M3/M4 up/downs from becoming a problem
  * Z axis is dispatched in floating operation and would not follow the UP/DOWN path in the history
  * That would result in a crash. The easy way to prevent this is to add a break and 
  * stop the history from retractring further back
  * A break is basically just a position block with all positive coords
  * The only motion that uses positive targets is the homing motion
  * therefore while homing breaks are ignored
  * in normal operation there are only negative targets (all negative space)
  * Make sure your machine is set to all negative space!
  **/
void G_EDM_PLANNER::push_break_to_position_history(){
    int32_t target[MAX_N_AXIS];
    for( int axis = 0; axis < MAX_N_AXIS; ++axis ){target[axis] = 1;}
    push_to_position_history( target, false, 0 );
}


bool G_EDM_PLANNER::position_history_sync( Line_Config &line ){
    gconf.gedm_retraction_motion  = false;
    gconf.gedm_planner_sync     = true;
    bool _success               = true;
    bool motion_ready           = false;
    bool current_is_last_block  = false;
    int  direction              = 0; // 0 = forward, 1 = backward, //2 = move to short trigger block
    int  last_direction         = 0; 
    bool enable_history         = line.enable_position_history; // history is only useful for wire edm    
    _state.total_retraction_steps = 0;

    motion_ready = position_history_is_at_final_index();

    while( !motion_ready ){

        motion_ready = false;

        if( 
            sys.abort 
            || sys_rt_exec_state.bit.motionCancel 
            || gconf.gedm_stop_process 
            || sys_rt_exec_alarm != ExecAlarm::None
        ){ 
            _success = false;
            break; 
        }

        if( enable_history ){

            // default direction is forward
            // if history is enabled it can retract backwards
            // only used for 2D wire; 3D floating doesn't use history
            // Note: in a backward retraction the line backwards is canceled as soon as the short
            // circuit is canceled. It results in a success=false
            // the line was not fully finished and therefore returns false
            // but this is actually a success in cancelling the short circuit
            direction = _success 
                         ? get_success_direction( last_direction ) 
                         : get_failure_direction( last_direction );

        }

        // move in the wanted direction
        _success = process_direction( direction, line );
        last_direction = direction;
        current_is_last_block = position_history_is_at_final_index();

        // break conditions
        if( _success ){
            if( direction == 0 ){
                // this is the same for motion with and without history
                // if the last block was succesfull in forward direction
                // it is finished
                if( current_is_last_block ){
                    motion_ready = true;
                    break;
                }
            }
        } else {
            if( ! enable_history && current_is_last_block ){
                // if it is a non history motion and reached the last block exit and return the success state
                motion_ready = true;
                break;
            }
        }
    }
    //Serial.println("@all sync");
    gconf.gedm_planner_sync      = 0;
    gconf.gedm_retraction_motion = false;
    recovered                    = true;
    // to be safe that history is synced
    // even after a hard motion cancel etc.
    position_history_force_sync();
    return _success;
}

/**
  * 
  * Success and failure only refers to the last line
  * If a line was finished it is a success
  * if a line was not finished it is a failure
  * A backward motion to cancel short circuits don't need to run the full line
  * So a failure in a backward motion always indicates that the short was canceled before the line finished
  * 
  **/
int G_EDM_PLANNER::get_success_direction( int last_direction ){
    // last motion executed succesfull
    switch (last_direction){
        case 0:
            // last success motion was in forward direction
            // this can be a normal feed or a recover forward motion 
            // if this is still within a recovery it needs some extra checks
            // to keep it in recovery until at the initial position
            //Serial.println( "    Forward success" );
            return 0; // keep going forward
            break;
    
        case 1:
            // last success motion was backward
            // this motion was not enough to cancel a short circuit
            // a short was not canceled and needs more retraction
            //Serial.println("    Short not canceled");
            no_load_steps = 0;
            // no matter what history depth is set
            // if within an arc it is overwritten
            // an arc is seen as a single line here
            if( has_reverse >= MAX( _state.arc_counter, plconfig.max_reverse_depth ) ){
                return 0; // force forward
            }
            return 1;
            break;
    }
    return 0;
}
int G_EDM_PLANNER::get_failure_direction( int last_direction ){
    // last motion failed
    switch (last_direction){
        case 0:
            // last failed motion was in forward direction
            // this indicates that a normal feed motion 
            // or a forward recover motion created a short
            // a recover motion follows a retraction with the goal to get back to the initial position
            //Serial.println( "    Forward shorted" );
            return 1; // no matter the details the next move is backwards/retraction
            break;

        case 1:
            // last failed motion in backward direction
            // this motion canceled a short circuit
            // there are no other options for a failed backward motion except the 
            // successfull cancelation of short circuits
            // the backward line was not fully drawn since the short was gone somewhere within the line
            //Serial.println("    Short canceled");
            //delayMicroseconds(200); // #todo #review this delay is very old. Maybe it should be removed?
            no_load_steps = 0;
            return 0; // back to forward???
            break;
    }
    return 1;
}
bool G_EDM_PLANNER::process_direction( int direction, Line_Config &line ){
    bool _success = true;
    switch (direction){
        case 0:
            return position_history_move_forward( false, line );
            break;
        case 1:
            gconf.gedm_retraction_motion = true;
            _success = position_history_move_back();
            gconf.gedm_retraction_motion = false;
            return _success;
            break;
    }
    return false;
}
















/**
  * Used to move Z up via M3 command
  **/
void G_EDM_PLANNER::z_axis_up(){
    if( gconf.gedm_wire_gcode_task ){
        return;
    }
    if( simulate_gcode || _state.z_axis_is_up ){
        toggle_pwm( true, 0 );
        push_break_to_position_history();
        _state.z_axis_is_up = true;
        return;
    }
    Line_Config zup_line; // default config
    zup_line.step_delay = process_speeds.RAPID;

    int32_t __target[MAX_N_AXIS]; // work target
    memcpy(__target, sys_position, sizeof(sys_position));

    int max_pos       = round( DEFAULT_HOMING_PULLOFF * axis_settings[sinker_axis]->steps_per_mm->get() ) * -1;
    int offset        = round( DEFAULT_HOMING_PULLOFF * axis_settings[sinker_axis]->steps_per_mm->get() );

    int axis_target_step = sys_probe_position_final[Z_AXIS] + offset;

    if( axis_target_step > max_pos ){
        axis_target_step = max_pos;
        if( __target[Z_AXIS] > axis_target_step ){
            axis_target_step = __target[Z_AXIS];
        }
    }
    __target[Z_AXIS] = axis_target_step;

    toggle_pwm( true, 0 );
    process_stage( __target, zup_line );
    _state.z_axis_is_up = true;
    push_break_to_position_history(); // set a stop block to the history to prevent it from retracting there
}




/**
  * Used to move Z down via M4 command
  **/
void G_EDM_PLANNER::z_axis_down(){
    if( gconf.gedm_wire_gcode_task ){
        return;
    }
    if( simulate_gcode ){
        _state.z_axis_is_up = false;
        return;
    }
    Line_Config zdown_line;
    // backup some stuff
    zdown_line.step_delay = process_speeds.REPROBE; // lower the speed

    int32_t __target[MAX_N_AXIS]; // work target
    memcpy(__target, sys_position, sizeof(sys_position));

    int travel_limit      = 0;
    // this is just to set a rough travel target
    // the real target is determined through probing
    if( last_workload_at != 0 && last_workload_at != sys_probe_position_final[Z_AXIS] ){
        // move to last workload if it is not the initial probe point 
        travel_limit = last_workload_at;
    } else{
        // no travel limit set and last work load is probe position
        // this can be a reprobe position too where there is already
        travel_limit = sys_probe_position_final[Z_AXIS];
    }
    __target[Z_AXIS] = travel_limit;
    toggle_pwm( false, 4 );
    gprobe.set_probe_direction( false );
    sys_probe_state = Probe::Active;
    probe_touched   = false;
    _state.z_axis_is_up = false;
    process_stage( __target, zdown_line );
    // it does probe but it keeps some limits and sometimes doesn't touch the probe
    // this limit is to keep the process a little even
    // turn off probe after the line is finished
    // probe disables the pwm on success and it needs to be reactivated
    sys_probe_state = Probe::Off;
    toggle_pwm( false, 4 );
    push_break_to_position_history(); // set a stop point after electrode moved down
}







bool G_EDM_PLANNER::z_axis_move_mm( float mm ){
    if( mm == 0.0 ){ return false; }
    int steps         = round( mm * axis_settings[Z_AXIS]->steps_per_mm->get() );
    int32_t __target[MAX_N_AXIS]; // work target
    memcpy(__target, sys_position, sizeof(sys_position));
    __target[Z_AXIS] += steps;
    int max_pos = round( DEFAULT_HOMING_PULLOFF * axis_settings[Z_AXIS]->steps_per_mm->get() ) * -1;
    if( __target[Z_AXIS] > max_pos ){ return false; }
    Line_Config move_mm_line; // default config
    move_mm_line.step_delay = process_speeds.RAPID;
    return move_line( __target, move_mm_line );
}


/** 
  * called between lines! curently in the protocol loop 
  **/
bool G_EDM_PLANNER::reprobing_routine(){
    // turn off sd card readings
    bool axis_was_up = _state.z_axis_is_up;
    // move axis up
    if( ! _state.z_axis_is_up ){
        z_axis_up();
    }
    float* current_position = system_get_mpos();
    float target[MAX_N_AXIS];
    float backup_position[MAX_N_AXIS];
    memcpy( target,          current_position, sizeof(current_position[0]) * N_AXIS );
    memcpy( backup_position, current_position, sizeof(current_position[0]) * N_AXIS );
    if( gconf.gedm_probe_position_x == 0 && gconf.gedm_probe_position_y == 0 ){
        // no probe points set
        // falling back to 0,0 workposition
        float* work_position = current_position;// system_get_mpos();
        float work_position_copy[MAX_N_AXIS];
        mpos_to_wpos( work_position );
        memcpy( work_position_copy, work_position, sizeof(work_position[0]) * N_AXIS );
        //target[X_AXIS] = gc_state.position[X_AXIS] - gc_state.coord_offset[X_AXIS] - work_position_copy[X_AXIS];
        //target[Y_AXIS] = gc_state.position[Y_AXIS] - gc_state.coord_offset[Y_AXIS] - work_position_copy[Y_AXIS];
        if( work_position_copy[X_AXIS] < 0 ){
            target[X_AXIS] += (work_position_copy[X_AXIS]*-1);
        } else{
            target[X_AXIS] -= work_position_copy[X_AXIS];
        }
        if( work_position_copy[Y_AXIS] < 0 ){
            target[Y_AXIS] += (work_position_copy[Y_AXIS]*-1);
        } else{
            target[Y_AXIS] -= work_position_copy[Y_AXIS];
        }
    } else{
        target[X_AXIS] = gconf.gedm_probe_position_x;
        target[Y_AXIS] = gconf.gedm_probe_position_y;
    }
    gconf.gedm_reprobe_motion = true;
    Line_Config reprobe_line; // default config
    reprobe_line.step_delay = process_speeds.RAPID;
    int32_t __target[MAX_N_AXIS];
    convert_target_to_steps( target, __target );
    bool _success = move_line( __target, reprobe_line ); 
    gconf.gedm_reprobe_motion = false;
    is_between_steps        = true;
    // wait for reprobe confirmation
    gconf.gedm_reprobe_block = true;
    while( gconf.gedm_reprobe_block || gconf.edm_pause_motion ){
        if( sys.abort || sys_rt_exec_state.bit.motionCancel ){
            break;
        }
        // wait for confirmation
        vTaskDelay(10);
    }
    // since this routine is called from within a planner line request
    // the gc_position is not updated yet and still thinks it is a the previous position
    // sync the current position
    gcode_core.gc_sync_position();
    // send the G10 command to reset the new Z position
    char command_buf[20];
    sprintf(command_buf,"G10 P1 L20 Z0\r" );
    gproto.execute_line( command_buf );
    // update some variables and recalculate the cutting depth limit etc.
    // calculate the offset from the old probe and workload positions
    int offset_steps = MAX( 0, sys_probe_position_final[Z_AXIS] + (last_workload_at*-1) );
    sys_probe_position_final[Z_AXIS] = sys_position[Z_AXIS];
    last_workload_at                 = sys_probe_position_final[Z_AXIS] - offset_steps;
    set_cutting_depth_limit( z_cutting_depth );
    // move Z axis up to the travel offset position
    _state.z_axis_is_up = false;
    z_axis_up();
    is_between_steps = false;
    // write the new z position to the backup position
    // and move back to the initial XY position
    override_target_with_current( target );
    backup_position[Z_AXIS] = target[Z_AXIS];
    gconf.gedm_reprobe_motion = true;
    convert_target_to_steps( backup_position, __target );
    _success = move_line( __target, reprobe_line ); // again without adding it to the history
    gconf.gedm_reprobe_motion = false;
    gconf.gedm_insert_probe   = false;
    force_redraw            = true;
    is_between_steps        = true;  
    // move z back down if ti was down
    if( ! axis_was_up ){
        z_axis_down();
    }
    return _success;
}



















void G_EDM_PLANNER::set_sinker_direction( int direction ){
    _state.sinker_direction = direction;
}


bool IRAM_ATTR G_EDM_PLANNER::do_flush_if_needed(){
    //return false;
    if( ui_controller.check_if_time_to_flush() ){
        gconf.gedm_flushing_motion = true;
        float flush_offset_mm      = 1.0 / axis_settings[sinker_axis]->steps_per_mm->get() * float( flush_offset_steps );
        retraction( sinker_axis, flush_retract_mm, flush_offset_mm, disable_spark_for_flushing );
        ui_controller.reset_flush_retract_timer();
        _state.total_retraction_steps = 0;
        short_circuit_start_time      = 0;
        gconf.gedm_flushing_motion    = false;
        return true;
    } 
    return false;
}

bool G_EDM_PLANNER::retraction( int stepper, float travel_mm, float offset_mm, bool disable_pwm ){
    if( disable_pwm ){
        toggle_pwm( true, 0 );
    }

    // get the current position and save the position for backup usage
    int32_t __backup_position[MAX_N_AXIS]; // backup target
    int32_t __target[MAX_N_AXIS]; // work target

    memcpy(__backup_position, sys_position, sizeof(sys_position));
    memcpy(__target,          sys_position, sizeof(sys_position));

    int __travel_steps      = round( travel_mm * axis_settings[sinker_axis]->steps_per_mm->get() ); // number of steps to retract
    int __offset_steps      = round( offset_mm * axis_settings[sinker_axis]->steps_per_mm->get() ); // number of offset steps for return
    int start_step_position = __backup_position[sinker_axis]; // log start position for this axis

    int axis_target_step, max_pos;

    //_state.sinker_direction
    if( _state.sinker_direction == 1 ){
        // sinker mode in positive direction; sinker towards home;
        // retract negative from home away
        axis_target_step = __target[sinker_axis] - __travel_steps; // retraction target step position
        max_pos          = glimits.limitsMinPosition(sinker_axis);

        if( axis_target_step < max_pos ){
            axis_target_step = max_pos;
            if( __target[sinker_axis] < axis_target_step ){
                axis_target_step = __target[sinker_axis];
            }
        }


    } else {
        // sinker mode in negative direction; sinker away from home;
        // retract positive towards home
        axis_target_step = __target[sinker_axis] + __travel_steps; // retraction target step position
        max_pos          = round( DEFAULT_HOMING_PULLOFF * axis_settings[sinker_axis]->steps_per_mm->get() ) * -1;
        //glimits.limitsMaxPosition(sinker_axis);

        if( axis_target_step > max_pos ){
            axis_target_step = max_pos;
            if( __target[sinker_axis] > axis_target_step ){
                axis_target_step = __target[sinker_axis];
            }
        }


    }

    // retract
    __target[sinker_axis] = axis_target_step;
    Line_Config line; // default config
    line.step_delay        = process_speeds.RAPID;
    line.ignore_feed_limit = true; // not needed but set
    move_line( __target, line );
    // re-enable PWM and exit if this was a one way motion
    if( disable_pwm ){ 
        toggle_pwm( false, 0 );
    }

    // move back to offset position
    if( _state.sinker_direction == 1 ){
        // sinker mode in positive direction; sinker towards home;
        // move back positive towards home
        __backup_position[sinker_axis] -= __offset_steps;
    
    } else {
        // sinker mode in negative direction; sinker away from home;
        // move back negative away from home
        __backup_position[sinker_axis] += __offset_steps;

    }

    line.exit_on_contact = true;
    line.step_delay      = process_speeds.RAPID;
    // return to offset position
    move_line( __backup_position, line );
    if( sys_position[sinker_axis] != start_step_position ){
        return false;
    }
    return true; 
}




/**
  * Takes the target positions and creates the
  * step/direction bits
  */
bool IRAM_ATTR G_EDM_PLANNER::move_line( int32_t* target, Line_Config &line ){
    //for(int i = 0; i<3; ++i){Serial.println( int(target[i]) );}
    if( sys_rt_exec_state.bit.motionCancel ){
        return false;
    }


    gconf.current_line = position_history_index_current;

    line.last_motion_plan        = 0;
    line.setpoint_step_position  = 0;
    line.failure_step_pos        = 0;
    line.step_bits               = 0;
    line.direction_bits          = 0;
    line.direction_bits_inverted = 0;
    line.step_event_count        = 0;

    int32_t target_steps[MAX_N_AXIS], position_steps[MAX_N_AXIS];
    line.step_count            = 0;
    bool    _success           = true;
    int floating_axis          = -1;
    int current_step_delay     = line.step_delay; // backup

    if( line.floating_enabled ){
        floating_axis = sinker_axis;
    } 

    /** Ensure Z axis does not overshoot the cutting limit and is 
        set to the current position for floating z motion **/
    memcpy(position_steps, sys_position, sizeof(sys_position));
    for( int axis=0; axis<N_AXIS; ++axis ) {
        line.line_math[axis].counter = 0.0;
        if( line.ignore_z_motion
            && axis == Z_AXIS 
        ){
            // overwrite the Z position with the current system position
            // to prevent step generation for z
            target_steps[axis] = lround( sys_position[Z_AXIS] );
        } else{
            target_steps[axis] = target[axis];//lround(target[axis] * axis_settings[axis]->steps_per_mm->get());
        }
        line.line_math[axis].steps    = labs(target_steps[axis] - position_steps[axis]);
        line.step_event_count         = MAX( line.step_event_count, line.line_math[axis].steps );
        line.line_math[axis].delta    = (target_steps[axis] - position_steps[axis]) / axis_settings[axis]->steps_per_mm->get();
        if (line.line_math[axis].delta < 0.0) {
            line.direction_bits |= bit(axis);
        } else {
            line.direction_bits_inverted |= bit(axis);
        }
    }

    if( line.step_event_count <= 0 ){
        if( edm_process_is_running ){
            if( gconf.gedm_retraction_motion ){
                return true;
            } 
            //Serial.println("Bug");
            /*if( position_history_is_at_final_index() ){
                gconf.edm_process_finished = true;
            }*/
            return true;
        }
        delayMicroseconds(10);
        return false;
    }
    motor_manager.motors_direction( line.direction_bits, line.floating_enabled?Z_AXIS:-1 );

    int accel_time   = 20;//us 
    int accel_rounds = line.motion_plan_enabled?0:100;
    if( accel_rounds > 0 && ( accel_rounds * 2 ) > line.step_event_count ){
        // ensure accel and deaccel...
        accel_rounds = MAX(1, int(line.step_event_count/2)-10);
        accel_time   = 2; // ratio?... yeah.. well...
    }
    //accel_rounds=0;

    int64_t stepped_at = esp_timer_get_time();
    int wait_for_delay_end = 0;
    int sync_steps = 0;
    /** the final loop to pulse the motors **/
    while( line.step_count < line.step_event_count ){
        
        // reset defaults
        line.step_delay       = current_step_delay; // restore previous step delay
        line.step_bits        = 0;                  // reset step bits
        line.skip_floating    = false;
        line.skip_feed        = false;
        line.failure_step_pos = 0;                  // 3D floating specific; z position that triggered a short
        // insert pause on request
        if( gconf.edm_pause_motion ){
            pause();
        }

        /*if( sensors.limit_switch_event_detected ){
            GRBL_LIMITS::limits_get_state();
        }*/

        if( 
            sys.abort 
            || sys_rt_exec_state.bit.motionCancel 
            || probe_check( line ) 
            || sys_rt_exec_alarm != ExecAlarm::None
            || ( !line.ignore_limit_switch && GRBL_LIMITS::limits_get_state() )
         ){
            _success = false;
            break;
        }

        if( line.motion_plan_enabled ){
            _success = line.enable_position_history ? process_wire( line ) : pre_process_floating( 0, line );
            if( ! _success ){
                break;
            }
            if( !gconf.gedm_retraction_motion ){
                    //position_history_create_microhistory( line ); 
                    if( line.enable_flushing ){
                        if( do_flush_if_needed() ){
                            // to prevent troubles with position mismatches
                            // this function breaks and jumps to a position history back
                            _success = false;
                            break;
                        }
                    }
                    if( line.enable_no_work_finish ){
                        if( no_workload_finish_reached() ){
                            gconf.edm_process_finished = true;
                            //sys.gedm_stop_process    = true;
                            _success = false;
                            break;
                        }
                    }
            }
            if( !line.ignore_feed_limit && line.step_delay < max_feeds_micros[ Z_AXIS ] ){
                line.step_delay = max_feeds_micros[ Z_AXIS ];
            }
        }

        if( accel_rounds > 0 ){
            int steps_left = line.step_event_count-line.step_count;
            // poor mans accel
            if( line.step_count <= accel_rounds ){ // 99 done; 100 accel
                // accel
                line.step_delay += accel_time*(accel_rounds-line.step_count);
            } else if( steps_left <= accel_rounds ){ // 1000 total - 300 done = 700left
                // deaccel
                int steps_to_deaccel = line.step_event_count-line.step_count;
                line.step_delay += accel_time*(accel_rounds-steps_to_deaccel);
            } else {
                line.step_delay = current_step_delay;
            }
        }

        if( 
            ( line.skip_feed || line.ignore_xy_motion ) 
            || esp_timer_get_time() < stepped_at+line.step_delay 
        ){
            continue;
        } else {
            if( line.enable_position_history ){
                if( gconf.gedm_retraction_motion ){
                    ++_state.total_retraction_steps;
                    ++rconf.steps_done;
                } else {
                    ++no_load_steps; // very dirty.. This resets all around the code and is not accurate at all
                    if( _state.total_retraction_steps > 0 ){
                        --_state.total_retraction_steps;
                    }
                }
            }
        }

        sync_steps = 0;
        // update the step position for each axis
        // this loops over all axis and does one single step
        for (int axis = 0; axis < N_AXIS; axis++) {
            line.line_math[axis].counter += line.line_math[axis].steps;
            if(line.line_math[axis].counter > line.step_event_count) { // "">" could it be ">="?
                line.step_bits |= bit(axis);
                line.line_math[axis].counter -= line.step_event_count;
                    if (line.direction_bits & bit(axis)) {
                        sys_position[axis]--;
                    } else {
                        sys_position[axis]++;
                    }
                ++sync_steps;
            } 
        }
        // do the step
        motor_manager.motors_step( line.step_bits, floating_axis );
        stepped_at = esp_timer_get_time();
        if( sync_steps > 1 ){
            stepped_at += process_speeds.RAPID; // if there where multiple steps give it some extra time
        }
        delayMicroseconds(1);
        ++line.step_count;
    }
    
    if( gconf.gedm_single_axis_drill_task ){
        if( work_is_at_cutting_depth_limit() ){
            gconf.edm_process_finished = true;
            //sys.gedm_stop_process    = true;
            toggle_pwm( true, 0 );
            _success = true;
        }
    }

    if( _success ){
        if( gconf.gedm_wire_gcode_task && !gconf.gedm_retraction_motion ){
            wire_line_end( line );
        }
    }

    // delay the minimum required
    int64_t time_has = esp_timer_get_time() - stepped_at;
    int delay_rest = process_speeds.RAPID - time_has;
    if( delay_rest > 0 ){ delayMicroseconds( delay_rest ); }

    return _success;

}




/** 
  * This is the main gateway for lines
  * All normal lines are passed through this except for some special motions
  **/
uint8_t IRAM_ATTR G_EDM_PLANNER::plan_history_line( float* target, plan_line_data_t* pl_data ) {
    bool _success = true;

    // exit clean on aborts and motionstops
    if( 
        sys.abort 
        || sys_rt_exec_state.bit.motionCancel 
        || gconf.edm_process_finished 
    ){
        idle_timer = esp_timer_get_time();
        override_target_with_current( target );
        gcode_core.gc_sync_position();
        gconf.gedm_planner_line_running = false;
        return false;
    }

    sys.state = State::Cycle;
    gconf.gedm_planner_line_running = true; 

    Line_Config line;

    if( pl_data->is_arc ){
        line.is_arc = true;
        ++_state.arc_counter;
    } else {
        line.is_arc = false;
        _state.arc_counter = 0;
    }

    // change the step delay / speed for this line; This may be overwritten in the process and is just a default value
    if( pl_data->step_delay ){
        line.step_delay = pl_data->step_delay;
    } else{
        line.step_delay = process_speeds.RAPID;
    }

    // set the default line configuration
    line.ignore_limit_switch = pl_data->use_limit_switches ? false : true; // defaults is no limits except for homing!

    if( 
        (!G_EDM_SENSORS::is_probing()&&!pl_data->motion.systemMotion) && // not a system motion //maybe deprecated here
        ( 
            gconf.gedm_floating_z_gcode_task     // is a 3D floating gcode line
            || gconf.gedm_single_axis_drill_task // or a drill/sinker single line
            || gconf.gedm_wire_gcode_task
        ) 
    ){

        // this line is within the EDM process
        // it can be a travel motion or a normal work motion
        // M3/M4 up/down is not passed through this function
        // so this can only be a travel or work movement
        // G0 is a rapid move and normally only the G0 travels should 
        // be flagged as rapids in the process
        // and normally there should have been a M3 / Z up command before
        // if it is a 3D floating gcode process

        if( gconf.gedm_floating_z_gcode_task ){
            if( pl_data->motion.rapidMotion == 1 ){
                // should be a travel move
                line.ignore_z_motion = true; // just to be safe ignore z
            } else {

                // should be a work move
                line.ignore_z_motion     = true;
                line.floating_enabled    = true;
                line.motion_plan_enabled = true;
                line.step_delay          = process_speeds.EDM;
            }
        } else if( gconf.gedm_single_axis_drill_task ){
            line.enable_position_history = true;
            line.step_delay              = process_speeds.EDM;
            line.ignore_z_motion         = false;
            line.motion_plan_enabled     = true;
            line.enable_flushing         = true;
            line.enable_no_work_finish   = true;
        } else if( gconf.gedm_wire_gcode_task ){
            line.enable_position_history = true;
            line.step_delay              = process_speeds.EDM;
            line.ignore_z_motion         = true;
            line.motion_plan_enabled     = true;
        }

        if( simulate_gcode ){
            line.ignore_z_motion     = true;
            line.motion_plan_enabled = false;
            line.step_delay          = process_speeds.RAPID;
        }

    } else if( G_EDM_SENSORS::is_probing() ){
        line.step_delay = process_speeds.PROBING;
    }

    if( line.motion_plan_enabled ){
        set_retraction_steps();
    }

    int32_t __target[MAX_N_AXIS];
    convert_target_to_steps( target, __target );

    _success = process_stage( __target, line );
    if( ! _success || sys_rt_exec_state.bit.motionCancel ){
        override_target_with_current( target );
    }
    position_history_force_sync();
    gcode_core.gc_sync_position(); 
    idle_timer = esp_timer_get_time();
    gconf.gedm_planner_line_running = false;

    return _success;
}
