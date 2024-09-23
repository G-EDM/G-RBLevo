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
#include "api.h"
#include "sd_card/filehandler.h"
#include "Report.h"
#include "GCode.h"

namespace api{

    int32_t get_step_position( int axis ){
      return sys_position[axis];
    }
    void push_cmd(const char *text, bool block_until_idle ){ 
        portENTER_CRITICAL(&myMutex);
        inputBuffer.push(text); 
        portEXIT_CRITICAL(&myMutex);
        vTaskDelay(50);
        if( block_until_idle ){
            vTaskDelay(50);
            block_while_not_idle();
            while( inputBuffer.peek() != -1 ){
                vTaskDelay(50);
            }
        }
    }
    void lock_while_z_moves(){}
    void unlock_machine(){
        /** enable steppers and unlock machine state**/
        push_cmd("$X\r");
        vTaskDelay(50);
    }
    bool machine_is_idle(){
        return ( ( sys.state == State::Alarm || sys.state == State::Idle ) && ! gconf.gedm_planner_line_running) ? true : false;
    }
    void block_while_not_idle(){
        int count = 0;
        while( ! machine_is_idle() ){
            if( sys.abort || gconf.gedm_stop_process || sys_rt_exec_state.bit.motionCancel ){
                if( ++count > 20 ){ break; }
            }
            vTaskDelay(50);
        }
    }
    void probe_block(){
        idle_timer = esp_timer_get_time();
        while( ! sys.probe_succeeded || sys_probe_state == Probe::Active ){
            if( gconf.gedm_stop_process || sys.abort || sys_rt_exec_state.bit.motionCancel ){ break; }
            vTaskDelay(500);
        }
        idle_timer = esp_timer_get_time();
    }
    void block_while_homing( int axis_index ){
        while( ! sys_axis_homed[axis_index] || gconf.gedm_planner_line_running ){
            if( sys.abort || sys_rt_exec_state.bit.motionCancel ){ break; }
            vTaskDelay(10);
        }
        if( machine_is_fully_homed() ){
            //push_cmd("G90 G54 X-30 Y-30\r\n");
            push_cmd("G28.1\r\n");                 // set home position
        }
        block_while_not_idle();
    }
    void cancel_running_job(){
      sys_rt_exec_state.bit.motionCancel = true;
      filehandler.close_file();
    }
    void show_positions(){
      float* current_position = system_get_mpos();
      mpos_to_wpos(current_position);
    }
    float* get_wpos(){
        float* current_position = system_get_mpos();
        mpos_to_wpos(current_position);
        return current_position;
    }
    bool machine_is_fully_homed(){
        bool fully_homed = true;
        for( int i = 0; i < N_AXIS; ++i ){
            if( i == Z_AXIS && z_no_home ){
                // skip Z if homing is not wanted. Electrode wears and if it starts to get shorter homing may be a problem.
                continue;
            }
            if( ! sys_axis_homed[i] ){
                fully_homed = false;
            }
        }
        return fully_homed;
    }

    void home_axis( int axis ){
        switch (axis){
            case X_AXIS:
                home_x();
            break;
            case Y_AXIS:
                home_y();
            break;
            case Z_AXIS:
                if( ! z_no_home ){ home_z(); } 
            break;
        }
        block_while_not_idle();
    }

    void home_machine(){
        if( ! z_no_home ){ home_z(); } 
        home_x();
        home_y();
        block_while_not_idle();
    }
    void home_x(){
        if( sys_rt_exec_state.bit.motionCancel ){ return; }
        sys_axis_homed[X_AXIS] = false;
        push_cmd( "$HX\r" );
        block_while_homing( X_AXIS );
        vTaskDelay(500);
    }
    void home_y(){
        if( sys_rt_exec_state.bit.motionCancel ){ return; }
        sys_axis_homed[Y_AXIS] = false;
        push_cmd( "$HY\r" );
        block_while_homing( Y_AXIS );
        vTaskDelay(500);
    }
    void home_z(){
        if( sys_rt_exec_state.bit.motionCancel ){ return; }
        #ifdef Z_AXIS_NOT_USED
            sys_axis_homed[Z_AXIS] = true;
        #else
            sys_axis_homed[Z_AXIS] = false;
            push_cmd( "$HZ\r" );
            block_while_homing( Z_AXIS );
            vTaskDelay(500);
        #endif
    }

    void force_home( int axis ){ // don't use!
        if( sys_position[axis] <= (DEFAULT_HOMING_PULLOFF*-1) * axis_settings[ axis ]->steps_per_mm->get() ){
            return;
        }
        sys_position[axis] = (DEFAULT_HOMING_PULLOFF*-1) * axis_settings[ axis ]->steps_per_mm->get(); 
        gcode_core.gc_sync_position();
    }

    void reset_probe_points(){
        for( int i = 0; i < N_AXIS; ++i ){
            sys_probed_axis[i] = false;
        }
    }
    void set_current_position_as_probe_position(){
        sys.probe_succeeded = true;
        memcpy(sys_probe_position, sys_position, sizeof(sys_position));
        for( int i = 0; i < N_AXIS; ++i ){
            sys_probed_axis[i]          = true;
            sys_probe_position_final[i] = sys_probe_position[i];
        }
        push_cmd("G91 G10 P1 L20 X0 Y0 Z0\r\n");
        vTaskDelay(500);
    }
    bool machine_is_fully_probed(){
        bool is_fully_probed = true;
        for( int i = 0; i < N_AXIS; ++i ){
            if( ! sys_probed_axis[i] ){
                is_fully_probed = false;
            }
        }
        return is_fully_probed;
    }
    void show_probe_points(){

    }
    void auto_set_probepositions(){
      for (int i = 0; i < N_AXIS; ++i)
      {
        if (!sys_probed_axis[i])
        {

          if (i == X_AXIS)
          {
            push_cmd("G91 G10 P1 L20 X0\r"); 
          }
          else if (i == Y_AXIS)
          {
            push_cmd("G91 G10 P1 L20 Y0\r");
          }
          else if (i == Z_AXIS)
          {
            push_cmd("G91 G10 P1 L20 Z0\r");
          }
          sys_probed_axis[i] = true;
          sys_probe_position_final[i] = sys_position[i];
        }
      }
      push_cmd("G90 G54 G21\r\n",true);
      vTaskDelay(1000);
    }

    void set_reprobe_points(){
        float* current_position = system_get_mpos();
        float position_copy[MAX_N_AXIS];
        memcpy(position_copy, current_position, sizeof(current_position[0]) * N_AXIS);
        set_reprobe_point_x( position_copy[ X_AXIS ] );
        set_reprobe_point_y( position_copy[ Y_AXIS ] );
    }
    void set_reprobe_point_x( float x ){
        gconf.gedm_probe_position_x = x;
    }
    void set_reprobe_point_y( float y ){
        gconf.gedm_probe_position_y = y;
    }
    /** probe inside a hole or pocket and find the center position **/
    void probe_xy_center( void ){
    }
    bool jog_axis( float mm, const char *axis_name, int speed, int axis ){
        if(  gconf.gedm_planner_line_running ){
          sys_rt_exec_state.bit.motionCancel = true;
          while( gconf.gedm_planner_line_running ){
            vTaskDelay(1);
            sys_rt_exec_state.bit.motionCancel = true;
          }
        }
        block_while_not_idle();
        sys_rt_exec_state.bit.motionCancel = false;

        float* current_position = system_get_mpos();
        float position_copy[MAX_N_AXIS];
        memcpy(position_copy, current_position, sizeof(current_position[0]) * N_AXIS);
        position_copy[axis] += mm;


        if( glimits.limitsCheckTravel( position_copy ) ){
            return false;
        }



        /** cancel current jog motion if there is any **/
        char command_buf[40];
        sprintf(command_buf,"$J=G91 G21 %s%.5f F%d\r\n", axis_name, mm, speed);    
        /** push new jog command to input buffer **/
        push_cmd( command_buf );
        return true;
    }
    bool jog_up( float mm ){
        return jog_axis( mm, "Z", Z_AXIS_JOG_SPEED, Z_AXIS );
    }
    bool jog_down( float mm ){
        return jog_axis( (mm*-1), "Z", Z_AXIS_JOG_SPEED, Z_AXIS );
    }
    bool jog_forward( float mm ){
        return jog_axis( mm, "Y", X_AXIS_JOG_SPEED, Y_AXIS );
    }
    bool jog_back( float mm ){
        return jog_axis( (mm*-1), "Y", X_AXIS_JOG_SPEED, Y_AXIS );
    }
    bool jog_left( float mm ){
        return jog_axis( mm, "X", X_AXIS_JOG_SPEED, X_AXIS );
    }
    bool jog_right( float mm ){
        return jog_axis( (mm*-1), "X", X_AXIS_JOG_SPEED, X_AXIS );
    }



};