#include "probing_routines.h"


G_EDM_PROBE_ROUTINES::G_EDM_PROBE_ROUTINES(){}


float G_EDM_PROBE_ROUTINES::get_max_travel_possible( int axis, int direction ){ // direction true = negative
    // buggy and needs a review. Doesn't work!
    float* current_position = system_get_mpos();
    float current[MAX_N_AXIS];
    memcpy( current, current_position, sizeof(current_position[0]) * N_AXIS );
    float travel;
    if( direction ){
        // negative direction
        travel  = glimits.limitsMinPosition( axis ); 
        travel += DEFAULT_HOMING_PULLOFF; 
        travel -= current[axis]; //
        travel  = MIN(0,travel);
    } else {
        // positive direction
        travel  = current[axis]; 
        travel += DEFAULT_HOMING_PULLOFF; 
        travel *= -1;
        travel  = MAX(0,travel);
    }
    return travel;
}

void G_EDM_PROBE_ROUTINES::pulloff_axis( int axis, int direction ){
    float* current_position = system_get_mpos();
    char command_buf[40];
    float max_travel =  direction ? 2.0//  MIN( 2.0, get_max_travel_possible( axis, false ) )
                                  : -2.0;//( MIN( 2.0, get_max_travel_possible( axis, true ) ) ) * -1;
    char axis_name = get_axis_name( axis );
    sprintf(command_buf,"G91 G0 %c%.5f\r\n", axis_name, max_travel );
    api::push_cmd( command_buf );
}

void G_EDM_PROBE_ROUTINES::probe_axis( int axis, int direction ){
    probe_touched           = false;
    sys.probe_succeeded     = false;
    
    /** send the probing command and set a max distance to probe **/
    char command_buf[40];
    char axis_name = get_axis_name( axis );
    
    float max_travel = direction ? get_max_travel_possible( axis, true )   // probe negative
                                 : get_max_travel_possible( axis, false ); // probe positive

    sprintf(command_buf,"G91 G38.2 %c%.5f F40\r\n", axis_name, max_travel );
    api::push_cmd( command_buf );
    api::probe_block();

}
void G_EDM_PROBE_ROUTINES::probe_z( float mm ){
    probe_axis( Z_AXIS, ( mm<0?true:false ) );    
    if(!sys_rt_exec_state.bit.motionCancel && sys.probe_succeeded ){
        sys_probe_position_final[Z_AXIS] = sys_probe_position[Z_AXIS];
        sys_probed_axis[Z_AXIS] = true;    
        api::push_cmd( "G91 G10 P1 L20 Z0\r\n" );
        pulloff_axis( Z_AXIS, true );
    }
    vTaskDelay(100);
    api::block_while_not_idle();
}


void G_EDM_PROBE_ROUTINES::probe_x( float mm, bool pulloff ){

    probe_axis( X_AXIS, ( mm<0?true:false ) );

    char command_buf[40];
 
    if(!sys_rt_exec_state.bit.motionCancel && sys.probe_succeeded ){
        sys_probe_position_final[X_AXIS] = sys_probe_position[X_AXIS];
        sys_probed_axis[X_AXIS]          = true;
        float tool_radius                = tool_diameter>0.0?tool_diameter/2.0:0.0;
        sprintf(command_buf,"G91 G10 P1 L20 X%.5f\r\n", ( mm < 0 ? tool_radius : (tool_radius*-1) ) );
        api::push_cmd( command_buf );
        if(pulloff){
            pulloff_axis( X_AXIS, ( mm<0 ? true : false ) );
        }
    }
    vTaskDelay(100);
    api::block_while_not_idle();
}
void G_EDM_PROBE_ROUTINES::probe_y( float mm, bool pulloff ){
    probe_axis( Y_AXIS, ( mm<0?true:false ) );

    char command_buf[40];
    
    if(!sys_rt_exec_state.bit.motionCancel && sys.probe_succeeded ){
        sys_probe_position_final[Y_AXIS] = sys_probe_position[Y_AXIS];
        sys_probed_axis[Y_AXIS] = true;
        float tool_radius = tool_diameter>0.0?tool_diameter/2.0:0.0;
        sprintf(command_buf,"G91 G10 P1 L20 Y%.5f\r\n", ( mm < 0 ? tool_radius : (tool_radius*-1) ) );
        api::push_cmd( command_buf );
        if(pulloff){
            pulloff_axis( Y_AXIS, ( mm<0 ? true : false ) );
        }
    }
    vTaskDelay(100);
    api::block_while_not_idle();
}





void G_EDM_PROBE_ROUTINES::left_front_edge_3d(){
    probe_z(-1.0); // first probe z
    api::push_cmd("G91 G0 X-15\r\n");
    api::push_cmd("G91 G0 Z-6\r\n", true);
    probe_x(1.0,true);
    api::push_cmd("G91 G0 Z6\r\n", true);
    api::push_cmd("G91 G0 X10 Y-15\r\n");
    api::push_cmd("G91 G0 Z-6\r\n", true);
    probe_y(1.0,true);
    api::push_cmd("G91 G0 Z6\r\n");
    api::push_cmd("G90 G54 G0 X0 Y0\r\n", true);
}
void G_EDM_PROBE_ROUTINES::left_front_edge_2d(){
    probe_x(1.0,true); //left right
    api::push_cmd("G91 G0 Y-15\r\n");
    api::push_cmd("G91 G0 X10\r\n");
    probe_y(1.0,true); //back froward
    api::push_cmd("G90 G54 X-2 Y-2\r\n", true);
}
void G_EDM_PROBE_ROUTINES::left_back_edge_3d(){
    probe_z(-1.0);
    // probe_mode_off();
    api::push_cmd("G91 G0 X-15\r\n");
    api::push_cmd("G91 G0 Z-6\r\n", true);
    probe_x(1.0,true);
    api::push_cmd("G91 G0 Z6\r\n", true);
    api::push_cmd("G91 G0 X10 Y15\r\n");
    api::push_cmd("G91 G0 Z-6\r\n", true);
    probe_y(-1.0,true);
    api::push_cmd("G91 G0 Z6\r\n");
    api::push_cmd("G90 G54 G0 X0 Y0\r\n", true);
}
void G_EDM_PROBE_ROUTINES::left_back_edge_2d(){
    probe_x(1.0,true); //left right
    api::push_cmd("G91 G0 Y15\r\n");
    api::push_cmd("G91 G0 X10\r\n");
    probe_y(-1.0,true); //back froward
    api::push_cmd("G90 G54 X-2 Y2\r\n", true);
}
void G_EDM_PROBE_ROUTINES::right_back_edge_3d(){
    probe_z(-1.0);
    api::push_cmd("G91 G0 X15\r\n");
    api::push_cmd("G91 G0 Z-6\r\n", true);
    probe_x(-1.0,true);
    api::push_cmd("G91 G0 Z6\r\n", true);
    api::push_cmd("G91 G0 X-10 Y15\r\n");
    api::push_cmd("G91 G0 Z-6\r\n", true);
    probe_y(-1.0,true);
    api::push_cmd("G91 G0 Z6\r\n");
    api::push_cmd("G90 G54 G0 X0 Y0\r\n", true);
}
void G_EDM_PROBE_ROUTINES::right_back_edge_2d(){
    probe_x(-1.0,true); //left right
    api::push_cmd("G91 G0 Y15\r\n");
    api::push_cmd("G91 G0 X-10\r\n");
    probe_y(-1.0,true); //back froward
    api::push_cmd("G90 G54 X2 Y2\r\n", true);
}
void G_EDM_PROBE_ROUTINES::right_front_edge_3d(){
    probe_z(-1.0);
    api::push_cmd("G91 G0 X15\r\n");
    api::push_cmd("G91 G0 Z-6\r\n", true);
    probe_x(-1.0,true);
    api::push_cmd("G91 G0 Z6\r\n", true);
    api::push_cmd("G91 G0 X-10 Y-15\r\n");
    api::push_cmd("G91 G0 Z-6\r\n", true);
    probe_y(1.0,true);
    api::push_cmd("G91 G0 Z6\r\n");
    api::push_cmd("G90 G54 G0 X0 Y0\r\n", true);
}
void G_EDM_PROBE_ROUTINES::right_front_edge_2d(){
    probe_x(-1.0,true); //left right
    api::push_cmd("G91 G0 Y-15\r\n");
    api::push_cmd("G91 G0 X-10\r\n");
    probe_y(1.0,true); //back froward
    api::push_cmd("G90 G54 X2 Y-2\r\n", true);
}

/** 
  * Electrode needs to be somewhere within the pocket and z axis down. 
  * This routine doesn't generate any z motions
  **/
void G_EDM_PROBE_ROUTINES::center_finder_2d(){
    int y_neg, y_pos, x_neg, x_pos, x_target, y_target;
    char command_buf[40];
    float target_mpos;
    probe_x( -1.0, false );
    x_neg       = sys_probe_position_final[X_AXIS];
    api::push_cmd("G90 G0 X1.0\r\n");
    probe_x( 1.0, false );
    x_pos       = sys_probe_position_final[X_AXIS];
    x_target    = ( x_neg - x_pos ) / 2;
    // convert step positions to mpos and move to center of probe points
    target_mpos = system_convert_axis_steps_to_mpos( x_target, X_AXIS);
    sprintf(command_buf,"G90 G0 X%.5f F40\r\n", target_mpos );
    api::push_cmd( command_buf );
    api::block_while_not_idle();
    probe_y( -1.0, false );
    y_neg       = sys_probe_position_final[Y_AXIS];
    api::push_cmd("G90 G0 Y1.0\r\n");
    probe_y( 1.0, false );
    y_pos       = sys_probe_position_final[Y_AXIS];
    y_target    = ( y_neg - y_pos ) / 2;
    // convert step positions to mpos and move to center of probe points
    target_mpos = system_convert_axis_steps_to_mpos( y_target, Y_AXIS);
    sprintf(command_buf,"G90 G0 Y%.5f F40\r\n", target_mpos );
    api::push_cmd( command_buf );
    api::block_while_not_idle();
    // set work zero position
    api::push_cmd("G91 G10 P1 L20 X0 Y0\r\n");
    api::block_while_not_idle();
}

 


