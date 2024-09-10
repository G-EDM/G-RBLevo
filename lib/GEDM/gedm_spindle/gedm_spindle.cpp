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
#include "gedm_spindle.h"
#include "config/definitions.h"

G_EDM_SPINDLE gedm_spindle = G_EDM_SPINDLE();

G_EDM_SPINDLE::G_EDM_SPINDLE(){}

void G_EDM_SPINDLE::setup_spindle( int _dir_pin, int _step_pin ){
    default_frequency = 5000;
    backup_frequency  = default_frequency;
    dir_pin           = _dir_pin;
    step_pin          = _step_pin;
    frequency         = default_frequency;
    dir_inverted      = false;

    #ifndef USE_IDF_650

    //if( dir_pin != 1 ){ // problem on espressif 6.5
        pinMode(dir_pin, OUTPUT);   
        digitalWrite(dir_pin,LOW);
    //}

    #endif
    
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, step_pin);
    mcpwm_config_t pwm_config = {};
    pwm_config.frequency      = frequency;
    pwm_config.cmpr_a         = 0;
    pwm_config.cmpr_b         = 0;
    pwm_config.counter_mode   = MCPWM_UP_COUNTER;
    pwm_config.duty_mode      = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    stop_spindle();
}

int G_EDM_SPINDLE::rpm_to_frequency( float rpm ){
    return round(rpm/60.0*float(200*DEFAULT_A_MICROSTEPS) );//0.46/60*(200*64)
}
float G_EDM_SPINDLE::frequency_to_rpm( float _frequency ){
    return _frequency*60/(200*DEFAULT_A_MICROSTEPS);//0.46/60*(200*64)
}
void G_EDM_SPINDLE::reset_spindle(){
    set_speed( default_frequency );
}
int G_EDM_SPINDLE::get_speed(){
    return frequency;
}
bool G_EDM_SPINDLE::restore_backup_speed(){
    return set_speed( backup_frequency, false );
}
bool G_EDM_SPINDLE::relative_rpm_up( float add_rpm ){
    if( add_rpm <= 0.0 ){ return false; }
    // this function does not override the backup frequency 
    // don't use this to change the main spindle speed
    // it will always add the given rpms relative to the backup_frequency
    // calling it multipe times with the same value will produce the same speed
    int frequency_add    = rpm_to_frequency( add_rpm );
    int frequency_target = backup_frequency+frequency_add;
    if( frequency_target > DEFAULT_A_MAX_FREQUENCY ){
        frequency_target = DEFAULT_A_MAX_FREQUENCY;
    }
    set_speed( frequency_target, false );
    return true;
}
bool G_EDM_SPINDLE::set_speed( int __frequency, bool backup ){
    if( frequency == __frequency ){ return true; }

    if( __frequency > frequency ){
        // faster 
    } else {
        // slower
    }

    frequency = __frequency;
    if( backup ){
        backup_frequency = frequency;
    }
    mcpwm_set_frequency( MCPWM_UNIT_0, MCPWM_TIMER_0, __frequency );
    if( !is_running ){ return false; }
    mcpwm_set_duty( MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 );
    return true;
}
void G_EDM_SPINDLE::start_spindle(){
    if( is_running ){ return; }
    int __frequency = 0;
    mcpwm_set_frequency( MCPWM_UNIT_0, MCPWM_TIMER_0, 10 );
    mcpwm_set_duty( MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 );
    while( __frequency < frequency ){
        // soft start
        if( sys.abort || sys_rt_exec_state.bit.motionCancel ){ break; }
        __frequency += 10;
        mcpwm_set_frequency( MCPWM_UNIT_0, MCPWM_TIMER_0, __frequency );
        mcpwm_set_duty( MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 );
        vTaskDelay(20);
    }
    mcpwm_set_frequency( MCPWM_UNIT_0, MCPWM_TIMER_0, frequency );
    mcpwm_set_duty( MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 );
    is_running = true;
}
void G_EDM_SPINDLE::stop_spindle(){
    if( ! is_running ){ return; }
    //mcpwm_set_frequency( MCPWM_UNIT_0, MCPWM_TIMER_2, 0 );
    mcpwm_set_duty( MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0 );
    is_running = false;
}
void G_EDM_SPINDLE::set_spindle_direction( bool direction ){
}
bool G_EDM_SPINDLE::spindle_is_running(){
    return is_running;
}
void G_EDM_SPINDLE::reverse_direction(){
    if( dir_inverted ){
        digitalWrite(dir_pin,LOW);
        dir_inverted = false;
    } else{
        digitalWrite(dir_pin,HIGH);
        dir_inverted = true;
    }
}
bool G_EDM_SPINDLE::dir_is_inverted(){
    return dir_inverted;
}
/** only use as rough orientation for wire speeds **/
float G_EDM_SPINDLE::convert_mm_min_to_frequency( float mm_min ){
    float u = 3.14159 * WIRE_SPINDLE_DIAMETER; // mm per rotation
    float rotation_per_min = 1.0 / u * mm_min;
    return rpm_to_frequency( rotation_per_min );
}
float G_EDM_SPINDLE::convert_frequency_to_mm_min( float frequency ){
    float u = 3.14159 * WIRE_SPINDLE_DIAMETER; // mm per rotation
    return u * frequency_to_rpm( frequency );
}