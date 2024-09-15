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
#include "pwm_controller.h"
//#include <HardwareSerial.h>
#include "gedm_sensors/sensors.h"
#include <driver/i2s.h>


/*
#include "xtensa/core-macros.h"
uint32_t ccount = XTHAL_GET_CCOUNT();
*/


#include "HardwareSerial.h"
DMA_ATTR volatile pulse_data pconf;
DMA_ATTR volatile bool pconf_is_running = false;

#ifdef PWM_IS_INVERTED
    DMA_ATTR volatile mcpwm_duty_type_t duty_mode = MCPWM_DUTY_MODE_1;
#else
    DMA_ATTR volatile mcpwm_duty_type_t duty_mode = MCPWM_DUTY_MODE_0;
#endif

void IRAM_ATTR pwm_isr( void ){
  if( 
    pconf_is_running 
    || xQueueIsQueueEmptyFromISR( vsense_queue ) == pdFALSE
    || ( edm_process_is_running && !gconf.gedm_planner_line_running ) // prevent sampling between lines to not mess with sd access
  ){ return; }
  pconf_is_running = true;
  int data = 2;
  //int is_high = (GPIO_REG_READ(GPIO_IN_REG) >> (gpio_num_t)(GEDM_PWM_PIN & 0x1F)) & 1U;
  xQueueOverwriteFromISR(vsense_queue, &data, NULL);
  pconf_is_running = false;
}


void G_EDM_PWM_CONTROLLER::update_values(){      
  duty_cycle_percent  = pwm_duty_cycle_percent;
  pwm_period          = 1.0   / float( pwm_frequency_intern );
  pwm_t_on            = pwm_period / 100.0 * float( duty_cycle_percent );
  pwm_t_off           = pwm_period - pwm_t_on;
  pwm_period          = pwm_period;
  pconf.pwm_period_us = int( round( pwm_period * 1000.0 * 1000.0 ) );
  pconf.pwm_t_on_us   = int( round( pwm_t_on   * 1000.0 * 1000.0 ) );
  pconf.pwm_t_off_us  = int( round( pwm_t_off  * 1000.0 * 1000.0 ) );
}
bool IRAM_ATTR G_EDM_PWM_CONTROLLER::get_pulse_is_high(){
  return pconf.pulse_end >= esp_timer_get_time() ? true : false;
  //int is_high = (GPIO_REG_READ(GPIO_IN_REG) >> (gpio_num_t)(GEDM_PWM_PIN & 0x1F)) & 1U;
}
bool G_EDM_PWM_CONTROLLER::attach_events(){
  
  #ifndef USE_IDF_650
      pinMode(pwm_pin, GPIO_MODE_INPUT_OUTPUT); // problem on espressif 6.5
  #endif
  
  #ifdef ENABLE_PWM_EVENTS
      attachInterrupt(pwm_pin, pwm_isr, RISING);
  #endif
  return true;
}


int G_EDM_PWM_CONTROLLER::get_pwm_pin(){
  return pwm_pin;
}

IRAM_ATTR void G_EDM_PWM_CONTROLLER::set_pwm_state( bool low ){
    #ifdef USE_IDF_650
        mcpwm_set_signal_low( MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_GEN_A );
    #else
        mcpwm_set_signal_low( MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_OPR_A );
    #endif
}
IRAM_ATTR void G_EDM_PWM_CONTROLLER::reset_pwm_state(){
    #ifdef USE_IDF_650
        #ifdef PWM_IS_INVERTED
            mcpwm_set_duty_type( MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_1 );
        #else
            mcpwm_set_duty_type( MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0 );
        #endif
    #else
        #ifdef PWM_IS_INVERTED
            mcpwm_set_duty_type( MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1 );
        #else
            mcpwm_set_duty_type( MCPWM_UNIT_1,MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0 );
        #endif
    #endif
}

void G_EDM_PWM_CONTROLLER::setup_pwm_channel(){
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, pwm_pin);
    mcpwm_config_t pwm_config = {};
    pwm_config.frequency      = pwm_frequency;
    pwm_config.cmpr_a         = 0;
    pwm_config.cmpr_b         = 0;
    pwm_config.counter_mode   = MCPWM_UP_COUNTER;
    pwm_config.duty_mode      = duty_mode;
    set_pwm_state( true );
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    attach_events();
    pconf.pulse_high = 1;
    change_pwm_duty(0.0);
}
G_EDM_PWM_CONTROLLER::G_EDM_PWM_CONTROLLER(){
  pwm_frequency_intern   = 5000;
  pwm_duty_cycle_percent = 0;
  pwm_duty_cycle         = 0;
  pwm_max_duty_cycle     = 1023;//255;//@8bit 1023@@10bit resolution
  duty_cycle_percent     = 0.0;
  pwm_period             = 0.0;
  pwm_t_on               = 0.0;
  pwm_t_off              = 0.0;
  spark_generator_is_running = false;
};
void G_EDM_PWM_CONTROLLER::set_pwm_pin( int pin ){
  pwm_pin = pin;
}
bool G_EDM_PWM_CONTROLLER::pwm_is_enabled(){
  return spark_generator_is_running;
}
int G_EDM_PWM_CONTROLLER::get_freq(){
  return pwm_frequency_intern;
}
float G_EDM_PWM_CONTROLLER::get_duty_percent(){
  return duty_cycle_percent;
}
float G_EDM_PWM_CONTROLLER::get_period(){
  return pwm_period;
}
float G_EDM_PWM_CONTROLLER::get_t_on(){
  return pwm_t_on;
}
float G_EDM_PWM_CONTROLLER::get_t_off(){
  return pwm_t_off;
}    

void IRAM_ATTR G_EDM_PWM_CONTROLLER::pwm_off( bool soft ){
  change_pwm_duty( 0.0 );
  pconf.pwm_is_off = true;
}
void IRAM_ATTR G_EDM_PWM_CONTROLLER::pwm_on( bool soft ){
  pconf.pwm_is_off = false;
  set_pwm_state( true );
  change_pwm_frequency( pwm_frequency_intern );
  change_pwm_duty( pwm_duty_cycle_percent );
}

void IRAM_ATTR G_EDM_PWM_CONTROLLER::change_pwm_frequency( int freq ){
  if( freq < PWM_FREQUENCY_MIN ){
    freq = PWM_FREQUENCY_MIN;  
  } else if( freq > PWM_FREQUENCY_MAX ){
    freq = PWM_FREQUENCY_MAX;
  }
  pconf.frequency      = freq;
  pwm_frequency_intern = freq;
  update_values();
  if( edm_process_is_running && pconf.pwm_is_off ){
    return;
  }
  mcpwm_set_frequency(MCPWM_UNIT_1, MCPWM_TIMER_0, freq);
}
void IRAM_ATTR G_EDM_PWM_CONTROLLER::change_pwm_duty( float duty ){
  if( edm_process_is_running && pconf.pwm_is_off ){
    return;
  }
  if( !spark_generator_is_running && !lock_reference_voltage ){
    duty = 0;
  }
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, float( duty ) );
  reset_pwm_state();
}
void G_EDM_PWM_CONTROLLER::update_duty( float duty_percent ){
  change_pwm_duty( duty_percent );
  pwm_duty_cycle_percent = duty_percent;
  update_values();
}
bool IRAM_ATTR G_EDM_PWM_CONTROLLER::get_pwm_is_off(){
  return (pconf.pwm_is_off?true:false);
}

void G_EDM_PWM_CONTROLLER::toggle_pwm_on_off(){
  return spark_generator_is_running ? disable_spark_generator() : enable_spark_generator();
}
void G_EDM_PWM_CONTROLLER::disable_spark_generator(){
    pwm_off();
    if( sensor_queue_main != NULL ){
        int data = 11;
        xQueueSend( sensor_queue_main, &data, 50000 );
    }
    spark_generator_is_running = false;
    i2sconf.I2S_FORCE_RESTART = true;
}
void G_EDM_PWM_CONTROLLER::enable_spark_generator(){
    if( sensor_queue_main != NULL ){
        int data = 10;
        xQueueSend( sensor_queue_main, &data, 50000 );
    }
    spark_generator_is_running = true;
    pwm_on();
    spark_generator_is_running = true;
    i2sconf.I2S_FORCE_RESTART = true;
}
void G_EDM_PWM_CONTROLLER::probe_mode_on(){
  lock_reference_voltage = true;
  duty_percent_backup = get_duty_percent();
  frequency_backup    = get_freq();
  pwm_on();
  change_pwm_frequency( pwm_frequency_probing );
  update_duty( pwm_duty_probing );
}
void G_EDM_PWM_CONTROLLER::probe_mode_off(){
  lock_reference_voltage = false;
  change_pwm_frequency( frequency_backup );
  update_duty( duty_percent_backup );
  pwm_off();
}
