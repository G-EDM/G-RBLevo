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

#include "config/definitions.h"
#include "driver/mcpwm.h"
#include <esp32-hal-gpio.h>
#include <stdio.h>

typedef struct pulse_data {
  int frequency            = 5000;
  int64_t pulse_end        = 0;
  int pwm_period_us        = 0;
  int pwm_t_on_us          = 0;
  int pwm_t_off_us         = 0;
  int pulse_high           = 0;
  bool pwm_is_off          = false;
  int pwm_pulse_index      = 0;
  int pwm_pulse_index_work = 0;
} pulse_data;

extern DMA_ATTR volatile pulse_data pconf;


/**
 * PWM controller class
 **/
class G_EDM_PWM_CONTROLLER
{
private:
  bool pwm_is_off;
  int pwm_pin;  // PWM+ output pin
  int freq_max;
  int freq_min;
  int pwm_frequency_intern;
  int pwm_duty_cycle;
  float pwm_duty_cycle_percent;
  int pwm_max_duty_cycle; // 255;//@8bit 1023@@10bit resolution
  float duty_cycle_percent;
  float pwm_period;
  float pwm_t_on;
  float pwm_t_off;
  bool spark_generator_is_running; // this only marks if the spark engine is running. Even with a duty of 0 it can be running
  //ledc_timer_config_t   spark_pwm_timer;
  //ledc_channel_config_t spark_pwm_channel;
  float duty_percent_backup;
  int frequency_backup;

public:
  G_EDM_PWM_CONTROLLER(void);
  bool attach_events( void );
  IRAM_ATTR void set_pwm_state( bool low = true );
  IRAM_ATTR void reset_pwm_state( void );
  static bool IRAM_ATTR get_pulse_is_high( void );
  bool IRAM_ATTR get_pwm_is_off( void );
  void setup_pwm_channel(void);
  bool pwm_is_enabled(void);
  int get_freq(void);
  float get_duty_percent(void);
  float get_period(void);
  float get_t_on(void);
  float get_t_off(void);
  void set_pwm_pin( int pin );
  int get_pwm_pin();
  void IRAM_ATTR change_pwm_frequency(int freq);
  void IRAM_ATTR change_pwm_duty(float duty);
  void IRAM_ATTR pwm_off( bool soft = false );
  void IRAM_ATTR pwm_on( bool soft = false );
  void update_values(void);
  void toggle_pwm_on_off(void);
  void disable_spark_generator(void);
  void enable_spark_generator(void);
  void update_duty(float duty_percent);
  void probe_mode_on( void );
  void probe_mode_off( void );

};
