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
#include "gedm_api/api.h"

#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/timer.h>
#include <esp32-hal-gpio.h>


static const int vsense_num_tasks                = 5;
static const int vsense_sampler_buffer_size      = 200;

extern xQueueHandle sensor_queue_main;

typedef struct pulse_extractor {
    bool     full_pulse_collected    = false;
    bool     pulse_collect_start     = false;
    bool     pulse_tail_start        = false;
    uint16_t total_full_pulses       = 0;
    uint16_t pulse_batch_num_samples = 0;
    uint64_t pulse_batch_adc         = 0;
    uint64_t last_pulse_adc          = 0;
    uint64_t current_pulse_adc       = 0;
} pulse_extractor;



typedef struct spark_counter_data {
    int64_t start_time        = 0;
    int64_t pulse_count       = 0;
    int16_t pulses_per_second = 0;
    bool    pulse_block       = false;
    int rate                  = 0;
} spark_counter_data;

typedef struct adc_critical_config {
  int  max_channel_shorts_for_pulse_off = 10;
  int  forward_block_channel_shorts_num = 2; // number of shorts within the i2s voltage batch that will block the next forward motion
  int  forward_block_after_short_micros = 100;
  int  voltage_feedback_treshhold      = 1500;
  int  pwm_off_count                   = 0;
  int  pulse_off_duration              = 1;
  int  pulse_count_zero_jitter         = 0;
  int  full_range_average_treshhold    = 6;
  int  short_circuit_max_duration_ms   = 300;
  int  zeros_jitter                    = 1; // the adc is not really synched and sometimes it captures the offtime. That would create a false reset of the high in a row counter.
  int  highs_in_a_row_max              = 40;
  int  zeros_in_a_row_max              = 4;
  int  zero_treshhold                  = 30;
  int  high_plan_at                    = 3;
  int  edge_treshhold                  = 40;  // value is in adc resolution. 12bit = 0-4095. Changes above this treshhold will be considered an edge
  int  retract_confirmations           = RETRACT_CONFIRMATIONS;
} adc_critical_config;

typedef struct adc_sampling_params {
    int32_t rising_avg     = RISING_AVG;
    int32_t falling_avg    = FALLING_AVG;
    int32_t default_avg    = DEFAULT_AVG;
    int32_t i2s_avg        = 1000; // default it to max out all possible I2S settings; 
                                   // I2S loads a batch fo samples (buff_len). This value defines the number of samples 
                                   // that are used from the I2S batch; Lowering the buff_len creates more and more interrupts
                                   // and it will stop working at some point. Therefore just load a bigger batch and if needed change the number of samples 
                                   // from this batch. 
} adc_sampling_params;

typedef struct adc_edge_data {
  bool writeout                       = true;
  bool falling_edge_accepted          = false;
  bool multiple_rising_edges          = false;
  bool is_rising_edge                 = false;
  bool is_falling_edge                = false;
} adc_edge_data;

typedef struct ADC_WAVE_CONTROL {
  int has_next_rising_edge        = 0;
  int multiple_rising_edges_count = 0;
  uint64_t falling_edge_block_start_time = 0;
  bool     adc_start_collection       = false;
  int16_t  adc_collected              = 0;
  int16_t  adc_pre_rising_edge        = -1;
  bool     rising_edge_normalized     = true;
  bool     edge_recovery_failed       = false;
  bool     rising_edge_block          = false;
  bool     falling_edge_block         = false;
  bool     is_next_rising_edge        = false;
  int16_t  edge_to_edge_block_count   = 0;
  int16_t  last_rising_edge_adc       = 0;
  int64_t  timestamp                  = 0;
} ADC_WAVE_CONTROL;

typedef struct adc_sampling_stats {
  int     sample_rate = 0; // real archived rate
  int16_t us_per_read = 0;
} adc_sampling_stats;

typedef struct adc_setpoints {
  uint32_t forward       = 0;
  uint32_t lower         = 0;
  uint32_t upper         = 0;
  uint32_t center        = 0;
  uint32_t probing       = 0;
  uint32_t short_circuit = 0;
  uint32_t short_circuit_break = 0;
} adc_setpoints;

typedef struct adc_readings {
  int previous_plan       = 0;
  int previous_edge       = 0;
  uint32_t voltage_channel_adc_recent = 0;
  uint32_t voltage_channel_shorts     = 0;
  int zeros_in_a_row      = 0;
  int highs_in_a_row      = 0;
  int      plan           = 0;
  int      edge_type      = 0;
  uint32_t recent         = 0;
  uint32_t sampled        = 0;
  uint32_t avg_full_range_last = 0;
  uint32_t avg_full_range = 0;
  int64_t  timestamp      = 0;
  int64_t  rising_edge_ignition_delay  = 0;
} adc_readings;

typedef struct ADC_TIMER {
    bool use_hw_timer           = false;
    bool timer_is_at_high_speed = false;
    bool timer_is_running       = false;
    int  interval               = VSENSE_FEEDBACK_TIMER_INTERVAL_EDM;
} ADC_TIMER;

typedef struct I2S_conf {
  size_t num_bytes         = sizeof(uint16_t) * I2S_NUM_SAMPLES;   
  int  I2S_pool_size       = 1;      
  int  I2S_full_range      = 30;      
  int  I2S_buff_count      = I2S_BUFF_COUNT;
  int  I2S_buff_len        = I2S_NUM_SAMPLES;
  int  I2S_sample_rate     = I2S_SAMPLE_RATE;
  bool I2S_READY           = false;
  bool I2S_is_restarting   = false;
  bool I2S_failure         = false;
  int  I2S_failure_count   = 0;
  int  I2S_core_id         = 1;
  int  I2S_equal_reading   = 0;
  bool I2S_busy            = false;
  bool block_write_out     = false;
  bool writing             = false;
  bool I2S_USE_DELAY       = false;
  bool I2S_FORCE_RESTART   = false;
  bool I2S_is_in_read      = false;
  bool I2S_stop            = false;
  bool I2S_start           = false;
  bool I2S_is_paused       = false;
  uint64_t read_sum        = 0;
} I2S_conf;

typedef struct SENSOR_STATES {
  bool digital_sense_event_detected = true;
  bool on_off_switch                = false;
  bool limit_switch                 = false;
  bool on_off_switch_event_detected = false;
  bool limit_switch_event_detected  = false;
} SENSOR_STATES;

typedef struct edge_control {
    uint16_t edges_peak_avg            = 0;
    int64_t timestamp                  = 0;
    int64_t rising_edge_start          = esp_timer_get_time();
    int     multiple_edges_total       = 0;
    bool    locked                     = false;
    int     empty_blocks_between_edges = 0;
    int     edges_total                = 0;
    int     edges_adc_total            = 0;
    int     edges_solved_total         = 0;
    int     cycle_count                = 0;
    int     edge_solved_after          = 0;
    bool    has_unsolved_edge          = false;
    bool    edge_solved_in_cycle       = true;
    int     edge_solved_confirmations  = 0;
    int     empty_blocks_tail          = 0; // this is the tail or head of the empty blockes (PWM off). It will count up until a non empty block is processed. Could be useful.
} edge_control;

typedef struct SECOND_FEEDBACK_DATA {
  bool     pin_is_low            = false;
  uint64_t last_short_condition  = 0;
  int      count                 = 0;
  int      reset_count           = 0;
  int      had_short_condition   = 0;
  int      had_short_condition_b = 0;
} SECOND_FEEDBACK_DATA;


extern DRAM_ATTR edge_control edge_ctrl;
extern DMA_ATTR volatile SENSOR_STATES        sensors;
extern DMA_ATTR volatile adc_readings         adc_data;
extern DMA_ATTR volatile adc_setpoints        setpoints;
extern DMA_ATTR volatile I2S_conf             i2sconf;
extern DMA_ATTR volatile adc_sampling_stats   adc_stats;
extern DMA_ATTR volatile adc_sampling_params  adc_params;
extern DMA_ATTR volatile ADC_WAVE_CONTROL     adc_wave;
extern DMA_ATTR volatile adc_critical_config  adc_critical_conf;
extern DMA_ATTR volatile ADC_TIMER            adc_hw_timer;
extern DMA_ATTR volatile SECOND_FEEDBACK_DATA sfeedback;
extern DMA_ATTR volatile spark_counter_data   spark_counter;
extern DMA_ATTR volatile pulse_extractor      pex_data;





extern xQueueHandle vsense_queue; 
extern xQueueHandle motion_plan_queue; 

extern hw_timer_t * vsense_adc_sampler_timer; 



extern TaskHandle_t vsense_tasks[vsense_num_tasks];

extern DMA_ATTR volatile uint32_t feedback_voltage;
extern DMA_ATTR volatile int      counter[10];
extern DMA_ATTR volatile int      multisample_counts;
extern DMA_ATTR volatile uint32_t multisample_buffer[vsense_sampler_buffer_size];
extern DMA_ATTR volatile int reference_adc_value;
void IRAM_ATTR vsense_on_timer( void );
IRAM_ATTR void read_voltage_feedback( volatile adc_readings &adc_data );


int      IRAM_ATTR generate_average_adc( int adc_value );
uint32_t IRAM_ATTR read_multisample( int raw, bool write_only, int num_samples, bool read_only );
bool     IRAM_ATTR i2s_shift_out( void );
void     IRAM_ATTR toogle_i2s( bool start = true );
void     IRAM_ATTR configure_i2s( void );
void     IRAM_ATTR restart_i2s( void );
void     IRAM_ATTR stop_i2s( void );
void     IRAM_ATTR vsense_task_listener( void * parameter );
void     IRAM_ATTR update_vsense_task( void * parameter );
void     IRAM_ATTR default_sensor_task( void * parameter );
void     IRAM_ATTR single_shot_core_config( void *parameter );
void     IRAM_ATTR switch_on_interrupt( void );
void     IRAM_ATTR limit_switch_on_interrupt( void );
void     IRAM_ATTR read_vsense( bool wait_for_high_pwm = false );
void     IRAM_ATTR block_while_sampling( int num_samples = 2 );
bool     IRAM_ATTR I2S_is_ready( bool include_all = true );
void     IRAM_ATTR gpio_35_on_interrupt( void );


class G_EDM_SENSORS {
private:
    bool has_init;


public:
    G_EDM_SENSORS();
    int probe_positive_count;
    void init_adc( void );
    int32_t IRAM_ATTR get_avg( bool rising );
    void    IRAM_ATTR set_avg( bool rising, int avg );
    void    IRAM_ATTR set_avg_default( int avg );
    int32_t IRAM_ATTR get_avg_default( void );
    int32_t IRAM_ATTR get_avg_i2s( void );
    void    IRAM_ATTR set_avg_i2s( int avg );
    void    IRAM_ATTR ignition_ramp_up( void );

    void            IRAM_ATTR change_sampling_rate( int rate );
    int             percentage_to_mv( float percent );
    uint32_t        IRAM_ATTR get_adc_value( int index );
    static int      IRAM_ATTR calculate_gain( int adc_value_a, int adc_value_b );
    static bool     IRAM_ATTR vsense_is_outdated( void );
    static uint32_t IRAM_ATTR adc_to_voltage( int adc_value );
    static uint32_t IRAM_ATTR adc1_get_fast(int channel);

    static int      IRAM_ATTR get_motion_plan( volatile adc_readings &adc_data );
    static int      IRAM_ATTR get_raw_plan( int adc_value );
    static bool     IRAM_ATTR edge_handler( adc_edge_data &edge, volatile adc_readings &adc_data );
    static void     IRAM_ATTR get_edge_type( adc_edge_data &edge, int16_t adc_old, int16_t adc_new );
    static bool     IRAM_ATTR horizontal_forward_blocker( int &fctrl, volatile adc_readings &adc_data );
    static bool     IRAM_ATTR adc_to_scope( bool recent = false );

    /*
    IRAM_ATTR static void adjust_plan_to_edge( volatile adc_readings &adc_data, int &plan );
    IRAM_ATTR static void adjust_plan_to_signals( volatile adc_readings &adc_data, int &plan, bool &create_negative_plan );
    IRAM_ATTR static void set_signals( volatile adc_readings &adc_data, int &realtime_plan );
    IRAM_ATTR static void do_retract_soft( volatile adc_readings &adc_data, int &plan, int &full_range_plan, int &realtime_plan );
    IRAM_ATTR static void do_retract_hard( int &plan, int &full_range_plan, int &realtime_plan );
    IRAM_ATTR static void do_forward_motion( volatile adc_readings &adc_data, int &plan, int &full_range_plan, int &realtime_plan , bool &create_negative_plan );
    */

    static void     IRAM_ATTR pwm_off_protection( bool enforce );
    static void     IRAM_ATTR reset_buffer( int exclude = 0 );
    static float    IRAM_ATTR get_drop_in_percent_realtime( int raw );
    static float    IRAM_ATTR get_drop_in_percent( bool _update );
    static void     IRAM_ATTR update_feedback_data( void );
    static float    IRAM_ATTR calulate_source_voltage( void );
    static int      IRAM_ATTR get_calculated_motion_plan( void );
    static bool     IRAM_ATTR is_probing( void );
    static bool     IRAM_ATTR edm_start_stop( void );
    static bool     IRAM_ATTR limit_switch_read( void );
    static int32_t  IRAM_ATTR get_vsense_raw( void );
    static void     IRAM_ATTR generate_setpoint_min_max_adc( int adc_reading_reference = 0 );
    static int      IRAM_ATTR get_number_of_samples( void );
    static int      IRAM_ATTR collect_vsense( int counts );
    static bool     IRAM_ATTR is_flateline( void );
    static void     IRAM_ATTR set_high_low( uint16_t adc_value );
    int IRAM_ATTR calculate_cycles( int duration );


    static void IRAM_ATTR reset_sensor( void );
    static void IRAM_ATTR reset_edge( void );

    static void enable_hw_timer( int _val = 0 );
    static void disable_hw_timer( void );
    static void change_hw_timer( int _val = 0 );


    void run_sensor_service( void );
    uint32_t get_feedback_voltage( int adc_value = 0 );
    bool limit_is_touched( void );
    bool continue_edm_process( void );
    void generate_reference_voltage( void );
    bool start_edm_process( void );
    void collect_probe_samples( void );


};
