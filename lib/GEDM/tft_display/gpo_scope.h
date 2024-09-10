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

#include <stdint.h>
#include <config/definitions.h>
#include "tft_display/ili9341_config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

extern xQueueHandle scope_queue; 
extern TaskHandle_t scope_task_handle;

struct gpo_config
{
    bool enable_ksps  = true;
    bool enable_mv    = true;
    bool enable_us    = true;
    bool enable_line  = true;
    bool enable_plan  = true;
    bool enablewave   = true;
    bool drawlinear   = false;
};

struct gpo_scope_data
{
    int  full_range_tresh        = 0;
    bool average_above_setpoint  = false;
    bool average_within_setpoint = false;
    bool average_above_center    = false;
    bool average_below_setpoint  = false;
    bool has_off_note = false;
    uint32_t setpoint_min    = 0;
    uint32_t setpoint_max    = 0;
    uint32_t center_setpoint = 0;
    bool scope_active        = false;
    bool scope_frame_ready   = false;
    bool canvas_processing   = false;
    bool show_values         = true;
    bool header_drawn        = false;
    bool has_task_queue      = false;
    int16_t scope_current_cursor = 0;
    int16_t scope_width          = 0;
    int16_t scope_height         = 0;
    int16_t scope_height_total   = 0;
    int16_t scope_width_total    = 0;
    int16_t scope_pos_x          = 0;
    int16_t scope_pos_y          = 0;
    int16_t scope_last_x         = 0;
    int16_t scope_last_y         = 0;
    int16_t resolution           = 0;
    float scope_ratio        = 1.0;
    bool skip_push           = false;
    int16_t sample_rate      = 0;
    int16_t us_per_read      = 0;
    int16_t last_adc_value  = 0;
    int16_t motion_plan     = 0;
    int16_t total_avg       = 0;
    int motion_signal       = 0;
    int dstate              = 1;
};

extern DMA_ATTR volatile gpo_scope_data gpo_data;
extern DMA_ATTR volatile gpo_config gpconf;
//extern DMA_ATTR uint16_t scope_values[320]; // max display widht is 320px

class GPO_SCOPE
{

    private:
    
        IRAM_ATTR int16_t adc_to_voltage( int16_t adc_value );

    public:

        GPO_SCOPE( void );
        IRAM_ATTR void start( void );
        IRAM_ATTR void set_motion_plan( int16_t plan );
        IRAM_ATTR void set_sample_rate( int16_t sample_rate );
        IRAM_ATTR void set_us_per_read( int16_t us_per_read );
        IRAM_ATTR void set_allow_motion( int motion_signal );
        IRAM_ATTR void set_digital_state( int state );
        IRAM_ATTR void set_total_avg( int16_t adc_value );
        IRAM_ATTR int16_t get_total_avg( void );
        IRAM_ATTR void stop( void );
        IRAM_ATTR void draw_scope();
        IRAM_ATTR void reset( void );
        IRAM_ATTR void init( int16_t width, int16_t height, int16_t posx, int16_t posy );
        IRAM_ATTR int  add_to_scope( int16_t adc_value, int16_t plan );
        IRAM_ATTR bool is_blocked( void );
        IRAM_ATTR void zoom( void );
        IRAM_ATTR void set_scope_resolution( float ratio );
        IRAM_ATTR void draw_header( void );
        IRAM_ATTR void toogle_values( void );
        IRAM_ATTR bool scope_is_running( void );
        IRAM_ATTR float get_zoom( void );
        IRAM_ATTR void setup( void );
        IRAM_ATTR void update_setpoint( uint32_t smin, uint32_t smax );
        IRAM_ATTR void draw_setpoint( void );
        IRAM_ATTR void toggle_scope( bool enable = true );
        IRAM_ATTR void add_peak_sample( int16_t adc_value );
        IRAM_ATTR void draw_peak_avg( void );
        IRAM_ATTR void draw_wave( int steps = 10 );
        IRAM_ATTR void draw_scope_meta( void );
        IRAM_ATTR void refresh_scope_canvas( void );
        IRAM_ATTR void draw_error( int error_code );
        IRAM_ATTR bool avg_above_setpoint( void );
        IRAM_ATTR bool avg_within_setpoint( void );
        IRAM_ATTR bool avg_above_center( void );
        IRAM_ATTR bool avg_below_setpoint( void );
        IRAM_ATTR int  avg_control( void );
        IRAM_ATTR void set_full_range_avg_treshhold( int tresh );
};


extern GPO_SCOPE gscope;

