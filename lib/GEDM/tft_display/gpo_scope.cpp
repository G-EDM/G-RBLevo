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

#include "gpo_scope.h"

xQueueHandle scope_queue; 
TaskHandle_t scope_task_handle;

GPO_SCOPE gscope;

static const int scope_buffer_size = 320;

DMA_ATTR int16_t scope_values_plan[scope_buffer_size];
DMA_ATTR int16_t scope_values[scope_buffer_size];
DMA_ATTR volatile gpo_scope_data gpo_data;
DMA_ATTR volatile gpo_config     gpconf;

GPO_SCOPE::GPO_SCOPE(){}

void IRAM_ATTR scope_task( void * parameter ){
    GPO_SCOPE *__this = (GPO_SCOPE *)parameter;
    int16_t data[queue_item_size];
    for(;;){
      xQueueReceive(scope_queue, &data, portMAX_DELAY);
      if( gpo_data.canvas_processing ){
        continue;
      }
      //vTaskDelay(1);continue;
      if( data[0] != -1 ){
        __this->add_to_scope( data[0], data[5] );
        __this->set_total_avg( data[3] );
      }

      if( !gpo_data.show_values ){ 
        continue; 
      }
      if( data[1] != 0 && gpconf.enable_ksps ){
        __this->set_sample_rate( data[1] );
      }
      __this->set_allow_motion( data[2] );
      __this->set_digital_state( data[4] );

    }
    vTaskDelete(NULL);
}   

IRAM_ATTR void GPO_SCOPE::set_digital_state( int state ){
    //if( 0 == state ){++gpo_data.dstate_count;}
    gpo_data.voltage_feedback = state;
    if( state < gpo_data.vrop_tresh ){
        state = 0;
    } else {
        state = 1;
    }
    gpo_data.dstate = state;
}
IRAM_ATTR void GPO_SCOPE::set_allow_motion( int value ){
    gpo_data.motion_signal = value;
}


IRAM_ATTR void GPO_SCOPE::set_total_avg( int16_t adc_value ){
    gpo_data.total_avg = adc_value;
}

IRAM_ATTR int16_t GPO_SCOPE::get_total_avg(){
    return gpo_data.total_avg;
    uint64_t average = 0;
    for( int i = 0; i < gpo_data.scope_width; ++i ){
        average += scope_values[i];
        //Serial.println( scope_values[i] );
    }
    average/=gpo_data.scope_width;
    return ( int16_t ) average;
}

IRAM_ATTR void GPO_SCOPE::set_motion_plan( int16_t plan ){
    gpo_data.motion_plan = plan;
}

IRAM_ATTR int GPO_SCOPE::add_to_scope( int16_t adc_value, int16_t plan ){
    //int64_t start = esp_timer_get_time();          
    gpo_data.last_adc_value = adc_value;
    if( !gpo_data.scope_active || gpo_data.scope_frame_ready ){
        vTaskDelay(1);
        return gpo_data.scope_frame_ready ? 1 : 0;
    }
    scope_values[gpo_data.scope_current_cursor]      = adc_value;
    scope_values_plan[gpo_data.scope_current_cursor] = plan;

    //gpo_data.scope_last_x = gpo_data.scope_current_cursor;
    //gpo_data.scope_last_y = scope_values[gpo_data.scope_current_cursor];
    if( ++gpo_data.scope_current_cursor >= gpo_data.scope_width ){
        gpo_data.scope_frame_ready    = true;
        gpo_data.scope_current_cursor = 0;
    }
    return gpo_data.scope_frame_ready ? 1 : 0;
}

IRAM_ATTR void GPO_SCOPE::draw_wave( int steps ){

    if( !gpo_data.scope_active || !gpconf.enablewave ){
        if( !gpo_data.show_values ){
            return;
        }
        canvas_b.deleteSprite();
        canvas_b.createSprite( gpo_data.scope_width_total-gpo_data.right_bar_width, 20 );
        draw_scope_meta();//vTaskDelay(1);
        canvas_b.pushSprite( gpo_data.scope_pos_x, gpo_data.scope_pos_y+20 );vTaskDelay(1);
        canvas_b.deleteSprite();
        return;
    }

    //reset();
    int16_t reference_y    = gpo_data.scope_height;
    int16_t start_x        = 0;
    int16_t start_y        = reference_y;
    int16_t adc_value      = 0;
    int16_t pixel_vertical = 0;
    int16_t current_x      = start_x+1;
    int16_t plan           = 0;
    int color              = TFT_GREEN;

    //int first = scope_values[0];
    //int last  = scope_values[gpo_data.scope_width-2];

    for( int i = 0; i < gpo_data.scope_width-1; ++i ){

        //###################################################
        // Calculate the pixel relative the the scope canvas
        //###################################################
        pixel_vertical = 0;
        adc_value      = scope_values[i];
        plan           = scope_values_plan[i];
        if( adc_value > 0 ){
            pixel_vertical = ( gpo_data.scope_height * 1000 / gpo_data.resolution * adc_value ) / 1000;
            if( pixel_vertical > gpo_data.scope_height ){
                pixel_vertical = gpo_data.scope_height;
            }
        }
        pixel_vertical+=1; // raise it a little

        //###################################################
        // Add coloration
        //###################################################
        if( plan < 0 ){
            plan*=-1;
        }

        color = TFT_GREEN;

        if( edm_process_is_running ){

        switch (plan)
        {
            case 1:
                color = TFT_WHITE;
            break;

            case 2:
                color = TFT_GREEN;
            break;

            case 3:
                color = TFT_PURPLE;
            break;

            case 4:
                color = TFT_ORANGE;
            break;

            case 5:
                color = TFT_MAROON;
            break;

            default:
                color = TFT_WHITE;
            break;

        }

        }

        //###################################################
        // Draw the line and update the last y position
        //###################################################
        current_x = start_x+1;
        if( gpconf.drawlinear ){
            canvas_b.drawFastVLine( current_x, gpo_data.scope_height-pixel_vertical, gpo_data.scope_height, color  );
        } else {
            canvas_b.drawLine( 
                start_x, 
                start_y, 
                current_x, 
                gpo_data.scope_height-pixel_vertical, 
                color 
            );
        }
        start_x = current_x;
        start_y = reference_y-pixel_vertical;

    }
    
    //###################################################
    // Calculate the pixel relative the the scope canvas
    //###################################################
    gpo_data.total_avg = get_total_avg();
    pixel_vertical = ( gpo_data.scope_height * 1000 / VSENSE_RESOLUTION * gpo_data.total_avg ) / 1000;
    if( pixel_vertical > gpo_data.scope_height ){
        pixel_vertical = gpo_data.scope_height;
    }
    color = avg_control();
    canvas_b.drawFastHLine( 0, gpo_data.scope_height-pixel_vertical, gpo_data.favg_size, color  );
    canvas_b.drawFastHLine( 0, gpo_data.scope_height-pixel_vertical+1 , gpo_data.favg_size, color  );
    pixel_vertical = ( gpo_data.scope_height * 1000 / VSENSE_RESOLUTION * gpo_data.voltage_feedback ) / 1000;
    canvas_b.drawFastHLine( gpo_data.scope_width-15, gpo_data.scope_height-pixel_vertical, 15, TFT_DARKGREY  );
    draw_scope_meta();//vTaskDelay(1);
    canvas_b.pushSprite( gpo_data.scope_pos_x, gpo_data.scope_pos_y+20 );vTaskDelay(1);
    canvas_b.deleteSprite();

}


IRAM_ATTR int GPO_SCOPE::avg_control(){
    //if( gpo_data.full_range_tresh == 0 ){ return TFT_WHITE; }
    //return gpo_data.total_avg > gpo_data.full_range_tresh ? TFT_RED : TFT_WHITE;
    return gpo_data.total_avg > gpo_data.setpoint_max ? TFT_RED : TFT_WHITE;
}

IRAM_ATTR bool GPO_SCOPE::avg_above_setpoint(){
    return gpo_data.average_above_setpoint;
}
IRAM_ATTR bool GPO_SCOPE::avg_above_center(){
    return gpo_data.average_above_center;
}
IRAM_ATTR bool GPO_SCOPE::avg_below_setpoint(){
    return gpo_data.average_below_setpoint;
}



IRAM_ATTR void GPO_SCOPE::draw_scope_meta(){
        if( gpo_data.show_values ){
            int pos_start = 5;

            canvas_b.drawNumber( gpo_data.voltage_feedback, pos_start, 2,  2 );
            pos_start += 43;
            canvas_b.drawNumber( gpo_data.total_avg, pos_start, 2, 2 );
            pos_start += 43;
            canvas_b.drawNumber( gpo_data.sample_rate, pos_start, 2, 2 );
            pos_start += 43;
            canvas_b.drawNumber( gconf.current_line, pos_start, 2,  2 );

            int color = TFT_MAROON;
            if( gpo_data.motion_signal == 1 ){
                color = TFT_GREEN;
            } else if( gpo_data.motion_signal == 2 ){
                color = TFT_ORANGE;
            }
            pos_start += 43;
            canvas_b.fillCircle( pos_start, 10, 3, color ); 
            pos_start += 10;
            canvas_b.fillCircle( pos_start, 10, 3, (gpo_data.dstate==1?TFT_GREEN:TFT_MAROON) ); 

        }

}

IRAM_ATTR void GPO_SCOPE::draw_header(){
    int pos_start = 5;
    canvas_b.deleteSprite();
    canvas_b.createSprite( gpo_data.scope_width_total, gpo_data.scope_height_total );vTaskDelay(1);
    canvas_b.fillSprite( TFT_BLACK );
    canvas_b.pushSprite( gpo_data.scope_pos_x, gpo_data.scope_pos_y );vTaskDelay(1);
    canvas_b.deleteSprite();
    canvas_b.createSprite( gpo_data.scope_width_total, 19 );vTaskDelay(1);
    canvas_b.fillSprite( TFT_DARKGREY );vTaskDelay(1);
    canvas_b.setTextColor( TFT_WHITE );
        
        canvas_b.drawString( "vFd", pos_start, 2,  2 );vTaskDelay(1);
        pos_start += 43;
        canvas_b.drawString( "cFd", pos_start, 2, 2 );vTaskDelay(1);
        pos_start += 43;
        canvas_b.drawString( "kSps", pos_start, 2, 2 );vTaskDelay(1);
        pos_start += 43;
        canvas_b.drawString( "Line", pos_start, 2, 2 );vTaskDelay(1);

    canvas_b.fillRect( gpo_data.scope_width_total-25, 0, 25, 19, TFT_MAROON );vTaskDelay(1);
    canvas_b.drawString( "T", gpo_data.scope_width_total-15, 2, 2 );vTaskDelay(1);
    canvas_b.pushSprite( gpo_data.scope_pos_x, gpo_data.scope_pos_y );vTaskDelay(1);
    canvas_b.deleteSprite();
    canvas_b.setTextColor( TFT_GREENYELLOW );
    gpo_data.header_drawn = true;
    draw_setpoint();
    reset();
}


IRAM_ATTR void GPO_SCOPE::setup(){
    #ifdef SCOPE_USE_QUEUE
        if( ! gpo_data.has_task_queue ){
            scope_queue = xQueueCreate( 1, sizeof(int16_t)*queue_item_size );
            xTaskCreatePinnedToCore(scope_task, "scope_task", 2000, this, 
            TASK_SCOPE_PRIORITY, 
            &scope_task_handle, xPortGetCoreID());
        }
    #endif
    gpo_data.has_task_queue = true;
    stop();
}
IRAM_ATTR float GPO_SCOPE::get_zoom(){
    return gpo_data.scope_ratio;
}
IRAM_ATTR void GPO_SCOPE::set_us_per_read( int16_t us_per_read ){
    gpo_data.us_per_read = us_per_read; // value in uS
}
IRAM_ATTR void GPO_SCOPE::set_sample_rate( int16_t sample_rate ){
    gpo_data.sample_rate = sample_rate; // value in khz
}
IRAM_ATTR void GPO_SCOPE::toogle_values(){
    gpo_data.show_values = !gpo_data.show_values;
}
IRAM_ATTR void GPO_SCOPE::draw_setpoint(){
    gpo_data.skip_push = true;
    while( gpo_data.canvas_processing ){
        vTaskDelay(1);
    }
    canvas_b.deleteSprite();
    canvas_b.createSprite( gpo_data.right_bar_width, gpo_data.scope_height );vTaskDelay(1);
    canvas_b.fillSprite( TFT_MAROON );vTaskDelay(1);
    int32_t min = ( gpo_data.scope_height - ( gpo_data.scope_height * 1000 / VSENSE_RESOLUTION * gpo_data.setpoint_min ) / 1000 );
    int32_t max = ( gpo_data.scope_height - ( gpo_data.scope_height * 1000 / VSENSE_RESOLUTION * gpo_data.setpoint_max ) / 1000 );
    canvas_b.fillRect( 0,  max, gpo_data.right_bar_width, min-max, TFT_GREEN );vTaskDelay(1);
    canvas_b.pushSprite( gpo_data.scope_pos_x+gpo_data.scope_width, gpo_data.scope_pos_y+20 );vTaskDelay(1);
    canvas_b.deleteSprite();
    gpo_data.skip_push = false;
}


IRAM_ATTR bool GPO_SCOPE::scope_is_running(){
    return gpo_data.scope_active;
}
IRAM_ATTR void GPO_SCOPE::start(){
    if( !gpo_data.header_drawn ){
         gscope.draw_header();
    }
    if( !gpo_data.scope_active ){
        //vTaskResume( scope_task_handle );
    }
    gpo_data.scope_active = true;
    gpo_data.has_off_note = false;
    reset();
}
IRAM_ATTR void GPO_SCOPE::stop(){
    if( gpo_data.scope_active ){
        //vTaskSuspend( scope_task_handle );
    }
    gpo_data.scope_active = false;
    gpo_data.header_drawn = false;
    gpo_data.has_off_note = false;
    #ifdef SCOPE_USE_QUEUE
        xQueueReset( scope_queue );
    #endif
    while( gpo_data.canvas_processing ){
        vTaskDelay(1);
    }
    canvas_b.deleteSprite();vTaskDelay(1);
}
IRAM_ATTR void GPO_SCOPE::init( int16_t width, int16_t height, int16_t posx, int16_t posy ){
    if( gpo_data.scope_active ){ stop(); }
    if( gpo_data.resolution < 1 ){
        set_scope_resolution(1.0);
    }
    gpo_data.header_drawn = false;
    setup();
    gpo_data.skip_push = true;
    while( gpo_data.canvas_processing ){
        vTaskDelay(1);
    }
    gpo_data.scope_width        = width-gpo_data.right_bar_width;
    gpo_data.scope_width_total  = width;
    gpo_data.scope_height_total = height;
    gpo_data.scope_height       = height-20;
    gpo_data.scope_pos_x        = posx;
    gpo_data.scope_pos_y        = posy;
    //gpo_data.scope_ratio  = 1.0;
    canvas_b.setTextColor( TFT_GREENYELLOW );
    reset();
    stop();
    gscope.draw_header();
    gpo_data.skip_push            = false;
    gpo_data.scope_frame_ready    = false;
}
IRAM_ATTR void GPO_SCOPE::set_scope_resolution( float ratio ){
    gpo_data.skip_push = true;
    if( ratio > 10.0 ){
        ratio = 10.0;
    } else if( ratio < 1.0 ){
        ratio = 1.0;
    }
    gpo_data.scope_ratio = ratio;
    gpo_data.resolution = round(float(VSENSE_RESOLUTION)/ratio);
    if( gpo_data.resolution < 1 ){
        gpo_data.resolution = VSENSE_RESOLUTION;
    }
    reset();
}
IRAM_ATTR void GPO_SCOPE::reset(){
    canvas_b.createSprite( gpo_data.scope_width, gpo_data.scope_height );vTaskDelay(1);
    gpo_data.scope_current_cursor = 0;
    gpo_data.scope_last_x         = 0;
    gpo_data.scope_last_y         = 0;
    gpo_data.skip_push            = false;
    gpo_data.scope_frame_ready    = false;
    gpo_data.last_adc_value       = 0;
}

IRAM_ATTR void GPO_SCOPE::toggle_scope( bool enable ){
    bool reactivate = gpo_data.scope_active;
    gpo_data.scope_active = false;
    #ifdef SCOPE_USE_QUEUE
        xQueueReset( scope_queue );
        vTaskSuspend( scope_task_handle );
    #endif
    while( gpo_data.canvas_processing ){
        vTaskDelay(1);
    }
    gpo_data.skip_push    = true;
    gpo_data.has_off_note = false;
    gpconf.enablewave     = enable;
    gpo_data.scope_active = reactivate;
    #ifdef SCOPE_USE_QUEUE
        vTaskResume( scope_task_handle );
    #endif
    reset();
}


IRAM_ATTR void GPO_SCOPE::zoom(){
    gpo_data.skip_push = true;
    stop();
    //vTaskDelay(1);
    if( gpo_data.scope_ratio> 4.5 ){
        gpo_data.scope_ratio = 1.0; 
    } else {
        gpo_data.scope_ratio += 0.5;
    }
    set_scope_resolution( gpo_data.scope_ratio );
    reset();
    start();
}

IRAM_ATTR void GPO_SCOPE::draw_error( int error_code ){
    String text = "";
    switch ( error_code ){
        case 1:
            text = "Check wire";
            break;
        case 2:
            text = "Shorted";
            break;
        case 3:
            text = "Shorted";
            break;     
        case 4:
            text = "MP timeout";
            break; 
        case 5:
            text = "Retract limit";
            break; 

        default:
            text = "Unknown error";
            break;
    }
    canvas_b.deleteSprite();
    canvas_b.createSprite( gpo_data.scope_width_total/2, 15 );vTaskDelay(1);
    canvas_b.fillSprite( TFT_BLACK );
    canvas_b.setTextColor( TFT_RED );
    canvas_b.drawString( text, 5, 0, 2 );
    canvas_b.pushSprite( gpo_data.scope_pos_x, gpo_data.scope_pos_y+gpo_data.scope_height_total-15 );vTaskDelay(1);
    canvas_b.deleteSprite();
}


IRAM_ATTR void GPO_SCOPE::draw_scope(){
    if( gconf.edm_pause_motion && gconf.process_error != 0 ){
        draw_error( gconf.process_error );
    }
    if( !gpo_data.scope_active || gpo_data.canvas_processing || ( !gpo_data.scope_frame_ready && gpconf.enablewave ) ){ 
        return; 
    }
    if( ! gpo_data.skip_push ){
        gpo_data.canvas_processing = true;
        draw_wave();
        reset(); 
        gpo_data.canvas_processing = false;
    }
}
IRAM_ATTR bool GPO_SCOPE::is_blocked(){
    return gpo_data.scope_frame_ready;
}
IRAM_ATTR void GPO_SCOPE::update_setpoint( uint32_t smin, uint32_t smax ){
    gpo_data.setpoint_min    = smin;
    gpo_data.setpoint_max    = smax;
    if( gpo_data.scope_active ){
        draw_setpoint();
    }
}
int16_t IRAM_ATTR GPO_SCOPE::adc_to_voltage( int16_t adc_value ){
    if( adc_value <= 0 ){ return 0; }
    return round(VSENSE_MAX*1000/VSENSE_RESOLUTION*adc_value/1000);
}
IRAM_ATTR void GPO_SCOPE::set_vdrop_treshhold( int tresh ){
    gpo_data.vrop_tresh = tresh;
}
IRAM_ATTR void GPO_SCOPE::set_full_range_size( int size ){
    //
    gpo_data.favg_size = size;
}