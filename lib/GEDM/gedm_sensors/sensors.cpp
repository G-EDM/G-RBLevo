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
#include "sensors.h"
#include "tft_display/ili9341_tft.h"
#include "tft_display/gpo_scope.h"
#include <soc/rtc_wdt.h>
#include <driver/adc.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <esp_attr.h>
#include <esp_adc_cal.h>
#include <driver/i2s.h>
#include "MotionControl.h"

#include "soc/syscon_reg.h"
#include "soc/syscon_struct.h"


TaskHandle_t vsense_tasks[vsense_num_tasks];
TaskHandle_t main_sensor_task;

xQueueHandle vsense_queue      = NULL; 
xQueueHandle motion_plan_queue = NULL; 
xQueueHandle sensor_queue_main = NULL;


hw_timer_t * vsense_adc_sampler_timer = NULL; 
hw_timer_t * benchmark_timer          = NULL; 

DRAM_ATTR edge_control                 edge_ctrl;
DMA_ATTR volatile SENSOR_STATES        sensors;
DMA_ATTR volatile adc_readings         adc_data;
DMA_ATTR volatile adc_setpoints        setpoints;
DMA_ATTR volatile I2S_conf             i2sconf;
DMA_ATTR volatile adc_sampling_stats   adc_stats;
DMA_ATTR volatile adc_sampling_params  adc_params;
DMA_ATTR volatile ADC_WAVE_CONTROL     adc_wave;
DMA_ATTR volatile adc_critical_config  adc_critical_conf;
DMA_ATTR volatile ADC_TIMER            adc_hw_timer;
DMA_ATTR volatile SECOND_FEEDBACK_DATA sfeedback;
DMA_ATTR volatile spark_counter_data   spark_counter;
DMA_ATTR volatile pulse_extractor      pex_data;

DMA_ATTR int16_t data_stats[queue_item_size];
DMA_ATTR int64_t i2s_read_start = esp_timer_get_time();


DMA_ATTR volatile int      adc_counter         = 0;
DMA_ATTR volatile int64_t  adc_timer           = 0;
DMA_ATTR volatile int      reference_adc_value = 0;
DMA_ATTR volatile uint32_t feedback_voltage    = 0;
DMA_ATTR volatile int      counter[10]         = {0,0,0,0,0,0,0,0,0,0};
DMA_ATTR volatile int      multisample_counts  = 0;
DMA_ATTR volatile uint32_t multisample_buffer[vsense_sampler_buffer_size];
bool is_ready = false;
DMA_ATTR bool benchmark_sample_rate = false;

DMA_ATTR int bench_timer_interval               = 1000;
DMA_ATTR volatile bool block_short_low_override = false;



static esp_adc_cal_characteristics_t adc1_chars;



int IRAM_ATTR gpio_35_read( bool deep_check = false ){
    int state = 1;
    #ifndef USE_DUAL_CHANNEL
    //uint32_t state = (gpio_input_get_high()>>(DIGITAL_INPUT_FEED_HOLD - 32))&BIT0;
    state = (REG_READ(GPIO_IN1_REG) >> (DIGITAL_INPUT_FEED_HOLD-32)) & 0x01;
    //if( deep_check && 0 == state ){ state = digitalRead( DIGITAL_INPUT_FEED_HOLD ); }
    if( state == 1 ){
        sfeedback.pin_is_low = false;
        //if( !block_short_low_override) { sfeedback.count = 0; }
    } else {
        sfeedback.pin_is_low = true;
        if( ++sfeedback.count > 100000 ){
            sfeedback.count = 100000;
        }
    }
    #endif
    return state;
}


void IRAM_ATTR gpio_35_on_interrupt( void ){
    /*int state = (REG_READ(GPIO_IN1_REG) >> (DIGITAL_INPUT_FEED_HOLD-32)) & 0x01;
    if( state == 1 ){
        sfeedback.had_short_condition_b = 1;
    }*/
}





G_EDM_SENSORS::G_EDM_SENSORS(){}

void G_EDM_SENSORS::init_adc(){
    //###################################################
    // Create the queues
    //###################################################
    vsense_queue       = xQueueCreate( 1,  sizeof(int) ); 
    motion_plan_queue  = xQueueCreate( 1,  sizeof(int) );
    sensor_queue_main  = xQueueCreate( 20, sizeof(int) );
    //###################################################
    // Start tasks
    //###################################################
    xTaskCreatePinnedToCore( default_sensor_task, "sensor_task", 1500, this, 
        1,//TASK_SENSORS_DEFAULT_PRIORITY, 
        &main_sensor_task, 0);
    xTaskCreatePinnedToCore( vsense_task_listener, "vsense_task_listener", 4000, this, 
        TASK_VSENSE_RECEIVER_PRIORITY, 
        &vsense_tasks[1],  !i2sconf.I2S_core_id); // core one can give 2mSps but kills the motion
}

void G_EDM_SENSORS::run_sensor_service(){
    vTaskDelay(10);
    adc1_config_width(VSENSE_BIT_WIDTH);
    adc1_config_channel_atten( VSENSE_CHANNEL, ADC_ATTEN_11db );
    #ifdef USE_DUAL_CHANNEL
        adc1_config_channel_atten(BSENSE_CHANNEL, ADC_ATTEN_DB_11);
        adc1_get_raw( BSENSE_CHANNEL ); 
    #endif
    adc1_get_raw( VSENSE_CHANNEL ); 
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, VSENSE_BIT_WIDTH, 1100, &adc1_chars);
    pinMode( VSENSE_FEEDBACK_PIN,     INPUT );
    pinMode( ON_OFF_SWITCH_PIN,       INPUT );
    pinMode( STEPPERS_LIMIT_ALL_PIN,  INPUT );
    pinMode( DIGITAL_INPUT_FEED_HOLD, INPUT );
    #ifndef USE_IDF_650
        pinMode(VSENSE_FEEDBACK_PIN, GPIO_MODE_INPUT_OUTPUT); // fix problem on espressif 6.5
        pinMode(DIGITAL_INPUT_FEED_HOLD, GPIO_MODE_INPUT_OUTPUT); // fix problem on espressif 6.5
    #endif
    attachInterrupt( ON_OFF_SWITCH_PIN,      switch_on_interrupt,       CHANGE );
    attachInterrupt( STEPPERS_LIMIT_ALL_PIN, limit_switch_on_interrupt, CHANGE );
    //#ifndef USE_DUAL_CHANNEL
        //attachInterrupt( DIGITAL_INPUT_FEED_HOLD, gpio_35_on_interrupt, FALLING );
        //gpio_35_on_interrupt();
    //#endif
    i2sconf.I2S_failure = false;
    is_ready            = true;
}

void IRAM_ATTR single_shot_core_config(void *parameter){
    vTaskDelete(NULL);
}


void IRAM_ATTR G_EDM_SENSORS::reset_edge(){
    edge_ctrl.rising_edge_start          = esp_timer_get_time();
    edge_ctrl.locked                     = false;
    edge_ctrl.edges_total                = 0;
    edge_ctrl.edges_adc_total            = 0;
    edge_ctrl.edges_solved_total         = 0;
    edge_ctrl.has_unsolved_edge          = false;
    edge_ctrl.edge_solved_after          = 0;
    edge_ctrl.edge_solved_in_cycle       = true;
    edge_ctrl.multiple_edges_total       = 0;
}

void IRAM_ATTR G_EDM_SENSORS::reset_sensor(){
    counter[0] = 0;
    counter[1] = 0;
    counter[2] = 0;
    counter[3] = 0;
    counter[4] = 0;
    counter[5] = 0;
    counter[6] = 0;
    counter[7] = 0;
    counter[8] = 0;
    reset_edge();
    adc_wave.multiple_rising_edges_count = 0;
    adc_wave.rising_edge_block         = false;
    adc_wave.rising_edge_normalized    = true;
    adc_wave.adc_pre_rising_edge       = -1;
    adc_wave.last_rising_edge_adc      = setpoints.upper; // dirty solution from the past. No idea why.
    adc_wave.adc_collected             = 0;
    adc_wave.edge_recovery_failed      = false;
    block_short_low_override = false;
    ui_controller.reset_pwm_state();
    adc_data.zeros_in_a_row = 0;
    adc_data.highs_in_a_row = 0;
    adc_data.previous_edge  = 0;
    sfeedback.reset_count   = 0;
    sfeedback.had_short_condition = 0;
}

uint32_t IRAM_ATTR read_multisample( int raw, bool write_only, int num_samples, bool read_only ){
    uint32_t average = 0;
    if( !read_only ){
        if(++multisample_counts == vsense_sampler_buffer_size){
            multisample_counts = 0;
        }
        multisample_buffer[multisample_counts] = raw;
    } 
    if( !write_only ){
        int history_depth = num_samples;
        int index         = multisample_counts;
        for( int i = 0; i<history_depth; ++i ){
            average += multisample_buffer[index];
            if( index == 0 ){
                index = vsense_sampler_buffer_size;
            }
            index--;
        }
        average/=history_depth;
    } else {
        average = raw;
    }
    return average;
}

bool IRAM_ATTR I2S_is_ready( bool include_all ){
    if( !include_all ){
        if( !i2sconf.I2S_READY ){ 
            return false; 
        }
    } else if( !i2sconf.I2S_READY || i2sconf.I2S_failure || i2sconf.I2S_is_restarting ){ 
        return false; 
    }
    return true;
}
void IRAM_ATTR stop_i2s( void ){
    i2sconf.I2S_READY = false;
    while( i2sconf.I2S_busy ){
        vTaskDelay(1);
    }
    i2sconf.I2S_READY = false;
    #ifdef SCOPE_USE_QUEUE
        xQueueReset( scope_queue );
    #endif
    i2s_stop(I2S_NUM_0);
    i2s_adc_disable(I2S_NUM_0);
    i2s_driver_uninstall( I2S_NUM_0 );
}
void IRAM_ATTR restart_i2s() {
    if( i2sconf.I2S_is_restarting ){ return; }
    i2sconf.I2S_is_restarting = true;
    stop_i2s();
    configure_i2s();
    i2sconf.I2S_is_restarting = false;
    i2sconf.I2S_FORCE_RESTART = false;
    adc_counter = 0;
}

void IRAM_ATTR configure_i2s() {
    gscope.set_full_range_size( i2sconf.I2S_full_range );
    gscope.set_vdrop_treshhold( adc_critical_conf.voltage_feedback_treshhold );

    i2sconf.num_bytes = sizeof(uint16_t) * i2sconf.I2S_buff_len;

    #ifdef USE_IDF_650
    i2s_config_t i2s_config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate          = i2sconf.I2S_sample_rate,              
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT, 
        .channel_format       = I2S_CHANNEL_FMT_ALL_LEFT,//I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_MSB);,
        .intr_alloc_flags     = (ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_INTRDISABLED),//ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = i2sconf.I2S_buff_count,
        .dma_buf_len          = i2sconf.I2S_buff_len,
        .use_apll             = true,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = 2000000
    };
    #else

    i2s_config_t i2s_config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate          = i2sconf.I2S_sample_rate,              
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT, 
        .channel_format       = I2S_CHANNEL_FMT_ALL_LEFT,//I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags     = 1,
        .dma_buf_count        = i2sconf.I2S_buff_count,
        .dma_buf_len          = i2sconf.I2S_buff_len,
        .use_apll             = true,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = 2000000
    };

    #endif

    while( ESP_OK != i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) ){ vTaskDelay(5); }
    i2s_set_adc_mode(ADC_UNIT_1, VSENSE_CHANNEL);
    i2s_adc_enable(I2S_NUM_0);

    #ifdef USE_DUAL_CHANNEL
        SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, 1, SYSCON_SARADC_SAR1_PATT_LEN_S);
        WRITE_PERI_REG(SYSCON_SARADC_SAR1_PATT_TAB1_REG, 0x7F6F0000);
        //SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, 7, SYSCON_SARADC_SAR1_PATT_LEN_S);
        
        #ifndef USE_IDF_650
            SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);
        #endif

    #else

    #endif

    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2s_start(I2S_NUM_0);
    i2sconf.I2S_failure         = false;
    i2sconf.I2S_READY           = true;
    i2sconf.I2S_failure_count   = 0;
    i2sconf.I2S_equal_reading   = 0;
    i2sconf.block_write_out     = false;

    //SYSCON.saradc_ctrl2.meas_num_limit = 0;

}




void G_EDM_SENSORS::generate_reference_voltage(){
    generate_setpoint_min_max_adc( 0 );
}

void IRAM_ATTR G_EDM_SENSORS::generate_setpoint_min_max_adc( int adc_reading_reference ){
    if( adc_reading_reference <= 0 ){
        adc_reading_reference = reference_adc_value;
    }
    float resolution;
    resolution        = float( VSENSE_RESOLUTION ) / 100.0;
    setpoints.lower   = round( resolution * vsense_drop_range_min );
    setpoints.upper   = round( resolution * vsense_drop_range_max );
    setpoints.probing = round( resolution * probe_trigger_current ); 
    gscope.update_setpoint( setpoints.lower, setpoints.upper );
    reset_sensor();
}

int G_EDM_SENSORS::percentage_to_mv( float percent ){
    float adc_percentage = float( VSENSE_RESOLUTION ) / 100.0 * percent;
    return round( VSENSE_MAX*1000 / VSENSE_RESOLUTION * int( adc_percentage ) / 1000 );
}
uint32_t IRAM_ATTR G_EDM_SENSORS::get_adc_value( int index ){
    int adc_value = 0;
    return adc_data.sampled;
}
uint32_t G_EDM_SENSORS::get_feedback_voltage( int adc_value )
{
    if( adc_value == 0 ){ adc_value = get_adc_value(1); }
    feedback_voltage = adc_to_voltage( adc_value );
    return feedback_voltage;
}
bool G_EDM_SENSORS::is_probing(){
    return sys_probe_state == Probe::Active ? true : false;
}
bool G_EDM_SENSORS::continue_edm_process(){
  return stop_edm_task? false : true;
}
bool G_EDM_SENSORS::start_edm_process(){
  if( 
    sys_rt_exec_alarm != ExecAlarm::None 
    || gconf.gedm_stop_process 
    || sys_rt_exec_state.bit.motionCancel
){
    return false;
  } return start_edm_task;
}
uint32_t IRAM_ATTR G_EDM_SENSORS::adc_to_voltage( int adc_value ){
    if( adc_value <= 0 ){ return 0; }
    return round(VSENSE_MAX*1000/VSENSE_RESOLUTION*adc_value/1000);
    //return esp_adc_cal_raw_to_voltage(adc_value, &adc1_chars); // not so good.. Offsets the 0v input to 142mV and may need calibration.
}



//###########################################################################
// Flag estop event
//###########################################################################
void IRAM_ATTR switch_on_interrupt(){
    sensors.on_off_switch_event_detected = true;
    int data = 1;
    xQueueSendFromISR( sensor_queue_main, &data, NULL );
}
//###########################################################################
// Flag limit event
//###########################################################################
void IRAM_ATTR limit_switch_on_interrupt() {
    if( 
        !gconf.gedm_disable_limits && 
        ( sys.state    != State::Alarm 
        && sys.state != State::Homing 
        && sys_rt_exec_alarm == ExecAlarm::None )
    ){
        sensors.limit_switch_event_detected = true;
        int data = 2;
        xQueueSendFromISR( sensor_queue_main, &data, NULL );
    }
}
//###########################################################################
// Check the limit switch
//###########################################################################
bool IRAM_ATTR G_EDM_SENSORS::limit_switch_read(){
    vTaskDelay( 32 / portTICK_PERIOD_MS ); 
    bool state  = false;
    if ( GRBL_LIMITS::limits_get_state() )
    {
        mc_reset(); 
        sys_rt_exec_alarm = ExecAlarm::HardLimit;
        if( 
            sys.state    != State::Alarm 
            && sys.state != State::Homing 
            && sys_rt_exec_alarm == ExecAlarm::None
        ){
            //sys_rt_exec_state.bit.motionCancel = true; //problem with homing.. no time to fix it yet
        }

        state = true;
    }
    else
    {
        state = false;
    }
    sensors.limit_switch_event_detected = false;
    return state;
}
//###########################################################################
// Check the estop switch
//###########################################################################
IRAM_ATTR bool G_EDM_SENSORS::edm_start_stop()
{
    vTaskDelay( 32 / portTICK_PERIOD_MS ); 
    bool state  = false;
    if (digitalRead(ON_OFF_SWITCH_PIN))
    {
        stop_edm_task = 0;
        if (!edm_process_is_running)
        {
            start_edm_task = 1;
        }
        if (sys_rt_exec_state.bit.motionCancel)
        {
            force_redraw = true;
        }
        sys_rt_exec_state.bit.motionCancel = false;
        state = true;
    }
    else
    {
        start_edm_task = 0;
        if (edm_process_is_running)
        {
            stop_edm_task = 1;
        }
        if (!sys_rt_exec_state.bit.motionCancel)
        {
            force_redraw = true;
        }
        sys_rt_exec_state.bit.motionCancel = true;
        state = false;
    }
    sensors.on_off_switch_event_detected = false;
    return state;
}

void IRAM_ATTR bench_on_timer() {
    if( benchmark_sample_rate ){ return; }
    benchmark_sample_rate = true;
}






//###########################################################################
// To increase sampling it is possible to enable a hardware timer interrupt
// For 20khz the hardware timer is not needed
// With HW timer disabled it will read the I2S buffer only a in sync
// with the PWM rising edge. 
//###########################################################################
void IRAM_ATTR vsense_on_timer() {
  if( 
    #ifndef DEBUG_PROCESS_SCREEN
        //( edm_process_is_running && pconf.pwm_is_off )
        xQueueIsQueueEmptyFromISR( vsense_queue ) == pdFALSE
        || ( edm_process_is_running && ( gconf.edm_pause_motion || !gconf.gedm_planner_line_running ) ) // prevent sampling between lines to not mess with sd access
    #else
        xQueueIsQueueEmptyFromISR( vsense_queue ) == pdFALSE
    #endif
  ){ return; }
  int priority = 1; // lower priority
  xQueueOverwriteFromISR(vsense_queue, &priority, NULL);
}
void G_EDM_SENSORS::enable_hw_timer( int _val ){
    if( !adc_hw_timer.use_hw_timer ){
        if( adc_hw_timer.timer_is_running ){
            disable_hw_timer();
        }
        return;
    }
    if( adc_hw_timer.timer_is_running ){ return; }
    adc_hw_timer.timer_is_running = true;
    vsense_adc_sampler_timer = timerBegin(3, 80, true);
    timerAttachInterrupt(vsense_adc_sampler_timer, &vsense_on_timer, true);
    if( _val == 0 ){ _val = adc_hw_timer.interval; }
    timerAlarmWrite(vsense_adc_sampler_timer, _val, true);
    timerAlarmEnable(vsense_adc_sampler_timer);
}
void G_EDM_SENSORS::disable_hw_timer( void ){
    if( !adc_hw_timer.timer_is_running ){ return; }
    timerEnd( vsense_adc_sampler_timer );
    adc_hw_timer.timer_is_running = false;
}



//##############################################
// Default event queue loop
//##############################################
void IRAM_ATTR default_sensor_task(void *parameter)
{
    //##############################################
    // Timer interrupt for benchmarking the kSps
    //##############################################
    benchmark_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(benchmark_timer, &bench_on_timer, true);
    timerAlarmWrite(benchmark_timer, bench_timer_interval*1000, true);
    timerAlarmEnable(benchmark_timer);
    int data = 0;
    for (;;)
    {
        //###################
        // Enter wait queue 
        //###################
        xQueueReceive( sensor_queue_main, &data, portMAX_DELAY ); 
        switch ( data )
        {
            case 1:
                G_EDM_SENSORS::edm_start_stop();
                break;
            case 2:
                G_EDM_SENSORS::limit_switch_read();
                break;
            case 3:
                break;
            case 4:
                ui_controller.pwm_off();
                break;
            case 5:
                ui_controller.pwm_on();
                break;
            case 6:
                ui_controller.stop_spindle(); 
                break;
            case 7:
                ui_controller.start_spindle();
                break;
            case 8:
                i2sconf.I2S_stop = true;
                while( !i2sconf.I2S_is_paused ){
                    vTaskDelay(1);
                }
                break;
            case 9:
                i2sconf.I2S_start = true;
                while( i2sconf.I2S_is_paused ){
                    vTaskDelay(1);
                }
                break;
            case 10:
                G_EDM_SENSORS::enable_hw_timer();
                break;
            case 11:
                G_EDM_SENSORS::disable_hw_timer();
                break;
            case 12:
                break;
            case 13:
                // edm process pause start
                break;
            default:
                break;
        }
        data = 0;
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}











void IRAM_ATTR change_sampling_rate( int rate ){
    if( rate > I2S_SAMPLE_RATE ){
        rate = I2S_SAMPLE_RATE;
    } else if ( rate < I2S_SAMPLE_RATE/10 ){
        rate = I2S_SAMPLE_RATE/10;
    }
    i2sconf.I2S_sample_rate = rate;
    i2sconf.block_write_out = true;
    vTaskDelay(1);
}
int32_t IRAM_ATTR G_EDM_SENSORS::get_avg( bool rising ){
    return rising ? adc_params.rising_avg : adc_params.falling_avg;
}
void IRAM_ATTR G_EDM_SENSORS::set_avg( bool rising, int avg ){
    if( rising ){
        if( avg >= ( vsense_sampler_buffer_size-1 ) ){
            avg = vsense_sampler_buffer_size-1;
        } 
        if( avg <= 0 ){
            avg = 1;
        }
        adc_params.rising_avg = avg;
    } else {
        if( avg < 0 ){
            avg = 0;
        } else if ( avg > 99000 ){
            avg = 99000;
        }
        adc_params.falling_avg = avg;
    }
}
void IRAM_ATTR G_EDM_SENSORS::set_avg_default( int avg ){
    if( avg >= ( vsense_sampler_buffer_size-1 ) ){
        avg = vsense_sampler_buffer_size-1;
    } else if( avg <= 0 ){
        avg = 1;
    }
    adc_params.default_avg = avg;
}
int32_t IRAM_ATTR G_EDM_SENSORS::get_avg_default(){
    return adc_params.default_avg;
}
int32_t IRAM_ATTR G_EDM_SENSORS::get_avg_i2s(){
    return adc_params.i2s_avg;
}
void IRAM_ATTR G_EDM_SENSORS::set_avg_i2s( int avg ){
    if( avg <= 0 ){
        avg = 1;
    }
    adc_params.i2s_avg = avg;
}

typedef struct i2s_shift_out_data {
    uint64_t read_sum_a = 0;
    uint64_t read_sum_b = 0;
    int16_t  count_a    = 0;
    int16_t  count_b    = 0;
    int16_t highest_adc = 0;
    int last_adc      = 0;
    bool success      = false;
    int  samples_read = 0;
    uint64_t read_sum = 0;
    int collected     = 0;
    int use_samples   = 0;
    int16_t sample    = 0;
    int64_t timestamp = 0;
    int num_samples_in_pool = 0;
} i2s_shift_out_data;

DMA_ATTR i2s_shift_out_data i2s_data;




bool IRAM_ATTR G_EDM_SENSORS::adc_to_scope( bool recent ){
    // associated motion plan 
    data_stats[5] = adc_data.plan;
    // add the digital state of the short circuit pin
    //data_stats[4] = adc_data.voltage_channel_shorts;//adc_data.voltage_channel_adc_recent;//sfeedback.pin_is_low ? 0 : 1;
    data_stats[4] = adc_data.voltage_channel_adc_recent;//adc_data.voltage_channel_adc_recent;//sfeedback.pin_is_low ? 0 : 1;
    // add the adc reading 
    data_stats[0] = recent ? adc_data.recent : adc_data.sampled;
    // add the sample rate
    data_stats[1] = adc_stats.sample_rate;
    // add the stop/hold/feed signal state
    if( adc_data.zeros_in_a_row >= adc_critical_conf.zeros_in_a_row_max ){
        data_stats[2] = 1;
    } else if( adc_data.highs_in_a_row >= adc_critical_conf.highs_in_a_row_max ){
        data_stats[2] = 0;
    } else {
        data_stats[2] = 2;
    }
    // add the full range average if needed
    data_stats[3] = recent ? adc_data.recent : adc_data.avg_full_range;
    if( benchmark_sample_rate ){ // calculate kSps
        adc_counter          *= i2sconf.I2S_buff_len;
        adc_stats.sample_rate = ( int16_t )( adc_counter/bench_timer_interval );
        adc_counter           = 0;
        benchmark_sample_rate = false;
    }
    // push it to the queue if not blocked
    if( !gscope.is_blocked() ){
        xQueueOverwriteFromISR( scope_queue, &data_stats, NULL );   
        return true;             
    }  return false;
}


//###########################################################################
// Shift out an I2S buffer
//###########################################################################
bool IRAM_ATTR i2s_shift_out(){
    if( 
        i2sconf.I2S_busy || !I2S_is_ready() 
    ){ return true; }
    i2sconf.I2S_busy             = true;
    i2s_data.success             = false;
    i2s_data.samples_read        = 0;

    i2s_data.read_sum_a = 0;
    i2s_data.read_sum_b = 0;
    i2s_data.count_a    = 0;
    i2s_data.count_b    = 0;
    adc_data.voltage_channel_shorts = 0;

    uint16_t adc_batch[i2sconf.I2S_buff_len];
    size_t   bytes_read;
    //###########################################################################
    // Get a buffer
    //###########################################################################
    if( ESP_OK == i2s_read(I2S_NUM_0, &adc_batch, ( int ) i2sconf.num_bytes, &bytes_read, I2S_TIMEOUT_TICKS) ){
        i2s_data.samples_read = bytes_read / 2;
        if( i2s_data.samples_read == i2sconf.I2S_buff_len ){
            i2s_data.success = true;
        }
    } 

    ++adc_counter;
    //###########################################################################
    // Add some black magic to the adc value to create the waveform
    //###########################################################################
    if( i2s_data.success && !i2sconf.block_write_out ){

        //###########################################################################
        // Process the I2S buffer
        // The reading produced with the old IDF is inverted
        // 4096 == no signal while 0 = max voltage
        // this is fixed with the new version (not used for now)
        //###########################################################################
        uint16_t sample           = 0;
        uint8_t  selected_channel = 0;

        for (int i = 0; i < bytes_read / 2; i++){

                selected_channel = (adc_batch[i] >> 12) & 0x07;
                sample           = adc_batch[i] & 0xfff;

                if( selected_channel == 6 ){ // current sense
                    i2s_data.read_sum_a += sample;
                    ++i2s_data.count_a;
                } else if( selected_channel == 7 ) { // voltage sense
                    i2s_data.read_sum_b += sample;
                    ++i2s_data.count_b;
                    if( sample < adc_critical_conf.voltage_feedback_treshhold ){
                        ++adc_data.voltage_channel_shorts;
                    }
                } 

        }
        adc_data.voltage_channel_adc_recent = i2s_data.read_sum_b > 0 ? int( i2s_data.read_sum_b / i2s_data.count_b ) : 0; 
        adc_data.recent                     = i2s_data.read_sum_a > 0 ? int( i2s_data.read_sum_a / i2s_data.count_a ) : 0; 
    }
    i2sconf.I2S_busy = false;
    return i2s_data.success;
}




int IRAM_ATTR calculate_cycles( int duration ){
    if( adc_stats.us_per_read == 0 ){
        return 1;
    }
    return MAX( 1, round( duration / ( int ) adc_stats.us_per_read ) );
}





void IRAM_ATTR vsense_task_listener(void *parameter){ 
    vTaskDelay(100);
    configure_i2s();
    vTaskDelay(100);
    while( !I2S_is_ready() ){ vTaskDelay(10); } // wait until I2S is ready
    vTaskDelay(10);
    int data = 1;
    for(;;){

        if( ! i2sconf.I2S_READY ){ 
            vTaskDelay(10); 
            continue;
        } else if( i2sconf.I2S_FORCE_RESTART ){
            restart_i2s();
        } 

        i2s_read_start = esp_timer_get_time();
        xQueueReceive(vsense_queue, &data, portMAX_DELAY);
        if( i2sconf.block_write_out || i2sconf.I2S_is_paused ){ 
            vTaskDelay(1); 
            continue;
        }

        i2s_data.last_adc    = adc_data.recent;
        i2s_data.use_samples = MIN( adc_params.i2s_avg, i2s_data.samples_read );
        i2s_data.sample      = -1;  
        
        adc_edge_data edge;

        //#########################################################
        // Get the I2S buffer and parse it to a single ADC value
        //#########################################################
        if( !i2s_shift_out() ){ 
            continue; 
        }

        read_voltage_feedback( adc_data );

        if( !G_EDM_SENSORS::is_probing() ){
            if( 
                block_short_low_override // set to true if pwm protection turn off was triggered; this is just to skip a reading with the off time results;
                || sfeedback.had_short_condition>0 
            ){ 
                if( --sfeedback.had_short_condition < 0 || adc_data.recent > setpoints.lower ){
                    sfeedback.had_short_condition = 0;
                }
                block_short_low_override = false;
                if( adc_data.recent < setpoints.lower ){
                    G_EDM_SENSORS::pwm_off_protection( false );
                    continue; // ignore the reading after a manual pwm off
                }
            }
            G_EDM_SENSORS::pwm_off_protection( false );
        }

        //################################################################
        // Wait until the adc pool is filled and select the highest value
        //################################################################
        int pool_size = gconf.gedm_retraction_motion ? i2sconf.I2S_pool_size+1 : i2sconf.I2S_pool_size;
        if( 
            ++i2s_data.num_samples_in_pool <= pool_size 
        ){
            i2s_data.highest_adc = MAX( adc_data.recent, i2s_data.highest_adc );
            if( i2s_data.highest_adc <= setpoints.lower ){ // only collect more if forward motion is indicated
                G_EDM_SENSORS::pwm_off_protection( false );
                continue;
            }
        } 

        adc_data.recent              = i2s_data.highest_adc;
        i2s_data.highest_adc         = 0;
        i2s_data.num_samples_in_pool = 0;
    
        if( !pconf.pwm_is_off ){
            //###########################################################################
            // Determine the edge type and produce the motion plan
            //###########################################################################
            G_EDM_SENSORS::get_edge_type( edge, adc_data.sampled, adc_data.recent );
            G_EDM_SENSORS::edge_handler( edge, adc_data );  
            adc_data.sampled        = read_multisample( adc_data.recent, false, adc_params.default_avg, false );                
            adc_data.avg_full_range = read_multisample( 0, false, i2sconf.I2S_full_range, true );
            adc_data.plan           = G_EDM_SENSORS::get_motion_plan( adc_data );   
            int plan                = adc_data.plan;
            //###########################################################################
            // Distribute the motion plan
            //###########################################################################
            xQueueOverwriteFromISR(  motion_plan_queue, &plan, NULL ); // push data to the queue used for the motion
            G_EDM_SENSORS::adc_to_scope( false );
        }

        adc_data.timestamp    = esp_timer_get_time();
        adc_stats.us_per_read = int( adc_data.timestamp - i2s_read_start );

    }
    vTaskDelete(NULL);
}




void IRAM_ATTR G_EDM_SENSORS::get_edge_type( adc_edge_data &edge, int16_t adc_old, int16_t adc_new ){
    //##############################################################################
    // Determine edge type
    //##############################################################################
    edge.is_rising_edge  = false;
    edge.is_falling_edge = false;
    if( adc_new > adc_old + adc_critical_conf.edge_treshhold ){ 
        edge.is_rising_edge = true;
    } else if( 
        adc_wave.rising_edge_block &&
        adc_new < adc_old - adc_critical_conf.edge_treshhold 
    ){
        edge.is_falling_edge = true;
    } else {
        // no load condition after a rising+falling edge
        // this would end in a motion block loop
        // this can be a load and a no load situation
        // edge not rising but still at an equal level to another rising edge
        // or edge not falling but still at a low level
        int force_recovery_after_pulses = 10;
        int total_time_for_recovery = pconf.pwm_period_us * force_recovery_after_pulses;
        if( adc_wave.falling_edge_block || adc_wave.rising_edge_block ){
            if( esp_timer_get_time()-adc_wave.falling_edge_block_start_time > total_time_for_recovery ){
                adc_wave.falling_edge_block = false;
                adc_wave.rising_edge_block  = false;
            }
        }
    }
}


IRAM_ATTR bool G_EDM_SENSORS::edge_handler( adc_edge_data &edge, volatile adc_readings &adc_data ){
    adc_data.edge_type = 0; // default flag
    if( edge.is_rising_edge ){

        ++adc_wave.multiple_rising_edges_count;
        //adc_data.rising_edge_ignition_delay = esp_timer_get_time()+pconf.pwm_period_us*2;
        //###########################################################################
        // Rising edge...
        //###########################################################################
        adc_wave.rising_edge_block        = true;
        adc_wave.falling_edge_block       = false;
        adc_wave.rising_edge_normalized   = false;
        adc_wave.edge_to_edge_block_count = round( 2500 / pconf.pwm_period_us ); // 2500us until forced recovery (this is not exact. i2s timing differs etc )

        //###########################################################################
        // Partly override the multisample buffer with this sample
        //###########################################################################
        int samples = MIN( adc_params.rising_avg, adc_params.default_avg );
        for( int i = 0; i < samples; ++i ){
            read_multisample( adc_data.recent, true, 1, false );
        }
        if( adc_wave.adc_pre_rising_edge == -1 ){

            //###########################################################################
            // Lock the previous adc value. After a spark the feedback should normalize 
            // back to the pre rising edge state. "Should"... "Somehow"... "Ideally"...
            //###########################################################################
            adc_wave.adc_pre_rising_edge  = adc_data.sampled; 
            adc_wave.last_rising_edge_adc = adc_data.recent;
            adc_wave.edge_recovery_failed = false;

        } else {
            //###############################
            // existing rising edges
            //###############################
            edge.multiple_rising_edges = true;
      
        }
        
    } else if( edge.is_falling_edge) {

        adc_wave.falling_edge_block_start_time = esp_timer_get_time();
        adc_wave.falling_edge_block            = true;
        adc_wave.rising_edge_block             = false;
        adc_data.edge_type                     = -1;

    }
    if( adc_wave.adc_pre_rising_edge != -1 ){
        //###########################################################################
        // Unsolved rising edge waiting for normalisation
        //###########################################################################
        if( 
            --adc_wave.edge_to_edge_block_count <= 0 ||
            adc_data.recent <= adc_wave.adc_pre_rising_edge 
        ){
            // adc back to the pre rising level
            edge.falling_edge_accepted = true;
            adc_wave.multiple_rising_edges_count=0;
        }
    }
    if( edge.falling_edge_accepted ){
        
        if( adc_wave.edge_to_edge_block_count <= 0 ){
            adc_wave.edge_recovery_failed = true;
        } else {
            adc_wave.edge_recovery_failed = false;
        }
        adc_wave.adc_pre_rising_edge       = -1;
        adc_wave.rising_edge_normalized    = true;
        adc_wave.edge_to_edge_block_count  = 0;
    }
    if(  edge.is_rising_edge || !adc_wave.rising_edge_normalized ){
        adc_data.edge_type = 1;
    } 
    if( edge.multiple_rising_edges ){
        adc_data.edge_type = adc_wave.multiple_rising_edges_count>4?3:2;
    }
    return true;
}


//######################################################
// Sets and resets the variables used for hold/go flags
//######################################################
IRAM_ATTR void set_signals( volatile adc_readings &adc_data ){
    if( adc_data.recent <= adc_critical_conf.zero_treshhold ){
        if( ++adc_data.zeros_in_a_row > adc_critical_conf.zeros_jitter ){
            adc_data.highs_in_a_row = 0;
        }
    } else {
        if( adc_data.recent >= setpoints.upper ){ 
            ++adc_data.highs_in_a_row;
        } else {
            adc_data.highs_in_a_row = 0;
        }
        adc_data.zeros_in_a_row = 0;
    }
}

//######################################################
// Adjust the plan if needed
//######################################################
IRAM_ATTR void adjust_plan_to_signals( volatile adc_readings &adc_data, int &plan, bool &create_negative_plan ){
    if( use_stop_and_go_flags ){
        if( plan == 1 && adc_data.zeros_in_a_row <= adc_critical_conf.zeros_in_a_row_max ){
            plan = 2;
            create_negative_plan = true;
        } else if( adc_data.highs_in_a_row >= adc_critical_conf.highs_in_a_row_max ){
            if( plan == 1 ){ create_negative_plan = true; }
            plan = MAX( plan, 3 );
        }
    }
}

IRAM_ATTR void do_retract_soft( volatile adc_readings &adc_data, int &plan, int &full_range_plan, int &realtime_plan ){
    if( 
        adc_data.avg_full_range > setpoints.upper 
        || sfeedback.reset_count>0
    ){ plan = MAX( plan, 4 ); }
}
IRAM_ATTR void do_retract_hard( int &plan, int &full_range_plan, int &realtime_plan ){
    if( 
        sfeedback.pin_is_low 
        //|| (adc_data.previous_plan==5&&(plan==4))
    ){ 
        plan                          = 5; 
        sfeedback.had_short_condition = 3;
        counter[0]                    = 0;

    }
}


IRAM_ATTR void do_forward_motion( 
      volatile adc_readings &adc_data
    , int      &plan
    , int      &full_range_plan
    , int      &realtime_plan 
    , bool     &create_negative_plan
){

    if( plan > 1 ){ return; }

    if( 
           full_range_plan == 1 
        && plan            == 1
        && realtime_plan   == 1
        && adc_data.avg_full_range <= adc_data.avg_full_range_last // only on a falling full range average
        && sfeedback.count <= 0 // only if there was no short going on
        && ( esp_timer_get_time() - sfeedback.last_short_condition >= adc_critical_conf.forward_block_after_short_micros ) // last short given microseconds in the past
        && adc_data.voltage_channel_shorts <= adc_critical_conf.forward_block_channel_shorts_num
    ){
        // green light for forward if nothing else happens

    } else {
        // failed to confirm a forward motion
        // increase the plan to hold and flag it negative
        ++plan;
        create_negative_plan = true;
    }

}

IRAM_ATTR void adjust_plan_to_edge( volatile adc_readings &adc_data, int &plan, bool &create_negative_plan ){
    
    if( 
        adc_data.edge_type >= 2 
        || adc_wave.falling_edge_block 
        //|| adc_wave.rising_edge_block
    ){ // multiple rising edges
        if( plan == 1 ){ create_negative_plan = true; }
        plan = MAX( 2, plan );
    }
    
    /*if( adc_data.edge_type > 0 ){ // rising edge situation
        if( adc_data.edge_type == 1 ){ // rising edge from bottom
            if( plan == 1 ){ create_negative_plan = true; }
            plan = MAX( 2, plan );
        } else if( adc_data.edge_type >= 2 ){ // multiple rising edges
            if( plan == 1 ){ create_negative_plan = true; }
            plan = MAX( adc_data.edge_type==2?3:4, plan );
        }
    }*/
    /*if( adc_data.previous_edge > 1 && plan == 4 ){
        plan = 5;
    }*/
    adc_data.previous_edge = adc_data.edge_type;

}
IRAM_ATTR int G_EDM_SENSORS::get_motion_plan( volatile adc_readings &adc_data ){
    int      plan                 = 2;
    bool     reset_counter        = true;
    bool     create_negative_plan = false; // negative plan is used to keep track of no load steps in the planner if we override a forward plan
    uint16_t adc_value            = adc_data.sampled;

    //#############################################################
    // Probing motion plan is a little different
    //#############################################################
    if( is_probing() ){

        if( probe_touched ){ counter[5] = 0; return 4; }
        if (
            adc_data.voltage_channel_adc_recent <= probe_logic_low_trigger_treshold ||
            adc_value > setpoints.probing
        ){
            plan = 2; // pause motion
            if( ++counter[5] >= MOTION_PLAN_PROBE_CONFIRMATIONS ){
                plan = 4; // probe confirmed
                probe_touched = true;
            }
        } else {
            probe_touched = false;
            if( counter[5] > 0 || adc_data.edge_type > 0 ){
                plan = 2; // skip one round after a positive
            } else {
                plan = 1;
            }
            counter[5] = 0; // fully reset the counter
        }
        return plan;
    }


    //###########################################################################
    // Calculate the raw pure plan based on the feedback provided
    //###########################################################################
    plan                = get_raw_plan( adc_value );               // fast average; can be set to single reading too...
    int full_range_plan = get_raw_plan( adc_data.avg_full_range ); // slow average
    int realtime_plan   = get_raw_plan( adc_data.recent );         // most recent sample

    /*if( realtime_plan == 1 ){
        ++counter[1];
        if( adc_wave.rising_edge_block && counter[1] < 10 ){

        }
    }*/

    //###########################################################################
    // At this point every plan is either a 1 or a 2
    // Now check for 4 and 5
    //###########################################################################
    do_retract_soft( adc_data, plan, full_range_plan, realtime_plan );
    do_retract_hard( plan, full_range_plan, realtime_plan );

    //###########################################################################
    // Check if all plans indicate a forward motion
    //###########################################################################
    do_forward_motion( adc_data, plan, full_range_plan, realtime_plan, create_negative_plan );

    set_signals( adc_data );
    adjust_plan_to_edge( adc_data, plan, create_negative_plan );
    adjust_plan_to_signals( adc_data, plan, create_negative_plan );

    adc_data.previous_plan       = plan;
    adc_data.avg_full_range_last = adc_data.avg_full_range;

    if( sfeedback.had_short_condition > 0 ){
        if( plan < 5 ){
            ++counter[0];
        }
        if( plan < 5 && counter[0] <= 3 ){
            plan = 5;
        } 
        if( plan < 4 && counter[0] >= 3 ){
            sfeedback.had_short_condition = 0;
        }
    }


    if( plan >= 4 && ++counter[7] < adc_critical_conf.retract_confirmations ){
        --plan;
    } else if( plan < 4 ){
        counter[7] = 0;
        sfeedback.reset_count = 0;
    }



    if( gconf.gedm_retraction_motion ){

        return MAX( plan, full_range_plan );

    } else {

        /*if( plan >= 4 && ++counter[7] < adc_critical_conf.retract_confirmations ){
            --plan; // don't be overreactive with hard retractions. A tiny single short drop may not be worth it
        } else if( plan < 4 ){
            counter[7] = 0;
            sfeedback.reset_count = 0;
        }*/

    }


    if( create_negative_plan ){
        plan *= -1;
    }
    return plan;
}

IRAM_ATTR void read_voltage_feedback( volatile adc_readings &adc_data ){

    if( 
        //adc_data.voltage_channel_shorts > 10 ||
        adc_data.voltage_channel_adc_recent < adc_critical_conf.voltage_feedback_treshhold 
    ){
        // voltage dropped below threshold; shorted;
        sfeedback.pin_is_low = true;
        sfeedback.reset_count = 3;
        if( ++sfeedback.count > 100000 ){ 
            sfeedback.count = 100000; 
        };
    } else {
        if( sfeedback.reset_count > 0 ){
            --sfeedback.reset_count;
        }
        if( sfeedback.reset_count == 0 ){
            sfeedback.pin_is_low = false;
        }
    }
    //gpio_35_on_interrupt();

}

IRAM_ATTR void G_EDM_SENSORS::pwm_off_protection( bool enforce = false ){
  
    if( 
        edm_process_is_running 
        && ( 
            enforce 
            || ( 
                 ( 
                    sfeedback.pin_is_low // voltage feedback ADC i2s batch was below threshold 
                    //|| sfeedback.had_short_condition > 0
                    || adc_data.voltage_channel_shorts > adc_critical_conf.max_channel_shorts_for_pulse_off // given number of samples in the i2s batch where below threshold
                 ) 
                 && adc_critical_conf.pulse_off_duration>0 
                 && sfeedback.count > adc_critical_conf.pwm_off_count 
                )
            )
    ){ 
        block_short_low_override = true;
        ui_controller.set_pwm_state( true );
        delayMicroseconds( adc_critical_conf.pulse_off_duration*pconf.pwm_period_us );
        ui_controller.reset_pwm_state();
        sfeedback.last_short_condition = esp_timer_get_time();
    }

    if( !sfeedback.pin_is_low ) { 
        sfeedback.count = 0;
    }

}

























//######################################################################
// 1 = no load; move forward
// 2 = above forward treshhold
// 3 = 2 and 3 are the same in regards of the motion
// the upper setpoint is used for the slow average and should always be 
// below the pulse peak. A 3 helps to have a difference between a 
// 2 and a 3. A 3 is basically just a clear load condition while a 2
// may not be so clear all the time
//######################################################################
IRAM_ATTR int G_EDM_SENSORS::get_raw_plan( int adc_value ){
    int plan = 2;
    if( adc_value < setpoints.lower ){
        plan = 1; // below setpoint
    } /*else if( adc_value > setpoints.upper ){
        plan = 2; // setpoints upper is only used for the slow average
    }*/
    return plan;
}



//###########################################################################
// Called from planner on core 1
// Get the motionplan in use
//###########################################################################
int IRAM_ATTR G_EDM_SENSORS::get_calculated_motion_plan(){
    int plan = 0;
    if( plconfig.reset_sense_queue ){
        xQueueReset( motion_plan_queue );
    }
    if( ! xQueueReceive( motion_plan_queue, &plan, 15000 ) ){
        return 9;
    }
    motion_plan = plan;
    return plan;
}


