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

/*
# Press the left DPM button for a few seconds to enter the menu
# then navigate through it and set it to this values:
#
# 5-CS = 0
# 6-bd = 115.2 // maximum baudrate the DPM supports; baudrate for ESP and DPM needs to be the same
# 4-Fd = 1
# 3-ON = 0 // output enabled at boot = 1; with 0 the output is disabled at start until it is manually turned on
# 7-ad 1
# 8 ch 1
*/


#include "config/definitions.h"

#include <HardwareSerial.h>
#include <esp_timer.h>

const int BUFFER_SIZE = 30;
const int CURRENT_ERROR_TRESHHOLD = 6000;  // the DPM8605 delivers up to 5A/5000mA. But it may happen that some peak currents exceed those values randomly. 
const int VOLTAGE_ERROR_TRESHHOLD = 65000; // mV even if the DPM is 60v/60000mV max again allow the result to be a little over the top

extern DMA_ATTR int dpm_locked;
extern DMA_ATTR int settings_milliamps;
extern DMA_ATTR int settings_millivolts;
extern DMA_ATTR int measured_ma;
extern DMA_ATTR int measured_mv;


class G_EDM_DPM_DRIVER {

    private:
        bool is_ready;
        char* error;
        HardwareSerial *dpm_serial;
        float processString(String str);
        int64_t last_amp_reading_timestamp;
        bool IRAM_ATTR send( char* cmd, char* response, bool blocking = true );
        bool IRAM_ATTR fetch( char* response, bool blocking = true );


    public:
        G_EDM_DPM_DRIVER();
        void end( void );
        bool get_error( void );
        bool setup( HardwareSerial &serial );
        bool init( void );
        bool power_on_off( int turn_on, int max_retry = 5 );
        int  IRAM_ATTR extract( int &value,  int multiplier, char* response );
        bool IRAM_ATTR extract_ok( char* response );
        int  set_voltage_and_current(float v, float c);
        bool set_setting_voltage(float value);
        bool set_setting_current(float value);

    float read(char cmd);
    bool IRAM_ATTR get_measured_voltage_mv( int &voltage );
    bool IRAM_ATTR get_measured_current_ma( int &current );
    bool IRAM_ATTR request_measured_current_ma( void );
    bool IRAM_ATTR fetch_measured_current_ma( int &current );
    int  IRAM_ATTR extract_reverse( int &value, int multiplier, char* response );
    bool IRAM_ATTR measure_ma_is_outdated( int max_age );

    bool get_setting_voltage_and_current( int &voltage, int &current );
    bool get_setting_voltage( int &voltage );
    bool get_setting_current( int &current );
    bool get_power_is_on( void );//:01r12=0,
};

extern G_EDM_DPM_DRIVER dpm_driver;