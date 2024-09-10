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
/*#include "config/definitions.h"
#include <SPI.h>
#include <esp32-hal-gpio.h>


class G_EDM_ADC {
    private:
        bool is_ready;
        SPIClass adc_spi;
        int chipselect_pin;
        uint32_t clk_frequency;

        int   adc_busy;
        float reference_voltage;
        float adc_resolution;
        float voltage_per_step;
        void adc_deselect( void );
        bool adc_select( void );

    public:
        G_EDM_ADC();
        void set_spi_instance( SPIClass &_spi );
        bool setup( void );
        bool IRAM_ATTR begin( int cs_pin, uint32_t frequency=4000000 );
        int  IRAM_ATTR read( void );
        int  IRAM_ATTR read_multiple( void );
        void  IRAM_ATTR lock( void );
        void  IRAM_ATTR unlock( void );

};

extern G_EDM_ADC AD4000_adc;
*/