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
/*#include "gedm_adc_driver.h"

G_EDM_ADC AD4000_adc;

G_EDM_ADC::G_EDM_ADC(){
    is_ready = false;

}

bool G_EDM_ADC::setup(){
    return true;
}

void G_EDM_ADC::set_spi_instance( SPIClass &_spi ){
  adc_spi = _spi;
}
bool IRAM_ATTR G_EDM_ADC::begin( int cs_pin, uint32_t frequency ){
    chipselect_pin    = cs_pin;
    clk_frequency     = frequency;
    pinMode( chipselect_pin, OUTPUT );
    adc_deselect();
    adc_busy          = false;
    reference_voltage = EXTERNAL_SPI_ADC_REF_VOLTAGE;
    adc_resolution    = EXTERNAL_SPI_ADC_RESOLUTION;
    voltage_per_step  = reference_voltage / adc_resolution;
    //adc_spi.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, -1); 
    is_ready = true;
    return true;
}
void G_EDM_ADC::adc_deselect(){
    //GPIO_REG_WRITE( GPIO_OUT_W1TC_REG, 1<<EXTERNAL_SPI_ADC_CS_PIN );//low
    GPIO_REG_WRITE( GPIO_OUT_W1TS_REG, 1<<EXTERNAL_SPI_ADC_CS_PIN ); //high
    //delayMicroseconds(1);
}
bool G_EDM_ADC::adc_select( ){
    //GPIO_REG_WRITE( GPIO_OUT_W1TS_REG, 1<<EXTERNAL_SPI_ADC_CS_PIN ); //high
    GPIO_REG_WRITE( GPIO_OUT_W1TC_REG, 1<<EXTERNAL_SPI_ADC_CS_PIN );//low
    //delayMicroseconds(1);
    return true;
}
int IRAM_ATTR G_EDM_ADC::read(){
    adc_spi.beginTransaction(SPISettings(clk_frequency, SPI_MSBFIRST, SPI_MODE0));
    adc_select();
    unsigned int adc_value = adc_spi.transfer16(0x0000);
    adc_deselect();
    adc_spi.endTransaction();
    return adc_value;
}

//32250

void IRAM_ATTR G_EDM_ADC::lock(){
    adc_spi.beginTransaction(SPISettings(clk_frequency, SPI_MSBFIRST, SPI_MODE0));
}
void IRAM_ATTR G_EDM_ADC::unlock(){
    adc_spi.endTransaction();
}
int IRAM_ATTR G_EDM_ADC::read_multiple(){
    adc_select();
    unsigned int adc_value = adc_spi.transfer16(0x0000);
    adc_deselect();
    return adc_value;
}
*/