// Remove all serial.prints


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
# Working base since 7.Sep.2024      
*/   
//#include <Arduino.h>

//################################################
// Main includes
//################################################
#include <driver/adc.h>
#include "gedm_main.h"
#include "config/definitions.h"
#include "sd_card/filehandler.h"

//################################################
// GRBL_ESP32
//################################################
#include "Grbl.h"


String active_profile = "None";

TaskHandle_t ui_task_handle  = NULL;
TaskHandle_t protocol_task   = NULL;

void start_services(){
  ui_controller.run_sensor_service();
  ui_controller.start_ui_task();
}

void update_ui_controller(){

}


void setup() {

    if( ENABLE_SERIAL ){
      Serial.begin(115200);
      while(!Serial);
      vTaskDelay(10);
    }

    disableCore0WDT();
    disableCore1WDT();
    grbl.grbl_init();
    ui_controller.configure();
    ui_controller.set_filehandler( &filehandler );
    update_ui_controller();
    start_services();
    //grbl.grbl_init();
    ui_controller.set_is_ready();
    grbl.reset_variables();
    
    xTaskCreatePinnedToCore( 
      protocol_main_loop, "protocol_task", 15000, &gproto, 
      TASK_PROTOCOL_PRIORITY, &protocol_task, 1 );

      i2sconf.I2S_FORCE_RESTART = true;

}


void loop(){
  vTaskDelete(NULL);
}

