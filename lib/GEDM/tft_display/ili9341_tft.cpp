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
#
# Thanks to Shrawan Khatri for pointing out the issue with the duty cycle
# After changing the PWM generation to MCPWM I missed to remove the 
# resolution calculations and the PWM was way off      
*/ 

#include "ili9341_tft.h"
#include "Motors/Motors.h"
//#include <nvs.h>
#include "Protocol.h"

#include "gedm_dpm_driver/gedm_dpm_driver.h"

/*#ifdef USE_DPM_RXTX_SERIAL_COMMUNICATION
    #include "gedm_dpm_driver/gedm_dpm_driver.h"
#endif*/

#include "gpo_scope.h"

portMUX_TYPE sprite_mutex = portMUX_INITIALIZER_UNLOCKED;

DRAM_ATTR volatile bool scope_frame_ready = false;

void G_EDM_UI_CONTROLLER::start_ui_task()
{
  xTaskCreatePinnedToCore(ui_task, "ui_task", UI_TASK_STACK_SIZE, this, 
      TASK_UI_PRIORITY, &ui_task_handle, !xPortGetCoreID());
}



/**
 * 
 * GEDM specific GCodes
 *
 * Warning: This is Frankenstein G-RBL EDM. It doesn't work with gcode senders and the GCode does need to only contain the raw XY path and tool up/down for path changes.
 *
 * In drill/sinker mode no gcode is used and no homing of xy is done to allow easy positioning and just drill/sink down.
 *
 * In wire Mode the X and Y axis retract if needed. ( work in progress. Don't use it for now. )
 *
 * In floating Z mode a rod is used as electrode that floats on the surface of the workpiece and constantly tries
 * to progress down while the gcode file is looping. So any probing or homing commands in the file would repeat after every round if not removed.
 *
 * M7-9 toogle the different modes on and off. If the machine was set to wire edm and is then switched
 * to a different mode it is not needed to turn the previous mode off. A new mode will disable the others.
 * 
 * M7 = toggle EDM drill/sinker mode on/off
 * M8 = toggle EDM floating z axis on/off
 * M9 = toggle Wire EDM with XY retractions on/off
 * 
 * M3-4 commands are ignored in wire mode. In wire mode every command in the gcode file is interpreted as a work motion
 * A G0 command produces the same result as a G1 command. Z motion is fully disabled even if there is any in the gcode!
 * 
 * M3 = Move z up for toolpath change, it disables the PWM to prevent unwanted eroding while moving up
 * M4 = move z back down after a toolpath change, re-enables PWM and moves down until contact is made.
 *
 * While a motion is done it blocks the gcode parser from parsing new commands.
 * 
 **/
TaskHandle_t drill_task_handle = NULL;
DRAM_ATTR int64_t last_minimal_ui_update_micros = esp_timer_get_time();

G_EDM_UI_CONTROLLER ui_controller;

G_EDM_UI_CONTROLLER::G_EDM_UI_CONTROLLER() {
  change_sinker_axis_direction(0);
}



/** runs once after the ui is ready **/
void G_EDM_UI_CONTROLLER::run_once_if_ready(){
  /** enable machine on startup **/
  //motor_manager.calculate_speeds();
  planner.init();
  sys_axis_homed[X_AXIS] = false;
  sys_axis_homed[Y_AXIS] = false;
  sys_axis_homed[Z_AXIS] = false;
  // set initial position
  // this is dangerous but allows to use the machine without homing
  // the position is set to the center of each axis without pulloff distance etc.
  // each axis will be able to move 50% of the travel in either direction without homing
  float xffpos = float(DEFAULT_X_MAX_TRAVEL/2)*-1;
  float yffpos = float(DEFAULT_Y_MAX_TRAVEL/2)*-1;
  float zffpos = float(DEFAULT_Z_MAX_TRAVEL/2)*-1;
  int xfakepos = round(xffpos*axis_settings[ X_AXIS ]->steps_per_mm->get());
  int yfakepos = round(yffpos*axis_settings[ Y_AXIS ]->steps_per_mm->get());
  int zfakepos = round(zffpos*axis_settings[ Z_AXIS ]->steps_per_mm->get());
  sys_position[X_AXIS] = xfakepos;
  sys_position[Y_AXIS] = yfakepos;
  sys_position[Z_AXIS] = zfakepos;
  sys_axis_homed[X_AXIS] = true;
  sys_axis_homed[Y_AXIS] = true;
  sys_axis_homed[Z_AXIS] = true;
  gcode_core.gc_sync_position();
  api::unlock_machine();
  api::reset_probe_points(); 
  api::push_cmd("G91 G54 G21\r\n");
  reset_defaults();
  api::push_cmd("G91 G10 L20 P1 X0Y0Z0\r\n"); // reset work coordinates
    /*char command_buf[60];
    sprintf(command_buf,"G91 G10 L20 P1 X%.5fY%.5fZ%.5f\r\n", xffpos*-1, yffpos*-1, zffpos*-1 );
    api::push_cmd(command_buf);*/
  setup_spindle(GEDM_A_DIRECTION_PIN, GEDM_A_STEP_PIN);
  planner.position_history_reset();
  planner.update_max_feed();
  generate_setpoint_min_max_adc();

}

void G_EDM_UI_CONTROLLER::update_speeds(){
  int slowest_axis                 = motor_manager.get_slowest_axis();
  process_speeds.RAPID             = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.RAPID );  
  process_speeds.EDM               = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.EDM );
  process_speeds.DEFAULT_DELAY     = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.DEFAULT_DELAY );
  process_speeds.PROBING           = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.PROBING );
  process_speeds.REPROBE           = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.REPROBE );
  process_speeds.RETRACT           = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.RETRACT );
  process_speeds.HOMING_FEED       = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.HOMING_FEED );
  process_speeds.HOMING_SEEK       = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.HOMING_SEEK );
  process_speeds.MICROFEED         = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.MICROFEED );
  process_speeds.WIRE_RETRACT_HARD = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.WIRE_RETRACT_HARD );
  process_speeds.WIRE_RETRACT_SOFT = motor_manager.get_motor( slowest_axis )->get_step_delay_for_speed( process_speeds_mm_min.WIRE_RETRACT_SOFT );
}


IRAM_ATTR void G_EDM_UI_CONTROLLER::fill_screen( int color ){
  //color = TFT_BACKGROUND_COLOR;
  // drawing a full screen is a heavy task and the ISR doesn't like it
  // having an ISR running at highspeed parallel to a heavy display ui needs some hacks
  int divider        = 10;
  int display_width  = 320;
  int display_height = 240;
  int pos_y          = 0;
  int height_fragment = display_height / divider;
  for( int i = 0; i < divider; ++i ){
    tft.fillRect( 0, pos_y, display_width, height_fragment, color );
    pos_y += height_fragment;
    vTaskDelay(1);
  }
}
void G_EDM_UI_CONTROLLER::configure()
{
  //TFT_eSPI tft = TFT_eSPI();
  set_motion_tab_active(1);
  spark_on_off_indicator_icon_last_state = -1;
  spark_indicator_bar_width = 0;
  is_ready = false;
  keyboard_is_active = false;
  button_width = 45;
  button_height = 45;
  button_margin = 5;
  keyboard_value = "";
  keyboard_initial_value = 0.0;
  active_page = 0;
  has_sd_card = false;
  gcode_file = "";
}
void G_EDM_UI_CONTROLLER::loader_screen()
{
  fill_screen(TFT_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("GEDM", 20, 20, 2);vTaskDelay(1);
  tft.setTextSize(2);
  tft.setTextColor(TFT_PURPLE);
  tft.drawString("EVO II", 20, 70, 2);vTaskDelay(1);
  tft.setTextColor(TFT_DARKCYAN);
  tft.setTextSize(1);
  tft.drawString("Powered by TFT_eSPI and G-RBL", 20, 120, 2);vTaskDelay(1);
  tft.drawString("Credits:", 20, 160, 2);vTaskDelay(1);
  tft.drawString("8kate, Teelicht, VB", 20, 180, 2);vTaskDelay(1);
  tft.drawString("Ethan Hall, Alex Treseder, Shrawan", 20, 200, 2);vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::init(bool disable_calibration)
{
  disable_tft_calibration = disable_calibration;
  start_time = esp_timer_get_time();
  last_settings_copy = "";
  disable_spark_generator();
  tft.init();vTaskDelay(1);
  tft.setRotation(3);vTaskDelay(1);
  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setCursor(0, 0, 4);vTaskDelay(1);
  touch_calibrate( false );vTaskDelay(1);
  loader_screen();vTaskDelay(1);
  tft.setCursor(0, 0, 4);
}


/**
 * Starting the UI on core 0
 **/
void IRAM_ATTR ui_task(void *parameter){
  G_EDM_UI_CONTROLLER *ui_controller = (G_EDM_UI_CONTROLLER *)parameter;

  ui_controller->init(DISABLE_TFT_CALIBRATION);
  ui_controller->filehandler->set_spi_instance( tft.getSPIinstance() );
  ui_controller->filehandler->filehandler_initialize(true);

  /** wait until ui is ready **/
  while (!ui_controller->get_is_ready()){
    vTaskDelay(100);
  }
  ui_controller->update_speeds();
  ui_controller->set_pwm_pin( GEDM_PWM_PIN );
  ui_controller->setup_pwm_channel();
  ui_controller->change_pwm_frequency( ( int ) pwm_frequency );
  ui_controller->update_duty( pwm_duty );
  ui_controller->disable_spark_generator();

  /** restore the last session **/
  ui_controller->filehandler->create_directory_tree(3);
  ui_controller->load_settings("last_settings");

  dpm_driver.setup( Serial );
  dpm_driver.init();

  if( enable_dpm_rxtx ){
      vTaskDelay(100);
      dpm_driver.power_on_off( false, 3 );
  }



  /** draw frontpage **/
  ui_controller->run_once_if_ready();
  G_EDM_SENSORS::edm_start_stop();
  ui_controller->render_page(1, true);
  ui_controller->last_settings_write = esp_timer_get_time();
  force_redraw = false;
  int64_t time_since = 0;
  ui_controller->init_adc();
  canvas.deleteSprite();

  int data = 0;
  for( int i = 0; i < 3; ++i ){
    ++data;
    xQueueSendFromISR( sensor_queue_main, &data, NULL );
  }

  Serial.println("G-EDM ready for you!");

  i2sconf.I2S_FORCE_RESTART = true;

  //ui_controller->enable_spark_generator();

  for (;;)
  {

    if (ui_controller->is_probing()){ // the normal process will never come here ince probing is a blocking function inside this task. But let it be.
      vTaskDelay(1000);
      continue;
    }
    if (sys.state == State::Alarm) {
      ui_controller->alarm_handler();
    }
    if (!edm_process_is_running)
    {
      if (force_redraw)
      {
        if( ui_controller->get_active_page() != 1 ){
          ui_controller->render_page( ui_controller->get_active_page(), true );
          force_redraw = false;
        } else {
          ui_controller->render_page(1, true);
          force_redraw = false;
        }
      }
      if (ui_controller->ready_to_run())
      {
        /**
         * it gets here after an SD job is finished too and needs a
         * check if this is just the end of an SD job
         **/
        if (ui_controller->filehandler->has_job_finished())
        {
          /** ensure that the machine is finished **/
          if (api::machine_is_idle())
          {
            ui_controller->reset_after_job();
          }
          continue; // mashine still working
        }
        /** everything ready to run **/
        ui_controller->start_process();
      }
      ui_controller->monitor_touch_input();
      if (ui_controller->get_active_page() == 1)
      {
        ui_controller->draw_mpos();
        ui_controller->redraw_vsense();
        ui_controller->draw_spark_on_off_indicator_icon();
        ui_controller->draw_motion_navigation_buttons();
        ui_controller->sd_card_handler();
        vTaskDelay(1);
        gscope.draw_scope();
      }
      else
      {
        ui_controller->last_settings_write = esp_timer_get_time();
      }

      #ifdef DEBUG_PROCESS_SCREEN
          edm_process_is_running = true;
          operation_mode = 4;
          ui_controller->render_page(9,true);
      #endif

    } else {

          if ( // break condition
            #ifdef DEBUG_PROCESS_SCREEN
                false
            #else
                !ui_controller->start_edm_process() 
                || sys_rt_exec_alarm != ExecAlarm::None 
                || gconf.edm_process_finished 
                || api::machine_is_idle()
            #endif
          ){

              vTaskDelay(100);
              ui_controller->reset_after_job();
              if( gconf.edm_process_finished && operation_mode==1 ){
                  api::push_cmd("G90 G54 G21 Z0\r\n");
              } else{ api::push_cmd("G90 G54 G21\r\n"); }
              vTaskDelay(100);
              api::block_while_not_idle();
              gconf.edm_process_finished = false;

          } else {
              time_since = esp_timer_get_time() - last_minimal_ui_update_micros;

              if( gconf.edm_pause_motion || time_since > 280000 ){ //200ms = 100*1000 = 200000uS
                  if( time_since > 50000 ){
                       ui_controller->process_overlay_coords();vTaskDelay(1); // wpos, mpos
                       if( gconf.edm_pause_motion ){
                           ui_controller->process_overlay_pause_button(); // hackaround to update the button
                       }
                       last_minimal_ui_update_micros = esp_timer_get_time();
                  }
                  ui_controller->ui_in_process_update(); // touch
              }
              gscope.draw_scope();

          }

    }

    vTaskDelay(1);continue;
    vTaskDelay( ( edm_process_is_running 
                  ? ( planner.get_is_paused() 
                      ? INTERFACE_INTERVAL 
                      : INTERFACE_INTERVAL_WHILE_EDM 
                    ) 
                  : INTERFACE_INTERVAL ) );

  }
  vTaskDelete(NULL);
}


bool G_EDM_UI_CONTROLLER::ui_in_process_update(){

  if(
    #ifdef DEBUG_PROCESS_SCREEN
      true
    #else
      start_edm_process() 
      //&& ( is_between_steps || gconf.edm_pause_motion || gconf.gedm_flushing_motion || force_redraw )
    #endif
  ){

        force_redraw = false;

        if (gconf.gedm_reprobe_block){
            process_overlay_reprobe_confirm();
        } else {
            if (force_redraw){
                render_page(9, true);
                force_redraw = false;
            } else {
                update_display_minimal();
            }
        }

    return true;

  }

  return false;
}

void G_EDM_UI_CONTROLLER::sd_card_handler(){
    if (!filehandler->is_locked_for_ui())
    {
      filehandler->sd_card_refresh();vTaskDelay(1);
      render_page_settings_menu_sd_card(filehandler->get_has_sd_card(), false);vTaskDelay(1);

      if( esp_timer_get_time() - last_settings_write > 4000000 )
      {
        // update last settings every x seconds if on the frontpage
        // if nothing has changed it will skip
        save_settings("last_settings");vTaskDelay(1);
        last_settings_write = esp_timer_get_time();
      }
    }
}

void G_EDM_UI_CONTROLLER::alarm_handler(){
  edm_process_is_running = false;
  vTaskDelay(1);
  if (enable_spindle)
  {
    stop_spindle();
  }
  disable_spark_generator();
  vTaskDelay(200);
  fill_screen(TFT_BLACK);
  vTaskDelay(1);
  tft.setTextSize(2);
  tft.setTextColor(TFT_RED);
  tft.drawString("Alarm!", 10, 50, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  String error = "";
  int error_code = static_cast<int>( sys_last_alarm );

  String alarm_msg = "Something failed";

  switch (error_code){
      case 1:
          alarm_msg = "Hard limit was triggered";
      break;
      case 2:
          alarm_msg = "Target out of reach";
      break; 
      case 3:
          alarm_msg = "Reset while in motion";
      break;
      case 4:          
      case 5:
          alarm_msg = "Probe fail";
      break;  
      case 6:
      case 7: 
      case 8:
      case 9:
          alarm_msg = "Homing fail";
      break;  
  }
  tft.drawString("Alarm Code: " + String( error_code ), 10, 100, 2);vTaskDelay(1);
  tft.drawString("Alarm MSG: "  + alarm_msg, 10, 120, 2);vTaskDelay(1);
  tft.drawString("Touch to reset", 10, 140, 2);vTaskDelay(1);
  sys_rt_exec_alarm = ExecAlarm::None;
  uint16_t x, y;
  sys.abort = true;
  gproto.protocol_exec_rt_system();
  while (!get_touch(&x, &y)){ vTaskDelay(100); }
  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(2);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("Resetting.....", 10, 50, 2);vTaskDelay(1);
  reset_after_job();
  gproto.protocol_exec_rt_system();
  vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::wire_gcode_task(){
  /** basic settings for the process **/
  motor_manager.set_ignore_disable(true);
  planner.set_cutting_depth_limit( 0.0 ); // normally not needed but reset the z limit anyway
  api::push_cmd("G54\r");
  planner.configure();
  api::push_cmd("M9\r"); 
  vTaskDelay(1000);
  // set wire speed
  set_speed( wire_spindle_speed );
  if ( enable_spindle && ! simulate_gcode )
  {
    // spindle should alway run in wire edm to pull the wire
    // but for testing it may be useful to allow the spindle to be off
    start_spindle();
  }
  pwm_on();
  render_page(9, true);
  SD_ready_next = true; // start
}

/**
 * Drill/Sinker mode
 **/
void G_EDM_UI_CONTROLLER::sd_card_task()
{
  /** basic settings for the process **/
  motor_manager.set_ignore_disable(true);
  /** set the cutting depth in steps after probing **/
  float cutting_depth = get_z_stop();
  planner.set_cutting_depth_limit(cutting_depth > 0.0 ? cutting_depth : 0.0);
  /** probe the workpiece **/
  api::push_cmd("G54\r");
  planner.configure();
  api::push_cmd("M8\r"); // send M8 to toggle gcode mode this needs to be done after the probing stuff and before the first gcode line is loaded from file
  vTaskDelay(1000);
  if (enable_spindle)
  {
    set_speed(wire_spindle_speed);
    start_spindle();
  }
  /** pwm is disabled after the probe. Reenable it **/
  pwm_on();
  reset_flush_retract_timer();
  render_page(9, true);
  SD_ready_next = true; // start
}

/**
 * Reamer mode
 * This mode does not use any XY motion and does not home X or Y
 * In reamer mode the Z axis just constantly moves up and down with spindle on or off
 * It can use a timer to stop the operation after a given amount of time and the distance of the up/down movement can be set too.
 * The movement starts with a down movement from the current Z axis position. No homing is used.
 * This mode is used to even out and enlarge deepholes
 * Reamer mode doesn't care about short circuits!
 **/
void IRAM_ATTR reamer_task_single_axis(void *parameter)
{

  G_EDM_UI_CONTROLLER *ui_controller = (G_EDM_UI_CONTROLLER *)parameter;

  if (enable_spindle)
  {
    ui_controller->set_speed(wire_spindle_speed);
    ui_controller->start_spindle();
  }

  vTaskDelay(500);

  ui_controller->pwm_on();

  unsigned long start_time = millis();
  bool is_moving_down = false;
  char command_buf_down[40];
  char command_buf_up[40];

  sprintf(command_buf_down, "$J=G91 G21 Z%.5f F60\r\n", reamer_travel_mm * -1);
  sprintf(command_buf_up, "$J=G91 G21 Z%.5f F60\r\n", reamer_travel_mm);

  for (;;)
  {

    vTaskDelay(10);
        if( ui_controller->get_pwm_is_off() ){
            ui_controller->pwm_on();
            delayMicroseconds(10);
        }
    // ui_controller->update_display_minimal();

    if (
        sys.abort 
        || !ui_controller->start_edm_process() 
        || (reamer_duration > 0.0 && float( ( millis() - start_time) / 1000.0 ) >= reamer_duration )
    ){ break; }

    if (gconf.gedm_planner_line_running)
    {

      /** axis is moving. Skipping this round **/
      continue;
    }
    else
    {

      if (is_moving_down)
      {

        api::push_cmd(command_buf_up);
        is_moving_down = false;
        vTaskDelay(500);
      }
      else
      {

        api::push_cmd(command_buf_down);
        is_moving_down = true;
        vTaskDelay(500);
      }
    }

  }

  ui_controller->pwm_off();
  gconf.edm_process_finished = true;

  /** todo: move back up after job is done **/
  // ui_controller->reset_after_job();
  vTaskDelete(NULL);
}

void G_EDM_UI_CONTROLLER::pre_process_defaults()
{

  if( enable_dpm_rxtx ){
      if( !dpm_driver.power_on_off( 1, 3 ) ){
        // problem... unhandled yet
      }// turn on
      vTaskDelay(200);
  }

  gconf.edm_process_finished = false;
  planner.push_break_to_position_history();
  api::unlock_machine();
  //edm_process_is_running = true;
  motion_plan = 0;
  render_page(9, true);
  edm_process_is_running = true;
  generate_reference_voltage();
}

void G_EDM_UI_CONTROLLER::start_process(void)
{

  if (operation_mode == 1)
  {
    sinker_drill_single_axis();
  }
  else if (operation_mode == 2)
  {
    pre_process_defaults();
    xTaskCreatePinnedToCore(reamer_task_single_axis, "reamer_task", 2500, this, 0, &drill_task_handle, 0);
  }
  else if (operation_mode == 3)
  {

    /** gcode from file **/
    // gcode mode requires the machine to be fully homed
    // if no probe position is set we just set the current position as G54 0,0 position
    if (!api::machine_is_fully_homed())
    {
      add_dialog_screen("Machine needs to be fully homed!");
      return;
    }

    if (!api::machine_is_fully_probed())
    {
      // if the machine is not probed
      // we just set the current position of the unprobed axes
      // to the probe position#
      api::auto_set_probepositions();
    }

    /** if a gcode file is set time to run it **/
    if (gcode_file.length() <= 0)
    {
      add_dialog_screen("Please select a gcode file");
      /** no file specified **/
    }
    else
    {
      open_gcode_file();
      sd_card_task();

      //xTaskCreatePinnedToCore(sd_card_task, "sd_task", 2500, this, 0, &drill_task_handle, 0);
    }
  } else if( operation_mode == 4 ){

    if (!api::machine_is_fully_homed())
    {
      add_dialog_screen("Machine needs to be fully homed!");
      return;
    }

    if( !sys_probed_axis[X_AXIS] || !sys_probed_axis[Y_AXIS] ){
      // if the machine is not probed
      // we just set the current position of the unprobed axes
      // to the probe position#
      api::auto_set_probepositions();
    }
    
    if (gcode_file.length() <= 0)
    {
      add_dialog_screen("Please select a gcode file");
      /** no file specified **/
    } else{
      open_gcode_file();
      wire_gcode_task();
    }

  }
}


bool G_EDM_UI_CONTROLLER::open_gcode_file()
{
    String path = String(ROOT_FOLDER) + String(GCODE_FOLDER) + "/" + gcode_file;
    //char file[255];
    //path.toCharArray(file, 200);
    while (filehandler->get_is_busy()){
      if (sys.abort || !start_edm_process())
      {
        break;
      }
      vTaskDelay(100);
    }
    pre_process_defaults();
    filehandler->close_file();
    filehandler->open_gcode_file(path);
    return true;
}

/**
 * this is only used for floating Z gcode tasks
 * the reprobe point is the point where z axis reprobes the work piece
 * in the running process
 **/
void G_EDM_UI_CONTROLLER::set_reprobe_point(){
  api::set_reprobe_points();
}

void G_EDM_UI_CONTROLLER::render_page_reprobe_confirmation()
{
  fill_screen(TFT_BLACK);
  vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Step 1", 10, 10, 2);vTaskDelay(1);
  tft.drawString("Step 2 ( optional )", 10, 100, 2);vTaskDelay(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("Loosen the drill chuck and let the", 10, 40, 2);vTaskDelay(1);
  tft.drawString("electrode slide on top of the work piece", 10, 60, 2);vTaskDelay(1);
  tft.drawString("Use arrows to move Z up or down", 10, 130, 2);vTaskDelay(1);
  tft.drawString("to regain some lost travel if needed", 10, 150, 2);vTaskDelay(1);
  tft.fillTriangle(290, 110, 280, 130, 300, 130, TFT_GREEN);vTaskDelay(1);
  tft.fillTriangle(280, 140, 290, 160, 300, 140, TFT_GREEN);vTaskDelay(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);vTaskDelay(1);
  tft.drawString("Confirm new Z0 G54 work position", 20, 200, 2);vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::process_overlay_reprobe_confirm()
{
  render_page_reprobe_confirmation();
  bool done = false;
  uint16_t x, y;
  bool block_touch = false;
  while (!done)
  {
    if (sys.abort || !gconf.gedm_reprobe_block || !start_edm_process())
    {
      break;
    }
    if (get_touch(&x, &y))
    {
      if (block_touch)
      {
        /** debounce **/
        continue;
      }
      block_touch = true;
      if (x > 280 && x < 300 && y > 110 && y < 130)
      {
        open_keyboard(0.0, "Move Z up (mm)", "Move Z axis up\r" + get_mpos_string());
        planner.z_axis_move_mm(get_keyboard_result());
        render_page_reprobe_confirmation();
      }
      else if (x > 280 && x < 300 && y > 140 && y < 160)
      {
        open_keyboard(0.0, "Move Z down (mm)", "Move Z axis down\r" + get_mpos_string());
        planner.z_axis_move_mm(get_keyboard_result() * -1);
        render_page_reprobe_confirmation();
      }
      else if (x > 10 && x < 310 && y > 190 && y < 230)
      {
        render_page(9, true);
        done = true;
        gconf.gedm_reprobe_block = false;
      }
    }
    else
    {
      block_touch = false;
    }
    vTaskDelay(1);
  }
}


void G_EDM_UI_CONTROLLER::edit_rapid_delay(){
    open_keyboard( (float)process_speeds.RAPID, "Step delay used for rapid moves (ms)", "Rapid motion speed\rHigher values produce slower motions");
    int delay = MAX(10,round( get_keyboard_result() ));
    process_speeds.RAPID = delay;
}
void G_EDM_UI_CONTROLLER::edit_max_feed_xy(){
    open_keyboard( (float)process_speeds.EDM, "Step delay in EDM process (ms)", "The step delay used in the EDM process\rHigher values produce slower motions");
    int delay = MAX(process_speeds.RAPID,round( get_keyboard_result() ));
    process_speeds.EDM = delay;
}

void G_EDM_UI_CONTROLLER::edit_max_feed( int axis ){    
    open_keyboard( max_feeds[axis], "Max Feedrate (mm/min)", "The maximum feedrate used for in the\rEDM process. Not exact!");
    float mm_min = get_keyboard_result();
    max_feeds[axis] = mm_min;
    planner.update_max_feed();
}

void G_EDM_UI_CONTROLLER::edit_pwm_frequency()
{
  open_keyboard(get_freq(), "PWM Frequency (hz)", "PWM Frequency in hz");
  change_pwm_frequency((int)get_keyboard_result() );
};
void G_EDM_UI_CONTROLLER::edit_pwm_duty()
{
  open_keyboard(get_duty_percent(), "PWM duty (%)", "PWM Duty cycle in percent");
  float new_duty = get_keyboard_result();
  if (new_duty > PWM_DUTY_MAX)
  {
    new_duty = PWM_DUTY_MAX;
  }
  update_duty(new_duty);
};
void G_EDM_UI_CONTROLLER::edit_setpoint_min()
{
  open_keyboard(vsense_drop_range_min, "Setpoint minimum (%)", "If feedback is below this value the \rtool moves on. (" + String( vsense_drop_range_min ) + "% = " + String( percentage_to_mv( vsense_drop_range_min ) ) + "mV)");
  float new_setpoint_min = get_keyboard_result();
  /** setpoint range min is toolow **/
  if (new_setpoint_min <= 0)
  {
    new_setpoint_min = 1;
  }
  /** if setpoint range min is higher then setpoint range max we use the setpoint range max minus 1 **/
  if (new_setpoint_min >= vsense_drop_range_max)
  {
    new_setpoint_min = vsense_drop_range_max - 0.1;
  }
  vsense_drop_range_min = new_setpoint_min;
  generate_setpoint_min_max_adc();
};
void G_EDM_UI_CONTROLLER::edit_setpoint_max()
{
  open_keyboard(vsense_drop_range_max, "Setpoint maximum (%)", "If fAVG is above this the \rtool retracts soft. (" + String( vsense_drop_range_max ) + "% = " + String( percentage_to_mv( vsense_drop_range_max ) ) + "mV)");
  float new_setpoint_max = get_keyboard_result();
  /** if setpoint range max is lower then setpoint range min we use the setpoint range min plus 1 **/
  if (new_setpoint_max <= vsense_drop_range_min)
  {
    new_setpoint_max = vsense_drop_range_min + 0.1;
  }
  if (new_setpoint_max > 100.0)
  {
    new_setpoint_max = 100.0;
  }
  vsense_drop_range_max = new_setpoint_max;
  generate_setpoint_min_max_adc();
};

void IRAM_ATTR G_EDM_UI_CONTROLLER::reset_flush_retract_timer()
{
  flush_retract_timer_us = esp_timer_get_time();
}

bool IRAM_ATTR G_EDM_UI_CONTROLLER::check_if_time_to_flush()
{
  if (flush_retract_after > 0 && (esp_timer_get_time() - flush_retract_timer_us >= flush_retract_after))
  {
    reset_flush_retract_timer();
    return true;
  }
  return false;
}

void G_EDM_UI_CONTROLLER::reset_defaults()
{
  probe_dimension                 = 0;
  sys_probe_state                 = Probe::Off;
  sys_rt_exec_alarm               = ExecAlarm::None;
  gconf.gedm_retraction_motion      = false;
  gconf.gedm_reprobe_block          = false;
  gconf.gedm_reprobe_motion         = false;
  gconf.gedm_single_axis_drill_task = false;
  gconf.gedm_floating_z_gcode_task  = false;
  gconf.gedm_wire_gcode_task        = false;
  gconf.edm_pause_motion          = false;
  //sys.edm_pause_motion_probe      = false;
  gconf.gedm_insert_probe           = false;
  sys.probe_succeeded             = true;
  gconf.gedm_reset_protocol         = false;
  gconf.gedm_stop_process           = false;
  gconf.gedm_probe_position_x       = 0.0;
  gconf.gedm_probe_position_y       = 0.0;
  enable_correction               = false;
  motion_plan                     = 0;
  has_last_position               = false;
  probe_touched                   = false;
  limit_touched                   = false;
  edm_process_is_running          = false;
  sys_rt_exec_state.bit.motionCancel = true;
  sys_rt_exec_state.bit.reset        = false;
  api::block_while_not_idle();
  sys.abort                         = false;
  force_redraw                      = true;
  planner.reset_planner_state();

  last_settings_write = 0;
  save_settings("last_settings");
  last_settings_write = esp_timer_get_time();
  sys.state = State::Idle;
  api::push_cmd("G91 G54 G21\r\n");
}

void G_EDM_UI_CONTROLLER::reset_after_job()
{

  if( enable_dpm_rxtx ){
      if( !dpm_driver.power_on_off( 0, 3 ) ){
        // problem... unhandled yet
      }// turn on
      vTaskDelay(200);
  }

  disable_spark_generator();
  stop_spindle();
  gconf.gedm_stop_process            = true;
  sys_rt_exec_state.bit.motionCancel = true;
  api::block_while_not_idle();
  filehandler->close_file();
  reset_spindle();
  motor_manager.set_ignore_disable(false);
  motor_manager.restore();
  vTaskDelay(100);
  planner.position_history_reset();
  filehandler->reset_job_finished();
  sys.state         = State::Idle;
  sys_rt_exec_alarm = ExecAlarm::None;
  sys_rt_exec_state.bit.reset = false;
  api::block_while_not_idle();
  render_page(1, true);
  force_redraw = true;
  api::unlock_machine();
  disable_spark_generator();
  api::block_while_not_idle();
  reset_defaults();
  render_page(1,true);

}



bool G_EDM_UI_CONTROLLER::ready_to_run()
{

  if (
      sys.abort
      /** if on/off switch is off **/
      || !start_edm_process()
      /** if pwm is off **/
      || !pwm_is_enabled()
      /** if edm is already running **/
      || edm_process_is_running
      /** if source voltage is below min voltage **/
      || (get_feedback_voltage() < vsense_voltage_min)
      || !I2S_is_ready()
      || !api::machine_is_idle()
  )
  {
    return false;
  }
  vTaskDelay(50);
  return true;
}


bool G_EDM_UI_CONTROLLER::gcode_job_running()
{

  return filehandler->is_locked_for_ui();
}

bool G_EDM_UI_CONTROLLER::get_is_ready()
{
  if (
      !is_ready
      || !protocol_ready || !filehandler->get_is_ready() || ( esp_timer_get_time() - start_time ) < LOADER_DELAY )
  { return false; }
  return true;
}
void G_EDM_UI_CONTROLLER::set_is_ready()
{
  vTaskDelay(500);
  is_ready = true;
}


/**
 *
 * Render the pages based on their pagenumber
 *
 **/
void G_EDM_UI_CONTROLLER::render_page(int value, bool redraw_full)
{
  //value = 9; operation_mode=4; // force to show the process overlay page
  int active_page = get_active_page();
  if (active_page != value)
  {
    redraw_full = true;
  }
  set_active_page(value);
  set_active_profile(active_profile);
  draw_page(value, redraw_full);
  vTaskDelay(200);
}

void G_EDM_UI_CONTROLLER::set_filehandler(G_FILEHANDLER *ptrfilehandler)
{

  filehandler = ptrfilehandler;
}





void G_EDM_UI_CONTROLLER::touch_calibrate( bool force )
{
  if (disable_tft_calibration && !force ){
    return;
  }
  uint16_t calData[5];
  uint8_t calDataOK = 0;
  if (!SPIFFS.begin()){
    SPIFFS.format();
    SPIFFS.begin();
  }
  if (SPIFFS.exists(TOUCH_CALIBRATION_FILE)){
    if (REPEAT_DISPLAY_CALIBRATION || force){
      SPIFFS.remove(TOUCH_CALIBRATION_FILE);
    } else {
      File file = SPIFFS.open(TOUCH_CALIBRATION_FILE, "r");
      if (file){
        if (file.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        file.close();
      }
    }
  }
  if (calDataOK && !REPEAT_DISPLAY_CALIBRATION){
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    fill_screen(TFT_BLACK);vTaskDelay(1);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.println("Touch corners as indicated");vTaskDelay(1);
    tft.setTextFont(1);
    tft.println();vTaskDelay(1);
    if( REPEAT_DISPLAY_CALIBRATION ){
      tft.setTextColor(TFT_GREENYELLOW);
      tft.println("");
      tft.println("To prevent this calibration from running again set");vTaskDelay(1);
      tft.println("");
      tft.println("REPEAT_DISPLAY_CALIBRATION in definitions_common.h");vTaskDelay(1);
      tft.println("");
      tft.println("to false and reflash the board");vTaskDelay(1);
    }
    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);
    tft.setTextColor(TFT_GREENYELLOW);
    tft.println("Calibration finished.");vTaskDelay(1);
    File file = SPIFFS.open(TOUCH_CALIBRATION_FILE, "w");
    if (file){
      file.write((const unsigned char *)calData, 14);vTaskDelay(1);
      file.close();vTaskDelay(1);
    }
  }
}

void G_EDM_UI_CONTROLLER::change_wire_spindle_speed(){
  open_keyboard( frequency_to_rpm( wire_spindle_speed ), "Wire spindle speed (rpm)", "Speed of the spindle stepper in wire mode\rRotations per minute");
  wire_spindle_speed = rpm_to_frequency( ( get_keyboard_result() ) );
  if( wire_spindle_speed > DEFAULT_A_MAX_FREQUENCY ){
    wire_spindle_speed = DEFAULT_A_MAX_FREQUENCY;
  }
  set_speed(wire_spindle_speed);
}

void G_EDM_UI_CONTROLLER::stop_spi(void)
{
  tft.getSPIinstance().end();
}
void G_EDM_UI_CONTROLLER::start_spi(void)
{
  tft.getSPIinstance().begin();
}

String G_EDM_UI_CONTROLLER::get_gcode_file(void)
{
  return gcode_file;
}
bool G_EDM_UI_CONTROLLER::get_touch(uint16_t *x, uint16_t *y)
{
  bool touched = false;
  if( tft.getTouch(x, y, 500) ){
    touched = true;
  }
  return touched;
}
float G_EDM_UI_CONTROLLER::get_z_stop()
{
  return z_stop_after_mm;
}

void G_EDM_UI_CONTROLLER::set_gcode_file(String filename)
{
  if (filename.length() <= 0)
  {
    if (operation_mode > 2)
    {
      // operation_mode = 1; // default to drill mode only after gcode job is done
    }
  }
  else
  {
    //operation_mode = 3;set_operation_mode
  }
  gcode_file = filename;
}
void G_EDM_UI_CONTROLLER::set_debug_bg_color(int num)
{
  debug_bg_color = num;
};
void G_EDM_UI_CONTROLLER::set_workload_travel(float total_workload_travel)
{
  this->total_workload_travel = total_workload_travel;
}
void G_EDM_UI_CONTROLLER::set_total_iterations(int total_iterations)
{
  this->total_iterations = total_iterations;
}
void G_EDM_UI_CONTROLLER::set_short_iterations(int short_iterations)
{
  this->short_iterations = short_iterations;
}
void G_EDM_UI_CONTROLLER::set_total_time(unsigned long total_time)
{
  this->total_time = total_time;
}
void G_EDM_UI_CONTROLLER::set_travel_time(unsigned long travel_time)
{
  this->travel_time = travel_time;
}
void G_EDM_UI_CONTROLLER::set_z_stop(float value)
{
  z_stop_after_mm = value; // global
}

void G_EDM_UI_CONTROLLER::set_z_no_home(bool value)
{
  z_no_home = value;
}
void G_EDM_UI_CONTROLLER::set_reamer_travel_mm(float _mm)
{
  reamer_travel_mm = _mm;
}
void G_EDM_UI_CONTROLLER::set_reamer_duration(float _seconds)
{
  reamer_duration = _seconds;
}
void G_EDM_UI_CONTROLLER::set_operation_mode(int mode)
{
  operation_mode = mode;
  if( mode > 2 && ! filehandler->get_has_sd_card() ){
    mode = 1; // fallback if no sd card is inserted
  }
  if( mode == 4 ){
    probe_dimension = 1;
  } else{
    probe_dimension = 0;
  }
}
void G_EDM_UI_CONTROLLER::set_flush_retract_mm(float _mm)
{
  flush_retract_mm = _mm;
}
void G_EDM_UI_CONTROLLER::set_disable_spark_for_flushing(bool disable)
{
  disable_spark_for_flushing = disable;
}
void G_EDM_UI_CONTROLLER::set_flush_offset_steps(int _steps)
{
  flush_offset_steps = _steps;
}
void G_EDM_UI_CONTROLLER::set_active_profile(String profile)
{
  this->active_profile = profile;
}

int G_EDM_UI_CONTROLLER::get_active_page()
{
  return active_page;
}
void G_EDM_UI_CONTROLLER::set_active_page(int page_num)
{
  active_page = page_num;
}
/** converting seconds to different units s/ms/us/ns **/
String G_EDM_UI_CONTROLLER::convert_timings(float seconds)
{
  if (seconds > 60)
  {
    return String(seconds / 60.0) + "min";
  }
  else if (seconds > 0.1)
  {
    return String(seconds) + "s";
  }
  else if (seconds > 0.001)
  {
    return String(seconds * 1000.0) + "ms";
  }
  else if (seconds > 0.000001)
  {
    return String(seconds * 1000000.0) + "us";
  }
  else
  {
    return String(seconds * 1000000000.0) + "ns";
  }
}
float G_EDM_UI_CONTROLLER::get_keyboard_result()
{
  return keyboard_value.length() > 0 ? keyboard_value.toFloat() : keyboard_initial_value;
}
String G_EDM_UI_CONTROLLER::get_keyboard_result_alpha()
{
  return keyboard_value.length() > 0 ? String(keyboard_value) : "";
}
void G_EDM_UI_CONTROLLER::close_keyboard()
{
  keyboard_is_active = false;
  has_first_stroke = false;
  spark_indicator_bar_width = 0;
}
void G_EDM_UI_CONTROLLER::open_keyboard_alpha(String value, String text, String infotext)
{
  if (keyboard_is_active)
  {
    return;
  }
  has_first_stroke = false;
  fill_screen(TFT_BLACK);
  vTaskDelay(1);
  keyboard_value = String(value);
  keyboard_initial_value_alpha = value;
  keyboard_is_active = true;
  int w = 30;
  int h = 30;
  int margin = 5;
  int px = margin;
  int py = margin;
  int t = 0;
  int c = 0;
  const char *data_sets[] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "-", "_"};
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  while (t++ <= 37)
  {
    vTaskDelay(1);
    if (c++ >= 10)
    {
      c = 1;
      py = py + h + 1;
      px = margin;
    }
    tft.fillRect(px, py, w, h, TFT_WHITE);vTaskDelay(1);
    tft.drawString(String(data_sets[t - 1]), px + 10, py + 10, 2);vTaskDelay(1);
    px = px + 1 + w;
  }
  /** delete button **/
  tft.fillRect(px, py, w * 2 + 1, h, TFT_RED);vTaskDelay(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("DEL", px + 20, py + 10, 2);vTaskDelay(1);
  /** text field **/
  py = py + h + 1;
  tft.fillRect(margin, py, 309, h + 10, TFT_BLACK);vTaskDelay(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(keyboard_value, margin + 10, py + 10, 2);vTaskDelay(1);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString(text, margin + 10, py + 40, 2);vTaskDelay(1);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  /** ok button **/
  py = py + h + 1;
  tft.fillRect(margin, py + 1 + 40, 248, h, TFT_MAROON);vTaskDelay(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Cancel", margin + 90, py + 48, 2);vTaskDelay(1);
  tft.fillRect(254, py + 1 + 40, 60, h, TFT_GREEN);vTaskDelay(1);
  tft.setTextColor(TFT_BLACK);
  tft.drawString("OK", 253 + 25, py + 48, 2);vTaskDelay(1);
  bool block_touch = false;
  uint16_t x, y;
  while (keyboard_is_active)
  {
    vTaskDelay(1);
    if (get_touch(&x, &y))
    {
      if (block_touch)
      {
        continue;
      }
      block_touch = true;
      if (x >= 5 && x <= 35 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "1";
      }
      else if (x >= 36 && x <= 66 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "2";
      }
      else if (x >= 67 && x <= 97 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "3";
      }
      else if (x >= 98 && x <= 128 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "4";
      }
      else if (x >= 129 && x <= 159 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "5";
      }
      else if (x >= 160 && x <= 190 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "6";
      }
      else if (x >= 191 && x <= 221 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "7";
      }
      else if (x >= 222 && x <= 252 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "8";
      }
      else if (x >= 253 && x <= 283 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "9";
      }
      else if (x >= 284 && x <= 314 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "0";
      }
      else if (x >= 5 && x <= 35 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "A";
      }
      else if (x >= 36 && x <= 66 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "B";
      }
      else if (x >= 67 && x <= 97 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "C";
      }
      else if (x >= 98 && x <= 128 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "D";
      }
      else if (x >= 129 && x <= 159 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "E";
      }
      else if (x >= 160 && x <= 190 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "F";
      }
      else if (x >= 191 && x <= 221 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "G";
      }
      else if (x >= 222 && x <= 252 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "H";
      }
      else if (x >= 253 && x <= 283 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "I";
      }
      else if (x >= 284 && x <= 314 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "J";
      }
      else if (x >= 5 && x <= 35 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "K";
      }
      else if (x >= 36 && x <= 66 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "L";
      }
      else if (x >= 67 && x <= 97 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "M";
      }
      else if (x >= 98 && x <= 128 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "N";
      }
      else if (x >= 129 && x <= 159 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "O";
      }
      else if (x >= 160 && x <= 190 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "P";
      }
      else if (x >= 191 && x <= 221 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "Q";
      }
      else if (x >= 222 && x <= 252 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "R";
      }
      else if (x >= 253 && x <= 283 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "S";
      }
      else if (x >= 284 && x <= 314 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "T";
      }
      else if (x >= 5 && x <= 35 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "U";
      }
      else if (x >= 36 && x <= 66 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "V";
      }
      else if (x >= 67 && x <= 97 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "W";
      }
      else if (x >= 98 && x <= 128 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "X";
      }
      else if (x >= 129 && x <= 159 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "Y";
      }
      else if (x >= 160 && x <= 190 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "Z";
      }
      else if (x >= 191 && x <= 221 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "-";
      }
      else if (x >= 222 && x <= 252 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "_";
      }
      else if (x >= 253 && x <= 313 && y >= 98 && y <= 128)
      {
        // delete
        keyboard_value = "";
      }
      else if (x >= 5 && x <= 254 && y >= 201 && y <= 240)
      {
        // cancel
        keyboard_value = "";
        break;
      }
      else if (x >= 253 && y >= 201 && y <= 240)
      {
        // ok
        break;
      }
      else
      {
        vTaskDelay(1);
        continue;
      }
      tft.fillRect(5, 129, 309, 40, TFT_BLACK);vTaskDelay(1);
      tft.setTextColor(TFT_WHITE);
      tft.drawString(keyboard_value, 15, 139, 2);vTaskDelay(1);
    }
    else
    {
      block_touch = false;
    }
    vTaskDelay(1);
  }
  close_keyboard();
  vTaskDelay(10);
  while( get_touch( &x, &y ) ){
    vTaskDelay(100);
  }
}
void G_EDM_UI_CONTROLLER::open_keyboard(float value, String text, String infotext)
{
  if (keyboard_is_active)
  {
    return;
  }
  fill_screen(TFT_BLACK);vTaskDelay(1);
  has_first_stroke = false;
  keyboard_value = String(value,3);
  keyboard_initial_value = value;
  keyboard_is_active = true;
  int w = button_width;
  int h = button_height;
  int margin = button_margin;
  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK);
  tft.fillRect(0, 0, 320, 2 * margin + h, TFT_BLACK);vTaskDelay(1);
  // 0-9
  tft.fillRect(margin, margin, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString("1", margin + w / 2, margin, 2);vTaskDelay(1);
  tft.fillRect(2 * margin + w, margin, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString("2", 2 * margin + w + w / 2, margin, 2);vTaskDelay(1);
  tft.fillRect(3 * margin + 2 * w, margin, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString("3", 3 * margin + 2 * w + w / 2, margin, 2);vTaskDelay(1);
  tft.fillRect(4 * margin + 3 * w, margin, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString("4", 4 * margin + 3 * w + w / 2, margin, 2);vTaskDelay(1);
  tft.fillRect(5 * margin + 4 * w, margin, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString("5", 5 * margin + 4 * w + w / 2, margin, 2);vTaskDelay(1);
  tft.fillRect(margin, 2 * margin + h, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString("6", margin + w / 2, 2 * margin + h, 2);vTaskDelay(1);
  tft.fillRect(2 * margin + w, 2 * margin + h, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString("7", 2 * margin + w + w / 2, 2 * margin + h, 2);vTaskDelay(1);
  tft.fillRect(3 * margin + 2 * w, 2 * margin + h, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString("8", 3 * margin + 2 * w + w / 2, 2 * margin + h, 2);vTaskDelay(1);
  tft.fillRect(4 * margin + 3 * w, 2 * margin + h, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString("9", 4 * margin + 3 * w + w / 2, 2 * margin + h, 2);vTaskDelay(1);
  tft.fillRect(5 * margin + 4 * w, 2 * margin + h, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString("0", 5 * margin + 4 * w + w / 2, 2 * margin + h, 2);vTaskDelay(1);
  // undo
  tft.fillRect(6 * margin + 5 * w, margin, w, h, TFT_RED);vTaskDelay(1);
  tft.drawString("DEL", 6 * margin + 5 * w + 2, 2 * margin, 2);vTaskDelay(1);
  // ,
  tft.fillRect(6 * margin + 5 * w, 2 * margin + h, w, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString(",", 6 * margin + 5 * w + w / 2, 2 * margin + h, 2);vTaskDelay(1);
  // result field
  draw_keyboard_result();
  // done
  tft.fillRect(5 * margin + 4 * w, 3 * margin + 2 * h, 2 * w + margin, h, TFT_GREEN);vTaskDelay(1);
  tft.drawString("OK", 5 * margin + 4 * w + w - margin, 3 * margin + 2 * h + margin, 2);vTaskDelay(1);
  // textbox title
  // tft.fillRect(margin, 4 * margin + 3 * h, 6 * w + 5 * margin, 2 * h, TFT_YELLOW);
  // tft.setTextColor(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);

  tft.setTextSize(1);
  tft.drawString(text, 2 * margin, 5 * margin + 3 * h, 2);vTaskDelay(1);

  tft.setTextColor(TFT_LIGHTGREY);

  // infotext
  if (infotext != "")
  {
    int size;
    String *t = split(infotext, '\r', size);

    if (size <= 1)
    {
      tft.drawString(infotext, 2 * margin, 5 * margin + 3 * h + 20, 2);vTaskDelay(1);
    }
    else
    {

      for (int i = 0; i < size; i++)
      {
        tft.drawString(t[i], 2 * margin, 5 * margin + 3 * h + 20 + (i * 20), 2);vTaskDelay(1);
      }
    }

    delete[] t;
  }

  bool block_touch = false;
  uint16_t x, y;
  while (keyboard_is_active)
  {
    vTaskDelay(1);
    if (get_touch(&x, &y))
    {
      if (block_touch)
      {
        continue;
      }
      block_touch = true;
      map_keys(x, y);
    }
    else
    {
      block_touch = false;
    }
    vTaskDelay(1);
  }
  close_keyboard();
}
void G_EDM_UI_CONTROLLER::draw_keyboard_result()
{
  int w = button_width;
  int h = button_height;
  int margin = button_margin;
  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK);
  tft.fillRect(margin, 3 * margin + 2 * h, 4 * w + 3 * margin, h, TFT_WHITE);vTaskDelay(1);
  tft.drawString(keyboard_value, 2 * margin, 4 * margin + 2 * h, 2);vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::map_keys(int x, int y)
{
  int w = button_width;
  int h = button_height;
  int margin = button_margin;
  int r = 500;
  if (x >= margin && x <= margin + w && y >= margin && y <= margin + h)
  {
    // 1
    r = 1;
  }
  else if (x >= 2 * margin + w && x <= 2 * margin + 2 * w && y >= margin && y <= margin + h)
  {
    // 2
    r = 2;
  }
  else if (x >= 3 * margin + 2 * w && x <= 3 * margin + 3 * w && y >= margin && y <= margin + h)
  {
    // 3
    r = 3;
  }
  else if (x >= 4 * margin + 3 * w && x <= 4 * margin + 4 * w && y >= margin && y <= margin + h)
  {
    // 4
    r = 4;
  }
  else if (x >= 5 * margin + 4 * w && x <= 5 * margin + 5 * w && y >= margin && y <= margin + h)
  {
    // 5
    r = 5;
  }
  else if (x >= 6 * margin + 5 * w && x <= 6 * margin + 6 * w && y >= margin && y <= margin + h)
  {
    // del
    r = 100;
  }
  else if (x >= margin && x <= margin + w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // 6
    r = 6;
  }
  else if (x >= 2 * margin + w && x <= 2 * margin + 2 * w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // 7
    r = 7;
  }
  else if (x >= 3 * margin + 2 * w && x <= 3 * margin + 3 * w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // 8
    r = 8;
  }
  else if (x >= 4 * margin + 3 * w && x <= 4 * margin + 4 * w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // 9
    r = 9;
  }
  else if (x >= 5 * margin + 4 * w && x <= 5 * margin + 5 * w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // 0
    r = 0;
  }
  else if (x >= 6 * margin + 5 * w && x <= 6 * margin + 6 * w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // ,
    r = 200;
  }
  else if (x >= 4 * margin + 3 * w && x <= 6 * margin + 6 * w && y >= 3 * margin + 2 * h && y <= 3 * margin + 3 * h)
  {
    // ok
    r = 300;
  }
  if (r == 100)
  {
    // delete
    keyboard_value = "";
  }
  else if (r == 300)
  {
    // done
    close_keyboard();
    return;
  }
  else if (r == 200)
  {
    if (!has_first_stroke)
    {
      has_first_stroke = true;
      keyboard_value = "";
    }
    if (keyboard_value.indexOf(".") == -1)
    {
      keyboard_value += ".";
    }
  }
  else if (r == 500)
  {
    // nothing
  }
  else
  {
    if (!has_first_stroke)
    {
      has_first_stroke = true;
      keyboard_value = "";
    }
    keyboard_value += String(r);
  }
  draw_keyboard_result();
}

void G_EDM_UI_CONTROLLER::draw_spark_on_off_indicator_icon()
{
  if (keyboard_is_active)
  {
    return;
  }
  int state; // 0=gray 1=green 2=red

  if (pwm_is_enabled())
  {
    state = 2; // allow disable
  }
  else
  {
    state = 1; // allow enable
  }

  if (get_feedback_voltage() < vsense_voltage_min)
  {
    state = 0; // voltage too low
  }

  if (state == spark_on_off_indicator_icon_last_state)
  {
    // no changes
    return;
  }

  spark_on_off_indicator_icon_last_state = state;

  tft.fillRect(249, 169, 62, 62, TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(1);

  switch (state)
  {

  case 0:
    tft.setTextColor(TFT_BLACK);
    tft.fillRoundRect(250, 170, 60, 60, 5, TFT_DARKGREY);vTaskDelay(1);
    tft.drawString("EDM", 255, 180, 2);vTaskDelay(1);
    tft.drawString("START", 255, 200, 2);vTaskDelay(1);
    break;

  case 1:
    tft.setTextColor(TFT_BLACK);
    tft.fillRoundRect(250, 170, 60, 60, 5, TFT_OLIVE);vTaskDelay(1);
    tft.drawString("EDM", 255, 180, 2);vTaskDelay(1);
    tft.drawString("START", 255, 200, 2);vTaskDelay(1);
    break;

  case 2:
    tft.setTextColor(TFT_WHITE);
    tft.fillRoundRect(250, 170, 60, 60, 5, TFT_RED);vTaskDelay(1);
    tft.drawString("EDM", 255, 180, 2);vTaskDelay(1);
    tft.drawString("STOP", 255, 200, 2);vTaskDelay(1);
    break;
  }

  vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::render_page_settings_menu()
{
  int ypos   = 0;
  int height = 40;
  for( int i = 0; i < 5; ++i ){
      tft.fillRect(0, ypos, 90, 40, TFT_OLIVE);vTaskDelay(1);
      ypos+=height;
      tft.drawLine( 0, ypos-1, 90, ypos-1, TFT_BLACK);vTaskDelay(1);
  }
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.drawString("PWM",      20, 12, 2);vTaskDelay(1);
  tft.drawString("Flushing", 20, 52, 2);vTaskDelay(1);
  tft.drawString("Spark",    20, 92, 2);vTaskDelay(1);
  tft.drawString("Mode",     20, 132, 2);vTaskDelay(1);
  tft.drawString("Motion",   20, 172, 2);vTaskDelay(1);
  render_page_settings_menu_sd_card(has_sd_card, true);
  vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::draw_sdcard_button( int _state ){

  if (keyboard_is_active || active_page != 1)
  {
    return;
  }
  tft.setTextSize(1);

  switch (_state)
  {
    case 0:
      tft.setTextColor(TFT_WHITE);
      tft.fillRect(0, 200, 90, 40, TFT_MAROON);vTaskDelay(1);
    break;
  
    case 1:
      tft.setTextColor(TFT_WHITE);
      tft.fillRect(0, 200, 90, 40, TFT_DARKGREY);vTaskDelay(1);
    break;

    case 2:
      tft.setTextColor(TFT_BLACK);
      tft.fillRect(0, 200, 90, 40, TFT_OLIVE);vTaskDelay(1);
    break;

    default:
      tft.setTextColor(TFT_BLACK);
      tft.fillRect(0, 200, 90, 40, TFT_OLIVE);vTaskDelay(1);
    break;

  }
  tft.drawString("SD_Card", 20, 212, 2);vTaskDelay(1);
  draw_mode_buttons();

}

void G_EDM_UI_CONTROLLER::render_page_settings_menu_sd_card(bool sd_card_has, bool force_redraw)
{
  if (keyboard_is_active || active_page != 1)
  {
    return;
  }
  bool is_locked_for_ui = filehandler->is_locked_for_ui();

  if (is_locked_for_ui)
  {
    sd_card_has = false;
  }

  if (has_sd_card == sd_card_has && !force_redraw)
  {
    return;
  }

  if (!sd_card_has)
  {
    set_gcode_file("");
  }

  has_sd_card = sd_card_has;
  tft.setTextSize(1);
  if (!has_sd_card)
  {
    draw_sdcard_button( is_locked_for_ui ? 1 : 0 );
  }
  else
  {
    draw_sdcard_button( 2 );
  }
  tft.drawString("SD_Card", 20, 212, 2);vTaskDelay(1);
  vTaskDelay(1);
}
/** Draws the values for the input PSU voltage and the vSense voltage reading **/
void G_EDM_UI_CONTROLLER::redraw_vsense()
{
}

void G_EDM_UI_CONTROLLER::draw_mpos()
{
  if (keyboard_is_active || active_page != 1){ return; }
  canvas.createSprite( 211, 17 );vTaskDelay(1);
  //canvas.fillSprite( TFT_BLACK );vTaskDelay(1);
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.drawString(get_mpos_string(), 0, 0, 2);vTaskDelay(1);
  canvas.pushSprite(95,35);vTaskDelay(1);
  canvas.deleteSprite();vTaskDelay(1);
}
/** PWM wave **/
void G_EDM_UI_CONTROLLER::draw_period_wave()
{
  if (keyboard_is_active)
  {
    return;
  }

  String text = convert_frequency_units(get_freq()) + " @ " + String(get_duty_percent()) + "%";
  float max_width_for_duty = 200.0;
  tft.fillRect(109, 61, 202, 32, TFT_BLACK);vTaskDelay(1);
  tft.fillRect(99, 215, 150, 20, TFT_BLACK);vTaskDelay(1);
  if (!pwm_is_enabled())
  {
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED);
    tft.drawString("PWM OFF " + text, 110, 71, 2);vTaskDelay(1);
  }
  else
  {
    // get the length of the duty line
    int duty_line_length = round(float(get_duty_percent()) / 100.0 * max_width_for_duty);
    if (duty_line_length <= 10)
    {
      duty_line_length = 10;
    }
    // raising edge
    tft.drawLine(110, 62, 110, 92, TFT_BLUE);vTaskDelay(1);
    // t_on
    tft.drawLine(110, 62, 110 + duty_line_length, 62, TFT_BLUE);vTaskDelay(1);
    // falling edge
    tft.drawLine(110 + duty_line_length, 62, 110 + duty_line_length, 92, TFT_BLUE);vTaskDelay(1);
    // t_off
    tft.drawLine(110 + duty_line_length, 92, 110 + max_width_for_duty, 92, TFT_BLUE);vTaskDelay(1);
  }
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.setTextColor(TFT_PURPLE);

  if ( operation_mode == 3 || operation_mode == 4 )
  {
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString("gCode: " + (gcode_file.length() > 0 ? gcode_file : "No file selected"), 110, 145, 2);vTaskDelay(1);
  }

  vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::draw_mode_buttons()

{

  if (keyboard_is_active || active_page != 1 )
  {
    return;
  }
  int width  = 45;
  int height = 30;
  int margin = 1;
  int active_color = TFT_GREENYELLOW;
  tft.setTextColor(TFT_BLACK);
  tft.fillRect(95, 0, width, height, operation_mode==1?active_color:TFT_OLIVE);vTaskDelay(1); //x=95,y=0,h,30,w,45
  tft.drawString("Drill", 103, 6, 2);vTaskDelay(1);
  tft.fillRect(95+margin+width, 0, width, height, operation_mode==2?active_color:TFT_OLIVE);vTaskDelay(1);
  tft.drawString("Ream",  103+1+width, 6, 2);vTaskDelay(1);
      if( ! filehandler->get_has_sd_card() ){
          tft.setTextColor(TFT_LIGHTGREY);
          tft.fillRect(95+2*margin+2*width, 0, width, height, TFT_DARKGREY);vTaskDelay(1);
          //tft.fillRect(95+3*margin+3*width, 0, width, height, TFT_DARKGREY);vTaskDelay(1);
      } else{
          //tft.fillRect(95+2*margin+2*width, 0, width, height, operation_mode==3?active_color:TFT_OLIVE);vTaskDelay(1);
          tft.fillRect(95+2*margin+2*width, 0, width, height, operation_mode==4?active_color:TFT_OLIVE);vTaskDelay(1);
          //tft.fillRect(95+3*margin+3*width, 0, width, height, operation_mode==4?active_color:(EXTENDED_FEATURES?TFT_OLIVE:TFT_DARKGREY));vTaskDelay(1);
      }
      //tft.drawString("3D",    96+2*5+2*width, 1, 2);vTaskDelay(1);
      //tft.drawString("Float", 96+2*5+2*width, 13, 2);vTaskDelay(1);
      tft.drawString("2D",   96+2*5+2*width, 1, 2);vTaskDelay(1);
      tft.drawString("Wire", 96+2*5+2*width, 13, 2);vTaskDelay(1);
      
      //tft.drawString("2D",    92+3*5+3*width, 1, 2);vTaskDelay(1);
      //tft.drawString("Wire",  92+3*5+3*width, 13, 2);vTaskDelay(1);
}




bool G_EDM_UI_CONTROLLER::motion_input_is_blocked()
{
  if ( edm_process_is_running 
       || sys_rt_exec_state.bit.motionCancel 
       || sys.state == State::CheckMode )
  {
    return true;
  }

  return false;
}
void G_EDM_UI_CONTROLLER::draw_motion_navigation_buttons()
{

  /** motion controller **/
  int color = TFT_GREEN;
  if ( motion_input_is_blocked() )
  {
    color = TFT_DARKGREY;
  }
  // tft.setTextColor( TFT_DARKGREEN );
  tft.fillTriangle(220, 178, 210, 198, 230, 198, color);vTaskDelay(1);
  tft.fillTriangle(210, 208, 220, 228, 230, 208, color);vTaskDelay(1);
  tft.fillTriangle(150, 178, 140, 198, 160, 198, color);vTaskDelay(1);
  tft.fillTriangle(140, 208, 150, 228, 160, 208, color);vTaskDelay(1);
  tft.fillTriangle(170, 193, 170, 213, 190, 203, color);vTaskDelay(1);
  tft.fillTriangle(110, 203, 130, 213, 130, 193, color);vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::render_page_front()
{
  spark_on_off_indicator_icon_last_state = -1; // reset
  draw_spark_on_off_indicator_icon();
  draw_mpos();
  redraw_vsense();
  //draw _period_wave();
  draw_motion_navigation_buttons();
  draw_mode_buttons();
}
void G_EDM_UI_CONTROLLER::render_page_settings_pwm()
{
  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("PWM Settings", 10, 10, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Frequency", 20, 40, 2);vTaskDelay(1);
  tft.drawString("Duty Cycle", 20, 70, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("Period", 20, 110, 2);vTaskDelay(1);
  tft.drawString("T_ON", 20, 130, 2);vTaskDelay(1);
  tft.drawString("T_OFF", 20, 150, 2);vTaskDelay(1);
  String unit;
  float value;
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN);
  tft.drawString(convert_frequency_units(get_freq()), 100, 40, 2);vTaskDelay(1);
  tft.drawString(String(get_duty_percent()) + "%", 100, 70, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.drawString(convert_timings(get_period()), 100, 110, 2);vTaskDelay(1);
  tft.drawString(convert_timings(get_t_on()), 100, 130, 2);vTaskDelay(1);
  tft.drawString(convert_timings(get_t_off()), 100, 150, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);vTaskDelay(1);
  tft.drawString("Done", 20, 200, 2);vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::render_page_settings_flushing()
{
  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("Flushing Settings", 10, 10, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Interval", 20, 40, 2);vTaskDelay(1);
  tft.drawString("Distance", 20, 70, 2);vTaskDelay(1);
  tft.drawString("Disable spark", 20, 100, 2);vTaskDelay(1);
  tft.drawString("Offset Steps", 20, 130, 2);vTaskDelay(1);
  tft.setTextColor(TFT_GREEN);
  tft.drawString(String(get_flushing_interval()) + "s", 120, 40, 2);vTaskDelay(1);
  tft.drawString(String(flush_retract_mm) + "mm", 120, 70, 2);vTaskDelay(1);
  tft.drawString(String(disable_spark_for_flushing ? "YES" : "NO"), 120, 100, 2);vTaskDelay(1);
  tft.drawString(String(flush_offset_steps), 120, 130, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);vTaskDelay(1);
  tft.drawString("Done", 20, 200, 2);vTaskDelay(1);
}


float G_EDM_UI_CONTROLLER::get_flushing_interval(){
  return float(flush_retract_after/1000/1000);
}

void G_EDM_UI_CONTROLLER::render_page_settings_spark()
{
  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("Spark Settings", 10, 10, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Setpoint Min", 20, 40, 2);vTaskDelay(1);
  tft.drawString("Setpoint Max", 20, 70, 2);vTaskDelay(1);
  tft.drawString("Retract Hard", 20, 100, 2);vTaskDelay(1);
  tft.drawString("Retract Soft", 20, 130, 2);vTaskDelay(1);
  tft.drawString("@", 193, 100, 2);vTaskDelay(1);
  tft.drawString("@", 193, 130, 2);vTaskDelay(1);
  tft.setTextColor(TFT_GREEN);
  tft.drawString(String( vsense_drop_range_min ) + "%" + " (" + String( percentage_to_mv( vsense_drop_range_min ) ) + "mV)", 130, 40, 2);vTaskDelay(1);
  tft.drawString(String( vsense_drop_range_max ) + "%" + " (" + String( percentage_to_mv( vsense_drop_range_max ) ) + "mV)", 130, 70, 2);vTaskDelay(1);
  tft.drawString(String( retconf.hard_retraction, 3 )+"mm", 130, 100, 2);vTaskDelay(1);
  tft.drawString(String( process_speeds_mm_min.WIRE_RETRACT_HARD, 3 )+"mm/min", 215, 100, 2);vTaskDelay(1);
  tft.drawString(String( retconf.soft_retraction, 3 )+"mm", 130, 130, 2);vTaskDelay(1);
  tft.drawString(String( process_speeds_mm_min.WIRE_RETRACT_SOFT, 3 )+"mm/min", 215, 130, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);vTaskDelay(1);
  tft.drawString("Done", 20, 200, 2);vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::render_page_settings_mode()
{
  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("Mode Settings", 10, 10, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Mode", 20, 40, 2);vTaskDelay(1);
  tft.setTextColor(TFT_GREEN);
  if (operation_mode == 1)
  {
    tft.drawString("Drill / Sinker", 100, 40, 2);vTaskDelay(1);
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Cutting depth", 20, 80, 2);vTaskDelay(1);
    tft.drawString("Direction", 20, 110, 2);vTaskDelay(1);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(String(z_stop_after_mm) + "mm", 160, 80, 2);vTaskDelay(1);
    String direction = "";
    switch (single_axis_mode_direction)
    {
        case 0:
            direction = "Z down";
        break;
        case 1:
            direction = "X left";
        break;
        case 2:
            direction = "X right";
        break;
        case 3:
            direction = "Y back";
        break;
        case 4:
            direction = "Y forward";
        break;
    }
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString(direction, 160, 110, 2);vTaskDelay(1);
  }
  else if (operation_mode == 2)
  {
    tft.drawString("Reamer", 100, 40, 2);vTaskDelay(1);
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Oscillation distance", 20, 80, 2);vTaskDelay(1);
    tft.drawString("Duration", 20, 110, 2);vTaskDelay(1);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(String(reamer_travel_mm) + "mm", 160, 80, 2);
    tft.drawString(String(reamer_duration) + "s", 160, 110, 2);vTaskDelay(1);
  }
  else if (operation_mode == 3)
  {
    tft.drawString("3D Floating GCode", 100, 40, 2);
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Cutting depth", 20, 80, 2);vTaskDelay(1);
    tft.drawString("Simulate GCode", 20, 110, 2);vTaskDelay(1);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(String(z_stop_after_mm) + "mm", 160, 80, 2);vTaskDelay(1);
    tft.drawString((simulate_gcode ? "YES" : "NO"), 160, 110, 2);vTaskDelay(1);
  }
  else if (operation_mode == 4)
  {
    tft.drawString("2D Wire GCode", 100, 40, 2);vTaskDelay(1);
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Wire spindle", 20, 80, 2);vTaskDelay(1);
    tft.drawString("Simulate GCode", 20, 110, 2);vTaskDelay(1);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(String(frequency_to_rpm(wire_spindle_speed)) + "rpm", 160, 80, 2);vTaskDelay(1);
    tft.drawString((simulate_gcode ? "YES" : "NO"), 160, 110, 2);vTaskDelay(1);

  }


  if (operation_mode <= 4)
  {
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Enable spindle", 20, 140, 2);vTaskDelay(1);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(enable_spindle ? "ON" : "OFF", 160, 140, 2);vTaskDelay(1);
  }
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);vTaskDelay(1);
  tft.drawString("Done", 20, 200, 2);vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::draw_homing_buttons(bool enabled)
{

  tft.fillRect(10, 129, 300, 42, TFT_LIGHTGREY);vTaskDelay(1);

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);

  if (sys_axis_homed[X_AXIS])
  {
    tft.drawString("Homed!", 95, 100, 2);vTaskDelay(1);
  }
  if (sys_axis_homed[Y_AXIS])
  {
    tft.drawString("Homed!", 165, 100, 2);vTaskDelay(1);
  }
  #ifndef Z_AXIS_NOT_USED
  if (sys_axis_homed[Z_AXIS] || z_no_home)
  {
    tft.drawString("Homed!", 235, 100, 2);vTaskDelay(1);
  }
  #endif

  if (!enabled||sys_rt_exec_state.bit.motionCancel)
  {
    tft.fillRoundRect(20, 130, 60, 40, 5, TFT_DARKGREY);vTaskDelay(1);
    tft.fillRoundRect(90, 130, 60, 40, 5, TFT_DARKGREY);vTaskDelay(1);
    tft.fillRoundRect(160, 130, 60, 40, 5, TFT_DARKGREY);vTaskDelay(1);
    #ifndef Z_AXIS_NOT_USED
    tft.fillRoundRect(230, 130, 60, 40, 5, TFT_DARKGREY);vTaskDelay(1);
    #endif
  }
  else
  {
    tft.fillRoundRect(20, 130, 60, 40, 5, TFT_OLIVE);vTaskDelay(1);
    tft.fillRoundRect(90, 130, 60, 40, 5, TFT_OLIVE);vTaskDelay(1);
    tft.fillRoundRect(160, 130, 60, 40, 5, TFT_OLIVE);vTaskDelay(1);
    #ifndef Z_AXIS_NOT_USED
    tft.fillRoundRect(230, 130, 60, 40, 5, TFT_OLIVE);vTaskDelay(1);
    #endif
  }
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  #ifdef Z_AXIS_NOT_USED
      tft.drawString("XY", 33, 132, 2);vTaskDelay(1);
  #else
      tft.drawString("ZXY", 27, 132, 2);vTaskDelay(1);
      tft.drawString("Z", 252, 132, 2);vTaskDelay(1);
  #endif
  tft.drawString("X", 112, 132, 2);vTaskDelay(1);
  tft.drawString("Y", 182, 132, 2);vTaskDelay(1);
}

String G_EDM_UI_CONTROLLER::convert_frequency_units(int frequency)
{
  String text;
  if (frequency > 1000000)
  {
    return String(float(frequency) / 1000000.0,1) + "MHz";
  }
  else if (frequency >= 500)
  {
    return String(float(frequency) / 1000.0,1) + "KHz";
  }
  else
  {
    return String(float(frequency),1) + "Hz";
  }
}


void G_EDM_UI_CONTROLLER::draw_toggle_probe_dimension()
{
  tft.setTextSize(1);
  tft.setTextColor(( probe_dimension == 0 ? TFT_WHITE : TFT_BLACK ));
  tft.fillRoundRect(155, 125, 37, 45, 5, ( probe_dimension == 0 ? TFT_DARKCYAN : TFT_GREEN ) );vTaskDelay(1);
  tft.drawString( ( probe_dimension == 0 ? "3D" : "2D"), 165, 140, 2);vTaskDelay(1);
}



void G_EDM_UI_CONTROLLER::draw_set_reprobe_button()
{
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.fillRoundRect(200, 125, 100, 45, 5, TFT_OLIVE);vTaskDelay(1);
  tft.drawString("Reprobe MPos", 205, 130, 2);vTaskDelay(1);
  tft.drawString("X" + String(gconf.gedm_probe_position_x) + " Y" + String(gconf.gedm_probe_position_y), 205, 145, 2);vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::draw_probing_buttons( int active )
{

  if( active != 0 ){
    render_page_settings_motion();
  }

  draw_toggle_probe_dimension();

  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Duty", 160, 50, 2);vTaskDelay(1);
  tft.drawString("Frequency", 160, 75, 2);vTaskDelay(1);
  tft.drawString("Trigger", 160, 100, 2);vTaskDelay(1);

  draw_set_reprobe_button();

  tft.setTextColor(TFT_MAROON);
  tft.drawString(String(pwm_duty_probing) + "%", 230, 50, 2);vTaskDelay(1);
  tft.drawString(convert_frequency_units(pwm_frequency_probing), 230, 75, 2);vTaskDelay(1);
  tft.drawString(String(vSense_drop_range_noload) + "%", 230, 100, 2);vTaskDelay(1);

  if (!machine_is_homed())
  {

    tft.setTextSize(1);
    tft.setTextColor(TFT_BLACK);
  }

  if (get_feedback_voltage() >= vsense_voltage_min && !motion_input_is_blocked()) 
  {

    sys_axis_homed[X_AXIS] = true;
    sys_axis_homed[Y_AXIS] = true;

    int active_color_one = TFT_DARKCYAN;
    if( probe_dimension == 1 ){
      active_color_one = TFT_GREEN;
    }

    tft.fillRect(20, 50, 35, 35, (active != 1 && sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? active_color_one : TFT_DARKGREY);vTaskDelay(1); //
    tft.fillRect(56, 50, 35, 35, (active != 2 && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? TFT_OLIVE : TFT_DARKGREY);vTaskDelay(1);                           // y
    tft.fillRect(92, 50, 35, 35, (active != 3 && sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? active_color_one : TFT_DARKGREY);vTaskDelay(1); //

    tft.fillRect(20, 86, 35, 35, (active != 4 && sys_axis_homed[X_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? TFT_OLIVE : TFT_DARKGREY);vTaskDelay(1); // x
    
    if( probe_dimension == 0 ){
        tft.fillRect(56, 86, 35, 35, (active != 5 && ( sys_axis_homed[Z_AXIS] || z_no_home ) ) ? TFT_OLIVE : TFT_DARKGREY);vTaskDelay(1);                             // z
    } else if( probe_dimension == 1 ){
      // 2d center finder
      //tft.fillCircle(73, 103, 17, TFT_BLACK);
      //tft.fillCircle(73, 103, 10, TFT_WHITE);
      tft.fillRect(56, 86, 35, 35, (active != 5 ? TFT_OLIVE : TFT_DARKGREY ) );vTaskDelay(1); // z
      tft.fillRect(59, 89, 29, 29, TFT_WHITE);vTaskDelay(1); // z
 

    }
    
    
    
    tft.fillRect(92, 86, 35, 35, (active != 6 && sys_axis_homed[X_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? TFT_OLIVE : TFT_DARKGREY);vTaskDelay(1); // x

    tft.fillRect(20, 122, 35, 35, (active != 7 && sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? active_color_one : TFT_DARKGREY);vTaskDelay(1); //
    tft.fillRect(56, 122, 35, 35, (active != 8 && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? TFT_OLIVE : TFT_DARKGREY);vTaskDelay(1);                           // y
    tft.fillRect(92, 122, 35, 35, (active != 9 && sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? active_color_one : TFT_DARKGREY);vTaskDelay(1); //

    tft.fillCircle(20, 50, 3, TFT_BLACK);vTaskDelay(1);
    tft.fillCircle(73, 50, 3, TFT_BLACK);vTaskDelay(1);
    tft.fillCircle(127, 50, 3, TFT_BLACK);vTaskDelay(1);

    tft.fillCircle(20, 103, 3, TFT_BLACK);vTaskDelay(1);
    tft.fillCircle(73, 103, 3, TFT_BLACK);vTaskDelay(1);
    tft.fillCircle(127, 103, 3, TFT_BLACK);vTaskDelay(1);

    tft.fillCircle(20, 156, 3, TFT_BLACK);vTaskDelay(1);
    tft.fillCircle(73, 156, 3, TFT_BLACK);vTaskDelay(1);
    tft.fillCircle(127, 156, 3, TFT_BLACK);vTaskDelay(1);
  }
  else
  {

    tft.setTextSize(1);
    tft.setTextColor(TFT_BLACK);
    if( motion_input_is_blocked() ){
        tft.drawString("Motion turned off!", 20, 50, 2);vTaskDelay(1);
    } else{
        tft.drawString("Voltage too low", 20, 50, 2);vTaskDelay(1);
    }
    tft.drawString("Probing disabled", 20, 70, 2);vTaskDelay(1);
  }
}

bool G_EDM_UI_CONTROLLER::machine_is_homed()
{

  if (
      sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home))
  {

    return true;
  }

  return false;
}

void G_EDM_UI_CONTROLLER::draw_motion_tab()
{

  tft.setTextSize(1);
  tft.fillRect(0, 39, 320, 206, TFT_BLACK);vTaskDelay(1);
  tft.fillRect(10, 40, 300, 135, TFT_LIGHTGREY);vTaskDelay(1); // content wrapper

  if (motion_tab_active == 1)
  {
    tft.fillRect(10, 10, 100, 30, TFT_LIGHTGREY);vTaskDelay(1); // tab homing
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString("Probing", 130, 15, 2);vTaskDelay(1);
    tft.drawString("More", 230, 15, 2);vTaskDelay(1);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("Homing", 30, 15, 2);vTaskDelay(1);
  }
  else if (motion_tab_active == 2)
  {
    tft.fillRect(110, 10, 100, 30, TFT_LIGHTGREY);vTaskDelay(1); // tab probing
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString("Homing", 30, 15, 2);vTaskDelay(1);
    tft.drawString("More", 230, 15, 2);vTaskDelay(1);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("Probing", 130, 15, 2);vTaskDelay(1);
  }
  else if (motion_tab_active == 3)
  {
    tft.fillRect(210, 10, 100, 30, TFT_LIGHTGREY);vTaskDelay(1); // tab tool
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString("Homing", 30, 15, 2);vTaskDelay(1);
    tft.drawString("Probing", 130, 15, 2);vTaskDelay(1);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("More", 230, 15, 2);vTaskDelay(1);
  }
}

String G_EDM_UI_CONTROLLER::get_mpos_string()
{

  //int64_t start = esp_timer_get_time();
  float *mpos = system_get_mpos();

  String mpos_string = "";

  for (int axis = 0; axis < N_AXIS; ++axis)
  {
    switch (axis)
    {
    case X_AXIS:
      mpos_string += "X" + String(mpos[axis]) + " ";
      break;
    case Y_AXIS:
      mpos_string += "Y" + String(mpos[axis]) + " ";
      break;
    case Z_AXIS:
      mpos_string += "Z" + String(mpos[axis]) + " ";
      break;
    case A_AXIS:
      mpos_string += "A" + String(mpos[axis]) + " ";
      break;
    case B_AXIS:
      mpos_string += "B" + String(mpos[axis]) + " ";
      break;
    case C_AXIS:
      mpos_string += "C" + String(mpos[axis]) + " ";
      break;
    default:
      mpos_string += "?" + String(mpos[axis]) + " ";
      break;
    }
  }

  //Serial.println(int(esp_timer_get_time()-start));//very slow function 144-350uS.... STring concat..

  return mpos_string;
}
void G_EDM_UI_CONTROLLER::render_page_process_overlay()
{
  // stop scope
  if( gscope.scope_is_running() ){
    gscope.stop();
  }
  fill_screen(TFT_BLACK);vTaskDelay(1);
  canvas.createSprite( 140, 80 );vTaskDelay(1);
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.drawString("WPos", 20, 0, 2);vTaskDelay(1);
  canvas.drawString("MPos", 90, 0, 2);vTaskDelay(1);
  canvas.drawString("X", 0, 20, 2);vTaskDelay(1);
  canvas.drawString("Y", 0, 40, 2);vTaskDelay(1);
  canvas.drawString("Z", 0, 60, 2);vTaskDelay(1);
  canvas.pushSprite( 0, 110 );vTaskDelay(1);
  canvas.deleteSprite();vTaskDelay(1);

  process_overlay_pause_button();
  process_overlay_coords();
  draw_process_params();
  process_overlay_reprobe_button();
  gscope.stop();
  gscope.init( 320, 95, 0, 0 );
  //gscope.init( 215, 95, 0, 0 );
  //gscope.init( 300, 95, 0, 0 );
  gscope.start();


}
volatile int pause_color_blink = 0;
volatile int64_t blink_timer = 0;
void G_EDM_UI_CONTROLLER::process_overlay_coords()
{ // only update function called in the process. it is either this one or the scope
  //int64_t start = esp_timer_get_time();
  float *mpos = api::get_wpos();
  canvas.createSprite( 150, 62 );vTaskDelay(1);
  if( gconf.edm_pause_motion ){
    int64_t timestamp = esp_timer_get_time();
    if( timestamp-blink_timer > 1000000 ){
        pause_color_blink = !pause_color_blink;
        blink_timer = timestamp;
    }
    if( pause_color_blink ){
      //canvas.fillSprite( TFT_BLACK );
    } else {
      canvas.fillSprite( TFT_RED );
    }
  } else {
    //canvas.fillSprite( TFT_BLACK );
  }
  vTaskDelay(1);
  int color = TFT_GREENYELLOW;
  if( gconf.edm_pause_motion ){
    color = TFT_LIGHTGREY;
  } else if( gconf.gedm_retraction_motion ){
    color = motion_plan < 5 ? TFT_CYAN : TFT_RED;
  }
  canvas.setTextColor( color );
  canvas.setTextSize(1);

  //canvas.drawString( sfeedback.pin_is_low?"low":"high", 0, 0, 2);vTaskDelay(1);

  canvas.drawFloat( mpos[0], 3, 0, 0, 2);vTaskDelay(1);
  canvas.drawFloat( mpos[1], 3, 0, 20, 2);vTaskDelay(1);
  canvas.drawFloat( mpos[2], 3, 0, 40, 2);vTaskDelay(1);
  mpos = system_get_mpos();
  canvas.drawFloat( mpos[0], 3, 70, 0, 2);vTaskDelay(1);
  canvas.drawFloat( mpos[1], 3, 70, 20, 2);vTaskDelay(1);
  canvas.drawFloat( mpos[2], 3, 70, 40, 2);vTaskDelay(1);
  canvas.pushSprite(20,130); vTaskDelay(1);
  canvas.deleteSprite(); vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::process_overlay_wire_spindle_speed_buttons()
{

}
void G_EDM_UI_CONTROLLER::process_overlay_reprobe_button()
{
  if (operation_mode != 3)
  {
    /** this is gcode floating z specific **/
    return;
  }
  canvas.createSprite( 60, 60 );vTaskDelay(1);
  //canvas.fillSprite( TFT_BLACK );vTaskDelay(1);
  canvas.setTextSize(1);
  canvas.setTextColor(gconf.gedm_insert_probe ? TFT_WHITE : TFT_BLACK);
  canvas.fillRoundRect(0, 0, 60, 60, 5, gconf.gedm_insert_probe ? TFT_MAROON : TFT_OLIVE);vTaskDelay(1);
  canvas.drawString(gconf.gedm_insert_probe ? "Reprobe" : "Reprobe", 5, 10, 2);vTaskDelay(1);
  canvas.drawString(gconf.gedm_insert_probe ? "Active" : "Insert", 5, 30, 2);vTaskDelay(1);
  canvas.pushSprite(250,100);vTaskDelay(1);
  canvas.deleteSprite();vTaskDelay(1);
}


void G_EDM_UI_CONTROLLER::draw_process_params()
{
  canvas.createSprite( 145, 90 );vTaskDelay(1);
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString("PWM",   0, 0,  2);vTaskDelay(1);
  canvas.drawString("Setp.", 0, 20, 2);vTaskDelay(1);
  canvas.drawString("fMax",  0, 40, 2);vTaskDelay(1);
  canvas.drawString("Spin",  0, 60, 2);vTaskDelay(1);
  canvas.drawString( 
    convert_frequency_units(get_freq())
    +" "+String(get_duty_percent(),1)
    +"%", 40, 0,  2);vTaskDelay(1);
  canvas.drawString( 
    String(vsense_drop_range_min,1)
    +"-"+String(vsense_drop_range_max,1)
    +"%", 40, 20,  2);vTaskDelay(1);
  canvas.drawString(
    String(max_feeds[Z_AXIS],2) + "mm/min",
    40, 40, 2);vTaskDelay(1);
  String text = "OFF";
  if( enable_spindle ){
    text = String( frequency_to_rpm(wire_spindle_speed),1 );
  }
  canvas.drawString( text, 40, 60, 2 );
  canvas.pushSprite(175,110);
  canvas.deleteSprite();
} 




void G_EDM_UI_CONTROLLER::process_overlay_pause_button()
{
  int width = 320;
  canvas.createSprite( 239, 30 );
  canvas.fillSprite( gconf.edm_pause_motion ? TFT_OLIVE : TFT_RED );
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString( gconf.edm_pause_motion ? "Resume" : "Pause", 100, 7, 2 );vTaskDelay(1);
  canvas.pushSprite( width-239, 210 );vTaskDelay(1);
  canvas.deleteSprite();vTaskDelay(1);
  canvas.createSprite( 80, 30 );
  canvas.fillSprite( TFT_OLIVE );
  canvas.drawString( "Settings", 12, 7, 2 );vTaskDelay(1);
  canvas.pushSprite( 0, 210 );vTaskDelay(1);
  canvas.deleteSprite();vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::update_process_meta()
{



}



void G_EDM_UI_CONTROLLER::render_page_settings_motion()
{

  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(1);
  draw_motion_tab();

  if (motion_tab_active == 1)
  {

    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Home Z", 20, 50, 2);vTaskDelay(1);
    tft.drawString("Press to move to origin", 20, 75, 2);vTaskDelay(1);
    tft.setTextColor(TFT_MAROON);
    tft.drawString(String(z_no_home ? "NO" : "YES"), 100, 50, 2);vTaskDelay(1);
    draw_homing_buttons(true);
  }
  else if (motion_tab_active == 2)
  {
    draw_probing_buttons();
  }
  else if (motion_tab_active == 3)
  {
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Tool diameter", 20, 50, 2);vTaskDelay(1);
    tft.drawString("Spindle", 20, 75, 2);vTaskDelay(1);
    tft.drawString("Feedrate MAX", 20, 100, 2);vTaskDelay(1);
    tft.drawString("Stepdelay EDM", 20, 125, 2);vTaskDelay(1);
    tft.drawString("Stepdelay Rapids", 20, 150, 2);vTaskDelay(1);
    tft.setTextColor(TFT_MAROON);
    tft.drawString(String(tool_diameter) + "mm", 140, 50, 2);vTaskDelay(1);
    tft.drawString( spindle_is_running() ? "Stop" : "Start", 140, 75, 2);vTaskDelay(1);
    tft.drawString(String(max_feeds[Z_AXIS],3) + "mm/min", 140, 100, 2);vTaskDelay(1);
    tft.drawString(String(process_speeds.EDM) + "ms", 140, 125, 2);vTaskDelay(1);
    tft.drawString(String(process_speeds.RAPID) + "ms", 140, 150, 2);vTaskDelay(1);
  }

  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);vTaskDelay(1);
  tft.drawString("Done", 20, 200, 2);vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::render_page_settings_sd(String active_profile)
{
  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("SD Card", 10, 10, 2);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Save current burn profile", 20, 40, 2);vTaskDelay(1);
  tft.drawString("Burn profile", 20, 70, 2);vTaskDelay(1);
  tft.drawString("gCode file", 20, 100, 2);vTaskDelay(1);

  tft.setTextColor(TFT_OLIVE);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);vTaskDelay(1);
  tft.drawString("Done", 20, 200, 2);vTaskDelay(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(active_profile, 120, 70, 2);vTaskDelay(1);
  tft.drawString((gcode_file.length() > 0 ? gcode_file : "None"), 120, 100, 2);vTaskDelay(1);
  if (gcode_file.length() > 0)
  {

    tft.setTextColor(TFT_RED);
    tft.drawString("Unload gCode", 20, 140, 2);vTaskDelay(1);
  }
}
void G_EDM_UI_CONTROLLER::render_page_settings_sd_profiles()
{
  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("Burn profiles", 10, 10, 2);vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::render_page_settings_sd_gcode()
{
  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("gCode", 10, 10, 2);vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::render_page_settings_sd_add_entry(String entry, int row)
{
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  int margin_top = 40;
  int row_height = 30;
  int y = margin_top + (row_height * row);
  tft.drawString(entry, 20, y, 2);vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::render_page_settings_sd_redraw_canvas()
{
  tft.fillRect(0, 39, 320, 160, TFT_BLACK);vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::draw_navigation_buttons(int current_page, int total_pages)
{
  tft.fillRect(0, 194, 320, 46, TFT_BLACK);vTaskDelay(1);
  if (total_pages <= 1)
  {
    // return;
  }
  if (current_page > 1)
  {
    // highlight backbutton
    tft.fillRect(90, 200, 50, 35, TFT_OLIVE);vTaskDelay(1);
    tft.drawLine(100, 218, 125, 208, TFT_BLACK);vTaskDelay(1);
    tft.drawLine(100, 218, 125, 228, TFT_BLACK);vTaskDelay(1);
  }
  else
  {
    tft.fillRect(90, 200, 50, 35, TFT_MAROON);vTaskDelay(1);
    tft.drawLine(100, 218, 125, 208, TFT_BLACK);vTaskDelay(1);
    tft.drawLine(100, 218, 125, 228, TFT_BLACK);vTaskDelay(1);
  }
  if (current_page == total_pages)
  {
    // no more forward
    tft.fillRect(265, 200, 50, 35, TFT_MAROON);vTaskDelay(1);
    tft.drawLine(305, 218, 280, 208, TFT_BLACK);vTaskDelay(1);
    tft.drawLine(305, 218, 280, 228, TFT_BLACK);vTaskDelay(1);
  }
  else
  {
    tft.fillRect(265, 200, 50, 35, TFT_OLIVE);vTaskDelay(1);
    tft.drawLine(305, 218, 280, 208, TFT_BLACK);vTaskDelay(1);
    tft.drawLine(305, 218, 280, 228, TFT_BLACK);vTaskDelay(1);
  }
  tft.fillRect(5, 200, 80, 35, TFT_MAROON);vTaskDelay(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(String(current_page) + " / " + String(total_pages), 185, 210, 2);vTaskDelay(1);
  tft.drawString("Close", 30, 208, 2);vTaskDelay(1);vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::select_list_begin(int num_files, char files[][100])
{

  int current_page = 1;
  int batch_size = 5;
  int current = 0;
  int from = 0;
  int to = batch_size;
  int i = 0;
  bool has_changed = true;
  bool finish = false;
  int total_pages = num_files / batch_size;
  bool block_touch = false;
  uint16_t ix, iy;

  if (float(num_files) / float(batch_size) != total_pages)
  {
    ++total_pages;
  }

  if (num_files <= 0)
  {

    render_page_settings_sd_add_entry("No .gcode files found. Touch to return.", 0);

    vTaskDelay(200);

    while (true)
    {

      if (get_touch(&ix, &iy))
      {

        break;
      }
    }
  }
  else
  {

    while (true)
    {

      if (has_changed)
      {
        /** delete current canvas ( fill black below title ) **/
        render_page_settings_sd_redraw_canvas();
      }

      /** add list items **/
      while (current < batch_size && i < num_files)
      {
        render_page_settings_sd_add_entry(String(files[i]), current);
        ++current;
        ++i;
        has_changed = true;
      }

      if (has_changed)
      {
        draw_navigation_buttons(current_page, total_pages);
        has_changed = false;
      }

      block_touch = false;

      while (true)
      {

        if (get_touch(&ix, &iy))
        {

          if (block_touch)
          {
            continue;
          }

          block_touch = true;

          if (ix >= 5 && ix <= 85 && iy >= 200 && iy <= 240)
          {

            // close
            finish = true;
            break;
          }
          else if (current_page > 1 && ix >= 90 && ix <= 140 && iy >= 200 && iy <= 240)
          {

            // previous page
            --current_page;
            i = i - (current + 1) - batch_size;
            current = 0;
            if (i < 0)
            {
              i = 0;
            }

            has_changed = true;
            vTaskDelay(50);
            break;
          }
          else if (total_pages > 1 && current_page < total_pages && ix >= 265 && ix <= 315 && iy >= 200 && iy <= 240)
          {

            // next page
            ++current_page;
            current = 0;
            has_changed = true;
            vTaskDelay(50);
            break;
          }
          else if (ix > 0 && ix < 320 && iy > 40 && iy < 200)
          {

            // touch within list canvas
            // row height is 30px and the first entry is 40px below the top.
            int selected_row = (iy - 40) / 30;
            float f_selected_row = (float(ix) - 40.0) / 30.0;

            if (selected_row != f_selected_row)
            {
              ++selected_row;
            }

            /** the current index "i" is the last entry on the current page **/
            int array_item = i - 1 - current + selected_row;
            selected_list_item = files[array_item];
            finish = true;
            break;
          }

          // end if xy
        }

        block_touch = false;
      }

      if (finish)
      {
        break;
      }
    }
  }
}

String G_EDM_UI_CONTROLLER::get_selected_item()
{
  return selected_list_item;
}
void G_EDM_UI_CONTROLLER::unselected_item()
{
  selected_list_item = "";
}


void G_EDM_UI_CONTROLLER::probe_done(){
  if( enable_spindle ){
    stop_spindle();
    set_speed(probe_backup_frequency);
  }  
  probe_mode_off();
}
bool G_EDM_UI_CONTROLLER::probe_prepare( int disable_index, bool is_3d ){
    draw_probing_buttons( disable_index );
    probe_mode_on();
    generate_reference_voltage();
    probe_backup_frequency = get_speed();
    if( enable_spindle ){
      set_speed(60);
      start_spindle();
    }
    return true;
}
void G_EDM_UI_CONTROLLER::draw_interface()
{
  fill_screen(TFT_BLACK);vTaskDelay(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  render_page_settings_menu();
  vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::draw_page(int page_num, bool redraw_full)
{

  edm_start_stop();

  if (page_num == 1)
  {
    /** Front page **/
    if (redraw_full)
    {
      draw_interface();
    }
    render_page_front();
    gscope.init( 215, 95, 95, 65 );
    gscope.start();
  }
  else if (page_num == 2)
  { // was benchmark in the past...
    /** Front page **/
    if (redraw_full)
    {
      draw_interface();
    }
    render_page_front();
  }
  else if (page_num == 3)
  {
    /** PWM settings page **/
    render_page_settings_pwm();
  }
  else if (page_num == 4)
  {
    /** Flushing settings page **/
    render_page_settings_flushing();
  }
  else if (page_num == 5)
  {
    /** Spark settings page **/
    render_page_settings_spark();
  }
  else if (page_num == 6)
  {
    /** Mode settings page **/
    render_page_settings_mode();
  }
  else if (page_num == 7)
  {
    /** Motion settings page **/
    // set_motion_tab_active( 1 );
    render_page_settings_motion();
  }
  else if (page_num == 8)
  {
    /** Motion settings page **/
    render_page_settings_sd(active_profile);
  }
  else if (page_num == 9)
  {
    render_page_process_overlay();
  }
}

void G_EDM_UI_CONTROLLER::set_motion_tab_active(int active_tab)
{

  motion_tab_active = active_tab;
}



/**
 * Monitoring touch events
 * and map them to the callback functions
 **/
void G_EDM_UI_CONTROLLER::monitor_touch_input()
{

  uint16_t x, y;
  bool redraw = false;
  bool block_touch = false;


  if (get_touch(&x, &y))
  {

    if (block_touch)
    {
      /** debounce **/
      vTaskDelay(1);
      return;
    }

    block_touch = true;

    if (active_page == 1)
    {

      if( x >= 95 && x <= 295 && y >= 65 && y <= 110 ){
        open_scope_settings();
      }
      else if( x >= 295 && x <= 320 && y >= 65 && y <= 85 ){
        gscope.toogle_values();
      }

      /** frontpage **/
      if (x >= 0 && x <= 90 && y >= 0 && y <= 40)
      {

        /** show pwm settings page **/
        render_page(3, true);
      }
      else if (x >= 0 && x <= 90 && y >= 40 && y <= 80)
      {

        /** show flushing settings page **/
        render_page(4, true);
      }
      else if (x >= 0 && x <= 90 && y >= 80 && y <= 120)
      {

        /** show spark settings page **/
        render_page(5, true);
      }
      else if (x >= 0 && x <= 90 && y >= 120 && y <= 160)
      {

        /** show mode settings page **/
        render_page(6, true);
      }
      else if (x >= 0 && x <= 90 && y >= 160 && y <= 200)
      {

        /** show motion settings page **/
        render_page(7, true);
      }
      else if (x >= 0 && x <= 90 && y >= 200 && y <= 240)
      {

        if (!filehandler->is_locked_for_ui())
        {

          filehandler->sd_card_refresh();

          if (filehandler->get_has_sd_card())
          {

            // show save/load/restore settings page
            render_page(8, true);
          }
          else
          {

            // render_page( 1, false );
          }
        }
      }
      else if (x >= 240 && y >= 160)
      {
        /** PWM on/off button **/
        /** disable touch deactivation if edm process is running **/
        if (!edm_process_is_running)
        {
          if (get_feedback_voltage() < vsense_voltage_min)
          {
            disable_spark_generator();
          }
          else
          {
            /*if( USE_DPM_RXTX_SERIAL_COMMUNICATION ){
              dpm_driver.power_on_off( int( !pwm_is_enabled() ) );
              vTaskDelay(100);
            }*/

            toggle_pwm_on_off();
          }
          redraw = true;
        }
      }

      /** motion via front page navigation arrows **/
      else if (x >= 210 && x <= 230 && y >= 178 && y <= 198)
      {
        /** move Z up **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog Z up (mm)", "Jogging Z up\r" + get_mpos_string());
          if( ! api::jog_up( get_keyboard_result() ) ){
            add_dialog_screen( "The wanted position is out of reach" );
          }
          redraw = true;
        } else {
          add_dialog_screen( "Turn on/off switch on first!" );
          redraw = true;
        }
      }
      else if (x >= 210 && x <= 230 && y >= 208 && y <= 228)
      {
        /** move Z down **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog Z down (mm)", "Jogging Z down\r" + get_mpos_string());
          if( ! api::jog_down( get_keyboard_result() ) ){
            add_dialog_screen( "The wanted position is out of reach" );
          }
          redraw = true;
        } else {
          add_dialog_screen( "Turn on/off switch on first!" );
          redraw = true;
        }
      }


      #ifdef CNC_TYPE_IS_ROUTER


      else if (x >= 140 && x <= 160 && y >= 208 && y <= 228)
      {
        /** move Y up **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog Y down (mm)", "Jogging Y down\r" + get_mpos_string());
          if( ! api::jog_back( get_keyboard_result() ) ){
            add_dialog_screen( "The wanted position is out of reach" );
          }
          redraw = true;
        } else {
          add_dialog_screen( "Turn on/off switch on first!" );
          redraw = true;
        }
      }
      else if (x >= 140 && x <= 160 && y >= 178 && y <= 198)
      {
        /** move Y down **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog Y up (mm)", "Jogging Y up\r" + get_mpos_string());
          if( ! api::jog_forward( get_keyboard_result() ) ){
              add_dialog_screen( "The wanted position is out of reach" );
          }
          redraw = true;
        } else {
          add_dialog_screen( "Turn on/off switch on first!" );
          redraw = true;
        }
      }
      else if (x >= 170 && x <= 190 && y >= 193 && y <= 213)
      {
        /** move X left **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog X right (mm)", "Jogging X right\r" + get_mpos_string());
          if( ! api::jog_left( get_keyboard_result() ) ){
              add_dialog_screen( "The wanted position is out of reach" );
          }
          redraw = true;
        } else {
          add_dialog_screen( "Turn on/off switch on first!" );
          redraw = true;
        }
      }
      else if (x >= 110 && x <= 130 && y >= 193 && y <= 213)
      {
        /** move X right **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog X left (mm)", "Jogging X left\r" + get_mpos_string());
          if( ! api::jog_right( get_keyboard_result() ) ){
              add_dialog_screen( "The wanted position is out of reach" );
          }
          redraw = true;
        } else {
          add_dialog_screen( "Turn on/off switch on first!" );
          redraw = true;
        }
      }


      #else

      else if (x >= 140 && x <= 160 && y >= 178 && y <= 198)
      {
        /** move Y up **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog Y up (mm)", "Jogging Y up\r" + get_mpos_string());
          if( ! api::jog_back( get_keyboard_result() ) ){
            add_dialog_screen( "The wanted position is out of reach" );
          }
          redraw = true;
        } else {
          add_dialog_screen( "Turn on/off switch on first!" );
          redraw = true;
        }
      }
      else if (x >= 140 && x <= 160 && y >= 208 && y <= 228)
      {
        /** move Y down **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog Y down (mm)", "Jogging Y down\r" + get_mpos_string());
          if( ! api::jog_forward( get_keyboard_result() ) ){
              add_dialog_screen( "The wanted position is out of reach" );
          }
          redraw = true;
        } else {
          add_dialog_screen( "Turn on/off switch on first!" );
          redraw = true;
        }
      }
      else if (x >= 110 && x <= 130 && y >= 193 && y <= 213)
      {
        /** move X left **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog X left (mm)", "Jogging X left\r" + get_mpos_string());
          if( ! api::jog_left( get_keyboard_result() ) ){
              add_dialog_screen( "The wanted position is out of reach" );
          }
          redraw = true;
        } else {
          add_dialog_screen( "Turn on/off switch on first!" );
          redraw = true;
        }
      }
      else if (x >= 170 && x <= 190 && y >= 193 && y <= 213)
      {
        /** move X right **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog X right (mm)", "Jogging X right\r" + get_mpos_string());
          if( ! api::jog_right( get_keyboard_result() ) ){
              add_dialog_screen( "The wanted position is out of reach" );
          }
          redraw = true;
        } else {
          add_dialog_screen( "Turn on/off switch on first!" );
          redraw = true;
        }
      }



      #endif








      else if (x >= 95 && x <= 140 && y >= 0 && y <= 30)
      {
        /** drill mode **/
        set_operation_mode(1);
        redraw = true;
      }
      else if (x >= 141 && x <= 186 && y >= 0 && y <= 30)
      {
        /** reamer mode **/
        set_operation_mode(2);
        redraw = true;
      } 
      /*else if (x >= 187 && x <= 232 && y >= 0 && y <= 30)
      {
        if( filehandler->get_has_sd_card() ){
            set_operation_mode(3);
            redraw = true;
        }
      } */
      else if (x >= 187 && x <= 232 && y >= 0 && y <= 30)
      //else if (x >= 232 && x <= 277 && y >= 0 && y <= 30)
      {

        if( EXTENDED_FEATURES && filehandler->get_has_sd_card() ){
            /** 2d wire mode **/
            set_operation_mode(4);
            redraw = true;
        }


      } 

      if (redraw)
      {
        render_page(1, true);
      }
    }
    else if (active_page == 2)
    {

      /** benchmark page is touched. Back to the frontpage **/
      render_page(1, true);
    }
    else if (active_page == 3)
    {

      /** PWM settings page **/
      if (x >= 10 && x <= 310 && y >= 190 && y <= 230)
      {

        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 0 && x <= 320 && y >= 30 && y <= 55)
      {

        /** change PWM frequency **/
        edit_pwm_frequency();
        redraw = true;
      }
      else if (x >= 0 && x <= 160 && y >= 60 && y <= 85)
      {

        /** change PWM duty **/
        edit_pwm_duty();
        redraw = true;
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    else if (active_page == 4)
    {
      /** Flushing settings page **/
      if (x >= 10 && x <= 310 && y >= 190 && y <= 230)
      {
        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 0 && x <= 320 && y >= 30 && y <= 55)
      {
        /** change flushing movement interval in ms **/
        open_keyboard(get_flushing_interval(), "F_int (s)", "Up/Down flushing interval in seconds");
        float seconds = get_keyboard_result();
        flush_retract_after = round(seconds*1000.0*1000.0);
        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 60 && y <= 85)
      {
        /** change flushing movement distance in mm **/
        open_keyboard(flush_retract_mm, "F_dis (mm)", "Tool moves given mm up while flushing");
        flush_retract_mm = get_keyboard_result();
        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 90 && y <= 115)
      {
        disable_spark_for_flushing = !disable_spark_for_flushing;
        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 120 && y <= 145)
      {
        /** change flushing movement offset steps **/
        open_keyboard(flush_offset_steps, "Offset steps (steps)", "Axis moves back to flushing start position \rminus given offset steps");
        int result = int(round(get_keyboard_result()));
        flush_offset_steps = result;
        redraw = true;
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    else if (active_page == 5)
    {

      /**Spark settings page **/
      if (x >= 10 && x <= 310 && y >= 190 && y <= 230)
      {

        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 0 && x <= 320 && y >= 30 && y <= 55)
      {
        /** changing setpoint range min **/
        edit_setpoint_min();
        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 60 && y <= 85)
      {
        /** changing setpoint range max **/
        edit_setpoint_max();
        redraw = true;
      }

      else if (x >= 0 && x <= 193 && y >= 90 && y <= 115)
      {

        change_retraction_distance( 1 );
        redraw = true;
 
      }
      else if (x >= 0 && x <= 193 && y >= 120 && y <= 145)
      {
        change_retraction_distance( 2 );
        redraw = true;

      }
      else if (x >= 215 && x <= 320 && y >= 90 && y <= 115)
      {
        change_retraction_speed( 1 );
        redraw = true;
      }
      else if (x >= 215 && x <= 320 && y >= 120 && y <= 145)
      {
        change_retraction_speed( 2 );
        redraw = true;
      }
      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    else if (active_page == 6)
    {

      /** Mode settings page **/
      if (x >= 10 && x <= 310 && y >= 190 && y <= 230)
      {

        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 0 && x <= 320 && y >= 30 && y <= 55)
      {

        /** changing the operation mode **/
        ++operation_mode;
        if( operation_mode == 3 ){
          operation_mode = 4; // skip the floating mode for now. To much has changed and it is not tested
        }

        if ( operation_mode > 4 
             || ( operation_mode > 2 && ! filehandler->get_has_sd_card() )
             || ( operation_mode > 3 && !EXTENDED_FEATURES ) 
           )
        {
          set_operation_mode(1); // jump back to first
        }
        set_operation_mode( operation_mode );

        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 70 && y <= 95)
      {

        if (operation_mode == 1 || operation_mode == 3)
        {

          /** change Z travel limit; 0 means no limits **/
          open_keyboard(z_stop_after_mm, "Z_STOP (mm)", "Tool travels given mm counting from the \rfirst contact");
          set_z_stop(get_keyboard_result());

          redraw = true;
        }
        else if (operation_mode == 2)
        {

          open_keyboard(reamer_travel_mm, "REAMER_DIS (mm)", "Tool oscillates given mm up and down");
          reamer_travel_mm = get_keyboard_result();

          redraw = true;
        }
        else if (operation_mode == 4)
        {
          change_wire_spindle_speed();
          redraw = true;
        }
      }
      else if (x >= 0 && x <= 320 && y >= 100 && y <= 125)
      {

        if(operation_mode==1){
          if(++single_axis_mode_direction>4){single_axis_mode_direction=0;}
          change_sinker_axis_direction(single_axis_mode_direction);
          redraw = true;
        }

        if (operation_mode == 2)
        {

          open_keyboard(reamer_duration, "Reamer duration (s)", "Total duration of the reaming process in \rseconds");
          reamer_duration = get_keyboard_result();

          redraw = true;
        }
        else if ( operation_mode == 3 || operation_mode == 4 )
        {
          simulate_gcode = !simulate_gcode;
          redraw = true;
        }
      }
      else if (x >= 0 && x <= 320 && y >= 130 && y <= 155)
      {

        if (operation_mode <= 4)
        {
          enable_spindle = !enable_spindle;
          redraw = true;
        }
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    else if (active_page == 7)
    {

      /** Motion settings page **/
      if (x >= 10 && x <= 300 && y >= 190 && y <= 230)
      {

        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 10 && x <= 100 && y >= 10 && y <= 40)
      {

        // homing tab
        if (motion_tab_active != 1)
        {
          set_motion_tab_active(1);
          redraw = true;
        }
      }
      else if (x >= 100 && x <= 200 && y >= 10 && y <= 40)
      {

        // probing tab
        if (motion_tab_active != 2)
        {
          set_motion_tab_active(2);
          redraw = true;
        }
      }
      else if (x >= 200 && x <= 300 && y >= 10 && y <= 40)
      {

        // probing tab
        if (motion_tab_active != 3)
        {
          set_motion_tab_active(3);
          redraw = true;
        }
      }

      if (motion_tab_active == 1)
      {

        /** homing tab octive**/
        if (x >= 0 && x <= 320 && y >= 40 && y <= 65)
        {

          /** toggle Z axis no home **/
          z_no_home = !z_no_home;
          redraw = true;
        }

        else if(x >= 0 && x <= 320 && y >= 65 && y <= 90 && !motion_input_is_blocked()){
          // move to 0,0 WPos
          api::push_cmd("G90 G0 X0 Y0\r\n");
          redraw = true;
        }

        else if (x >= 20 && x <= 80 && y >= 130 && y <= 170 && !motion_input_is_blocked() )
        {

          for (int i = 0; i < N_AXIS; ++i)
          {
            sys_axis_homed[i] = false;
          }

          draw_homing_buttons(false);
          api::home_z();
          api::home_x();
          api::home_y();
          redraw = true;
        }

        else if (x >= 90 && x <= 150 && y >= 130 && y <= 170 && !motion_input_is_blocked())
        {

          draw_homing_buttons(false);
          api::home_x();
          redraw = true;
        }
        else if (x >= 160 && x <= 220 && y >= 130 && y <= 170 && !motion_input_is_blocked())
        {

          draw_homing_buttons(false);
          api::home_y();
          redraw = true;
        }
        #ifndef Z_AXIS_NOT_USED
        else if (x >= 230 && x <= 290 && y >= 130 && y <= 170 && !motion_input_is_blocked())
        {

          draw_homing_buttons(false);
          api::home_z();
          redraw = true;
        }
        #endif
      }
      else if (motion_tab_active == 2)
      {

        /** probing tab active **/
        if (x >= 160 && x <= 320 && y >= 40 && y <= 65)
        {

          // change probing duty
          open_keyboard(pwm_duty_probing, "Probing PWM duty (%)", "Duty cycle used for probing");
          pwm_duty_probing = get_keyboard_result();
          if (pwm_duty_probing > PWM_DUTY_MAX)
          {
            pwm_duty_probing = PWM_DUTY_MAX;
          }
          redraw = true;
        }
        else if (x >= 160 && x <= 320 && y >= 65 && y <= 90)
        {

          // change probing frequency
          open_keyboard(pwm_frequency_probing, "Probing PWM frequency (hz)", "PWM frequency used for probing");
          pwm_frequency_probing = int(get_keyboard_result());
          if (pwm_frequency_probing < PWM_FREQUENCY_MIN)
          {
            pwm_frequency_probing = PWM_FREQUENCY_MIN;
          }
          if (pwm_frequency_probing > PWM_FREQUENCY_MAX)
          {
            pwm_frequency_probing = PWM_FREQUENCY_MAX;
          }
          redraw = true;
        }
        else if (x >= 160 && x <= 320 && y >= 90 && y <= 115)
        {
          /** changing the minimum inputvoltage. Makes debugging sometimes easier if the process starts even at 0v. **/
          open_keyboard(vSense_drop_range_noload, "Voltage drop for probe (%)", "A voltagedrop above this value is considered a contact");
          float new_drop_noload = get_keyboard_result();

          if (new_drop_noload <= 1.0)
          {
            new_drop_noload = 1.0;
          }
          else if (new_drop_noload >= 50.0)
          {
            new_drop_noload = 50.0;
          }

          vSense_drop_range_noload = new_drop_noload;

          redraw = true;
        }
        else if (x >= 200 && x <= 320 && y >= 125 && y <= 170)
        {

          set_reprobe_point();
          draw_set_reprobe_button();
        }
        else if (x >= 155 && x <= 192 && y >= 125 && y <= 170)
        {
          if( ++probe_dimension > 1 ){
            probe_dimension = 0;
          }
          if( operation_mode == 4 ){
            // in 2D wire mode no 3D probe is allowed as it would crash the wire extension
            probe_dimension = 1; // force 2D probe 
          }
          draw_probing_buttons();
          //draw_toggle_probe_dimension();
        }
        else if (get_feedback_voltage() >= vsense_voltage_min && !motion_input_is_blocked())
        {

          if (x >= 56 && x <= 91 && y >= 86 && y <= 121 && (sys_axis_homed[Z_AXIS] || z_no_home))
          {
            probe_prepare(5,probe_dimension==1?false:true);
            /** probing only z **/
            if( probe_dimension == 0 ){
              probe_z(-1.0);
            } else if( probe_dimension == 1 ){
              center_finder_2d();
            }
            probe_done();
            redraw = true;
          }
          else if (x >= 56 && x <= 91 && y >= 50 && y <= 85 && (sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(2);
            probe_y(-1.0, true);
            probe_done();
            redraw = true;
          }
          else if (x >= 56 && x <= 91 && y >= 122 && y <= 157 && (sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(8);
            probe_y(1.0, true);
            probe_done();
            redraw = true;
          }
          else if (x >= 92 && x <= 127 && y >= 86 && y <= 121 && (sys_axis_homed[X_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(6);
            probe_x(-1.0, true);
            probe_done();
            redraw = true;
          }
          else if (x >= 20 && x <= 55 && y >= 86 && y <= 121 && (sys_axis_homed[X_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(4);
            probe_x(1.0, true);
            probe_done();
            redraw = true;
          }
          else if (x >= 92 && x <= 127 && y >= 50 && y <= 85 && (sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(3,probe_dimension==1?false:true);
            if( probe_dimension == 0 ){
                // 3D probing
                right_back_edge_3d();
            } else if( probe_dimension == 1 ){
                // 2D probing: doesn't involve any Z motions
                right_back_edge_2d();
            }
            probe_done();
            redraw = true;
          }
          else if (x >= 92 && x <= 127 && y >= 122 && y <= 157 && (sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(9,probe_dimension==1?false:true);
            if( probe_dimension == 0 ){
              // 3D probing
              right_front_edge_3d();
            } else if( probe_dimension == 1 ){
              // 2D probing: doesn't involve any Z motions
              right_front_edge_2d();
            }
            probe_done();
            redraw = true;
          }
          else if (x >= 20 && x <= 55 && y >= 50 && y <= 85 && (sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(1,probe_dimension==1?false:true);
            if( probe_dimension == 0 ){
                // 3D probing
                left_back_edge_3d();
            } else if( probe_dimension == 1 ){
              // 2D probing: doesn't involve any Z motions
              left_back_edge_2d();
            }

            probe_done();
            redraw = true;
          }
          else if (x >= 20 && x <= 55 && y >= 122 && y <= 157 && (sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(7,probe_dimension==1?false:true);
            if( probe_dimension == 0 ){
                // 3D probing: First probe Z then X and Y
                left_front_edge_3d();
            } else if( probe_dimension == 1 ){
              // 2D probing: doesn't involve any Z motions
              left_front_edge_2d();
            }
            probe_done();
            redraw = true;
          }
        }
      }
      else if (motion_tab_active == 3)
      {

        if (x >= 0 && x <= 320 && y >= 40 && y <= 65)
        {
          open_keyboard(tool_diameter, "Tool diameter (mm)", "Diameter of the tool used");
          tool_diameter = get_keyboard_result();
          redraw = true;
        }
        else if (x >= 0 && x <= 320 && y >= 65 && y <= 90)
        {
          if (spindle_is_running())
          {
            stop_spindle();
          }
          else
          {
            set_speed(wire_spindle_speed);
            start_spindle();
          }
          vTaskDelay(100);
          redraw = true;
        }
        else if(x >= 0 && x <= 320 && y >= 90 && y <= 115)
        {
          edit_max_feed( Z_AXIS );
          redraw = true;
        }
        else if(x >= 0 && x <= 320 && y >= 110 && y <= 140)
        {
          edit_max_feed_xy();
          redraw = true;
        }
        else if(x >= 0 && x <= 320 && y >= 140 && y <= 175)
        {
          edit_rapid_delay();
          redraw = true;
        }
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    else if (active_page == 8)
    {

      if (!filehandler->get_has_sd_card())
      {
        render_page(1, true);
        return;
      }

      /** settings manager page **/
      if (x >= 10 && x <= 310 && y >= 190 && y <= 230)
      {

        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 0 && x <= 320 && y >= 30 && y <= 55)
      {

        String file_name = "burn-profile";

        /** save current settings to sd card file **/
        open_keyboard_alpha(file_name, "Enter filename or press OK for autonaming", "");
        file_name = get_keyboard_result_alpha();

        if (file_name == "" || file_name.length() <= 0)
        {
        }
        else
        {

          String folder = filehandler->get_folder_settings();

          if (SD.exists(folder + "/" + file_name + ".pro"))
          {
            int i = 0;

            while (filehandler->get_file_exists(folder + "/" + file_name + "-" + String(i) + ".pro"))
            {
              ++i;
            }

            file_name = file_name + "-" + String(i);
          }

          save_settings(file_name);
          active_profile = file_name;
        }

        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 60 && y <= 85)
      {

        /** load settings from sd card file **/
        render_page_settings_sd_profiles();

        /** count the profile files (subfolders are ignored!) **/
        int num_files = filehandler->count_files_in_folder_by_extension(filehandler->get_folder_settings(), ".pro");

        char files[num_files][100];

        // load the filenames
        filehandler->get_files_in_folder_by_extension(filehandler->get_folder_settings(), ".pro", files);

        select_list_begin(num_files, files);

        String item = get_selected_item();

        if (item.length() > 0)
        {
          load_settings(item);
        }

        unselected_item();

        filehandler->close_current_folder();

        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 90 && y <= 115)
      {

        /** load settings from sd card file **/
        render_page_settings_sd_gcode();

        /** count the profile files (subfolders are ignored!) **/
        int num_files = filehandler->count_files_in_folder_by_extension(filehandler->get_folder_gcode(), ".gcode");

        char files[num_files][100];

        // load the filenames
        filehandler->get_files_in_folder_by_extension(filehandler->get_folder_gcode(), ".gcode", files);

        select_list_begin(num_files, files);

        String item = get_selected_item();

        if (item.length() > 0)
        {
          set_gcode_file(item);
        }

        unselected_item();

        filehandler->close_current_folder();

        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 130 && y <= 155)
      {

        if (get_gcode_file().length() > 0)
        {

          set_gcode_file("");
        }

        redraw = true;
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    
    while (get_touch(&x, &y)){
      if( sys.abort ){ break; }
      vTaskDelay(1);
    }

  }
  else
  {
    block_touch = false;
  }

}

String *G_EDM_UI_CONTROLLER::split(String &v, char delimiter, int &length)
{
  length = 1;
  bool found = false;
  for (int i = 0; i < v.length(); i++)
  {
    if (v[i] == delimiter)
    {
      length++;
      found = true;
    }
  }
  if (found)
  {
    String *valores = new String[length];
    int i = 0;
    for (int itemIndex = 0; itemIndex < length; itemIndex++)
    {
      for (; i < v.length(); i++)
      {

        if (v[i] == delimiter)
        {
          i++;
          break;
        }
        valores[itemIndex] += v[i];
      }
    }
    return valores;
  }
  return nullptr;
}
void G_EDM_UI_CONTROLLER::load_settings(String file_name)
{

  String file_path = filehandler->get_folder_settings() + "/" + file_name;

  if (file_path.indexOf(".pro") == -1 && file_path.indexOf(".PRO") == -1)
  {
    file_path += ".PRO";
  }
  String settings = filehandler->get_file_contents(file_path);
  if (settings.length() <= 0)
  {
    return;
  }
  int qtde;
  String *t = split(settings, ';', qtde);
  String setting;
  String value;
  for (int i = 0; i < qtde; i++)
  {
    setting = t[i].substring(0, t[i].indexOf(":"));
    value = t[i].substring(t[i].indexOf(":") + 1, t[i].length());
    update_setting(setting, value);
  }
  delete[] t;
  active_profile = file_name;
}

/**
 * All this String stuff is ugly
 * May change later to use NVS or Spiffs since SD card access is slow
 **/
void G_EDM_UI_CONTROLLER::save_settings(String file_name)
{
  String settings = "";
  settings += "1:" + String(get_freq()) + ";";
  settings += "2:" + String(get_duty_percent()) + ";";
  settings += "3:" + String(int(flush_retract_after)) + ";";
  settings += "4:" + String(flush_retract_mm) + ";";
  settings += "5:" + String(disable_spark_for_flushing ? "true" : "false") + ";";
  settings += "6:" + String(flush_offset_steps) + ";";
  settings += "7:" + String(vsense_drop_range_min) + ";";
  settings += "8:" + String(vsense_drop_range_max) + ";";
  //settings += "9:" + String(source_voltage_min) + ";";
  settings += "10:" + String(operation_mode) + ";";
  settings += "11:" + String(get_z_stop()) + ";";
  settings += "12:" + String(reamer_travel_mm) + ";";
  settings += "13:" + String(reamer_duration) + ";";
  settings += "14:" + String(z_no_home ? "true" : "false") + ";";
  settings += "15:" + String(vSense_drop_range_noload) + ";";
  settings += "16:" + String(enable_spindle ? "true" : "false") + ";";
  settings += "17:" + String(tool_diameter) + ";";
  settings += "18:" + String(simulate_gcode ? "true" : "false") + ";";
  settings += "19:" + String(pwm_frequency_probing) + ";";
  settings += "20:" + String(pwm_duty_probing) + ";";
  settings += "21:" + String(max_feeds[Z_AXIS],3) + ";";
  settings += "22:" + String(wire_spindle_speed) + ";";
  settings += "23:" + String(single_axis_mode_direction) + ";";
  settings += "24:" + String(i2sconf.I2S_full_range) + ";";
  settings += "25:" + String( gscope.get_zoom() ) + ";";
  settings += "26:" + String( get_avg( true ) ) + ";";
  settings += "27:" + String( get_avg( false ) ) + ";";
  settings += "28:" + String( get_avg_default() ) + ";";
  settings += "29:" + String( retconf.hard_retraction, 3 ) + ";";
  settings += "30:" + String( retconf.soft_retraction, 3 ) + ";";
  settings += "31:" + String( adc_critical_conf.edge_treshhold ) + ";";
  settings += "32:" + String( process_speeds_mm_min.WIRE_RETRACT_HARD, 3 ) + ";";
  settings += "33:" + String( process_speeds_mm_min.WIRE_RETRACT_SOFT, 3 ) + ";";
  settings += "34:" + String( get_avg_i2s() ) + ";";
  settings += "35:" + String( adc_critical_conf.short_circuit_max_duration_ms ) + ";";
  settings += "36:" + String( planner.get_retraction_pwm_off( 3 ) ? 1 : 0 ) + ";";
  settings += "38:" + String( retconf.wire_break_distance, 3 ) + ";";
  settings += "39:" + String( gpconf.drawlinear?1:0 ) + ";";
  settings += "40:" + String( adc_critical_conf.retract_confirmations ) + ";";
  settings += "41:" + String( plconfig.reset_sense_queue ) + ";";

  settings += "44:" + String( plconfig.line_to_line_confirm_counts ) + ";"; 
  settings += "45:" + String( i2sconf.I2S_pool_size ) + ";";
  settings += "46:" + String( adc_critical_conf.pwm_off_count ) + ";";
  settings += "47:" + String( plconfig.max_reverse_depth ) + ";";
  settings += "48:" + String( adc_hw_timer.use_hw_timer      ? "true" : "false") + ";";
  settings += "50:" + String( retconf.early_exit ? "true" : "false" ) + ";";
  settings += "51:" + String( adc_critical_conf.highs_in_a_row_max ) + ";";
  settings += "52:" + String( adc_critical_conf.zeros_in_a_row_max ) + ";";
  settings += "53:" + String( adc_critical_conf.zero_treshhold ) + ";";
  settings += "54:" + String( use_stop_and_go_flags ? "true" : "false") + ";";
  settings += "55:" + String( enable_dpm_rxtx ? "true" : "false") + ";";
  settings += "56:" + String( adc_critical_conf.voltage_feedback_treshhold ) + ";";
  settings += "57:" + String( plconfig.early_exit_on_plan ) + ";";
  settings += "58:" + String( adc_critical_conf.pulse_off_duration ) + ";";
  settings += "59:" + String( adc_critical_conf.zeros_jitter ) + ";";
  //settings += "60:" + String( adc_critical_conf.full_range_average_treshhold ) + ";";
  //settings += "61:" + String( adc_critical_conf.pulse_count_zero_jitter ) + ";";

  settings += "62:" + String( steps_per_mm_config.x ) + ";";
  settings += "63:" + String( steps_per_mm_config.y ) + ";";
  settings += "64:" + String( steps_per_mm_config.z ) + ";";


  String file_path = filehandler->get_folder_settings() + "/" + file_name;

  if (file_path.indexOf(".pro") == -1 && file_path.indexOf(".PRO") == -1)
  {
    file_path += ".PRO";
  }
  if (last_settings_copy == "")
  {
    last_settings_copy = settings;
  }
  if (file_name == "last_settings" && last_settings_copy == settings)
  {
    return;
  }
  last_settings_copy = settings;
  draw_sdcard_button( 1 );
  bool success = filehandler->write_file_contents(file_path, settings);
  if( ! success ){
    //last_settings_copy = "failed";
  }
  render_page_settings_menu_sd_card(filehandler->get_has_sd_card(), true);

}

void G_EDM_UI_CONTROLLER::update_setting(String setting, String value)
{
  if (setting == "1")
  {
    change_pwm_frequency(value.toInt());
  }
  else if (setting == "2")
  {
    update_duty(value.toFloat());
  }
  else if (setting == "3")
  {
    flush_retract_after = ( int64_t ) value.toInt();
  }
  else if (setting == "4")
  {
    flush_retract_mm = value.toFloat();
  }
  else if (setting == "5")
  {
    disable_spark_for_flushing = value == "true" ? true : false;
  }
  else if (setting == "6")
  {
    flush_offset_steps = value.toInt();
  }
  else if (setting == "7")
  {
    vsense_drop_range_min = value.toFloat();
    generate_setpoint_min_max_adc();
  }
  else if (setting == "8")
  {
    vsense_drop_range_max = value.toFloat();
    generate_setpoint_min_max_adc();
  }
  else if (setting == "9")
  {
    //source_voltage_min = value.toFloat();
  }
  else if (setting == "10")
  {
    set_operation_mode( value.toInt() );
  }
  else if (setting == "11")
  {
    set_z_stop(value.toFloat());
  }
  else if (setting == "12")
  {
    reamer_travel_mm = value.toFloat();
  }
  else if (setting == "13")
  {
    reamer_duration = value.toFloat();
  }
  else if (setting == "14")
  {
    z_no_home = value == "true" ? true : false;
    //(z_no_home);
  }
  else if (setting == "15")
  {
    vSense_drop_range_noload = value.toFloat();
  }
  else if (setting == "16")
  {
    enable_spindle = value == "true" ? true : false;
  }
  else if (setting == "17")
  {
    tool_diameter = value.toFloat();
  }
  else if (setting == "18")
  {
    // simulate_gcode = value == "true" ? true : false;
  }
  else if (setting == "19")
  {
    pwm_frequency_probing = value.toFloat();
  }
  else if (setting == "20")
  {
    pwm_duty_probing = value.toFloat();
  }
  else if (setting == "21")
  {
    max_feeds[Z_AXIS] = value.toFloat();
  }
  else if(setting == "22")
  {
    wire_spindle_speed = value.toInt();
  }
  else if(setting == "23")
  {
    change_sinker_axis_direction( value.toInt() );
  }
  else if(setting == "24")
  {
    i2sconf.I2S_full_range = value.toInt();
    gscope.set_full_range_size( i2sconf.I2S_full_range );
  }
  else if(setting == "25")
  {
    gscope.set_scope_resolution( value.toFloat() );
  }
  else if(setting == "26")
  {
    set_avg( true, value.toInt() );
  }
  else if(setting == "27")
  {
    set_avg( false, value.toInt() );
  }
  else if(setting == "28")
  {
    set_avg_default( value.toInt() );
  }
  else if(setting == "29")
  {
    retconf.hard_retraction = value.toFloat();
  }
  else if(setting == "30")
  {
    retconf.soft_retraction = value.toFloat();
  }
  else if(setting == "31")
  {
    adc_critical_conf.edge_treshhold = value.toInt();
  }
  else if(setting == "32")
  {
    process_speeds_mm_min.WIRE_RETRACT_HARD = value.toFloat();
    update_speeds();
  }
  else if(setting == "33")
  {
    process_speeds_mm_min.WIRE_RETRACT_SOFT = value.toFloat();
    update_speeds();
  }
  else if(setting == "34")
  {
    set_avg_i2s( value.toInt() );
  }
  else if(setting == "35")
  {
    adc_critical_conf.short_circuit_max_duration_ms = value.toInt();
  }
  else if(setting == "36")
  {
    planner.set_retraction_pwm_off( 5, ( value.toInt() == 0 ? false : true ) );
  }
  else if(setting == "37")
  {
  }
  else if(setting == "38")
  {
    retconf.wire_break_distance = value.toFloat();
  }
  else if(setting == "39")
  {
    gpconf.drawlinear = value.toInt() == 0 ? false : true;
  }
  else if(setting == "40")
  {
    adc_critical_conf.retract_confirmations = value.toInt();
  }
  else if(setting == "41")
  {
    plconfig.reset_sense_queue = value == "true" ? true : false;
  }
  else if(setting == "42")
  {
  }
  else if(setting == "43")
  {
  }
  else if(setting == "44")
  {
    plconfig.line_to_line_confirm_counts = value.toInt();
  }
  else if(setting == "45")
  {
    i2sconf.I2S_pool_size = value.toInt();
  }
  else if(setting == "46")
  {
    adc_critical_conf.pwm_off_count = value.toInt();
  }
  else if(setting == "47")
  {
    plconfig.max_reverse_depth = value.toInt();
  }
  else if(setting == "48")
  {
    adc_hw_timer.use_hw_timer = value == "true" ? true : false;
  }
  else if(setting == "49"){
  }
  else if(setting == "50"){
    retconf.early_exit = value == "true" ? true : false;
  }
  else if(setting == "51"){
    adc_critical_conf.highs_in_a_row_max = value.toInt();
  }
  else if(setting == "52"){
    adc_critical_conf.zeros_in_a_row_max = value.toInt();
  }
  else if(setting == "53"){
    adc_critical_conf.zero_treshhold = value.toInt();
  }
  else if(setting == "54"){
    use_stop_and_go_flags = value == "true" ? true : false;
  }
  else if(setting == "55"){
    enable_dpm_rxtx = value == "true" ? true : false;
  }
  else if(setting == "56"){
    adc_critical_conf.voltage_feedback_treshhold = value.toInt();
  }
  else if(setting == "57"){
    plconfig.early_exit_on_plan = value.toInt();
  }
  else if(setting == "58"){
    adc_critical_conf.pulse_off_duration = value.toInt();
  }
  else if(setting == "59"){
    adc_critical_conf.zeros_jitter = value.toInt();
  }
  else if(setting == "60"){
    //set_full_range_avg_treshhold( value.toInt() );
  }
  else if(setting == "61"){
    //adc_critical_conf.pulse_count_zero_jitter = value.toInt();
  }
  else if(setting == "62"){
    steps_per_mm_config.x = value.toInt();
    motor_manager.set_steps_per_mm( steps_per_mm_config.x, X_AXIS );
    update_speeds();
  }
  else if(setting == "63"){
    steps_per_mm_config.y = value.toInt();
    motor_manager.set_steps_per_mm( steps_per_mm_config.y, Y_AXIS );
    update_speeds();
  }
  else if(setting == "64"){
    steps_per_mm_config.z = value.toInt();
    motor_manager.set_steps_per_mm( steps_per_mm_config.z, Z_AXIS );
    update_speeds();
  }


}
int G_EDM_UI_CONTROLLER::change_sinker_axis_direction( int direction ){
    single_axis_mode_direction = direction;
    int axis = 0;
    switch (single_axis_mode_direction)
    {
        case 0:
            axis = Z_AXIS;
        break;
        case 1:
            axis = X_AXIS;
        break;
        case 2:
            axis = X_AXIS;
        break;
        case 3:
            axis = Y_AXIS;
        break;
        case 4:
            axis = Y_AXIS;
        break;
    }
    planner.set_sinker_axis( axis );
    return axis;
}





/**
 * Drill/Sinker mode
 * This mode does not use any XY motion and does not home X or Y
 * It is used to drill holes and simple z axis based sinker operations
 **/
//void IRAM_ATTR sinker_drill_task_single_axis(void *parameter)
void G_EDM_UI_CONTROLLER::sinker_drill_single_axis(){

    motor_manager.set_ignore_disable(true);

    int    sinker_axis                = change_sinker_axis_direction( single_axis_mode_direction );
    bool   sinker_axis_is_homed       = sys_axis_homed[sinker_axis];
    bool   sinker_axis_needs_homing   = true;
    bool   sinker_axis_move_negative  = true;
    float  cutting_depth              = get_z_stop();
    float* current_position           = system_get_mpos();
    float  max_travel                 = 0.0;
    float  target_depth               = 0.0;
    float  target[MAX_N_AXIS];
    float  backup_position[MAX_N_AXIS];

    memcpy( target, current_position, sizeof(current_position[0]) * N_AXIS );
    memcpy( backup_position, current_position, sizeof(current_position[0]) * N_AXIS );

    if( 
         single_axis_mode_direction == 0 // z negative
      || single_axis_mode_direction == 2 // x negative
      || single_axis_mode_direction == 3 // y negative
    ){
      sinker_axis_needs_homing = false;
    }

    if( !sinker_axis_is_homed && sinker_axis_needs_homing ){
      // axis is not homed but homing is needed
      disable_spark_generator();
      add_dialog_screen("Axis needs to be homed!");
      return;
    }

    pre_process_defaults();

    if (enable_spindle){
      set_speed(wire_spindle_speed);
      start_spindle();
    }

    pwm_on();
    probe_mode_on();
    generate_reference_voltage();

    switch (single_axis_mode_direction){
        case 0:
            probe_z(-1.0); // z axis down 
        break;
        case 1:
            probe_x(1.0); // x table to the left = positive
            sinker_axis_move_negative = false;
        break;
        case 2:
            probe_x(-1.0); // x table to the right = negative
        break;
        case 3:
            probe_y(-1.0); // y table back = negative
        break;
        case 4:
            probe_y(1.0); // y table forward = positive
            sinker_axis_move_negative = false;
        break;
    }

    probe_mode_off();
    planner.set_cutting_depth_limit(cutting_depth > 0.0 ? cutting_depth : 0.0);
    planner.configure();
    //api::push_cmd("G10 P1 L20 X0Y0\r");
    api::push_cmd("M7\r"); // send M7 to toggle drill/sinker mode this needs to be done after the probing stuff!
      
    memcpy( target, current_position, sizeof(current_position[0]) * N_AXIS );

    vTaskDelay(500);
    pwm_on();
    reset_flush_retract_timer();

    if( cutting_depth > 0.0 ){
        // this move has a target position
        if( sinker_axis_move_negative ){
            target[sinker_axis] -= cutting_depth;
            target[sinker_axis] = MAX( target[sinker_axis], glimits.limitsMinPosition(sinker_axis) );
        } else {
          target[sinker_axis] += cutting_depth;
          target[sinker_axis] = MIN( target[sinker_axis], glimits.limitsMaxPosition(sinker_axis) );
        }
    } else {
      // no cutting limit set just use the max possible travel
      if( sinker_axis_move_negative ){
        target[sinker_axis] = glimits.limitsMinPosition(sinker_axis);
      } else {
        target[sinker_axis] = glimits.limitsMaxPosition(sinker_axis);
      }
    }
    // the sinker uses a history line and needs somewhere to retract
    // lets push the retration target to the history planner
    // if the axis needs to move one step back in history
    // it will use this position as target
    backup_position[sinker_axis] = glimits.limitsMaxPosition(sinker_axis);
    planner.push_break_to_position_history();
    int32_t __target[MAX_N_AXIS];
    planner.convert_target_to_steps( backup_position, __target );
    planner.push_to_position_history( __target, false, 0 );
    planner.push_current_mpos_to_position_history();
    planner.position_history_force_sync();
    // push the motion command to the input buffer
    char command_buf[40];
    float position = target[sinker_axis];
    sprintf( command_buf, "G90 G53 %c%.5f F30\r", get_axis_name(sinker_axis), position ); 
    api::push_cmd(command_buf);
}



int in_process_page = 1;
int max_pages       = 10;

void IRAM_ATTR G_EDM_UI_CONTROLLER::draw_scope_settings(){
}

void IRAM_ATTR G_EDM_UI_CONTROLLER::open_scope_settings(){
    open_in_process_settings();
}

void G_EDM_UI_CONTROLLER::draw_on_off_button( int x, int y, int w, int h, bool is_active ){
  int wh = round(w/2);
  if( is_active ){
    tft.fillRect(x,   y, wh, h, TFT_GREEN);   vTaskDelay(1);
    tft.fillRect(x+wh, y, wh, h, TFT_DARKGREY);vTaskDelay(1);
  } else {
    tft.fillRect(x,   y, wh, h, TFT_DARKGREY);vTaskDelay(1);
    tft.fillRect(x+wh, y, wh, h, TFT_GREEN);   vTaskDelay(1);
  }
  tft.setTextColor(TFT_BLACK);
  tft.drawString( "ON" , x+9,    y+2, 2 );vTaskDelay(1);
  tft.drawString( "OFF", x+9+wh, y+2, 2 );vTaskDelay(1);
}

void IRAM_ATTR G_EDM_UI_CONTROLLER::draw_in_process_settings(){
    fill_screen(TFT_BLACK);vTaskDelay(1);
    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREEN);
    if( in_process_page == 1 ){
        tft.drawString("Frequency",    5, 10,  2);vTaskDelay(1);
        tft.drawString("Duty",         5, 40,  2);vTaskDelay(1);
        tft.drawString("Max Speed",    5, 70,  2);vTaskDelay(1);
        tft.drawString("Setpoint Min", 5, 100, 2);vTaskDelay(1);
        tft.drawString("Setpoint Max", 5, 130, 2);vTaskDelay(1);
        tft.drawString("Spindle RPM",  5, 160, 2);vTaskDelay(1);
    } else if( in_process_page == 2 ){
        tft.drawString("Draw linear", 5,  10, 2);vTaskDelay(1);
        tft.drawString("Shortduration (us)", 5, 40, 2);vTaskDelay(1);
        tft.drawString("Brokenwire (mm)", 5, 70, 2);vTaskDelay(1);
    } else if( in_process_page == 3 ){
        tft.drawString("eTreshhold",    5,  10, 2);vTaskDelay(1);
        tft.drawString("Zoom",          5,  40, 2);vTaskDelay(1);
        tft.drawString("I2S samples",   5,  70, 2);vTaskDelay(1);
        tft.drawString("I2S rate kSps", 5,  100, 2);vTaskDelay(1);
        tft.drawString("I2S Buffer",    5,  130, 2);vTaskDelay(1);
        tft.drawString("Length",        110, 130, 2);vTaskDelay(1);
        tft.drawString("Count",         210, 130, 2);vTaskDelay(1);
        tft.drawString("Averaging",     5,  160, 2);vTaskDelay(1);
        tft.drawString("Rise",          110, 160, 2);vTaskDelay(1);
        tft.drawString("Main",          210, 160, 2);vTaskDelay(1);
    } else if( in_process_page == 4 ){
        tft.drawString("Early retract exit", 5, 10, 2);vTaskDelay(1);
        tft.drawString("Retractconfirm",     5, 40, 2);vTaskDelay(1);
        tft.drawString("Max Reverse Depth", 5, 70, 2);vTaskDelay(1);
        tft.drawString("Retract Hard",    5, 110, 2);vTaskDelay(1);
        tft.drawString("Retract Soft",    5, 140, 2);vTaskDelay(1);
        tft.drawString("@",               183, 110, 2);vTaskDelay(1);
        tft.drawString("@",               183, 140, 2);vTaskDelay(1);
    } else if( in_process_page == 5 ){
        tft.drawString("HW Timer ADC", 5, 10, 2);vTaskDelay(1);
        tft.drawString("SampleBestOf", 5, 40, 2);vTaskDelay(1);
        tft.drawString("FullRangeAVG size", 5, 70, 2);vTaskDelay(1);
        tft.drawString("vDrop treshhold",    5, 100, 2);vTaskDelay(1);
        //tft.drawString("Pulse count jitter", 5, 130, 2);vTaskDelay(1);
    } else if( in_process_page == 6 ){
      tft.drawString("Hard short circuits", 5, 10, 2);vTaskDelay(1);
      tft.drawString("Pulse off duration", 5, 40, 2);vTaskDelay(1);
      tft.drawString("Early exit on", 5, 70, 2 );vTaskDelay(1);
      tft.drawString("Line end confirms", 5, 100, 2 );vTaskDelay(1);
      tft.drawString("ResetSenseQueue", 5, 130, 2 );vTaskDelay(1);
      tft.drawString("PWM Off after", 5, 160, 2 );vTaskDelay(1);
    } else if( in_process_page == 7 ){
        tft.drawString("Hold/Go signals",   5, 10, 2);vTaskDelay(1);
        tft.drawString("Zeros Jitter", 5, 40, 2);vTaskDelay(1);
        tft.drawString("Readings High",  5, 70, 2);vTaskDelay(1);
        tft.drawString("Readings Low",   5, 100, 2);vTaskDelay(1);
        tft.drawString("Zero treshhold", 5, 130, 2);vTaskDelay(1);
        //tft.drawString("High count at", 5, 160, 2);vTaskDelay(1);
    } else if( in_process_page == 8 ){
        tft.drawString("DPM Settings", 5, 10, 2);vTaskDelay(1);
        tft.drawString("Enable RXTX",  5, 40, 2);vTaskDelay(1);
        if( enable_dpm_rxtx ){
            tft.drawString("Power",        5, 70, 2);vTaskDelay(1);
            tft.drawString("Voltage",      5, 100, 2);vTaskDelay(1);
            tft.drawString("Current",      5, 130, 2);vTaskDelay(1);
        } else {
            tft.drawString("DPM not available", 5, 70, 2);vTaskDelay(1);
        }
    } else if( in_process_page == 9 ){
        tft.drawString("Display & UI",   5, 10, 2);vTaskDelay(1);
        tft.drawString("Calibrate Touch", 5, 40, 2);vTaskDelay(1);
    } else if( in_process_page == 10 ){
        tft.drawString("Motor Settings",   5, 10, 2);vTaskDelay(1);
        tft.drawString("X steps per mm",   5, 40, 2);vTaskDelay(1);
        tft.drawString("Y steps per mm",   5, 70, 2);vTaskDelay(1);
        tft.drawString("Z steps per mm",   5, 100, 2);vTaskDelay(1);
        tft.fillRect(5, 160, 310, 30, TFT_RED);   vTaskDelay(1);
        tft.setTextColor(TFT_LIGHTGREY);
        tft.drawString("Restart ESP now",   65, 166, 2);vTaskDelay(1);
    }




    tft.setTextColor(TFT_GREEN);

    if( in_process_page == 1 ){
        tft.drawString( String(convert_frequency_units(get_freq())),  100, 10,   2);vTaskDelay(1);
        tft.drawString( String(get_duty_percent()) + "%",             100, 40,   2);vTaskDelay(1);
        tft.drawString( String(max_feeds[Z_AXIS],2) + "mm/min",       100, 70,   2);vTaskDelay(1);
        tft.drawString( String(vsense_drop_range_min) + "%",          100, 100,  2);vTaskDelay(1);
        tft.drawString( String(vsense_drop_range_max) + "%",          100, 130,  2);vTaskDelay(1);
        tft.drawString( String(vsense_drop_range_max) + "%",          100, 130,  2);vTaskDelay(1);
        tft.drawFloat( frequency_to_rpm( wire_spindle_speed ), 2, 100, 160,  2 );vTaskDelay(1);
        draw_on_off_button( 230, 160, 80, 20, enable_spindle );
    } else if( in_process_page == 2 ){
        draw_on_off_button( 230, 10, 80, 20, ( gpconf.drawlinear ? true : false ) );
        tft.setTextColor(TFT_GREEN);
        tft.drawNumber( round(adc_critical_conf.short_circuit_max_duration_ms/1000), 140, 40, 2);vTaskDelay(1);
        tft.drawFloat(  retconf.wire_break_distance, 2,                              140, 70, 2);vTaskDelay(1);
    } else if( in_process_page == 3 ){
        tft.drawNumber( adc_critical_conf.edge_treshhold,        140, 10, 2 );vTaskDelay(1);
        tft.drawFloat( gscope.get_zoom(),1,                      140, 40, 2);vTaskDelay(1);
        tft.drawNumber( get_avg_i2s(),                           140, 70, 2);vTaskDelay(1);
        tft.drawNumber( round( i2sconf.I2S_sample_rate / 1000 ), 140, 100, 2);vTaskDelay(1);
        tft.drawNumber( int( i2sconf.I2S_buff_len ),             160, 130, 2 );vTaskDelay(1);
        tft.drawNumber( int( i2sconf.I2S_buff_count ),           260, 130, 2 );vTaskDelay(1);
        tft.drawNumber( get_avg( true ),                         160, 160, 2);vTaskDelay(1);
        tft.drawNumber( get_avg_default(),                       260, 160, 2);vTaskDelay(1);
    } else if( in_process_page == 4 ){
        draw_on_off_button( 230, 10, 80, 20, ( retconf.early_exit ? true : false ) );
        tft.setTextColor(TFT_GREEN);
        tft.drawNumber( adc_critical_conf.retract_confirmations, 180, 40, 2);vTaskDelay(1);
        tft.drawNumber( plconfig.max_reverse_depth, 180, 70, 2);vTaskDelay(1);
        tft.drawString( String( retconf.hard_retraction, 3 )+"mm", 120, 110, 2);vTaskDelay(1);
        tft.drawString( String( process_speeds_mm_min.WIRE_RETRACT_HARD, 3 )+"mm/min", 205, 110, 2);vTaskDelay(1);
        tft.drawString( String( retconf.soft_retraction, 3 )+"mm", 120, 140, 2);vTaskDelay(1);
        tft.drawString( String( process_speeds_mm_min.WIRE_RETRACT_SOFT, 3 )+"mm/min", 205, 140, 2);vTaskDelay(1);
    } else if( in_process_page == 5 ){
        draw_on_off_button( 230, 10, 80, 20, ( adc_hw_timer.use_hw_timer ? true : false ) );
        tft.setTextColor(TFT_GREEN);
        tft.drawNumber( i2sconf.I2S_pool_size, 180, 40, 2);vTaskDelay(1);
        tft.drawNumber( i2sconf.I2S_full_range, 180, 70, 2);vTaskDelay(1);
        float tresh_percent = 100.0/float(VSENSE_RESOLUTION)*float(adc_critical_conf.voltage_feedback_treshhold);
        tft.drawString( String( tresh_percent, 2 ) + "% / " + String(adc_critical_conf.voltage_feedback_treshhold), 180, 100, 2);vTaskDelay(1);
        //tft.drawNumber( adc_critical_conf.pulse_count_zero_jitter, 180, 130, 2 );vTaskDelay(1);
    } else if( in_process_page == 6 ){
        draw_on_off_button( 230, 130, 80, 20, ( plconfig.reset_sense_queue ? true : false ) );
        tft.setTextColor(TFT_GREEN);
        tft.drawString( String( adc_critical_conf.pulse_off_duration ), 180, 40, 2);vTaskDelay(1);
        tft.drawNumber( plconfig.early_exit_on_plan, 180, 70, 2 );vTaskDelay(1);
        tft.drawNumber( plconfig.line_to_line_confirm_counts, 180, 100, 2 );vTaskDelay(1);
        tft.drawNumber( adc_critical_conf.pwm_off_count, 180, 160, 2 );vTaskDelay(1);
    } else if( in_process_page == 7 ){
        draw_on_off_button( 230, 10, 80, 20, ( use_stop_and_go_flags ? true : false ) );
        tft.setTextColor(TFT_GREEN);
        tft.drawString( String( adc_critical_conf.zeros_jitter ),       260, 40, 2);vTaskDelay(1);
        tft.drawString( String( adc_critical_conf.highs_in_a_row_max ), 260, 70, 2);vTaskDelay(1);
        tft.drawString( String( adc_critical_conf.zeros_in_a_row_max ), 260, 100, 2);vTaskDelay(1);
        tft.drawString( String( adc_critical_conf.zero_treshhold ), 260, 130, 2);vTaskDelay(1);
        //tft.drawString( String( adc_critical_conf.high_plan_at==3?"Upper SetP":"Above SetP" ), 220, 160, 2);vTaskDelay(1);
    } else if( in_process_page == 8 ){
        // get DPM data
        draw_on_off_button( 230, 40, 80, 20, ( enable_dpm_rxtx ? true : false ) );
        if( enable_dpm_rxtx ){
            bool dpm_is_on = dpm_driver.get_power_is_on();
            draw_on_off_button( 230, 70, 80, 20, ( dpm_is_on ? true : false ) );
            tft.setTextColor(TFT_GREEN);
            int voltage, current;
            dpm_driver.get_setting_voltage_and_current( voltage, current ); // return values are in mV/mA
            float v, c;
            v = float(voltage);
            c = float(current);
            if( voltage > -1 ){
              v /= 1000.0;
            }
            if( current > -1 ){
              c /= 1000.0;
            }
            tft.drawString( String(v)+"V", 140, 100, 2);vTaskDelay(1);
            tft.drawString( String(c)+"A", 140, 130, 2);vTaskDelay(1);
        }
    } else if( in_process_page == 9 ){
      tft.fillRect(230, 40, 80, 20, TFT_GREEN);   vTaskDelay(1);
      tft.setTextColor(TFT_BLACK);
      tft.drawString( "Start" , 239, 42, 2 );vTaskDelay(1);
    } else if( in_process_page == 10 ){
        tft.drawString("Motor Settings",   5, 10, 2);vTaskDelay(1);
        tft.drawNumber(int( steps_per_mm_config.x ),   260, 40, 2);vTaskDelay(1);
        tft.drawNumber(int( steps_per_mm_config.y ),   260, 70, 2);vTaskDelay(1);
        tft.drawNumber(int( steps_per_mm_config.z ),   260, 100, 2);vTaskDelay(1);
    }
    tft.setTextColor(TFT_BLACK);
    tft.fillRect( 200, 210, 60, 30, ( in_process_page==1 ? TFT_DARKGREY : TFT_OLIVE ) );vTaskDelay(1);
    tft.fillRect( 260, 210, 60, 30, ( in_process_page==max_pages ? TFT_DARKGREY : TFT_OLIVE ) );vTaskDelay(1);
    tft.drawString( "<<<" , 220, 216, 2);vTaskDelay(1);
    tft.drawString( ">>>" , 280, 216, 2);vTaskDelay(1);
    tft.fillRect(0, 210, 200, 30, TFT_OLIVE);vTaskDelay(1);
    tft.drawString("Done", 80, 216, 2);vTaskDelay(1);
    tft.drawFastVLine( 200, 210, 30, TFT_BLACK );
    tft.drawFastVLine( 260, 210, 30, TFT_BLACK );
    uint16_t x, y;
    while( get_touch(&x, &y) ){
      vTaskDelay(10);
    }
}



void G_EDM_UI_CONTROLLER::monitor_settings_touch(){
    bool i2s_changed = false;
    uint16_t x, y;
    uint16_t xx, yy;
    while (get_touch(&x, &y)){
      if( sys.abort ){ break; }
      vTaskDelay(1);
    }
    while(true){
        if( sys.abort ){ break; }
        if(get_touch(&x, &y)){
            vTaskDelay(1);

            while( get_touch(&xx, &yy) ){
                vTaskDelay(10);
            }



            //###################################################
            // Navigation and done buttons first
            //###################################################
            if( x >= 200 && x <= 260 && y >= 210 && y <= 240 ){
                if( in_process_page == 1 ){
                  in_process_page = max_pages;
                } else {
                  --in_process_page;
                }
                draw_in_process_settings();
                continue;
            } else if( x >= 260 && x <= 320 && y >= 210 && y <= 240 ){
                if( in_process_page == max_pages ){
                  in_process_page = 1;
                } else {
                  ++in_process_page;
                }
                draw_in_process_settings();
                continue;
            } else if( x >= 0 && x <= 200 && y >= 210 && y <= 240 ){
                //in_process_page = 1;
                break;
            } 



            if( in_process_page == 1 ){

                if( x >= 0 && x <= 220 && y >= 5 && y <= 35 ){
                    /** change PWM frequency **/
                    edit_pwm_frequency();
                    draw_in_process_settings();
                    continue;
                } 
                else if( x >= 0 && x <= 220 && y >= 35 && y <= 65 ){
                    /** change PWM duty **/
                    edit_pwm_duty();
                    draw_in_process_settings();
                    continue;
                } 
                else if( x >= 0 && x <= 220 && y >= 65 && y <= 95 ){
                    // changing feedrate
                    edit_max_feed( Z_AXIS );
                    draw_in_process_settings();
                    continue;
                } 
                else if( x >= 0 && x <= 220 && y >= 95 && y <= 125 ){
                    /** change setpoint min **/
                    edit_setpoint_min();
                    draw_in_process_settings();
                    continue;
                } 
                else if( x >= 0 && x <= 220 && y >= 125 && y <= 155 ){
                    /** change setpoint min **/
                    edit_setpoint_max();
                    draw_in_process_settings();
                    continue;
                } 
                else if( x >= 230 && x <= 310 && y >= 160 && y <= 180 ){
                    enable_spindle=!enable_spindle;
                    draw_in_process_settings();
                    continue;
                 } 
                else if( x >= 0 && x <= 220 && y >= 160 && y <= 190 ){
                    change_wire_spindle_speed();
                    draw_in_process_settings();
                    continue;
                } 

            } else if( in_process_page == 2 ){

                if( x >= 230 && x <= 310 && y >= 0 && y <= 40 ){
                  gpconf.drawlinear = !gpconf.drawlinear;
                  draw_in_process_settings();
                  continue;
                } else if( x >= 0 && x <= 240 && y >= 35 && y <= 65 ){
                    open_keyboard( float( adc_critical_conf.short_circuit_max_duration_ms/1000 ), "Short circuit max duration (ms)", "Max time in milliseconds above setpoint\runtil a short circuit pause. ");
                    int result = round( get_keyboard_result() );
                    if( result < 10 ){
                        result = 10;
                    }
                    adc_critical_conf.short_circuit_max_duration_ms = result*1000;
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 240 && y >= 65 && y <= 95 ){
                    open_keyboard( retconf.wire_break_distance, "Broken Wire distance (mm)", "Max travel in mm without load until \ra broken wire pause is inserted. ");
                    retconf.wire_break_distance = get_keyboard_result();
                    planner.set_retraction_steps();
                    draw_in_process_settings();
                    continue;
                }

            } else if( in_process_page == 3 ){

                if( x >= 0 && x <= 240 && y >= 0 && y <= 40 ){
                    open_keyboard( adc_critical_conf.edge_treshhold, "Edge Treshhold", "ADC value needed to deside if it is a rising/falling edge\rThe fresh reading is compared against the old reading +-treshhold");
                    int result = round( get_keyboard_result() );
                    if( result < 1 ){
                      result = 1;
                    } else if( result > VSENSE_RESOLUTION ){
                      result = VSENSE_RESOLUTION;
                    }
                    adc_critical_conf.edge_treshhold = result;
                    draw_in_process_settings();
                    continue;

                } else if( x >= 0 && x <= 240 && y >= 40 && y <= 70 ){
                    open_keyboard( gscope.get_zoom(), "Scope zoom", "At higher zoom the scope may not fit\rthe full amplitude of the signal");
                    float new_zoom = get_keyboard_result();
                    gscope.set_scope_resolution( new_zoom );
                    draw_in_process_settings();
                    continue;

                } else if( x >= 0 && x <= 240 && y >= 70 && y <= 100 ){
                    open_keyboard( get_avg_i2s(), "I2S samples", "Number of samples used from the I2S buffer.\rI2S batchsize is defined by I2S buffer length.");
                    int samples = int( get_keyboard_result() ); 
                    set_avg_i2s( samples );
                    draw_in_process_settings();
                    continue;

                } else if( x >= 0 && x <= 240 && y >= 100 && y <= 130 ){
                    open_keyboard( round( i2sconf.I2S_sample_rate / 1000 ), "Samplerate (kSps)", "I2S sample rate\rWARNING: Wrong settings can break I2S.");
                    int result = round( get_keyboard_result() * 1000.0 );
                    if( result > 5000000 ){
                      result = 5000000;
                    } else if( result < 10000 ){
                      result = 10000;
                    }
                    i2sconf.I2S_sample_rate =  result;
                    i2s_changed = true;
                    draw_in_process_settings();
                    continue;

                } else if( x >= 110 && x <= 210 && y >= 130 && y <= 160 ){
                    open_keyboard( round( i2sconf.I2S_buff_len ), "Buff_len", "I2S Buffer length\rWARNING: Wrong settings can break I2S.");
                    int result = round( get_keyboard_result() );
                    if( result > 1000 ){
                      result = 1000;
                    } else if( result < 10 ){
                      result = 10;
                    }
                    i2sconf.I2S_buff_len =  result;
                    i2s_changed = true;
                    draw_in_process_settings();
                    continue;

                } else if( x >= 210 && x <= 320 && y >= 130 && y <= 160 ){
                    open_keyboard( round( i2sconf.I2S_buff_count ), "Buff_count", "I2S Buffer count\rWARNING: Wrong settings can break I2S.");
                    int result = round( get_keyboard_result() );
                    if( result > 100 ){
                      result = 100;
                    } else if( result < 2 ){
                      result = 2;
                    }
                    i2sconf.I2S_buff_count =  result;

                    i2s_changed = true;
                    draw_in_process_settings();
                    continue;


                } else if( x >= 110 && x <= 210 && y >= 160 && y <= 190 ){
                    open_keyboard( get_avg( true ), "Rising edge gain factor", "A high value will increase the rising edge");
                    set_avg( true, int( get_keyboard_result() ) );
                    draw_in_process_settings();
                    continue;

                } else if( x >= 210 && x <= 320 && y >= 160 && y <= 190 ){
                    open_keyboard( get_avg_default(), "Default average", "Number of buffer samples used to smoothen \rthe average wave");
                    int samples = int( get_keyboard_result() );
                    if( samples < 1 ){
                        samples = 1;
                    } else if( samples > vsense_sampler_buffer_size-1 ){
                        samples = vsense_sampler_buffer_size-1;
                    }
                    set_avg_default( samples );
                    draw_in_process_settings();
                    continue;
                } 


            } else if( in_process_page == 4 ){
                if( x >= 230 && x <= 310 && y >= 0 && y <= 35 ){
                    retconf.early_exit = !retconf.early_exit;
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 320 && y >= 35 && y <= 65 ){
                    open_keyboard( float( adc_critical_conf.retract_confirmations ), "Retract confirmations", "Confirm a retraction x times before\ractually doing the move. ");
                    int result = round( get_keyboard_result() );
                    if( result < 1 ){
                        result = 1;
                    }
                    adc_critical_conf.retract_confirmations = result;
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 240 && y >= 65 && y <= 95 ){
                    open_keyboard( plconfig.max_reverse_depth, "Maximum lines to retract", "Defines the maximum allowed lines\rto move back in history.");
                    int result = round( get_keyboard_result() );
                    if( result < 1 ){
                      result = 1;
                    } 
                    plconfig.max_reverse_depth = result;
                    draw_in_process_settings();
                    continue;
                } else if( x >= 120 && x <= 183 && y >= 105 && y <= 135 ){
                    change_retraction_distance( 1 );
                    draw_in_process_settings();
                    continue;
                } else if( x >= 120 && x <= 183 && y >= 135 && y <= 165 ){
                    change_retraction_distance( 2 );
                    draw_in_process_settings();
                    continue;
                } else if( x >= 205 && x <= 320 && y >= 105 && y <= 135 ){
                    change_retraction_speed( 1 );
                    draw_in_process_settings();
                    continue;
                } else if( x >= 205 && x <= 320 && y >= 135 && y <= 165 ){
                    change_retraction_speed( 2 );
                    draw_in_process_settings();
                    continue;
                } 

            } else if( in_process_page == 5 ){

                if(  x >= 230 && x <= 310 && y >= 5 && y <= 35 ){
                  adc_hw_timer.use_hw_timer = !adc_hw_timer.use_hw_timer;
                  if( !adc_hw_timer.use_hw_timer ){
                        if( sensor_queue_main != NULL ){
                          int data = 11;
                          xQueueSend( sensor_queue_main, &data, 50000 );
                        }
                  } else {
                    if( !ui_controller.get_pwm_is_off() ){
                        if( sensor_queue_main != NULL ){
                          int data = 10;
                          xQueueSend( sensor_queue_main, &data, 50000 );
                        }
                    }
                  }
                  draw_in_process_settings();
                  continue;
                } else if( x >= 0 && x <= 320 && y >= 35 && y <= 65 ){
                    open_keyboard( float( i2sconf.I2S_pool_size ), "Sample pool size", "The number of samples added to the pool\rto choose from.");
                    int result = round( get_keyboard_result() );
                    if( result < 1 ){
                        result = 1;
                    }
                    i2sconf.I2S_pool_size = result;
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 320 && y >= 65 && y <= 95 ){
                    open_keyboard( float( i2sconf.I2S_full_range ), "Full-Range-Average samples", "Number of samples used to create the\rfull range (slow) average.");
                    int result = round( get_keyboard_result() );
                    if( result < 1 ){
                        result = 1;
                    } else if( result > vsense_sampler_buffer_size-1 ){
                      result = vsense_sampler_buffer_size-1;
                    }
                    i2sconf.I2S_full_range = result;
                    gscope.set_full_range_size( result );
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 320 && y >= 95 && y <= 125 ){
                    float tresh_percent = 100.0/float(VSENSE_RESOLUTION)*float(adc_critical_conf.voltage_feedback_treshhold);
                    open_keyboard( tresh_percent, "Voltage drop treshhold (%)", "If the voltage drops below this\rit is considered a hard short circuit.");
                    float new_tresh = get_keyboard_result();
                    new_tresh = round( float(VSENSE_RESOLUTION)/100.0*new_tresh );
                    adc_critical_conf.voltage_feedback_treshhold = new_tresh;
                    gscope.set_vdrop_treshhold( adc_critical_conf.voltage_feedback_treshhold );
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 320 && y >= 125 && y <= 155 ){
                    /*open_keyboard( float( adc_critical_conf.pulse_count_zero_jitter ), "ADC rock bottom jitter", "To prevent tiny noise spikes be counted\ras a pulse define the rock bottom ADC.");
                    int result = round( get_keyboard_result() );
                    adc_critical_conf.pulse_count_zero_jitter = result;
                    draw_in_process_settings();
                    continue;*/
                }

            }  else if( in_process_page == 6 ){

                if(  x >= 0 && x <= 310 && y >= 35 && y <= 65 ){
                    open_keyboard( float( adc_critical_conf.pulse_off_duration ), "Pulse off duration (Pulses)", "Turn PWM off on shorts for given pulses\rRelative to frequency: (Pulses*Pulseperiod)");
                    int result = round( get_keyboard_result() );
                    adc_critical_conf.pulse_off_duration = result;
                    draw_in_process_settings();
                    continue;
                } else if(  x >= 0 && x <= 310 && y >= 65 && y <= 95 ){
                    if(++plconfig.early_exit_on_plan>4){
                      plconfig.early_exit_on_plan = 1;
                    }
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 310 && y >= 95 && y <= 125 ){
                  open_keyboard( float( plconfig.line_to_line_confirm_counts ), "Line end confirmations", "Required low load readings before\rmoving to the next line.");
                  int result = round( get_keyboard_result() );
                  plconfig.line_to_line_confirm_counts = result;
                  draw_in_process_settings();
                  continue;
                } else if( x >= 230 && x <= 310 && y >= 125 && y <= 155 ){
                  plconfig.reset_sense_queue = !plconfig.reset_sense_queue;
                  draw_in_process_settings();
                  continue;
                } else if( x >= 0 && x <= 310 && y >= 155 && y <= 185 ){
                  open_keyboard( float( adc_critical_conf.pwm_off_count ), "PWM Off after x digital lows", "Toggle PWM off after X digital lows\rHigher value = less jitter.");
                  int result = round( get_keyboard_result() );
                  adc_critical_conf.pwm_off_count = result;
                  draw_in_process_settings();
                  continue;
                }

            } else if( in_process_page == 7 ){
                if(  x >= 230 && x <= 310 && y >= 0 && y <= 35 ){
                    use_stop_and_go_flags = !use_stop_and_go_flags;
                    draw_in_process_settings();
                    continue;
                } if(  x >= 230 && x <= 310 && y >= 35 && y <= 65 ){
                    open_keyboard( float( adc_critical_conf.zeros_jitter ), "Off time zero jitter", "Number of zero reading needed to reset\rthe high counter.");
                    int result = round( get_keyboard_result() );
                    if( result < 0 ){ result = 0; }
                    adc_critical_conf.zeros_jitter = result;
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 320 && y >= 65 && y <= 95 ){
                    open_keyboard( float( adc_critical_conf.highs_in_a_row_max ), "High readings in a row alert", "Alert signal after given\rnumber of highs in a row.");
                    int result = round( get_keyboard_result() );
                    if( result < 1 ){ result = 1; }
                    adc_critical_conf.highs_in_a_row_max = result;
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 320 && y >= 95 && y <= 135 ){
                    open_keyboard( float( adc_critical_conf.zeros_in_a_row_max ), "Low readings in a row alert", "Alert signal after given\rnumber of lows in a row.");
                    int result = round( get_keyboard_result() );
                    if( result < 1 ){
                        result = 1;
                    }
                    adc_critical_conf.zeros_in_a_row_max = result;
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 320 && y >= 135 && y <= 165 ){
                    open_keyboard( float( adc_critical_conf.zero_treshhold ), "Zero treshhold", "Readings below this are\rconsidered empty.");
                    int result = round( get_keyboard_result() );
                    adc_critical_conf.zero_treshhold = result;
                    draw_in_process_settings();
                    continue;
                } else if( x >= 0 && x <= 320 && y >= 165 && y <= 195 ){
                    /*if( adc_critical_conf.high_plan_at == 4){
                      adc_critical_conf.high_plan_at = 3;
                    } else {
                      adc_critical_conf.high_plan_at = 4;
                    }
                    draw_in_process_settings();
                    continue;*/
                }

            } else if( in_process_page == 8 ){

              if(  x >= 230 && x <= 310 && y >= 35 && y <= 65 ){
                  enable_dpm_rxtx = !enable_dpm_rxtx;
                  draw_in_process_settings();
                  continue;
              }


              if( enable_dpm_rxtx ){
                  if( x >= 230 && x <= 310 && y >= 65 && y <= 95 ){
                      bool dpm_is_on = dpm_driver.get_power_is_on();
                      int turn_on = dpm_is_on ? 0 : 1;
                      dpm_driver.power_on_off( turn_on, 10 );
                      draw_in_process_settings();
                      continue;
                  } else if( x >= 0 && x <= 240 && y >= 95 && y <= 125 ){
                    int voltage;
                    dpm_driver.get_setting_voltage( voltage ); // result is in mV
                    float v = float( voltage ) / 1000.0;
                    open_keyboard( v, "DPM voltage", "Set the voltage\ron the DPM.");
                    float result = get_keyboard_result();
                    dpm_driver.set_setting_voltage( result );
                    draw_in_process_settings();
                    continue;
                  } else if( x >= 0 && x <= 240 && y >= 125 && y <= 155 ){
                    int current;
                    dpm_driver.get_setting_current( current ); // result is in mA
                    float c = float( current ) / 1000.0;
                    open_keyboard( c, "DPM current", "Set the current\ron the DPM.");
                    float result = get_keyboard_result();
                    dpm_driver.set_setting_current( result );
                    draw_in_process_settings();
                    continue;
                  }
              }
            } else if( in_process_page == 9 ){
                if( x >= 230 && x <= 320 && y >= 35 && y <= 65 ){
                  if( !edm_process_is_running ){
                      touch_calibrate( true );
                      Serial.end();
                      sys.abort = true;
                      ESP.restart();
                      while( true ){
                          vTaskDelay(10);
                      }
                  }
                }
            } else if( in_process_page == 10 ){
              if( x >= 230 && x <= 320 && y >= 35 && y <= 65 ){
                open_keyboard( float( steps_per_mm_config.x ), "X axis steps per mm", "RESTART REQUIRED AFTER CHANGE!\r");
                int result = round( get_keyboard_result() );
                steps_per_mm_config.x = result;
                save_settings("last_settings");
                draw_in_process_settings();
                continue;
              } else if( x >= 230 && x <= 320 && y >= 65 && y <= 95 ){
                open_keyboard( float( steps_per_mm_config.y ), "Y axis steps per mm", "RESTART REQUIRED AFTER CHANGE!\r");
                int result = round( get_keyboard_result() );
                steps_per_mm_config.y = result;
                save_settings("last_settings");
                draw_in_process_settings();
                continue;
              } else if( x >= 230 && x <= 320 && y >= 95 && y <= 125 ){
                open_keyboard( float( steps_per_mm_config.z ), "Z axis steps per mm", "RESTART REQUIRED AFTER CHANGE!\r");
                int result = round( get_keyboard_result() );
                steps_per_mm_config.z = result;
                save_settings("last_settings");
                draw_in_process_settings();
                continue;
              } else if( x >= 0 && x <= 320 && y >= 160 && y <= 190 ){
                Serial.end();
                sys.abort = true;
                ESP.restart();
                while( true ){
                  vTaskDelay(10);
                }
              }





            }

        }
    }

    i2sconf.block_write_out = false;
    if( i2s_changed ){
      i2sconf.I2S_FORCE_RESTART = true;
    }

    force_redraw = true;
    vTaskDelay(10);

}


void IRAM_ATTR G_EDM_UI_CONTROLLER::open_in_process_settings(){
    i2sconf.block_write_out = true;
    while( i2sconf.I2S_busy ){
      vTaskDelay(10);
    }
    draw_in_process_settings();
    monitor_settings_touch();
}


void G_EDM_UI_CONTROLLER::update_display_minimal()
{
  /** check for touch **/
  uint16_t x, y;
  bool block_touch = false;
  bool redraw      = false;
  bool is_paused   = planner.get_is_paused();
  int  rounds      = 0;
  if (!is_probing()&&get_touch(&x, &y))
  {
    if (block_touch || gconf.gedm_reprobe_motion)
    {
      /** debounce **/
      return;
    }
    #ifndef DEBUG_PROCESS_SCREEN
    if (!is_paused)
    {
      /** pause process on touch events **/
      gconf.edm_pause_motion = true;
      
      while( !planner.get_is_paused() || ++rounds>2000 ){
        gconf.edm_pause_motion = true;
        if( sys.abort || sys_rt_exec_state.bit.motionCancel || gconf.edm_process_finished ){ break; }
        vTaskDelay(1);
      }
    }
    if( rounds>= 2000){
      return;
    }
    #else
        gconf.edm_process_finished = false;
        sys_rt_exec_state.bit.motionCancel = false;
    #endif
    block_touch = true;
    bool scope_active = gscope.scope_is_running();
    if( scope_active && x > 195 && x < 320 && y > 0 && y < 20 ){
      gscope.toogle_values();
    } 
    else if( scope_active && x > 0 && x < 100 && y > 0 && y < 50 ){
      open_scope_settings();
    } 
    else if (x > 81 && x < 320 && y > 210 && y < 240)
    {
      // pause or resume process
      gconf.edm_pause_motion = is_paused ? false : true;
      process_overlay_pause_button();
    }
    else if (x > 0 && x < 80 && y > 210 && y < 240)
    {
      // pause or resume process
      open_in_process_settings();
    }
    /*else if (operation_mode == 3)
    {
      // floating z gcode process
      if (x > 250 && x < 310 && y > 100 && y < 160)
      {
        // force reprobe with the next line
        if (gconf.gedm_insert_probe)
        {
          gconf.gedm_insert_probe = false; // undo
        }
        else
        {
          gconf.gedm_insert_probe = true;
        }
        redraw = true;
      }
    } */
    
    /** block until untouched **/
    while (get_touch(&x, &y)){
      if( sys.abort || sys_rt_exec_state.bit.motionCancel || gconf.edm_process_finished ){ break; }
      vTaskDelay(1);
    }
    if (redraw||force_redraw){
      render_page(9, true);vTaskDelay(1);
    }
  } 
}






void G_EDM_UI_CONTROLLER::change_retraction_distance( int option ){

  if( option == 1 ){
        open_keyboard(retconf.hard_retraction, "Hard retraction (mm)", "Retractiondistance on short circuits \rwithout checking the ADC feedback.");
        float result = get_keyboard_result();
        if( result > 5.0 ){
          result = 5.0;
        }
        retconf.hard_retraction = result;
  } else if ( option == 2 ){
        open_keyboard(retconf.soft_retraction, "Soft retraction (mm)", "Retractiondistance if in the upper setpoint range \rwithout checking the ADC feedback.");
        float result = get_keyboard_result();
        if( result > 5.0 ){
          result = 5.0;
        }
        retconf.soft_retraction = result;
  }
  planner.set_retraction_steps();
}
void G_EDM_UI_CONTROLLER::change_retraction_speed( int option ){
  if( option == 1 ){
        open_keyboard(process_speeds_mm_min.WIRE_RETRACT_HARD, "Hard retraction speed (mm/min)", "Retractionspeed if above the upper setpoint.\rSpeed is just a rough calculation.");
        float result = get_keyboard_result();
        if( result > 80.0 ){
          result = 80.0;
        }
        process_speeds_mm_min.WIRE_RETRACT_HARD = result;
        update_speeds();
  } else if ( option == 2 ){
        open_keyboard(process_speeds_mm_min.WIRE_RETRACT_SOFT, "Soft retraction speed (mm/min)", "Retractionspeed if in the upper setpoint range.\rSpeed is just a rough calculation.");
        float result = get_keyboard_result();
        if( result > 80.0 ){
          result = 80.0;
        }
        process_speeds_mm_min.WIRE_RETRACT_SOFT = result;
        update_speeds();
  }
}

void G_EDM_UI_CONTROLLER::add_dialog_screen( String text ){
    uint16_t x, y;
    while(get_touch(&x, &y)){vTaskDelay(1);}
    fill_screen(TFT_BLACK);vTaskDelay(1);
    tft.setTextSize(2);
    tft.setTextColor(TFT_GREENYELLOW);
    tft.drawString("Info:", 10, 50, 2);vTaskDelay(1);
    tft.setTextSize(1);
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString( text, 10, 100, 2 );vTaskDelay(1);
    tft.drawString("Touch to close message", 10, 140, 2);vTaskDelay(1);
    disable_spark_generator();
    while(!get_touch(&x, &y)){vTaskDelay(1);}
    force_redraw = true;
}

/*void G_EDM_UI_CONTROLLER::set_full_range_avg_treshhold( int tresh ){
  adc_critical_conf.full_range_average_treshhold = tresh;
  gscope.set_full_range_avg_treshhold( tresh );
}*/
#ifdef UNIT_TESTING

void test_dpm_communication(){
  // enable;disable
  // send/receive

  

  

}






void test_pwm(){}
void test_motors(){}
void test_display(){}
void test_sd_card(){}
void test_limits(){}
void test_on_off(){

}

#endif
