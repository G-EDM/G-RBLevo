#pragma once

#include <stdint.h>
#include "esp_attr.h"
#include "definitions_common.h"

/*
#
#  ██████        ███████ ██████  ███    ███  
# ██             ██      ██   ██ ████  ████  
# ██   ███ █████ █████   ██   ██ ██ ████ ██ 
# ██    ██       ██      ██   ██ ██  ██  ██ 
#  ██████        ███████ ██████  ██      ██ 
#
# Configuration file
#
*/   
/** 
 * there are two version of the ESP32 breakoutboard. 
 * Uncomment to fallback to V1  
 **/







#define FORWARD_CONFIRMATIONS 1
#define VSENSE_FEEDBACK_TIMER_INTERVAL_EDM 40//25//40 // 50==20khz 100==10khz
#define VSENSE_FEEDBACK_TIMER_INTERVAL_DEFAULT 1000 //1000==1khz
#define MOTION_PLAN_PROBE_CONFIRMATIONS 5 
#define MOTION_PLAN_NOISE_LEVEL_IGNORE 2
#define RETRACT_CONFIRMATIONS 2
#define MAX_ADC_READING_AGE 50//us 

#define POSITION_HISTOTY_SEGMENTATION_STEPS 200 // steps; after every x steps forward it will create a new history position object 





/**
  * 
  * Pinout
  * 
  * (The display pinout is not included here. The TFT pins are configured within the platform.io file)
  * 
  * The complete Pinout can be found in here:
  * /build-tutorial/ESP32-Pinout.txt
  * 
  * Note: vSense uses ADC Channels 
  * So changing the Pin here won't work. The ADC channel needs to be changed in the sensors.cpp file
  * in this case
  * 
  * A list with the GPIOs and the corresponding ADC channel can be found here:
  * https://github.com/espressif/esp-idf/blob/e9d442d2b72/components/soc/esp32/include/soc/adc_channel.h
  * 
  **/
#define ON_OFF_SWITCH_PIN        36
#define DIGITAL_INPUT_FEED_HOLD  35 // work in progress
#define VSENSE_FEEDBACK_PIN      34
#define GEDM_PWM_PIN             22 // PWM for the pulsegenerator

#define GEDM_A_STEP_PIN          12
#define GEDM_A_DIRECTION_PIN_NEW 17 // 16 evo change
#define GEDM_A_DIRECTION_PIN     17// 1  evo change // spindle dir pin. Note this is the TX1 Pin. If UART is used it will changed the Pin




#define X_STEP_PIN             GPIO_NUM_27
#define X_DIRECTION_PIN        GPIO_NUM_26
#define X_LIMIT_PIN            GPIO_NUM_39
#define Y_STEP_PIN             GPIO_NUM_13
#define Y_DIRECTION_PIN        GPIO_NUM_14
#define Y_LIMIT_PIN            GPIO_NUM_39
#define Z_STEP_PIN             GPIO_NUM_25
#define Z_DIRECTION_PIN        GPIO_NUM_33
#define Z_LIMIT_PIN            GPIO_NUM_39
#define STEPPERS_DISABLE_PIN   GPIO_NUM_32 // one disable pin for all
#define STEPPERS_LIMIT_ALL_PIN GPIO_NUM_39 // one limit pin for all
#define VSENSE_CHANNEL ADC1_CHANNEL_6
#define BSENSE_CHANNEL ADC1_CHANNEL_7

// changing microsteps via software for Z is deprecated
// somewhere very far in the future those two Pins will be used for other stuff
#define Z_STEPPER_MS2_PIN      16          // only Z can use software controlled microstepping
#define Z_STEPPER_MS1_PIN      17          // only Z can use software controlled microstepping

/*
#define GEDM_PIN_MISO  19 // T_DO, SD_MISO, SDO<MISO>
#define GEDM_PIN_MOSI  23 // T_DIN, SD_MOSI, SDI<MOSI>
#define GEDM_PIN_SCLK  18 // T_CLK, SD_SCK, SCK
#define GEDM_PIN_CS    5 // 2 evo change
#define GEDM_PIN_RST   4
#define GEDM_PIN_T_CS  21
#define GEDM_PIN_DC    15
#define GEDM_PIN_SD_CS 2 // 5 evo change
*/


/***

39 -
36 -
35      input only -> used for shunt opamp feedback in next version!
34 -
33 -    green
32 -    green
31 ? not exposed
30 ? not exposed
29 ? not exposed
28 ? not exposed
27 -   green
26 -   green
25 -   green
24 ? not exposed
23 -   green
22 -   green
21 -   green
20 ? not exposed
19 -   green
18 -   green
17     green this
16 -   green
15 -   green
14 -   green
13 -   green
12 -
11 x flash spi
10 x flash spi
9  x flash spi
8  x flash spi
7  x flash spi
6  x flash spi
5 -    green
4 -    green
3     rx pin, can be ioGPIO3 (RX) as an OUTPUT is NOT recommended; 330Ohm
2 -    green
1   -used in v1; not used in v2    tx pin can be io
0      output ok, outputs PWM signal at boot, must be LOW to enter flashing mode

0  Output OK - ADC2-CH1 (GPIO 0) ???
35 I

17 IO -> spi cs
3  RX->IO spi in
1  TX->V2 spi clk
16 = spindel dir... IO

*/







const int X_AXIS = 0;  // Axis indexing value.
const int Y_AXIS = 1;
const int Z_AXIS = 2;
#define A_AXIS 3
#define B_AXIS 4
#define C_AXIS 5
#ifndef N_AXIS
#    define N_AXIS 3
#endif

/**
 * HOMING
 * 
 * Seek and feedrate is not used
 * 
 **/
#define DEFAULT_HOMING_ENABLE         1
#define DEFAULT_HOMING_FEED_RATE      50.0  // mm/min
#define DEFAULT_HOMING_SEEK_RATE      200.0 // mm/min
#define DEFAULT_HOMING_DEBOUNCE_DELAY 100   // msec (0-65k)
#define DEFAULT_HOMING_PULLOFF        3.0   // mm
#define DEFAULT_HOMING_DIR_MASK       0     // homing all axis into positive direction
//#define DEFAULT_HOMING_DIR_MASK (1<<Y_AXIS) 
//#define DEFAULT_HOMING_DIR_MASK (1<<X_AXIS) // x axis homing in inverted direction ( table seeks the switch to the right )
//#define DEFAULT_HOMING_DIR_MASK ((1<<X_AXIS)|(1<<Y_AXIS))
//#define HOMING_FORCE_SET_ORIGIN
//HOMING_FORCE_ORIGIN

/**
 * SPEEDS
 * 
 * At the moment it doesn't use mm based feedrates
 * GCode F feedrates are ginored
 * It uses different stepdelays for different tasks
 * Only Z uses a max feedrate that it doesn't exceeds
 **/



// this gets overwritten
typedef struct {
    int RAPID         = 80;
    int EDM           = 150;
    int DEFAULT_DELAY = 150;
    int PROBING       = 800;
    int REPROBE       = 250;
    int RETRACT       = 150;
    int HOMING_FEED   = 400;
    int HOMING_SEEK   = 90;
    int MICROFEED     = 29296;
    int WIRE_RETRACT_HARD = 400;    
    int WIRE_RETRACT_SOFT = 200;    
} speeds;
extern speeds process_speeds;



extern float max_feeds[N_AXIS];
extern uint16_t max_feeds_micros[N_AXIS];

#define REPORT_ECHO_RAW_LINE_RECEIVED // debug only
#define MACHINE_NAME    "G-EDM"
#define ROOT_FOLDER     "/gedm3"
#define SETTINGS_FOLDER "/settings"
#define GCODE_FOLDER    "/gcode"
#define STEP_PULSE_DELAY 0
#define TOUCH_CALIBRATION_FILE "/touch_calibration_data"

/**
 Setting Value 	Mask 	Invert X 	Invert Y 	Invert Z
0 	00000000 	N 	N 	N
1 	00000001 	Y 	N 	N
2 	00000010 	N 	Y 	N
3 	00000011 	Y 	Y 	N
4 	00000100 	N 	N 	Y
5 	00000101 	Y 	N 	Y
6 	00000110 	N 	Y 	Y
7 	00000111 	Y 	Y 	Y
 **/
//#define DEFAULT_DIRECTION_INVERT_MASK 4 // uint8_t
#define DEFAULT_DIRECTION_INVERT_MASK 6 // uint8_t
//#define DEFAULT_DIRECTION_INVERT_MASK 7 // uint8_t


#ifdef IS_GEDM_EVO2_ROUTER
#define DEFAULT_Z_MAX_TRAVEL 100.0 // mm NOTE: Must be a positive value.
#define DEFAULT_X_MAX_TRAVEL 240.0 // mm NOTE: Must be a positive value.
#define DEFAULT_Y_MAX_TRAVEL 300.0 // mm NOTE: Must be a positive value.
#else
#define DEFAULT_Z_MAX_TRAVEL 300.0 // mm NOTE: Must be a positive value.
#define DEFAULT_X_MAX_TRAVEL 240.0 // mm NOTE: Must be a positive value.
#define DEFAULT_Y_MAX_TRAVEL 106.0 // mm NOTE: Must be a positive value.
#endif



/**
  *
  * Z Axis
  *
  **/
//#define DEFAULT_HOMING_CYCLE_2 bit(Z_AXIS)
#define DEFAULT_Z_MAX_RATE 200.0 // mm/min
#define DEFAULT_Z_ACCELERATION 100.0 // mm/sec^2
#define Z_AXIS_JOG_SPEED 200



/**
  *
  * X Axis
  *
  **/
#define DEFAULT_HOMING_CYCLE_0 bit(X_AXIS)
#define DEFAULT_X_MAX_RATE 200.0 // mm/min
#define DEFAULT_X_ACCELERATION 100.0 // mm/sec^2
#define X_AXIS_JOG_SPEED 200


/**
  *
  * Y Axis
  *
  **/
#define DEFAULT_HOMING_CYCLE_1 bit(Y_AXIS)
#define DEFAULT_Y_MAX_RATE 200.0 // mm/min
#define DEFAULT_Y_ACCELERATION 100.0 // mm/sec^2
#define Y_AXIS_JOG_SPEED 200


/**
  *
  * A axis / Spindle/Wire stepper 
  * 
  **/
#define DEFAULT_A_MAX_FREQUENCY 10000
#define WIRE_VARIABLE_SPEED_RANGE 100.0


// HARD_LIMIT_FORCE_STATE_CHECK
#define DEFAULT_SOFT_LIMIT_ENABLE 1 
#define DEFAULT_HARD_LIMIT_ENABLE 1 
#define DISABLE_LIMIT_PIN_PULL_UP
#define ENABLE_SOFTWARE_DEBOUNCE
#define DEFAULT_INVERT_LIMIT_PINS 0
#define DEFAULT_INVERT_ST_ENABLE 0
//#define LIMITS_TWO_SWITCHES_ON_AXES
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // stay on
#define DEFAULT_SPINDLE_BIT_PRECISION 12
#define DEFAULT_SPINDLE_RPM_MIN   0.0
#define DEFAULT_SPINDLE_RPM_MAX   1023.0
#define DEFAULT_SPINDLE_MIN_VALUE 0.0
#define DEFAULT_SPINDLE_MAX_VALUE 1023.0
#define DEFAULT_SPINDLE_FREQ      20000.0
#define SPINDLE_PWM_BASE_FREQ     20000.0
#define DEFAULT_LASER_FULL_POWER  1023
#define DEFAULT_LASER_MODE        0
#define DEFAULT_SPINDLE_OFF_VALUE 0.0



#define EDM_NO_PROBE false
#define OPTIMIZE_MICROSTEPPING true // deprecated;
#define PWM_FREQUENCY_MAX 200000 // never go that high. It is untested. 
                                 // Won't destroy anything but may not work with the code very well. Further testing needed.
#define PWM_FREQUENCY_MIN 4000  // same as above. The PWM function can only get as low as 1000. in theory it goes lower but throws errors below 1000. Still seems to work if lower.
#define PWM_DUTY_MAX 100.0       // 100% should never be used. Maximum 80% to give a small off time.

#define PROBE_SPEED 60.0 // mm/min; speed used until the first contact (short) was made
#define PROBE_RETRACTION_MM 1.0


#define VSENSE_SAMPLES 5 // the number of samples taken from the pin for each update of the vSense value
#define INTERFACE_INTERVAL 100             // faster if not in EMD process
#define INTERFACE_INTERVAL_WHILE_EDM 300 // lesser updates if in EDM process since the updates interupt our step pulses
#define LOADER_DELAY 2000000


// https://github.com/G-EDM/G-EDM/#donations
#define EXTENDED_FEATURES true


#define UI_TASK_STACK_SIZE 10240
#define DUTY_SHORT_CIRCUIT 20.0
#define FREQUENCY_SHORT_CIRCUIT 40000
#define WIRE_SPINDLE_DIAMETER 12.0 // this is used to roughly get the wire speed. Over time the diameter get thicker and thicker and wire speed increases. Therefore this is not really accurate


/**
  * 
  * ╔════╗         ╔╗      ╔═══╗               ╔╗           
  * ║╔╗╔╗║         ║║      ║╔═╗║              ╔╝╚╗          
  * ╚╝║║╚╝╔══╗ ╔══╗║║╔╗    ║╚═╝║╔═╗╔╗╔══╗╔═╗╔╗╚╗╔╝╔╗╔══╗╔══╗
  *   ║║  ╚ ╗║ ║══╣║╚╝╝    ║╔══╝║╔╝╠╣║╔╗║║╔╝╠╣ ║║ ╠╣║╔╗║║══╣
  *  ╔╝╚╗ ║╚╝╚╗╠══║║╔╗╗    ║║   ║║ ║║║╚╝║║║ ║║ ║╚╗║║║║═╣╠══║
  *  ╚══╝ ╚═══╝╚══╝╚╝╚╝    ╚╝   ╚╝ ╚╝╚══╝╚╝ ╚╝ ╚═╝╚╝╚══╝╚══╝
  *
  * Better don't change anything here. Wrong task priorities will totally mess up everything
  * 
  * Tasks:
  *     1. Main UI task
  *     2. Sensor main task
  *     3. I2S watchdog task
  *     4. I2S receiver task
  *     5. grbl limit task ( just waiting for an interupt on the limti pin )
  *     6. main loop
  *     8. Scope task to receive the adc data
  *     ...
  * 
  **/
#define TASK_UI_PRIORITY              3 // the user interface task
#define TASK_SENSORS_DEFAULT_PRIORITY 4 // default sensor task. Currently just checking for an of/off switch event to debounce the switch
//#define TASK_VSENSE_SENDER_PRIORITY   3 // default ADC sender task. Only used if PWM is off and no edm process is running. Can be low priority. Exists to read the adc on the non process ui too.
#define TASK_SCOPE_PRIORITY           1 // task that fills the scope with data
#define TASK_LIMIT_PRIORITY           5 // limit switch debounce task

#define TASK_PROTOCOL_PRIORITY        1
#define TASK_VSENSE_RECEIVER_PRIORITY 1 // This task does the actual sampling after it received a request

extern float wire_speed_mm_min; // not used yet
extern int   wire_spindle_speed; // frequency in hz
extern float pwm_frequency; // 5khz - 10khz are common for rough cuts
extern float pwm_duty;      // the deeper the hole gets the higher the duty should be. I used 80% for a deep hole.
extern int   operation_mode; // 1 = drill/sinker, 2 = reamer (just a constant up/down movement without progressing down), 3 = 3D floating engraver, 4 = 2D wire
extern int   finish_after_mm; // f the tool moves 2mm without load it considers the process as done
extern float z_stop_after_mm; // the maximum travel into the workpiece; Note: EDM has electrode wear of x percent and setting this to 1mm doesn't produce a 1mm deep hole
extern bool  z_no_home; // set to true if no homing is needed and edm process starts without homing
extern int   vsense_voltage_min; // ignored for now
extern DMA_ATTR bool lock_reference_voltage;
extern float vsense_drop_range_min; // target range bottom. We try to be between min and max and hodl there
extern float vsense_drop_range_max; // target range top. A default range from min+10 it could be min+40.
                                    // That produces lesser motion but a little motion is not a bad thing
                                    // for a more steady burn use min+20/30/40

extern float probe_trigger_current; 
extern int probe_logic_low_trigger_treshold;   


extern float VSENSE_DROP_RANGE_SHORT;  // voltage drops above this value are concidered a full short circuit
extern float pwm_duty_probing;
extern int   pwm_frequency_probing;
extern float electrode_up_offset_mm;   // distance above the probe position where the electrode up movement stops
extern float electrode_down_probe_mm;  // electrode down moves quickly down until there are given mm left to the target position and then starts reprobing
                                       // this increases speed a little and helps prevent some unexpected problems like false positives in deeper cuts etc 
extern bool protocol_ready;
extern DMA_ATTR bool enable_realtime_vsense_update;
extern DMA_ATTR bool is_between_steps;
extern DMA_ATTR bool force_redraw;
extern DMA_ATTR bool edm_process_done;       // if set to true in the edm process the task gets stopped from within the mainloop and also disables the spark PWM
extern DMA_ATTR volatile bool edm_process_is_running; // this is true if the machine actually is doing work
extern DMA_ATTR int  stop_edm_task;
extern DMA_ATTR int  start_edm_task;
extern DMA_ATTR volatile unsigned long interface_timer; // this is used for the interval to update the interface
extern DMA_ATTR volatile bool limit_touched;
extern DMA_ATTR volatile bool probe_touched;
extern DMA_ATTR volatile float position_target[6];
extern DMA_ATTR volatile bool has_last_position;
extern DMA_ATTR volatile bool cutting_depth_reached;
extern DMA_ATTR volatile bool take_over_z;
extern DMA_ATTR volatile bool enable_correction;
extern DMA_ATTR volatile int64_t flush_retract_timer_us;
extern DMA_ATTR volatile int64_t flush_retract_after; // retract tool after x us to allow flushing; set to 0 to disable;


extern int   flush_offset_steps; // after moving the axis up for flushing it goes back down to the start position minus the offset steps
extern float flush_retract_mm;
extern bool  disable_spark_for_flushing;
extern bool  enable_spindle;
extern float tool_diameter;
extern bool simulate_gcode; // if enabled the gcode will only move XY and won't react to up/down commands
extern DMA_ATTR volatile float reamer_travel_mm; // the travel distance for the reamer up/down oscillation.
                               // NOTE: In reamer moder the axis is homed and then moves down the reamer_travel_mm!!
                               // This is to not overschoot the home switch. It is necessary to make sure the axis can move the
                               // given mm down without crashing.
extern DMA_ATTR volatile float reamer_duration;  // the duration of the reasming process in seconds . 0.0 means it runs until manually stopped

extern char get_axis_name(uint8_t axis);
extern DMA_ATTR volatile int last_used_motion_plan;

extern DMA_ATTR volatile int debug_log;
extern DMA_ATTR volatile int debug_flag;

extern DMA_ATTR volatile bool use_stop_and_go_flags;
extern DMA_ATTR volatile bool enable_dpm_rxtx;


typedef struct gedm_conf {
    int  gscope_estop                = 0;
    int  process_error               = 0;
    bool gedm_planner_line_running   = false;
    bool gedm_planner_sync           = false;
    bool gedm_flushing_motion        = false;
    bool edm_pause_motion            = false;
    bool gedm_single_axis_drill_task = false;
    bool gedm_floating_z_gcode_task  = false;
    bool gedm_wire_gcode_task        = false;
    bool gedm_retraction_motion      = false;
    bool gedm_reprobe_motion         = false;
    bool gedm_reprobe_block          = false;
    bool edm_process_finished        = false;
    bool gedm_stop_process           = false;
    bool gedm_reset_protocol         = false;
    bool gedm_insert_probe           = false;
    float gedm_probe_position_x      = false;
    float gedm_probe_position_y      = false;
    bool gedm_disable_limits         = false;
    int current_line                 = 0;
} gedm_conf;
extern DMA_ATTR volatile gedm_conf gconf;



// the below settings highly depends on the machine and axis backlash
#define ENGRAVER_MOTION_PLAN_Z_CASE_3_MIN 0.005
//#define ENGRAVER_MOTION_PLAN_Z_CASE_3_MAX 0.01
#define ENGRAVER_MOTION_PLAN_Z_CASE_2_MIN 0.0025
#define HISTORY_MOTION_PLAN_CASE_3_RETRACT_HIGH_SPEED 0.001 
#define HISTORY_MOTION_PLAN_CASE_4_RETRACT_HIGH_SPEED 0.002 // deprecated
#define HISTORY_MOTION_PLAN_CASE_5_RETRACT_HIGH_SPEED 0.05


typedef struct retraction_distances {
  float soft_retraction      = HISTORY_MOTION_PLAN_CASE_3_RETRACT_HIGH_SPEED;
  float medium_retraction    = HISTORY_MOTION_PLAN_CASE_4_RETRACT_HIGH_SPEED;
  float hard_retraction      = HISTORY_MOTION_PLAN_CASE_5_RETRACT_HIGH_SPEED;
  float wire_break_distance  = 1.0;
  bool  early_exit           = true; // if true it will allow to exit a retraction without moving the full distance
} retraction_distances;
extern DMA_ATTR volatile retraction_distances retconf;

extern DMA_ATTR volatile int motion_plan;
