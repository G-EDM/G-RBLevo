#pragma once

#include <driver/adc.h>
#include <pins_arduino.h>

/** 
  * OTA firmware update instructions for an already installed system:
  * Compile the firmware with vStudio code and platform.io
  * The file name needed is firmware.bin
  * location should be within the project folder at:
  * .pio/build/esp32dev/firmware.bin
  * Copy the file to the sd card at top directory "/", put it into the G-EDM 
  * and restart the machine
  * the file will be renamed to firmware.bak to prevent another install
  **/
#define FIRMWARE_VERSION 2.0


/** comment this out to use the default max travel config for the tower 
  * or uncomment it to use the default travel config for the GEDM EVO2 router 
  **/
//#define IS_GEDM_EVO2_ROUTER
#define IS_GEDM_EVO2_ROUTER

/** 
 * The current code is build around mill type CNCs where the XY table is moving 
 * If a router type CNC is used uncomment this, for mill type cnc comment it out like:
 * //#define CNC_TYPE_IS_ROUTER
 **/
//#define CNC_TYPE_IS_ROUTER
#define CNC_TYPE_IS_ROUTER

/** 
 * If no z axis exists uncomment this
 **/
//#define Z_AXIS_NOT_USED
#define Z_AXIS_NOT_USED


#ifdef IS_GEDM_EVO2_ROUTER
/** the below parameters are the most critical **/
typedef struct {
    float RAPID         = 180.0;
    float EDM           = 60.0;
    float DEFAULT_DELAY = 100.0;
    float PROBING       = 30.0;
    float REPROBE       = 58.5;
    float RETRACT       = 80.0;
    float HOMING_FEED   = 36.6;
    float HOMING_SEEK   = 220.0;
    float MICROFEED     = 0.5;
    float WIRE_RETRACT_HARD = 20.0;
    float WIRE_RETRACT_SOFT = 5.0;
} speeds_mm_min;
#else
/** the below parameters are the most critical **/
typedef struct {
    float RAPID         = 140.0;
    float EDM           = 60.0;
    float DEFAULT_DELAY = 100.0;
    float PROBING       = 30.0;
    float REPROBE       = 58.5;
    float RETRACT       = 80.0;
    float HOMING_FEED   = 36.6;
    float HOMING_SEEK   = 140.0;
    float MICROFEED     = 0.5;
    float WIRE_RETRACT_HARD = 30.0;
    float WIRE_RETRACT_SOFT = 15.0;
} speeds_mm_min;
#endif


extern speeds_mm_min process_speeds_mm_min;

#define WIRE_SPEED_SHORT_CIRCUIT_RPM_UP 1.0 // on short circuits the wire can melt to the work and get sticky. This can be a problem
                                            // while retracting around corners. To overcome this the wire does a small but rapid speed change
                                            // that gives the wire a small bump to unstick it. With a reliable wire tension this may not be needed
                                            // but for more basic wire EDMs this makes a big difference. 
                                            // the value is in RPM and added relative to the wire speed. If the wire spindle runs
                                            // at 4RPM it will increase to 4RPM+ThisValue on real short circuits.
                                            // Set it to 0.0 to disable this feature

#define ENABLE_SERIAL true//true
#define ENABLE_SD_CARD true

typedef struct {
    int x = 3840;
    int y = 3840;
    int z = 4096;
    int a = 0;
    int b = 0;
    int c = 0;
    //todo: a axis spindle speed is currently based on steps per rotation (200*DEFAULT_A_MICROSTEPS)
 
} steps_per_mm_settings;
extern steps_per_mm_settings steps_per_mm_config;


//#define DEFAULT_Z_STEPS_PER_MM    4096.0 //2048.0 1024.0
//#define DEFAULT_X_STEPS_PER_MM    3840.0
//#define DEFAULT_Y_STEPS_PER_MM    3840.0
// microstep settings for xyz are deprecated
#define GEDM_DEFAULT_Z_MICROSTEPS 64 //64 // default
#define GEDM_DEFAULT_X_MICROSTEPS 32
#define GEDM_DEFAULT_Y_MICROSTEPS 32
#define DEFAULT_A_MICROSTEPS      16
#define SUPPLY_VOLTAGE_MAX        60.0 // this is the voltage corresponding to he max possible
                                       // 2.8v vSense feedback (at 2.8v vSense supply voltage = this)
                                       // this is only used for displaay purposes 
                                       // the edm process doesn't care about it since it only calculates with
                                       // with the vSense voltage
                                       // if the pulsegenerator works at 100v just set this to 100
#define STEPPER_DIR_CHANGE_DELAY        100
#define DEFAULT_STEP_PULSE_MICROSECONDS 4



#define USE_MOTION_BOARD_VERSION_V2 // uncomment if V1 is used (v2 = jumpers under the z tmc driver to control z microstepping)

#define HOLD_LOOP_BREAK_COUNT 5         // if the lower range of the setpoint is hit it will try to hold the position and thus ignore some potential shorts to stabilize the motion
#define SHORT_CANCELED_AT_MOTION_PLAN 4 // if the axis is retracting it will cancel the retraction as soon as it reaches this motion plan

#define ENABLE_PWM_EVENTS // if set to true the pwm will flag a variable for falling and raising edges; maybe useful 






/**
  * 
  * ╔════╗ ╔═══╗╔════╗    ╔═══╗          ╔╗           
  * ║╔╗╔╗║ ║╔══╝║╔╗╔╗║    ╚╗╔╗║          ║║           
  * ╚╝║║╚╝ ║╚══╗╚╝║║╚╝     ║║║║╔╗╔══╗╔══╗║║ ╔══╗ ╔╗ ╔╗
  *   ║║   ║╔══╝  ║║       ║║║║╠╣║══╣║╔╗║║║ ╚ ╗║ ║║ ║║
  *  ╔╝╚╗ ╔╝╚╗   ╔╝╚╗     ╔╝╚╝║║║╠══║║╚╝║║╚╗║╚╝╚╗║╚═╝║
  *  ╚══╝ ╚══╝   ╚══╝     ╚═══╝╚╝╚══╝║╔═╝╚═╝╚═══╝╚═╗╔╝
  *                                  ║║          ╔═╝║ 
  *                                  ╚╝          ╚══╝ 
  * TFT Display
  * 
  **/
#define REPEAT_DISPLAY_CALIBRATION false // if set to true display calibration is forced to repeat; after calibratioset this to false again and reflash the firmware
#define DISABLE_TFT_CALIBRATION    false // if no display is attached the first time calibration will block everything

/**
  * 
  *     ╔═══╗                
  *     ║╔═╗║                
  * ╔╗╔╗║╚══╗╔══╗╔═╗ ╔══╗╔══╗
  * ║╚╝║╚══╗║║╔╗║║╔╗╗║══╣║╔╗║
  * ╚╗╔╝║╚═╝║║║═╣║║║║╠══║║║═╣
  *  ╚╝ ╚═══╝╚══╝╚╝╚╝╚══╝╚══╝
  *
  * vSense
  *                          
  **/
#define VSENSE_MAX        3300 
//#define USE_10_BIT

#ifdef USE_10_BIT
    #define VSENSE_BIT_WIDTH ADC_WIDTH_10Bit
    #define BIT_WIDTH_MAX 0x3ff
    #define VSENSE_RESOLUTION 1023 // default 10bit ESP ADC
#else
    #define VSENSE_BIT_WIDTH ADC_WIDTH_12Bit
    #define BIT_WIDTH_MAX 0xfff
    #define VSENSE_RESOLUTION 4095 // default 12bit ESP ADC
#endif

#define RISING_AVG 1
#define FALLING_AVG 3 // deprecated
#define DEFAULT_AVG 3

#define SCOPE_USE_QUEUE 

/**
  * 
  * ╔═══╗╔╗           ╔╗     ╔═══╗    ╔═══╗        
  * ║╔═╗║║║          ╔╝╚╗    ║╔═╗║    ║╔═╗║        
  * ║╚══╗║╚═╗╔╗╔╗╔═╗ ╚╗╔╝    ║║ ║║╔══╗║║ ║║╔╗╔╗╔══╗
  * ╚══╗║║╔╗║║║║║║╔╗╗ ║║     ║║ ║║║╔╗║║╚═╝║║╚╝║║╔╗║
  * ║╚═╝║║║║║║╚╝║║║║║ ║╚╗    ║╚═╝║║╚╝║║╔═╗║║║║║║╚╝║
  * ╚═══╝╚╝╚╝╚══╝╚╝╚╝ ╚═╝    ╚═══╝║╔═╝╚╝ ╚╝╚╩╩╝║╔═╝
  *                               ║║           ║║  
  *                               ╚╝           ╚╝  
  *
  * Shunt OpAmp
  * 
  * All testing and work is done with a 5A/75mV Shunt
  * It is not needed to have an accurate representation of the current
  * 
  **/
//#define USE_SHUNT_OPAMP_FEEDBACK
#define SHUNT_OPAMP_ADC_AT_MAX_AMP 4095 // depends on the setup. 4095 equals 3.1v and is the max ADC resolution
#define SHUNT_OPAMP_MAX_AMP        5000 // 5A / 5000mA max amps


static const int queue_item_size = 6; // used to transmitt adc data to other locations


/**
  *
  * ╔══╗╔═══╗╔═══╗    ╔═══╗╔═╗╔═╗╔═══╗    ╔═══╗╔═══╗╔═══╗
  * ╚╣╠╝║╔═╗║║╔═╗║    ╚╗╔╗║║║╚╝║║║╔═╗║    ║╔═╗║╚╗╔╗║║╔═╗║
  *  ║║ ╚╝╔╝║║╚══╗     ║║║║║╔╗╔╗║║║ ║║    ║║ ║║ ║║║║║║ ╚╝
  *  ║║ ╔═╝╔╝╚══╗║     ║║║║║║║║║║║╚═╝║    ║╚═╝║ ║║║║║║ ╔╗
  * ╔╣╠╗║║╚═╗║╚═╝║    ╔╝╚╝║║║║║║║║╔═╗║    ║╔═╗║╔╝╚╝║║╚═╝║
  * ╚══╝╚═══╝╚═══╝    ╚═══╝╚╝╚╝╚╝╚╝ ╚╝    ╚╝ ╚╝╚═══╝╚═══╝
  *
  * I2S DMA ADC
  *  
  **/
#define USE_DUAL_CHANNEL
#define USE_I2S_DMA_ADC // if true it will use the I2S DMA ADC mode to sample the ADC. This is fast but maybe unstable. Needs deeper testing. 

//#define USE_IDF_650 // default now. Don't disable.

#ifdef USE_IDF_650
    #define I2S_DMA_BUF_LEN   (100)
    #define I2S_TIMEOUT_TICKS (20) 
    #define I2S_SAMPLE_RATE   (600000) 
#else
    #define I2S_DMA_BUF_LEN   (100)//150
    #define I2S_TIMEOUT_TICKS (20)
    #define I2S_SAMPLE_RATE   (600000) //650000
    //#define I2S_SAMPLE_RATE   (100000)
#endif

#define I2S_BUFF_COUNT  (2)//100

//#define I2S_USE_QUEUE // comment out to not use the I2S build-in queue function. it seems to slow down the max possible frequency heavily
                      // possible that the timer based implementation overloads I2S and starts a read as soon as possible
                      // while the integrated queue fires only after the full cleanup
                      // but it may just need some code tweaks currently the max rate I am able to get with the build-in queue is 650kSps
                      // while the custom implementation is able to get 1mSps. Big difference.
                      // maybe it is still worth using the build-in queue as it removes the hw timerand therefore is less ressource intensive?

#ifdef I2S_USE_QUEUE
    #undef I2S_SAMPLE_RATE
    #undef I2S_TIMEOUT_TICKS
    #define I2S_TIMEOUT_TICKS (200)
    #define I2S_SAMPLE_RATE (200000) // old tests: 600k was about the max possible value that produced stable results. Above it it starts to decrease the kSps
#endif


#define I2S_NUM_SAMPLES I2S_DMA_BUF_LEN


/**
  * 
  * ╔═╗ ╔╗     ╔╗                   ╔╗     ╔╗    ╔═══╗        ╔╗ ╔╗                 
  * ║║╚╗║║    ╔╝╚╗                  ║║     ║║    ╚╗╔╗║        ║║╔╝╚╗                
  * ║╔╗╚╝║╔══╗╚╗╔╝    ╔═╗╔══╗╔══╗ ╔═╝║╔╗ ╔╗║║     ║║║║╔══╗╔═╗ ╚╝╚╗╔╝    ╔╗╔╗╔══╗╔══╗
  * ║║╚╗║║║╔╗║ ║║     ║╔╝║╔╗║╚ ╗║ ║╔╗║║║ ║║╚╝     ║║║║║╔╗║║╔╗╗   ║║     ║║║║║══╣║╔╗║
  * ║║ ║║║║╚╝║ ║╚╗    ║║ ║║═╣║╚╝╚╗║╚╝║║╚═╝║╔╗    ╔╝╚╝║║╚╝║║║║║   ║╚╗    ║╚╝║╠══║║║═╣
  * ╚╝ ╚═╝╚══╝ ╚═╝    ╚╝ ╚══╝╚═══╝╚══╝╚═╗╔╝╚╝    ╚═══╝╚══╝╚╝╚╝   ╚═╝    ╚══╝╚══╝╚══╝
  *                                   ╔═╝║                                          
  *                                   ╚══╝                                          
  * 
  * WARNING: The below section is not fully impemented yet. Don't use.
  * GPIO35 = ADC1_CHANNEL_7
  **/







/** External ADC specific section **/
/** 
  * The ESP32 is a very nice a powerfull little board 
  * but the internal ADC (Analog to Digital converter) of the ESP32 is not good. 
  * It is not super fast and very noisy. The best option is to use an external ADC. 
  * Depending on the circuit used for the feedback the voltage may raise with more load or drop
  * with more load. Default behaviour is build around a sharp voltage drop on a current controlled PSU.
  * In this case the initial feedback voltage is about 2.8v and drops relative to the load down to 0v.
  * If the initial feedback voltage used is 0v and it goes up relative to the load than vSense is inverted.
  * A 16Bit ADC will provide enough precision.
  **/
#define USE_EXTERNAL_SPI_ADC false // work in progress! Not ready yet!
#define EXTERNAL_SPI_ADC_CS_PIN      17       // external spi adc will use the pins used to control z microstepping on the old motion board
#define EXTERNAL_SPI_ADC_FREQUENCY   20000000 // hz
#define EXTERNAL_SPI_ADC_REF_VOLTAGE 5000      // the reference voltage used in mV
#define EXTERNAL_SPI_ADC_RESOLUTION  65536    //16 bit = 65536 steps; 12 bit = 4096 steps
/** ./ADC specific section **/

/** 
  * DPM specific section; DPM needs to run on address "01". 
  * This is the default address and can be changed on the DPM controller 
  * Warning: This implementation only works with the TTL simple protocol DPM version 
  * The default baudrate on the DPM is 9600. This needs to be changed to 115200 on the DPM. See the DPM manual for details.
  * 115200 is fast and needs a good level shifter circuit. I use a simple NPN transistor based level shifter circuit.
  * Available cheap ready to use HighSpeed Fullduplex logic level converters may not work. I tried one from Amazon without success.
  * DPM operates at 5v, ESP 3.3v. 
  **/
//#define USE_DPM_RXTX_SERIAL_COMMUNICATION
#define DPM_RXTX_SERIAL_COMMUNICATION_BAUD_RATE 115200 // valid: 2400, 4800,9600, 19200, 38400, 57600, 115200
#define DPM_INTERFACE_TX TX
#define DPM_INTERFACE_RX RX
/** ./DPM specific section **/



//#define I2S_USE_HWTIMER
//#define DEBUG_PROCESS_SCREEN
//#define DEBUG_SAMPLER_EXECUTION_SPEED


