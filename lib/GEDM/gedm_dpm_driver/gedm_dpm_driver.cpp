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


#include "gedm_dpm_driver.h"

DMA_ATTR int dpm_locked          = false;
DMA_ATTR int settings_milliamps  = 0;
DMA_ATTR int settings_millivolts = 0;
DMA_ATTR int measured_ma         = 0;
DMA_ATTR int measured_mv         = 0;

G_EDM_DPM_DRIVER dpm_driver;


G_EDM_DPM_DRIVER::G_EDM_DPM_DRIVER(){
    is_ready = false;
}

void G_EDM_DPM_DRIVER::end(){

    if( dpm_serial ){
        dpm_serial->end();
    }

}


bool G_EDM_DPM_DRIVER::setup( HardwareSerial& serial )
{
    dpm_serial = &serial;
    pinMode( DPM_INTERFACE_TX, OUTPUT );
    pinMode( DPM_INTERFACE_RX, INPUT );
    return true;
}
bool G_EDM_DPM_DRIVER::init()
{
    dpm_serial->updateBaudRate( DPM_RXTX_SERIAL_COMMUNICATION_BAUD_RATE );
    dpm_serial->flush();
    return true;
}

bool G_EDM_DPM_DRIVER::power_on_off( int turn_on, int max_retry ){
    char response[BUFFER_SIZE];
    char command_buf[10];
    bool success = false;
    sprintf(command_buf,":01w12=%d,", turn_on); 
    do{
        dpm_serial->flush();
        success = send( command_buf, response );
        if( success ){ //:01ok should be returned
            success = extract_ok( response );
        } else {
            vTaskDelay(100);
        }
        dpm_serial->flush();
        vTaskDelay(10);
    } while ( !success && --max_retry>=0 );
    return success;
}

bool G_EDM_DPM_DRIVER::set_setting_current(float value)
{
    char response[BUFFER_SIZE];
    dpm_serial->flush();
    int i_value = floor(value * 1000);
    char command_buf[15];
    sprintf(command_buf,":01w11=%d,", i_value);  
    bool success = send( command_buf, response );
    if( success ){ //:01ok should be returned
        success = extract_ok( response );
    }
    dpm_serial->flush();
    vTaskDelay(10);
    return success ? true : false;
}
bool G_EDM_DPM_DRIVER::set_setting_voltage(float value)
{
    char response[BUFFER_SIZE];
    dpm_serial->flush();
    int i_value = floor(value * 100);
    char command_buf[15];
    sprintf(command_buf,":01w10=%d,", i_value);  
    bool success = send( command_buf, response );
    if( success ){ //:01ok should be returned
        success = extract_ok( response );
    }
    dpm_serial->flush();
    vTaskDelay(10);
    return success ? true : false;
}

/*int G_EDM_DPM_DRIVER::set_voltage_and_current(float v, float c)
{
    int8_t retry = 0;
    char response[BUFFER_SIZE];
    bool completed = false;
    int _v = floor(v * 100);
    int _c = floor(c * 1000);
    do
    {
        dpm_serial->println(":01w20=" + String(_v) + "," + String(_c) + ",");
        completed = fetch(response);
    } while ((!completed) && (++retry < 100));
    return completed ? 1 : 0;
}*/

bool G_EDM_DPM_DRIVER::get_power_is_on( void ){
    // send cmd: :01r12=0
    // receive:  :01r12=0, or  :01r12=1,
    // 0 = off, 1 = on
    char response[BUFFER_SIZE];
    char command_buf[10];
    int status;
    sprintf(command_buf,":01r12=%d,", 0);  
    bool success = send( command_buf, response );
    if( success ){ 
        success = extract( status, 1, response );
    }
    dpm_serial->flush();
    vTaskDelay(10);
    if( ! success ){ return false; }
    return status == 1 ? true : false;
}//:01r12=0,

bool G_EDM_DPM_DRIVER::get_setting_voltage( int &voltage ){
    char response[BUFFER_SIZE];
    dpm_serial->flush();
    char command_buf[10];
    sprintf(command_buf,":01r10=%d,", 0);   
    bool success = send( command_buf, response );
    if( success ){
        extract( voltage, 10, response );
        success = voltage < 0 ? false : true;
    }
    dpm_serial->flush();
    vTaskDelay(10);
    if( ! success ){
        voltage = -1;
        return success;
    }
    memcpy(&settings_millivolts, &voltage, sizeof(int));
    return true;
}
bool G_EDM_DPM_DRIVER::get_setting_current( int &current ){
    char response[BUFFER_SIZE];
    dpm_serial->flush();
    char command_buf[10];
    sprintf(command_buf,":01r11=%d,", 0);   
    bool success = send( command_buf, response );
    //bool success = send( ":01r11=0," );
    if( success ){
        extract( current, 1, response );
        success = current < 0 ? false : true;
    }
    dpm_serial->flush();
    vTaskDelay(10);
    if( ! success ){
        current = -1;
        return success;
    }
    memcpy(&settings_milliamps, &current, sizeof(int));
    return true;
}

/** this only returns the voltage and current that is configured in the setting and not the realtime measurement **/
bool G_EDM_DPM_DRIVER::get_setting_voltage_and_current( int &voltage, int &current ){
    //bool success = send( ":01r10=0," );
    bool success = get_setting_voltage( voltage );
    if( success ){
        success = get_setting_current( current );
    }
    if( ! success ){
        voltage = -1;
        current = -1;
    }
    return success;
}



bool IRAM_ATTR G_EDM_DPM_DRIVER::get_measured_voltage_mv( int &voltage ){
    char response[BUFFER_SIZE];
    char command_buf[10];
    char cmd[] = ":01r30=0,";
    bool success = send( cmd, response );
    if( success ){
        extract( voltage, 10, response );
        success = ( voltage < 0 || voltage > VOLTAGE_ERROR_TRESHHOLD ) ? false : true;
        if( success ){
            memcpy(&measured_mv, &voltage, sizeof(int));
        }
    }
    return success;
}
bool IRAM_ATTR G_EDM_DPM_DRIVER::get_measured_current_ma( int &current ){
    char response[BUFFER_SIZE];
    char cmd[] = ":01r31=0,";
    bool success = send( cmd, response, false );
    if( success ){
        //Serial.println(response);
        extract( current, 1, response );
        success = ( current < 0 || current > CURRENT_ERROR_TRESHHOLD ) ? false : true;
        if( success ){
            memcpy(&measured_ma, &current, sizeof(int));
            last_amp_reading_timestamp = esp_timer_get_time();
        }
    }
    return success;
}

bool IRAM_ATTR G_EDM_DPM_DRIVER::measure_ma_is_outdated( int max_age ){
    if( esp_timer_get_time() - last_amp_reading_timestamp > max_age ){
        return true;
    } return false;
}






/**
 * For current measurement r31 is what should be received
 * There may be still a useful and valid result within a garbaged response
 * :01r31=3057. (valid)
 * 
 * :01r31O&&�rj (garbage)
 * :01r31�0. (garbage)
 * :�1r31=1699.(result still valid but garbage in the signal)
 * :41r31=1671. ( another valid result with garbage in the signal)
 * :01r31=736.� ( result again still valid )
 * :01r31=3153.:01r31=3147. (linebreak victim of garbage->double result in the pipe; latest is newest)
 * :01r51=4:01r31=3153. (garbaged double result)
 * :01r50=1533. ( some garbage, r50, but result valid )
 * :01r36=33225. ( invalid result >33A and also invalid return cmd r36 )
 * 3121:01r31=3121:01r3r�B�(��?���?␂ (garbage but 3121 would be valid..)
 * :01�31=3135. ( valid result )
 * �:01r31=682. ( valid result )
 * =3092:01r31=3073.����(��?���?␂
 * 8:01r31=3058.:01r31=����(��?���?␂
 * 
 * :01r42=6000.
 * :01r37=22105.
 * :01r51=:01r31=3078.
 * :01r36=31576.
 * :01r34=33699.
 * 
 * 
 * **/


/** send request for current without fetching the result **/
bool IRAM_ATTR G_EDM_DPM_DRIVER::request_measured_current_ma( void ){
    dpm_serial->println( ":01r31=0," );
    return true;
}
/** read current from stream if available **/
bool IRAM_ATTR G_EDM_DPM_DRIVER::fetch_measured_current_ma( int &current ){

    // the wanted value should be encapsulated between "=" and ".". cmdxy=XYZ.
    // keep fetching the stream until a "=" is found and get the value after it
    // keep fetching the stream to see if newer ones are following
    while( dpm_serial->available() ){

    }

    return true;

}


bool IRAM_ATTR G_EDM_DPM_DRIVER::send( char* cmd, char* response, bool blocking )
{

    while( dpm_locked ){}
    dpm_locked = true;
    int8_t retry = 0;
    bool completed = false;
    do{
        if( retry > 10 ){vTaskDelay(1);}
        memset( response, 0, sizeof(response)-1 );
        dpm_serial->println( cmd );
        completed = fetch( response, blocking );
    } while ((!completed) && (++retry < 100) );
    dpm_locked = false;
    return completed ? true : false;

}
bool IRAM_ATTR G_EDM_DPM_DRIVER::fetch( char* response, bool blocking )
{
    bool success = false;
    int length   = 0;
    if( blocking ){
        //vTaskDelay(1000);
        unsigned long start = millis();
        while( !dpm_serial->available() ){ if( millis()-start > 50 ){ break; } }
    }

    while( dpm_serial->available() ){
        char c = dpm_serial->read();
        if (c == '\n'){
            success= true;
            //response[++length] = '\0';
            break;
        } else { 
            response[length] = c;
            if(++length>=BUFFER_SIZE){
                dpm_serial->flush();
                break;
            }
        }
    }

     //while( dpm_serial->available() ){
        //Serial.println( dpm_serial->read() );
       // dpm_serial->read();
     //}
    //dpm_serial->flush();
    return success;
}
bool IRAM_ATTR G_EDM_DPM_DRIVER::extract_ok( char* response ){
    bool ok = false;
    for( int i = 0; i < BUFFER_SIZE; ++i ){
        if( i >= (BUFFER_SIZE-2) ){
            break;
        }
        if( response[i] == 'o' && response[i+1] == 'k' ){
            ok = true;
            break;
        }
    }
    return ok;
}
int IRAM_ATTR G_EDM_DPM_DRIVER::extract( int &value, int multiplier, char* response ){
    bool start = false;
    value      = 0;
    bool has_point = false;
    for( int i = 5; i < BUFFER_SIZE; ++i ){
        char c = response[i];
        //“,” or “.” or “,,”
        if( c == '.' || c == ',' ){ //  || c == ',,' not handled yet
            has_point = true;
            break;
        } else if( !start && c == '=' ){
            start = true;
        } else if( start && isdigit(c) ){
            if( ! value ){
                value = c - '0';
            } else {
                value *= 10;
                value += c - '0';
            }
        } else if( start && !has_point && !isdigit(c) ){
            // garbage?
            break;
        }
    }
    if( !start || !has_point ){
        //error
        value = -1;
    } else {
        if( multiplier > 0 ){
            value *= multiplier;// mV and mA
        } else if( multiplier < 0 ){
            value /= multiplier;
        }
    }
    return 1; 
}