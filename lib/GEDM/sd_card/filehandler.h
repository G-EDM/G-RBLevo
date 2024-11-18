#pragma once

#include <WString.h>
//#include <FS.h>
#include "SD.h"
#include "config/definitions.h"
#include <stdint.h>
#include "shared.h"
//#include <SPI.h>

extern bool SD_ready_next;
extern bool filehandler_is_ready;

class G_FILEHANDLER
{
private:
    File   current_file;
    String folder_root;
    String folder_settings;
    String folder_gcode;
    uint64_t card_size;
    uint32_t current_line_number;
    int64_t last_refresh;
    int error_code;
    int state;
    int cursor_position;
    bool ready;
    bool has_sd_card;
    bool ignore_sd;
    bool locked_for_ui;
    bool job_finished;

public:
    G_FILEHANDLER(void);
    SPIClass file_spi;
    String get_file_contents(  String file_path  );    
    String get_folder_settings( void );
    String get_folder_gcode( void );
    String get_next_filename( String extension );
    uint32_t get_current_line_number();
    bool is_locked_for_ui( void );
    bool get_is_ready( void );
    void create_directory_tree( int rounds ); 
    void filehandler_initialize( bool ignore );   
    void sd_card_refresh( void );
    bool get_has_sd_card( void );
    bool get_is_busy( void );
    bool write_file_contents( String file_path, String _data );    
    int get_error_code( void );
    int count_files_in_folder_by_extension( String folder, String extension );
    void get_files_in_folder_by_extension( String folder, String extension, char files[][100] );
    int open_file( String folder, String file );
    bool get_file_exists( String full_path );
    bool has_gcode_running( void );
    bool has_job_finished( void );
    void reset_job_finished( void );
    bool reset_and_repeat_file( void );
    bool get_line(char* line, int size );
    bool is_available( void );
    void set_spi_instance( SPIClass &_spi );
    bool open_gcode_file( String path );
    bool close_file( void );
    void open_folder( String folder );
    void close_current_folder( void );
};

extern G_FILEHANDLER filehandler;