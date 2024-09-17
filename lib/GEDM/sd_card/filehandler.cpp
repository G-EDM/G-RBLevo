#include "filehandler.h"

SDState sd_state          = SDState::Idle;
bool SD_ready_next        = false;
bool filehandler_is_ready = false;

G_FILEHANDLER filehandler;

G_FILEHANDLER::G_FILEHANDLER(){
  filehandler_is_ready = false;
  locked_for_ui        = false;
  job_finished         = false;
  cursor_position      = 0;
  current_line_number  = 0;
}

void G_FILEHANDLER::filehandler_initialize(bool enable){
  if( is_locked_for_ui() ){ return; }
  sd_state             = SDState::NotPresent;
  filehandler_is_ready = false;
  ignore_sd            = enable ? false : true;
  folder_root          = String(ROOT_FOLDER);
  folder_settings      = folder_root + String(SETTINGS_FOLDER);
  folder_gcode         = folder_root + String(GCODE_FOLDER);
  current_line_number  = 0;
  error_code           = 0;
  has_sd_card          = false;
  if (ignore_sd){
    filehandler_is_ready = true;
    sd_state = SDState::Idle;
    return;
  }

  if (!SD.begin(SS_OVERRIDE, file_spi)){
    error_code           = 1;
    filehandler_is_ready = true;
    sd_state = SDState::Idle;
  } else {
      create_directory_tree( 3 );
      has_sd_card          = true;
      filehandler_is_ready = true;
      sd_state             = SDState::Idle;
  }


}



void G_FILEHANDLER::create_directory_tree( int rounds ){
  if( ! has_sd_card || ignore_sd ){
    return;
  }
  if (!SD.exists(folder_root)){
    SD.mkdir(folder_root);vTaskDelay(1);
  }
  if (!SD.exists(folder_settings)){
    SD.mkdir(folder_settings);vTaskDelay(1);
  }
  if (!SD.exists(folder_gcode)){
    SD.mkdir(folder_gcode);vTaskDelay(1);
  }
};
bool G_FILEHANDLER::write_file_contents( String file_path, String _data ){
  if( ignore_sd || is_locked_for_ui() ){ 
    return false; 
  }
  if (!SD.exists(folder_root) || SD.cardSize() <= 0)
  {
    SD.end();
    filehandler_initialize(true);vTaskDelay(1);
  }
  sd_state = SDState::BusyCustom;
  if (SD.exists(file_path))
  {
    SD.remove(file_path);vTaskDelay(1);
  }
  int rounds   = 2;
  bool success = false;
  while( ! success && --rounds >= 0 ){
      current_file = SD.open(file_path, FILE_WRITE);vTaskDelay(1);
      if (current_file)
      {
        current_file.print(_data);vTaskDelay(1);
        current_file.flush();vTaskDelay(1);
      } 
      if( current_file.size() <= 0 || ! current_file.available() ){
          current_file.close();vTaskDelay(1);
          SD.end();
          filehandler_initialize(true);vTaskDelay(1);
      } else{
        success = true;
        current_file.close();vTaskDelay(1);
    }
  }
  sd_state = SDState::Idle;
  return success;
};
bool G_FILEHANDLER::get_file_exists( String full_path ){
  return SD.exists(full_path);
}
String G_FILEHANDLER::get_file_contents( String file_path ){
  if (ignore_sd || is_locked_for_ui() ){
    return "";
  }
  if (!SD.exists(folder_root) || SD.cardSize() <= 0){
    SD.end();vTaskDelay(1);
    filehandler_initialize(true);vTaskDelay(1);
  }

  if (!SD.exists(file_path)){
    return "";
  }
  sd_state = SDState::BusyCustom;
  current_file = SD.open(file_path, FILE_READ);
  String data = "";
  if (current_file){
    while (current_file.available()){
      data += String((char)current_file.read());vTaskDelay(1);
    }
    current_file.close();
  }
  sd_state = SDState::Idle;
  return data;
};

bool G_FILEHANDLER::get_line( char *line, int size ){
  if(!current_file){
    return false;
  }
  current_line_number += 1;
  int length = 0;
  while (current_file.available()){
    ++cursor_position;
    char c = current_file.read();
    //if (c == '\n')
    if (c == '\n' || c == '\r'){
      if( length == 0 ){
        // linebreak at the beginning
        continue;
      }
      break;
    }
    line[length] = c;
    if (++length >= size){
      return false;
    }

  }
  line[length] = '\0';
  int _continue = length || current_file.available();
  return _continue;
}

/** if we use a floating z axis we repeat the 2D path until the cutting depth matches the settings **/
bool G_FILEHANDLER::reset_and_repeat_file(){
  if (!current_file){
    return false;
  }
  //noInterrupts();
  if(current_file.seek(0)){ // reset the file position{
    current_line_number = 0; // reset the current line number
    cursor_position     = 0;
    return true;
  } else {
    return false;
  }
  //interrupts();
}

void G_FILEHANDLER::sd_card_refresh(){
  if( ignore_sd || is_locked_for_ui() ){ 
    return; 
  }
  if( esp_timer_get_time() - last_refresh > 10000000 ){
    last_refresh = esp_timer_get_time();
    if (!SD.exists(folder_root) || SD.cardSize() <= 0)
    {
      if( current_file ){
        current_file.close();
      }
      SD.end();
      filehandler_initialize(true);vTaskDelay(1);
    }
  }
}


String G_FILEHANDLER::get_next_filename( String extension ){
  if( is_locked_for_ui() ){ 
    return ""; 
  }
  File entry = current_file.openNextFile();
  if (!entry){
    return "";
  }
  String extension_upper = extension;
  extension_upper.toUpperCase();
  while (
      entry &&
      (
        (String(entry.name()).indexOf(extension) == -1 && String(entry.name()).indexOf(extension_upper) == -1) ||
        entry.isDirectory()
       )
      )
  {
    entry = current_file.openNextFile();vTaskDelay(1);
  }
  String file_name = entry ? String(entry.name()) : "";
  String file_name_no_path = file_name;
  if (file_name.lastIndexOf("/") != -1)
  {
    file_name_no_path = file_name.substring(file_name.lastIndexOf("/") + 1, file_name.length());
  }
  entry.close();
  return file_name_no_path;
}

int G_FILEHANDLER::count_files_in_folder_by_extension( String folder, String extension ){
  if( is_locked_for_ui() ){ 
    return 0; 
  }
  int num_files = 0;
  String file_name;
  open_folder(folder);
  while (true){
    file_name = get_next_filename(extension);
    if (file_name.length() <= 0){
      break;
    }
    ++num_files;vTaskDelay(1);
  }
  close_current_folder();
  return num_files;
}

void G_FILEHANDLER::get_files_in_folder_by_extension( String folder, String extension, char files[][100] ){
  if( is_locked_for_ui() ){ 
    return; 
  }
  open_folder(folder);
  String file_name;
  int index = 0;
  while (true){
    file_name = get_next_filename(extension);
    if (file_name.length() <= 0){
      break;
    }
    char c[100];
    file_name.toCharArray(c, sizeof(c));
    memcpy(files[index], c, sizeof(files[index]));
    ++index;vTaskDelay(1);
  }
  close_current_folder();
}


void G_FILEHANDLER::open_folder( String folder ){
  sd_state = SDState::BusyCustom;
  current_file = SD.open(folder);vTaskDelay(1);
  current_file.rewindDirectory();vTaskDelay(1);
}
int G_FILEHANDLER::open_file( String folder, String file ){
  if( is_locked_for_ui() ){ return 1; }
  close_current_folder();
  String file_path = folder + "/" + file;
  if( is_locked_for_ui() || !SD.exists(file_path) ){
    return 1;
  }
  sd_state     = SDState::BusyCustom;
  current_file = SD.open(file_path, FILE_READ);vTaskDelay(1);
  return current_file?0:2;
}
bool G_FILEHANDLER::open_gcode_file( String path ){
  current_file = SD.open(path, FILE_READ);
  if (!current_file){ 
    return false;
  }
  job_finished        = false;
  sd_state            = SDState::BusyPrinting;
  SD_ready_next       = false; 
  current_line_number = 0;
  return true;
}
bool G_FILEHANDLER::close_file(){
  if (!current_file){
    SD_ready_next       = false;
    job_finished        = true;
    current_line_number = 0;
    sd_state = SDState::Idle;
    return false;
  }
  close_current_folder();
  SD_ready_next      = false;
  current_line_number = 0;
  cursor_position     = 0;
  job_finished = true;
  return true;
}
void G_FILEHANDLER::close_current_folder(){
  current_file.close();
  sd_state = SDState::Idle;
}

void G_FILEHANDLER::reset_job_finished(){
  job_finished = false;
}
bool G_FILEHANDLER::has_job_finished(){
  return job_finished;
}
bool G_FILEHANDLER::is_locked_for_ui(){
  return is_available() ? false : true;
}
bool G_FILEHANDLER::get_is_busy(){
  return sd_state != SDState::Idle ? true : false;
}
bool G_FILEHANDLER::is_available(){
  return SDState::BusyPrinting == sd_state ? false : true;
}
String G_FILEHANDLER::get_folder_settings(){
  return String(folder_settings);
}
String G_FILEHANDLER::get_folder_gcode(){
  return String(folder_gcode);
}
bool G_FILEHANDLER::get_has_sd_card(){
  return ignore_sd ? false : has_sd_card;
}
int G_FILEHANDLER::get_error_code(){
  return error_code;
}
uint32_t G_FILEHANDLER::get_current_line_number(){
  return current_line_number;
}
bool G_FILEHANDLER::get_is_ready(){
  return filehandler_is_ready;
}
void G_FILEHANDLER::set_spi_instance( SPIClass &_spi ){
  file_spi = _spi;
}