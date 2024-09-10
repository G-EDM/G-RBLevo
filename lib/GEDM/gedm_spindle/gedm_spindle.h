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
#include "driver/mcpwm.h"

#include "Config.h"
#include "Error.h"
#include "Probe.h"
#include "System.h"

class G_EDM_SPINDLE {

    protected:

        int dir_pin;
        int step_pin;
        bool is_running;
        bool dir_inverted;
        int frequency;
        int default_frequency;
        int backup_frequency;

    public:

        G_EDM_SPINDLE();
        void setup_spindle( int _dir_pin, int _step_pin );
        void start_spindle( void );
        void stop_spindle( void );
        void set_spindle_direction( bool direction );
        bool spindle_is_running( void );
        void reverse_direction( void );
        bool dir_is_inverted( void );
        bool set_speed( int __frequency, bool backup = true );
        bool restore_backup_speed( void );
        bool relative_rpm_up( float add_rpm );
        int get_speed( void );
        void reset_spindle( void );
        int rpm_to_frequency( float rpm );
        float frequency_to_rpm( float _frequency );
        void backup_spindle_speed( void );
        void restore_spindle_speed( void );
        float convert_mm_min_to_frequency( float mm_min );
        float convert_frequency_to_mm_min( float frequency );
};


extern G_EDM_SPINDLE gedm_spindle;