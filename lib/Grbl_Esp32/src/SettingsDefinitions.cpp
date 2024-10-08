#include "Grbl.h"

StringSetting* startup_line_0;
StringSetting* startup_line_1;
StringSetting* build_info;

IntSetting* pulse_microseconds;
IntSetting* stepper_idle_lock_time;
IntSetting* direction_delay_microseconds;
IntSetting* enable_delay_microseconds;

AxisMaskSetting* step_invert_mask;
AxisMaskSetting* dir_invert_mask;
// TODO Settings - need to call st_generate_step_invert_masks;
AxisMaskSetting* homing_dir_mask;
AxisMaskSetting* homing_squared_axes;
AxisMaskSetting* stallguard_debug_mask;

FlagSetting* step_enable_invert;
FlagSetting* limit_invert;
FlagSetting* probe_invert;
FlagSetting* report_inches;
FlagSetting* soft_limits;
// TODO Settings - need to check for HOMING_ENABLE
FlagSetting* hard_limits;
// TODO Settings - need to call limits_init;
FlagSetting* homing_enable;
// TODO Settings - also need to clear, but not set, soft_limits
FlagSetting* laser_mode;
// TODO Settings - also need to call my_spindle->init;
IntSetting* laser_full_power;

IntSetting*   status_mask;
FloatSetting* junction_deviation;
FloatSetting* arc_tolerance;

FloatSetting*    homing_debounce;
FloatSetting*    homing_pulloff;
AxisMaskSetting* homing_cycle[MAX_N_AXIS];
FloatSetting*    spindle_pwm_freq;
FloatSetting*    rpm_max;
FloatSetting*    rpm_min;
FloatSetting*    spindle_delay_spinup;
FloatSetting*    spindle_delay_spindown;
FlagSetting*     spindle_enbl_off_with_zero_speed;
FlagSetting*     spindle_enable_invert;
FlagSetting*     spindle_output_invert;

FloatSetting* spindle_pwm_off_value;
FloatSetting* spindle_pwm_min_value;
FloatSetting* spindle_pwm_max_value;
IntSetting*   spindle_pwm_bit_precision;

EnumSetting* spindle_type;

AxisSettings* x_axis_settings;
AxisSettings* y_axis_settings;
AxisSettings* z_axis_settings;
AxisSettings* a_axis_settings;
AxisSettings* b_axis_settings;
AxisSettings* c_axis_settings;

AxisSettings* axis_settings[MAX_N_AXIS];

typedef struct {
    const char* name;
    float       steps_per_mm;
    float       max_rate;
    float       acceleration;
    float       max_travel;
    float       home_mpos;
    float       run_current;
    float       hold_current;
    uint16_t    microsteps;
    uint16_t    stallguard;
} axis_defaults_t;
axis_defaults_t axis_defaults[] = { { "X",
                                      DEFAULT_X_STEPS_PER_MM,
                                      DEFAULT_X_MAX_RATE,
                                      DEFAULT_X_ACCELERATION,
                                      DEFAULT_X_MAX_TRAVEL,
                                      DEFAULT_X_HOMING_MPOS,
                                      DEFAULT_X_CURRENT,
                                      DEFAULT_X_HOLD_CURRENT,
                                      DEFAULT_X_MICROSTEPS,
                                      DEFAULT_X_STALLGUARD },
                                    { "Y",
                                      DEFAULT_Y_STEPS_PER_MM,
                                      DEFAULT_Y_MAX_RATE,
                                      DEFAULT_Y_ACCELERATION,
                                      DEFAULT_Y_MAX_TRAVEL,
                                      DEFAULT_Y_HOMING_MPOS,
                                      DEFAULT_Y_CURRENT,
                                      DEFAULT_Y_HOLD_CURRENT,
                                      DEFAULT_Y_MICROSTEPS,
                                      DEFAULT_Y_STALLGUARD },
                                    { "Z",
                                      DEFAULT_Z_STEPS_PER_MM,
                                      DEFAULT_Z_MAX_RATE,
                                      DEFAULT_Z_ACCELERATION,
                                      DEFAULT_Z_MAX_TRAVEL,
                                      DEFAULT_Z_HOMING_MPOS,
                                      DEFAULT_Z_CURRENT,
                                      DEFAULT_Z_HOLD_CURRENT,
                                      DEFAULT_Z_MICROSTEPS,
                                      DEFAULT_Z_STALLGUARD },
                                    { "A",
                                      DEFAULT_A_STEPS_PER_MM,
                                      DEFAULT_A_MAX_RATE,
                                      DEFAULT_A_ACCELERATION,
                                      DEFAULT_A_MAX_TRAVEL,
                                      DEFAULT_A_HOMING_MPOS,
                                      DEFAULT_A_CURRENT,
                                      DEFAULT_A_HOLD_CURRENT,
                                      DEFAULT_A_MICROSTEPS,
                                      DEFAULT_A_STALLGUARD },
                                    { "B",
                                      DEFAULT_B_STEPS_PER_MM,
                                      DEFAULT_B_MAX_RATE,
                                      DEFAULT_B_ACCELERATION,
                                      DEFAULT_B_MAX_TRAVEL,
                                      DEFAULT_B_HOMING_MPOS,
                                      DEFAULT_B_CURRENT,
                                      DEFAULT_B_HOLD_CURRENT,
                                      DEFAULT_B_MICROSTEPS,
                                      DEFAULT_B_STALLGUARD },
                                    { "C",
                                      DEFAULT_C_STEPS_PER_MM,
                                      DEFAULT_C_MAX_RATE,
                                      DEFAULT_C_ACCELERATION,
                                      DEFAULT_C_MAX_TRAVEL,
                                      DEFAULT_C_HOMING_MPOS,
                                      DEFAULT_C_CURRENT,
                                      DEFAULT_C_HOLD_CURRENT,
                                      DEFAULT_C_MICROSTEPS,
                                      DEFAULT_C_STALLGUARD } };

// Construct e.g. X_MAX_RATE from axisName "X" and tail "_MAX_RATE"
// in dynamically allocated memory that will not be freed.

static const char* makename(const char* axisName, const char* tail) {
    char* retval = (char*)malloc(strlen(axisName) + strlen(tail) + 2);

    strcpy(retval, axisName);
    strcat(retval, "/");
    return strcat(retval, tail);
}

static bool checkStartupLine(char* value) {
    if (!value) {  // No POST functionality
        return true;
    }
    if (sys.state != State::Idle) {
        return false;
    }
    return gcode_core.gc_execute_line(value) == Error::Ok;
}

static bool postMotorSetting(char* value) {
    if (!value) {
        motor_manager.motors_read_settings();
    }
    return true;
}

static bool checkSpindleChange(char* val) {
    return true;
}

// Generates a string like "122" from axisNum 2 and base 120
static const char* makeGrblName(int axisNum, int base) {
    // To omit A,B,C axes:
    // if (axisNum > 2) return NULL;
    char buf[4];
    snprintf(buf, 4, "%d", axisNum + base);
    char* retval = (char*)malloc(strlen(buf));
    return strcpy(retval, buf);
}

void make_coordinate(CoordIndex index, const char* name) {
    float coord_data[MAX_N_AXIS] = { 0.0 };
    auto  coord                  = new Coordinates(name);
    coords[index]                = coord;
    if (!coord->load()) {
        coords[index]->setDefault();
    }
}
void make_settings() {
    Setting::init();

    // Propagate old coordinate system data to the new format if necessary.
    // G54 - G59 work coordinate systems, G28, G30 reference positions, etc
    make_coordinate(CoordIndex::G54, "G54");
    make_coordinate(CoordIndex::G55, "G55");
    make_coordinate(CoordIndex::G56, "G56");
    make_coordinate(CoordIndex::G57, "G57");
    make_coordinate(CoordIndex::G58, "G58");
    make_coordinate(CoordIndex::G59, "G59");
    make_coordinate(CoordIndex::G28, "G28");
    make_coordinate(CoordIndex::G30, "G30");

    // Create the axis settings in the order that people are
    // accustomed to seeing.
    int              axis;
    axis_defaults_t* def;
    for (axis = 0; axis < MAX_N_AXIS; axis++) {
        def                 = &axis_defaults[axis];
        axis_settings[axis] = new AxisSettings(def->name);
    }
    x_axis_settings = axis_settings[X_AXIS];
    y_axis_settings = axis_settings[Y_AXIS];
    z_axis_settings = axis_settings[Z_AXIS];
    a_axis_settings = axis_settings[A_AXIS];
    b_axis_settings = axis_settings[B_AXIS];
    c_axis_settings = axis_settings[C_AXIS];
    for (axis = MAX_N_AXIS - 1; axis >= 0; axis--) {
        def          = &axis_defaults[axis];
        auto setting = new IntSetting(EXTENDED, WG, makeGrblName(axis, 170), makename(def->name, "StallGuard"), def->stallguard, -64, 255, postMotorSetting);
        setting->setAxis(axis);
        axis_settings[axis]->stallguard = setting;
    }
    for (axis = MAX_N_AXIS - 1; axis >= 0; axis--) {
        def          = &axis_defaults[axis];
        auto setting = new IntSetting(EXTENDED, WG, makeGrblName(axis, 160), makename(def->name, "Microsteps"), def->microsteps, 0, 256, postMotorSetting);
        setting->setAxis(axis);
        axis_settings[axis]->microsteps = setting;
    }
    for (axis = MAX_N_AXIS - 1; axis >= 0; axis--) {
        def          = &axis_defaults[axis];
        auto setting = new FloatSetting(EXTENDED, WG, makeGrblName(axis, 150), makename(def->name, "Current/Hold"), def->hold_current, 0.05, 20.0, postMotorSetting);  // Amps
        setting->setAxis(axis);
        axis_settings[axis]->hold_current = setting;
    }
    for (axis = MAX_N_AXIS - 1; axis >= 0; axis--) {
        def          = &axis_defaults[axis];
        auto setting = new FloatSetting(EXTENDED, WG, makeGrblName(axis, 140), makename(def->name, "Current/Run"), def->run_current, 0.0, 20.0, postMotorSetting);  // Amps
        setting->setAxis(axis);
        axis_settings[axis]->run_current = setting;
    }
    for (axis = MAX_N_AXIS - 1; axis >= 0; axis--) {
        def          = &axis_defaults[axis];
        auto setting = new FloatSetting(GRBL, WG, makeGrblName(axis, 130), makename(def->name, "MaxTravel"), def->max_travel, 0, 100000.0);
        setting->setAxis(axis);
        axis_settings[axis]->max_travel = setting;
    }

    for (axis = MAX_N_AXIS - 1; axis >= 0; axis--) {
        def          = &axis_defaults[axis];
        auto setting = new FloatSetting(EXTENDED, WG, NULL, makename(def->name, "Home/Mpos"), def->home_mpos, -100000.0, 100000.0);
        setting->setAxis(axis);
        axis_settings[axis]->home_mpos = setting;
    }

    for (axis = MAX_N_AXIS - 1; axis >= 0; axis--) {
        def = &axis_defaults[axis];
        auto setting =
            new FloatSetting(GRBL, WG, makeGrblName(axis, 120), makename(def->name, "Acceleration"), def->acceleration, 1.0, 100000.0);
        setting->setAxis(axis);
        axis_settings[axis]->acceleration = setting;
    }
    for (axis = MAX_N_AXIS - 1; axis >= 0; axis--) {
        def          = &axis_defaults[axis];
        auto setting = new FloatSetting(GRBL, WG, makeGrblName(axis, 110), makename(def->name, "MaxRate"), def->max_rate, 1.0, 100000.0);
        setting->setAxis(axis);
        axis_settings[axis]->max_rate = setting;
    }
    for (axis = MAX_N_AXIS - 1; axis >= 0; axis--) {
        def = &axis_defaults[axis];
        auto setting = new FloatSetting(GRBL, WG, makeGrblName(axis, 100), makename(def->name, "StepsPerMm"), def->steps_per_mm, 1.0, 100000.0);
        setting->setAxis(axis);
        axis_settings[axis]->steps_per_mm = setting;
    }


    startup_line_0      = new StringSetting(EXTENDED, WG, "N0", "GCode/Line0", "", checkStartupLine);
    startup_line_1      = new StringSetting(EXTENDED, WG, "N1", "GCode/Line1", "", checkStartupLine);
    homing_pulloff      = new FloatSetting(GRBL, WG, "27", "Homing/Pulloff", DEFAULT_HOMING_PULLOFF, 0, 1000);
    homing_debounce     = new FloatSetting(GRBL, WG, "26", "Homing/Debounce", DEFAULT_HOMING_DEBOUNCE_DELAY, 0, 10000);
    homing_squared_axes = new AxisMaskSetting(EXTENDED, WG, NULL, "Homing/Squared", DEFAULT_HOMING_SQUARED_AXES);
    homing_dir_mask     = new AxisMaskSetting(GRBL, WG, "23", "Homing/DirInvert", DEFAULT_HOMING_DIR_MASK);
    homing_enable       = new FlagSetting(GRBL, WG, "22", "Homing/Enable", DEFAULT_HOMING_ENABLE);
    hard_limits         = new FlagSetting(GRBL, WG, "21", "Limits/Hard", DEFAULT_HARD_LIMIT_ENABLE);
    soft_limits         = new FlagSetting(GRBL, WG, "20", "Limits/Soft", DEFAULT_SOFT_LIMIT_ENABLE, NULL);
    build_info          = new StringSetting(EXTENDED, WG, NULL, "Firmware/Build", "");
    report_inches       = new FlagSetting(GRBL, WG, "13", "Report/Inches", DEFAULT_REPORT_INCHES);
   
    // TODO Settings - also need to clear, but not set, soft_limits
    arc_tolerance                = new FloatSetting(GRBL, WG, "12", "GCode/ArcTolerance", DEFAULT_ARC_TOLERANCE, 0, 1);
    junction_deviation           = new FloatSetting(GRBL, WG, "11", "GCode/JunctionDeviation", DEFAULT_JUNCTION_DEVIATION, 0, 10);
    status_mask                  = new IntSetting(GRBL, WG, "10", "Report/Status", DEFAULT_STATUS_REPORT_MASK, 0, 3);
    probe_invert                 = new FlagSetting(GRBL, WG, "6", "Probe/Invert", DEFAULT_INVERT_PROBE_PIN);
    limit_invert                 = new FlagSetting(GRBL, WG, "5", "Limits/Invert", DEFAULT_INVERT_LIMIT_PINS);
    step_enable_invert           = new FlagSetting(GRBL, WG, "4", "Stepper/EnableInvert", DEFAULT_INVERT_ST_ENABLE);
    dir_invert_mask              = new AxisMaskSetting(GRBL, WG, "3", "Stepper/DirInvert", DEFAULT_DIRECTION_INVERT_MASK, postMotorSetting);
    step_invert_mask             = new AxisMaskSetting(GRBL, WG, "2", "Stepper/StepInvert", DEFAULT_STEPPING_INVERT_MASK, postMotorSetting);
    stepper_idle_lock_time       = new IntSetting(GRBL, WG, "1", "Stepper/IdleTime", DEFAULT_STEPPER_IDLE_LOCK_TIME, 0, 255);
    pulse_microseconds           = new IntSetting(GRBL, WG, "0", "Stepper/Pulse", DEFAULT_STEP_PULSE_MICROSECONDS, 3, 1000);
    direction_delay_microseconds = new IntSetting(EXTENDED, WG, NULL, "Stepper/Direction/Delay", STEP_PULSE_DELAY, 0, 1000);
    enable_delay_microseconds    = new IntSetting(EXTENDED, WG, NULL, "Stepper/Enable/Delay", DEFAULT_STEP_ENABLE_DELAY, 0, 1000);  // microseconds
    stallguard_debug_mask        = new AxisMaskSetting(EXTENDED, WG, NULL, "Report/StallGuard", 0, postMotorSetting);
    
    homing_cycle[5] = new AxisMaskSetting(EXTENDED, WG, NULL, "Homing/Cycle5", DEFAULT_HOMING_CYCLE_5);
    homing_cycle[4] = new AxisMaskSetting(EXTENDED, WG, NULL, "Homing/Cycle4", DEFAULT_HOMING_CYCLE_4);
    homing_cycle[3] = new AxisMaskSetting(EXTENDED, WG, NULL, "Homing/Cycle3", DEFAULT_HOMING_CYCLE_3);
    homing_cycle[2] = new AxisMaskSetting(EXTENDED, WG, NULL, "Homing/Cycle2", DEFAULT_HOMING_CYCLE_2);
    homing_cycle[1] = new AxisMaskSetting(EXTENDED, WG, NULL, "Homing/Cycle1", DEFAULT_HOMING_CYCLE_1);
    homing_cycle[0] = new AxisMaskSetting(EXTENDED, WG, NULL, "Homing/Cycle0", DEFAULT_HOMING_CYCLE_0);
    
}
