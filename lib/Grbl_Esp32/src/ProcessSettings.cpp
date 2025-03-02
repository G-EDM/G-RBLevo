#include "Grbl.h"
#include <map>
#include "Regex.h"

// WG Readable and writable as guest
// WU Readable and writable as user and admin
// WA Readable as user and admin, writable as admin

void settings_restore(uint8_t restore_flag) {

    if (restore_flag & SettingsRestore::Defaults) {
        bool restore_startup = restore_flag & SettingsRestore::StartupLines;
        for (Setting* s = Setting::List; s; s = s->next()) {
            if (!s->getDescription()) {
                const char* name = s->getName();
                if (restore_startup) {  // all settings get restored
                    s->setDefault();
                } else if ((strcmp(name, "Line0") != 0) && (strcmp(name, "Line1") != 0)) {  // non startup settings get restored
                    s->setDefault();
                }
            }
        }
    }
    if (restore_flag & SettingsRestore::Parameters) {
        for (auto idx = CoordIndex::Begin; idx < CoordIndex::End; ++idx) {
            coords[idx]->setDefault();
        }
    }
}

// Get settings values from non volatile storage into memory
void load_settings() {
    for (Setting* s = Setting::List; s; s = s->next()) {
        s->load();
    }
}

extern void make_settings();
extern void make_grbl_commands();


void settings_init() {
    make_settings();
    make_grbl_commands();
    load_settings();
}



Error show_grbl_help(const char* value) {
    return Error::Ok;
}

Error report_gcode(const char* value) {
    return Error::Ok;
}

void show_grbl_settings(type_t type, bool wantAxis) {
}
Error report_normal_settings(const char* value) {
    return Error::Ok;
}
Error report_extended_settings(const char* value) {
    return Error::Ok;
}
Error list_grbl_names(const char* value) {
    return Error::Ok;
}
Error list_settings(const char* value) {
    return Error::Ok;
}
Error list_changed_settings(const char* value) {
    return Error::Ok;
}
Error list_commands(const char* value) {

    return Error::Ok;
}
Error toggle_check_mode(const char* value) {
    // Perform reset when toggling off. Check g-code mode should only work if Grbl
    // is idle and ready, regardless of alarm locks. This is mainly to keep things
    // simple and consistent.
    if (sys.state == State::CheckMode) {
        mc_reset();
    } else {
        if (sys.state != State::Idle) {
            return Error::IdleError;  // Requires no alarm mode.
        }
        sys.state = State::CheckMode;
    }
    return Error::Ok;
}
Error disable_alarm_lock(const char* value) {
    if (sys.state == State::Alarm) {
        sys.state = State::Idle;
        // Don't run startup script. Prevents stored moves in startup from causing accidents.
    }  // Otherwise, no effect.
    return Error::Ok;
}
Error report_ngc(const char* value) {
    return Error::Ok;
}
Error home(int cycle) {
    if (homing_enable->get() == false) {
        return Error::SettingDisabled;
    }

    sys.state = State::Homing;  // Set system state variable

    mc_homing_cycle(cycle);

    if (!sys.abort) {             // Execute startup scripts after successful homing.
        sys.state = State::Idle;  // Set to IDLE when complete.
        //st_go_idle();             // Set steppers to the settings idle state before returning.
        if (cycle == HOMING_CYCLE_ALL) {
            char line[128];
            system_execute_startup(line);
        }
    }
    return Error::Ok;
}
Error home_all(const char* value) {
    return home(HOMING_CYCLE_ALL);
}
Error home_x(const char* value) {
    return home(bit(X_AXIS));
}
Error home_y(const char* value) {
    return home(bit(Y_AXIS));
}
Error home_z(const char* value) {
    return home(bit(Z_AXIS));
}
Error home_a(const char* value) {
    return home(bit(A_AXIS));
}
Error home_b(const char* value) {
    return home(bit(B_AXIS));
}
Error home_c(const char* value) {
    return home(bit(C_AXIS));
}
Error sleep_grbl(const char* value) {
    sys_rt_exec_state.bit.sleep = true;
    return Error::Ok;
}
Error get_report_build_info(const char* value) {
    if (!value) {
        return Error::Ok;
    }
    return Error::InvalidStatement;
}

std::map<const char*, uint8_t, cmp_str> restoreCommands = {
#ifdef ENABLE_RESTORE_DEFAULT_SETTINGS
    { "$", SettingsRestore::Defaults },   { "settings", SettingsRestore::Defaults },
#endif
#ifdef ENABLE_RESTORE_CLEAR_PARAMETERS
    { "#", SettingsRestore::Parameters }, { "gcode", SettingsRestore::Parameters },
#endif
#ifdef ENABLE_RESTORE_WIPE_ALL
    { "*", SettingsRestore::All },        { "all", SettingsRestore::All },
#endif
    { "@", SettingsRestore::Wifi },       { "wifi", SettingsRestore::Wifi },
};
Error restore_settings(const char* value) {
    if (!value) {
        return Error::InvalidStatement;
    }
    auto it = restoreCommands.find(value);
    if (it == restoreCommands.end()) {
        return Error::InvalidStatement;
    }
    settings_restore(it->second);
    return Error::Ok;
}

Error showState(const char* value) {
    return Error::Ok;
}
Error doJog(const char* value) {
    // For jogging, you must give gc_execute_line() a line that
    // begins with $J=.  There are several ways we can get here,
    // including  $J, $J=xxx, [J]xxx.  For any form other than
    // $J without =, we reconstruct a $J= line for gc_execute_line().
    if (!value) {
        return Error::InvalidStatement;
    }
    char jogLine[DEFAULT_LINE_BUFFER_SIZE];
    strcpy(jogLine, "$J=");
    strcat(jogLine, value);
    return gcode_core.gc_execute_line(jogLine);
}

const char* alarmString(ExecAlarm alarmNumber) {
    auto it = AlarmNames.find(alarmNumber);
    return it == AlarmNames.end() ? NULL : it->second;
}

Error listAlarms(const char* value) {
    return Error::Ok;
}

const char* errorString(Error errorNumber) {
    auto it = ErrorNames.find(errorNumber);
    return it == ErrorNames.end() ? NULL : it->second;
}

Error listErrors(const char* value) {
    return Error::Ok;
}

Error motor_disable(const char* value) {
    char* s;
    if (value == NULL) {
        value = "\0";
    }

    s = strdup(value);
    s = trim(s);

    int32_t convertedValue;
    char*   endptr;
    if (*s == '\0') {
        convertedValue = 255;  // all axes
    } else {
        convertedValue = strtol(s, &endptr, 10);
        if (endptr == s || *endptr != '\0') {
            // Try to convert as an axis list
            convertedValue = 0;
            auto axisNames = String("XYZABC");
            while (*s) {
                int index = axisNames.indexOf(toupper(*s++));
                if (index < 0) {
                    return Error::BadNumberFormat;
                }
                convertedValue |= bit(index);
            }
        }
    }
    motor_manager.motors_set_disable(true, convertedValue);
    return Error::Ok;
}

// Commands use the same syntax as Settings, but instead of setting or
// displaying a persistent value, a command causes some action to occur.
// That action could be anything, from displaying a run-time parameter
// to performing some system state change.  Each command is responsible
// for decoding its own value string, if it needs one.
void make_grbl_commands() {
    new GrblCommand("J", "Jog", doJog, anyState);
    new GrblCommand("$", "GrblSettings/List", report_normal_settings, anyState);
    new GrblCommand("X", "Alarm/Disable", disable_alarm_lock, anyState);
    new GrblCommand("#", "GCode/Offsets", report_ngc, anyState);
    new GrblCommand("H", "Home", home_all, anyState);
    new GrblCommand("MD", "Motor/Disable", motor_disable, anyState);

#ifdef HOMING_SINGLE_AXIS_COMMANDS
    new GrblCommand("HX", "Home/X", home_x, anyState);
    new GrblCommand("HY", "Home/Y", home_y, anyState);
    new GrblCommand("HZ", "Home/Z", home_z, anyState);
    new GrblCommand("HA", "Home/A", home_a, anyState);
    new GrblCommand("HB", "Home/B", home_b, anyState);
    new GrblCommand("HC", "Home/C", home_c, anyState);
#endif
};

// normalize_key puts a key string into canonical form -
// without whitespace.
// start points to a null-terminated string.
// Returns the first substring that does not contain whitespace.
// Case is unchanged because comparisons are case-insensitive.
char* normalize_key(char* start) {
    char c;

    // In the usual case, this loop will exit on the very first test,
    // because the first character is likely to be non-white.
    // Null ('\0') is not considered to be a space character.
    while (isspace(c = *start) && c != '\0') {
        ++start;
    }

    // start now points to either a printable character or end of string
    if (c == '\0') {
        return start;
    }

    // Having found the beginning of the printable string,
    // we now scan forward until we find a space character.
    char* end;
    for (end = start; (c = *end) != '\0' && !isspace(c); end++) {}

    // end now points to either a whitespace character of end of string
    // In either case it is okay to place a null there
    *end = '\0';

    return start;
}

// This is the handler for all forms of settings commands,
// $..= and [..], with and without a value.
Error do_command_or_setting(const char* key, char* value) {
    // If value is NULL, it means that there was no value string, i.e.
    // $key without =, or [key] with nothing following.
    // If value is not NULL, but the string is empty, that is the form
    // $key= with nothing following the = .  It is important to distinguish
    // those cases so that you can say "$N0=" to clear a startup line.

    // First search the settings list by text name.  If found, set a new
    // value if one is given, otherwise display the current value
    for (Setting* s = Setting::List; s; s = s->next()) {
        if (strcasecmp(s->getName(), key) == 0) {
            if (value) {
                return s->setStringValue(value);
            } else {
                return Error::Ok;
            }
        }
    }

    // Then search the setting list by compatible name.  If found, set a new
    // value if one is given, otherwise display the current value in compatible mode
    for (Setting* s = Setting::List; s; s = s->next()) {
        if (s->getGrblName() && strcasecmp(s->getGrblName(), key) == 0) {
            if (value) {
                return s->setStringValue(value);
            } else {
                return Error::Ok;
            }
        }
    }
    // If we did not find a setting, look for a command.  Commands
    // handle values internally; you cannot determine whether to set
    // or display solely based on the presence of a value.
    for (Command* cp = Command::List; cp; cp = cp->next()) {
        if ((strcasecmp(cp->getName(), key) == 0) || (cp->getGrblName() && strcasecmp(cp->getGrblName(), key) == 0)) {
            return cp->action(value);
        }
    }

    // If we did not find an exact match and there is no value,
    // indicating a display operation, we allow partial matches
    // and display every possibility.  This only applies to the
    // text form of the name, not to the nnn and ESPnnn forms.
    Error retval = Error::InvalidStatement;
    if (!value) {
        auto lcKey = String(key);
        lcKey.toLowerCase();
        bool found = false;
        for (Setting* s = Setting::List; s; s = s->next()) {
            auto lcTest = String(s->getName());
            lcTest.toLowerCase();

            if (regexMatch(lcKey.c_str(), lcTest.c_str())) {
                const char* displayValue = s->getStringValue();
                found = true;
            }
        }
        if (found) {
            return Error::Ok;
        }
    }
    return Error::InvalidStatement;
}

Error system_execute_line(char* line) {

    char* value;
    if (*line++ == '[') {  // [ESPxxx] form
        value = strrchr(line, ']');
        if (!value) {
            // Missing ] is an error in this form
            return Error::InvalidStatement;
        }
        // ']' was found; replace it with null and set value to the rest of the line.
        *value++ = '\0';
        // If the rest of the line is empty, replace value with NULL.
        if (*value == '\0') {
            value = NULL;
        }
    } else {
        // $xxx form
        value = strchr(line, '=');
        if (value) {
            // $xxx=yyy form.
            *value++ = '\0';
        }
    }

    char* key = normalize_key(line);

    // At this point there are three possibilities for value
    // NULL - $xxx without =
    // NULL - [ESPxxx] with nothing after ]
    // empty string - $xxx= with nothing after
    // non-empty string - [ESPxxx]yyy or $xxx=yyy
    return do_command_or_setting(key, value);
}

void system_execute_startup(char* line) {
    char  gcline[256];
    strncpy(gcline, startup_line_0->get(), 255);
    if (*gcline) {
        gcode_core.gc_execute_line(gcline);
    }
    strncpy(gcline, startup_line_1->get(), 255);
    if (*gcline) {
        gcode_core.gc_execute_line(gcline);
    }
}
