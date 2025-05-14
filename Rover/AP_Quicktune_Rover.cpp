/*
  C++ implementation of rover quicktune based on original lua script
  https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/rover-quicktune.lua
 */


// quicktune is not performance sensitive, save flash
#pragma GCC optimize("Os")

#include "Rover.h"
#include "AP_Quicktune_Rover.h"

#if AP_QUICKTUNE_ENABLED

#define UPDATE_RATE_HZ  40           //this script updates at 40hz
#define AXIS_CHANGE_DELAY  4.0       //delay of 4 seconds between axis to allow vehicle to settle
#define PILOT_INPUT_DELAY  4.0       // gains are not updated for 4 seconds after pilot releases sticks
#define FLTD_MUL  0.5                //ATC_STR_RAT_FLTD set to 0.5 * INS_GYRO_FILTER
#define FLTT_MUL  0.5                // ATC_STR_RAT_FLTT set to 0.5 * INS_GYRO_FILTER
#define STR_RAT_FF_TURNRATE_MIN  10   // steering rate feedforward min vehicle turn rate (in radians/sec)
#define STR_RAT_FF_STEERING_MIN 0.10  // steering rate feedforward min steering output (in the range 0 to 1)
#define SPEED_FF_SPEED_MIN  0.5      // speed feedforward minimum vehicle speed (in m/s)
#define SPEED_FF_THROTTLE_MIN  0.20  // speed feedforward requires throttle output (in the range 0 to 1)

#define DEBUG_FREQ_FAST 1
#define DEBUG_FREQ_SLOW 300

const AP_Param::GroupInfo AP_Quicktune_Rover::var_info[] = {

    // @Param: RTUN_ENABLE
    // @DisplayName: Rover Quicktune enable
    // @Description: Enable quicktune system
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_ENABLE", 1, AP_Quicktune_Rover, enable, 1),

    // @Param: RTUN_AXES
    // @DisplayName: Rover Quicktune axes
    // @Description: axes to tune
    // @Bitmask: 0:Steering,1:Speed
    // @User: Standard
    AP_GROUPINFO("_AXES", 2, AP_Quicktune_Rover, axes, 3),


    // @Param: RTUN_STR_FFRATIO
    // @DisplayName: Rover Quicktune Steering Rate FeedForward ratio
    // @Description: Ratio between measured response and FF gain. Raise this to get a higher FF gain
    // @Range: 0 1.0
    // @User: Standard
    AP_GROUPINFO("_STR_FFRATIO", 3, AP_Quicktune_Rover, strFFRatio, 0.9),

    // @Param: RTUN_STR_P_RATIO
    // @DisplayName: Rover Quicktune Steering FF to P ratio
    // @Description: Ratio between steering FF and P gains. Raise this to get a higher P gain, 0 to leave P unchanged
    // @Range: 0 2.0
    // @User: Standard
    AP_GROUPINFO("_STR_P_RATIO", 4, AP_Quicktune_Rover, strPRatio, 0.45),


    // @Param: RTUN_STR_I_RATIO
    // @DisplayName: Rover Quicktune Steering FF to I ratio
    // @Description: Ratio between steering FF and I gains. Raise this to get a higher I gain, 0 to leave I unchanged
    // @Range: 0 2.0
    // @User: Standard
    AP_GROUPINFO("_STR_I_RATIO", 5, AP_Quicktune_Rover, strIRatio, 0.5),


    // @Param: RTUN_SPD_FFRATIO
    // @DisplayName: Rover Quicktune Speed FeedForward (equivalent) ratio
    // @Description: Ratio between measured response and CRUISE_THROTTLE value. Raise this to get a higher CRUISE_THROTTLE value
    // @Range: 0 1.0
    // @User: Standard
    AP_GROUPINFO("_SPD_FFRATIO", 6, AP_Quicktune_Rover, spdFFRatio, 1.0),

    // @Param: RTUN_SPD_P_RATIO
    // @DisplayName: Rover Quicktune Speed FF to P ratio
    // @Description: Ratio between speed FF and P gain. Raise this to get a higher P gain, 0 to leave P unchanged
    // @Range: 0 2.0
    // @User: Standard
    AP_GROUPINFO("_SPD_P_RATIO", 7, AP_Quicktune_Rover, spdPRatio, 1.0),


    // @Param: RTUN_SPD_I_RATIO
    // @DisplayName: Rover Quicktune Speed FF to I ratio
    // @Description: Ratio between speed FF and I gain. Raise this to get a higher I gain, 0 to leave I unchanged
    // @Range: 0 2.0
    // @User: Standard
    AP_GROUPINFO("_SPD_I_RATIO", 8, AP_Quicktune_Rover, spdIRatio, 1.0),


    // @Param: RTUN_AUTO_FILTER
    // @DisplayName: Rover Quicktune auto filter enable
    // @Description: When enabled the PID filter settings are automatically set based on INS_GYRO_FILTER
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_AUTO_FILTER", 9, AP_Quicktune_Rover, autoFilter, 1),

    // @Param: RTUN_AUTO_SAVE
    // @DisplayName: Rover Quicktune auto save
    // @Description: Number of seconds after completion of tune to auto-save. This is useful when using a 2 position switch for quicktune
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_AUTO_SAVE", 10, AP_Quicktune_Rover, autoSave, 5),

    // @Param: RTUN_RC_FUNC
    // @DisplayName: Rover Quicktune RC function
    // @Description: RCn_OPTION number to use to control tuning stop/start/save
    // @Values: 300:Scripting1, 301:Scripting2, 302:Scripting3, 303:Scripting4, 304:Scripting5, 305:Scripting6, 306:Scripting7, 307:Scripting8
    // @User: Standard
    AP_GROUPINFO("_RC_FUNC", 11, AP_Quicktune_Rover, rcFunc, 300),

    AP_GROUPEND
};

const char* AP_Quicktune_Rover::axis_names[] = {"ATC_STR_RAT", "ATC_SPEED"};
const char* AP_Quicktune_Rover::param_suffixes[] = {"FF", "P", "I", "D", "FLTT", "FLTD", "FLTE"};
const char* AP_Quicktune_Rover::params_extra[] = {"CRUISE_SPEED", "CRUISE_THROTTLE"};

AP_Quicktune_Rover::AP_Quicktune_Rover()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_Quicktune_Rover::enter()
{
    init_params_tables();
    reset_axes_done();
    get_all_params();

    //backup GCS_PID_MASK to value before tuning
    gcs_pid_mask_orig = (AP_Int16 *) AP_Param::find("GCS_PID_MASK", &gcs_ptype);

    //other vehicle parameters used by this script
    INS_GYRO_FILTER = AP_Param::find("INS_GYRO_FILTER", &gyro_ptype);
    GCS_PID_MASK = AP_Param::find("GCS_PID_MASK", &gcs_ptype);
    RCMAP_ROLL = AP_Param::find("RCMAP_ROLL", &roll_ptype);
    RCMAP_THROTTLE= AP_Param::find("RCMAP_THROTTLE", &throttle_ptype);

    sw_pos_tune = SwitchPos::MID;
    sw_pos_save = SwitchPos::HIGH;

    gcs().send_text(MAV_SEVERITY_INFO, "Rover quicktune loaded");
    return true;
}

void AP_Quicktune_Rover::update()
{
    if (enable <= 0) {
        return;
    }

    if (sw_pos == SwitchPos::LOW || !rover.arming.is_armed()) {
        // Abort and revert parameters
        if (need_restore) {
            need_restore = false;
            restore_all_params();
            //Restore gcs pid mask;
            restore_gcs_pid_mask();
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTun: gains reverted");
        }
        reset_axes_done();
        return;
    }

    if (have_pilot_input()) {
        last_pilot_input = get_time();
    }

    if (!init_done) {
        enter();
        init_done = true;
    }

    float steering_out, throttle_out;
    get_steering_and_throttle(steering_out, throttle_out);

    // Check switch position (0: low, 1: middle, 2: high)
    if (sw_pos == sw_pos_tune && (!rover.arming.is_armed() || !rover.g2.motors.active()) && get_time() > last_warning + 5) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "RTun: must be armed and moving to tune");
        last_warning = get_time();
        return;
    }

    if (sw_pos == sw_pos_save) {
        // Save all parameters
        if (need_restore) {
            need_restore = false;
            save_all_params();
            restore_gcs_pid_mask();
        }
    }

    // If not in tuning mode, exit
    if (sw_pos != sw_pos_tune) {
        return;
    }

    // Return if recently changed stages
    if (get_time() - last_axis_change < AXIS_CHANGE_DELAY) {
        return;
    }

    // Get the current axis being tuned
    const char* axis = get_current_axis();

    // If no axis is being tuned, check auto-save
    if (axis == nullptr ) {
        if (tune_done_time > 0 && autoSave > 0) {
            if (get_time() - tune_done_time > autoSave) {
                need_restore = false;
                save_all_params();
                restore_gcs_pid_mask();
                tune_done_time = 0;
            }
        }
        return;
    }
    if (!need_restore) {
        // Just starting tuning, get current values
        get_all_params();
    }

    // Return immediately if pilot has provided input recently
    if (get_time() - last_pilot_input < PILOT_INPUT_DELAY) {
        return;
    }


    // Check if filters have been set for this axis
    for (size_t i = 0; i < sizeof(filters_done) / sizeof(filters_done[0]); ++i) {
        if (strcmp(filters_done[i].name, axis) == 0 && !filters_done[i].value) {
            gcs().send_text(MAV_SEVERITY_INFO,"RTun: starting %s tune", axis);
            last_warning = get_time();
            setup_filters(axis);
        }
    }
    for (size_t i = 0; i < sizeof(gcs_pid_mask_done) / sizeof(gcs_pid_mask_done[0]); ++i) {
        // Check if GCS_PID_MASK has been set for this axis
        if (strcmp(gcs_pid_mask_done[i].name, axis) == 0 &&!gcs_pid_mask_done[i].value) {
            setup_gcs_pid_mask(axis);
        }
    }

    char pname[PARAM_MAX_NAME_SIZE];
    snprintf(pname, sizeof(pname), "%s_FF", axis);
    char message[100];
    // Feedforward tuning
    bool ff_done = false;
    if (strcmp(axis,"ATC_STR_RAT") == 0) {
        ff_done = update_steering_ff(pname);
    } else if (strcmp(axis,"ATC_SPEED") == 0) {
        ff_done = update_speed_ff(pname);
    } else {
        snprintf(message, sizeof(message), "RTun: unsupported FF tuning %s", pname);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s", message);
        ff_done = true;
    }

    if (ff_done) {
        snprintf(message, sizeof(message), "RTun: %s tuning done", pname);
        gcs().send_text(MAV_SEVERITY_NOTICE, "%s", message);
        advance_axis(axis);
    }

}

//move to next axis of tune
void AP_Quicktune_Rover::advance_axis(const char* axis)
{
    float now_sec = get_time();

    // Store the current axis being tuned
    const char* prev_axis = get_current_axis();

    // Mark the current axis as done
    for (auto& axis_status : axes_done) {
        if (strcmp(axis_status.name, axis) == 0) {
            axis_status.value = true;
            break;
        }
    }

    // Check for tuning completion
    if (prev_axis != nullptr && get_current_axis() == nullptr) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "RTun: Tuning DONE");
        tune_done_time = now_sec;
    }

    // Update the last axis change timestamp
    last_axis_change = now_sec;
}


// run steering turn rate controller feedforward calibration
// ff_pname is the FF parameter being tuned
// returns true once the tuning has completed
bool AP_Quicktune_Rover::update_steering_ff(const char* ff_pname)
{
    // Get steering and turn rate
    float steering_out, throttle_out;
    get_steering_and_throttle(steering_out, throttle_out);
    float turn_rate_rads = fabsf(rover.ahrs.get_gyro().z);

    // Update user every 5 seconds
    float now_sec = get_time();
    bool update_user = false;
    if (now_sec > ff_last_warning + 5.0f) {
        update_user = true;
        ff_last_warning = now_sec;
    }

    // Calculate percentage complete
    float turn_rate_complete_pct = (ff_turn_rate_sum / (M_PI * 2.0f)) * 100.0f;
    float time_complete_pct = (static_cast<float>(ff_turn_rate_count) / (10.0f * UPDATE_RATE_HZ)) * 100.0f;
    float complete_pct = (turn_rate_complete_pct < time_complete_pct) ? turn_rate_complete_pct : time_complete_pct;

    // Check steering and turn rate validity
    bool steering_ok = steering_out >= STR_RAT_FF_STEERING_MIN;
    bool turnrate_ok = fabsf(turn_rate_rads) > (STR_RAT_FF_TURNRATE_MIN*(M_PI / 180.0));

    if (steering_ok && turnrate_ok) {
        ff_steering_sum += steering_out;
        ff_turn_rate_sum += fabsf(turn_rate_rads);
        ff_turn_rate_count++;

        if (update_user) {
            char message[80];
            snprintf(message, sizeof(message), "RTun: %s %.0f%% complete", ff_pname, complete_pct);
            gcs().send_text(MAV_SEVERITY_INFO, "%s", message);
        }
    } else {
        if (update_user) {
            char warning[100];
            if (!steering_ok) {
                snprintf(warning, sizeof(warning),
                         "RTun: increase steering (%d%% < %d%%)",
                         static_cast<int>(steering_out * 100),
                         static_cast<int>(STR_RAT_FF_STEERING_MIN * 100));
                gcs().send_text(MAV_SEVERITY_WARNING, "%s", warning);
            } else if (!turnrate_ok) {
                snprintf(warning, sizeof(warning),
                         "RTun: increase turn rate (%d deg/s < %d)",
                         static_cast<int>(degrees(std::abs(turn_rate_rads))),
                         static_cast<int>(STR_RAT_FF_TURNRATE_MIN));
                gcs().send_text(MAV_SEVERITY_WARNING, "%s", warning);
            }
        }
    }

    // Check for completion
    if (complete_pct >= 100.0f) {
        float FF_new_gain = (ff_steering_sum / ff_turn_rate_sum) * strFFRatio;

        adjust_gain(ff_pname, FF_new_gain);

        // Set P gain
        if (strPRatio > 0) {
            char pname[PARAM_MAX_NAME_SIZE];
            strncpy(pname, ff_pname, PARAM_MAX_NAME_SIZE);
            replace_substring(pname, "_FF", "_P");
            adjust_gain(pname, FF_new_gain * strPRatio);
        }

        // Set I gain
        if (strIRatio > 0) {
            char iname[PARAM_MAX_NAME_SIZE];
            strncpy(iname, ff_pname, PARAM_MAX_NAME_SIZE);
            replace_substring(iname, "_FF", "_I");
            adjust_gain(iname, FF_new_gain * strIRatio);
        }

        return true;
    }

    return false;
}

bool AP_Quicktune_Rover::update_speed_ff(const char* ff_pname)
{
    // Variables to hold throttle and speed
    float throttle_out = 0.0f;
    float speed = 0.0f;

    // Get steering and throttle values
    float steering_out = 0.0f;
    get_steering_and_throttle(steering_out, throttle_out);

    // Get velocity in NED frame and convert to body frame
    Vector3f velocity_ned;
    if (rover.ahrs.get_velocity_NED(velocity_ned)) {
        speed = rover.ahrs.earth_to_body(velocity_ned).x;
    }

    // Update user every 5 seconds
    float now_sec = get_time();
    bool update_user = false;
    if (now_sec > ff_last_warning + 5.0f) {
        update_user = true;
        ff_last_warning = now_sec;
    }

    // Calculate percentage complete
    float complete_pct = (ff_speed_count / (10.0f * UPDATE_RATE_HZ)) * 100.0f;

    // Check throttle and speed thresholds
    bool throttle_ok = throttle_out >= SPEED_FF_THROTTLE_MIN;
    bool speed_ok = speed > SPEED_FF_SPEED_MIN;
    if (throttle_ok && speed_ok) {
        ff_throttle_sum += throttle_out;
        ff_speed_sum += speed;
        ff_speed_count += 1;

        if (update_user) {
            char msg[50];
            snprintf(msg, sizeof(msg), "RTun: %s %.0f%% complete", ff_pname, complete_pct);
            gcs().send_text(MAV_SEVERITY_INFO,"%s", msg);
        }
    } else {
        if (update_user) {
            if (!throttle_ok) {
                char msg[50];
                snprintf(msg, sizeof(msg),
                         "RTun: increase throttle (%d < %d)",
                         static_cast<int>(throttle_out * 100.0f),
                         static_cast<int>(SPEED_FF_THROTTLE_MIN * 100.0f));
                gcs().send_text(MAV_SEVERITY_WARNING,"%s", msg);
            } else if (!speed_ok) {
                char msg[50];
                snprintf(msg, sizeof(msg),
                         "RTun: increase speed (%3.1f < %3.1f)",
                         speed, SPEED_FF_SPEED_MIN);
                gcs().send_text(MAV_SEVERITY_WARNING, "%s", msg);
            }
        }
    }

    // Check for 10 seconds of data
    if (complete_pct >= 100.0f) {
        float cruise_speed_new = ff_speed_sum / ff_speed_count;
        float cruise_throttle_new =
            (ff_throttle_sum / ff_speed_count) * 100.0f * spdFFRatio.get();

        adjust_gain("CRUISE_SPEED", cruise_speed_new);
        adjust_gain("CRUISE_THROTTLE", cruise_throttle_new);

        // Calculate FF equivalent gain for P and I settings
        float speed_ff_equivalent =
            (ff_throttle_sum / ff_speed_sum) * spdFFRatio.get();

        // Set P gain
        if (spdPRatio.get() > 0.0f) {
            char pname[PARAM_MAX_NAME_SIZE];
            strncpy(pname, ff_pname, PARAM_MAX_NAME_SIZE);
            replace_substring(pname, "_FF", "_P");
            float P_new_gain = speed_ff_equivalent * spdPRatio.get();
            adjust_gain(pname, P_new_gain);
        }

        // Set I gain
        if (spdIRatio.get() > 0.0f) {
            char iname[PARAM_MAX_NAME_SIZE];
            strncpy(iname, ff_pname, PARAM_MAX_NAME_SIZE);
            replace_substring(iname, "_FF", "_I");
            float I_new_gain = speed_ff_equivalent * spdIRatio.get();
            adjust_gain(iname, I_new_gain);
        }

        return true;
    }

    return false;
}

// Replace a substring in a char array with another substring
void AP_Quicktune_Rover::replace_substring(char* str, const char* old_sub, const char* new_sub)
{
    char buffer[PARAM_MAX_NAME_SIZE];
    char* pos = strstr(str, old_sub); // Find the first occurrence of old_sub
    if (!pos) {
        return; // If old_sub is not found, do nothing
    }

    // Copy the part before the old_sub
    size_t prefix_len = pos - str;
    strncpy(buffer, str, prefix_len);
    buffer[prefix_len] = '\0';

    // Append the new_sub
    strncat(buffer, new_sub, PARAM_MAX_NAME_SIZE - strlen(buffer) - 1);

    // Append the part after the old_sub
    strncat(buffer, pos + strlen(old_sub), PARAM_MAX_NAME_SIZE - strlen(buffer) - 1);

    // Copy back to the original string
    strncpy(str, buffer, PARAM_MAX_NAME_SIZE - 1);
    str[PARAM_MAX_NAME_SIZE - 1] = '\0'; // Ensure null termination
}

//setup GCS_PID_MASK to provide real-time PID info to GCS during tuning
void AP_Quicktune_Rover::setup_gcs_pid_mask(const char* axis)
{
    if (strcmp(axis, "ATC_STR_RAT") == 0) {
        GCS_PID_MASK->set_float(1, gcs_ptype);
    } else if (strcmp(axis, "ATC_SPEED") == 0) {
        GCS_PID_MASK->set_float(2, gcs_ptype);
    } else {
        char message[100];
        snprintf(message, sizeof(message), "RTun: setup_gcs_pid_mask received unhandled axis %s", axis);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s", message);
    }

    // Mark the axis as done in gcs_pid_mask_done
    for (auto& item : gcs_pid_mask_done) {
        if (strcmp(item.name, axis) == 0) {
            item.value = true;
            break;
        }
    }
}

// Function to set up filter frequencies for a specific axis
void AP_Quicktune_Rover::setup_filters(const char* axis)
{
    if (autoFilter > 0) {
        if (strcmp(axis, "ATC_STR_RAT") == 0) {
            // Adjust the FLTT filter
            char fltt_param_name[PARAM_MAX_NAME_SIZE];
            snprintf(fltt_param_name, sizeof(fltt_param_name), "%s_FLTT", axis);
            adjust_gain(fltt_param_name, INS_GYRO_FILTER->cast_to_float(gyro_ptype) * FLTT_MUL);

            // Adjust the FLTD filter
            char fltd_param_name[PARAM_MAX_NAME_SIZE];
            snprintf(fltd_param_name, sizeof(fltd_param_name), "%s_FLTD", axis);
            adjust_gain(fltd_param_name, INS_GYRO_FILTER->cast_to_float(gyro_ptype) * FLTD_MUL);
        }
    }

    // Mark the filter setup as done for this axis
    for (size_t i = 0; i < sizeof(axis_names)/sizeof(axis_names[0]); ++i) {
        if (strcmp(axis_names[i], axis) == 0) {
            strncpy(filters_done[i].name, axis, PARAM_MAX_NAME_SIZE);
            filters_done[i].value = true;
            break;
        }
    }

}

// Function to adjust a gain, log, and notify the user
void AP_Quicktune_Rover::adjust_gain(const char* pname, float value)
{
    enum ap_var_type ptype;
    AP_Param* param = AP_Param::find(pname, &ptype);
    if (param == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "RTun: parameter not found %s", pname);
        return;
    }

    // Get the current value of the parameter
    float old_value = param->cast_to_float(ptype);

    // Set the new value
    param->set_float(value, ptype);

    // Mark the parameter as changed
    need_restore = true;
    for (int i = 0; i < MAX_PARAMS; ++i) {
        if (strcmp(pname, parameters[i].name) == 0) {
            parameters[i].changed = true;
            break;
        }
    }

    Write_RTUN(value, pname);
    gcs().send_text(MAV_SEVERITY_INFO, "RTun: adjusted %s %.3f -> %.3f", pname, old_value, value);
}

float AP_Quicktune_Rover::get_time()
{
    uint32_t millis = AP_HAL::millis();
    return millis * 0.001f;
}

const char* AP_Quicktune_Rover::get_current_axis()
{
    for (size_t i = 0; i < sizeof(axis_names) / sizeof(axis_names[0]); ++i) {
        int mask = (1 << i);  // Generate bitmask for the current axis
        // If the axis is active and not done, return the axis name
        if ((mask & axes) != 0 && !axes_done[i].value) {
            return axis_names[i];
        }
    }

    return nullptr;
}

void AP_Quicktune_Rover::restore_gcs_pid_mask()
{
    GCS_PID_MASK->set_float(gcs_pid_mask_orig->get(), gcs_ptype);
}

//check for pilot input to pause tune
bool AP_Quicktune_Rover::have_pilot_input()
{
    auto &RC = rc();
    const float roll = RC.get_roll_channel().norm_input_dz();
    const float throttle = RC.get_throttle_channel().norm_input_dz();

    if (fabsf(roll) > 0 || fabsf(throttle) > 0) {
        return true;
    }
    return false;
}

// Initialize parameter tables
void AP_Quicktune_Rover::init_params_tables()
{
    // Combine axis names with suffixes to create parameter names
    for (const char *axis : axis_names) {
        for (const char *suffix : param_suffixes) {
            char param_name[PARAM_MAX_NAME_SIZE];
            snprintf(param_name, sizeof(param_name), "%s_%s", axis, suffix);
            add_parameter(param_name, axis);
        }
    }

    // Add extra parameters
    for (const char *param : params_extra) {
        add_parameter(param, "");
    }
}

void AP_Quicktune_Rover::add_parameter(const char *name, const char *axis)
{
    if (param_count >= MAX_PARAMS) {
        return;
    }

    enum ap_var_type ptype;
    AP_Param *param = AP_Param::find(name, &ptype);
    if (param != nullptr) {
        parameters[param_count].param = param;
        strncpy(parameters[param_count].name, name, PARAM_MAX_NAME_SIZE - 1);
        parameters[param_count].name[PARAM_MAX_NAME_SIZE - 1] = '\0';

        strncpy(parameters[param_count].axis, axis, PARAM_MAX_NAME_SIZE - 1);
        parameters[param_count].axis[PARAM_MAX_NAME_SIZE - 1] = '\0';

        parameters[param_count].changed = false;
        parameters[param_count].ptype = ptype;
        param_count++;
    }
}

//get all current param values into param_saved dictionary
void AP_Quicktune_Rover::get_all_params()
{
    for (size_t i = 0; i < sizeof(parameters)/sizeof(parameters[0]) ; ++i) {
        if (parameters[i].param != nullptr) {
            float current_value = parameters[i].param->cast_to_float(parameters[i].ptype);
            strncpy(param_saved[i].name, parameters[i].name, PARAM_MAX_NAME_SIZE);
            param_saved[i].value = current_value;
        }
    }
}

//restore all param values from param_saved dictionary
void AP_Quicktune_Rover::restore_all_params()
{
    for (size_t i = 0; i < sizeof(parameters)/sizeof(parameters[0]); ++i) {
        if (parameters[i].changed) {
            float saved_value = param_saved[i].value;
            parameters[i].param->set_float(saved_value, parameters[i].ptype);
            parameters[i].changed = false;
        }
    }
}

// save all param values to storage
void AP_Quicktune_Rover::save_all_params()
{
    for (size_t i = 0; i < sizeof(parameters)/sizeof(parameters[0]); ++i) {
        if (parameters[i].changed) {
            float current_value = parameters[i].param->cast_to_float(parameters[i].ptype);
            parameters[i].param->set_float(current_value, parameters[i].ptype);
            parameters[i].param->save(true);
            strncpy(param_saved[i].name, parameters[i].name, PARAM_MAX_NAME_SIZE);
            param_saved[i].value = current_value;
            parameters[i].changed = false;
        }
    }
    gcs().send_text(MAV_SEVERITY_NOTICE, "RTun: tuning gains saved");
}

// Function to reset all axis states
void AP_Quicktune_Rover::reset_axes_done()
{
    // Iterate over all axis names and reset their associated states
    for (size_t i = 0; i < sizeof(axis_names) / sizeof(axis_names[0]); ++i) {
        const char* axis = axis_names[i];

        // Reset axes_done state
        strncpy(axes_done[i].name, axis, PARAM_MAX_NAME_SIZE-1);
        axes_done[i].name[PARAM_MAX_NAME_SIZE - 1] = '\0';
        axes_done[i].value = false;  // Reset to false

        // Reset filters_done state
        strncpy(filters_done[i].name, axis, PARAM_MAX_NAME_SIZE-1);
        filters_done[i].name[PARAM_MAX_NAME_SIZE - 1] = '\0';
        filters_done[i].value = false;  // Reset to false

        // Reset gcs_pid_mask_done state
        strncpy(gcs_pid_mask_done[i].name, axis, PARAM_MAX_NAME_SIZE-1);
        gcs_pid_mask_done[i].name[PARAM_MAX_NAME_SIZE - 1] = '\0';
        gcs_pid_mask_done[i].value = false;  // Reset to false
    }

    tune_done_time = 0;

    // Initialize the FF (Feed Forward) functions
    init_steering_ff();
    init_speed_ff();
}

// initialise steering ff tuning
void AP_Quicktune_Rover::init_steering_ff()
{
    ff_steering_sum = 0;
    ff_turn_rate_sum = 0;
    ff_turn_rate_count = 0;
}

//initialise speed ff tuning
void AP_Quicktune_Rover::init_speed_ff()
{
    ff_throttle_sum = 0;
    ff_speed_sum = 0;
    ff_speed_count = 0;
}


// get steering and throttle (-1 to +1) (for use by scripting.)
bool AP_Quicktune_Rover::get_steering_and_throttle(float& steering, float& throttle)
{
    steering = rover.g2.motors.get_steering() / 4500.0;
    throttle = rover.g2.motors.get_throttle() * 0.01;
    return true;
}

int AP_Quicktune_Rover::snprintf(char* str, size_t size, const char *format, ...) const
{
    va_list ap;
    va_start(ap, format);
    int res = hal.util->vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

void AP_Quicktune_Rover::update_switch_pos(const  RC_Channel::AuxSwitchPos ch_flag)
{
    sw_pos = SwitchPos(ch_flag);
}

// get slew rate parameter for an axis
float AP_Quicktune_Rover::get_slew_rate(const char* axis) const
{
    auto &attitude_control = *AR_AttitudeControl::get_singleton();
    if (strcmp(axis, "ATC_STR_RAT")) {
        return attitude_control.get_steering_rate_pid().get_pid_info().slew_rate;
    } else if (strcmp(axis, "ATC_SPEED")) {
        return attitude_control.get_throttle_speed_pid_info().slew_rate;
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "RTUN: get_slew_rate unsupported axis:%s", axis);
    return 0.0;
}

// @LoggerMessage: RTUN
// @Description: Quicktune
// @Field: SRate: slew rate
// @Field: Gain: test gain for current axis and PID element
// @Field: Param: name of parameter being being tuned
void AP_Quicktune_Rover::Write_RTUN(float gain, const char* param)
{
#if HAL_LOGGING_ENABLED
    float srate = get_slew_rate(get_current_axis());
    AP::logger().Write("RTUN","SRate,Gain,Param", "ffN",
                                srate,
                                gain,
                                param);
#endif
}
#endif //AP_QUICKTUNE_ENABLED