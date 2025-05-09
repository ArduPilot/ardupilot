/*
  Quick tune library for rover
 */
#pragma once

#include "Rover.h"

#include <AP_Param/AP_Param.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Quicktune/AP_Quicktune.h>

#if AP_QUICKTUNE_ENABLED
#define MAX_PARAMS 15
#define PARAM_MAX_NAME_SIZE 20

class AP_Quicktune_Rover
{
    struct PACKED param_rtun {
        AP_Param *param;
        char axis[PARAM_MAX_NAME_SIZE];
        bool changed;
        char name[PARAM_MAX_NAME_SIZE];
        ap_var_type ptype;
    };

    // Array to store saved parameters
    struct PACKED param_saved {
        char name[PARAM_MAX_NAME_SIZE];
        float value;
    };

    struct PACKED status_rtun {
        char name[PARAM_MAX_NAME_SIZE];
        bool value;
    };

    struct PACKED param_ext {
        AP_Param *param;
        ap_var_type ptype;
    };

public:
    static const char* axis_names[];
    static const char* param_suffixes[];
    static const char* params_extra[];

    // need a constructor for parameters
    AP_Quicktune_Rover();

    // Does not allow copies
    CLASS_NO_COPY(AP_Quicktune_Rover);

    // methods that affect movement of the vehicle in this mode
    void update();
    void update_switch_pos(const  RC_Channel::AuxSwitchPos ch_flag);

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // Low, Mid and High must be in the same positions as they are in RC_Channel::AuxSwitchPos
    enum class SwitchPos : uint8_t {
        LOW,
        MID,
        HIGH,
        NONE,
    };
    AP_Int8 enable;
    AP_Int8 axes;
    AP_Float strFFRatio;
    AP_Float strPRatio;
    AP_Float strIRatio;
    AP_Float spdFFRatio;
    AP_Float spdPRatio;
    AP_Float spdIRatio;
    AP_Int8 autoFilter;
    AP_Int8 autoSave;
    AP_Int8 rcFunc;
    AP_Int16 *gcs_pid_mask_orig;
    AP_Param *INS_GYRO_FILTER, *GCS_PID_MASK, *RCMAP_ROLL, *RCMAP_THROTTLE;
    ap_var_type gyro_ptype, gcs_ptype, roll_ptype, throttle_ptype;

    uint32_t last_warning, last_axis_change, last_pilot_input, tune_done_time, ff_last_warning, last_debug_warning;
    //int sw_pos;
    SwitchPos sw_pos; //Switch pos to be set by aux func
    SwitchPos sw_pos_tune;
    SwitchPos sw_pos_save;
    bool need_restore;
    param_rtun parameters[MAX_PARAMS];
    param_saved param_saved[MAX_PARAMS];
    status_rtun axes_done[2];
    status_rtun filters_done[2];
    status_rtun gcs_pid_mask_done[2];
    param_ext param_extras[5];
    bool init_done;

    size_t param_count;

    // feed forward tuning related local variables
    float ff_throttle_sum = 0;               // total throttle recorded during speed FF tuning (divided by count to calc average)
    float ff_speed_sum = 0;                  //total speed recorded during speed FF tuning (divided by count to calc average)
    float ff_speed_count = 0;                // number of speed and throttle samples taken during FF tuning
    float ff_steering_sum = 0;               // total steering input recorded during steering rate FF tuning (divided by count to calc average)
    float ff_turn_rate_sum = 0;              // total turn rate recorded during steering rate FF tuning (divided by count to calc average)
    float ff_turn_rate_count = 0;            // number of steering and turn rate samples taken during FF tuning

    bool enter();
    void replace_substring(char* str, const char* old_sub, const char* new_sub);
    bool get_steering_and_throttle(float& steering, float& throttle);
    int snprintf(char* str, size_t size, const char *format, ...) const;
    bool have_pilot_input();
    void init_params_tables();
    void add_parameter(const char *name, const char *axis);
    void get_all_params();
    void restore_all_params();
    void save_all_params();
    void reset_axes_done();
    void restore_gcs_pid_mask();
    const char* get_current_axis();
    float get_time();
    void setup_filters(const char* axis);
    void adjust_gain(const char* pname, float value);
    void setup_gcs_pid_mask(const char* axis);
    bool update_steering_ff(const char* ff_pname);
    bool update_speed_ff(const char* ff_pname);
    void advance_axis(const char* axis);
    void init_steering_ff();
    void init_speed_ff();
    float get_slew_rate(const char* axis) const;
    void Write_RTUN(float gain, const char* param);
};

#endif // AP_QUICKTUNE_ENABLED