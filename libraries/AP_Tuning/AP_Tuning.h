#pragma once

#include "AP_Tuning_config.h"

#if AP_TUNING_ENABLED

#include <AP_Param/AP_Param.h>

#include "stdint.h"

/*
  transmitter tuning library. Meant to be subclassed per vehicle type
 */
class AP_Tuning
{
public:
    struct tuning_set {
        uint8_t set;
        uint8_t num_parms;
        const uint8_t *parms;
    };

    struct tuning_name {
        uint8_t parm;
        const char *name;
    };

    // constructor
    AP_Tuning(const struct tuning_set *sets, const struct tuning_name *names) :
        tuning_sets(sets),
        tuning_names(names) {
        AP_Param::setup_object_defaults(this, var_info);
    }
    
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // update function called on new radio frames
    void check_input(uint8_t flightmode);

    // base parameter number for tuning sets of parameters in one flight
    const uint8_t set_base = 100;
    
private:
    AP_Int8 channel;
    AP_Int16 channel_min;
    AP_Int16 channel_max;
    AP_Int8 selector;
    AP_Float range;
    AP_Int8 mode_revert;
    AP_Float error_threshold;

    // when selector was triggered
    uint32_t selector_start_ms;

    // are we waiting for channel mid-point?
    bool mid_point_wait;

    // last input from tuning channel
    float last_channel_value;
    
    // mid-value for current parameter
    float center_value;

    uint32_t last_check_ms;

    // last tuning value scaled
    float old_value;
    
    void Log_Write_Parameter_Tuning(float value);
    
    // the parameter we are tuning
    uint8_t current_parm;

    // current index into the parameter set
    uint8_t current_parm_index;

    // current parameter set
    uint8_t current_set;

    // true if tune has changed
    bool changed;

    // mask of params in set that need reverting
    uint32_t need_revert;
    
    // last flight mode we were tuning in
    uint8_t last_flightmode;

    const tuning_set *tuning_sets;
    const tuning_name *tuning_names;
    
    void check_selector_switch(void);
    void re_center(void);
    void next_parameter(void);
    void save_parameters(void);
    void revert_parameters(void);
    const char *get_tuning_name(uint8_t parm);

protected:
    // virtual functions that must be implemented in vehicle subclass
    virtual AP_Float *get_param_pointer(uint8_t parm) = 0;
    virtual void save_value(uint8_t parm) = 0;
    virtual void reload_value(uint8_t parm) = 0;
    virtual void set_value(uint8_t parm, float value) = 0;

    // parmset is in vehicle subclass var table
    AP_Int16 parmset;
};

#endif  // AP_TUNING_ENABLED
