// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	RC_Channel.h
/// @brief	RC_Channel manager, with EEPROM-backed storage of constants.

#ifndef RC_Channel_h
#define RC_Channel_h

#include <AP_Common.h>
#include <APM_RC.h>

/// @class	RC_Channel
/// @brief	Object managing one RC channel
class RC_Channel {
public:
    /// Constructor
    ///
    /// @param key      EEPROM storage key for the channel trim parameters.
    /// @param name     Optional name for the group.
    ///
    RC_Channel(uint8_t ch_out) :
        scale_output(1.0),
        _filter(false),
        _high(1),
        _ch_out(ch_out) {
		if (_reverse == 0) {
			_reverse = 1;
		}
    }

    // setup min and max radio values in CLI
    void        update_min_max();
    void        zero_min_max();

    // startup
    void        load_eeprom(void);
    void        save_eeprom(void);
    void        save_trim(void);
    void        set_filter(bool filter);
    void        set_type(uint8_t t);

    // setup the control preferences
    void        set_range(int16_t low, int16_t high);
    void        set_range_out(int16_t low, int16_t high);
    void        set_angle(int16_t angle);
    void        set_reverse(bool reverse);
    bool        get_reverse(void);
    void        set_dead_zone(int16_t dzone);

    // read input from APM_RC - create a control_in value
    void        set_pwm(int16_t pwm);

    // pwm is stored here
    int16_t        radio_in;

    // call after first set_pwm
    void        trim();

    // did our read come in 50µs below the min?
    bool        get_failsafe(void);

    // value generated from PWM
    int16_t         control_in;

    int16_t         control_mix(float value);

    // current values to the servos - degrees * 100 (approx assuming servo is -45 to 45 degrees except [3] is 0 to 100
    int16_t        servo_out;

    // generate PWM from servo_out value
    void        calc_pwm(void);

    // PWM is without the offset from radio_min
    int16_t         pwm_out;
    int16_t         radio_out;

    AP_Int16        radio_min;
    AP_Int16        radio_trim;
    AP_Int16        radio_max;

    // includes offset from PWM
    //int16_t   get_radio_out(void);

    int16_t                                         pwm_to_angle();
    float                                           norm_input();
    float                                           norm_output();
    int16_t                                         angle_to_pwm();
    int16_t                                         pwm_to_range();
    int16_t                                         range_to_pwm();

    float                                           scale_output;
    static void                                     set_apm_rc(APM_RC_Class * apm_rc);
    void                                            output();
    void                                            enable_out();
    static APM_RC_Class *                           _apm_rc;

    static const struct AP_Param::GroupInfo         var_info[];

private:
    bool            _filter;
    AP_Int8         _reverse;
    AP_Int16        _dead_zone;
    uint8_t         _type;
    int16_t         _high;
    int16_t         _low;
    int16_t         _high_out;
    int16_t         _low_out;
    uint8_t         _ch_out;
};

// This is ugly, but it fixes compilation on arduino
#include "RC_Channel_aux.h"

#endif
