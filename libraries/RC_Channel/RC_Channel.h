// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	RC_Channel.h
/// @brief	RC_Channel manager, with EEPROM-backed storage of constants.
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#define RC_CHANNEL_TYPE_ANGLE       0
#define RC_CHANNEL_TYPE_RANGE       1
#define RC_CHANNEL_TYPE_ANGLE_RAW   2

#define RC_MAX_CHANNELS 14

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
        _high_in(1),
        _ch_out(ch_out) 
    {
		  AP_Param::setup_object_defaults(this, var_info);
        if (ch_out < RC_MAX_CHANNELS) {
            _rc_ch[ch_out] = this;
        }
    }

    // used to get min/max/trim limit value based on _reverse
    enum LimitValue {
        RC_CHANNEL_LIMIT_TRIM,
        RC_CHANNEL_LIMIT_MIN,
        RC_CHANNEL_LIMIT_MAX
    };

    // setup min and max radio values in CLI
    void        update_min_max();
    void        zero_min_max();

    // startup
    void        load_eeprom(void);
    void        save_eeprom(void);
    void        save_trim(void);
    void        set_type(uint8_t t);
    void        set_type_in(uint8_t t);
    void        set_type_out(uint8_t t);

    // setup the control preferences
    void        set_range(int16_t low, int16_t high);
    void        set_range_out(int16_t low, int16_t high);
    void        set_range_in(int16_t low, int16_t high);
    void        set_angle(int16_t angle);
    void        set_angle_in(int16_t angle);
    void        set_angle_out(int16_t angle);
    void        set_reverse(bool reverse);
    bool        get_reverse(void) const;
    void        set_default_dead_zone(int16_t dzone);
    uint16_t    get_dead_zone(void) const { return _dead_zone; }
    
    // get the channel number
    uint8_t     get_ch_out(void) const { return _ch_out; }

    // get the center stick position expressed as a control_in value
    int16_t     get_control_mid() const;

    // read input from APM_RC - create a control_in value
    void        set_pwm(int16_t pwm);
    static void set_pwm_all(void);
    void        set_pwm_no_deadzone(int16_t pwm);

    // return a limit PWM value
    uint16_t    get_limit_pwm(LimitValue limit) const;

    // call after first set_pwm
    void        trim();

    // generate PWM from servo_out value
    void        calc_pwm(void);

    int16_t     pwm_to_angle_dz_trim(uint16_t dead_zone, uint16_t trim);
    int16_t     pwm_to_angle_dz(uint16_t dead_zone);
    int16_t     pwm_to_angle();

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. Ignore deadzone.
     */
    float       norm_input();

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. Take into account the deadzone
    */
    float       norm_input_dz();

    uint8_t     percent_input();
    float       norm_output();
    int16_t     angle_to_pwm();
    int16_t     pwm_to_range();
    int16_t     pwm_to_range_dz(uint16_t dead_zone);
    int16_t     range_to_pwm();
    void        output() const;
    void        output_trim() const;
    static void output_trim_all();
    static void setup_failsafe_trim_mask(uint16_t chmask);
    static void setup_failsafe_trim_all();
    uint16_t    read() const;
    void        input();
    void        enable_out();
    void        disable_out();

    static const struct AP_Param::GroupInfo         var_info[];

    static RC_Channel *rc_channel(uint8_t i);

    static RC_Channel **rc_channel_array(void) 
    {
        return _rc_ch;
    }
    
    bool       in_trim_dz();

    int16_t    get_radio_in() const { return _radio_in;}
    void       set_radio_in(int16_t val){_radio_in = val;}

    int16_t    get_control_in() const { return _control_in;}
    void       set_control_in(int16_t val) { _control_in = val;}

    int16_t    get_servo_out() const {return _servo_out;}
    void       set_servo_out(int16_t val){_servo_out = val;}

    int16_t    get_pwm_out() const { return _pwm_out;}

    int16_t    get_radio_out() const { return _radio_out;}
    void       set_radio_out(int16_t val){ _radio_out = val;}

    int16_t    get_radio_min() const {return _radio_min.get();}
    void       set_radio_min(int16_t val){_radio_min = val;}

    int16_t    get_radio_max() const {return _radio_max.get();}
    void       set_radio_max(int16_t val){_radio_max = val;}

    int16_t    get_radio_trim() const { return _radio_trim.get();}
    void       set_radio_trim(int16_t val) { _radio_trim.set(val);}
    void       save_radio_trim() { _radio_trim.save();}
    
    bool min_max_configured()
    {
        return _radio_min.configured() && _radio_max.configured();
    }
    
private:

    // pwm is stored here
    int16_t     _radio_in;
    // value generated from PWM
    int16_t     _control_in;
    // current values to the servos - degrees * 100 (approx assuming servo is -45 to 45 degrees except [3] is 0 to 100
    int16_t     _servo_out;
    // PWM is without the offset from radio_min
    int16_t     _pwm_out;
    int16_t     _radio_out;

    AP_Int16    _radio_min;
    AP_Int16    _radio_trim;
    AP_Int16    _radio_max;

    AP_Int8     _reverse;
    AP_Int16    _dead_zone;
    uint8_t     _type_in;
    int16_t     _high_in;
    int16_t     _low_in;
    uint8_t     _type_out;
    int16_t     _high_out;
    int16_t     _low_out;

    static RC_Channel *_rc_ch[RC_MAX_CHANNELS];

protected:
    uint8_t     _ch_out;
};

// This is ugly, but it fixes poorly architected library
#include "RC_Channel_aux.h"
