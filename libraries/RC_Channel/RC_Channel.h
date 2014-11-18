// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	RC_Channel.h
/// @brief	RC_Channel manager, with EEPROM-backed storage of constants.

#ifndef __RC_CHANNEL_H__
#define __RC_CHANNEL_H__

#include <AP_Common.h>
#include <AP_Param.h>

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
        _high(1),
        _ch_out(ch_out) {
		AP_Param::setup_object_defaults(this, var_info);
        if (ch_out < RC_MAX_CHANNELS) {
            rc_ch[ch_out] = this;
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

    // setup the control preferences
    void        set_range(int16_t low, int16_t high);
    void        set_range_out(int16_t low, int16_t high);
    void        set_angle(int16_t angle);
    void        set_reverse(bool reverse);
    bool        get_reverse(void) const;
    void        set_default_dead_zone(int16_t dzone);
    
    // get the channel number
    uint8_t     get_ch_out(void) const { return _ch_out; };

    // get the center stick position expressed as a control_in value
    int16_t     get_control_mid() const;

    // read input from APM_RC - create a control_in value
    void        set_pwm(int16_t pwm);
    static void set_pwm_all(void);
    void        set_pwm_no_deadzone(int16_t pwm);

    // return a limit PWM value
    uint16_t    get_limit_pwm(LimitValue limit) const;

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

    int16_t                                         pwm_to_angle_dz(uint16_t dead_zone);
    int16_t                                         pwm_to_angle();

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. Ignore deadzone.
     */
    float                                           norm_input();

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. Take into account the deadzone
    */
    float                                           norm_input_dz();

    uint8_t                                         percent_input();
    float                                           norm_output();
    int16_t                                         angle_to_pwm();
    int16_t                                         pwm_to_range();
    int16_t                                         pwm_to_range_dz(uint16_t dead_zone);
    int16_t                                         range_to_pwm();

    void                                            output() const;
    void                                            output_trim() const;
    static void                                     output_trim_all();
    static void                                     setup_failsafe_trim_all();
    uint16_t                                        read() const;
    void                                            input();
    void                                            enable_out();
    void                                            disable_out();

    static const struct AP_Param::GroupInfo         var_info[];

    static RC_Channel *rc_channel(uint8_t i);

private:
    AP_Int8         _reverse;
    AP_Int16        _dead_zone;
    uint8_t         _type;
    int16_t         _high;
    int16_t         _low;
    int16_t         _high_out;
    int16_t         _low_out;

    static RC_Channel *rc_ch[RC_MAX_CHANNELS];

protected:
    uint8_t         _ch_out;
};

// This is ugly, but it fixes poorly architected library
#include "RC_Channel_aux.h"

#endif
