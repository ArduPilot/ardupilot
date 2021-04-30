/// @file	RC_Channel.h
/// @brief	RC_Channel manager, with EEPROM-backed storage of constants.
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AuxFunc/AP_AuxFunc.h>

#define NUM_RC_CHANNELS 16

/// @class	RC_Channel
/// @brief	Object managing one RC channel
class RC_Channel {
public:
    friend class SRV_Channels;
    friend class RC_Channels;
    // Constructor
    RC_Channel(void);

    enum ChannelType {
        RC_CHANNEL_TYPE_ANGLE = 0,
        RC_CHANNEL_TYPE_RANGE = 1,
    };

    // setup the control preferences
    void        set_range(uint16_t high);
    uint16_t    get_range() const { return high_in; }
    void        set_angle(uint16_t angle);
    bool        get_reverse(void) const;
    void        set_default_dead_zone(int16_t dzone);
    uint16_t    get_dead_zone(void) const { return dead_zone; }

    // get the center stick position expressed as a control_in value
    int16_t     get_control_mid() const;

    // read input from hal.rcin - create a control_in value
    bool        update(void);
    void        recompute_pwm_no_deadzone();

    // calculate an angle given dead_zone and trim. This is used by the quadplane code
    // for hover throttle
    int16_t     pwm_to_angle_dz_trim(uint16_t dead_zone, uint16_t trim) const;

    // return a normalised input for a channel, in range -1 to 1,
    // centered around the channel trim. Ignore deadzone.
    float       norm_input() const;

    // return a normalised input for a channel, in range -1 to 1,
    // centered around the channel trim. Take into account the deadzone
    float       norm_input_dz() const;

    // return a normalised input for a channel, in range -1 to 1,
    // ignores trim and deadzone
    float       norm_input_ignore_trim() const;

    uint8_t     percent_input() const;
    int16_t     pwm_to_range() const;
    int16_t     pwm_to_range_dz(uint16_t dead_zone) const;

    static const struct AP_Param::GroupInfo var_info[];

    // return true if input is within deadzone of trim
    bool       in_trim_dz() const;

    int16_t    get_radio_in() const { return radio_in;}
    void       set_radio_in(int16_t val) {radio_in = val;}

    int16_t    get_control_in() const { return control_in;}
    void       set_control_in(int16_t val) { control_in = val;}

    void       clear_override();
    void       set_override(const uint16_t v, const uint32_t timestamp_ms);
    bool       has_override() const;

    int16_t    stick_mixing(const int16_t servo_in);

    // get control input with zero deadzone
    int16_t    get_control_in_zero_dz(void) const;

    int16_t    get_radio_min() const {return radio_min.get();}
    void       set_radio_min(int16_t val) { radio_min = val;}

    int16_t    get_radio_max() const {return radio_max.get();}
    void       set_radio_max(int16_t val) {radio_max = val;}

    int16_t    get_radio_trim() const { return radio_trim.get();}
    void       set_radio_trim(int16_t val) { radio_trim.set(val);}
    void       save_radio_trim() { radio_trim.save();}

    void       set_and_save_trim() { radio_trim.set_and_save_ifchanged(radio_in);}

    // set and save trim if changed
    void       set_and_save_radio_trim(int16_t val) { radio_trim.set_and_save_ifchanged(val);}

    ChannelType get_type(void) const { return type_in; }

    AP_Int16    option; // e.g. activate EPM gripper / enable fence

    // auxiliary switch support
    void init_aux();
    bool read_aux();

    bool read_3pos_switch(AP_AuxFunc::SwitchPos &ret) const WARN_IF_UNUSED;
    bool read_6pos_switch(int8_t& position) WARN_IF_UNUSED;
    AP_AuxFunc::SwitchPos get_aux_switch_pos() const;

    // pwm value under which we consider that Radio value is invalid
    static const uint16_t RC_MIN_LIMIT_PWM = 900;
    // pwm value above which we consider that Radio value is invalid
    static const uint16_t RC_MAX_LIMIT_PWM = 2200;

    // pwm value above which we condider that Radio min value is invalid
    static const uint16_t RC_CALIB_MIN_LIMIT_PWM = 1300;
    // pwm value under which we condider that Radio max value is invalid
    static const uint16_t RC_CALIB_MAX_LIMIT_PWM = 1700;

    // pwm value above which the switch/button will be invoked:
    static const uint16_t AUX_SWITCH_PWM_TRIGGER_HIGH = 1800;
    // pwm value below which the switch/button will be disabled:
    static const uint16_t AUX_SWITCH_PWM_TRIGGER_LOW = 1200;

    // pwm value above which the option will be invoked:
    static const uint16_t AUX_PWM_TRIGGER_HIGH = 1700;
    // pwm value below which the option will be disabled:
    static const uint16_t AUX_PWM_TRIGGER_LOW = 1300;

protected:

    typedef int8_t modeswitch_pos_t;
    virtual void mode_switch_changed(modeswitch_pos_t new_pos) {
        // no action by default (e.g. Tracker, Sub, who do their own thing)
    };


private:

    // pwm is stored here
    int16_t     radio_in;

    // value generated from PWM normalised to configured scale
    int16_t    control_in;

    AP_Int16    radio_min;
    AP_Int16    radio_trim;
    AP_Int16    radio_max;

    AP_Int8     reversed;
    AP_Int16    dead_zone;

    ChannelType type_in;
    int16_t     high_in;

    // the input channel this corresponds to
    uint8_t     ch_in;

    // overrides
    uint16_t override_value;
    uint32_t last_override_time;

    int16_t pwm_to_angle() const;
    int16_t pwm_to_angle_dz(uint16_t dead_zone) const;

    // Structure used to detect and debounce switch changes
    struct {
        int8_t debounce_position = -1;
        int8_t current_position = -1;
        uint32_t last_edge_time_ms;
    } switch_state;

    void reset_mode_switch();
    void read_mode_switch();
    bool debounce_completed(int8_t position);

};


/*
  class	RC_Channels. Hold the full set of RC_Channel objects
*/
class RC_Channels {
public:
    friend class SRV_Channels;
    friend class RC_Channel;
    // constructor
    RC_Channels(void);

    void init(void);

    // get singleton instance
    static RC_Channels *get_singleton() {
        return _singleton;
    }

    static const struct AP_Param::GroupInfo var_info[];

    // compatability functions for Plane:
    static uint16_t get_radio_in(const uint8_t chan) {
        RC_Channel *c = _singleton->channel(chan);
        if (c == nullptr) {
            return 0;
        }
        return c->get_radio_in();
    }
    static RC_Channel *rc_channel(const uint8_t chan) {
        return _singleton->channel(chan);
    }
    //end compatability functions for Plane

    // this function is implemented in the child class in the vehicle
    // code
    virtual RC_Channel *channel(uint8_t chan) = 0;

    uint8_t get_radio_in(uint16_t *chans, const uint8_t num_channels); // reads a block of chanel radio_in values starting from channel 0
                                                                       // returns the number of valid channels

    static uint8_t get_valid_channel_count(void);                      // returns the number of valid channels in the last read
    static int16_t get_receiver_rssi(void);                            // returns [0, 255] for receiver RSSI (0 is no link) if present, otherwise -1
    static int16_t get_receiver_link_quality(void);                         // returns 0-100 % of last 100 packets received at receiver are valid
    bool read_input(void);                                             // returns true if new input has been read in
    static void clear_overrides(void);                                 // clears any active overrides
    static bool receiver_bind(const int dsmMode);                      // puts the receiver in bind mode if present, returns true if success
    static void set_override(const uint8_t chan, const int16_t value, const uint32_t timestamp_ms = 0); // set a channels override value
    static bool has_active_overrides(void);                            // returns true if there are overrides applied that are valid

    // returns a mask indicating which channels have overrides.  Bit 0
    // is RC channel 1.  Beware this is not a cheap call.
    static uint16_t get_override_mask();

    class RC_Channel *find_channel_for_option(const AP_AuxFunc::Function option);
    bool duplicate_options_exist();
    AP_AuxFunc::SwitchPos get_channel_pos(const uint8_t rcmapchan) const;

    void init_aux_all();
    void read_aux_all();

    // mode switch handling
    void reset_mode_switch();
    virtual void read_mode_switch();

    // has_valid_input should be pure-virtual when Plane is converted
    virtual bool has_valid_input() const { return false; };

    virtual RC_Channel *get_arming_channel(void) const { return nullptr; };

    bool gcs_overrides_enabled() const { return _gcs_overrides_enabled; }
    void set_gcs_overrides_enabled(bool enable) {
        _gcs_overrides_enabled = enable;
        if (!_gcs_overrides_enabled) {
            clear_overrides();
        }
    }

    // should we ignore RC failsafe bits from receivers?
    bool ignore_rc_failsafe(void) const {
        return get_singleton() != nullptr && (_options & uint32_t(Option::IGNORE_FAILSAFE));
    }

    // should we add a pad byte to Fport data
    bool fport_pad(void) const {
        return get_singleton() != nullptr && (_options & uint32_t(Option::FPORT_PAD));
    }

    // returns true if we should pass through data for crsf telemetry
    bool crsf_custom_telemetry(void) const {
        return get_singleton() != nullptr && (_options & uint32_t(Option::CRSF_CUSTOM_TELEMETRY));
    }

    // should a channel reverse option affect aux switches
    bool switch_reverse_allowed(void) const {
        return get_singleton() != nullptr && (_options & uint32_t(Option::ALLOW_SWITCH_REV));
    }

    bool ignore_overrides() const {
        return _options & uint32_t(Option::IGNORE_OVERRIDES);
    }

    bool ignore_receiver() const {
        return _options & uint32_t(Option::IGNORE_RECEIVER);
    }

    bool log_raw_data() const {
        return _options & uint32_t(Option::LOG_DATA);
    }
    
    bool arming_check_throttle() const {
        return _options & uint32_t(Option::ARMING_CHECK_THROTTLE);
    }

    bool arming_skip_checks_rpy() const {
        return _options & uint32_t(Option::ARMING_SKIP_CHECK_RPY);
    }

    bool suppress_crsf_message(void) const {
        return get_singleton() != nullptr && (_options & uint32_t(Option::SUPPRESS_CRSF_MESSAGE));
    }



    // returns true if overrides should time out.  If true is returned
    // then returned_timeout_ms will contain the timeout in
    // milliseconds, with 0 meaning overrides are disabled.
    bool get_override_timeout_ms(uint32_t &returned_timeout_ms) const {
        const float value = _override_timeout.get();
        if (is_positive(value)) {
            returned_timeout_ms = uint32_t(value * 1e3f);
            return true;
        }
        if (is_zero(value)) {
            returned_timeout_ms = 0;
            return true;
        }
        // overrides will not time out
        return false;
    }

    // get mask of enabled protocols
    uint32_t enabled_protocols() const;

    // returns true if we have had a direct detach RC reciever, does not include overrides
    bool has_had_rc_receiver() const { return _has_had_rc_receiver; }

    /*
      get the RC input PWM value given a channel number.  Note that
      channel numbers start at 1, as this API is designed for use in
      LUA
    */
    bool get_pwm(uint8_t channel, uint16_t &pwm) const;

    uint32_t last_input_ms() const { return last_update_ms; };

    // check if flight mode channel is assigned RC option
    // return true if assigned
    bool flight_mode_channel_conflicts_with_rc_option() const;

    // flight_mode_channel_number must be overridden in vehicle specific code
    virtual int8_t flight_mode_channel_number() const = 0;

protected:

    enum class Option {
        IGNORE_RECEIVER         = (1U << 0), // RC receiver modules
        IGNORE_OVERRIDES        = (1U << 1), // MAVLink overrides
        IGNORE_FAILSAFE         = (1U << 2), // ignore RC failsafe bits
        FPORT_PAD               = (1U << 3), // pad fport telem output
        LOG_DATA                = (1U << 4), // log rc input bytes
        ARMING_CHECK_THROTTLE   = (1U << 5), // run an arming check for neutral throttle
        ARMING_SKIP_CHECK_RPY   = (1U << 6), // skip the an arming checks for the roll/pitch/yaw channels
        ALLOW_SWITCH_REV        = (1U << 7), // honor the reversed flag on switches
        CRSF_CUSTOM_TELEMETRY   = (1U << 8), // use passthrough data for crsf telemetry
        SUPPRESS_CRSF_MESSAGE   = (1U << 9), // suppress CRSF mode/rate message for ELRS systems
    };

    void new_override_received() {
        has_new_overrides = true;
    }

private:
    static RC_Channels *_singleton;
    // this static arrangement is to avoid static pointers in AP_Param tables
    static RC_Channel *channels;

    uint32_t last_update_ms;
    bool has_new_overrides;
    bool _has_had_rc_receiver; // true if we have had a direct detach RC reciever, does not include overrides

    AP_Float _override_timeout;
    AP_Int32  _options;
    AP_Int32  _protocols;

    RC_Channel *flight_mode_channel() const;

    // Allow override by default at start
    bool _gcs_overrides_enabled = true;
};

RC_Channels &rc();
