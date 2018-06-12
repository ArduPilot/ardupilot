/// @file	RC_Channel.h
/// @brief	RC_Channel manager, with EEPROM-backed storage of constants.
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#define RC_CHANNEL_TYPE_ANGLE       0
#define RC_CHANNEL_TYPE_RANGE       1

#define NUM_RC_CHANNELS 16

/// @class	RC_Channel
/// @brief	Object managing one RC channel
class RC_Channel {
public:
    friend class SRV_Channels;
    friend class RC_Channels;
    // Constructor
    RC_Channel(void);

    // used to get min/max/trim limit value based on _reverse
    enum LimitValue {
        RC_CHANNEL_LIMIT_TRIM,
        RC_CHANNEL_LIMIT_MIN,
        RC_CHANNEL_LIMIT_MAX
    };

    // setup the control preferences
    void        set_range(uint16_t high);
    void        set_angle(uint16_t angle);
    bool        get_reverse(void) const;
    void        set_default_dead_zone(int16_t dzone);
    uint16_t    get_dead_zone(void) const { return dead_zone; }
    
    // get the center stick position expressed as a control_in value
    int16_t     get_control_mid() const;

    // read input from hal.rcin - create a control_in value
    void        set_pwm(int16_t pwm);
    void        recompute_pwm_no_deadzone();

    // calculate an angle given dead_zone and trim. This is used by the quadplane code
    // for hover throttle
    int16_t     pwm_to_angle_dz_trim(uint16_t dead_zone, uint16_t trim);

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
    int16_t     pwm_to_range();
    int16_t     pwm_to_range_dz(uint16_t dead_zone);

    // read the input value from hal.rcin for this channel
    uint16_t    read() const;

    static const struct AP_Param::GroupInfo var_info[];

    // return true if input is within deadzone of trim
    bool       in_trim_dz();

    int16_t    get_radio_in() const { return radio_in;}
    void       set_radio_in(int16_t val) {radio_in = val;}

    int16_t    get_control_in() const { return control_in;}
    void       set_control_in(int16_t val) { control_in = val;}

    void       clear_override();
    void       set_override(const uint16_t v, const uint32_t timestamp_us=0);
    bool       has_override() const;

    // get control input with zero deadzone
    int16_t     get_control_in_zero_dz(void);
    
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
    
    bool min_max_configured() const;
    
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

    uint8_t     type_in;
    int16_t     high_in;

    // the input channel this corresponds to
    uint8_t     ch_in;

    // overrides
    uint16_t override_value;
    uint32_t last_override_time;

    // bits set when channel has been identified as configured
    static uint32_t configured_mask;

    int16_t pwm_to_angle();
    int16_t pwm_to_angle_dz(uint16_t dead_zone);
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

    static const struct AP_Param::GroupInfo var_info[];

    static RC_Channel *rc_channel(uint8_t chan) {
        return (chan < NUM_RC_CHANNELS)?&channels[chan]:nullptr;
    }

    static uint16_t get_radio_in(const uint8_t chan); // returns the last read radio_in value from a chan, 0 if the channel is out of range
    static uint8_t get_radio_in(uint16_t *chans, const uint8_t num_channels); // reads a block of chanel radio_in values starting from channel 0
                                                                              // returns the number of valid channels

    static uint8_t get_valid_channel_count(void);                      // returns the number of valid channels in the last read
    static int16_t get_receiver_rssi(void);                            // returns [0, 255] for receiver RSSI (0 is no link) if present, otherwise -1
    static bool read_input(void);                                      // returns true if new input has been read in
    static void clear_overrides(void);                                 // clears any active overrides
    static bool receiver_bind(const int dsmMode);                      // puts the reciever in bind mode if present, returns true if success
    static void set_override(const uint8_t chan, const int16_t value, const uint32_t timestamp_ms = 0); // set a channels override value
    static bool has_active_overrides(void);                            // returns true if there are overrides applied that are valid

private:
    // this static arrangement is to avoid static pointers in AP_Param tables
    static RC_Channel *channels;
    static bool has_new_overrides;
    static AP_Float *override_timeout;
    RC_Channel obj_channels[NUM_RC_CHANNELS];
    AP_Float _override_timeout;
};
