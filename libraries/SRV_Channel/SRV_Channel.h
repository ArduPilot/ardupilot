/*
  control of servo output ranges, trim and servo reversal. This can
  optionally be used to provide separation of input and output channel
  ranges so that RCn_MIN, RCn_MAX, RCn_TRIM and RCn_REV only apply to
  the input side of RC_Channel

  It works by running servo output calculations as normal, then
  re-mapping the output according to the servo MIN/MAX/TRIM/REV from
  this object

  Only 4 channels of ranges are defined as those match the input
  channels for R/C sticks
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <AP_Common/Bitmask.h>

#define NUM_SERVO_CHANNELS 16

class SRV_Channels;

/*
  class SRV_Channel. The class SRV_Channels contains an array of
  SRV_Channel objects. This is done to fit within the AP_Param limit
  of 64 parameters per object.
*/
class SRV_Channel {
public:
    friend class SRV_Channels;
    // constructor
    SRV_Channel(void);

    static const struct AP_Param::GroupInfo var_info[];

    typedef enum
    {
        k_none                  = 0,            ///< disabled
        k_manual                = 1,            ///< manual, just pass-thru the RC in signal
        k_flap                  = 2,            ///< flap
        k_flap_auto             = 3,            ///< flap automated
        k_aileron               = 4,            ///< aileron
        k_unused1               = 5,            ///< unused function
        k_mount_pan             = 6,            ///< mount yaw (pan)
        k_mount_tilt            = 7,            ///< mount pitch (tilt)
        k_mount_roll            = 8,            ///< mount roll
        k_mount_open            = 9,            ///< mount open (deploy) / close (retract)
        k_cam_trigger           = 10,           ///< camera trigger
        k_egg_drop              = 11,           ///< egg drop
        k_mount2_pan            = 12,           ///< mount2 yaw (pan)
        k_mount2_tilt           = 13,           ///< mount2 pitch (tilt)
        k_mount2_roll           = 14,           ///< mount2 roll
        k_mount2_open           = 15,           ///< mount2 open (deploy) / close (retract)
        k_dspoiler1             = 16,           ///< differential spoiler 1 (left wing)
        k_dspoiler2             = 17,           ///< differential spoiler 2 (right wing)
        k_aileron_with_input    = 18,            ///< aileron, with rc input
        k_elevator              = 19,            ///< elevator
        k_elevator_with_input   = 20,            ///< elevator, with rc input
        k_rudder                = 21,            ///< secondary rudder channel
        k_sprayer_pump          = 22,            ///< crop sprayer pump channel
        k_sprayer_spinner       = 23,            ///< crop sprayer spinner channel
        k_flaperon1             = 24,            ///< flaperon, left wing
        k_flaperon2             = 25,            ///< flaperon, right wing
        k_steering              = 26,            ///< ground steering, used to separate from rudder
        k_parachute_release     = 27,            ///< parachute release
        k_gripper               = 28,            ///< gripper
        k_landing_gear_control  = 29,            ///< landing gear controller
        k_engine_run_enable     = 30,            ///< engine kill switch, used for gas airplanes and helicopters
        k_heli_rsc              = 31,            ///< helicopter RSC output
        k_heli_tail_rsc         = 32,            ///< helicopter tail RSC output
        k_motor1                = 33,            ///< these allow remapping of copter motors
        k_motor2                = 34,
        k_motor3                = 35,
        k_motor4                = 36,
        k_motor5                = 37,
        k_motor6                = 38,
        k_motor7                = 39,
        k_motor8                = 40,
        k_motor_tilt            = 41,            ///< tiltrotor motor tilt control
        k_rcin1                 = 51,            ///< these are for pass-thru from arbitrary rc inputs
        k_rcin2                 = 52,
        k_rcin3                 = 53,
        k_rcin4                 = 54,
        k_rcin5                 = 55,
        k_rcin6                 = 56,
        k_rcin7                 = 57,
        k_rcin8                 = 58,
        k_rcin9                 = 59,
        k_rcin10                = 60,
        k_rcin11                = 61,
        k_rcin12                = 62,
        k_rcin13                = 63,
        k_rcin14                = 64,
        k_rcin15                = 65,
        k_rcin16                = 66,
        k_ignition              = 67,
        k_choke                 = 68,
        k_starter               = 69,
        k_throttle              = 70,
        k_tracker_yaw           = 71,            ///< antennatracker yaw
        k_tracker_pitch         = 72,            ///< antennatracker pitch
        k_throttleLeft          = 73,
        k_throttleRight         = 74,
        k_tiltMotorLeft         = 75,            ///< vectored thrust, left tilt
        k_tiltMotorRight        = 76,            ///< vectored thrust, right tilt
        k_nr_aux_servo_functions         ///< This must be the last enum value (only add new values _before_ this one)
    } Aux_servo_function_t;

    // used to get min/max/trim limit value based on reverse
    enum LimitValue {
        SRV_CHANNEL_LIMIT_TRIM,
        SRV_CHANNEL_LIMIT_MIN,
        SRV_CHANNEL_LIMIT_MAX,
        SRV_CHANNEL_LIMIT_ZERO_PWM
    };

    // set the output value as a pwm value
    void set_output_pwm(uint16_t pwm);

    // get the output value as a pwm value
    uint16_t get_output_pwm(void) const { return output_pwm; }

    // set angular range of scaled output
    void set_angle(int16_t angle);

    // set range of scaled output. Low is always zero
    void set_range(uint16_t high);

    // return true if the channel is reversed
    bool get_reversed(void) const {
        return reversed?true:false;
    }

    // set MIN/MAX parameters
    void set_output_min(uint16_t pwm) {
        servo_min.set(pwm);
    }
    void set_output_max(uint16_t pwm) {
        servo_max.set(pwm);
    }

    // get MIN/MAX/TRIM parameters
    uint16_t get_output_min(void) const {
        return servo_min;
    }
    uint16_t get_output_max(void) const {
        return servo_max;
    }
    uint16_t get_trim(void) const {
        return servo_trim;
    }

private:
    AP_Int16 servo_min;
    AP_Int16 servo_max;
    AP_Int16 servo_trim;
    // reversal, following convention that 1 means reversed, 0 means normal
    AP_Int8 reversed;
    AP_Int8 function;

    // a pending output value as PWM
    uint16_t output_pwm;

    // true for angle output type
    bool type_angle:1;

    // set_range() or set_angle() has been called
    bool type_setup:1;
    
    // the hal channel number
    uint8_t ch_num;

    // high point of angle or range output
    uint16_t high_out;

    // convert a 0..range_max to a pwm
    uint16_t pwm_from_range(int16_t scaled_value) const;

    // convert a -angle_max..angle_max to a pwm
    uint16_t pwm_from_angle(int16_t scaled_value) const;

    // convert a scaled output to a pwm value
    void calc_pwm(int16_t output_scaled);

    // output value based on function
    void output_ch(void);

    // setup output type and range based on function
    void aux_servo_function_setup(void);

    // return PWM for a given limit value
    uint16_t get_limit_pwm(LimitValue limit) const;

    // get normalised output from -1 to 1
    float get_output_norm(void);
    
    // a bitmask type wide enough for NUM_SERVO_CHANNELS
    typedef uint16_t servo_mask_t;

    // mask of channels where we have a output_pwm value. Cleared when a
    // scaled value is written. 
    static servo_mask_t have_pwm_mask;
};

/*
  class	SRV_Channels
*/
class SRV_Channels {
public:
    friend class SRV_Channel;
    
    // constructor
    SRV_Channels(void);

    static const struct AP_Param::GroupInfo var_info[];

    // set the default function for a channel
    static void set_default_function(uint8_t chan, SRV_Channel::Aux_servo_function_t function);

    // set output value for a function channel as a pwm value
    static void set_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t value);

    // set output value for a function channel as a pwm value on the first matching channel
    static void set_output_pwm_first(SRV_Channel::Aux_servo_function_t function, uint16_t value);

    // set output value for a function channel as a scaled value. This
    // calls calc_pwm() to also set the pwm value
    static void set_output_scaled(SRV_Channel::Aux_servo_function_t function, int16_t value);

    // get scaled output for the given function type.
    static int16_t get_output_scaled(SRV_Channel::Aux_servo_function_t function);

    // get pwm output for the first channel of the given function type.
    static bool get_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t &value);

    // get normalised output (-1 to 1 for angle, 0 to 1 for range). Value is taken from pwm value
    // return zero on error.
    static float get_output_norm(SRV_Channel::Aux_servo_function_t function);

    // limit slew rate to given limit in percent per second
    static void limit_slew_rate(SRV_Channel::Aux_servo_function_t function, float slew_rate, float dt);

    // call output_ch() on all channels
    static void output_ch_all(void);

    // setup output ESC scaling based on a channels MIN/MAX
    void set_esc_scaling_for(SRV_Channel::Aux_servo_function_t function);

    // return true when auto_trim enabled
    bool auto_trim_enabled(void) const { return auto_trim; }

    // adjust trim of a channel by a small increment
    void adjust_trim(SRV_Channel::Aux_servo_function_t function, float v);

    // save trims
    void save_trim(void);

    // setup for a reversible k_throttle (from -100 to 100)
    void set_reversible_throttle(void) {
        flags.k_throttle_reversible = true;
    }

    // set all outputs to the TRIM value
    static void output_trim_all(void);

    // setup IO failsafe for all channels to trim
    static void setup_failsafe_trim_all(void);

    // set output for all channels matching the given function type, allow radio_trim to center servo
    static void set_output_pwm_trimmed(SRV_Channel::Aux_servo_function_t function, int16_t value);

    // set and save the trim for a function channel to radio_in on matching input channel
    static void set_trim_to_radio_in_for(SRV_Channel::Aux_servo_function_t function);

    // set the trim for a function channel to min of the channel
    static void set_trim_to_min_for(SRV_Channel::Aux_servo_function_t function);

    // set the trim for a function channel to given pwm
    static void set_trim_to_pwm_for(SRV_Channel::Aux_servo_function_t function, int16_t pwm);

    // set output to min value
    static void set_output_to_min(SRV_Channel::Aux_servo_function_t function);

    // set output to max value
    static void set_output_to_max(SRV_Channel::Aux_servo_function_t function);

    // set output to trim value
    static void set_output_to_trim(SRV_Channel::Aux_servo_function_t function);

    // copy radio_in to radio_out
    static void copy_radio_in_out(SRV_Channel::Aux_servo_function_t function, bool do_input_output=false);

    // setup failsafe for an auxiliary channel function, by pwm
    static void set_failsafe_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t pwm);

    // setup failsafe for an auxiliary channel function
    static void set_failsafe_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::LimitValue limit);

    // setup safety for an auxiliary channel function (used when disarmed)
    static void set_safety_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::LimitValue limit);

    // set servo to a LimitValue
    static void set_output_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::LimitValue limit);

    // return true if a function is assigned to a channel
    static bool function_assigned(SRV_Channel::Aux_servo_function_t function);

    // set a servo_out value, and angle range, then calc_pwm
    static void move_servo(SRV_Channel::Aux_servo_function_t function,
                           int16_t value, int16_t angle_min, int16_t angle_max);

    // assign and enable auxiliary channels
    static void enable_aux_servos(void);

    // return the current function for a channel
    static SRV_Channel::Aux_servo_function_t channel_function(uint8_t channel);

    // refresh aux servo to function mapping
    static void update_aux_servo_function(void);

    // set default channel for an auxiliary function
    static bool set_aux_channel_default(SRV_Channel::Aux_servo_function_t function, uint8_t channel);

    // find first channel that a function is assigned to
    static bool find_channel(SRV_Channel::Aux_servo_function_t function, uint8_t &chan);

    // find first channel that a function is assigned to, returning SRV_Channel object
    static SRV_Channel *get_channel_for(SRV_Channel::Aux_servo_function_t function, int8_t default_chan=-1);

    // call set_angle() on matching channels
    static void set_angle(SRV_Channel::Aux_servo_function_t function, uint16_t angle);

    // call set_range() on matching channels
    static void set_range(SRV_Channel::Aux_servo_function_t function, uint16_t range);

    // set output refresh frequency on a servo function
    static void set_rc_frequency(SRV_Channel::Aux_servo_function_t function, uint16_t frequency);
    
    // control pass-thru of channels
    void disable_passthrough(bool disable) {
        disabled_passthrough = disable;
    }

    // constrain to output min/max for function
    static void constrain_pwm(SRV_Channel::Aux_servo_function_t function);
    
    // calculate PWM for all channels
    static void calc_pwm(void);

    static SRV_Channel *srv_channel(uint8_t i) {
        return i<NUM_SERVO_CHANNELS?&channels[i]:nullptr;
    }

    // upgrade RC* parameters into SERVO* parameters
    static bool upgrade_parameters(const uint8_t old_keys[14], uint16_t aux_channel_mask, RCMapper *rcmap);
    static void upgrade_motors_servo(uint8_t ap_motors_key, uint8_t ap_motors_idx, uint8_t new_channel);
    
    static uint32_t get_can_servo_bm(void) {
        if(p_can_servo_bm != nullptr)
            return *p_can_servo_bm;
        else
            return 0;
    }
    static uint32_t get_can_esc_bm(void) {
        if(p_can_esc_bm != nullptr)
            return *p_can_esc_bm;
        else
            return 0;
    }

private:
    struct {
        bool k_throttle_reversible:1;
    } flags;

    static bool disabled_passthrough;

    SRV_Channel::servo_mask_t trimmed_mask;

    static Bitmask function_mask;
    static bool initialised;
    
    // this static arrangement is to avoid having static objects in AP_Param tables
    static SRV_Channel *channels;
    SRV_Channel obj_channels[NUM_SERVO_CHANNELS];

    static struct srv_function {
        // mask of what channels this applies to
        SRV_Channel::servo_mask_t channel_mask;

        // scaled output for this function
        int16_t output_scaled;
    } functions[SRV_Channel::k_nr_aux_servo_functions];

    AP_Int8 auto_trim;

    // return true if passthrough is disabled
    static bool passthrough_disabled(void) {
        return disabled_passthrough;
    }

    AP_Int32 can_servo_bm;
    AP_Int32 can_esc_bm;
    static AP_Int32 *p_can_servo_bm;
    static AP_Int32 *p_can_esc_bm;
};
