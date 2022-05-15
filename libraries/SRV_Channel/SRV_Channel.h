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

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Bitmask.h>
#include <AP_Volz_Protocol/AP_Volz_Protocol.h>
#include <AP_RobotisServo/AP_RobotisServo.h>
#include <AP_SBusOut/AP_SBusOut.h>
#include <AP_BLHeli/AP_BLHeli.h>
#include <AP_FETtecOneWire/AP_FETtecOneWire.h>

#ifndef NUM_SERVO_CHANNELS
#if defined(HAL_BUILD_AP_PERIPH) && defined(HAL_PWM_COUNT)
    #define NUM_SERVO_CHANNELS HAL_PWM_COUNT
#elif defined(HAL_BUILD_AP_PERIPH)
    #define NUM_SERVO_CHANNELS 0
#else
    #if !HAL_MINIMIZE_FEATURES
        #define NUM_SERVO_CHANNELS 32
    #else
        #define NUM_SERVO_CHANNELS 16
    #endif
#endif
#endif
static_assert(NUM_SERVO_CHANNELS <= 32, "More than 32 servos not supported");

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
        k_GPIO                  = -1,           ///< used as GPIO pin (input or output)
        k_none                  = 0,            ///< general use PWM output used by do-set-servo commands and lua scripts
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
        k_egg_drop              = 11,           ///< egg drop, deprecated
        k_mount2_pan            = 12,           ///< mount2 yaw (pan)
        k_mount2_tilt           = 13,           ///< mount2 pitch (tilt)
        k_mount2_roll           = 14,           ///< mount2 roll
        k_mount2_open           = 15,           ///< mount2 open (deploy) / close (retract)
        k_dspoilerLeft1         = 16,           ///< differential spoiler 1 (left wing)
        k_dspoilerRight1        = 17,           ///< differential spoiler 1 (right wing)
        k_aileron_with_input    = 18,            ///< aileron, with rc input, deprecated
        k_elevator              = 19,            ///< elevator
        k_elevator_with_input   = 20,            ///< elevator, with rc input, deprecated
        k_rudder                = 21,            ///< secondary rudder channel
        k_sprayer_pump          = 22,            ///< crop sprayer pump channel
        k_sprayer_spinner       = 23,            ///< crop sprayer spinner channel
        k_flaperon_left         = 24,            ///< flaperon, left wing
        k_flaperon_right        = 25,            ///< flaperon, right wing
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
        k_generator_control     = 42,            ///< state control for generator
        k_tiltMotorRear         = 45,            ///<vectored thrust, rear tilt
        k_tiltMotorRearLeft     = 46,            ///<vectored thrust, rear left tilt
        k_tiltMotorRearRight    = 47,            ///<vectored thrust, rear right tilt
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
        k_choke                 = 68,           /// not used
        k_starter               = 69,
        k_throttle              = 70,
        k_tracker_yaw           = 71,            ///< antennatracker yaw
        k_tracker_pitch         = 72,            ///< antennatracker pitch
        k_throttleLeft          = 73,
        k_throttleRight         = 74,
        k_tiltMotorLeft         = 75,            ///< vectored thrust, left tilt
        k_tiltMotorRight        = 76,            ///< vectored thrust, right tilt
        k_elevon_left           = 77,
        k_elevon_right          = 78,
        k_vtail_left            = 79,
        k_vtail_right           = 80,
        k_boost_throttle        = 81,            ///< vertical booster throttle
        k_motor9                = 82,
        k_motor10               = 83,
        k_motor11               = 84,
        k_motor12               = 85,
        k_dspoilerLeft2         = 86,           ///< differential spoiler 2 (left wing)
        k_dspoilerRight2        = 87,           ///< differential spoiler 2 (right wing)
        k_winch                 = 88,
        k_mainsail_sheet        = 89,           ///< Main Sail control via sheet
        k_cam_iso               = 90,
        k_cam_aperture          = 91,
        k_cam_focus             = 92,
        k_cam_shutter_speed     = 93,
        k_scripting1            = 94,           ///< Scripting related outputs
        k_scripting2            = 95,
        k_scripting3            = 96,
        k_scripting4            = 97,
        k_scripting5            = 98,
        k_scripting6            = 99,
        k_scripting7            = 100,
        k_scripting8            = 101,
        k_scripting9            = 102,
        k_scripting10           = 103,
        k_scripting11           = 104,
        k_scripting12           = 105,
        k_scripting13           = 106,
        k_scripting14           = 107,
        k_scripting15           = 108,
        k_scripting16           = 109,
        k_airbrake              = 110,
        k_LED_neopixel1         = 120,
        k_LED_neopixel2         = 121,
        k_LED_neopixel3         = 122,
        k_LED_neopixel4         = 123,
        k_roll_out              = 124,
        k_pitch_out             = 125,
        k_thrust_out            = 126,
        k_yaw_out               = 127,
        k_wingsail_elevator     = 128,
        k_ProfiLED_1            = 129,
        k_ProfiLED_2            = 130,
        k_ProfiLED_3            = 131,
        k_ProfiLED_Clock        = 132,
        k_winch_clutch          = 133,
        k_min                   = 134,  // always outputs SERVOn_MIN
        k_trim                  = 135,  // always outputs SERVOn_TRIM
        k_max                   = 136,  // always outputs SERVOn_MAX
        k_mast_rotation         = 137,
        k_alarm                 = 138,
        k_alarm_inverted        = 139,
        k_nr_aux_servo_functions         ///< This must be the last enum value (only add new values _before_ this one)
    } Aux_servo_function_t;

    // check if a function is valid for indexing into functions
    static bool valid_function(Aux_servo_function_t fn) {
        return fn >= k_none && fn < k_nr_aux_servo_functions;
    }
    bool valid_function(void) const {
        return valid_function(function);
    }
    
    // used to get min/max/trim limit value based on reverse
    enum class Limit {
        TRIM,
        MIN,
        MAX,
        ZERO_PWM
    };

    // set the output value as a pwm value
    void set_output_pwm(uint16_t pwm, bool force = false);

    // get the output value as a pwm value
    uint16_t get_output_pwm(void) const { return output_pwm; }

    // set normalised output from -1 to 1, assuming 0 at mid point of servo_min/servo_max
    void set_output_norm(float value);

    // set angular range of scaled output
    void set_angle(int16_t angle);

    // set range of scaled output. Low is always zero
    void set_range(uint16_t high);

    // return true if the channel is reversed
    bool get_reversed(void) const {
        return reversed != 0;
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

    // return true if function is for a multicopter motor
    static bool is_motor(SRV_Channel::Aux_servo_function_t function);

    // return true if function is for anything that should be stopped in a e-stop situation, ie is dangerous
    static bool should_e_stop(SRV_Channel::Aux_servo_function_t function);

    // return true if function is for a control surface
    static bool is_control_surface(SRV_Channel::Aux_servo_function_t function);

    // return the function of a channel
    SRV_Channel::Aux_servo_function_t get_function(void) const {
        return (SRV_Channel::Aux_servo_function_t)function.get();
    }

    // return the motor number of a channel, or -1 if not a motor
    int8_t get_motor_num(void) const;

    // set and save function for channel. Used in upgrade of parameters in plane
    void function_set_and_save(SRV_Channel::Aux_servo_function_t f) {
        function.set_and_save(int8_t(f));
    }

    // set and save function for reversed. Used in upgrade of parameters in plane
    void reversed_set_and_save_ifchanged(bool r) {
        reversed.set_and_save_ifchanged(r?1:0);
    }
    
    // return true if the SERVOn_FUNCTION has been configured in
    // either storage or a defaults file. This is used for upgrade of
    // parameters in plane
    bool function_configured(void) const {
        return function.configured();
    }

    // specify that small rc input changes should be ignored during passthrough
    // used by DO_SET_SERVO commands
    void ignore_small_rcin_changes() { ign_small_rcin_changes = true; }

private:
    AP_Int16 servo_min;
    AP_Int16 servo_max;
    AP_Int16 servo_trim;
    // reversal, following convention that 1 means reversed, 0 means normal
    AP_Int8 reversed;
    AP_Enum16<Aux_servo_function_t> function;

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
    uint16_t pwm_from_range(float scaled_value) const;

    // convert a -angle_max..angle_max to a pwm
    uint16_t pwm_from_angle(float scaled_value) const;

    // convert a scaled output to a pwm value
    void calc_pwm(float output_scaled);

    // output value based on function
    void output_ch(void);

    // setup output type and range based on function
    void aux_servo_function_setup(void);

    // return PWM for a given limit value
    uint16_t get_limit_pwm(Limit limit) const;

    // get normalised output from -1 to 1, assuming 0 at mid point of servo_min/servo_max
    float get_output_norm(void);

    // a bitmask type wide enough for NUM_SERVO_CHANNELS
    typedef uint32_t servo_mask_t;

    // mask of channels where we have a output_pwm value. Cleared when a
    // scaled value is written. 
    static servo_mask_t have_pwm_mask;

    // previous radio_in during pass-thru
    int16_t previous_radio_in;

    // specify that small rcinput changes should be ignored during passthrough
    // used by DO_SET_SERVO commands
    bool ign_small_rcin_changes;

    // if true we should ignore all imputs on this channel
    bool override_active;

    void set_override(bool b) {override_active = b;};
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

    // set output value for a specific function channel as a pwm value
    static void set_output_pwm_chan(uint8_t chan, uint16_t value);

    // set output value for a specific function channel as a pwm value for specified override time in ms
    static void set_output_pwm_chan_timeout(uint8_t chan, uint16_t value, uint16_t timeout_ms);

    // set output value for a function channel as a scaled value. This
    // calls calc_pwm() to also set the pwm value
    static void set_output_scaled(SRV_Channel::Aux_servo_function_t function, float value);

    // get scaled output for the given function type.
    static float get_output_scaled(SRV_Channel::Aux_servo_function_t function);

    // get slew limited scaled output for the given function type
    static float get_slew_limited_output_scaled(SRV_Channel::Aux_servo_function_t function);

    // get pwm output for the first channel of the given function type.
    static bool get_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t &value);

    // get normalised output (-1 to 1 with 0 at mid point of servo_min/servo_max)
    // Value is taken from pwm value.  Returns zero on error.
    static float get_output_norm(SRV_Channel::Aux_servo_function_t function);

    // set normalised output (-1 to 1 with 0 at mid point of servo_min/servo_max) for the given function
    static void set_output_norm(SRV_Channel::Aux_servo_function_t function, float value);

    // get output channel mask for a function
    static uint32_t get_output_channel_mask(SRV_Channel::Aux_servo_function_t function);

    // limit slew rate to given limit in percent per second
    static void set_slew_rate(SRV_Channel::Aux_servo_function_t function, float slew_rate, uint16_t range, float dt);

    // call output_ch() on all channels
    static void output_ch_all(void);

    // setup output ESC scaling based on a channels MIN/MAX
    void set_esc_scaling_for(SRV_Channel::Aux_servo_function_t function);

    // return true when auto_trim enabled
    bool auto_trim_enabled(void) const { return auto_trim; }

    // adjust trim of a channel by a small increment
    void adjust_trim(SRV_Channel::Aux_servo_function_t function, float v);

    // set MIN/MAX parameters for a function
    static void set_output_min_max(SRV_Channel::Aux_servo_function_t function, uint16_t min_pwm, uint16_t max_pwm);
    
    // save trims
    void save_trim(void);

    // setup IO failsafe for all channels to trim
    static void setup_failsafe_trim_all_non_motors(void);

    // set output for all channels matching the given function type, allow radio_trim to center servo
    static void set_output_pwm_trimmed(SRV_Channel::Aux_servo_function_t function, int16_t value);

    // set and save the trim for a function channel to the output value
    static void set_trim_to_servo_out_for(SRV_Channel::Aux_servo_function_t function);

    // set the trim for a function channel to min of the channel honnoring reverse unless ignore_reversed is true
    static void set_trim_to_min_for(SRV_Channel::Aux_servo_function_t function, bool ignore_reversed = false);

    // set the trim for a function channel to given pwm
    static void set_trim_to_pwm_for(SRV_Channel::Aux_servo_function_t function, int16_t pwm);

    // set output to min value
    static void set_output_to_min(SRV_Channel::Aux_servo_function_t function);

    // set output to max value
    static void set_output_to_max(SRV_Channel::Aux_servo_function_t function);

    // set output to trim value
    static void set_output_to_trim(SRV_Channel::Aux_servo_function_t function);

    // copy radio_in to servo out
    static void copy_radio_in_out(SRV_Channel::Aux_servo_function_t function, bool do_input_output=false);

    // copy radio_in to servo_out by channel mask
    static void copy_radio_in_out_mask(uint32_t mask);

    // setup failsafe for an auxiliary channel function, by pwm
    static void set_failsafe_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t pwm);

    // setup failsafe for an auxiliary channel function
    static void set_failsafe_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::Limit limit);

    // set servo to a Limit
    static void set_output_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::Limit limit);

    // return true if a function is assigned to a channel
    static bool function_assigned(SRV_Channel::Aux_servo_function_t function);

    // set a servo_out value, and angle range, then calc_pwm
    static void move_servo(SRV_Channel::Aux_servo_function_t function,
                           int16_t value, int16_t angle_min, int16_t angle_max);

    // assign and enable auxiliary channels
    static void enable_aux_servos(void);

    // enable channels by mask
    static void enable_by_mask(uint32_t mask);

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

    // return the ESC type for dshot commands
    static AP_HAL::RCOutput::DshotEscType get_dshot_esc_type() { return AP_HAL::RCOutput::DshotEscType(_singleton->dshot_esc_type.get()); }

    static SRV_Channel *srv_channel(uint8_t i) {
#if NUM_SERVO_CHANNELS > 0
        return i<NUM_SERVO_CHANNELS?&channels[i]:nullptr;
#else
        return nullptr;
#endif
    }

    // SERVO* parameters
    static void upgrade_parameters(void);

    // given a zero-based motor channel, return the k_motor function for that channel
    static SRV_Channel::Aux_servo_function_t get_motor_function(uint8_t channel) {
        if (channel < 8) {
            return SRV_Channel::Aux_servo_function_t(SRV_Channel::k_motor1+channel);
        }
        return SRV_Channel::Aux_servo_function_t((SRV_Channel::k_motor9+(channel-8)));
    }
    
    static void cork();

    static void push();

    // disable PWM output to a set of channels given by a mask. This is used by the AP_BLHeli code
    static void set_disabled_channel_mask(uint32_t mask) { disabled_mask = mask; }
    static uint32_t get_disabled_channel_mask() { return disabled_mask; }

    // add to mask of outputs which use digital (non-PWM) output and optionally can reverse thrust, such as DShot
    static void set_digital_outputs(uint32_t dig_mask, uint32_t rev_mask);

    // return true if all of the outputs in mask are digital
    static bool have_digital_outputs(uint32_t mask) { return mask != 0 && (mask & digital_mask) == mask; }

    // return true if any of the outputs are digital
    static bool have_digital_outputs() { return digital_mask != 0; }

    // Set E - stop
    static void set_emergency_stop(bool state) {
        emergency_stop = state;
    }

    // get E - stop
    static bool get_emergency_stop() { return emergency_stop;}

    // singleton for Lua
    static SRV_Channels *get_singleton(void) {
        return _singleton;
    }

    static void zero_rc_outputs();

    // initialize before any call to push
    static void init();

    // return true if a channel is set to type GPIO
    static bool is_GPIO(uint8_t channel);

    // return true if a channel is set to type alarm
    static bool is_alarm(uint8_t channel) {
        return channel_function(channel) == SRV_Channel::k_alarm;
    }

    // return true if a channel is set to type alarm inverted
    static bool is_alarm_inverted(uint8_t channel) {
        return channel_function(channel) == SRV_Channel::k_alarm_inverted;
    }

private:

    static bool disabled_passthrough;

    SRV_Channel::servo_mask_t trimmed_mask;

    static Bitmask<SRV_Channel::k_nr_aux_servo_functions> function_mask;
    static bool initialised;

    // this static arrangement is to avoid having static objects in AP_Param tables
    static SRV_Channel *channels;
    static SRV_Channels *_singleton;

#ifndef HAL_BUILD_AP_PERIPH
    // support for Volz protocol
    AP_Volz_Protocol volz;
    static AP_Volz_Protocol *volz_ptr;

    // support for SBUS protocol
    AP_SBusOut sbus;
    static AP_SBusOut *sbus_ptr;

    // support for Robotis servo protocol
    AP_RobotisServo robotis;
    static AP_RobotisServo *robotis_ptr;
#endif // HAL_BUILD_AP_PERIPH

#if HAL_SUPPORT_RCOUT_SERIAL
    // support for BLHeli protocol
    AP_BLHeli blheli;
    static AP_BLHeli *blheli_ptr;
#endif

#if AP_FETTEC_ONEWIRE_ENABLED
    AP_FETtecOneWire fetteconwire;
    static AP_FETtecOneWire *fetteconwire_ptr;
#endif  // AP_FETTEC_ONEWIRE_ENABLED

    // mask of disabled channels
    static uint32_t disabled_mask;

    // mask of outputs which use a digital output protocol, not
    // PWM (eg. DShot)
    static uint32_t digital_mask;
    
    // mask of outputs which are digitally reversible (eg. DShot-3D)
    static uint32_t reversible_mask;

    // mask of channels with invalid funtions, eg GPIO
    static uint32_t invalid_mask;

    SRV_Channel obj_channels[NUM_SERVO_CHANNELS];

    // override loop counter
    static uint16_t override_counter[NUM_SERVO_CHANNELS];

    static struct srv_function {
        // mask of what channels this applies to
        SRV_Channel::servo_mask_t channel_mask;

        // scaled output for this function
        float output_scaled;
    } functions[SRV_Channel::k_nr_aux_servo_functions];

    AP_Int8 auto_trim;
    AP_Int16 default_rate;
    AP_Int8 dshot_rate;
    AP_Int8 dshot_esc_type;
    AP_Int32 gpio_mask;
#if NUM_SERVO_CHANNELS >= 17
    AP_Int8 enable_32_channels;
#endif

    // return true if passthrough is disabled
    static bool passthrough_disabled(void) {
        return disabled_passthrough;
    }

    static bool emergency_stop;

    // linked list for slew rate handling
    struct slew_list {
        slew_list(SRV_Channel::Aux_servo_function_t _func) : func(_func) {};
        const SRV_Channel::Aux_servo_function_t func;
        float last_scaled_output;
        float max_change;
        slew_list * next;
    };
    static slew_list *_slew;

    // semaphore for multi-thread use of override_counter array
    HAL_Semaphore override_counter_sem;
};
