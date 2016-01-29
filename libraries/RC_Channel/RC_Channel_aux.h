// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	RC_Channel_aux.h
/// @brief	RC_Channel manager for auxiliary channels (5..8), with EEPROM-backed storage of constants.
/// @author Amilcar Lucas

#ifndef __RC_CHANNEL_AUX_H__
#define __RC_CHANNEL_AUX_H__

#include <AP_HAL/AP_HAL.h>
#include "RC_Channel.h"

#define RC_AUX_MAX_CHANNELS 12

/// @class	RC_Channel_aux
/// @brief	Object managing one aux. RC channel (CH5-8), with information about its function
class RC_Channel_aux : public RC_Channel {
public:
    /// Constructor
    ///
    /// @param key      EEPROM storage key for the channel trim parameters.
    /// @param name     Optional name for the group.
    ///
    RC_Channel_aux(uint8_t ch_out) :
        RC_Channel(ch_out)
    {
        for (uint8_t i=0; i<RC_AUX_MAX_CHANNELS; i++) {
            if (_aux_channels[i] == NULL) {
                _aux_channels[i] = this;
                break;
            }
        }
		AP_Param::setup_object_defaults(this, var_info);
    }

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
        k_epm                   = 28,            ///< epm gripper
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
        k_nr_aux_servo_functions         ///< This must be the last enum value (only add new values _before_ this one)
    } Aux_servo_function_t;

    AP_Int8         function;           ///< see Aux_servo_function_t enum

    // output one auxillary channel
    void            output_ch(void);

    // output all auxillary channels
    static void     output_ch_all(void);

	// set radio_out for a function channel
	static void set_radio(Aux_servo_function_t function, int16_t value);

	// set radio_out for all channels matching the given function type, allow radio_trim to center servo
	static void set_radio_trimmed(Aux_servo_function_t function, int16_t value);

	// set and save the trim for a function channel to radio_in
	static void set_radio_trim(Aux_servo_function_t function);

	// set radio_out to radio_min
	static void set_radio_to_min(Aux_servo_function_t function);

	// set radio_out to radio_max
	static void set_radio_to_max(Aux_servo_function_t function);

	// set radio_out to radio_trim
	static void set_radio_to_trim(Aux_servo_function_t function);

	// copy radio_in to radio_out
	static void copy_radio_in_out(Aux_servo_function_t function, bool do_input_output=false);

	// set servo_out
	static void set_servo_out(Aux_servo_function_t function, int16_t value);

	// setup failsafe for an auxillary channel function
	static void set_servo_failsafe(Aux_servo_function_t function, RC_Channel::LimitValue limit);

	// set servo to a LimitValue
	static void set_servo_limit(Aux_servo_function_t function, RC_Channel::LimitValue limit);

	// return true if a function is assigned to a channel
	static bool function_assigned(Aux_servo_function_t function);

	// set a servo_out value, and angle range, then calc_pwm
	static void move_servo(Aux_servo_function_t function,
						   int16_t value, int16_t angle_min, int16_t angle_max);

    static const struct AP_Param::GroupInfo        var_info[];

    // assigned and enable auxillary channels
    static void enable_aux_servos(void);
    
    // prevent a channel from being used for auxillary functions
    static void disable_aux_channel(uint8_t channel);

    // return the current function for a channel
    static Aux_servo_function_t channel_function(uint8_t channel);

    // refresh aux servo to function mapping
    static void update_aux_servo_function(void);

    // set default channel for an auxillary function
    static bool set_aux_channel_default(Aux_servo_function_t function, uint8_t channel);

    // find first channel that a function is assigned to
    static bool find_channel(Aux_servo_function_t function, uint8_t &chan);
    
private:
    static uint64_t _function_mask;
    static bool _initialised;
    static RC_Channel_aux *_aux_channels[RC_AUX_MAX_CHANNELS];

    void aux_servo_function_setup(void);
};

#endif /* RC_CHANNEL_AUX_H_ */
