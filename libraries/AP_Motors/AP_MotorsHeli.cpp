/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsHeli.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli::var_info[] = {

    // 1 was ROL_MAX which has been replaced by CYC_MAX

    // 2 was PIT_MAX which has been replaced by CYC_MAX

    // @Param: COL_MIN
    // @DisplayName: Minimum Collective Pitch
    // @Description: Lowest possible servo position in PWM microseconds for the swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MIN", 3, AP_MotorsHeli, _collective_min, AP_MOTORS_HELI_COLLECTIVE_MIN),

    // @Param: COL_MAX
    // @DisplayName: Maximum Collective Pitch
    // @Description: Highest possible servo position in PWM microseconds for the swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MAX", 4, AP_MotorsHeli, _collective_max, AP_MOTORS_HELI_COLLECTIVE_MAX),

    // index 5 was COL_MID. Do not use this index in the future.

    // @Param: SV_MAN
    // @DisplayName: Manual Servo Mode
    // @Description: Manual servo override for swash set-up. Must be 0 (Disabled) for flight!
    // @Values: 0:Disabled,1:Passthrough,2:Max collective,3:Zero thrust collective,4:Min collective
    // @User: Standard
    AP_GROUPINFO("SV_MAN",  6, AP_MotorsHeli, _servo_mode, SERVO_CONTROL_MODE_AUTOMATED),

    // indices 7 and 8 were RSC parameters which were moved to RSC library. Do not use these indices in the future.

    // index 9 was LAND_COL_MIN. Do not use this index in the future.

    // indices 10-13 were RSC parameters which were moved to RSC library. Do not use these indices in the future.

    // index 14 was RSC_POWER_LOW. Do not use this index in the future.

    // index 15 was RSC_POWER_HIGH. Do not use this index in the future.

    // @Param: CYC_MAX
    // @DisplayName: Maximum Cyclic Pitch Angle
    // @Description: Maximum cyclic pitch angle of the swash plate.  There are no units to this parameter.  This should be adjusted to get the desired cyclic blade pitch for the pitch and roll axes.  Typically this should be 6-7 deg (measured blade pitch angle difference between stick centered and stick max deflection.
    // @Range: 0 4500
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("CYC_MAX", 16, AP_MotorsHeli, _cyclic_max, AP_MOTORS_HELI_SWASH_CYCLIC_MAX),

    // @Param: SV_TEST
    // @DisplayName: Boot-up Servo Test Cycles
    // @Description: Number of cycles to run servo test on boot-up
    // @Range: 0 10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SV_TEST",  17, AP_MotorsHeli, _servo_test, 0),

    // index 18 was RSC_POWER_NEGC. Do not use this index in the future.

    // index 19 was RSC_SLEWRATE and was moved to RSC library. Do not use this index in the future.

    // indices 20 to 24 was throttle curve. Do not use this index in the future.

    // @Group: RSC_
    // @Path: AP_MotorsHeli_RSC.cpp
    AP_SUBGROUPINFO(_main_rotor, "RSC_", 25, AP_MotorsHeli, AP_MotorsHeli_RSC),

    // @Param: COL_HOVER
    // @DisplayName: Collective Hover Value
    // @Description: Collective needed to hover expressed as a number from 0 to 1 where 0 is H_COL_MIN and 1 is H_COL_MAX
    // @Range: 0.3 0.8
    // @User: Advanced
    AP_GROUPINFO("COL_HOVER", 26, AP_MotorsHeli, _collective_hover, AP_MOTORS_HELI_COLLECTIVE_HOVER_DEFAULT),

    // @Param: HOVER_LEARN
    // @DisplayName: Hover Value Learning
    // @Description: Enable/Disable automatic learning of hover collective
    // @Values: 0:Disabled, 1:Learn, 2:Learn and Save
    // @User: Advanced
    AP_GROUPINFO("HOVER_LEARN", 27, AP_MotorsHeli, _collective_hover_learn, HOVER_LEARN_AND_SAVE),

    // @Param: OPTIONS
    // @DisplayName: Heli_Options
    // @Description: Bitmask of heli options.  Bit 0 changes how the pitch, roll, and yaw axis integrator term is managed for low speed and takeoff/landing. In AC 4.0 and earlier, scheme uses a leaky integrator for ground speeds less than 5 m/s and won't let the steady state integrator build above ILMI. The integrator is allowed to build to the ILMI value when it is landed.  The other integrator management scheme bases integrator limiting on takeoff and landing.  Whenever the aircraft is landed the integrator is set to zero.  When the aicraft is airborne, the integrator is only limited by IMAX. 
    // @Bitmask: 0:Use Leaky I
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 28, AP_MotorsHeli, _heli_options, (uint8_t)HeliOption::USE_LEAKY_I),

    // @Param: COL_ANG_MIN
    // @DisplayName: Collective Blade Pitch Angle Minimum
    // @Description: Minimum collective blade pitch angle in deg that corresponds to the PWM set for minimum collective pitch (H_COL_MIN).
    // @Range: -20 0
    // @Units: deg
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("COL_ANG_MIN", 29, AP_MotorsHeli, _collective_min_deg, AP_MOTORS_HELI_COLLECTIVE_MIN_DEG),

    // @Param: COL_ANG_MAX
    // @DisplayName: Collective Blade Pitch Angle Maximum
    // @Description: Maximum collective blade pitch angle in deg that corresponds to the PWM set for maximum collective pitch (H_COL_MAX).
    // @Range: 5 20
    // @Units: deg
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("COL_ANG_MAX", 30, AP_MotorsHeli, _collective_max_deg, AP_MOTORS_HELI_COLLECTIVE_MAX_DEG),

    // @Param: COL_ZERO_THRST
    // @DisplayName: Collective Blade Pitch at Zero Thrust
    // @Description: Collective blade pitch angle at zero thrust in degrees. For symetric airfoil blades this value is zero deg. For chambered airfoil blades this value is typically negative.
    // @Range: -5 0
    // @Units: deg
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("COL_ZERO_THRST", 31, AP_MotorsHeli, _collective_zero_thrust_deg, 0.0f),

    // @Param: COL_LAND_MIN
    // @DisplayName: Collective Blade Pitch Minimum when Landed
    // @Description: Minimum collective blade pitch angle when landed in degrees for non-manual collective modes (i.e. modes that use altitude hold).
    // @Range: -5 0
    // @Units: deg
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("COL_LAND_MIN", 32, AP_MotorsHeli, _collective_land_min_deg, AP_MOTORS_HELI_COLLECTIVE_LAND_MIN),

    AP_GROUPEND
};

//
// public methods
//

// init
void AP_MotorsHeli::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remember frame class and type
    _frame_type = frame_type;
    _frame_class = frame_class;

    // set update rate
    set_update_rate(_speed_hz);

    // load boot-up servo test cycles into counter to be consumed
    _servo_test_cycle_counter = _servo_test;

    // ensure inputs are not passed through to servos on start-up
    _servo_mode.set(SERVO_CONTROL_MODE_AUTOMATED);

    // initialise radio passthrough for collective to middle
    _throttle_radio_passthrough = 0.5f;

    // initialise Servo/PWM ranges and endpoints
    init_outputs();

    // calculate all scalars
    calculate_scalars();

    // set flag to true so targets are initialized once aircraft is armed for first time
    _heliflags.init_targets_on_arming = true;

    _mav_type = MAV_TYPE_HELICOPTER;
}

// output_min - sets servos to neutral point with motors stopped
void AP_MotorsHeli::output_min()
{
    // move swash to mid
    move_actuators(0.0f,0.0f,0.5f,0.0f);

    update_motor_control(AP_MotorsHeli_RSC::RotorControlState::STOP);

    // override limits flags
    set_limit_flag_pitch_roll_yaw(true);
    limit.throttle_lower = true;
    limit.throttle_upper = false;
}

// output - sends commands to the servos
void AP_MotorsHeli::output()
{
    // update throttle filter
    update_throttle_filter();

    // run spool logic
    output_logic();

    if (armed()) {
        // block servo_test from happening at disarm
        _servo_test_cycle_counter = 0;
        calculate_armed_scalars();
        output_armed_stabilizing();
    } else {
        output_disarmed();
    }

    update_turbine_start();

    output_to_motors();

};

// sends commands to the motors
void AP_MotorsHeli::output_armed_stabilizing()
{
    // if manual override active after arming, deactivate it and reinitialize servos
    if (_servo_mode != SERVO_CONTROL_MODE_AUTOMATED) {
        reset_flight_controls();
    }

    move_actuators(_roll_in, _pitch_in, get_throttle(), _yaw_in);
}

// output_disarmed - sends commands to the motors
void AP_MotorsHeli::output_disarmed()
{
    if (_servo_test_cycle_counter > 0){
        // set servo_test_flag
        _heliflags.servo_test_running = true;
        // perform boot-up servo test cycle if enabled
        servo_test();
    } else {
        // set servo_test flag
        _heliflags.servo_test_running = false;
        // manual override (i.e. when setting up swash)
        switch (_servo_mode) {
            case SERVO_CONTROL_MODE_MANUAL_PASSTHROUGH:
                // pass pilot commands straight through to swash
                _roll_in = _roll_radio_passthrough;
                _pitch_in = _pitch_radio_passthrough;
                _throttle_filter.reset(_throttle_radio_passthrough);
                _yaw_in = _yaw_radio_passthrough;
                break;
            case SERVO_CONTROL_MODE_MANUAL_CENTER:
                // fixate mid collective
                _roll_in = 0.0f;
                _pitch_in = 0.0f;
                _throttle_filter.reset(_collective_zero_thrust_pct);
                _yaw_in = 0.0f;
                break;
            case SERVO_CONTROL_MODE_MANUAL_MAX:
                // fixate max collective
                _roll_in = 0.0f;
                _pitch_in = 0.0f;
                _throttle_filter.reset(1.0f);
                if (_frame_class == MOTOR_FRAME_HELI_DUAL ||
                    _frame_class == MOTOR_FRAME_HELI_QUAD) {
                    _yaw_in = 0;
                } else {
                    _yaw_in = 1;
                }
                break;
            case SERVO_CONTROL_MODE_MANUAL_MIN:
                // fixate min collective
                _roll_in = 0.0f;
                _pitch_in = 0.0f;
                _throttle_filter.reset(0.0f);
                if (_frame_class == MOTOR_FRAME_HELI_DUAL ||
                    _frame_class == MOTOR_FRAME_HELI_QUAD) {
                    _yaw_in = 0;
                } else {
                    _yaw_in = -1;
                }
                break;
            case SERVO_CONTROL_MODE_MANUAL_OSCILLATE:
                // use servo_test function from child classes
                servo_test();
                break;
            default:
                // no manual override
                break;
        }
    }

    // ensure swash servo endpoints haven't been moved
    init_outputs();

    // continuously recalculate scalars to allow setup
    calculate_scalars();

    // helicopters always run stabilizing flight controls
    move_actuators(_roll_in, _pitch_in, get_throttle(), _yaw_in);
}

// run spool logic
void AP_MotorsHeli::output_logic()
{
    // force desired and current spool mode if disarmed and armed with interlock enabled
    if (armed()) {
        if (!get_interlock()) {
            _spool_desired = DesiredSpoolState::GROUND_IDLE;
        } else {
            _heliflags.init_targets_on_arming = false;
        }
    } else {
        _heliflags.init_targets_on_arming = true;
        _spool_desired = DesiredSpoolState::SHUT_DOWN;
        _spool_state = SpoolState::SHUT_DOWN;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // Motors should be stationary.
            // Servos set to their trim values or in a test condition.

            // set limits flags
            if (!using_leaky_integrator()) {
                set_limit_flag_pitch_roll_yaw(true);
            } else {
                set_limit_flag_pitch_roll_yaw(false);
            }

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DesiredSpoolState::SHUT_DOWN) {
                _spool_state = SpoolState::GROUND_IDLE;
                break;
            }

            break;

        case SpoolState::GROUND_IDLE: {
            // Motors should be stationary or at ground idle.
            // set limits flags
            if (_heliflags.land_complete && !using_leaky_integrator()) {
                set_limit_flag_pitch_roll_yaw(true);
            } else {
                set_limit_flag_pitch_roll_yaw(false);
            }

            // Servos should be moving to correct the current attitude.
            if (_spool_desired == DesiredSpoolState::SHUT_DOWN){
                _spool_state = SpoolState::SHUT_DOWN;
            } else if(_spool_desired == DesiredSpoolState::THROTTLE_UNLIMITED) {
                _spool_state = SpoolState::SPOOLING_UP;
            } else {    // _spool_desired == GROUND_IDLE

            }

            break;
        }
        case SpoolState::SPOOLING_UP:
            // Maximum throttle should move from minimum to maximum.
            // Servos should exhibit normal flight behavior.

            // set limits flags
            if (_heliflags.land_complete && !using_leaky_integrator()) {
                set_limit_flag_pitch_roll_yaw(true);
            } else {
                set_limit_flag_pitch_roll_yaw(false);
            }

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DesiredSpoolState::THROTTLE_UNLIMITED ){
                _spool_state = SpoolState::SPOOLING_DOWN;
                break;
            }

            if (_heliflags.rotor_runup_complete){
                _spool_state = SpoolState::THROTTLE_UNLIMITED;
            }
            break;

        case SpoolState::THROTTLE_UNLIMITED:
            // Throttle should exhibit normal flight behavior.
            // Servos should exhibit normal flight behavior.

            // set limits flags
            if (_heliflags.land_complete && !using_leaky_integrator()) {
                set_limit_flag_pitch_roll_yaw(true);
            } else {
                set_limit_flag_pitch_roll_yaw(false);
            }

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DesiredSpoolState::THROTTLE_UNLIMITED) {
                _spool_state = SpoolState::SPOOLING_DOWN;
                break;
            }


            break;

        case SpoolState::SPOOLING_DOWN:
            // Maximum throttle should move from maximum to minimum.
            // Servos should exhibit normal flight behavior.

            // set limits flags
            if (_heliflags.land_complete && !using_leaky_integrator()) {
                set_limit_flag_pitch_roll_yaw(true);
            } else {
                set_limit_flag_pitch_roll_yaw(false);
            }

            // make sure the motors are spooling in the correct direction
            if (_spool_desired == DesiredSpoolState::THROTTLE_UNLIMITED) {
                _spool_state = SpoolState::SPOOLING_UP;
                break;
            }
            if (_heliflags.rotor_spooldown_complete){
                _spool_state = SpoolState::GROUND_IDLE;
            }
            break;
    }
}

// update the throttle input filter
void AP_MotorsHeli::update_throttle_filter()
{
    _throttle_filter.apply(_throttle_in,  _dt);

    // constrain filtered throttle
    if (_throttle_filter.get() < 0.0f) {
        _throttle_filter.reset(0.0f);
    }
    if (_throttle_filter.get() > 1.0f) {
        _throttle_filter.reset(1.0f);
    }
}

// reset_flight_controls - resets all controls and scalars to flight status
void AP_MotorsHeli::reset_flight_controls()
{
    _servo_mode.set(SERVO_CONTROL_MODE_AUTOMATED);
    init_outputs();
    calculate_scalars();
}

// update the collective input filter.  should be called at 100hz
void AP_MotorsHeli::update_throttle_hover(float dt)
{
    if (_collective_hover_learn != HOVER_LEARN_DISABLED) {

        // Don't let _collective_hover go below H_COLL_ZERO_THRST
        float curr_collective = get_throttle();
        if (curr_collective < _collective_zero_thrust_pct) {
            curr_collective = _collective_zero_thrust_pct;
        }

        // we have chosen to constrain the hover collective to be within the range reachable by the third order expo polynomial.
        _collective_hover.set(constrain_float(_collective_hover + (dt / (dt + AP_MOTORS_HELI_COLLECTIVE_HOVER_TC)) * (curr_collective - _collective_hover), AP_MOTORS_HELI_COLLECTIVE_HOVER_MIN, AP_MOTORS_HELI_COLLECTIVE_HOVER_MAX));
    }
}

// save parameters as part of disarming
void AP_MotorsHeli::save_params_on_disarm()
{
    // save hover throttle
    if (_collective_hover_learn == HOVER_LEARN_AND_SAVE) {
        _collective_hover.save();
    }
}

// updates the takeoff collective flag
void AP_MotorsHeli::update_takeoff_collective_flag(float coll_out)
{
    if (coll_out > _collective_zero_thrust_pct + 0.5f * (_collective_hover - _collective_zero_thrust_pct)) {
        _heliflags.takeoff_collective = true;
    } else {
        _heliflags.takeoff_collective = false;
    }
}

// Determines if _heli_options bit is set
bool AP_MotorsHeli::heli_option(HeliOption opt) const
{
    return (_heli_options & (uint8_t)opt);
}

// updates the turbine start flag
void AP_MotorsHeli::update_turbine_start()
{
    if (_heliflags.start_engine) {
        _main_rotor.set_turbine_start(true);
    } else {
        _main_rotor.set_turbine_start(false);
    }
}

// Run arming checks
bool AP_MotorsHeli::arming_checks(size_t buflen, char *buffer) const
{
    // run base class checks
    if (!AP_Motors::arming_checks(buflen, buffer)) {
        return false;
    }

    if (_heliflags.servo_test_running) {
        hal.util->snprintf(buffer, buflen, "Servo Test is still running");
        return false;
    }

    // returns false if RSC Mode is not set to a valid control mode
    if (_main_rotor._rsc_mode.get() <= (int8_t)ROTOR_CONTROL_MODE_DISABLED || _main_rotor._rsc_mode.get() > (int8_t)ROTOR_CONTROL_MODE_AUTOTHROTTLE) {
        hal.util->snprintf(buffer, buflen, "H_RSC_MODE invalid");
        return false;
    }

    // returns false if rsc_setpoint is out of range
    if ( _main_rotor._rsc_setpoint.get() > 100 || _main_rotor._rsc_setpoint.get() < 10){
        hal.util->snprintf(buffer, buflen, "H_RSC_SETPOINT out of range");
        return false;
    }

    // returns false if idle output is out of range
    if ( _main_rotor._idle_output.get() > 100 || _main_rotor._idle_output.get() < 0){
        hal.util->snprintf(buffer, buflen, "H_RSC_IDLE out of range");
        return false;
    }

    // returns false if _rsc_critical is not between 0 and 100
    if (_main_rotor._critical_speed.get() > 100 || _main_rotor._critical_speed.get() < 0) {
        hal.util->snprintf(buffer, buflen, "H_RSC_CRITICAL out of range");
        return false;
    }

    // returns false if RSC Runup Time is less than Ramp time as this could cause undesired behaviour of rotor speed estimate
    if (_main_rotor._runup_time.get() <= _main_rotor._ramp_time.get()){
        hal.util->snprintf(buffer, buflen, "H_RUNUP_TIME too small");
        return false;
    }

    // returns false if _collective_min_deg is not default value which indicates users set parameter
    if (is_equal((float)_collective_min_deg, (float)AP_MOTORS_HELI_COLLECTIVE_MIN_DEG)) {
        hal.util->snprintf(buffer, buflen, "Set H_COL_ANG_MIN to measured min blade pitch in deg");
        return false;
    }

    // returns false if _collective_max_deg is not default value which indicates users set parameter
    if (is_equal((float)_collective_max_deg, (float)AP_MOTORS_HELI_COLLECTIVE_MAX_DEG)) {
        hal.util->snprintf(buffer, buflen, "Set H_COL_ANG_MAX to measured max blade pitch in deg");
        return false;
    }

    return true;
}

// Tell user motor test is disabled on heli
bool AP_MotorsHeli::motor_test_checks(size_t buflen, char *buffer) const
{
    hal.util->snprintf(buffer, buflen, "Disabled on heli");
    return false;
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsHeli::get_motor_mask()
{
    return _main_rotor.get_output_mask();
}

// set_desired_rotor_speed
void AP_MotorsHeli::set_desired_rotor_speed(float desired_speed)
{
    _main_rotor.set_desired_speed(desired_speed);
}

// Converts AP_Motors::SpoolState from _spool_state variable to AP_MotorsHeli_RSC::RotorControlState
AP_MotorsHeli_RSC::RotorControlState AP_MotorsHeli::get_rotor_control_state() const
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            return AP_MotorsHeli_RSC::RotorControlState::STOP;
        case SpoolState::GROUND_IDLE:
            // sends idle output to motors when armed. rotor could be static or turning (autorotation)
            return AP_MotorsHeli_RSC::RotorControlState::IDLE;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
            // set motor output based on thrust requests
            return AP_MotorsHeli_RSC::RotorControlState::ACTIVE;
        case SpoolState::SPOOLING_DOWN:
            // sends idle output to motors and wait for rotor to stop
            return AP_MotorsHeli_RSC::RotorControlState::IDLE;
    }

    // Should be unreachable, but needed to keep the compiler happy
    return AP_MotorsHeli_RSC::RotorControlState::STOP;
}

// Update _heliflags.rotor_runup_complete value writing log event on state change
void AP_MotorsHeli::set_rotor_runup_complete(bool new_value)
{
#if HAL_LOGGING_ENABLED
    if (!_heliflags.rotor_runup_complete && new_value) {
        LOGGER_WRITE_EVENT(LogEvent::ROTOR_RUNUP_COMPLETE);
    } else if (_heliflags.rotor_runup_complete && !new_value && !_heliflags.in_autorotation) {
        LOGGER_WRITE_EVENT(LogEvent::ROTOR_SPEED_BELOW_CRITICAL);
    }
#endif
    _heliflags.rotor_runup_complete = new_value;
}

#if HAL_LOGGING_ENABLED
// Returns the scaling value required to convert the collective angle parameters into the cyclic-output-to-angle conversion
float AP_MotorsHeli::get_cyclic_angle_scaler(void) const {
    // We want to use the collective min-max to angle relationship to calculate the cyclic input to angle relationship
    // First we scale the collective angle range by it's min-max output. Recall that we assume that the maximum possible
    // collective range is 1000, hence the *1e-3.
    // The factor 2.0 accounts for the fact that we scale the servo outputs from 0 ~ 1 to -1 ~ 1
    return ((float)(_collective_max-_collective_min))*1e-3 * (_collective_max_deg.get() - _collective_min_deg.get()) * 2.0;
}
#endif
