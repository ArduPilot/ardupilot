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

/*
 *       AP_MotorsHeli.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsHeli.h"
#include <GCS_MAVLink/GCS.h>

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

    // @Param: COL_MID
    // @DisplayName: Zero-Thrust Collective Pitch 
    // @Description: Swash servo position in PWM microseconds corresponding to zero collective pitch (or zero lift for Asymmetrical blades)
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MID", 5, AP_MotorsHeli, _collective_mid, AP_MOTORS_HELI_COLLECTIVE_MID),

    // @Param: SV_MAN
    // @DisplayName: Manual Servo Mode
    // @Description: Manual servo override for swash set-up. Do not set this manually!
    // @Values: 0:Disabled,1:Passthrough,2:Max collective,3:Mid collective,4:Min collective
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
    _servo_mode = SERVO_CONTROL_MODE_AUTOMATED;

    // initialise radio passthrough for collective to middle
    _throttle_radio_passthrough = 0.5f;

    // initialise Servo/PWM ranges and endpoints
    if (!init_outputs()) {
        // don't set initialised_ok
        return;
    }

    // calculate all scalars
    calculate_scalars();

    // record successful initialisation if what we setup was the desired frame_class
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_HELI);

    // set flag to true so targets are initialized once aircraft is armed for first time
    _heliflags.init_targets_on_arming = true;

}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsHeli::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_HELI);
}

// output_min - sets servos to neutral point with motors stopped
void AP_MotorsHeli::output_min()
{
    // move swash to mid
    move_actuators(0.0f,0.0f,0.5f,0.0f);

    update_motor_control(ROTOR_CONTROL_STOP);

    // override limits flags
    limit.roll = true;
    limit.pitch = true;
    limit.yaw = true;
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

    if (_flags.armed) {
        calculate_armed_scalars();
        if (!_flags.interlock) {
            output_armed_zero_throttle();
        } else {
            output_armed_stabilizing();
        }
    } else {
        output_disarmed();
    }
    
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

// output_armed_zero_throttle - sends commands to the motors
void AP_MotorsHeli::output_armed_zero_throttle()
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
        // perform boot-up servo test cycle if enabled
        servo_test();
    } else {
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
                _throttle_filter.reset(_collective_mid_pct);
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
    if (_flags.armed) {
        if (!_flags.interlock) {
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

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DesiredSpoolState::SHUT_DOWN) {
                _spool_state = SpoolState::GROUND_IDLE;
                break;
            }

            break;

        case SpoolState::GROUND_IDLE: {
            // Motors should be stationary or at ground idle.
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

            // make sure the motors are spooling in the correct direction
            if (_spool_desired != DesiredSpoolState::THROTTLE_UNLIMITED) {
                _spool_state = SpoolState::SPOOLING_DOWN;
                break;
            }


            break;

        case SpoolState::SPOOLING_DOWN:
            // Maximum throttle should move from maximum to minimum.
            // Servos should exhibit normal flight behavior.

            // make sure the motors are spooling in the correct direction
            if (_spool_desired == DesiredSpoolState::THROTTLE_UNLIMITED) {
                _spool_state = SpoolState::SPOOLING_UP;
                break;
            }
            if (!rotor_speed_above_critical()){
                _spool_state = SpoolState::GROUND_IDLE;
            }
            break;
    }
}

// parameter_check - check if helicopter specific parameters are sensible
bool AP_MotorsHeli::parameter_check(bool display_msg) const
{
    // returns false if RSC Mode is not set to a valid control mode
    if (_main_rotor._rsc_mode.get() <= (int8_t)ROTOR_CONTROL_MODE_DISABLED || _main_rotor._rsc_mode.get() > (int8_t)ROTOR_CONTROL_MODE_CLOSED_LOOP_POWER_OUTPUT) {
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_RSC_MODE invalid");
        }
        return false;
    }

    // returns false if rsc_setpoint is out of range
    if ( _main_rotor._rsc_setpoint.get() > 100 || _main_rotor._rsc_setpoint.get() < 10){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_RSC_SETPOINT out of range");
        }
        return false;
    }

    // returns false if idle output is out of range
    if ( _main_rotor._idle_output.get() > 100 || _main_rotor._idle_output.get() < 0){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_RSC_IDLE out of range");
        }
        return false;
    }

    // returns false if _rsc_critical is not between 0 and 100
    if (_main_rotor._critical_speed.get() > 100 || _main_rotor._critical_speed.get() < 0) {
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_RSC_CRITICAL out of range");
        }
        return false;
    }

    // returns false if RSC Runup Time is less than Ramp time as this could cause undesired behaviour of rotor speed estimate
    if (_main_rotor._runup_time.get() <= _main_rotor._ramp_time.get()){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_RUNUP_TIME too small");
        }
        return false;
    }

    // all other cases parameters are OK
    return true;
}

// reset_swash_servo
void AP_MotorsHeli::reset_swash_servo(SRV_Channel::Aux_servo_function_t function)
{
    // outputs are defined on a -500 to 500 range for swash servos
    SRV_Channels::set_range(function, 1000);

    // swash servos always use full endpoints as restricting them would lead to scaling errors
    SRV_Channels::set_output_min_max(function, 1000, 2000);
}

// update the throttle input filter
void AP_MotorsHeli::update_throttle_filter()
{
    _throttle_filter.apply(_throttle_in, 1.0f/_loop_rate);

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
    _servo_mode = SERVO_CONTROL_MODE_AUTOMATED;
    init_outputs();
    calculate_scalars();
}

// convert input in -1 to +1 range to pwm output for swashplate servo.
// The value 0 corresponds to the trim value of the servo. Swashplate
// servo travel range is fixed to 1000 pwm and therefore the input is
// multiplied by 500 to get PWM output.
void AP_MotorsHeli::rc_write_swash(uint8_t chan, float swash_in)
{
    uint16_t pwm = (uint16_t)(1500 + 500 * swash_in);
    SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(chan);
    SRV_Channels::set_output_pwm_trimmed(function, pwm);
}

