/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsHeli_Dual.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Dual::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),

    // Indices 1-6 were used by servo position params and should not be used

    // Indices 7-8 were used by phase angle params and should not be used

    // @Param: DUAL_MODE
    // @DisplayName: Dual Mode
    // @Description: Sets the dual mode of the heli, either as tandem or as transverse.
    // @Values: 0:Longitudinal, 1:Transverse
    // @User: Standard
    AP_GROUPINFO("DUAL_MODE", 9, AP_MotorsHeli_Dual, _dual_mode, AP_MOTORS_HELI_DUAL_MODE_TANDEM),

    // @Param: DCP_SCALER
    // @DisplayName: Differential-Collective-Pitch Scaler
    // @Description: Scaling factor applied to the differential-collective-pitch
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("DCP_SCALER", 10, AP_MotorsHeli_Dual, _dcp_scaler, AP_MOTORS_HELI_DUAL_DCP_SCALER),

    // @Param: DCP_YAW
    // @DisplayName: Differential-Collective-Pitch Yaw Mixing
    // @Description: Feed-forward compensation to automatically add yaw input when differential collective pitch is applied.
    // @Range: -10 10
    // @Increment: 0.1
    AP_GROUPINFO("DCP_YAW", 11, AP_MotorsHeli_Dual, _dcp_yaw_effect, 0),

    // @Param: YAW_SCALER
    // @DisplayName: Scaler for yaw mixing
    // @Description: Scaler for mixing yaw into roll or pitch.
    // @Range: -10 10
    // @Increment: 0.1
    AP_GROUPINFO("YAW_SCALER", 12, AP_MotorsHeli_Dual, _yaw_scaler, 1.0f),

    // Indices 13-15 were used by RSC_PWM_MIN, RSC_PWM_MAX and RSC_PWM_REV and should not be used

    // @Param: COL2_MIN
    // @DisplayName: Collective Pitch Minimum for rear swashplate
    // @Description: Lowest possible servo position in PWM microseconds for the rear swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL2_MIN", 16, AP_MotorsHeli_Dual, _collective2_min, AP_MOTORS_HELI_DUAL_COLLECTIVE2_MIN),

    // @Param: COL2_MAX
    // @DisplayName: Collective Pitch Maximum for rear swashplate
    // @Description: Highest possible servo position in PWM microseconds for the rear swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL2_MAX", 17, AP_MotorsHeli_Dual, _collective2_max, AP_MOTORS_HELI_DUAL_COLLECTIVE2_MAX),

    // @Param: COL2_MID
    // @DisplayName: Collective Pitch Mid-Point for rear swashplate
    // @Description: Swash servo position in PWM microseconds corresponding to zero collective pitch for the rear swashplate (or zero lift for Asymmetrical blades)
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL2_MID", 18, AP_MotorsHeli_Dual, _collective2_mid, AP_MOTORS_HELI_DUAL_COLLECTIVE2_MID),

    // @Param: COL_CTRL_DIR
    // @DisplayName: Collective Control Direction for swash 1
    // @Description: Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    AP_GROUPINFO("COL_CTRL_DIR", 19, AP_MotorsHeli_Dual, _swash1_coll_dir, COLLECTIVE_DIRECTION_NORMAL),

    // @Param: COL_CTRL_DIR2
    // @DisplayName: Collective Control Direction for swash 2
    // @Description: Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    AP_GROUPINFO("COL_CTRL_DIR2", 20, AP_MotorsHeli_Dual, _swash2_coll_dir, COLLECTIVE_DIRECTION_NORMAL),

    // @Param: SWASH1_TYPE
    // @DisplayName: Swashplate Type for swashplate 1
    // @Description: H3 is generic, three-servo only. H3_120/H3_140 plates have Motor1 left side, Motor2 right side, Motor3 elevator in rear. HR3_120/HR3_140 have Motor1 right side, Motor2 left side, Motor3 elevator in front - use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 Motors1&2 are left/right respectively, Motors3&4 are rear/front respectively. For H4-45 Motors1&2 are LF/RF, Motors3&4 are LR/RR 
    // @Values: 0:H3 Generic, 1:H1 non-CPPM, 2:H3_140, 3:H3_120, 4:H4_90, 5:H4_45
    // @User: Standard
    AP_GROUPINFO("SWASH1_TYPE", 21, AP_MotorsHeli_Dual, _swashplate1_type, SWASHPLATE_TYPE_H3),

    // @Param: SWASH2_TYPE
    // @DisplayName: Swashplate Type for swashplate 2
    // @Description: H3 is generic, three-servo only. H3_120/H3_140 plates have Motor1 left side, Motor2 right side, Motor3 elevator in rear. HR3_120/HR3_140 have Motor1 right side, Motor2 left side, Motor3 elevator in front - use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 Motors1&2 are left/right respectively, Motors3&4 are rear/front respectively. For H4-45 Motors1&2 are LF/RF, Motors3&4 are LR/RR 
    // @Values: 0:H3 Generic, 1:H1 non-CPPM, 2:H3_140, 3:H3_120, 4:H4_90, 5:H4_45
    // @User: Standard
    AP_GROUPINFO("SWASH2_TYPE", 22, AP_MotorsHeli_Dual, _swashplate2_type, SWASHPLATE_TYPE_H3),

    // @Group: SW1_H3_
    // @Path: AP_MotorsHeli_Swash.cpp
    AP_SUBGROUPINFO(_swashplate1, "SW1_H3_", 23, AP_MotorsHeli_Dual, AP_MotorsHeli_Swash),

    // @Group: SW2_H3_
    // @Path: AP_MotorsHeli_Swash.cpp
    AP_SUBGROUPINFO(_swashplate2, "SW2_H3_", 24, AP_MotorsHeli_Dual, AP_MotorsHeli_Swash),

    AP_GROUPEND
};

// set update rate to motors - a value in hertz
void AP_MotorsHeli_Dual::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint16_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS; i++) {
        mask |= 1U << (AP_MOTORS_MOT_1+i);
    }
    if (_swashplate1_type == SWASHPLATE_TYPE_H4_90 || _swashplate1_type == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << (AP_MOTORS_MOT_7);
    }
    if (_swashplate2_type == SWASHPLATE_TYPE_H4_90 || _swashplate2_type == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << (AP_MOTORS_MOT_8);
    }

    rc_set_freq(mask, _speed_hz);
}

// init_outputs
bool AP_MotorsHeli_Dual::init_outputs()
{
    if (!_flags.initialised_ok) {
        // make sure 6 output channels are mapped
        for (uint8_t i=0; i<AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS; i++) {
            add_motor_num(CH_1+i);
        }
        if (_swashplate1_type == SWASHPLATE_TYPE_H4_90 || _swashplate1_type == SWASHPLATE_TYPE_H4_45) {
            add_motor_num(CH_7);
        }
        if (_swashplate2_type == SWASHPLATE_TYPE_H4_90 || _swashplate2_type == SWASHPLATE_TYPE_H4_45) {
            add_motor_num(CH_8);
        }

        // set rotor servo range
        _rotor.init_servo();

    }

    // reset swash servo range and endpoints
    for (uint8_t i=0; i<AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS; i++) {
        reset_swash_servo(SRV_Channels::get_motor_function(i));
    }
    if (_swashplate1_type == SWASHPLATE_TYPE_H4_90 || _swashplate1_type == SWASHPLATE_TYPE_H4_45) {
        reset_swash_servo(SRV_Channels::get_motor_function(6));
    }
    if (_swashplate2_type == SWASHPLATE_TYPE_H4_90 || _swashplate2_type == SWASHPLATE_TYPE_H4_45) {
        reset_swash_servo(SRV_Channels::get_motor_function(7));
    }

    _flags.initialised_ok = true;

    return true;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Dual::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
    case 1:
        // swash servo 1
        rc_write(AP_MOTORS_MOT_1, pwm);
        break;
    case 2:
        // swash servo 2
        rc_write(AP_MOTORS_MOT_2, pwm);
        break;
    case 3:
        // swash servo 3
        rc_write(AP_MOTORS_MOT_3, pwm);
        break;
    case 4:
        // swash servo 4
        rc_write(AP_MOTORS_MOT_4, pwm);
        break;
    case 5:
        // swash servo 5
        rc_write(AP_MOTORS_MOT_5, pwm);
        break;
    case 6:
        // swash servo 6
        rc_write(AP_MOTORS_MOT_6, pwm);
        break;
    case 7:
        // main rotor
        rc_write(AP_MOTORS_HELI_DUAL_RSC, pwm);
        break;
    default:
        // do nothing
        break;
    }
}

// set_desired_rotor_speed
void AP_MotorsHeli_Dual::set_desired_rotor_speed(float desired_speed)
{
    _rotor.set_desired_speed(desired_speed);
}

// calculate_armed_scalars
void AP_MotorsHeli_Dual::calculate_armed_scalars()
{
    float thrcrv[5];
    for (uint8_t i = 0; i < 5; i++) {
        thrcrv[i]=_rsc_thrcrv[i]*0.001f;
    } 
    _rotor.set_ramp_time(_rsc_ramp_time);
    _rotor.set_runup_time(_rsc_runup_time);
    _rotor.set_critical_speed(_rsc_critical*0.001f);
    _rotor.set_idle_output(_rsc_idle_output*0.001f);
    _rotor.set_throttle_curve(thrcrv, (uint16_t)_rsc_slewrate.get());
}

// calculate_scalars
void AP_MotorsHeli_Dual::calculate_scalars()
{
    // range check collective min, max and mid
    if( _collective_min >= _collective_max ) {
        _collective_min = AP_MOTORS_HELI_COLLECTIVE_MIN;
        _collective_max = AP_MOTORS_HELI_COLLECTIVE_MAX;
    }


    // range check collective min, max and mid for rear swashplate
    if( _collective2_min >= _collective2_max ) {
        _collective2_min = AP_MOTORS_HELI_DUAL_COLLECTIVE2_MIN;
        _collective2_max = AP_MOTORS_HELI_DUAL_COLLECTIVE2_MAX;
    }

    _collective_mid = constrain_int16(_collective_mid, _collective_min, _collective_max);
    _collective2_mid = constrain_int16(_collective2_mid, _collective2_min, _collective2_max);

    // calculate collective mid point as a number from 0 to 1000
    _collective_mid_pct = ((float)(_collective_mid-_collective_min))/((float)(_collective_max-_collective_min));
    _collective2_mid_pct = ((float)(_collective2_mid-_collective2_min))/((float)(_collective2_max-_collective2_min));

    // configure swashplate 1 and update scalars
    if (_swashplate1_type == SWASHPLATE_TYPE_H3) {
        if (_swashplate1.get_enable() == 0) {
            _swashplate1.set_enable(1);
        }
    } else {
        if (_swashplate1.get_enable() == 1) {
            _swashplate1.set_enable(0);
        }
    }
    _swashplate1.set_swash_type(static_cast<SwashPlateType>(_swashplate1_type.get()));
    _swashplate1.set_collective_direction(static_cast<CollectiveDirection>(_swash1_coll_dir.get()));
    _swashplate1.calculate_roll_pitch_collective_factors();

    // configure swashplate 2 and update scalars
    if (_swashplate2_type == SWASHPLATE_TYPE_H3) {
        if (_swashplate2.get_enable() == 0) {
            _swashplate2.set_enable(1);
        }
    } else {
        if (_swashplate2.get_enable() == 1) {
            _swashplate2.set_enable(0);
        }
    }
    _swashplate2.set_swash_type(static_cast<SwashPlateType>(_swashplate2_type.get()));
    _swashplate2.set_collective_direction(static_cast<CollectiveDirection>(_swash2_coll_dir.get()));
    _swashplate2.calculate_roll_pitch_collective_factors();

    // set mode of main rotor controller and trigger recalculation of scalars
    _rotor.set_control_mode(static_cast<RotorControlMode>(_rsc_mode.get()));
    calculate_armed_scalars();
}

// get_swashplate - calculate movement of each swashplate based on configuration
float AP_MotorsHeli_Dual::get_swashplate (int8_t swash_num, int8_t swash_axis, float pitch_input, float roll_input, float yaw_input, float coll_input)
{
    float swash_tilt = 0.0f;
    if (_dual_mode == AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE) {
        // roll tilt
        if (swash_axis == AP_MOTORS_HELI_DUAL_SWASH_AXIS_ROLL) {
            if (swash_num == 1) {
                swash_tilt = 0.0f;
            } else if (swash_num == 2) {
                swash_tilt = 0.0f;
            }
        } else if (swash_axis == AP_MOTORS_HELI_DUAL_SWASH_AXIS_PITCH) {
        // pitch tilt
            if (swash_num == 1) {
                swash_tilt = pitch_input - _yaw_scaler * yaw_input;
            } else if (swash_num == 2) {
                swash_tilt = pitch_input + _yaw_scaler * yaw_input;
            }
        } else if (swash_axis == AP_MOTORS_HELI_DUAL_SWASH_AXIS_COLL) {
        // collective
            if (swash_num == 1) {
                swash_tilt = 0.45f * _dcp_scaler * roll_input + coll_input;
            } else if (swash_num == 2) {
                swash_tilt = -0.45f * _dcp_scaler * roll_input + coll_input;
            }
        }
    } else { // AP_MOTORS_HELI_DUAL_MODE_TANDEM
        // roll tilt
        if (swash_axis == AP_MOTORS_HELI_DUAL_SWASH_AXIS_ROLL) {
            if (swash_num == 1) {
                swash_tilt = roll_input + _yaw_scaler * yaw_input;
            } else if (swash_num == 2) {
                swash_tilt = roll_input - _yaw_scaler * yaw_input;
            }
        } else if (swash_axis == AP_MOTORS_HELI_DUAL_SWASH_AXIS_PITCH) {
        // pitch tilt
            if (swash_num == 1) {
                swash_tilt = 0.0f;
            } else if (swash_num == 2) {
                swash_tilt = 0.0f;
            }
        } else if (swash_axis == AP_MOTORS_HELI_DUAL_SWASH_AXIS_COLL) {
        // collective
            if (swash_num == 1) {
                swash_tilt = 0.45f * _dcp_scaler * pitch_input + coll_input;
            } else if (swash_num == 2) {
                swash_tilt = -0.45f * _dcp_scaler * pitch_input + coll_input;
            }
        }
    }
    return swash_tilt;
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeli_Dual::get_motor_mask()
{
    // dual heli uses channels 1,2,3,4,5,6 and 8
    uint16_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS; i++) {
        mask |= 1U << (AP_MOTORS_MOT_1+i);
    }
    if (_swashplate1_type == SWASHPLATE_TYPE_H4_90 || _swashplate1_type == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << AP_MOTORS_MOT_7;
    }
    if (_swashplate2_type == SWASHPLATE_TYPE_H4_90 || _swashplate2_type == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << AP_MOTORS_MOT_8;
    }
    mask |= 1U << AP_MOTORS_HELI_DUAL_RSC;
    return mask;
}

// update_motor_controls - sends commands to motor controllers
void AP_MotorsHeli_Dual::update_motor_control(RotorControlState state)
{
    // Send state update to motors
    _rotor.output(state);

    if (state == ROTOR_CONTROL_STOP) {
        // set engine run enable aux output to not run position to kill engine when disarmed
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    } else {
        // else if armed, set engine run enable output to run position
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    }

    // Check if rotors are run-up
    _heliflags.rotor_runup_complete = _rotor.is_runup_complete();
}

//
// move_actuators - moves swash plate to attitude of parameters passed in
//                - expected ranges:
//                       roll : -1 ~ +1
//                       pitch: -1 ~ +1
//                       collective: 0 ~ 1
//                       yaw:   -1 ~ +1
//
void AP_MotorsHeli_Dual::move_actuators(float roll_out, float pitch_out, float collective_in, float yaw_out)
{
    // initialize limits flag
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    if (_dual_mode == AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE) {
        if (pitch_out < -_cyclic_max/4500.0f) {
            pitch_out = -_cyclic_max/4500.0f;
            limit.roll_pitch = true;
        }

        if (pitch_out > _cyclic_max/4500.0f) {
            pitch_out = _cyclic_max/4500.0f;
            limit.roll_pitch = true;
        }
    } else {
        if (roll_out < -_cyclic_max/4500.0f) {
            roll_out = -_cyclic_max/4500.0f;
            limit.roll_pitch = true;
        }

        if (roll_out > _cyclic_max/4500.0f) {
            roll_out = _cyclic_max/4500.0f;
            limit.roll_pitch = true;
        }
    }

    if (_heliflags.inverted_flight) {
        collective_in = 1 - collective_in;
    }

    float yaw_compensation = 0.0f;

    // if servo output not in manual mode, process pre-compensation factors
    if (_servo_mode == SERVO_CONTROL_MODE_AUTOMATED) {
        // add differential collective pitch yaw compensation
        if (_dual_mode == AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE) {
            yaw_compensation = _dcp_yaw_effect * roll_out;
        } else { // AP_MOTORS_HELI_DUAL_MODE_TANDEM
            yaw_compensation = _dcp_yaw_effect * pitch_out;
        }
        yaw_out = yaw_out + yaw_compensation;
    }

    // scale yaw and update limits
    if (yaw_out < -_cyclic_max/4500.0f) {
        yaw_out = -_cyclic_max/4500.0f;
        limit.yaw = true;
    }
    if (yaw_out > _cyclic_max/4500.0f) {
        yaw_out = _cyclic_max/4500.0f;
        limit.yaw = true;
    }

    // constrain collective input
    float collective_out = collective_in;
    if (collective_out <= 0.0f) {
        collective_out = 0.0f;
        limit.throttle_lower = true;
    }
    if (collective_out >= 1.0f) {
        collective_out = 1.0f;
        limit.throttle_upper = true;
    }

    // ensure not below landed/landing collective
    if (_heliflags.landing_collective && collective_out < (_land_collective_min*0.001f)) {
        collective_out = _land_collective_min*0.001f;
        limit.throttle_lower = true;
    }

    // Set rear collective to midpoint if required
    float collective2_out = collective_out;
    if (_servo_mode == SERVO_CONTROL_MODE_MANUAL_CENTER) {
        collective2_out = _collective2_mid_pct;
    }

    // scale collective pitch for front swashplate (servos 1,2,3)
    float collective_scaler = ((float)(_collective_max-_collective_min))*0.001f;
    float collective_out_scaled = collective_out * collective_scaler + (_collective_min - 1000)*0.001f;

    // scale collective pitch for rear swashplate (servos 4,5,6)
    float collective2_scaler = ((float)(_collective2_max-_collective2_min))*0.001f;
    float collective2_out_scaled = collective2_out * collective2_scaler + (_collective2_min - 1000)*0.001f;

    // feed power estimate into main rotor controller
    // ToDo: add main rotor cyclic power?
    _rotor.set_collective(fabsf(collective_out));

    // compute swashplate tilt
    float swash1_pitch = get_swashplate(1, AP_MOTORS_HELI_DUAL_SWASH_AXIS_PITCH, pitch_out, roll_out, yaw_out, collective_out_scaled);
    float swash1_roll = get_swashplate(1, AP_MOTORS_HELI_DUAL_SWASH_AXIS_ROLL, pitch_out, roll_out, yaw_out, collective_out_scaled);
    float swash1_coll = get_swashplate(1, AP_MOTORS_HELI_DUAL_SWASH_AXIS_COLL, pitch_out, roll_out, yaw_out, collective_out_scaled);
    float swash2_pitch = get_swashplate(2, AP_MOTORS_HELI_DUAL_SWASH_AXIS_PITCH, pitch_out, roll_out, yaw_out, collective2_out_scaled);
    float swash2_roll = get_swashplate(2, AP_MOTORS_HELI_DUAL_SWASH_AXIS_ROLL, pitch_out, roll_out, yaw_out, collective2_out_scaled);
    float swash2_coll = get_swashplate(2, AP_MOTORS_HELI_DUAL_SWASH_AXIS_COLL, pitch_out, roll_out, yaw_out, collective2_out_scaled);
 
    // get servo positions from swashplate library
    _servo_out[CH_1] = _swashplate1.get_servo_out(CH_1,swash1_pitch,swash1_roll,swash1_coll);
    _servo_out[CH_2] = _swashplate1.get_servo_out(CH_2,swash1_pitch,swash1_roll,swash1_coll);
    _servo_out[CH_3] = _swashplate1.get_servo_out(CH_3,swash1_pitch,swash1_roll,swash1_coll);
    if (_swashplate1_type == SWASHPLATE_TYPE_H4_90 || _swashplate1_type == SWASHPLATE_TYPE_H4_45) {
        _servo_out[CH_7] = _swashplate1.get_servo_out(CH_4,swash1_pitch,swash1_roll,swash1_coll);
    }

    // get servo positions from swashplate library
    _servo_out[CH_4] = _swashplate2.get_servo_out(CH_1,swash2_pitch,swash2_roll,swash2_coll);
    _servo_out[CH_5] = _swashplate2.get_servo_out(CH_2,swash2_pitch,swash2_roll,swash2_coll);
    _servo_out[CH_6] = _swashplate2.get_servo_out(CH_3,swash2_pitch,swash2_roll,swash2_coll);
    if (_swashplate2_type == SWASHPLATE_TYPE_H4_90 || _swashplate2_type == SWASHPLATE_TYPE_H4_45) {
        _servo_out[CH_8] = _swashplate2.get_servo_out(CH_4,swash2_pitch,swash2_roll,swash2_coll);
    }

}

void AP_MotorsHeli_Dual::output_to_motors()
{
    if (!_flags.initialised_ok) {
        return;
    }
    // actually move the servos.  PWM is sent based on nominal 1500 center.  servo output shifts center based on trim value.
    for (uint8_t i=0; i<AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS; i++) {
        rc_write_swash(i, _servo_out[CH_1+i]);
    }

    // write to servo for 4 servo of 4 servo swashplate
    if (_swashplate1_type == SWASHPLATE_TYPE_H4_90 || _swashplate1_type == SWASHPLATE_TYPE_H4_45) {
        rc_write_swash(AP_MOTORS_MOT_7, _servo_out[CH_7]);
    }
    // write to servo for 4 servo of 4 servo swashplate
    if (_swashplate2_type == SWASHPLATE_TYPE_H4_90 || _swashplate2_type == SWASHPLATE_TYPE_H4_45) {
        rc_write_swash(AP_MOTORS_MOT_8, _servo_out[CH_8]);
    }

    switch (_spool_mode) {
        case SHUT_DOWN:
            // sends minimum values out to the motors
            update_motor_control(ROTOR_CONTROL_STOP);
            break;
        case GROUND_IDLE:
            // sends idle output to motors when armed. rotor could be static or turning (autorotation)
            update_motor_control(ROTOR_CONTROL_IDLE);
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
            // set motor output based on thrust requests
            update_motor_control(ROTOR_CONTROL_ACTIVE);
            break;
        case SPOOL_DOWN:
            // sends idle output to motors and wait for rotor to stop
            update_motor_control(ROTOR_CONTROL_IDLE);
            break;

    }
}

// servo_test - move servos through full range of movement
void AP_MotorsHeli_Dual::servo_test()
{
    // this test cycle is equivalent to that of AP_MotorsHeli_Single, but excluding
    // mixing of yaw, as that physical movement is represented by pitch and roll

    _servo_test_cycle_time += 1.0f / _loop_rate;

    if ((_servo_test_cycle_time >= 0.0f && _servo_test_cycle_time < 0.5f)||                                   // Tilt swash back
        (_servo_test_cycle_time >= 6.0f && _servo_test_cycle_time < 6.5f)){
        _pitch_test += (1.0f / (_loop_rate/2));
        _oscillate_angle += 8 * M_PI / _loop_rate;
    } else if ((_servo_test_cycle_time >= 0.5f && _servo_test_cycle_time < 4.5f)||                            // Roll swash around
               (_servo_test_cycle_time >= 6.5f && _servo_test_cycle_time < 10.5f)){
        _oscillate_angle += M_PI / (2 * _loop_rate);
        _roll_test = sinf(_oscillate_angle);
        _pitch_test = cosf(_oscillate_angle);
    } else if ((_servo_test_cycle_time >= 4.5f && _servo_test_cycle_time < 5.0f)||                            // Return swash to level
               (_servo_test_cycle_time >= 10.5f && _servo_test_cycle_time < 11.0f)){
        _pitch_test -= (1.0f / (_loop_rate/2));
        _oscillate_angle += 8 * M_PI / _loop_rate;
    } else if (_servo_test_cycle_time >= 5.0f && _servo_test_cycle_time < 6.0f){                              // Raise swash to top
        _collective_test += (1.0f / _loop_rate);
        _oscillate_angle += 2 * M_PI / _loop_rate;
    } else if (_servo_test_cycle_time >= 11.0f && _servo_test_cycle_time < 12.0f){                            // Lower swash to bottom
        _collective_test -= (1.0f / _loop_rate);
        _oscillate_angle += 2 * M_PI / _loop_rate;
    } else {                                                                                                  // reset cycle
        _servo_test_cycle_time = 0.0f;
        _oscillate_angle = 0.0f;
        _collective_test = 0.0f;
        _roll_test = 0.0f;
        _pitch_test = 0.0f;
        // decrement servo test cycle counter at the end of the cycle
        if (_servo_test_cycle_counter > 0){
            _servo_test_cycle_counter--;
        }
    }

    // over-ride servo commands to move servos through defined ranges

    _throttle_filter.reset(constrain_float(_collective_test, 0.0f, 1.0f));
    _roll_in = constrain_float(_roll_test, -1.0f, 1.0f);
    _pitch_in = constrain_float(_pitch_test, -1.0f, 1.0f);
}

// parameter_check - check if helicopter specific parameters are sensible
bool AP_MotorsHeli_Dual::parameter_check(bool display_msg) const
{
    // returns false if Phase Angle is outside of range for H3 swashplate 1
    if (_swashplate1_type == SWASHPLATE_TYPE_H3 && (_swashplate1.get_phase_angle() > 30 || _swashplate1.get_phase_angle() < -30)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_SW1_H3_PHANG out of range");
        }
        return false;
    }

    // returns false if Phase Angle is outside of range for H3 swashplate 2
    if (_swashplate2_type == SWASHPLATE_TYPE_H3 && (_swashplate2.get_phase_angle() > 30 || _swashplate2.get_phase_angle() < -30)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_SW2_H3_PHANG out of range");
        }
        return false;
    }

    // check parent class parameters
    return AP_MotorsHeli::parameter_check(display_msg);
}
