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
    // @Description: Sets the dual mode of the heli, either as tandem, transverse, or intermeshing/coaxial.
    // @Values: 0:Longitudinal, 1:Transverse, 2:Intermeshing/Coaxial
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
    // @Description: Feed-forward compensation to automatically add yaw input when differential collective pitch is applied.  Disabled for intermeshing mode.
    // @Range: -10 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("DCP_YAW", 11, AP_MotorsHeli_Dual, _dcp_yaw_effect, 0),

    // @Param: YAW_SCALER
    // @DisplayName: Scaler for yaw mixing
    // @Description: Scaler for mixing yaw into roll or pitch.
    // @Range: -10 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("YAW_SCALER", 12, AP_MotorsHeli_Dual, _yaw_scaler, 1.0f),

    // Indices 13-15 were used by RSC_PWM_MIN, RSC_PWM_MAX and RSC_PWM_REV and should not be used

    // @Param: COL2_MIN
    // @DisplayName: Swash 2 Minimum Collective Pitch
    // @Description: Lowest possible servo position in PWM microseconds for swashplate 2
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL2_MIN", 16, AP_MotorsHeli_Dual, _collective2_min, AP_MOTORS_HELI_DUAL_COLLECTIVE2_MIN),

    // @Param: COL2_MAX
    // @DisplayName: Swash 2 Maximum Collective Pitch
    // @Description: Highest possible servo position in PWM microseconds for swashplate 2
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL2_MAX", 17, AP_MotorsHeli_Dual, _collective2_max, AP_MOTORS_HELI_DUAL_COLLECTIVE2_MAX),

    // Indice 18 was used by COL2_MID and should not be used

    // Indice 19 was used by COL_CTRL_DIR and should not be used

    // @Param: SW_TYPE
    // @DisplayName: Swash 1 Type
    // @Description: H3 is generic, three-servo only. H3_120/H3_140 plates have Motor1 left side, Motor2 right side, Motor3 elevator in rear. HR3_120/HR3_140 have Motor1 right side, Motor2 left side, Motor3 elevator in front - use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 Motors1&2 are left/right respectively, Motors3&4 are rear/front respectively. For H4-45 Motors1&2 are LF/RF, Motors3&4 are LR/RR 
    // @Values: 0:H3 Generic,1:H1 non-CPPM,2:H3_140,3:H3_120,4:H4_90,5:H4_45
    // @User: Standard

    // @Param: SW_COL_DIR
    // @DisplayName: Swash 1 Collective Direction
    // @Description: Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed
    // @Values: 0:Normal,1:Reversed
    // @User: Standard

    // @Param: SW_LIN_SVO
    // @DisplayName: Linearize Swash 1 Servos
    // @Description: This linearizes the swashplate 1 servo's mechanical output to account for nonlinear output due to arm rotation.  This requires a specific setup procedure to work properly.  The servo arm must be centered on the mechanical throw at the servo trim position and the servo trim position kept as close to 1500 as possible. Leveling the swashplate can only be done through the pitch links.  See the ardupilot wiki for more details on setup.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard

    // @Param: SW_H3_ENABLE
    // @DisplayName: Swash 1 H3 Generic Enable
    // @Description: Automatically set when H3 generic swash type is selected for swashplate 1. Do not set manually.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced

    // @Param: SW_H3_SV1_POS
    // @DisplayName: Swash 1 H3 Generic Servo 1 Position
    // @Description: Azimuth position on swashplate for servo 1 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced

    // @Param: SW_H3_SV2_POS
    // @DisplayName: Swash 1 H3 Generic Servo 2 Position
    // @Description: Azimuth position on swashplate 1 for servo 2 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced

    // @Param: SW_H3_SV3_POS
    // @DisplayName: Swash 1 H3 Generic Servo 3 Position
    // @Description: Azimuth position on swashplate 1 for servo 3 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced

    // @Param: SW_H3_PHANG
    // @DisplayName: Swash 1 H3 Generic Phase Angle Comp
    // @Description: Only for H3 swashplate.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -30 30
    // @Units: deg
    // @User: Advanced
    // @Increment: 1
    AP_SUBGROUPINFO(_swashplate1, "SW_", 20, AP_MotorsHeli_Dual, AP_MotorsHeli_Swash),

    // @Param: SW2_TYPE
    // @DisplayName: Swash 2 Type
    // @Description: H3 is generic, three-servo only. H3_120/H3_140 plates have Motor1 left side, Motor2 right side, Motor3 elevator in rear. HR3_120/HR3_140 have Motor1 right side, Motor2 left side, Motor3 elevator in front - use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 Motors1&2 are left/right respectively, Motors3&4 are rear/front respectively. For H4-45 Motors1&2 are LF/RF, Motors3&4 are LR/RR 
    // @Values: 0:H3 Generic,1:H1 non-CPPM,2:H3_140,3:H3_120,4:H4_90,5:H4_45
    // @User: Standard

    // @Param: SW2_COL_DIR
    // @DisplayName: Swash 2 Collective Direction
    // @Description: Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed
    // @Values: 0:Normal,1:Reversed
    // @User: Standard

    // @Param: SW2_LIN_SVO
    // @DisplayName: Linearize Swash 2 Servos
    // @Description: This linearizes the swashplate 2 servo's mechanical output to account for nonlinear output due to arm rotation.  This requires a specific setup procedure to work properly.  The servo arm must be centered on the mechanical throw at the servo trim position and the servo trim position kept as close to 1500 as possible. Leveling the swashplate can only be done through the pitch links.  See the ardupilot wiki for more details on setup.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard

    // @Param: SW2_H3_ENABLE
    // @DisplayName: Swash 2 H3 Generic Enable
    // @Description: Automatically set when H3 generic swash type is selected for swashplate 2. Do not set manually.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced

    // @Param: SW2_H3_SV1_POS
    // @DisplayName: Swash 2 H3 Generic Servo 1 Position
    // @Description: Azimuth position on swashplate for servo 1 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced

    // @Param: SW2_H3_SV2_POS
    // @DisplayName: Swash 2 H3 Generic Servo 2 Position
    // @Description: Azimuth position on swashplate 2 for servo 2 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced

    // @Param: SW2_H3_SV3_POS
    // @DisplayName: Swash 2 H3 Generic Servo 3 Position
    // @Description: Azimuth position on swashplate 2 for servo 3 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced

    // @Param: SW2_H3_PHANG
    // @DisplayName: Swash 2 H3 Generic Phase Angle Comp
    // @Description: Only for H3 swashplate.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -30 30
    // @Units: deg
    // @User: Advanced
    // @Increment: 1
    AP_SUBGROUPINFO(_swashplate2, "SW2_", 21, AP_MotorsHeli_Dual, AP_MotorsHeli_Swash),

    // @Param: DCP_TRIM
    // @DisplayName: Differential Collective Pitch Trim
    // @Description: Removes I term bias due to center of gravity offsets or discrepancies between rotors in swashplate setup. If DCP axis has I term bias while hovering in calm winds, use value of bias in DCP_TRIM to re-center I term.
    // @Range: -0.2 0.2
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DCP_TRIM", 22, AP_MotorsHeli_Dual, _dcp_trim, 0.0f),

    // @Param: YAW_REV_EXPO
    // @DisplayName: Yaw reverser expo
    // @Description: For intermeshing mode only. Yaw revereser smoothing exponent, smoothen transition near zero collective region. Increase this parameter to shink smoothing range. Set to -1 to disable reverser. 
    // @Range: -1 1000
    // @Increment: 1.0
    // @User: Standard
    AP_GROUPINFO("YAW_REV_EXPO", 23, AP_MotorsHeli_Dual, _yaw_rev_expo, -1),

    AP_GROUPEND
};

// set update rate to motors - a value in hertz
void AP_MotorsHeli_Dual::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS; i++) {
        mask |= 1U << (AP_MOTORS_MOT_1+i);
    }
    if (_swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << (AP_MOTORS_MOT_7);
    }
    if (_swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << (AP_MOTORS_MOT_8);
    }

    rc_set_freq(mask, _speed_hz);
}

// init_outputs
bool AP_MotorsHeli_Dual::init_outputs()
{
    if (!initialised_ok()) {
        // make sure 6 output channels are mapped
        for (uint8_t i=0; i<AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS; i++) {
            add_motor_num(CH_1+i);
        }
        if (_swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
            add_motor_num(CH_7);
        }
        if (_swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
            add_motor_num(CH_8);
        }

        // set rotor servo range
        _main_rotor.init_servo();

    }

    // set signal value for main rotor external governor to know when to use autorotation bailout ramp up
    if (_main_rotor._rsc_mode.get() == ROTOR_CONTROL_MODE_SETPOINT  ||  _main_rotor._rsc_mode.get() == ROTOR_CONTROL_MODE_PASSTHROUGH) {
        _main_rotor.set_ext_gov_arot_bail(_main_rotor._ext_gov_arot_pct.get());
    } else {
        _main_rotor.set_ext_gov_arot_bail(0);
    }

    // reset swash servo range and endpoints
    for (uint8_t i=0; i<AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS; i++) {
        reset_swash_servo(SRV_Channels::get_motor_function(i));
    }
    if (_swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        reset_swash_servo(SRV_Channels::get_motor_function(6));
    }
    if (_swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        reset_swash_servo(SRV_Channels::get_motor_function(7));
    }

    set_initialised_ok(_frame_class == MOTOR_FRAME_HELI_DUAL);

    return true;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Dual::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
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
        rc_write(AP_MOTORS_HELI_RSC, pwm);
        break;
    default:
        // do nothing
        break;
    }
}

// set_desired_rotor_speed
void AP_MotorsHeli_Dual::set_desired_rotor_speed(float desired_speed)
{
    _main_rotor.set_desired_speed(desired_speed);
}

// calculate_armed_scalars
void AP_MotorsHeli_Dual::calculate_armed_scalars()
{
    // Set rsc mode specific parameters
    if (_main_rotor._rsc_mode.get() == ROTOR_CONTROL_MODE_THROTTLECURVE || _main_rotor._rsc_mode.get() == ROTOR_CONTROL_MODE_AUTOTHROTTLE) {
        _main_rotor.set_throttle_curve();
    }
    // keeps user from changing RSC mode while armed
    if (_main_rotor._rsc_mode.get() != _main_rotor.get_control_mode()) {
        _main_rotor.reset_rsc_mode_param();
        _heliflags.save_rsc_mode = true;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "RSC control mode change failed");
    }
    // saves rsc mode parameter when disarmed if it had been reset while armed
    if (_heliflags.save_rsc_mode && !armed()) {
        _main_rotor._rsc_mode.save();
        _heliflags.save_rsc_mode = false;
    }

    // set bailout ramp time
    _main_rotor.use_bailout_ramp_time(_heliflags.enable_bailout);

    // allow use of external governor autorotation bailout window on main rotor
    if (_main_rotor._ext_gov_arot_pct.get() > 0  &&  (_main_rotor._rsc_mode.get() == ROTOR_CONTROL_MODE_SETPOINT  ||  _main_rotor._rsc_mode.get() == ROTOR_CONTROL_MODE_PASSTHROUGH)){
        // RSC only needs to know that the vehicle is in an autorotation if using the bailout window on an external governor
        _main_rotor.set_autorotation_flag(_heliflags.in_autorotation);
    }
}

// calculate_scalars
void AP_MotorsHeli_Dual::calculate_scalars()
{
    // range check collective min, max and mid
    if( _collective_min >= _collective_max ) {
        _collective_min.set(AP_MOTORS_HELI_COLLECTIVE_MIN);
        _collective_max.set(AP_MOTORS_HELI_COLLECTIVE_MAX);
    }


    // range check collective min, max and mid for rear swashplate
    if( _collective2_min >= _collective2_max ) {
        _collective2_min.set(AP_MOTORS_HELI_DUAL_COLLECTIVE2_MIN);
        _collective2_max.set(AP_MOTORS_HELI_DUAL_COLLECTIVE2_MAX);
    }

    _collective_zero_thrust_deg.set(constrain_float(_collective_zero_thrust_deg, _collective_min_deg, _collective_max_deg));

    _collective_land_min_deg.set(constrain_float(_collective_land_min_deg, _collective_min_deg, _collective_max_deg));

    if (!is_equal((float)_collective_max_deg, (float)_collective_min_deg)) {
        // calculate collective zero thrust point as a number from 0 to 1
        _collective_zero_thrust_pct = (_collective_zero_thrust_deg-_collective_min_deg)/(_collective_max_deg-_collective_min_deg);

        // calculate collective land min point as a number from 0 to 1
        _collective_land_min_pct = (_collective_land_min_deg-_collective_min_deg)/(_collective_max_deg-_collective_min_deg);
    } else {
        _collective_zero_thrust_pct = 0.0f;
        _collective_land_min_pct = 0.0f;
    }

    _collective2_zero_thrst_pct = _collective_zero_thrust_pct;

    // configure swashplate 1 and update scalars
    _swashplate1.configure();
    _swashplate1.calculate_roll_pitch_collective_factors();

    // configure swashplate 2 and update scalars
    _swashplate2.configure();
    _swashplate2.calculate_roll_pitch_collective_factors();

    // set mode of main rotor controller and trigger recalculation of scalars
    _main_rotor.set_control_mode(static_cast<RotorControlMode>(_main_rotor._rsc_mode.get()));
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
                swash_tilt = 0.45f * _dcp_scaler * (roll_input + constrain_float(_dcp_trim, -0.2f, 0.2f)) + coll_input;
            } else if (swash_num == 2) {
                swash_tilt = -0.45f * _dcp_scaler * (roll_input + constrain_float(_dcp_trim, -0.2f, 0.2f)) + coll_input;
            }
        }
    } else if (_dual_mode == AP_MOTORS_HELI_DUAL_MODE_INTERMESHING) {
        // roll tilt
        if (swash_axis == AP_MOTORS_HELI_DUAL_SWASH_AXIS_ROLL) {
            if (swash_num == 1) {
                swash_tilt = roll_input;
            } else if (swash_num == 2) {
                swash_tilt = roll_input;
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
                swash_tilt = 0.45f * _dcp_scaler * yaw_input + coll_input;
            } else if (swash_num == 2) {
                swash_tilt = -0.45f * _dcp_scaler * yaw_input + coll_input;
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
                swash_tilt = 0.45f * _dcp_scaler * (pitch_input + constrain_float(_dcp_trim, -0.2f, 0.2f)) + coll_input;
            } else if (swash_num == 2) {
                swash_tilt = -0.45f * _dcp_scaler * (pitch_input + constrain_float(_dcp_trim, -0.2f, 0.2f)) + coll_input;
            }
        }
    }
    return swash_tilt;
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsHeli_Dual::get_motor_mask()
{
    // dual heli uses channels 1,2,3,4,5,6 and 8
    uint32_t mask = 0;
    for (uint8_t i=0; i<AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS; i++) {
        mask |= 1U << (AP_MOTORS_MOT_1+i);
    }
    if (_swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << AP_MOTORS_MOT_7;
    }
    if (_swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << AP_MOTORS_MOT_8;
    }
    mask |= 1U << AP_MOTORS_HELI_RSC;
    return mask;
}

// update_motor_controls - sends commands to motor controllers
void AP_MotorsHeli_Dual::update_motor_control(RotorControlState state)
{
    // Send state update to motors
    _main_rotor.output(state);

    if (state == ROTOR_CONTROL_STOP) {
        // set engine run enable aux output to not run position to kill engine when disarmed
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::MIN);
    } else {
        // else if armed, set engine run enable output to run position
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::MAX);
    }

    // Check if rotors are run-up
    _heliflags.rotor_runup_complete = _main_rotor.is_runup_complete();
    // Check if rotors are spooled down
    _heliflags.rotor_spooldown_complete = _main_rotor.is_spooldown_complete();
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
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    if (_dual_mode == AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE || _dual_mode == AP_MOTORS_HELI_DUAL_MODE_INTERMESHING) {
        if (pitch_out < -_cyclic_max/4500.0f) {
            pitch_out = -_cyclic_max/4500.0f;
            limit.pitch = true;
        }

        if (pitch_out > _cyclic_max/4500.0f) {
            pitch_out = _cyclic_max/4500.0f;
            limit.pitch = true;
        }
    } else {
        if (roll_out < -_cyclic_max/4500.0f) {
            roll_out = -_cyclic_max/4500.0f;
            limit.roll = true;
        }

        if (roll_out > _cyclic_max/4500.0f) {
            roll_out = _cyclic_max/4500.0f;
            limit.roll = true;
        }
    }

    if (_heliflags.inverted_flight) {
        collective_in = 1 - collective_in;
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
    if (_heliflags.landing_collective && collective_out < _collective_land_min_pct) {
        collective_out = _collective_land_min_pct;
        limit.throttle_lower = true;
    }

    // updates below land min collective flag
    if (collective_out <= _collective_land_min_pct) {
        _heliflags.below_land_min_coll = true;
    } else {
        _heliflags.below_land_min_coll = false;
    }

    // updates takeoff collective flag based on 50% hover collective
    update_takeoff_collective_flag(collective_out);

    // Set rear collective to midpoint if required
    float collective2_out = collective_out;
    if (_servo_mode == SERVO_CONTROL_MODE_MANUAL_CENTER) {
        collective2_out = _collective2_zero_thrst_pct;
    }

    // if servo output not in manual mode, process pre-compensation factors
    if (_servo_mode == SERVO_CONTROL_MODE_AUTOMATED) {
        // add differential collective pitch yaw compensation
        float yaw_compensation = 0.0f;

        if (_dual_mode == AP_MOTORS_HELI_DUAL_MODE_INTERMESHING) {
            // for intermeshing, reverse yaw in negative collective region and smoothen transition near zero collective
            if (_yaw_rev_expo > 0.01f) {
                // yaw_compensation range: (-1,1) S-shaped curve (Logistic Model) 1/(1 + e^kt)
                yaw_compensation = 1.0f - (2.0f / (1.0f + powf(2.7182818f , _yaw_rev_expo * (collective_out-_collective_zero_thrust_pct))));
                yaw_out = yaw_out * yaw_compensation;
            }
        } else {
            if (_dual_mode == AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE) {
                yaw_compensation = _dcp_yaw_effect * roll_out;
            } else { // AP_MOTORS_HELI_DUAL_MODE_TANDEM
                yaw_compensation = _dcp_yaw_effect * pitch_out;
            }
            yaw_out = yaw_out + yaw_compensation;
        }
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

    // scale collective pitch for front swashplate (servos 1,2,3)
    float collective_scaler = ((float)(_collective_max-_collective_min))*0.001f;
    float collective_out_scaled = collective_out * collective_scaler + (_collective_min - 1000)*0.001f;

    // scale collective pitch for rear swashplate (servos 4,5,6)
    float collective2_scaler = ((float)(_collective2_max-_collective2_min))*0.001f;
    float collective2_out_scaled = collective2_out * collective2_scaler + (_collective2_min - 1000)*0.001f;

    // feed power estimate into main rotor controller
    // ToDo: add main rotor cyclic power?
    _main_rotor.set_collective(fabsf(collective_out));

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
    if (_swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        _servo_out[CH_7] = _swashplate1.get_servo_out(CH_4,swash1_pitch,swash1_roll,swash1_coll);
    }

    // get servo positions from swashplate library
    _servo_out[CH_4] = _swashplate2.get_servo_out(CH_1,swash2_pitch,swash2_roll,swash2_coll);
    _servo_out[CH_5] = _swashplate2.get_servo_out(CH_2,swash2_pitch,swash2_roll,swash2_coll);
    _servo_out[CH_6] = _swashplate2.get_servo_out(CH_3,swash2_pitch,swash2_roll,swash2_coll);
    if (_swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        _servo_out[CH_8] = _swashplate2.get_servo_out(CH_4,swash2_pitch,swash2_roll,swash2_coll);
    }

}

void AP_MotorsHeli_Dual::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }
    // actually move the servos.  PWM is sent based on nominal 1500 center.  servo output shifts center based on trim value.
    for (uint8_t i=0; i<AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS; i++) {
        rc_write_swash(i, _servo_out[CH_1+i]);
    }

    // write to servo for 4 servo of 4 servo swashplate
    if (_swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate1.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        rc_write_swash(AP_MOTORS_MOT_7, _servo_out[CH_7]);
    }
    // write to servo for 4 servo of 4 servo swashplate
    if (_swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate2.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        rc_write_swash(AP_MOTORS_MOT_8, _servo_out[CH_8]);
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            update_motor_control(ROTOR_CONTROL_STOP);
            break;
        case SpoolState::GROUND_IDLE:
            // sends idle output to motors when armed. rotor could be static or turning (autorotation)
            update_motor_control(ROTOR_CONTROL_IDLE);
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
            // set motor output based on thrust requests
            update_motor_control(ROTOR_CONTROL_ACTIVE);
            break;
        case SpoolState::SPOOLING_DOWN:
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
    if (_swashplate1.get_swash_type() == SWASHPLATE_TYPE_H3 && (_swashplate1.get_phase_angle() > 30 || _swashplate1.get_phase_angle() < -30)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_SW1_H3_PHANG out of range");
        }
        return false;
    }

    // returns false if Phase Angle is outside of range for H3 swashplate 2
    if (_swashplate2.get_swash_type() == SWASHPLATE_TYPE_H3 && (_swashplate2.get_phase_angle() > 30 || _swashplate2.get_phase_angle() < -30)){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_SW2_H3_PHANG out of range");
        }
        return false;
    }

    // check parent class parameters
    return AP_MotorsHeli::parameter_check(display_msg);
}
