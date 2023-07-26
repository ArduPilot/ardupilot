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
#include <SRV_Channel/SRV_Channel.h>

#include "AP_MotorsHeli_Swash.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Swash::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Swashplate Type
    // @Description: H3 is generic, three-servo only. H3_120/H3_140 plates have Motor1 left side, Motor2 right side, Motor3 elevator in rear. HR3_120/HR3_140 have Motor1 right side, Motor2 left side, Motor3 elevator in front - use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 Motors1&2 are left/right respectively, Motors3&4 are rear/front respectively. For H4-45 Motors1&2 are LF/RF, Motors3&4 are LR/RR 
    // @Values: 0:H3 Generic,1:H1 non-CPPM,2:H3_140,3:H3_120,4:H4_90,5:H4_45
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_MotorsHeli_Swash, _swashplate_type, SWASHPLATE_TYPE_H3_120),

    // @Param: COL_DIR
    // @DisplayName: Swashplate Collective Control Direction
    // @Description: Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    AP_GROUPINFO("COL_DIR", 2, AP_MotorsHeli_Swash, _swash_coll_dir, COLLECTIVE_DIRECTION_NORMAL),

    // @Param: LIN_SVO
    // @DisplayName: Linearize Swashplate Servo Mechanical Throw
    // @Description: This linearizes the swashplate servo's mechanical output to account for nonlinear output due to arm rotation.  This requires a specific setup procedure to work properly.  The servo arm must be centered on the mechanical throw at the servo trim position and the servo trim position kept as close to 1500 as possible. Leveling the swashplate can only be done through the pitch links.  See the ardupilot wiki for more details on setup.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LIN_SVO", 3, AP_MotorsHeli_Swash, _linear_swash_servo, 0),

    // @Param: H3_ENABLE
    // @DisplayName: Enable Generic H3 Swashplate Settings
    // @Description: Automatically set when H3 generic swash type is selected. Do not set manually.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("H3_ENABLE", 4, AP_MotorsHeli_Swash, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: H3_SV1_POS
    // @DisplayName: Swashplate Servo 1 Position
    // @Description: Azimuth position on swashplate for servo 1 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H3_SV1_POS", 5, AP_MotorsHeli_Swash, _servo1_pos, -60),

    // @Param: H3_SV2_POS
    // @DisplayName: Swashplate Servo 2 Position
    // @Description: Azimuth position on swashplate for servo 2 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H3_SV2_POS", 6, AP_MotorsHeli_Swash, _servo2_pos, 60),

    // @Param: H3_SV3_POS
    // @DisplayName: Swashplate Servo 3 Position
    // @Description: Azimuth position on swashplate for servo 3 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H3_SV3_POS", 7, AP_MotorsHeli_Swash, _servo3_pos, 180),
    
    // @Param: H3_PHANG
    // @DisplayName: Swashplate Phase Angle Compensation
    // @Description: Only for H3 swashplate.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -30 30
    // @Units: deg
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("H3_PHANG", 8, AP_MotorsHeli_Swash, _phase_angle, 0),
   
    AP_GROUPEND
};

AP_MotorsHeli_Swash::AP_MotorsHeli_Swash(uint8_t mot_0, uint8_t mot_1, uint8_t mot_2, uint8_t mot_3)
{
    _motor_num[0] = mot_0;
    _motor_num[1] = mot_1;
    _motor_num[2] = mot_2;
    _motor_num[3] = mot_3;

    AP_Param::setup_object_defaults(this, var_info);
}

// configure - configure the swashplate settings for any updated parameters
void AP_MotorsHeli_Swash::configure()
{

    _swash_type = static_cast<SwashPlateType>(_swashplate_type.get());
    _collective_direction = static_cast<CollectiveDirection>(_swash_coll_dir.get());
    _make_servo_linear = _linear_swash_servo != 0;
    enable.set(_swash_type == SWASHPLATE_TYPE_H3);

    calculate_roll_pitch_collective_factors();
}

// CCPM Mixers - calculate mixing scale factors by swashplate type
void AP_MotorsHeli_Swash::calculate_roll_pitch_collective_factors()
{
    // Clear existing setup
    for (uint8_t i = 0; i < _max_num_servos; i++) {
        _enabled[i] = false;
        _rollFactor[i] = 0.0;
        _pitchFactor[i] = 0.0;
        _collectiveFactor[i] = 0.0;
    }

    switch (_swash_type) {
        case SWASHPLATE_TYPE_H3:
            // Three-servo roll/pitch mixer for adjustable servo position
            // can be any style swashplate, phase angle is adjustable
            add_servo_angle(CH_1, _servo1_pos - _phase_angle, 1.0);
            add_servo_angle(CH_2, _servo2_pos - _phase_angle, 1.0);
            add_servo_angle(CH_3, _servo3_pos - _phase_angle, 1.0);
            break;

        case SWASHPLATE_TYPE_H1:
            // CCPM mixing not being used, so H1 straight outputs
            add_servo_raw(CH_1, 1.0, 0.0, 0.0);
            add_servo_raw(CH_2, 0.0, 1.0, 0.0);
            add_servo_raw(CH_3, 0.0, 0.0, 1.0);
            break;


        case SWASHPLATE_TYPE_H3_140:
            // Three-servo roll/pitch mixer for H3-140
            // HR3-140 uses reversed servo and collective direction in heli setup
            // 1:1 pure input style, phase angle not adjustable
            add_servo_raw(CH_1,  1.0,  1.0, 1.0);
            add_servo_raw(CH_2, -1.0,  1.0, 1.0);
            add_servo_raw(CH_3,  0.0, -1.0, 1.0);
            break;

        case SWASHPLATE_TYPE_H3_120:
            // three-servo roll/pitch mixer for H3-120
            // HR3-120 uses reversed servo and collective direction in heli setup
            // not a pure mixing swashplate, phase angle is adjustable
            add_servo_angle(CH_1, -60.0, 1.0);
            add_servo_angle(CH_2,  60.0, 1.0);
            add_servo_angle(CH_3, 180.0, 1.0);
            break;

        case SWASHPLATE_TYPE_H4_90:
            // four-servo roll/pitch mixer for H4-90
            // 1:1 pure input style, phase angle not adjustable
            // servos 3 & 7 are elevator
            // can also be used for all versions of 90 deg three-servo swashplates
            add_servo_angle(CH_1, -90.0, 1.0);
            add_servo_angle(CH_2,  90.0, 1.0);
            add_servo_angle(CH_3, 180.0, 1.0);
            add_servo_angle(CH_4,   0.0, 1.0);
            break;

        case SWASHPLATE_TYPE_H4_45:
            // four-servo roll/pitch mixer for H4-45
            // 1:1 pure input style, phase angle not adjustable
            // for 45 deg plates servos 1&2 are LF&RF, 3&7 are LR&RR.
            add_servo_angle(CH_1,  -45.0, 1.0);
            add_servo_angle(CH_2,   45.0, 1.0);
            add_servo_angle(CH_3, -135.0, 1.0);
            add_servo_angle(CH_4,  135.0, 1.0);
            break;
    }

}

void AP_MotorsHeli_Swash::add_servo_angle(uint8_t num, float angle, float collective)
{
    add_servo_raw(num,
                  cosf(radians(angle + 90)),
                  cosf(radians(angle)),
                  collective);
}

void AP_MotorsHeli_Swash::add_servo_raw(uint8_t num, float roll, float pitch, float collective)
{
    if (num >= _max_num_servos) {
        // Indexing problem should never happen
        return;
    }

    _enabled[num] = true;
    _rollFactor[num] = roll * 0.45;
    _pitchFactor[num] = pitch * 0.45;
    _collectiveFactor[num] = collective;

    // Setup output function
    SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(_motor_num[num]);
    SRV_Channels::set_aux_channel_default(function, _motor_num[num]);

    // outputs are defined on a -500 to 500 range for swash servos
    SRV_Channels::set_range(function, 1000);

    // swash servos always use full endpoints as restricting them would lead to scaling errors
    SRV_Channels::set_output_min_max(function, 1000, 2000);

}

// calculates servo output
void AP_MotorsHeli_Swash::calculate(float roll, float pitch, float collective)
{
    // Collective control direction. Swash moves up for negative collective pitch, down for positive collective pitch
    if (_collective_direction == COLLECTIVE_DIRECTION_REVERSED){
        collective = 1 - collective;
    }

    for (uint8_t i = 0; i < _max_num_servos; i++) {
        if (!_enabled[i]) {
            // This servo is not enabled
            continue;
        }

        _output[i] = (_rollFactor[i] * roll) + (_pitchFactor[i] * pitch) + _collectiveFactor[i] * collective;
        if (_swash_type == SWASHPLATE_TYPE_H1 && (i == CH_1 || i == CH_2)) {
            _output[i] += 0.5f;
        }

        // rescale from -1..1, so we can use the pwm calc that includes trim
        _output[i] = 2.0f * _output[i] - 1.0f;

        if (_make_servo_linear) {
            _output[i] = get_linear_servo_output(_output[i]);
        }

    }
}

// set_linear_servo_out - sets swashplate servo output to be linear
float AP_MotorsHeli_Swash::get_linear_servo_output(float input) const
{

    input = constrain_float(input, -1.0f, 1.0f);

    //servo output is calculated by normalizing input to 50 deg arm rotation as full input for a linear throw
    return safe_asin(0.766044f * input) * 1.145916;

}

// Output calculated values to servos
void AP_MotorsHeli_Swash::output()
{
    for (uint8_t i = 0; i < _max_num_servos; i++) {
        if (_enabled[i]) {
            rc_write(_motor_num[i], _output[i]);
        }
    }
}

// convert input in -1 to +1 range to pwm output for swashplate servo.
// The value 0 corresponds to the trim value of the servo. Swashplate
// servo travel range is fixed to 1000 pwm and therefore the input is
// multiplied by 500 to get PWM output.
void AP_MotorsHeli_Swash::rc_write(uint8_t chan, float swash_in)
{
    uint16_t pwm = (uint16_t)(1500 + 500 * swash_in);
    SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(chan);
    SRV_Channels::set_output_pwm_trimmed(function, pwm);
}

// Get function output mask
uint32_t AP_MotorsHeli_Swash::get_output_mask() const
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < _max_num_servos; i++) {
        if (_enabled[i]) {
            mask |= 1U < _motor_num[i];
        }
    }
    return mask;
}
