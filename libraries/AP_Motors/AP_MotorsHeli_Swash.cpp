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

#include "AP_MotorsHeli_Swash.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Swash::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Swashplate Type
    // @Description: H3 is generic, three-servo only. H3_120/H3_140 plates have Motor1 left side, Motor2 right side, Motor3 elevator in rear. HR3_120/HR3_140 have Motor1 right side, Motor2 left side, Motor3 elevator in front - use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 Motors1&2 are left/right respectively, Motors3&4 are rear/front respectively. For H4-45 Motors1&2 are LF/RF, Motors3&4 are LR/RR 
    // @Values: 0:H3 Generic, 1:H1 non-CPPM, 2:H3_140, 3:H3_120, 4:H4_90, 5:H4_45
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_MotorsHeli_Swash, _swashplate_type, SWASHPLATE_TYPE_H3),

    // @Param: COL_DIR
    // @DisplayName: Collective Control Direction
    // @Description: Direction collective moves for positive pitch. 0 for Normal, 1 for Reversed
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    AP_GROUPINFO("COL_DIR", 2, AP_MotorsHeli_Swash, _swash_coll_dir, COLLECTIVE_DIRECTION_NORMAL),

    // @Param: LIN_SVO
    // @DisplayName: Linearize swashplate servo mechanical throw
    // @Description: This linearizes the swashplate servo's mechanical output to account for nonlinear output due to arm rotation.  This requires a specific setup procedure to work properly.  The servo arm must be centered on the mechanical throw at the servo trim position and the servo trim position kept as close to 1500 as possible. Leveling the swashplate can only be done through the pitch links.  See the ardupilot wiki for more details on setup.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LIN_SVO", 3, AP_MotorsHeli_Swash, _linear_swash_servo, 0),

    // @Param: H3_ENABLE
    // @DisplayName: Enable generic H3 swashplate settings
    // @Description: Automatically set when H3 generic swash type is selected. Do not set manually.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("H3_ENABLE", 4, AP_MotorsHeli_Swash, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: H3_SV1_POS
    // @DisplayName: servo 1 position
    // @Description: Azimuth position on swashplate for servo 1 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H3_SV1_POS", 5, AP_MotorsHeli_Swash, _servo1_pos, -60),

    // @Param: H3_SV2_POS
    // @DisplayName: servo 2 position
    // @Description: Azimuth position on swashplate for servo 2 with the front of the heli being 0 deg
    // @Range: -180 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("H3_SV2_POS", 6, AP_MotorsHeli_Swash, _servo2_pos, 60),

    // @Param: H3_SV3_POS
    // @DisplayName: servo 3 position
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

// configure - configure the swashplate settings for any updated parameters
void AP_MotorsHeli_Swash::configure()
{

    _swash_type = static_cast<SwashPlateType>(_swashplate_type.get());
    _collective_direction = static_cast<CollectiveDirection>(_swash_coll_dir.get());
    _make_servo_linear = _linear_swash_servo;
    if (_swash_type == SWASHPLATE_TYPE_H3) {
        enable = 1;
    } else {
        enable = 0;
    }
}

// CCPM Mixers - calculate mixing scale factors by swashplate type
void AP_MotorsHeli_Swash::calculate_roll_pitch_collective_factors()
{
    if (_swash_type == SWASHPLATE_TYPE_H1) {
        // CCPM mixing not used
        _collectiveFactor[CH_1] = 0;
        _collectiveFactor[CH_2] = 0;
        _collectiveFactor[CH_3] = 1;
    } else if ((_swash_type == SWASHPLATE_TYPE_H4_90) || (_swash_type == SWASHPLATE_TYPE_H4_45)) {
        // collective mixer for four-servo CCPM
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;
        _collectiveFactor[CH_4] = 1;
    } else {
        // collective mixer for three-servo CCPM
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;
    }

    if (_swash_type == SWASHPLATE_TYPE_H3) {
        // Three-servo roll/pitch mixer for adjustable servo position
        // can be any style swashplate, phase angle is adjustable
        _rollFactor[CH_1] = cosf(radians(_servo1_pos + 90 - _phase_angle));
        _rollFactor[CH_2] = cosf(radians(_servo2_pos + 90 - _phase_angle));
        _rollFactor[CH_3] = cosf(radians(_servo3_pos + 90 - _phase_angle));
        _pitchFactor[CH_1] = cosf(radians(_servo1_pos - _phase_angle));
        _pitchFactor[CH_2] = cosf(radians(_servo2_pos - _phase_angle));
        _pitchFactor[CH_3] = cosf(radians(_servo3_pos - _phase_angle));
        
        // defined swashplates, servo1 is always left, servo2 is right,
        // servo3 is elevator
    } else if (_swash_type == SWASHPLATE_TYPE_H3_140) {    //
        // Three-servo roll/pitch mixer for H3-140
        // HR3-140 uses reversed servo and collective direction in heli setup
        // 1:1 pure input style, phase angle not adjustable
        _rollFactor[CH_1] = 1;
        _rollFactor[CH_2] = -1;
        _rollFactor[CH_3] = 0;
        _pitchFactor[CH_1] = 1;
        _pitchFactor[CH_2] = 1;
        _pitchFactor[CH_3] = -1;
    } else if (_swash_type == SWASHPLATE_TYPE_H3_120) {
        // three-servo roll/pitch mixer for H3-120
        // HR3-120 uses reversed servo and collective direction in heli setup
        // not a pure mixing swashplate, phase angle is adjustable
        _rollFactor[CH_1] = 0.866025f;
        _rollFactor[CH_2] = -0.866025f;
        _rollFactor[CH_3] = 0;
        _pitchFactor[CH_1] = 0.5f;
        _pitchFactor[CH_2] = 0.5f;
        _pitchFactor[CH_3] = -1;
    } else if (_swash_type == SWASHPLATE_TYPE_H4_90) {
        // four-servo roll/pitch mixer for H4-90
        // 1:1 pure input style, phase angle not adjustable
        // servos 3 & 7 are elevator
        // can also be used for all versions of 90 deg three-servo swashplates
        _rollFactor[CH_1] = 1; 
        _rollFactor[CH_2] = -1;
        _rollFactor[CH_3] = 0;
        _rollFactor[CH_4] = 0;
        _pitchFactor[CH_1] = 0;
        _pitchFactor[CH_2] = 0;
        _pitchFactor[CH_3] = -1;
        _pitchFactor[CH_4] = 1;
    } else if (_swash_type == SWASHPLATE_TYPE_H4_45) {
        // four-servo roll/pitch mixer for H4-45
        // 1:1 pure input style, phase angle not adjustable
        // for 45 deg plates servos 1&2 are LF&RF, 3&7 are LR&RR.
        _rollFactor[CH_1] = 0.707107f; 
        _rollFactor[CH_2] = -0.707107f;
        _rollFactor[CH_3] = 0.707107f;
        _rollFactor[CH_4] = -0.707107f;
        _pitchFactor[CH_1] = 0.707107f;
        _pitchFactor[CH_2] = 0.707107f;
        _pitchFactor[CH_3] = -0.707f;
        _pitchFactor[CH_4] = -0.707f;
    } else {
        // CCPM mixing not being used, so H1 straight outputs
        _rollFactor[CH_1] = 1;
        _rollFactor[CH_2] = 0;
        _rollFactor[CH_3] = 0;
        _pitchFactor[CH_1] = 0;
        _pitchFactor[CH_2] = 1;
        _pitchFactor[CH_3] = 0;
    }
}

// get_servo_out - calculates servo output
float AP_MotorsHeli_Swash::get_servo_out(int8_t ch_num, float pitch, float roll, float collective) const
{
    // Collective control direction. Swash moves up for negative collective pitch, down for positive collective pitch
    if (_collective_direction == COLLECTIVE_DIRECTION_REVERSED){
        collective = 1 - collective;
    }

    float servo = ((_rollFactor[ch_num] * roll) + (_pitchFactor[ch_num] * pitch))*0.45f + _collectiveFactor[ch_num] * collective;
    if (_swash_type == SWASHPLATE_TYPE_H1 && (ch_num == CH_1 || ch_num == CH_2)) {
        servo += 0.5f;
    }

    // rescale from -1..1, so we can use the pwm calc that includes trim
    servo = 2.0f * servo - 1.0f;

    if (_make_servo_linear == 1) {
        servo = get_linear_servo_output(servo);
    }

    return servo;
}

// set_linear_servo_out - sets swashplate servo output to be linear
float AP_MotorsHeli_Swash::get_linear_servo_output(float input) const
{

    input = constrain_float(input, -1.0f, 1.0f);

    //servo output is calculated by normalizing input to 50 deg arm rotation as full input for a linear throw
    return safe_asin(0.766044f * input) * 1.145916;

}

