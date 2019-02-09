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

const AP_Param::GroupInfo SwashInt16Param::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for H3
    // @Description: Automatically set when H3 swash type is selected. Should not be set manually.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, SwashInt16Param, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SV1_POS
    // @DisplayName: servo1_pos
    // @Description: servo 1 position
    // @Range: -180 to 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("SV1_POS", 2, SwashInt16Param, servo1_pos, -60),

    // @Param: SV2_POS
    // @DisplayName: servo2_pos
    // @Description: servo 2 position
    // @Range: -180 to 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("SV2_POS", 3, SwashInt16Param, servo2_pos, 60),

    // @Param: SV3_POS
    // @DisplayName: servo3_pos
    // @Description: servo 3 position
    // @Range: -180 to 180
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("SV3_POS", 4, SwashInt16Param, servo3_pos, 180),
    
    // @Param: PHANG
    // @DisplayName: Swashplate Phase Angle Compensation
    // @Description: Only for H3 swashplate.  If pitching the swash forward induces a roll, this can be correct the problem
    // @Range: -30 30
    // @Units: deg
    // @User: Advanced
    // @Increment: 1
    AP_GROUPINFO("PHANG", 5, SwashInt16Param, phase_angle, 0),
   
    AP_GROUPEND
};

/*
  a manual swashplate definition with enable and servo position parameters - constructor
 */
SwashInt16Param::SwashInt16Param(void)
{
    AP_Param::setup_object_defaults(this, var_info);    
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
float AP_MotorsHeli_Swash::get_servo_out(int8_t CH_num, float pitch, float roll, float collective)
{
    // Collective control direction. Swash moves up for negative collective pitch, down for positive collective pitch
    if (_collective_direction == COLLECTIVE_DIRECTION_REVERSED){
        collective = 1 - collective;
    }

    float servo = ((_rollFactor[CH_num] * roll) + (_pitchFactor[CH_num] * pitch))*0.45f + _collectiveFactor[CH_num] * collective;
    if (_swash_type == SWASHPLATE_TYPE_H1 && (CH_num == CH_1 || CH_num == CH_2)) {
        servo += 0.5f;
    }

    // rescale from -1..1, so we can use the pwm calc that includes trim
    servo = 2.0f * servo - 1.0f;

    if (_make_servo_linear == 1) {
        servo = get_linear_servo_output(servo);
    }

    return servo;
}

float AP_MotorsHeli_Swash::get_linear_servo_output(float input)
{
    float ret;

    input = constrain_float(input, -1.0f, 1.0f);

    //servo output is normalized to 0.866 for a linear throw
    ret = asin(0.766044f * input) * 1.145916;

    return ret;

}

