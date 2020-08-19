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
  simple electric motor simulator class
*/

#include "SIM_Motor.h"
#include <AP_Motors/AP_Motors.h>

using namespace SITL;

// calculate rotational accel and thrust for a motor
void Motor::calculate_forces(const struct sitl_input &input,
                             const float thrust_scale,
                             uint8_t motor_offset,
                             Vector3f &rot_accel,
                             Vector3f &thrust)
{
    // fudge factors
    const float arm_scale = radians(5000);
    const float yaw_scale = radians(400);

    // get motor speed from 0 to 1
    float motor_speed = constrain_float((input.servos[motor_offset+servo]-1100)/900.0, 0, 1);

    // the yaw torque of the motor
    Vector3f rotor_torque(0, 0, yaw_factor * motor_speed * yaw_scale);

    // get thrust for untilted motor
    thrust = {0, 0, -motor_speed};

    // define the arm position relative to center of mass
    Vector3f arm(arm_scale * cosf(radians(angle)), arm_scale * sinf(radians(angle)), 0);

    // work out roll and pitch of motor relative to it pointing straight up
    float roll = 0, pitch = 0;

    uint64_t now = AP_HAL::micros64();
    
    // possibly roll and/or pitch the motor
    if (roll_servo >= 0) {
        uint16_t servoval = update_servo(input.servos[roll_servo+motor_offset], now, last_roll_value);
        if (roll_min < roll_max) {
            roll = constrain_float(roll_min + (servoval-1000)*0.001*(roll_max-roll_min), roll_min, roll_max);
        } else {
            roll = constrain_float(roll_max + (2000-servoval)*0.001*(roll_min-roll_max), roll_max, roll_min);
        }
    }
    if (pitch_servo >= 0) {
        uint16_t servoval = update_servo(input.servos[pitch_servo+motor_offset], now, last_pitch_value);
        if (pitch_min < pitch_max) {
            pitch = constrain_float(pitch_min + (servoval-1000)*0.001*(pitch_max-pitch_min), pitch_min, pitch_max);
        } else {
            pitch = constrain_float(pitch_max + (2000-servoval)*0.001*(pitch_min-pitch_max), pitch_max, pitch_min);
        }
    }
    last_change_usec = now;

    // possibly rotate the thrust vector and the rotor torque
    if (!is_zero(roll) || !is_zero(pitch)) {
        Matrix3f rotation;
        rotation.from_euler(radians(roll), radians(pitch), 0);
        thrust = rotation * thrust;
        rotor_torque = rotation * rotor_torque;
    }

    // calculate total rotational acceleration
    rot_accel = (arm % thrust) + rotor_torque;

    // scale the thrust
    thrust = thrust * thrust_scale;
}

/*
  update and return current value of a servo. Calculated as 1000..2000
 */
uint16_t Motor::update_servo(uint16_t demand, uint64_t time_usec, float &last_value)
{
    if (servo_rate <= 0) {
        return demand;
    }
    if (servo_type == SERVO_RETRACT) {
        // handle retract servos
        if (demand > 1700) {
            demand = 2000;
        } else if (demand < 1300) {
            demand = 1000;
        } else {
            demand = last_value;
        }
    }
    demand = constrain_int16(demand, 1000, 2000);
    float dt = (time_usec - last_change_usec) * 1.0e-6f;
    // assume servo moves through 90 degrees over 1000 to 2000
    float max_change = 1000 * (dt / servo_rate) * 60.0f / 90.0f;
    last_value = constrain_float(demand, last_value-max_change, last_value+max_change);
    return uint16_t(last_value+0.5);
}


// calculate current and voltage
void Motor::current_and_voltage(const struct sitl_input &input, float &voltage, float &current,
                                uint8_t motor_offset)
{
    // get motor speed from 0 to 1
    float motor_speed = constrain_float((input.servos[motor_offset+servo]-1100)/900.0, 0, 1);

    // assume 10A per motor at full speed
    current = 10 * motor_speed;

    // assume 3S, and full throttle drops voltage by 0.7V
    if (AP::sitl()) {
        voltage = AP::sitl()->batt_voltage - motor_speed * 0.7;
    }
}
