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
                             uint8_t motor_offset,
                             Vector3f &torque,
                             Vector3f &thrust,
                             const Vector3f &velocity_air_bf,
                             const Vector3f &gyro,
                             float air_density,
                             float voltage,
                             bool use_drag)
{

    const float pwm = input.servos[motor_offset+servo];
    float command = pwm_to_command(pwm);
    float voltage_scale = voltage / voltage_max;

    if (voltage_scale < 0.1) {
        // battery is dead
        torque.zero();
        thrust.zero();
        current = 0;
        return;
    }

    // apply slew limiter to command
    uint64_t now_us = AP_HAL::micros64();
    if (last_calc_us != 0 && slew_max > 0) {
        float dt = (now_us - last_calc_us)*1.0e-6;
        float slew_max_change = slew_max * dt;
        command = constrain_float(command, last_command-slew_max_change, last_command+slew_max_change);
    }
    last_calc_us = now_us;
    last_command = command;

    // velocity of motor through air
    Vector3f motor_vel = velocity_air_bf;

    // add velocity of motor about center due to vehicle rotation
    motor_vel += -(position % gyro);

    // calculate velocity into prop, clipping at zero
    float velocity_in = MAX(0, -motor_vel.projected(thrust_vector).z);

    // get thrust for untilted motor
    float motor_thrust = calc_thrust(command, air_density, velocity_in, voltage_scale);

    // the yaw torque of the motor
    const float yaw_scale = 0.05 * diagonal_size * motor_thrust;
    Vector3f rotor_torque = thrust_vector * yaw_factor * command * yaw_scale * -1.0;

    // thrust in bodyframe NED
    thrust = thrust_vector * motor_thrust;

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

    // calculate torque in newton-meters
    torque = (position % thrust) + rotor_torque;

    // possibly rotate the thrust vector and the rotor torque
    if (!is_zero(roll) || !is_zero(pitch)) {
        Matrix3f rotation;
        rotation.from_euler(radians(roll), radians(pitch), 0);
        thrust = rotation * thrust;
        torque = rotation * torque;
    }

    if (use_drag) {
        // calculate momentum drag per motor
        const float momentum_drag_factor = momentum_drag_coefficient * sqrtf(air_density * true_prop_area);
        Vector3f momentum_drag;
        momentum_drag.x = momentum_drag_factor * motor_vel.x * (sqrtf(fabsf(thrust.y)) + sqrtf(fabsf(thrust.z)));
        momentum_drag.y = momentum_drag_factor * motor_vel.y * (sqrtf(fabsf(thrust.x)) + sqrtf(fabsf(thrust.z)));
        // The application of momentum drag to the Z axis is a 'hack' to compensate for incorrect modelling
        // of the variation of thust with inflow velocity. If not applied, the vehicle will
        // climb at an unrealistic rate during operation in STABILIZE. TODO replace prop and motor model in
        // with one based on DC motor, momentum disc and blade element theory.
        momentum_drag.z = momentum_drag_factor * motor_vel.z * (sqrtf(fabsf(thrust.x)) + sqrtf(fabsf(thrust.y)) + sqrtf(fabsf(thrust.z)));

        thrust -= momentum_drag;
    }

    // calculate current
    float power = power_factor * fabsf(motor_thrust);
    current = power / MAX(voltage, 0.1);
}

/*
  update and return current value of a servo. Calculated as 1000..2000
 */
uint16_t Motor::update_servo(uint16_t demand, uint64_t time_usec, float &last_value) const
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
float Motor::get_current(void) const
{
    return current;
}

// setup PWM ranges for this motor
void Motor::setup_params(uint16_t _pwm_min, uint16_t _pwm_max, float _spin_min, float _spin_max, float _expo, float _slew_max,
                         float _diagonal_size, float _power_factor, float _voltage_max, float _effective_prop_area,
                         float _velocity_max, Vector3f _position, Vector3f _thrust_vector, float _yaw_factor, 
                         float _true_prop_area, float _momentum_drag_coefficient)
{
    mot_pwm_min = _pwm_min;
    mot_pwm_max = _pwm_max;
    mot_spin_min = _spin_min;
    mot_spin_max = _spin_max;
    mot_expo = _expo;
    slew_max = _slew_max;
    power_factor = _power_factor;
    voltage_max = _voltage_max;
    effective_prop_area = _effective_prop_area;
    max_outflow_velocity = _velocity_max;
    true_prop_area = _true_prop_area;
    momentum_drag_coefficient = _momentum_drag_coefficient;
    diagonal_size = _diagonal_size;

    if (!_position.is_zero()) {
        position = _position;
    } else {
        position.x = cosf(radians(angle)) * _diagonal_size;
        position.y =  sinf(radians(angle)) * _diagonal_size;
        position.z = 0;
    }

    if (!_thrust_vector.is_zero()) {
        thrust_vector = _thrust_vector;
    }
    if (!is_zero(_yaw_factor)) {
        yaw_factor = _yaw_factor;
    }
}

/*
  convert a PWM value to a command value from 0 to 1
*/
float Motor::pwm_to_command(float pwm) const
{
    const float pwm_thrust_max = mot_pwm_min + mot_spin_max * (mot_pwm_max - mot_pwm_min);
    const float pwm_thrust_min = mot_pwm_min + mot_spin_min * (mot_pwm_max - mot_pwm_min);
    const float pwm_thrust_range = pwm_thrust_max - pwm_thrust_min;
    return constrain_float((pwm-pwm_thrust_min)/pwm_thrust_range, 0, 1);
}

/*
  calculate thrust given a command value
*/
float Motor::calc_thrust(float command, float air_density, float velocity_in, float voltage_scale) const
{
    float velocity_out = voltage_scale * max_outflow_velocity * sqrtf((1-mot_expo)*command + mot_expo*sq(command));
    float ret = 0.5 * air_density * effective_prop_area * (sq(velocity_out) - sq(velocity_in));
#if 0
    if (command > 0) {
        ::printf("air_density=%f effective_prop_area=%f velocity_in=%f velocity_max=%f\n",
                 air_density, effective_prop_area, velocity_in, voltage_scale * max_outflow_velocity);
        ::printf("calc_thrust %.3f %.3f\n", command, ret);
    }
#endif
    return ret;
}
