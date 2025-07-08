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
  simple electric motor simulation class
*/

#pragma once

#include <AP_Math/AP_Math.h>
#include <SITL/SITL_Input.h>

namespace SITL {

/*
  class to describe a motor position
 */
class Motor {
public:
    float angle;
    float yaw_factor;
    uint8_t servo;
    uint8_t display_order;

    // support for tilting motors
    int8_t roll_servo = -1;
    float roll_min, roll_max;
    int8_t pitch_servo = -1;
    float pitch_min, pitch_max;

    // support for servo slew rate
    enum {SERVO_NORMAL, SERVO_RETRACT} servo_type;
    float servo_rate = 0.24; // seconds per 60 degrees
    uint64_t last_change_usec;
    float last_roll_value, last_pitch_value;

    Motor(uint8_t _servo, float _angle, float _yaw_factor, uint8_t _display_order) :
        angle(_angle), // angle in degrees from front
        yaw_factor(_yaw_factor), // positive is clockwise
        servo(_servo), // what servo output drives this motor
        display_order(_display_order) // order for clockwise display
    {
        position.x = cosf(radians(angle));
        position.y =  sinf(radians(angle));
        position.z = 0;

        thrust_vector.x = 0;
        thrust_vector.y = 0;
        thrust_vector.z = -1;
    }

    /*
      alternative constructor for tiltable motors
     */
    Motor(uint8_t _servo, float _angle, float _yaw_factor, uint8_t _display_order,
          int8_t _roll_servo, float _roll_min, float _roll_max,
          int8_t _pitch_servo, float _pitch_min, float _pitch_max) :
        angle(_angle), // angle in degrees from front
        yaw_factor(_yaw_factor), // positive is clockwise
        servo(_servo), // what servo output drives this motor
        display_order(_display_order), // order for clockwise display
        roll_servo(_roll_servo),
        roll_min(_roll_min),
        roll_max(_roll_max),
        pitch_servo(_pitch_servo),
        pitch_min(_pitch_min),
        pitch_max(_pitch_max)
    {
        position.x = cosf(radians(angle));
        position.y =  sinf(radians(angle));
        position.z = 0;

        thrust_vector.x = 0;
        thrust_vector.y = 0;
        thrust_vector.z = -1;
    }

    void calculate_forces(const struct sitl_input &input,
                          uint8_t motor_offset,
                          Vector3f &torque, // Newton meters
                          Vector3f &thrust, // Z is down, Newtons
                          const Vector3f &velocity_air_bf,
                          const Vector3f &gyro, // rad/sec
                          float air_density,
                          float voltage,
                          bool use_drag);

    uint16_t update_servo(uint16_t demand, uint64_t time_usec, float &last_value) const;

    // get current
    float get_current(void) const;

    // convert a PWM value to a thrust demand from 0 to 1
    float pwm_to_command(float pwm) const;

    // setup motor key parameters
    void setup_params(uint16_t _pwm_min, uint16_t _pwm_max, float _spin_min, float _spin_max, float _expo, float _slew_max,
                      float _diagonal_size, float _power_factor, float _voltage_max, float _effective_prop_area,
                      float _velocity_max, Vector3f _position, Vector3f _thrust_vector, float _yaw_factor,
                      float _true_prop_area, float _momentum_drag_coefficient);

    // override slew limit
    void set_slew_max(float _slew_max) {
        slew_max = _slew_max;
    }

    float get_command(void) const {
        return last_command;
    }

    // calculate thrust of motor
    float calc_thrust(float command, float air_density, float velocity_in, float voltage_scale) const;

private:
    float mot_pwm_min;
    float mot_pwm_max;
    float mot_spin_min;
    float mot_spin_max;
    float mot_expo;
    float slew_max;
    float current;
    float power_factor;
    float voltage_max;
    float effective_prop_area;
    float max_outflow_velocity;
    float true_prop_area;
    float momentum_drag_coefficient;
    float diagonal_size;

    float last_command;
    uint64_t last_calc_us;

    Vector3f position;
    Vector3f thrust_vector;
};

}
