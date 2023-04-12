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
  helicopter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a helicopter simulator
 */
class Helicopter : public Aircraft {
public:
    Helicopter(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Helicopter(frame_str);
    }

protected:

    void update_rotor_dynamics(Vector3f gyros, Vector2f ctrl_pos, Vector2f &tpp_angle, float dt);
    float update_rpm(float curr_rpm, float throttle, float &engine_torque, float collective, float dt);

    // buffers to provide time delay
    struct servos_stored {
        uint16_t servo1;
        uint16_t servo2;
        uint16_t servo3;
        uint16_t servo4;
        uint16_t servo5;
        uint16_t servo6;
    };
    uint16_t _servos_delayed[6];
    ObjectBuffer<servos_stored> *servos_stored_buffer;
    void push_to_buffer(const uint16_t servos_input[16]);
    void pull_from_buffer(uint16_t servos_delayed[6]);


private:
    float terminal_rotation_rate = 4*radians(360.0f);
    float hover_throttle = 0.5f;
    float terminal_velocity = 80;
    float hover_lean = 3.2f;
    float rotor_rot_accel = radians(20);
    float roll_rate_max = radians(1400);
    float pitch_rate_max = radians(1400);
    float yaw_rate_max = radians(1400);
    float rsc_setpoint = 0.8f;
    float izz = 0.2f;
    float iyy;
    float tr_dist = 0.85f;
    float cyclic_scalar = 7.2; // converts swashplate servo ouputs to cyclic blade pitch
    float thrust_scale;
    Vector2f _tpp_angle;
    Vector2f _tpp_angle_1;
    Vector2f _tpp_angle_2;
    float torque_scale;
    float torque_mpog;
    float hover_coll = 5.0f;
    bool motor_interlock;
    uint8_t _time_delay;
    enum frame_types {
        HELI_FRAME_CONVENTIONAL,
        HELI_FRAME_DUAL,
        HELI_FRAME_COMPOUND,
        HELI_FRAME_BLADE360,
    } frame_type = HELI_FRAME_CONVENTIONAL;
    bool gas_heli = false;
    float nominal_rpm;
};

} // namespace SITL
