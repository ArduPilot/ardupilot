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

#include "SIM_Helicopter.h"

#include <stdio.h>

namespace SITL {

Helicopter::Helicopter(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = 4.54f;

    /*
       scaling from motor power to Newtons. Allows the copter
       to hover against gravity when the motor is at hover_throttle
       normalized to hover at 1500RPM at 5 deg collective.
    */
    thrust_scale = (mass * GRAVITY_MSS) / (hover_coll * sq(157.0f));

    // calculates tail rotor thrust to overcome rotor torque using the lean angle in a hover
    torque_scale = 0.83f * mass * GRAVITY_MSS * sinf(radians(hover_lean)) * tr_dist / (hover_coll * sq(157.0f));

    // torque with zero collective pitch. Percentage of total hover torque is based on full scale helicopters.
    torque_mpog = 0.17f * mass * GRAVITY_MSS * sinf(radians(hover_lean)) * tr_dist / sq(157.0f);

    frame_height = 0.1;

    if (strstr(frame_str, "-dual")) {
        frame_type = HELI_FRAME_DUAL;
    } else if (strstr(frame_str, "-compound")) {
        frame_type = HELI_FRAME_COMPOUND;
    } else {
        frame_type = HELI_FRAME_CONVENTIONAL;
    }
    gas_heli = (strstr(frame_str, "-gas") != nullptr);

    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    lock_step_scheduled = true;
}

/*
  update the helicopter simulation by one time step
 */
void Helicopter::update(const struct sitl_input &input)
{
    const float dt = frame_time_us * 1.0e-6f;

    // get wind vector setup
    update_wind(input);

    motor_interlock = input.servos[7] > 1400;
    
    float rsc = constrain_float((input.servos[7]-1000) / 1000.0f, 0, 1);
    float rsc_scale = rsc/rsc_setpoint;

    float thrust = 0;
    float roll_rate = 0;
    float pitch_rate = 0;
    float yaw_rate = 0;
    float torque_effect_accel = 0;
    float lateral_x_thrust = 0;
    float lateral_y_thrust = 0;


    if (_time_delay == 0) {
        for (uint8_t i = 0; i < 6; i++) {
            _servos_delayed[i] = input.servos[i];
        }
    } else if (servos_stored_buffer == nullptr) {
        uint16_t buffer_size = constrain_int16(_time_delay, 1, 100) * 0.001f / dt;
        servos_stored_buffer = new ObjectBuffer<servos_stored>(buffer_size);
        while (servos_stored_buffer->space() != 0) {
            push_to_buffer(input.servos);
        }
        for (uint8_t i = 0; i < 6; i++) {
            _servos_delayed[i] = input.servos[i];
        }
    } else {
        pull_from_buffer(_servos_delayed);
        push_to_buffer(input.servos);
    }

    float swash1 = (_servos_delayed[0]-1000) / 1000.0f;
    float swash2 = (_servos_delayed[1]-1000) / 1000.0f;
    float swash3 = (_servos_delayed[2]-1000) / 1000.0f;



    Vector3f rot_accel;
    Vector3f air_resistance;

    switch (frame_type) {
    case HELI_FRAME_CONVENTIONAL: {
        // simulate a traditional helicopter

        float Ma1s = 522.0f;
        float Lb1s = 922.0f;
        float Mu = 0.003f;
        float Lv = -0.006;
        float Xu = -0.125;
        float Yv = -0.375;
        float Zw = -0.375;

        float tail_rotor = (_servos_delayed[3]-1000) / 1000.0f;

        // determine RPM
        rpm[0] = update_rpm(motor_interlock, dt);

        // thrust calculated based on 5 deg hover collective for 10lb aircraft at 1500RPM
        float coll = 50.0f * (swash1+swash2+swash3) / 3.0f - 25.0f;
        thrust = thrust_scale * sq(rpm[0] * 0.104667f) * (0.25* (coll - hover_coll) + hover_coll);

        // Calculate main rotor torque effect on body
        torque_effect_accel = -1 * sq(rpm[0] * 0.104667f) * (torque_mpog + torque_scale * fabsf(coll)) / izz;

        // Calculate rotor tip path plane angle
        float roll_cyclic = (swash1 - swash2) / cyclic_scalar;
        float pitch_cyclic = ((swash1+swash2) / 2.0f - swash3) / cyclic_scalar;
        Vector2f ctrl_pos = Vector2f(roll_cyclic, pitch_cyclic);
        update_rotor_dynamics(gyro, ctrl_pos, _tpp_angle, dt);

        float yaw_cmd = 2.0f * tail_rotor - 1.0f; // convert range to -1 to 1
        float tail_rotor_torque = (21.6f * 2.96f * yaw_cmd - 2.96f * gyro.z) * sq(rpm[0] * 0.104667f) / sq(157.0f);
        float tail_rotor_thrust =  -1.0f * tail_rotor_torque * izz / tr_dist;  //right pedal produces left body accel

        // rotational acceleration, in rad/s/s, in body frame
        rot_accel.x = _tpp_angle.x * Lb1s + Lv * velocity_air_bf.y;
        rot_accel.y = _tpp_angle.y * Ma1s + Mu * velocity_air_bf.x;
        rot_accel.z = tail_rotor_torque + torque_effect_accel;

        lateral_y_thrust = tail_rotor_thrust / mass + GRAVITY_MSS * _tpp_angle.x + Yv * velocity_air_bf.y;
        lateral_x_thrust = -1.0f * GRAVITY_MSS * _tpp_angle.y + Xu * velocity_air_bf.x;
        accel_body = Vector3f(lateral_x_thrust, lateral_y_thrust, -thrust / mass + velocity_air_bf.z * Zw);

        break;
    }

    case HELI_FRAME_DUAL: {
        // simulate a tandem helicopter
        thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;

        float swash4 = (_servos_delayed[3]-1000) / 1000.0f;
        float swash5 = (_servos_delayed[4]-1000) / 1000.0f;
        float swash6 = (_servos_delayed[5]-1000) / 1000.0f;

        thrust = (rsc / rsc_setpoint) * (swash1+swash2+swash3+swash4+swash5+swash6) / 6.0f;
        torque_effect_accel = (rsc_scale + rsc / rsc_setpoint) * rotor_rot_accel * ((swash1+swash2+swash3) - (swash4+swash5+swash6));

        roll_rate = (swash1-swash2) + (swash4-swash5);
        pitch_rate = (swash1+swash2+swash3) - (swash4+swash5+swash6);
        yaw_rate = (swash1-swash2) + (swash5-swash4);

        roll_rate *= rsc_scale;
        pitch_rate *= rsc_scale;
        yaw_rate *= rsc_scale;

        // rotational acceleration, in rad/s/s, in body frame
        rot_accel.x = roll_rate * roll_rate_max;
        rot_accel.y = pitch_rate * pitch_rate_max;
        rot_accel.z = yaw_rate * yaw_rate_max;

        // rotational air resistance
        rot_accel.x -= gyro.x * radians(5000.0) / terminal_rotation_rate;
        rot_accel.y -= gyro.y * radians(5000.0) / terminal_rotation_rate;
        rot_accel.z -= gyro.z * radians(400.0)  / terminal_rotation_rate;

        // torque effect on tail
        rot_accel.z += torque_effect_accel;

        // air resistance
        air_resistance = -velocity_air_ef * (GRAVITY_MSS/terminal_velocity);

        // simulate rotor speed
        rpm[0] = thrust * 1300;

        // scale thrust to newtons
        thrust *= thrust_scale;

        accel_body = Vector3f(lateral_x_thrust, lateral_y_thrust, -thrust / mass);
        accel_body += dcm.transposed() * air_resistance;

        break;
    }

    case HELI_FRAME_COMPOUND: {
        // simulate a compound helicopter

        float Ma1s = 522.0f;
        float Lb1s = 922.0f;
        float Mu = 0.003f;
        float Lv = -0.006;
        float Xu = -0.125;
        float Yv = -0.375;
        float Zw = -0.375;

        // determine RPM
        rpm[0] = update_rpm(motor_interlock, dt);

        // thrust calculated based on 5 deg hover collective for 10lb aircraft at 1500RPM
        float coll = 50.0f * (swash1+swash2+swash3) / 3.0f - 25.0f;
        thrust = thrust_scale * sq(rpm[0] * 0.104667f) * (0.25* (coll - hover_coll) + hover_coll);

        // Calculate main rotor torque effect on body
        torque_effect_accel = -1 * sq(rpm[0] * 0.104667f) * (torque_mpog + torque_scale * fabsf(coll)) / izz;

        // Calculate rotor tip path plane angle
        float roll_cyclic = (swash1 - swash2) / cyclic_scalar;
        float pitch_cyclic = ((swash1+swash2) / 2.0f - swash3) / cyclic_scalar;
        Vector2f ctrl_pos = Vector2f(roll_cyclic, pitch_cyclic);
        update_rotor_dynamics(gyro, ctrl_pos, _tpp_angle, dt);

        // Calculate thruster yaw and forward thrust effects
        // Thruster command range -1 to 1.  Positive is forward thrust for both
        float right_thruster_cmd = 2.0f * (_servos_delayed[3]-1000) / 1000.0f - 1.0f;
        float left_thruster_cmd = 2.0f * (_servos_delayed[4]-1000) / 1000.0f - 1.0f;

        // assume torque from each thruster only half of normal tailrotor since thrusters 1/2 distance from cg
        float right_thruster_torque = (-0.5f * 21.6f * 2.96f * right_thruster_cmd - 2.96f * gyro.z) * sq(rpm[0] * 0.104667f) / sq(157.0f);
        float left_thruster_torque = (0.5f * 21.6f * 2.96f * left_thruster_cmd - 2.96f * gyro.z) * sq(rpm[0] * 0.104667f) / sq(157.0f);

        float right_thruster_force = -1.0f * right_thruster_torque * izz / (0.5f * tr_dist);
        float left_thruster_force = left_thruster_torque * izz / (0.5f * tr_dist);

        // rotational acceleration, in rad/s/s, in body frame
        rot_accel.x = _tpp_angle.x * Lb1s + Lv * velocity_air_bf.y;
        rot_accel.y = _tpp_angle.y * Ma1s + Mu * velocity_air_bf.x;
        rot_accel.z = right_thruster_torque + left_thruster_torque + torque_effect_accel;

        lateral_y_thrust = GRAVITY_MSS * _tpp_angle.x + Yv * velocity_air_bf.y;
        lateral_x_thrust = (right_thruster_force + left_thruster_force) / mass - GRAVITY_MSS * _tpp_angle.y + Xu * velocity_air_bf.x;
        accel_body = Vector3f(lateral_x_thrust, lateral_y_thrust, -thrust / mass + velocity_air_bf.z * Zw);

        break;
    }
    }


    update_dynamics(rot_accel);
    
    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

void Helicopter::update_rotor_dynamics(Vector3f gyros, Vector2f ctrl_pos, Vector2f &tpp_angle, float dt)
{

    float tf_inv = 1.0f / 0.07135f;
    float Lfa1s = 0.83641f;
    float Mfb1s = -0.89074f;
    float Lflt = 1.7869f;
    float Lflg = -0.39394f;
    float Mflt = 0.46231f;
    float Mflg = 2.4099f;

    float b1s_dot = -1 * gyro.x - tf_inv * tpp_angle.x + tf_inv * (Lfa1s * tpp_angle.y + Lflt * ctrl_pos.x + Lflg * ctrl_pos.y);
    float a1s_dot = -1 * gyro.y - tf_inv * tpp_angle.y + tf_inv * (Mfb1s * tpp_angle.x + Mflt * ctrl_pos.x + Mflg * ctrl_pos.y);

    tpp_angle.x += b1s_dot * dt;
    tpp_angle.y += a1s_dot * dt;

}

float Helicopter::update_rpm(bool interlock, float dt)
{
    static float rotor_runup_output;
    float runup_time = 8.0f;
    // ramp speed estimate towards control out
    float runup_increment = dt / runup_time;
    if (interlock) {
        if (rotor_runup_output < 1.0f) {
            rotor_runup_output += runup_increment;
        } else {
            rotor_runup_output = 1.0f;
        }
    }else{
        if (rotor_runup_output > 0.0f) {
            rotor_runup_output -= runup_increment;
        } else {
            rotor_runup_output = 0.0f;
        }
    }

    return 1500.0f * constrain_float(rotor_runup_output,0.0f,1.0f);

}

// push servo input to buffer
void Helicopter::push_to_buffer(const uint16_t servos_input[16])
{
    servos_stored sample;
    sample.servo1 = servos_input[0];
    sample.servo2 = servos_input[1];
    sample.servo3 = servos_input[2];
    sample.servo4 = servos_input[3];
    sample.servo5 = servos_input[4];
    sample.servo6 = servos_input[5];
    servos_stored_buffer->push(sample);

}

// pull servo delay from buffer
void Helicopter::pull_from_buffer(uint16_t servos_delayed[6])
{
    servos_stored sample;
    if (!servos_stored_buffer->pop(sample)) {
        // no sample
        return;
    }
    servos_delayed[0] = sample.servo1;
    servos_delayed[1] = sample.servo2; 
    servos_delayed[2] = sample.servo3;
    servos_delayed[3] = sample.servo4;
    servos_delayed[4] = sample.servo5;
    servos_delayed[5] = sample.servo6;

}

} // namespace SITL
