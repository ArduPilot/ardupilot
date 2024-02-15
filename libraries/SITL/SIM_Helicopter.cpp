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
#include <GCS_MAVLink/GCS.h>

namespace SITL {

Helicopter::Helicopter(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = 4.54f;

    if (strstr(frame_str, "-dual")) {
        frame_type = HELI_FRAME_DUAL;
        _time_delay = 30;
        nominal_rpm = 1300;
        mass = 9.08f;
        iyy = 0.2f;
    } else if (strstr(frame_str, "-compound")) {
        frame_type = HELI_FRAME_COMPOUND;
        _time_delay = 50;
        nominal_rpm = 1500;
    } else if (strstr(frame_str, "-blade360")) {
        frame_type = HELI_FRAME_BLADE360;
        _time_delay = 40;
        nominal_rpm = 2100;
    } else {
        frame_type = HELI_FRAME_CONVENTIONAL;
        _time_delay = 50;
        nominal_rpm = 1500;
    }

    /*
       For conventional and compound
       scaling from motor power to Newtons. Allows the copter
       to hover against gravity when the motor is at hover_throttle
       normalized to hover at 1500RPM at 5 deg collective.
    */
    thrust_scale = (mass * GRAVITY_MSS) / (hover_coll * sq(nominal_rpm * 2.0f * M_PI / 60.0f));

    // torque with zero collective pitch. Percentage of total +hover torque is based on full scale helicopters.
    torque_mpog = 1.08f / sq(nominal_rpm * 2.0f * M_PI / 60.0f);  // based on separate calculation

    // torque_max based on power being weight^3/2.  Assuming thrust is linear with collective, it can be 
    // determined that max torque would be 3 times hover torque.  Then dividing by 1-MPOG_torq to get total torque.
    torque_max = ((mass * GRAVITY_MSS * sinf(radians(hover_lean)) * tr_dist - torque_mpog * sq(nominal_rpm * 2.0f * M_PI / 60.0f)) * powf(2.0f,1.5f) + torque_mpog * sq(nominal_rpm * 2.0f * M_PI / 60.0f)) / sq(nominal_rpm * 2.0f * M_PI / 60.0f);

    // calculates tail rotor thrust to overcome rotor torque using the lean angle in a hover
    torque_scale = (torque_max - torque_mpog) / powf(10.0f,1.5f);

    frame_height = 0.1;

    gas_heli = (strstr(frame_str, "-gas") != nullptr);

    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    lock_step_scheduled = true;

    motor_mask |= (1U<<0);
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

    float thrust = 0;
    float thrust_1 = 0;
    float thrust_2 = 0;
    float eng_torque = 0;
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

    switch (frame_type) {
    case HELI_FRAME_CONVENTIONAL: {
        // simulate a traditional helicopter

        float Ma1s = 617.5f;
        float Lb1s = 3588.6f;
        float Mu = 0.003f;
        float Lv = -0.006;
        float Xu = -0.125;
        float Yv = -0.375;
        float Zw = -2.25;

        float tail_rotor = (_servos_delayed[3]-1000) / 1000.0f;

        // thrust calculated based on 5 deg hover collective for 10lb aircraft at 1500RPM
        float coll = 50.0f * (swash1+swash2+swash3) / 3.0f - 25.0f;
        thrust = (rpm[0] / nominal_rpm) * thrust_scale * sq(nominal_rpm * 0.104667f) * coll;

        // determine RPM
        rpm[0] = update_rpm(rpm[0], rsc, eng_torque, coll, dt);

        // Calculate rotor tip path plane angle
        float roll_cyclic = 1.283 * (swash1 - swash2) / cyclic_scalar;
        float pitch_cyclic = 1.48 * ((swash1+swash2) / 2.0f - swash3) / cyclic_scalar;
        Vector2f ctrl_pos = Vector2f(roll_cyclic, pitch_cyclic);
        update_rotor_dynamics(gyro, ctrl_pos, _tpp_angle, dt);

        float yaw_cmd = 2.0f * tail_rotor - 1.0f; // convert range to -1 to 1
        float tail_rotor_torque = (21.6f * 2.96f * yaw_cmd - 2.96f * gyro.z) * sq(rpm[0]/nominal_rpm);
        float tail_rotor_thrust =  -1.0f * tail_rotor_torque * izz / tr_dist;  //right pedal produces left body accel

        // rotational acceleration, in rad/s/s, in body frame
        rot_accel.x = _tpp_angle.x * Lb1s + Lv * velocity_air_bf.y;
        rot_accel.y = _tpp_angle.y * Ma1s + Mu * velocity_air_bf.x;
        rot_accel.z = tail_rotor_torque - eng_torque;

        lateral_y_thrust = tail_rotor_thrust / mass + GRAVITY_MSS * _tpp_angle.x + Yv * velocity_air_bf.y;
        lateral_x_thrust = -1.0f * GRAVITY_MSS * _tpp_angle.y + Xu * velocity_air_bf.x;
        accel_body = Vector3f(lateral_x_thrust, lateral_y_thrust, -thrust / mass + velocity_air_bf.z * Zw);

        break;
    }

    case HELI_FRAME_BLADE360: {
        // simulate a Blade 360 helicopter.  This model was taken from the following reference.
        // Walker, J, Tishler, M, "Identification and Control Design of a Sub-Scale Flybarless Helicopter",
        // Vertical Flight Society’s 77th Annual Forum & Technology Display, Virtual, May 10-14, 2021.

        float Ma1s = 796.7f;
        float Lb1s = 5115.2f;
        float Mu = 2.7501f;
        float Mv = -2.3039f;
        float Lu = -28.7796f;
        float Lv = -5.5376f;
        float Xu = -0.2270f;
        float Yv = -0.1852f;
        float Yp = 0.2303f;
        float Zw = -0.5910f;
        float Nr = -2.0131f;
        float Nw = 5.7574f;
        float Nv = 1.7258f;
        float Ncol = -32.4616f;
        float Nped = 63.0040f;
        float Zcol = -22.3239f;

        float tail_rotor = (_servos_delayed[3]-1000) / 1000.0f;

        // collective adjusted for coll_min(1460) to coll_max(1740) as 0 to 1 with 1500 being zero thrust
        float coll = 3.51 * ((swash1+swash2+swash3) / 3.0f - 0.5f);

        // determine RPM
        rpm[0] = update_rpm(rpm[0], rsc, eng_torque, coll, dt);

        // Calculate rotor tip path plane angle
        float roll_cyclic = 1.283f * (swash1 - swash2);
        float pitch_cyclic = 1.48f * ((swash1+swash2) / 2.0f - swash3);
        Vector2f ctrl_pos = Vector2f(roll_cyclic, pitch_cyclic);
        update_rotor_dynamics(gyro, ctrl_pos, _tpp_angle, dt);

        float yaw_cmd = 1.45f * (2.0f * tail_rotor - 1.0f); // convert range to -1 to 1

        // rotational acceleration, in rad/s/s, in body frame
        rot_accel.x = _tpp_angle.x * Lb1s + Lu * velocity_air_bf.x + Lv * velocity_air_bf.y;
        rot_accel.y = _tpp_angle.y * Ma1s + Mu * velocity_air_bf.x + Mv * velocity_air_bf.y;
        rot_accel.z = Nv * velocity_air_bf.y + Nr * gyro.z + sq(rpm[0]/nominal_rpm) * Nped * yaw_cmd + Nw * velocity_air_bf.z + sq(rpm[0]/nominal_rpm) * Ncol * (coll - 0.5f);

        lateral_y_thrust = GRAVITY_MSS * _tpp_angle.x + Yv * velocity_air_bf.y + Yp * gyro.x - 3.2 * 0.01745 * GRAVITY_MSS;
        lateral_x_thrust = -1.0f * GRAVITY_MSS * _tpp_angle.y + Xu * velocity_air_bf.x;
        float vertical_thrust = Zcol * coll * sq(rpm[0]/nominal_rpm) + velocity_air_bf.z * Zw;
        accel_body = Vector3f(lateral_x_thrust, lateral_y_thrust, vertical_thrust);

        break;
    }

    case HELI_FRAME_DUAL: {

        float Ma1s = 617.5f;
        float Lb1s = 3588.6f;
        float Mu = 0.003f;
        float Lv = -0.006f;
        float Xu = -0.125f;
        float Yv = -0.375f;
        float Zw = -2.25f;
        float hub_dist = 1.8f; //meters

        float swash4 = (_servos_delayed[3]-1000) / 1000.0f;
        float swash5 = (_servos_delayed[4]-1000) / 1000.0f;
        float swash6 = (_servos_delayed[5]-1000) / 1000.0f;

        // Forward rotor is number 1
        // thrust calculated based on 5 deg hover collective for 10lb aircraft at 1500RPM
        float coll_1 = 50.0f * (swash1+swash2+swash3) / 3.0f - 25.0f;

        // Calculate rotor tip path plane angle
        float roll_cyclic_1 = 1.283 * (swash1 - swash2) / cyclic_scalar;
        float pitch_cyclic_1 = 1.48 * ((swash1+swash2) / 2.0f - swash3) / cyclic_scalar;
        Vector2f ctrl_pos_1 = Vector2f(roll_cyclic_1, pitch_cyclic_1);
        update_rotor_dynamics(gyro, ctrl_pos_1, _tpp_angle_1, dt);

        // Aft rotor is number 2
        // thrust calculated based on 5 deg hover collective for 10lb aircraft at 1500RPM
        float coll_2 = 50.0f * (swash4+swash5+swash6) / 3.0f - 25.0f;

        // Calculate rotor tip path plane angle
        float roll_cyclic_2 = 1.283 * (swash4 - swash5) / cyclic_scalar;
        float pitch_cyclic_2 = 1.48 * ((swash4+swash5) / 2.0f - swash6) / cyclic_scalar;
        Vector2f ctrl_pos_2 = Vector2f(roll_cyclic_2, pitch_cyclic_2);
        update_rotor_dynamics(gyro, ctrl_pos_2, _tpp_angle_2, dt);

        // determine RPM
        rpm[0] = update_rpm(rpm[0], rsc, eng_torque, (coll_1 + coll_2) * 0.5f, dt);

        thrust_1 = 0.5f * thrust_scale * sq(rpm[0] * 0.104667f) * coll_1;
        thrust_2 = 0.5f * thrust_scale * sq(rpm[0] * 0.104667f) * coll_2;

        // rotational acceleration, in rad/s/s, in body frame
        rot_accel.x = (_tpp_angle_1.x + _tpp_angle_2.x) * Lb1s + Lv * velocity_air_bf.y;
        rot_accel.y = (_tpp_angle_1.y + _tpp_angle_2.y) * Ma1s + (thrust_1 - thrust_2) * hub_dist / iyy + Mu * velocity_air_bf.x + hub_dist * gyro.y * Zw;
        rot_accel.z = (_tpp_angle_1.x * thrust_1 - _tpp_angle_2.x * thrust_2) * hub_dist / (iyy * 2.0f) - 0.5f * gyro.z;

        lateral_y_thrust = GRAVITY_MSS * (_tpp_angle_1.x + _tpp_angle_2.x) + Yv * velocity_air_bf.y;
        lateral_x_thrust = -1.0f * GRAVITY_MSS * (_tpp_angle_1.y + _tpp_angle_2.y) + Xu * velocity_air_bf.x;
        accel_body = Vector3f(lateral_x_thrust, lateral_y_thrust, -(thrust_1 + thrust_2) / mass + velocity_air_bf.z * Zw);


        break;
    }

    case HELI_FRAME_COMPOUND: {
        // simulate a compound helicopter

        float Ma1s = 617.5f;
        float Lb1s = 3588.6f;
        float Mu = 0.003f;
        float Lv = -0.006;
        float Xu = -0.125;
        float Yv = -0.375;
        float Zw = -2.25;

        // thrust calculated based on 5 deg hover collective for 10lb aircraft at 1500RPM
        float coll = 50.0f * (swash1+swash2+swash3) / 3.0f - 25.0f;
        thrust = thrust_scale * sq(rpm[0] * 0.104667f) * coll;

        // determine RPM
        rpm[0] = update_rpm(rpm[0], rsc, eng_torque, coll, dt);

         // Calculate rotor tip path plane angle
        float roll_cyclic = 1.283 * (swash1 - swash2) / cyclic_scalar;
        float pitch_cyclic = 1.48 * ((swash1+swash2) / 2.0f - swash3) / cyclic_scalar;
        Vector2f ctrl_pos = Vector2f(roll_cyclic, pitch_cyclic);
        update_rotor_dynamics(gyro, ctrl_pos, _tpp_angle, dt);

        // Calculate thruster yaw and forward thrust effects
        // Thruster command range -1 to 1.  Positive is forward thrust for both
        float right_thruster_cmd = 2.0f * (_servos_delayed[3]-1000) / 1000.0f - 1.0f;
        float left_thruster_cmd = 2.0f * (_servos_delayed[4]-1000) / 1000.0f - 1.0f;

        // assume torque from each thruster only half of normal tailrotor since thrusters 1/2 distance from cg
        float right_thruster_torque = (-0.5f * 21.6f * 2.96f * right_thruster_cmd - 2.96f * gyro.z) * sq(rpm[0] / nominal_rpm);
        float left_thruster_torque = (0.5f * 21.6f * 2.96f * left_thruster_cmd - 2.96f * gyro.z) * sq(rpm[0] / nominal_rpm);

        float right_thruster_force = -1.0f * right_thruster_torque * izz / (0.5f * tr_dist);
        float left_thruster_force = left_thruster_torque * izz / (0.5f * tr_dist);

        // rotational acceleration, in rad/s/s, in body frame
        rot_accel.x = _tpp_angle.x * Lb1s + Lv * velocity_air_bf.y;
        rot_accel.y = _tpp_angle.y * Ma1s + Mu * velocity_air_bf.x;
        rot_accel.z = right_thruster_torque + left_thruster_torque - eng_torque;

        lateral_y_thrust = GRAVITY_MSS * _tpp_angle.x + Yv * velocity_air_bf.y;
        lateral_x_thrust = (right_thruster_force + left_thruster_force) / mass - GRAVITY_MSS * _tpp_angle.y + Xu * velocity_air_bf.x;
        accel_body = Vector3f(lateral_x_thrust, lateral_y_thrust, -thrust / mass + velocity_air_bf.z * Zw);

        break;
    }
    }


    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

void Helicopter::update_rotor_dynamics(Vector3f gyros, Vector2f ctrl_pos, Vector2f &tpp_angle, float dt)
{

    float tf_inv;
    float Lfa1s;
    float Mfb1s;
    float Lflt;
    float Lflg;
    float Mflt;
    float Mflg;

    if (frame_type == HELI_FRAME_BLADE360) {
        tf_inv = 1.0f / 0.0353f;
        Lfa1s = 1.0477f;
        Mfb1s = -1.0057f;
        Lflt = 0.2375f;
        Lflg = -0.0286f;
        Mflt = 0.0344f;
        Mflg = 0.2292f;
    } else if (frame_type == HELI_FRAME_DUAL) { // remove coupling in rotor
        tf_inv = 1.0f / 0.068232f;
        Lfa1s = 0.0f;
        Mfb1s = 0.0f;
        Lflt = 1.7635f;
        Lflg = 0.0f;
        Mflt = 0.0f;
        Mflg = 1.9432f;
    } else {
        tf_inv = 1.0f / 0.068232f;
        Lfa1s = 1.2963f;
        Mfb1s = -1.3402f;
        Lflt = 1.7635f;
        Lflg = -0.61171f;
        Mflt = 0.52454f;
        Mflg = 1.9432f;
    }

    float b1s_dot = -1 * gyro.x - tf_inv * tpp_angle.x + tf_inv * (Lfa1s * tpp_angle.y + Lflt * ctrl_pos.x + Lflg * ctrl_pos.y);
    float a1s_dot = -1 * gyro.y - tf_inv * tpp_angle.y + tf_inv * (Mfb1s * tpp_angle.x + Mflt * ctrl_pos.x + Mflg * ctrl_pos.y);

    tpp_angle.x += b1s_dot * dt;
    tpp_angle.y += a1s_dot * dt;

}

float Helicopter::update_rpm(float curr_rpm, float throttle, float &engine_torque, float collective, float dt)
{
    static float rotor_runup_output;
    static uint8_t motor_status;
    float accel_scale;
    float input_torque;
    float auto_ss_torque;
    float descent_torque;
    float rotor_torque;
    float rpm_dot;
    float rpm_engine;

    //use this to make rpm model more realistic
    accel_scale = 100.0f;
    input_torque = 0.0f;

    // calculate aerodynamic rotor drag torque
    rotor_torque = (sq(curr_rpm * 0.104667f) * (torque_mpog + torque_scale * powf(fabsf(collective),1.5f))) / izz;

    // Calculate autorotation effect on rotor
    auto_ss_torque = sq(nominal_rpm * 0.104667f) * (torque_mpog + torque_scale * powf(fabsf(-1.0f),1.5f)) / izz;
    if (is_positive(velocity_air_bf.z)) {
        descent_torque = (velocity_air_bf.z - 5.3) * auto_ss_torque / 5.3f + auto_ss_torque;
    } else {
        descent_torque = 0.0f;
    }

    // calculate max engine torque
    float engine_torque_max = sq(nominal_rpm * 0.104667f) * torque_max / izz;

    if (gas_heli) {

        // calculate engine RPM.  Based off of throttle and it matches rotor RPM at 30% throttle
        rpm_engine = nominal_rpm * throttle / 0.3f;

        // calculate engine torque.  extra 20% given to have a little extra power
        engine_torque = 1.20f * throttle * engine_torque_max;

        // model clutch on gas heli 
        if (throttle >= 0.15f && rpm_engine > curr_rpm) {
            input_torque = engine_torque;
        } else {
            input_torque = 0.0f;
        }

        rpm_dot = 0.0f;
        // help spool down quickly go to zero
        if (throttle <= 0.15f && curr_rpm < 300) {
            rpm_dot = - 40.0f;
            if (curr_rpm <= 0.0f) {
                rpm_dot = 0.0f;
                curr_rpm = 0.0f;
            }
        } else {
            rpm_dot = accel_scale * (input_torque + descent_torque - rotor_torque);
            if (curr_rpm <= 0.0 && !is_positive(rpm_dot)) {
                rpm_dot = 0.0f;
                curr_rpm = 0.0f;
            }
        }
    } else {
        if (throttle > 0.25) {
            motor_status = 3; // throttle unlimited
        } else if (motor_status == 3 && throttle <= 0.25 && throttle > 0.15) {
            motor_status = 2; // autorotational window
        } else if (throttle <= 0.15) {
            motor_status = 1; // idle
        }

        float runup_time = 8.0f;
        if (motor_status == 2) {
            runup_time = 2.0f;
        }

        // ramp speed estimate towards control out
        float runup_increment = dt / runup_time;
        if (motor_status > 2) {
            if (rotor_runup_output < 1.0f) {
                rotor_runup_output += runup_increment;
            } else {
                rotor_runup_output = 1.0f;
            }
            if (curr_rpm < nominal_rpm - 25.0f) {
                accel_scale = 2000.0f / runup_time;
            }
        }else{
            if (rotor_runup_output > 0.0f) {
                rotor_runup_output -= runup_increment * 10.0f; // make ramp down 10 times faster
            } else {
                rotor_runup_output = 0.0f;
            }
        }

        // calculate engine torque just for start up and shutdown of rotor
        engine_torque = 0.333f * rotor_runup_output * engine_torque_max;

        // manage input torque so descent torque combined with engine torque doesn't allow rotor to overspeed
        if (rotor_runup_output >= 1.0f && curr_rpm > nominal_rpm - 100.0f) {
            // want the rpm to seek the nominal rpm so set the input torque to only be that for nominal RPM
            input_torque = rotor_torque * sq(nominal_rpm / curr_rpm);
        } else if (rotor_runup_output <= 0.0f) {
            input_torque = descent_torque;
        } else {
            input_torque = engine_torque + descent_torque;
        }

        rpm_dot = 0.0f;
        // Help spool down quickly got to zero 
        if (rotor_runup_output <= 0.0f && curr_rpm < 300) {
            rpm_dot = - 40.0f;
            if (curr_rpm <= 0.0f) {
                rpm_dot = 0.0f;
                curr_rpm = 0.0f;
            }
        } else {
            rpm_dot = accel_scale * (input_torque - rotor_torque);
        }
        engine_torque = input_torque;
    }

    curr_rpm += rpm_dot * dt;

    return curr_rpm;

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
