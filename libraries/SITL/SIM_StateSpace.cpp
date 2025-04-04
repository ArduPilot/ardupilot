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
  State Space simulator class

  States
  u, v, w - longitudinal, lateral, and vertical body frame velocity
  p, q, r - roll, pitch, and yaw body frame angular rates
  a1s, b1s - rotor longitudinal and lateral flapping
  dlatlag, dlonlag, dpedlag, dverlag - lagged states for control inputs

  Derivatives (all are shown for completeness but may not be in equation)
  Xu, Xv, Xw, Xp, Xq, Xr - longitudinal axis stability derivatives
  Yu, Yv, Yw, Yp, Yq, Yr - lateral axis stability derivatives
  Zu, Zv, Zw, Zp, Zq, Zr - vertical axis stability derivatives

  Lu, Lv, Lw, Lp, Lq, Lr - roll axis stability derivatives
  Mu, Mv, Mw, Mp, Mq, Mr - pitch axis stability derivatives
  Nu, Nv, Nw, Np, Nq, Nr - yaw axis stability derivatives

  Xlat, Xlon, Xped, Xvert - longitudinal axis control derivatives
  Ylat, Ylon, Yped, Yvert - lateral axis control derivatives
  Zlat, Zlon, Zped, Zvert - vertical axis control derivatives

  Llat, Llon, Lped, Lvert - roll axis control derivatives
  Mlat, Mlon, Mped, Mvert - pitch axis control derivatives
  Nlat, Nlon, Nped, Nvert - yaw axis control derivatives

  helicopter specific derivatives
  Xa1s, Yb1s - aircraft longitudinal and lateral axis rotor flapping derivatives
  Ma1s, Lb1s - aircraft pitch and roll axis rotor flapping derivatives
  Mfb1s, Lfa1s- rotor pitch and roll axis coupling derivative
  Mfln, Mflt, Lfln, Lflt - rotor control input derivatives
  tf - rotor flapping time constant

  multirotor specific terms
  Lag - lag cutoff frequency for motor response
  Lead - lead cutoff frequency for torque response

The multirotor state space equations of motion are given below
  
    roll accel = Lv * v + Lp * p + Llat * Dlatlag
    pitch accel = Mu * u + Mq * q + Mlon * Dlonlag
    yaw accel = Nr * r + (Nped - Lag * Lead) * Dpedlag + Lag * Lead * _yaw_in

    lateral accel = Yv * v + Ylat * Dlatlag
    longitudinal accel = Xu * u + Xlon * Dlonlag
    vertical accel = Zcol * (Dcollag - hover throttle) - 9.81 + Zw * w

equations for lagged control inputs for modeling motor response
    Dlatlag_dot = -Lag * Dlatlag + Lag * _roll_in
    Dlonlag_dot = -Lag * Dlonlag + Lag * _pitch_in
    Dcollag_dot = -Lag * Dcollag + Lag * _throttle_in
    Dpedlag_dot = -Lag * Dpedlag + Lag * _yaw_in

The Helicopter state space equations of motion are given below

the helicopter can be modeled in two ways however any combination of these models could be used as the equations
in the code include all of the terms from both model methods for modeling a helicopter

Without a rotor model
    roll accel = Lu * u + Lv * v + Lq * q + Lp * p + Lr * r + Llon * _pitch_in + Llat * _roll_in
    pitch accel = Mu * u + Mv * v + Mq * q + Mp * p + Mlon * _pitch_in + Mlat * _roll_in
    yaw accel = Nv * v + Nw * w + Nr * r + Ncol * (_coll_in - hover_coll) + Nped * _yaw_in

    lateral accel = Yv * v + Yp * p + Ylat * _roll_in - hover_lean * 9.81
    longitudinal accel = Xu * u + Xlon * _pitch_in
    vertical accel = Zcol * (_coll_in - hover coll) - 9.81 + Zw * w

With a rotor model
    roll accel = Lu * u + Lv * v + Lb1s * b1s
    pitch accel = Mu * u + Mv * v + Ma1s * a1s
    yaw accel = Nv * v + Nw * w + Nr * r + Ncol * (_coll_in - hover_coll) + Nped * _yaw_in

    lateral accel = Yv * v + Yp * p  + Yb1s * b1s - hover_lean * 9.81
    longitudinal accel = Xu * u + Xa1s * a1s
    vertical accel = Zcol * (_coll_in - hover coll) - 9.81 + Zw * w

    b1s_dot = - p - b1s / tf +  (Lfa1s * a1s + Lflt * roll_in + Lfln * pitch_in) / tf
    a1s_dot = - q - a1s / tf + (Mfb1s * b1s + Mflt * roll_in + Mfln * pitch_in) / tf

*/

#include "SIM_StateSpace.h"
#include <AP_Filesystem/AP_Filesystem.h>

#include <stdio.h>
#include <sys/stat.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

namespace SITL {

StateSpace::StateSpace(const char *frame_str) :
    Aircraft(frame_str)
{

    
    if (strstr(frame_str, "-heli")) {
        frame_type = HELI_FRAME;

    } else if (strstr(frame_str, "-multi")) {
        frame_type = MULTI_FRAME;

    } else {
        frame_type = MULTI_FRAME;
    }

    const char *colon = strchr(frame_str, ':');
    size_t slen = strlen(frame_str);
    if (colon != nullptr && slen > 5 && strcmp(&frame_str[slen-5], ".json") == 0) {
        load_frame_params(colon+1);
    }

    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    lock_step_scheduled = true;

    motor_mask |= (1U<<0);
}

/*
  update the helicopter simulation by one time step
 */
void StateSpace::update(const struct sitl_input &input)
{

    static uint64_t last_calc_us;
    uint64_t now_us = AP_HAL::micros64();
    float dt = 0.0f;
    if (last_calc_us != 0) {
        dt = (now_us - last_calc_us)*1.0e-6;
    }
    last_calc_us = now_us;

    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    switch (frame_type) {
        case HELI_FRAME: {
            if (model.coll_max - model.coll_min > 0.0f) {
                hover_coll = (model.coll_hover - model.coll_min) / (model.coll_max - model.coll_min);
            }
            float servos_raw[16];
            float swash_roll = (input.servos[0]-1000) / 1000.0f;
            float swash_pitch = (input.servos[1]-1000) / 1000.0f;
            float swash_coll = (input.servos[2]-1000) / 1000.0f;
            float tail_rotor = (input.servos[3] - 1000) / 1000.0f;
            float rsc = constrain_float((input.servos[7]-1000) / 1000.0f, 0, 1);

            // the factor of 0.9 comes from the swashplate factor for pitch and roll
            // roll command
            servos_raw[0] = (2.0f * swash_roll - 1.0f) / 0.9f;
            // pitch command
            servos_raw[1] = (2.0f * swash_pitch - 1.0f) / 0.9f;
            // collective adjusted for coll_min(1460) to coll_max(1740) as 0 to 1 with 1500 being zero thrust
            servos_raw[2] = swash_coll;
            // yaw command
            // scale factor of 1.25 is used because default servo range is 1100-1900
            servos_raw[3]  = (2.0f * tail_rotor - 1.0f) * 1.25f;  // yaw

            Log_Write_SimData(servos_raw[1], servos_raw[0], servos_raw[2], servos_raw[3]);

            // this adds time delay for roll and pitch inputs
            if ((uint16_t)model.time_delay_rp == 0 || is_zero(dt)) {
                for (uint8_t i = 0; i < 2; i++) {
                _servos_delayed_rp[i] = servos_raw[i];
                    }
            } else if (servos_stored_buffer == nullptr) {
                    uint16_t buffer_size = constrain_int16((uint16_t)model.time_delay_rp, 1, 100) * 0.001f / dt;
                    servos_stored_buffer = new ObjectBuffer<servos_stored>(buffer_size);
                    while (servos_stored_buffer->space() != 0) {
                        push_to_buffer_rp(servos_raw);
                    }
                    for (uint8_t i = 0; i < 2; i++) {
                        _servos_delayed_rp[i] = servos_raw[i];
                    }
            } else {
                    pull_from_buffer_rp(_servos_delayed_rp);
                    push_to_buffer_rp(servos_raw);
            }

            // this adds time delay for throttle inputs
            if ((uint16_t)model.time_delay_col == 0 || is_zero(dt)) {
                _servo_delayed_col = servos_raw[2];
            } else if (servo_stored_col_buffer == nullptr) {
                    uint16_t buffer_size = constrain_int16((uint16_t)model.time_delay_col, 1, 100) * 0.001f / dt;
                    servo_stored_col_buffer = new ObjectBuffer<servo_stored>(buffer_size);
                    while (servo_stored_col_buffer->space() != 0) {
                        push_to_buffer_col(servos_raw);
                    }
                    _servo_delayed_col = servos_raw[2];
            } else {
                    pull_from_buffer_col(_servo_delayed_col);
                    push_to_buffer_col(servos_raw);
            }

            // this adds time delay for pedal inputs
            if ((uint16_t)model.time_delay_ped == 0 || is_zero(dt)) {
                _servo_delayed_ped = servos_raw[3];
            } else if (servo_stored_ped_buffer == nullptr) {
                    uint16_t buffer_size = constrain_int16((uint16_t)model.time_delay_ped, 1, 100) * 0.001f / dt;
                    servo_stored_ped_buffer = new ObjectBuffer<servo_stored>(buffer_size);
                    while (servo_stored_ped_buffer->space() != 0) {
                        push_to_buffer_ped(servos_raw);
                    }
                    _servo_delayed_ped = servos_raw[3];
            } else {
                    pull_from_buffer_ped(_servo_delayed_ped);
                    push_to_buffer_ped(servos_raw);
            }

            // calculated motors class inputs from delayed servo_out for state space model input
            float _roll_in = _servos_delayed_rp[0];  // roll
            float _pitch_in = _servos_delayed_rp[1];  //pitch
            float _coll_in = _servo_delayed_col;  // throttle
            float _yaw_in  = _servo_delayed_ped;  // yaw

            // determine RPM
            rpm[0] = update_rpm(rpm[0], rsc, model.nominal_rpm, dt);

            // Calculate rotor tip path plane angle
            Vector2f ctrl_pos = Vector2f(_roll_in, _pitch_in);
            update_rotor_dynamics(gyro, ctrl_pos, _tpp_angle, dt);

            // rotational acceleration, in rad/s/s, in body frame
            rot_accel.x = _tpp_angle.x * model.Lb1s + model.Lu * velocity_air_bf.x + model.Lv * velocity_air_bf.y + model.Llon * _pitch_in + model.Llat * _roll_in + model.Lq * gyro.y + model.Lp * gyro.x;
            rot_accel.y = _tpp_angle.y * model.Ma1s + model.Mu * velocity_air_bf.x + model.Mv * velocity_air_bf.y + model.Mlon * _pitch_in + model.Mlat * _roll_in + model.Mq * gyro.y + model.Mp * gyro.x;
            rot_accel.z = model.Nv * velocity_air_bf.y + model.Nr * gyro.z + sq(rpm[0]/model.nominal_rpm) * model.Nped * _yaw_in + model.Nw * velocity_air_bf.z + sq(rpm[0]/model.nominal_rpm) * model.Ncol * (_coll_in - hover_coll);

            float lateral_y_thrust = model.Yb1s * _tpp_angle.x + model.Yv * velocity_air_bf.y + model.Yp * gyro.x - model.hover_lean * 0.01745 * GRAVITY_MSS + model.Ylat * _roll_in;
            float lateral_x_thrust = model.Xa1s * _tpp_angle.y + model.Xu * velocity_air_bf.x + model.Xlon * _pitch_in;
            float vertical_thrust = (model.Zcol * (_coll_in - hover_coll) - GRAVITY_MSS) * sq(rpm[0]/model.nominal_rpm) + velocity_air_bf.z * model.Zw;
            accel_body = Vector3f(lateral_x_thrust, lateral_y_thrust, vertical_thrust);

            break;
        }

        case MULTI_FRAME: {

            float servos_raw[16];
            float servos_adj[16];
            // A scale factor has to be used to make the motors class inputs match the calculated inputs from the servos.
            float servo_sf = 1.0;
            servos_adj[0]= (input.servos[0] - 1000) * servo_sf + 1000;
            servos_adj[1]= (input.servos[1] - 1000) * servo_sf + 1000;
            servos_adj[2]= (input.servos[2] - 1000) * servo_sf + 1000;
            servos_adj[3]= (input.servos[3] - 1000) * servo_sf + 1000;

            // this section turns the servo outputs into the motors class inputs in PWM
            // it uses the frame quad/plus
            // roll, pitch, and yaw scale is 1000-2000 with 1000 being -1 and 2000 being +1
            // throttle scale is 1000-2000 with 1000 being 0 and 2000 being +1
            servos_raw[0] = (servos_adj[1] - servos_adj[0]) * 0.001f;
            servos_raw[1] = (servos_adj[2] - servos_adj[3]) * 0.001f;
            servos_raw[2] = (servos_adj[0] + servos_adj[1] + servos_adj[2] + servos_adj[3]) * 0.00025f - 1.0f;
            servos_raw[3] = ((servos_adj[0] + servos_adj[1]) * 0.5f - (servos_adj[2] + servos_adj[3]) * 0.5f) * 0.001f;

            Log_Write_SimData(servos_raw[1], servos_raw[0], servos_raw[2], servos_raw[3]);

            // this adds time delay for roll and pitch inputs
            if ((uint16_t)model.time_delay_rp == 0 || is_zero(dt)) {
                for (uint8_t i = 0; i < 2; i++) {
                _servos_delayed_rp[i] = servos_raw[i];
                    }
            } else if (servos_stored_buffer == nullptr) {
                    uint16_t buffer_size = constrain_int16((uint16_t)model.time_delay_rp, 1, 100) * 0.001f / dt;
                    servos_stored_buffer = new ObjectBuffer<servos_stored>(buffer_size);
                    while (servos_stored_buffer->space() != 0) {
                        push_to_buffer_rp(servos_raw);
                    }
                    for (uint8_t i = 0; i < 2; i++) {
                        _servos_delayed_rp[i] = servos_raw[i];
                    }
            } else {
                    pull_from_buffer_rp(_servos_delayed_rp);
                    push_to_buffer_rp(servos_raw);
            }

            // this adds time delay for throttle inputs
            if ((uint16_t)model.time_delay_col == 0 || is_zero(dt)) {
                _servo_delayed_col = servos_raw[2];
            } else if (servo_stored_col_buffer == nullptr) {
                    uint16_t buffer_size = constrain_int16((uint16_t)model.time_delay_col, 1, 100) * 0.001f / dt;
                    servo_stored_col_buffer = new ObjectBuffer<servo_stored>(buffer_size);
                    while (servo_stored_col_buffer->space() != 0) {
                        push_to_buffer_col(servos_raw);
                    }
                    _servo_delayed_col = servos_raw[2];
            } else {
                    pull_from_buffer_col(_servo_delayed_col);
                    push_to_buffer_col(servos_raw);
            }

            // this adds time delay for pedal inputs
            if ((uint16_t)model.time_delay_ped == 0 || is_zero(dt)) {
                _servo_delayed_ped = servos_raw[3];
            } else if (servo_stored_ped_buffer == nullptr) {
                    uint16_t buffer_size = constrain_int16((uint16_t)model.time_delay_ped, 1, 100) * 0.001f / dt;
                    servo_stored_ped_buffer = new ObjectBuffer<servo_stored>(buffer_size);
                    while (servo_stored_ped_buffer->space() != 0) {
                        push_to_buffer_ped(servos_raw);
                    }
                    _servo_delayed_ped = servos_raw[3];
            } else {
                    pull_from_buffer_ped(_servo_delayed_ped);
                    push_to_buffer_ped(servos_raw);
            }

            // calculated motors class inputs from delayed servo_out for state space model input
            float _roll_in = _servos_delayed_rp[0];  // roll
            float _pitch_in = _servos_delayed_rp[1];  //pitch
            float _throttle_in = _servo_delayed_col;  // throttle
            float _yaw_in  = _servo_delayed_ped;  // yaw

            if (is_zero(_throttle_in)) {
            _pitch_in = 0;
            _roll_in = 0;
            _yaw_in = 0;
            }

            static float Dlonlag;
            static float Dlatlag;
            static float Dcollag;
            static float Dpedlag;

            // rotational acceleration (in rad/s/s?) in body frame
            rot_accel.x = (model.Lv)*(velocity_air_bf.y)+(model.Llat)*(Dlatlag)+(model.Lp)*(gyro.x);
            rot_accel.y = (model.Mu)*(velocity_air_bf.x)+(model.Mlon)*(Dlonlag)+(model.Mq)*(gyro.y);
            rot_accel.z = (model.Nr)*(gyro.z)+((model.Nped)-(model.Lag*model.Lead))*(Dpedlag)+(model.Lag*model.Lead)*(_yaw_in);

            float lateral_y_thrust = (model.Yv)*(velocity_air_bf.y)+(model.Ylat)*(Dlatlag);
            float lateral_x_thrust = (model.Xu)*(velocity_air_bf.x)+(model.Xlon)*Dlonlag;
            float thrust = model.Zcol * (Dcollag - model.thr_hover) - GRAVITY_MSS + (model.Zw * velocity_air_bf.z);
            accel_body = Vector3f(lateral_x_thrust, lateral_y_thrust, thrust);

            float Dlatlag_dot = (-model.Lag)*(Dlatlag)+(model.Lag)*(_roll_in);
            float Dlonlag_dot = (-model.Lag)*(Dlonlag)+(model.Lag)*(_pitch_in);
            float Dcollag_dot = (-model.Lag)*(Dcollag)+(model.Lag)*(_throttle_in);
            float Dpedlag_dot = (-model.Lag)*(Dpedlag)+(model.Lag)*(_yaw_in);

            Dlonlag += Dlonlag_dot * dt;
            Dlatlag += Dlatlag_dot * dt;
            Dcollag += Dcollag_dot * dt;
            Dpedlag += Dpedlag_dot * dt;

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

void StateSpace::update_rotor_dynamics(Vector3f gyros, Vector2f ctrl_pos, Vector2f &tpp_angle, float dt)
{

    float tf_inv = 1.0f/model.tf;

    float b1s_dot = -1 * gyro.x - tf_inv * tpp_angle.x + tf_inv * (model.Lfa1s * tpp_angle.y + model.Lflt * ctrl_pos.x + model.Lflg * ctrl_pos.y);
    float a1s_dot = -1 * gyro.y - tf_inv * tpp_angle.y + tf_inv * (model.Mfb1s * tpp_angle.x + model.Mflt * ctrl_pos.x + model.Mflg * ctrl_pos.y);

    tpp_angle.x += b1s_dot * dt;
    tpp_angle.y += a1s_dot * dt;

}

float StateSpace::update_rpm(float curr_rpm, float throttle, float nom_rpm, float dt)
{
    static float rotor_runup_output;
    static uint8_t motor_status;
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
        }else{
            if (rotor_runup_output > 0.0f) {
                rotor_runup_output -= runup_increment; // make ramp down 10 times faster
            } else {
                rotor_runup_output = 0.0f;
            }
        }

    curr_rpm = rotor_runup_output * nom_rpm;

    return curr_rpm;

}

// push servo input to buffer
void StateSpace::push_to_buffer_rp(const float servos_input[16])
{
    servos_stored sample;
    sample.servo1 = servos_input[0];
    sample.servo2 = servos_input[1];

    servos_stored_buffer->push(sample);

}

// pull servo delay from buffer
void StateSpace::pull_from_buffer_rp(float servos_delayed[3])
{
    servos_stored sample;
    if (!servos_stored_buffer->pop(sample)) {
        // no sample
        return;
    }
    servos_delayed[0] = sample.servo1;
    servos_delayed[1] = sample.servo2;

}

// push servo input to buffer COL
void StateSpace::push_to_buffer_col(const float servos_input[16])
{
    servo_stored sample;
    sample.servo = servos_input[2];

    servo_stored_col_buffer->push(sample);

}

// pull servo delay from buffer COL
void StateSpace::pull_from_buffer_col(float &servo_delayed)
{
    servo_stored sample;
    if (!servo_stored_col_buffer->pop(sample)) {
        // no sample
        return;
    }
    servo_delayed = sample.servo;

}

// push servo input to buffer PED
void StateSpace::push_to_buffer_ped(const float servos_input[16])
{
    servo_stored sample;
    sample.servo = servos_input[3];

    servo_stored_ped_buffer->push(sample);

}

// pull servo delay from buffer PED
void StateSpace::pull_from_buffer_ped(float &servo_delayed)
{
    servo_stored sample;
    if (!servo_stored_ped_buffer->pop(sample)) {
        // no sample
        return;
    }
    servo_delayed = sample.servo;
}

/*
  load frame specific parameters from a json file if available
 */
void StateSpace::load_frame_params(const char *model_json)
{
    char *fname = nullptr;
    struct stat st;
    if (AP::FS().stat(model_json, &st) == 0) {
        fname = strdup(model_json);
    } else {
        IGNORE_RETURN(asprintf(&fname, "@ROMFS/models/%s", model_json));
        if (AP::FS().stat(model_json, &st) != 0) {
            AP_HAL::panic("%s failed to load", model_json);
        }
    }
    if (fname == nullptr) {
        AP_HAL::panic("%s failed to load", model_json);
    }
    AP_JSON::value *obj = AP_JSON::load_json(model_json);
    if (obj == nullptr) {
        AP_HAL::panic("%s failed to load", model_json);
    }

    enum class VarType {
        FLOAT,
        UINT16_T
    };

    struct json_search {
        const char *label;
        void *ptr;
        VarType t;
    };

    json_search vars[] = {
    #define FRAME_VAR(s) { #s, &model.s, VarType::FLOAT }
        // common 6DOF state space derivatives
        FRAME_VAR(Lu),
        FRAME_VAR(Lv),
        FRAME_VAR(Lp),
        FRAME_VAR(Lr),
        FRAME_VAR(Mu),
        FRAME_VAR(Mv),
        FRAME_VAR(Mq),
        FRAME_VAR(Nr),
        FRAME_VAR(Nw),
        FRAME_VAR(Nv),
        FRAME_VAR(Xu),
        FRAME_VAR(Yv),
        FRAME_VAR(Yp),
        FRAME_VAR(Zw),
        FRAME_VAR(Yr),
        FRAME_VAR(Xlon),
        FRAME_VAR(Ylat),
        FRAME_VAR(Zcol),
        FRAME_VAR(Mlon),
        FRAME_VAR(Mlat),
        FRAME_VAR(Llon),
        FRAME_VAR(Llat),
        FRAME_VAR(Nped),
        FRAME_VAR(Ncol),
        FRAME_VAR(time_delay_rp),
        FRAME_VAR(time_delay_ped),
        FRAME_VAR(time_delay_col),

        // Multi specific derivatives
        FRAME_VAR(Lag),
        FRAME_VAR(Lead),
        FRAME_VAR(thr_hover),

        // Heli specific derivatives
        FRAME_VAR(Lb1s),
        FRAME_VAR(Ma1s),
        FRAME_VAR(tf),
        FRAME_VAR(Lfa1s),
        FRAME_VAR(Mfb1s),
        FRAME_VAR(Lflt),
        FRAME_VAR(Lflg),
        FRAME_VAR(Mflt),
        FRAME_VAR(Mflg),
        FRAME_VAR(hover_lean),
        FRAME_VAR(nominal_rpm),
        FRAME_VAR(coll_max),
        FRAME_VAR(coll_min),
        FRAME_VAR(coll_hover),
    };

    for (uint8_t i=0; i<ARRAY_SIZE(vars); i++) {
        auto v = obj->get(vars[i].label);
        if (v.is<AP_JSON::null>()) {
            // use default value
            continue;
        }
        if (vars[i].t == VarType::FLOAT) {
            parse_float(v, vars[i].label, *((float *)vars[i].ptr));
        }
    }

    delete obj;

    ::printf("Loaded model params from %s\n", model_json);
}

void StateSpace::parse_float(AP_JSON::value val, const char* label, float &param) {
    if (!val.is<double>()) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    param = val.get<double>();
}

void StateSpace::Log_Write_SimData(float dlong, float dlat, float dthr, float dped)
{
    // @LoggerMessage: SIMD
    // @Description: Sim data packet
    // @Vehicles: Copter
    // @Field: TimeUS: Time since system startup
    // @Field: dlng: longitudinal input
    // @Field: dlat: lateral input
    // @Field: dthr: throttle input
    // @Field: dped: pedal input
    AP::logger().WriteStreaming(
        "SIMD",
        "TimeUS,dlng,dlat,dthr,dped",
        "s----",
        "F0000",
        "Qffff",
        AP_HAL::micros64(),
        dlong,
        dlat,
        dthr,
        dped);
}
} // namespace SITL
