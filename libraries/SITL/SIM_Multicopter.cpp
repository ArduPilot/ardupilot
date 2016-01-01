/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  multicopter simulator class
*/

#include "SIM_Multicopter.h"

#include <stdio.h>

using namespace SITL;

static const Motor quad_plus_motors[4] =
{
    Motor(90,  false,  1),
    Motor(270, false,  2),
    Motor(0,   true,   3),
    Motor(180, true,   4)
};

static const Motor quad_x_motors[4] =
{
    Motor(45,  false,  1),
    Motor(225, false,  2),
    Motor(315, true,   3),
    Motor(135, true,   4)
};

static const Motor hexa_motors[6] =
{
    Motor(60,   false, 1),
    Motor(60,   true,  7),
    Motor(180,  true,  4),
    Motor(180,  false, 8),
    Motor(-60,  true,  2),
    Motor(-60,  false, 3),
};

static const Motor hexax_motors[6] =
{
    Motor(30,  false,  7),
    Motor(90,  true,   1),
    Motor(150, false,  4),
    Motor(210, true,   8),
    Motor(270, false,  2),
    Motor(330, true,   3)
};

static const Motor octa_motors[8] =
{
    Motor(0,    true,  1),
    Motor(180,  true,  2),
    Motor(45,   false, 3),
    Motor(135,  false, 4),
    Motor(-45,  false, 5),
    Motor(-135, false, 6),
    Motor(270,  true,  7),
    Motor(90,   true,  8)
};

static const Motor octa_quad_motors[8] =
{
    Motor(  45, false, 1),
    Motor( -45, true,  2),
    Motor(-135, false, 3),
    Motor( 135, true,  4),
    Motor( -45, false, 5),
    Motor(  45, true,  6),
    Motor( 135, false, 7),
    Motor(-135, true,  8)
};

/*
  table of supported frame types
 */
static Frame supported_frames[] =
{
    Frame("+",         4, quad_plus_motors),
    Frame("quad",      4, quad_plus_motors),
    Frame("copter",    4, quad_plus_motors),
    Frame("x",         4, quad_x_motors),
    Frame("hexa",      6, hexa_motors),
    Frame("hexax",     6, hexax_motors),
    Frame("octa",      8, octa_motors),
    Frame("octa-quad", 8, octa_quad_motors)
};

void Frame::init(float _mass, float hover_throttle, float _terminal_velocity, float _terminal_rotation_rate)
{
    mass = _mass;

    /*
       scaling from total motor power to Newtons. Allows the copter
       to hover against gravity when each motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / (num_motors * hover_throttle);

    terminal_velocity = _terminal_velocity;
    terminal_rotation_rate = _terminal_rotation_rate;
}

MultiCopter::MultiCopter(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    frame(NULL)
{
    for (uint8_t i=0; i < ARRAY_SIZE(supported_frames); i++) {
        if (strcasecmp(frame_str, supported_frames[i].name) == 0) {
            frame = &supported_frames[i];
        }
    }
    if (frame == NULL) {
        printf("Frame '%s' not found", frame_str);
        exit(1);
    }
    frame->init(1.5, 0.51, 15, 4*radians(360));
    frame_height = 0.1;
}

// calculate rotational and linear accelerations
void Frame::calculate_forces(const Aircraft &aircraft,
                             const Aircraft::sitl_input &input,
                             Vector3f &rot_accel,
                             Vector3f &body_accel)
{
    float motor_speed[num_motors];

    for (uint8_t i=0; i<num_motors; i++) {
        uint16_t servo = input.servos[motors[i].servo-1];
        // assume 1000 to 2000 PWM range
        if (servo <= 1000) {
            motor_speed[i] = 0;
        } else {
            motor_speed[i] = (servo-1000) / 1000.0f;
        }
    }

    // rotational acceleration, in rad/s/s, in body frame
    float thrust = 0.0f; // newtons

    for (uint8_t i=0; i<num_motors; i++) {
        rot_accel.x  += -radians(5000.0) * sinf(radians(motors[i].angle)) * motor_speed[i];
        rot_accel.y  +=  radians(5000.0) * cosf(radians(motors[i].angle)) * motor_speed[i];
        if (motors[i].clockwise) {
            rot_accel.z -= motor_speed[i] * radians(400.0);
        } else {
            rot_accel.z += motor_speed[i] * radians(400.0);
        }
        thrust += motor_speed[i] * thrust_scale; // newtons
    }

    // rotational air resistance
    const Vector3f &gyro = aircraft.get_gyro();
    rot_accel.x -= gyro.x * radians(400.0) / terminal_rotation_rate;
    rot_accel.y -= gyro.y * radians(400.0) / terminal_rotation_rate;
    rot_accel.z -= gyro.z * radians(400.0) / terminal_rotation_rate;

    // air resistance
    Vector3f air_resistance = -aircraft.get_velocity_ef() * (GRAVITY_MSS/terminal_velocity);

    body_accel = Vector3f(0, 0, -thrust / mass);
    body_accel += aircraft.get_dcm().transposed() * air_resistance;    

    // add some noise
    const float gyro_noise = radians(0.1);
    const float accel_noise = 0.3;
    const float noise_scale = thrust / (thrust_scale * num_motors);
    rot_accel += Vector3f(aircraft.rand_normal(0, 1),
                          aircraft.rand_normal(0, 1),
                          aircraft.rand_normal(0, 1)) * gyro_noise * noise_scale;
    body_accel += Vector3f(aircraft.rand_normal(0, 1),
                           aircraft.rand_normal(0, 1),
                           aircraft.rand_normal(0, 1)) * accel_noise * noise_scale;
}

// calculate rotational and linear accelerations
void MultiCopter::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    frame->calculate_forces(*this, input, rot_accel, body_accel);
}
    
/*
  update the multicopter simulation by one time step
 */
void MultiCopter::update(const struct sitl_input &input)
{
    // how much time has passed?
    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);

    update_dynamics(rot_accel);

    if (on_ground(position)) {
        // zero roll/pitch, but keep yaw
        float r, p, y;
        dcm.to_euler(&r, &p, &y);
        dcm.from_euler(0, 0, y);

        position.z = -(ground_level + frame_height - home.alt*0.01f);
    }
    
    // update lat/lon/altitude
    update_position();
}


