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
  multicopter frame simulator class
*/

#include "SIM_Frame.h"
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

static Motor quad_plus_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
};

static Motor quad_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

static Motor tiltquad_h_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, -1, 0, 0, 7, 10, -90),
};

static Motor hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_4,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor hexax_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4)
};

static Motor octa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor octa_quad_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6)
};

static Motor dodeca_hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3),
    Motor(AP_MOTORS_MOT_4,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4),
    Motor(AP_MOTORS_MOT_5,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7),
    Motor(AP_MOTORS_MOT_8, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8),
    Motor(AP_MOTORS_MOT_9,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10),
    Motor(AP_MOTORS_MOT_11, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   11),
    Motor(AP_MOTORS_MOT_12, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  12)
};

static Motor tri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2)
};

static Motor y6_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_6, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3)
};

/*
  FireflyY6 is a Y6 with front motors tiltable using servo on channel 9 (output 8)
 */
static Motor firefly_motors[] =
{
    Motor(AP_MOTORS_MOT_1, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_2,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_6, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6, -1, 0, 0, 6, 0, -90)
};

/*
  table of supported frame types. String order is important for
  partial name matching
 */
static Frame supported_frames[] =
{
    Frame("+",         4, quad_plus_motors),
    Frame("quad",      4, quad_plus_motors),
    Frame("copter",    4, quad_plus_motors),
    Frame("x",         4, quad_x_motors),
    Frame("tilthvec",  4, tiltquad_h_vectored_motors),
    Frame("hexax",     6, hexax_motors),
    Frame("hexa",      6, hexa_motors),
    Frame("octa-quad", 8, octa_quad_motors),
    Frame("octa",      8, octa_motors),
    Frame("dodeca-hexa", 12, dodeca_hexa_motors),
    Frame("tri",       3, tri_motors),
    Frame("tilttrivec",3, tilttri_vectored_motors),
    Frame("tilttri",   3, tilttri_motors),
    Frame("y6",        6, y6_motors),
    Frame("firefly",   6, firefly_motors)
};

void Frame::init(float _mass, float hover_throttle, float _terminal_velocity, float _terminal_rotation_rate)
{
    /*
       scaling from total motor power to Newtons. Allows the copter
       to hover against gravity when each motor is at hover_throttle
    */
    thrust_scale = (_mass * GRAVITY_MSS) / (num_motors * hover_throttle);

    terminal_velocity = _terminal_velocity;
    terminal_rotation_rate = _terminal_rotation_rate;
}

/*
  find a frame by name
 */
Frame *Frame::find_frame(const char *name)
{
    for (uint8_t i=0; i < ARRAY_SIZE(supported_frames); i++) {
        // do partial name matching to allow for frame variants
        if (strncasecmp(name, supported_frames[i].name, strlen(supported_frames[i].name)) == 0) {
            return &supported_frames[i];
        }
    }
    return nullptr;
}

// calculate rotational and linear accelerations
void Frame::calculate_forces(const Aircraft &aircraft,
                             const struct sitl_input &input,
                             Vector3f &rot_accel,
                             Vector3f &body_accel)
{
    Vector3f thrust; // newtons

    for (uint8_t i=0; i<num_motors; i++) {
        Vector3f mraccel, mthrust;
        motors[i].calculate_forces(input, thrust_scale, motor_offset, mraccel, mthrust);
        rot_accel += mraccel;
        thrust += mthrust;
    }

    body_accel = thrust/aircraft.gross_mass();

    if (terminal_rotation_rate > 0) {
        // rotational air resistance
        const Vector3f &gyro = aircraft.get_gyro();
        rot_accel.x -= gyro.x * radians(400.0) / terminal_rotation_rate;
        rot_accel.y -= gyro.y * radians(400.0) / terminal_rotation_rate;
        rot_accel.z -= gyro.z * radians(400.0) / terminal_rotation_rate;
    }

    if (terminal_velocity > 0) {
        // air resistance
        Vector3f air_resistance = -aircraft.get_velocity_air_ef() * (GRAVITY_MSS/terminal_velocity);
        body_accel += aircraft.get_dcm().transposed() * air_resistance;
    }

    // add some noise
    const float gyro_noise = radians(0.1);
    const float accel_noise = 0.3;
    const float noise_scale = thrust.length() / (thrust_scale * num_motors);
    rot_accel += Vector3f(aircraft.rand_normal(0, 1),
                          aircraft.rand_normal(0, 1),
                          aircraft.rand_normal(0, 1)) * gyro_noise * noise_scale;
    body_accel += Vector3f(aircraft.rand_normal(0, 1),
                           aircraft.rand_normal(0, 1),
                           aircraft.rand_normal(0, 1)) * accel_noise * noise_scale;
}


// calculate current and voltage
void Frame::current_and_voltage(const struct sitl_input &input, float &voltage, float &current)
{
    voltage = 0;
    current = 0;
    for (uint8_t i=0; i<num_motors; i++) {
        float c, v;
        motors[i].current_and_voltage(input, v, c, motor_offset);
        current += c;
        voltage += v;
    }
    // use average for voltage, total for current
    voltage /= num_motors;
}
