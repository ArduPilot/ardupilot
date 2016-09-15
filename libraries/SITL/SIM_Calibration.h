/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "SIM_Aircraft.h"

#include <AP_Math/AP_Math.h>

namespace SITL {

/**
 * Simulation model to simulate calibration of accelerometers and compasses.
 *
 * The vehicle rotation can be controlled by sending PWM values to the servos
 * input, denoted by PWM[i] for the i-th channel (starting by zero). All PWM
 * values must be in [1000, 2000], otherwise that will cause undefined
 * behavior.
 *
 * There are three control modes, that are set with PWM[4]:
 *
 *  1) Stop (1000 <= PWM[4] < 1100):
 *    Stop the vehicle, i.e., stop the actuation of the other modes.
 *
 *  2) Attitude (1100 <= PWM[4] < 1200):
 *    Rotate the vehicle to the specified attitude. The attitude is defined
 *    with the PWM channels 5, 6 and 7 for roll, pitch and yaw angles,
 *    respectively. The PWM value for a desired angle in radians is given by:
 *
 *        pwm(theta) = 1500 + 500 * round(theta / pi)
 *        where -pi <= theta <= pi
 *
 *  3) Angular Velocity (1200 <= PWM[4] <= 2000):
 *    Rotate the vehicle at a desired angular velocity. The angular velocity is
 *    specified by a rotation axis and an angular speed.
 *
 *    The x, y and z components of the rotation axis is given, respectively, by
 *    the PWM channels 5, 6 and 7 with an offset of 1500. The rotation axis is
 *    normalized internally, so that PWM[5,6,7] = [1600, 1300, 0] and
 *    PWM[5,6,7] = [1700, 1100, 0] means the same normalized rotation axis.
 *
 *    The angular speed value is specified by PWM[4]. The PWM value for a
 *    desired angular speed in radians/s is given by:
 *
 *        pwm(theta) = 1200 + 800 * round(theta / (2 * pi)),
 *        where 0 <= theta <= 2 * pi
 */
class Calibration : public Aircraft {
public:
    Calibration(const char *home_str, const char *frame_str);

    void update(const struct sitl_input& input);

    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Calibration(home_str, frame_str);
    }

private:
    void _stop_control(const struct sitl_input& input, Vector3f& rot_accel);

    void _attitude_set(float desired_roll, float desired_pitch, float desired_yaw,
                       Vector3f& rot_accel);

    void _attitude_control(const struct sitl_input& input,
                           Vector3f& rot_accel);

    void _angular_velocity_control(const struct sitl_input& input,
                                   Vector3f& rot_accel);

    void _calibration_poses(Vector3f& rot_accel);
};
}
