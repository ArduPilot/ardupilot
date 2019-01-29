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
 *  AHRS View class - for creating a 2nd view of the vehicle attitude
 *
 */

#include "AP_AHRS_View.h"
#include <stdio.h>

AP_AHRS_View::AP_AHRS_View(AP_AHRS &_ahrs, enum Rotation _rotation, float pitch_trim_deg, bool unspin) :
    rotation(_rotation),
    ahrs(_ahrs)
{
    _unspin = unspin;

    switch (rotation) {
    case ROTATION_NONE:
        y_angle = 0;
        break;
    case ROTATION_PITCH_90:
        y_angle = 90;
        break;
    case ROTATION_PITCH_270:
        y_angle =  270;
        break;
    default:
        AP_HAL::panic("Unsupported AHRS view %u\n", (unsigned)rotation);
    }

    // Add pitch trim
    y_angle = wrap_360(y_angle + pitch_trim_deg);

    rot_view.from_euler(0, radians(y_angle), 0);

    // setup initial state
    update();
}

// update state
void AP_AHRS_View::update(bool skip_ins_update)
{
    rot_body_to_ned = ahrs.get_rotation_body_to_ned();
    gyro = ahrs.get_gyro();

    if (!is_zero(y_angle)) {
        Matrix3f &r = rot_body_to_ned;
        r.transpose();
        r = rot_view * r;
        r.transpose();
        gyro.rotate(rotation);
    }

    rot_body_to_ned.to_euler(&roll, &pitch, &yaw);

    // unspin and recalculate NED rotations
    if (_unspin) {
        unspin_update();
        rot_body_to_ned.from_euler(roll, pitch, yaw);
    }

    roll_sensor  = degrees(roll) * 100;
    pitch_sensor = degrees(pitch) * 100;
    yaw_sensor   = degrees(yaw) * 100;
    if (yaw_sensor < 0) {
        yaw_sensor += 36000;
    }

    ahrs.calc_trig(rot_body_to_ned,
                   trig.cos_roll, trig.cos_pitch, trig.cos_yaw,
                   trig.sin_roll, trig.sin_pitch, trig.sin_yaw);
}

// return a smoothed and corrected gyro vector using the latest ins data (which may not have been consumed by the EKF yet)
Vector3f AP_AHRS_View::get_gyro_latest(void) const
{
    if (_unspin) {
        return gyro;
    }

    Vector3f gyro_latest = ahrs.get_gyro_latest();
    gyro_latest.rotate(rotation);
    return gyro_latest;
}

// rotate a 2D vector from earth frame to body frame
Vector2f AP_AHRS_View::rotate_earth_to_body2D(const Vector2f &ef) const
{
    return Vector2f(ef.x * trig.cos_yaw + ef.y * trig.sin_yaw,
                    -ef.x * trig.sin_yaw + ef.y * trig.cos_yaw);
}

// rotate a 2D vector from earth frame to body frame
Vector2f AP_AHRS_View::rotate_body_to_earth2D(const Vector2f &bf) const
{
    return Vector2f(bf.x * trig.cos_yaw - bf.y * trig.sin_yaw,
                    bf.x * trig.sin_yaw + bf.y * trig.cos_yaw);
}


// Create 'un-spinned' view for all rotating vehicles such as monocopter
void AP_AHRS_View::unspin_update(void)
{
    // buffer index, increment one for each sample
    const uint16_t last_end_index = end_index;
    end_index++;
    if (end_index > BUFFER_LENGTH - 1) {
        end_index = 0;
    }

    // insure buffer does not overlap
    if (end_index == start_index) {
        start_index++;
        if (start_index > BUFFER_LENGTH - 1) {
            start_index = 0;
        }
    }

    // rotate so pitch and roll are relative to fixed 'virtual' yaw
    const float sin_yaw = sinf(yaw - vitual_forward);
    const float cos_yaw = cosf(yaw - vitual_forward);
    virtual_roll[end_index] =  cos_yaw*roll - sin_yaw*pitch;
    virtual_pitch[end_index] = sin_yaw*roll + cos_yaw*pitch;
    virtual_roll_rate[end_index] =  cos_yaw*gyro.x - sin_yaw*gyro.y;
    virtual_pitch_rate[end_index] = sin_yaw*gyro.x + cos_yaw*gyro.y;

    // keep track of rotation angle
    yaw_diff[end_index] = wrap_PI(yaw - true_yaw);
    true_yaw = yaw;
    rotation_angle = wrap_2PI(vitual_forward - true_yaw);

    // keep track of time
    time[end_index] = AP_HAL::micros64();

    // Integrate virtual roll and pitch - trapezium rule
    roll_int[end_index] = (virtual_roll[end_index] + virtual_roll[last_end_index]) * 0.5f * (time[end_index] - time[last_end_index]);
    pitch_int[end_index] = (virtual_pitch[end_index] + virtual_pitch[last_end_index]) * 0.5f * (time[end_index] - time[last_end_index]);
    roll_rate_int[end_index] = (virtual_roll_rate[end_index] + virtual_roll_rate[last_end_index]) * 0.5f * (time[end_index] - time[last_end_index]);
    pitch_rate_int[end_index] = (virtual_pitch_rate[end_index] + virtual_pitch_rate[last_end_index]) * 0.5f * (time[end_index] - time[last_end_index]);

    // add up all since last full rotation
    int16_t i = start_index;
    float yaw_sum = 0.0f;
    float roll_int_sum = 0.0f;
    float pitch_int_sum = 0.0f;
    float roll_rate_int_sum = 0.0f;
    float pitch_rate_int_sum = 0.0f;
    while (i != end_index) {
        yaw_sum += yaw_diff[i];
        roll_int_sum += roll_int[i];
        pitch_int_sum += pitch_int[i];
        roll_rate_int_sum += roll_rate_int[i];
        pitch_rate_int_sum += pitch_rate_int[i];
        i++;
        if (i > BUFFER_LENGTH - 1) {
            i = 0;
        }
    }
    yaw_sum += yaw_diff[end_index];
    roll_int_sum += roll_int[end_index];
    pitch_int_sum += pitch_int[end_index];
    roll_rate_int_sum += roll_rate_int[end_index];
    pitch_rate_int_sum += pitch_rate_int[end_index];

    // take off oldest until less than 360
    while (abs(yaw_sum - yaw_diff[start_index]) > M_2PI) {
        yaw_sum -= yaw_diff[start_index];
        roll_int_sum -= roll_int[start_index];
        pitch_int_sum -= pitch_int[start_index];
        roll_rate_int_sum -= roll_rate_int[start_index];
        pitch_rate_int_sum -= pitch_rate_int[start_index];
        start_index++;
        if (start_index > BUFFER_LENGTH - 1) {
            start_index = 0;
        }
    }

    // Find time exactly 1 rotation ago, provided one rotation has past
    // this allows rolling integration for exactly one rotation
    const float yaw_sum_less_one = abs(yaw_sum - yaw_diff[start_index]);
    yaw_sum = abs(yaw_sum);
    rpm = 0.0f;
    if (yaw_sum > M_2PI) {
        // match virtual yaw rate to desired
        vitual_forward += yaw_rate * (time[end_index] - time[last_end_index]) * 0.000001f;
        vitual_forward = wrap_2PI(vitual_forward);
        yaw = vitual_forward;
        gyro.z = yaw_rate;

        // Calculate Interpolation factors
        const float interp_0 = (yaw_sum_less_one - M_2PI)/(yaw_sum_less_one - yaw_sum);
        const float interp_1 = 1 - interp_0;

        // Error caused by misalignment of arrays for values and difference/integrals
        // i.e. diff array one element shorter than values
        int16_t start_index_a = start_index - 1;
        if (start_index_a < 0) {
            start_index_a = BUFFER_LENGTH - 1;
        }

        // Interpolation time, roll and pitch exactly one rotation ago
        const float Cross_time = time[start_index_a] * interp_0 + time[start_index] * interp_1;
        const float Cross_roll = virtual_roll[start_index_a] * interp_0 + virtual_roll[start_index] * interp_1;
        const float Cross_pitch = virtual_pitch[start_index_a] * interp_0 + virtual_pitch[start_index] * interp_1;
        const float Cross_roll_rate = virtual_roll_rate[start_index_a] * interp_0 + virtual_roll_rate[start_index] * interp_1;
        const float Cross_pitch_rate = virtual_pitch_rate[start_index_a] * interp_0 + virtual_pitch_rate[start_index] * interp_1;

        // Trapezium rule integration from cross point to next point
        const float Cross_roll_int = (Cross_roll + virtual_roll[start_index]) * 0.5f * (time[start_index] - Cross_time);
        const float Cross_pitch_int = (Cross_pitch + virtual_pitch[start_index]) * 0.5f * (time[start_index] - Cross_time);
        const float Cross_roll_rate_int = (Cross_roll_rate + virtual_roll_rate[start_index]) * 0.5f * (time[start_index] - Cross_time);
        const float Cross_pitch_rate_int = (Cross_pitch_rate + virtual_pitch_rate[start_index]) * 0.5f * (time[start_index] - Cross_time);

        // rotation time
        const float rotation_time = time[end_index] - Cross_time;
        rpm = 60000000.0f / rotation_time;

        // Calculate total integrated roll and pitch and divide by rotation time
        roll = (roll_int_sum - roll_int[start_index] + Cross_roll_int) / rotation_time;
        pitch = (pitch_int_sum - pitch_int[start_index] + Cross_pitch_int) / rotation_time;
        gyro.x = (roll_rate_int_sum - roll_rate_int[start_index] + Cross_roll_rate_int) / rotation_time;
        gyro.y = (pitch_rate_int_sum - pitch_rate_int[start_index] + Cross_pitch_rate_int) / rotation_time;

        roll = wrap_PI(roll);
        pitch = wrap_PI(pitch);

    } else if (abs(gyro.z) < 0.01f) {
        // if were not in rotation 'lock' and not rotating then set current yaw as the new 'virtual forward' direction
        vitual_forward = yaw;
    }
}
