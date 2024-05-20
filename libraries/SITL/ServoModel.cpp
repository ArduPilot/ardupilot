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
  simple model of a servo. Model is:

    - time delay for transport protocol delay
    - slew limit
    - 2-pole butterworth
*/

#include "ServoModel.h"
#include "SITL.h"

// SITL servo model parameters
const AP_Param::GroupInfo SITL::SIM::ServoParams::var_info[] = {
    // @Param: SPEED
    // @DisplayName: servo speed
    // @Description: servo speed (time for 60 degree deflection). If DELAY and FILTER are not set then this is converted to a 1p lowpass filter. If DELAY or FILTER are set then this is treated as a rate of change limit
    // @Units: s
    AP_GROUPINFO("SPEED",     1, ServoParams,  servo_speed, 0.14),

    // @Param: DELAY
    // @DisplayName: servo delay
    // @Description: servo delay
    // @Units: s
    AP_GROUPINFO("DELAY",     2, ServoParams,  servo_delay, 0.0),

    // @Param: FILTER
    // @DisplayName: servo filter
    // @Description: servo filter
    // @Units: Hz
    AP_GROUPINFO("FILTER",    3, ServoParams,  servo_filter, 0),
    
    AP_GROUPEND
};

/*
  simpler filter used when SIM_SERVO_FILTER and SIM_SERVO_DELAY are not set
  this filter is a 1p low pass based on SIM_SERVO_SPEED
 */
float ServoModel::apply_simple_filter(float v, float dt)
{
    const auto *sitl = AP::sitl();
    if (!is_positive(sitl->servo.servo_speed)) {
        return v;
    }
    const float cutoff = 1.0f / (2 * M_PI * sitl->servo.servo_speed);
    filter1p.set_cutoff_frequency(cutoff);
    return filter1p.apply(v, dt);
}

/*
  apply a filter for a servo model consistting of a delay, speed and 2p filter
 */
float ServoModel::apply_filter(float v, float dt)
{
    const auto *sitl = AP::sitl();
    if (!sitl) {
        return v;
    }

    if (!is_positive(sitl->servo.servo_delay) &&
        !is_positive(sitl->servo.servo_filter)) {
        // fallback to a simpler 1p filter model
        return apply_simple_filter(v, dt);
    }

    // apply delay
    if (sitl->servo.servo_delay > 0) {
        uint32_t delay_len = MAX(1,sitl->servo.servo_delay * sitl->loop_rate_hz);
        if (!delay) {
            delay = new ObjectBuffer<float>();
        }
        if (delay->get_size() != delay_len) {
            delay->set_size(delay_len);
        }
        while (delay->space() > 0) {
            delay->push(v);
        }
        IGNORE_RETURN(delay->pop(v));
    }

    // apply slew limit
    if (sitl->servo.servo_speed > 0) {
        // assume SIM_SERVO_SPEED is time for 60 degrees
        float time_per_degree = sitl->servo.servo_speed / 60.0;
        float proportion_per_second = 1.0 / (angle_deg * time_per_degree);
        delta_max = proportion_per_second * dt;
        v = constrain_float(v, last_v-delta_max, last_v+delta_max);
        v = constrain_float(v, -1, 1);
        last_v = v;
    }

    // apply filter
    if (sitl->servo.servo_filter > 0) {
        filter.set_cutoff_frequency(sitl->loop_rate_hz, sitl->servo.servo_filter);
        v = filter.apply(v);
    }

    return v;
}

float ServoModel::filter_range(uint16_t pwm, float dt)
{
    const float v = (pwm - pwm_min)/float(pwm_max - pwm_min);
    return apply_filter(v, dt);
}

float ServoModel::filter_angle(uint16_t pwm, float dt)
{
    const float v = (pwm - 0.5*(pwm_max+pwm_min))/(0.5*float(pwm_max - pwm_min));
    return apply_filter(v, dt);
}

void ServoModel::set_pwm_range(uint16_t _pwm_min, uint16_t _pwm_max)
{
    pwm_min = _pwm_min;
    pwm_max = _pwm_max;
}

void ServoModel::set_deflection(float _angle_deg)
{
    angle_deg = fabsf(_angle_deg);
}
