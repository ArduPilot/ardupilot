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

#include <Filter/LowPassFilter2p.h>
#include <Filter/LowPassFilter.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Param/AP_Param.h>


class ServoModel {
public:
    float filter_angle(uint16_t pwm, float dt);
    float filter_range(uint16_t pwm, float dt);
    void set_pwm_range(uint16_t _pwm_min, uint16_t _pwm_max);
    void set_deflection(float _angle_deg);

    static const struct AP_Param::GroupInfo var_info[];

private:
    float apply_filter(float v, float dt);
    float apply_simple_filter(float v, float dt);

    LowPassFilter2pFloat filter;
    LowPassFilterFloat filter1p;
    ObjectBuffer<float> *delay;
    float last_v;
    float delta_max;
    uint16_t pwm_min = 1000;
    uint16_t pwm_max = 2000;
    float angle_deg = 45.0;
};

