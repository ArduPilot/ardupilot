// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#include <cmath>
#include <fcntl.h>
#include <linux/limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "Heat_Pwm.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

HeatPwm::HeatPwm(uint8_t pwm_num, float Kp, float Ki, uint32_t period_ns) :
    _Kp(Kp),
    _Ki(Ki),
    _period_ns(period_ns)
{
    _pwm = new PWM_Sysfs_Bebop(pwm_num);
    _pwm->set_period(_period_ns);
    _pwm->set_duty_cycle(0);
    _pwm->enable(true);
}

void HeatPwm::set_imu_temp(float current)
{
    float error, output;

    if (_target == nullptr) {
        // not configured
        return;
    }
    
    if (AP_HAL::millis() - _last_temp_update < 5) {
        return;
    }

    /* minimal PI algo without dt */
    error = ((float)*_target) - current;
    /* Don't accumulate errors if the integrated error is superior
     * to the max duty cycle(pwm_period)
     */
    if ((fabsf(_sum_error) * _Ki < _period_ns)) {
        _sum_error = _sum_error + error;
    }

    output = _Kp*error + _Ki * _sum_error;

    if (output > _period_ns) {
        output = _period_ns;
    } else if (output < 0) {
        output = 0;
    }

    _pwm->set_duty_cycle(output);
    _last_temp_update = AP_HAL::millis();
    // printf("target %.1f current %.1f out %.2f\n", _target, current, output);
}

void HeatPwm::set_imu_target_temp(int8_t *target)
{
    _target = target;
}

#endif
