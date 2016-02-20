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
#pragma once

#include "AP_HAL_Linux.h"
#include "PWM_Sysfs.h"
#include "Heat.h"

class Linux::HeatPwm : public Linux::Heat {
public:
    HeatPwm(uint8_t pwm_num, float Kp, float Ki,
            uint32_t period_ns, float target);
    void set_imu_temp(float current)override;

private:
    PWM_Sysfs_Base *_pwm;
    uint32_t _last_temp_update = 0;
    float _Kp;
    float _Ki;
    uint32_t _period_ns;
    float _sum_error;
    float _target;
};
