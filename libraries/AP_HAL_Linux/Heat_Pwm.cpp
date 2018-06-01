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

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/limits.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "Heat_Pwm.h"

extern const AP_HAL::HAL& hal;

#define HEAT_PWM_DUTY "duty_ns"
#define HEAT_PWM_PERIOD "period_ns"
#define HEAT_PWM_RUN "run"

using namespace Linux;

/*
 * Constructor :
 * argument : pwm_sysfs_path is the path to the pwm directory,
 * i.e /sys/class/pwm/pwm_6 on the bebop
 */
HeatPwm::HeatPwm(const char* pwm_sysfs_path, float Kp, float Ki, uint32_t period_ns, float target) :
    _Kp(Kp),
    _Ki(Ki),
    _period_ns(period_ns),
    _target(target)
{
    char *duty_path;
    char *period_path;
    char *run_path;

    if (asprintf(&duty_path, "%s/%s", pwm_sysfs_path, HEAT_PWM_DUTY) == -1) {
        hal.scheduler->panic("HeatPwm not enough memory\n");
    }
    _duty_fd = open(duty_path, O_RDWR);
    if (_duty_fd == -1) {
        perror("opening duty");
        hal.scheduler->panic("Error Initializing Pwm heat\n");
    }
    free(duty_path);

    if (asprintf(&period_path, "%s/%s", pwm_sysfs_path, HEAT_PWM_PERIOD) == -1) {
        hal.scheduler->panic("HeatPwm not enough memory\n");
    }
    _period_fd = open(period_path, O_RDWR);
    if (_period_fd == -1) {
        perror("opening period");
        hal.scheduler->panic("Error Initializing Pwm heat\n");
    }
    free(period_path);

    if (asprintf(&run_path, "%s/%s", pwm_sysfs_path, HEAT_PWM_RUN) == -1) {
        hal.scheduler->panic("HeatPwm not enough memory\n");
    }
    _run_fd = open(run_path, O_RDWR);
    if (_run_fd == -1) {
        perror("opening run");
        hal.scheduler->panic("Error Initializing Pwm heat\n");
    }
    free(run_path);

    _set_period(_period_ns);
    _set_duty(0);
    _set_run();
}

void HeatPwm::set_imu_temp(float current)
{
    float error, output;

    if (hal.scheduler->millis() - _last_temp_update < 5) {
        return;
    }

    /* minimal PI algo without dt */
    error = _target - current;
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

    _set_duty(output);
    _last_temp_update = hal.scheduler->millis();
}

void HeatPwm::_set_duty(uint32_t duty)
{
    if (dprintf(_duty_fd, "0x%x", duty) < 0) {
        perror("pwm set_duty");
    }
}

void HeatPwm::_set_period(uint32_t period)
{
    if (dprintf(_period_fd, "0x%x", period) < 0) {
        perror("pwm set_period");
    }
}

void HeatPwm::_set_run()
{
    if (dprintf(_run_fd, "1") < 0) {
        perror("pwm set_run");
    }
}

#endif
