/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "PWM_Sysfs.h"

#include <AP_Math/AP_Math.h>

#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define BASE_PWM_PATH "/sys/class/pwm/pwmchip"

/*
 * IMPORTANT: The maximum path that is needed to setup a PWM channel, when using it make
 * sure that this size is enough for your use.
 */
#define PWM_PATH_MAX sizeof(BASE_PWM_PATH "XXX/pwmXXX/duty_cycle")

#define SNPRINTF_CHECK ret < 0 || ret >= (int)PWM_PATH_MAX

PWM_Sysfs::PWM_Sysfs(uint8_t chip, uint8_t channel)
    : _chip(chip)
    , _channel(channel)
    , _duty_cycle_fd(-1)
    , _nsec_duty_cycle_value(0)
{
    char path[PWM_PATH_MAX];

    int ret = snprintf(path, sizeof(path), BASE_PWM_PATH "%d/export", _chip);
    if (SNPRINTF_CHECK) {
        hal.scheduler->panic("LinuxPWM_Sysfs: Error formating channel export "
                             "path of chip=%d channel=%d", _chip, _channel);
    }
    /*
     * Not checking for write errors here because if the PWM channel was
     * already exported, the write will return a EBUSY and we can catch up
     * any other error when trying to open the duty_cycle file descriptor.
     */
    Util::from(hal.util)->write_file(path, "%u", _channel);

    ret = snprintf(path, sizeof(path), BASE_PWM_PATH "%d/pwm%d/duty_cycle", _chip, _channel);
    if (SNPRINTF_CHECK) {
        hal.scheduler->panic("LinuxPWM_Sysfs: Error formating channel duty cycle "
                             "path of chip=%d channel=%d", _chip, _channel);
    }
    _duty_cycle_fd = ::open(path, O_RDWR | O_CLOEXEC);
    if (_duty_cycle_fd < 0) {
        hal.scheduler->panic("LinuxPWM_Sysfs: Unable to open file descriptor of chip=%d channel=%d",
                             _chip, _channel);
    }
}

PWM_Sysfs::~PWM_Sysfs()
{
    if (_duty_cycle_fd >= 0) {
        ::close(_duty_cycle_fd);
    }
}

void PWM_Sysfs::enable(bool enable)
{
    char path[PWM_PATH_MAX];

    int ret = snprintf(path, sizeof(path), BASE_PWM_PATH "%d/pwm%d/enable", _chip, _channel);
    if (SNPRINTF_CHECK || Util::from(hal.util)->write_file(path, "%u", enable) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to %s chip=%d channel=%d\n",
                            enable ? "enable" : "disable", _chip, _channel);
    }
}

bool PWM_Sysfs::is_enabled()
{
    char path[PWM_PATH_MAX];
    uint8_t enabled;

    int ret = snprintf(path, sizeof(path), BASE_PWM_PATH "%d/pwm%d/enable", _chip, _channel);
    if (SNPRINTF_CHECK || Util::from(hal.util)->read_file(path, "%u", &enabled) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to get status of chip=%d channel=%d\n",
                            _chip, _channel);
    }

    return enabled;
}

void PWM_Sysfs::set_period(uint32_t nsec_period)
{
    char path[PWM_PATH_MAX];

    int ret = snprintf(path, sizeof(path), BASE_PWM_PATH "%d/pwm%d/period", _chip, _channel);
    if (SNPRINTF_CHECK || Util::from(hal.util)->write_file(path, "%u", nsec_period) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to set period of chip=%d channel=%d\n",
                            _chip, _channel);
    }
}

uint32_t PWM_Sysfs::get_period()
{
    char path[PWM_PATH_MAX];
    uint32_t nsec_period;

    int ret = snprintf(path, sizeof(path), BASE_PWM_PATH "%d/pwm%d/period", _chip, _channel);
    if (SNPRINTF_CHECK || Util::from(hal.util)->read_file(path, "%u", &nsec_period) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to get period of chip=%d channel=%d\n",
                            _chip, _channel);
        return 0;
    }

    return nsec_period;
}

void PWM_Sysfs::set_freq(uint32_t freq)
{
    set_period(hz_to_nsec(freq));
}

uint32_t PWM_Sysfs::get_freq()
{
    return nsec_to_hz(get_period());
}

void PWM_Sysfs::set_duty_cycle(uint32_t nsec_duty_cycle)
{
    if (dprintf(_duty_cycle_fd, "%u", nsec_duty_cycle) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to set duty cycle of chip=%d channel=%d\n",
                            _chip, _channel);
    } else {
        _nsec_duty_cycle_value = nsec_duty_cycle;
    }
}

uint32_t PWM_Sysfs::get_duty_cycle()
{
    return _nsec_duty_cycle_value;
}

void PWM_Sysfs::set_polarity(PWM_Sysfs::Polarity polarity)
{
    char path[PWM_PATH_MAX];

    int ret = snprintf(path, sizeof(path), BASE_PWM_PATH "%d/pwm%d/polarity", _chip, _channel);
    if (SNPRINTF_CHECK || Util::from(hal.util)->write_file(path, "%s", polarity == NORMAL ? "normal" : "inversed") < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to set polarity of chip=%d channel=%d\n",
                            _chip, _channel);
    }
}

PWM_Sysfs::Polarity PWM_Sysfs::get_polarity()
{
    char path[PWM_PATH_MAX];
    char polarity[16];

    int ret = snprintf(path, sizeof(path), BASE_PWM_PATH "%d/pwm%d/polarity", _chip, _channel);
    if (SNPRINTF_CHECK || Util::from(hal.util)->read_file(path, "%s", &polarity) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to get polarity of chip=%d channel=%d\n",
                            _chip, _channel);
        return NORMAL;
    }
    return strncasecmp("normal", polarity, sizeof("normal")) ? INVERSE : NORMAL;
}

#endif
