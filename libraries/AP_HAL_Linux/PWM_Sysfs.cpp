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

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <AP_Math/AP_Math.h>

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace Linux {

/* Trick to use minimum stack space for each of the params */
union pwm_params {
    char export_gpio[sizeof("XXX/export")];
    char duty_cycle[sizeof("XXX/pwmXXX/duty_cycle")];
    char enable[sizeof("XXX/pwmXXX/enable")];
    char period[sizeof("XXX/pwmXXX/period")];
    char polarity[sizeof("XXX/pwmXXX/polarity")];
};
#define PWM_BASE_PATH "/sys/class/pwm/pwmchip"
#define PWM_PATH_MAX (sizeof(PWM_BASE_PATH) + sizeof(pwm_params) - 1)

PWM_Sysfs::PWM_Sysfs(uint8_t chip, uint8_t channel)
    : _chip(chip)
    , _channel(channel)
    , _duty_cycle_fd(-1)
    , _nsec_duty_cycle_value(0)
{
    char path[PWM_PATH_MAX];
    int r;

    r = snprintf(path, sizeof(path), PWM_BASE_PATH "%u/export", _chip);
    if (r < 0 || r >= (int)sizeof(path)) {
        AP_HAL::panic("LinuxPWM_Sysfs: chip=%u channel=%u "
                                "Error formatting pwm export: %s",
                                _chip, _channel, strerror(errno));
    }

    /*
     * Not checking for write errors here because if the PWM channel was
     * already exported, the write will return a EBUSY and we can catch up
     * any other error when trying to open the duty_cycle file descriptor.
     */
    Util::from(hal.util)->write_file(path, "%u", _channel);

    r = snprintf(path, sizeof(path), PWM_BASE_PATH "%u/pwm%u/duty_cycle",
                 _chip, _channel);
    if (r < 0 || r >= (int)sizeof(path)) {
        AP_HAL::panic("LinuxPWM_Sysfs: chip=%u channel=%u "
                             "Error formatting channel duty cycle: %s",
                             _chip, _channel, strerror(errno));
    }

    _duty_cycle_fd = ::open(path, O_RDWR | O_CLOEXEC);
    if (_duty_cycle_fd < 0) {
        AP_HAL::panic("LinuxPWM_Sysfs: chip=%u channel=%u "
                             "Unable to open file %s: %s",
                             _chip, _channel, path, strerror(errno));
    }
}

PWM_Sysfs::~PWM_Sysfs()
{
    if (_duty_cycle_fd >= 0) {
        ::close(_duty_cycle_fd);
    }
}

void PWM_Sysfs::enable(bool value)
{
    char path[PWM_PATH_MAX];

    int r = snprintf(path, sizeof(path), PWM_BASE_PATH "%u/pwm%u/enable",
                     _chip, _channel);
    if (r < 0 || r >= (int)sizeof(path)
        || Util::from(hal.util)->write_file(path, "%u", value) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: chip=%u channel=%u "
                            "Unable to %s\n",
                            _chip, _channel,
                            value ? "enable" : "disable");
    }
}

bool PWM_Sysfs::is_enabled()
{
    char path[PWM_PATH_MAX];
    unsigned int enabled;

    int r = snprintf(path, sizeof(path), PWM_BASE_PATH "%u/pwm%u/enable",
                     _chip, _channel);
    if (r < 0 || r >= (int)sizeof(path)
        || Util::from(hal.util)->read_file(path, "%u", &enabled) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: chip=%u channel=%u "
                            "Unable to get status\n",
                            _chip, _channel);
    }

    return enabled;
}

void PWM_Sysfs::set_period(uint32_t nsec_period)
{
    char path[PWM_PATH_MAX];

    int r = snprintf(path, sizeof(path), PWM_BASE_PATH "%u/pwm%u/period",
                     _chip, _channel);
    if (r < 0 || r >= (int)sizeof(path)
        || Util::from(hal.util)->write_file(path, "%u", nsec_period) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: chip=%u channel=%u "
                            "Unable to set period\n",
                            _chip, _channel);
    }
}

uint32_t PWM_Sysfs::get_period()
{
    char path[PWM_PATH_MAX];
    uint32_t nsec_period;

    int r = snprintf(path, sizeof(path), PWM_BASE_PATH "%u/pwm%u/period",
                     _chip, _channel);
    if (r < 0 || r >= (int)sizeof(path)
        || Util::from(hal.util)->read_file(path, "%u", &nsec_period) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: chip=%u channel=%u "
                            "Unable to get period\n",
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

bool PWM_Sysfs::set_duty_cycle(uint32_t nsec_duty_cycle)
{
    /* Don't log fails since this could spam the console */
    if (dprintf(_duty_cycle_fd, "%u", nsec_duty_cycle) < 0) {
        return false;
    }

    _nsec_duty_cycle_value = nsec_duty_cycle;
    return true;
}

uint32_t PWM_Sysfs::get_duty_cycle()
{
    return _nsec_duty_cycle_value;
}

void PWM_Sysfs::set_polarity(PWM_Sysfs::Polarity polarity)
{
    char path[PWM_PATH_MAX];

    int r = snprintf(path, sizeof(path), PWM_BASE_PATH "%u/pwm%u/polarity",
                     _chip, _channel);

    if (r < 0 || r >= (int)sizeof(path)
        || Util::from(hal.util)->write_file(path, "%s", polarity == NORMAL ? "normal" : "inversed") < 0) {
        hal.console->printf("LinuxPWM_Sysfs: chip=%u channel=%u "
                            "Unable to set polarity\n",
                            _chip, _channel);
    }
}

PWM_Sysfs::Polarity PWM_Sysfs::get_polarity()
{
    char path[PWM_PATH_MAX];
    char polarity[16];

    int r = snprintf(path, sizeof(path), PWM_BASE_PATH "%u/pwm%u/polarity",
                     _chip, _channel);
    if (r < 0 || r >= (int)sizeof(path)
        || Util::from(hal.util)->read_file(path, "%s", polarity) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: chip=%u channel=%u "
                            "Unable to get polarity\n",
                            _chip, _channel);
        return NORMAL;
    }
    return strncmp(polarity, "normal", sizeof(polarity)) ? INVERSE : NORMAL;
}
}

#endif
