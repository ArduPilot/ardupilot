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
#include "PWM_Sysfs.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

namespace Linux {

PWM_Sysfs_Base::PWM_Sysfs_Base(char* export_path, char* polarity_path,
                          char* enable_path, char* duty_path,
                          char* period_path, uint8_t channel)
    : _export_path(export_path)
    , _polarity_path(polarity_path)
    , _enable_path(enable_path)
    , _duty_path(duty_path)
    , _period_path(period_path)
    , _channel(channel)
{
}

PWM_Sysfs_Base::~PWM_Sysfs_Base()
{
    ::close(_duty_cycle_fd);

    free(_polarity_path);
    free(_enable_path);
    free(_period_path);
}

void PWM_Sysfs_Base::init()
{
    if (_export_path == nullptr || _enable_path == nullptr ||
        _period_path == nullptr || _duty_path == nullptr) {
        AP_HAL::panic("PWM_Sysfs: export=%p enable=%p period=%p duty=%p"
                      " required path is NULL", _export_path, _enable_path,
                      _period_path, _duty_path);
    }
    /* Not checking the return of write_file since it will fail if
     * the pwm has already been exported
     */
    Util::from(hal.util)->write_file(_export_path, "%u", _channel);
    free(_export_path);

    _duty_cycle_fd = ::open(_duty_path, O_RDWR | O_CLOEXEC);
    if (_duty_cycle_fd < 0) {
        AP_HAL::panic("LinuxPWM_Sysfs:Unable to open file %s: %s",
                      _duty_path, strerror(errno));
    }
    free(_duty_path);
}

void PWM_Sysfs_Base::enable(bool value)
{
    if (Util::from(hal.util)->write_file(_enable_path, "%u", value) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to %s\n",
                            _enable_path, value ? "enable" : "disable");
    }
}

bool PWM_Sysfs_Base::is_enabled()
{
    unsigned int enabled;

    if (Util::from(hal.util)->read_file(_enable_path, "%u", &enabled) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to get status\n",
                            _enable_path);
    }
    return enabled;
}

void PWM_Sysfs_Base::set_period(uint32_t nsec_period)
{
    set_duty_cycle(0);

    if (Util::from(hal.util)->write_file(_period_path, "%u", nsec_period) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to set period\n",
                            _period_path);
    }
}

uint32_t PWM_Sysfs_Base::get_period()
{
    uint32_t nsec_period;

    if (Util::from(hal.util)->read_file(_period_path, "%u", &nsec_period) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to get period\n",
                            _period_path);
        nsec_period = 0;
    }
    return nsec_period;
}

void PWM_Sysfs_Base::set_freq(uint32_t freq)
{
    set_period(hz_to_nsec(freq));
}

uint32_t PWM_Sysfs_Base::get_freq()
{
    return nsec_to_hz(get_period());
}

bool PWM_Sysfs_Base::set_duty_cycle(uint32_t nsec_duty_cycle)
{
    /* Don't log fails since this could spam the console */
    if (dprintf(_duty_cycle_fd, "%u", nsec_duty_cycle) < 0) {
        return false;
    }

    _nsec_duty_cycle_value = nsec_duty_cycle;
    return true;
}

uint32_t PWM_Sysfs_Base::get_duty_cycle()
{
    return _nsec_duty_cycle_value;
}

void PWM_Sysfs_Base::set_polarity(PWM_Sysfs_Base::Polarity polarity)
{
    if (Util::from(hal.util)->write_file(_polarity_path, "%s",
                                         polarity == NORMAL ?
                                         "normal" : "inversed") < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to set polarity\n",
                            _polarity_path);
    }
}

PWM_Sysfs_Base::Polarity PWM_Sysfs_Base::get_polarity()
{
    char polarity[16];

    if (Util::from(hal.util)->read_file(_polarity_path, "%s", polarity) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: %s Unable to get polarity\n",
                            _polarity_path);
        return NORMAL;
    }
    return strncmp(polarity, "normal", sizeof(polarity)) ? INVERSE : NORMAL;
}

/* PWM Sysfs api for mainline kernel */
char *PWM_Sysfs::_generate_export_path(uint8_t chip)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwmchip%u/export", chip);
    if (r == -1) {
        AP_HAL::panic("LinuxPWM_Sysfs :"
                      "couldn't allocate export path\n");
    }
    return path;
}

char *PWM_Sysfs::_generate_polarity_path(uint8_t chip, uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwmchip%u/pwm%u/polarity",
                     chip, channel);
    if (r == -1) {
        AP_HAL::panic("LinuxPWM_Sysfs :"
                      "couldn't allocate polarity path\n");
    }
    return path;
}

char *PWM_Sysfs::_generate_enable_path(uint8_t chip, uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwmchip%u/pwm%u/enable",
                     chip, channel);
    if (r == -1) {
        AP_HAL::panic("LinuxPWM_Sysfs :"
                      "couldn't allocate enable path\n");
    }
    return path;
}

char *PWM_Sysfs::_generate_duty_path(uint8_t chip, uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwmchip%u/pwm%u/duty_cycle",
                     chip, channel);
    if (r == -1) {
        AP_HAL::panic("LinuxPWM_Sysfs :"
                      "couldn't allocate duty path\n");
    }
    return path;
}

char *PWM_Sysfs::_generate_period_path(uint8_t chip, uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwmchip%u/pwm%u/period",
                     chip, channel);
    if (r == -1) {
        AP_HAL::panic("LinuxPWM_Sysfs :"
                      "couldn't allocate period path\n");
    }
    return path;
}

PWM_Sysfs::PWM_Sysfs(uint8_t chip, uint8_t channel) :
    PWM_Sysfs_Base(_generate_export_path(chip),
                   _generate_polarity_path(chip, channel),
                   _generate_enable_path(chip, channel),
                   _generate_duty_path(chip, channel),
                   _generate_period_path(chip, channel),
                   channel)
{
}

/* PWM Sysfs api for bebop kernel */
char *PWM_Sysfs_Bebop::_generate_export_path()
{
    return strdup("/sys/class/pwm/export");
}

char *PWM_Sysfs_Bebop::_generate_enable_path(uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwm_%u/run",
                     channel);
    if (r == -1) {
        AP_HAL::panic("LinuxPWM_Sysfs :"
                      "couldn't allocate enable path\n");
    }
    return path;
}

char *PWM_Sysfs_Bebop::_generate_duty_path(uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwm_%u/duty_ns",
                     channel);
    if (r == -1) {
        AP_HAL::panic("LinuxPWM_Sysfs :"
                      "couldn't allocate duty path\n");
    }
    return path;
}

char *PWM_Sysfs_Bebop::_generate_period_path(uint8_t channel)
{
    char *path;
    int r = asprintf(&path, "/sys/class/pwm/pwm_%u/period_ns",
                     channel);
    if (r == -1) {
        AP_HAL::panic("LinuxPWM_Sysfs :"
                      "couldn't allocate period path\n");
    }
    return path;
}

PWM_Sysfs_Bebop::PWM_Sysfs_Bebop(uint8_t channel) :
    PWM_Sysfs_Base(_generate_export_path(),
                   nullptr,
                   _generate_enable_path(channel),
                   _generate_duty_path(channel),
                   _generate_period_path(channel),
                   channel)
{
}

}
