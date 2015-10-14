// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "PWM_Sysfs.h"

#include <fcntl.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define BASE_PWM_PATH "/sys/class/pwm/pwmchip%d"

#define UINT32_STR_MAX 10 + 1

LinuxPWM_Sysfs::LinuxPWM_Sysfs(uint8_t chip, uint8_t channel)
    : _chip(chip)
    , _channel(channel)
    , _duty_cycle_fd(-1)
    , _nsec_duty_cycle_value(0)
{
    char path[PATH_MAX];
    LinuxUtil* util = LinuxUtil::from(hal.util);

    snprintf(path, sizeof(path), BASE_PWM_PATH, _chip);
    if (access(path, F_OK) < 0) {
        hal.scheduler->panic(PSTR("LinuxPWM_Sysfs: PWM chip not found."));
    }

    snprintf(path, sizeof(path), BASE_PWM_PATH "/export", _chip);
    if (util->write_file(path, "%u", _channel) < 1) {
        hal.scheduler->panic(PSTR("LinuxPWM_Sysfs: Unable to export PWM pin."));
    }

    snprintf(path, sizeof(path), BASE_PWM_PATH "/pwm%d/duty_cycle", _chip, _channel);
    _duty_cycle_fd = ::open(path, O_RDWR | O_CLOEXEC);
    if (_duty_cycle_fd < 0) {
        hal.scheduler->panic(PSTR("LinuxPWM_Sysfs: Unable to open file descriptor."));
    }
}

LinuxPWM_Sysfs::~LinuxPWM_Sysfs()
{
    if (_duty_cycle_fd >= 0) {
        ::close(_duty_cycle_fd);
    }
}

void LinuxPWM_Sysfs::enable(bool enable)
{
    char path[PATH_MAX];
    LinuxUtil* util = LinuxUtil::from(hal.util);

    snprintf(path, sizeof(path), BASE_PWM_PATH "/pwm%d/enable", _chip, _channel);
    if (util->write_file(path, "%u", enable) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to %s PWM[%d, %d]\n",
                            enable ? "enable" : "disable", _chip, _channel);
    }
}

bool LinuxPWM_Sysfs::is_enabled()
{
    char path[PATH_MAX];
    LinuxUtil* util = LinuxUtil::from(hal.util);
    uint8_t ret;

    snprintf(path, sizeof(path), BASE_PWM_PATH "/pwm%d/enable", _chip, _channel);
    if (util->read_file(path, "%u", &ret) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to get status of PWM[%d, %d]\n",
                            _chip, _channel);
    }

    return ret;
}

void LinuxPWM_Sysfs::set_period(uint32_t nsec_period)
{
    char path[PATH_MAX];
    LinuxUtil* util = LinuxUtil::from(hal.util);

    snprintf(path, sizeof(path), BASE_PWM_PATH "/pwm%d/period", _chip, _channel);
    if (util->write_file(path, "%u", nsec_period) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to set period of PWM[%d, %d]\n",
                            _chip, _channel);
    }
}

uint32_t LinuxPWM_Sysfs::get_period()
{
    char path[PATH_MAX];
    LinuxUtil* util = LinuxUtil::from(hal.util);
    uint32_t nsec_period;

    snprintf(path, sizeof(path), BASE_PWM_PATH "/pwm%d/period", _chip, _channel);
    if (util->read_file(path, "%u", &nsec_period) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to get period of PWM[%d, %d]\n",
                            _chip, _channel);
        return 0;
    }

    return nsec_period;
}

void LinuxPWM_Sysfs::set_freq(uint32_t freq)
{
    LinuxUtil* util = LinuxUtil::from(hal.util);

    set_period(util->hz_to_nsec(freq));
}

uint32_t LinuxPWM_Sysfs::get_freq()
{
    LinuxUtil* util = LinuxUtil::from(hal.util);

    return util->nsec_to_hz(get_period());
}

void LinuxPWM_Sysfs::set_duty_cycle(uint32_t nsec_duty_cycle)
{
    if (dprintf(_duty_cycle_fd, "%u", nsec_duty_cycle) < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to set duty cycle of PWM[%d, %d]\n",
                            _chip, _channel);
    } else {
        _nsec_duty_cycle_value = nsec_duty_cycle;
    }
}

uint32_t LinuxPWM_Sysfs::get_duty_cycle()
{
    return _nsec_duty_cycle_value;
}

void LinuxPWM_Sysfs::set_polarity(LinuxPWM_Sysfs::Polarity polarity)
{
    char path[PATH_MAX];
    LinuxUtil* util = LinuxUtil::from(hal.util);

    snprintf(path, sizeof(path), BASE_PWM_PATH "/pwm%d/polarity", _chip, _channel);
    if (util->write_file(path, "%s", polarity == NORMAL ? "normal" : "inversed") < 0) {
        hal.console->printf("LinuxPWM_Sysfs: Unable to set polarity of PWM[%d, %d]\n",
                            _chip, _channel);
    }
}

LinuxPWM_Sysfs::Polarity LinuxPWM_Sysfs::get_polarity()
{
    char path[PATH_MAX];
    LinuxUtil* util = LinuxUtil::from(hal.util);
    char polarity[16];

    snprintf(path, sizeof(path), BASE_PWM_PATH "/pwm%d/polarity", _chip, _channel);
    util->read_file(path, "%s", &polarity);
    return strncasecmp("normal", polarity, sizeof("normal")) ? INVERSE : NORMAL;
}

#endif
