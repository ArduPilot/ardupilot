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
#include <AP_Common/AP_Common.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCOutput_Sysfs.h"

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

LinuxRCOutput_Sysfs::LinuxRCOutput_Sysfs(uint8_t chip, uint8_t channel_count, uint16_t freq) :
    _chip(chip),
    _channel_count(channel_count),
    _default_freq(freq),
    _pwm_channels(new LinuxPWM_Sysfs*[_channel_count])
{
}

LinuxRCOutput_Sysfs::LinuxRCOutput_Sysfs(uint8_t chip, uint8_t channel_count) :
    LinuxRCOutput_Sysfs(chip, channel_count, 20000)
{
}

LinuxRCOutput_Sysfs::~LinuxRCOutput_Sysfs()
{
    for (uint8_t i = 0; i < _channel_count; i++) {
        delete _pwm_channels[i];
    }

    delete _pwm_channels;
}

void LinuxRCOutput_Sysfs::init(void* machtnichts)
{
    for (uint8_t i = 0; i < _channel_count; i++) {
        _pwm_channels[i] = new LinuxPWM_Sysfs(_chip, i);
        if (!_pwm_channels[i]) {
            hal.scheduler->panic(PSTR("LinuxRCOutput_Sysfs_PWM: Unable to setup PWM pin."));
        }
        _pwm_channels[i]->enable(false);
        _pwm_channels[i]->set_freq(_default_freq);
        _pwm_channels[i]->set_duty_cycle(0);
        _pwm_channels[i]->set_polarity(LinuxPWM_Sysfs::Polarity::NORMAL);
    }
}

void LinuxRCOutput_Sysfs::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    for (uint8_t i = 0; i < _channel_count; i++) {
        if (chmask & 1 << i) {
            _pwm_channels[i]->set_freq(freq_hz);
        }
    }
}

uint16_t LinuxRCOutput_Sysfs::get_freq(uint8_t ch)
{
    if (ch >= _channel_count) {
        return 0;
    }

    return _pwm_channels[ch]->get_freq();
}

void LinuxRCOutput_Sysfs::enable_ch(uint8_t ch)
{
    if (ch >= _channel_count) {
        return;
    }

    _pwm_channels[ch]->enable(true);
}

void LinuxRCOutput_Sysfs::disable_ch(uint8_t ch)
{
    if (ch >= _channel_count) {
        return;
    }

    _pwm_channels[ch]->enable(false);
}

void LinuxRCOutput_Sysfs::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= _channel_count) {
        return;
    }

    _pwm_channels[ch]->set_duty_cycle(period_us);
}

uint16_t LinuxRCOutput_Sysfs::read(uint8_t ch)
{
    if (ch >= _channel_count) {
        return 0;
    }

    return _pwm_channels[ch]->get_duty_cycle();
}

void LinuxRCOutput_Sysfs::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

#endif
