/*
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
 *
 */
#include "RCOutput.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

using namespace ESP32;

gpio_num_t outputs_pins[] = HAL_ESP32_RCOUT;

void RCOutput::init()
{
    _max_channels = MIN((size_t)LEDC_CHANNEL_MAX, ARRAY_SIZE(outputs_pins));
    for (int i=0; i < LEDC_CHANNEL_MAX; i++) {
        _channel_timers[i] = LEDC_TIMER_MAX;
    }
    int timer = get_timer(50);
    for (int i=0; i<_max_channels; i++) {
        ledc_channel_config_t config = {
            .gpio_num = outputs_pins[i],
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = (ledc_timer_t)timer,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&config);
        _channel_timers[i] = (ledc_timer_t)timer;
    }
    _initialized = true;
}

uint16_t RCOutput::get_timer(uint16_t freq)
{
    for (uint16_t ch = 0; ch < LEDC_CHANNEL_MAX; ch++) {
        if (_channel_timers[ch] != LEDC_TIMER_MAX
            && ledc_get_freq(LEDC_HIGH_SPEED_MODE, _channel_timers[ch]) == freq) {
            return _channel_timers[ch];
        }
    }
    uint16_t timer = find_free_timer();
    ledc_timer_config_t config = {
        LEDC_HIGH_SPEED_MODE,
        (ledc_timer_bit_t)duty_resolution,
        (ledc_timer_t)timer,
        freq
    };
    ledc_timer_config(&config);
    return timer;
}

uint16_t RCOutput::find_free_timer()
{
    for (uint16_t tm = 0; tm < LEDC_TIMER_MAX; ++tm) {
        bool used = false;
        for (uint16_t ch = 0; ch < LEDC_CHANNEL_MAX; ch++) {
            if (_channel_timers[ch] == tm) {
                used = true;
            }
        }
        if (!used) {
            return tm;
        }
    }
    return LEDC_TIMER_MAX;
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (!_initialized) {
        return;
    }
    for (uint8_t i = 0; i < _max_channels; i++) {
        if (chmask & 1 << i) {
            int timer = get_timer(freq_hz);
            ledc_bind_channel_timer(LEDC_HIGH_SPEED_MODE, i, timer);
            _channel_timers[i] = (ledc_timer_t)timer;
        }
    }
}

void RCOutput::set_default_rate(uint16_t freq_hz)
{
    if (!_initialized) {
        return;
    }
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, freq_hz);
}

uint16_t RCOutput::get_freq(uint8_t ch)
{
    if (!_initialized) {
        return 50;
    }
    return ledc_get_freq(LEDC_HIGH_SPEED_MODE, _channel_timers[ch]);
}

void RCOutput::enable_ch(uint8_t ch)
{
}

void RCOutput::disable_ch(uint8_t ch)
{
    write(ch, 0);
}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= _max_channels) {
        return;
    }
    if (_corked) {
        _pending[ch] = period_us;
        _pending_mask |= (1U<<ch);
    } else {
        write_int(ch, period_us);
    }
}

uint16_t RCOutput::read(uint8_t ch)
{
    if (ch >= _max_channels || !_initialized) {
        return 1000;
    }
    uint32_t freq = ledc_get_freq(LEDC_HIGH_SPEED_MODE, _channel_timers[ch]);
    uint32_t duty = ledc_get_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch);
    return (uint64_t(duty) * 1000000)/(freq * (1<< duty_resolution));

}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    for (int i = 0; i < MIN(len, _max_channels); i++) {
        period_us[i] = read(i);
    }
}

void RCOutput::cork(void)
{
    _corked = true;
}

void RCOutput::push(void)
{
    if (!_corked) {
        return;
    }
    for (uint8_t i=0; i<_max_channels; i++) {
        if ((1U<<i) & _pending_mask) {
            write_int(i, _pending[i]);
        }
    }
    _pending_mask = 0;
    _corked = false;
}

void RCOutput::write_int(uint8_t ch, uint16_t period_us)
{
    if (!_initialized) {
        return;
    }
    uint32_t freq = ledc_get_freq(LEDC_HIGH_SPEED_MODE, _channel_timers[ch]);
    uint32_t duty = (uint64_t(period_us) * freq * (1<< duty_resolution))/1000000;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch);
}
