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
#pragma once

#include "AP_HAL_ESP32.h"
#include "driver/ledc.h"

class ESP32::RCOutput : public AP_HAL::RCOutput {
public:
    RCOutput() {};

    ~RCOutput() {};

    static RCOutput *from(AP_HAL::RCOutput *rcoutput)
    {
        return static_cast<RCOutput *>(rcoutput);
    }

    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void enable_ch(uint8_t ch) override;
    void disable_ch(uint8_t ch) override;
    void write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t *period_us, uint8_t len) override;
    void cork(void) override;
    void push(void) override;
    void set_default_rate(uint16_t freq_hz) override;

private:
    const int duty_resolution = LEDC_TIMER_14_BIT;

    void write_int(uint8_t ch, uint16_t period_us);
    uint16_t get_timer(uint16_t freq);
    uint16_t find_free_timer();

    // for handling cork()/push()
    bool _corked;
    uint16_t _pending[LEDC_CHANNEL_MAX];
    uint32_t _pending_mask;

    uint16_t _max_channels;
    ledc_timer_t _channel_timers[LEDC_CHANNEL_MAX];
    bool _initialized;
};
