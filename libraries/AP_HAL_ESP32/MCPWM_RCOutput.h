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
 * Code by David "Buzz" Bussenschutt and others
 *
 */

#pragma once


#include <AP_HAL/RCOutput.h>
#include "HAL_ESP32_Namespace.h"
#include "driver/mcpwm.h"
#define HAL_PARAM_DEFAULTS_PATH nullptr
#include <AP_HAL/Util.h>

namespace ESP32 {

class RCOutput : public AP_HAL::RCOutput {
public:
    void init() override;

    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t chan) override;

    void     enable_ch(uint8_t chan) override;
    void     disable_ch(uint8_t chan) override;

    void write(uint8_t chan, uint16_t period_us) override;
//    void set_reversible_mask(uint16_t chanmask) override;
    void cork() override;
    void push() override;

    uint16_t read(uint8_t chan) override;
    void     read(uint16_t* period_us, uint8_t len) override;

    uint16_t read_last_sent(uint8_t chan) override;
    void     read_last_sent(uint16_t* period_us, uint8_t len) override;


//    void set_safety_pwm(uint32_t chmask, uint16_t period_us) override;
//    void set_failsafe_pwm(uint32_t chmask, uint16_t period_us) override;
    bool force_safety_on(void) override;
    void force_safety_off(void) override;
//    void force_safety_no_wait(void) override;
    void set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override {
       _esc_pwm_min = min_pwm;
       _esc_pwm_max = max_pwm;
}
    bool get_esc_scaling(uint16_t &min_pwm, uint16_t &max_pwm) override {
        min_pwm = _esc_pwm_min;
        max_pwm = _esc_pwm_max;
       return true;
}


//    float scale_esc_to_unity(uint16_t pwm) override;
//    bool enable_px4io_sbus_out(uint16_t rate_hz) override;
//    void timer_tick(void) override;
//    bool serial_setup_output(uint8_t chan, uint32_t baudrate, uint16_t chanmask) override;
//    bool serial_write_bytes(const uint8_t *bytes, uint16_t len) override;
//    uint16_t serial_read_bytes(uint8_t *buf, uint16_t len) override;
//    void serial_end(void) override;
//    void set_output_mode(uint16_t mask, enum output_mode mode) override;
//    void set_default_rate(uint16_t rate_hz) override;
//    void set_telem_request_mask(uint16_t mask) override;

private:
    struct pwm_out {
	int gpio_num;
	mcpwm_unit_t unit_num;
	mcpwm_timer_t timer_num;
	mcpwm_io_signals_t io_signal;
	mcpwm_operator_t op;
	uint8_t chan;
};

    struct pwm_group {
	pwm_out out_list[6];
	enum output_mode current_mode;
	uint16_t ch_mask;
    };

    static pwm_group pwm_group_list[];
    bool corked;
    static const uint8_t max_channels = 12;
    AP_HAL::Util::safety_state safety_state;
    uint16_t _esc_pwm_min;
    uint16_t _esc_pwm_max;
    uint16_t period[max_channels];
    uint16_t last_sent[max_channels];
    uint16_t safe_pwm[max_channels];

	uint8_t chan_offset;

};

}
