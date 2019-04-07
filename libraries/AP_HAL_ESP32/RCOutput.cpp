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

#include "RCOutput.h"



//HINTS: see more about MCPWM peripheral here: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/mcpwm.html


#include <AP_HAL/AP_HAL.h>

#define LIST_GROUP 0
#define NUM_GROUPS ARRAY_SIZE(pwm_group_list)
#define CHAN_DISABLED 255

using namespace ESP32;

struct RCOutput::pwm_group RCOutput::pwm_group_list[] = { LIST_GROUP };


void RCOutput::init()
{
        for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
                pwm_group &group = pwm_group_list[i];
                group.current_mode = MODE_PWM_NORMAL;
                for (uint8_t j = 0; j < 4; j++ ) {
                        pwm_out out = group.out_list[j];
                        uint8_t chan = out.chan;
                        if (chan >= 12) //chercher la valeur exacte
                                out.chan = CHAN_DISABLED;
                        if (out.chan != CHAN_DISABLED) {
                                group.ch_mask |= (1U << out.chan);
                                mcpwm_gpio_init(out.unit_num, out.io_signal, out.gpio_num);
                                mcpwm_config_t pwm_config;
                                pwm_config.frequency = 1000;    //frequency = 500Hz,
                                pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
                                pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
                                pwm_config.counter_mode = MCPWM_UP_COUNTER;
                                pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
                                mcpwm_init(out.unit_num, out.timer_num, &pwm_config);
                        }
                }
        }
        set_freq(0xFFFF ^ ((1U << 0) - 1), 500);
#ifdef HAL_GPIO_PIN_SAFETY_IN
        safety_state = AP_HAL::Util::SAFETY_DISARMED;
#endif

}



void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
        for (uint8_t i = 0; i < NUM_GROUPS; i++) {
                pwm_group &group = pwm_group_list[i];
                if (freq_hz > 400 && group.current_mode != MODE_PWM_BRUSHED)
                        freq_hz = 400;
                if ((group.ch_mask & chmask) != 0) {
                        for (int j = 0; j < 4; j++) {
                                pwm_out out = group.out_list[j];
                                mcpwm_set_frequency(out.unit_num, out.timer_num, freq_hz);
                        }
                }
        }
}


uint16_t RCOutput::get_freq(uint8_t chan)
{
        for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
                pwm_group &group = pwm_group_list[i];
                for (uint8_t j = 0; j < 4; j++) {
                        pwm_out &out = group.out_list[j];
                        if (out.chan == chan)
                                return mcpwm_get_frequency(out.unit_num, out.timer_num);
                }
        }
        return 0;
}

void RCOutput::enable_ch(uint8_t chan)
{
        for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
                pwm_group &group = pwm_group_list[i];
                for (uint8_t j = 0; j < 4; j++) {
                        pwm_out &out = group.out_list[j];
                        if (out.chan == chan)
                                mcpwm_start(out.unit_num, out.timer_num);
                }
        }
}

void RCOutput::disable_ch(uint8_t chan)
{
        for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
                pwm_group &group = pwm_group_list[i];
                for (uint8_t j = 0; j < 4; j++) {
                        pwm_out &out = group.out_list[j];
                        if (out.chan == chan)
                                mcpwm_stop(out.unit_num, out.timer_num);
                }
        }
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
        if (chan >= max_channels)
                return;
        last_sent[chan] = period_us;
        if (safety_state == AP_HAL::Util::SAFETY_DISARMED)
                period_us = safe_pwm[chan];
        period[chan] = period_us;

        if (!corked)
                push();
//    mcpwm_set_duty_in_us(mcpwm.unit, mcpwm.timer, mcpwm.op, period_us);
}

void RCOutput::cork()
{
        corked = true;
}

void RCOutput::push()
{
        corked = false;
        for (uint8_t i = 0; i < NUM_GROUPS; i++) {
                pwm_group &group = pwm_group_list[i];
                for (uint8_t j = 0; j < 4; j++) {
                        uint8_t chan = group.out_list[j].chan;
                        if (chan == CHAN_DISABLED)
                                continue;
                        uint32_t period_us = period[chan];
                        if (safety_state == AP_HAL::Util::SAFETY_DISARMED)
                                period_us = safe_pwm[chan];
                        if (group.current_mode == MODE_PWM_BRUSHED) {
                                if (period_us <= _esc_pwm_min)
                                        period_us = 0;
                                if (period_us >= _esc_pwm_max)
                                        period_us = _esc_pwm_max - 1;
                        } else if (group.current_mode == MODE_PWM_ONESHOT125)
                                period_us = ((chan / 1000000U) * period_us) / 8U;
                        else if (group.current_mode < MODE_PWM_DSHOT150)
                                period_us = (chan / 1000000U) * period_us;
                        pwm_out &out = group.out_list[j];
                        mcpwm_set_duty_in_us(out.unit_num, out.timer_num, out.op, period_us);
                }
        }
}

uint16_t RCOutput::read(uint8_t chan)
{
        if (chan >= max_channels)
                return 0;
        return period[chan];
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
        if (len > max_channels) {
                len = max_channels;
        }

        if (len <= chan_offset) {
                return;
        }

        len -= chan_offset;
        period_us += chan_offset;

        memcpy(period_us, period, len * sizeof(uint16_t));
}

uint16_t RCOutput::read_last_sent(uint8_t chan)
{
        if (chan >= max_channels)
                return 0;
        return last_sent[chan];
}

void RCOutput::read_last_sent(uint16_t *period_us, uint8_t len)
{
        if (len > max_channels)
                len = max_channels;
        for (uint8_t i = 0; i < len; i++) {
                period_us[i] = read_last_sent(i);
        }
}

void RCOutput::force_safety_off(void)
{
        safety_state = AP_HAL::Util::SAFETY_ARMED;
}

bool RCOutput::force_safety_on(void)
{
        safety_state = AP_HAL::Util::SAFETY_DISARMED;
        return true;
}
