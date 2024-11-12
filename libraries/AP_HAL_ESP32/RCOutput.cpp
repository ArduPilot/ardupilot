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
 * Code by Charles "Silvanosky" Villard, David "Buzz" Bussenschutt and Andrey "ARg" Romanov
 *
 */

#include "RCOutput.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "driver/rtc_io.h"

#include <stdio.h>

#include "esp_log.h"

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define TAG "RCOut"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

#ifdef HAL_ESP32_RCOUT

gpio_num_t outputs_pins[] = HAL_ESP32_RCOUT;

//If the RTC source is not required, then GPIO32/Pin12/32K_XP and GPIO33/Pin13/32K_XN can be used as digital GPIOs.

#else
gpio_num_t outputs_pins[] = {};

#endif

#define MAX_CHANNELS ARRAY_SIZE(outputs_pins)

struct RCOutput::pwm_out RCOutput::pwm_group_list[MAX_CHANNELS];

void RCOutput::init()
{
    _max_channels = MAX_CHANNELS;

#ifdef CONFIG_IDF_TARGET_ESP32
    // only on plain esp32
    // 32 and 33 are special as they dont default to gpio, but can be if u disable their rtc setup:
    rtc_gpio_deinit(GPIO_NUM_32);
    rtc_gpio_deinit(GPIO_NUM_33);
#endif

    printf("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo\n");
    printf("RCOutput::init() - channels available: %d \n",(int)MAX_CHANNELS);
    printf("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo\n");

    const int MCPWM_CNT = SOC_MCPWM_OPERATORS_PER_GROUP*SOC_MCPWM_GENERATORS_PER_OPERATOR;

    for (int i = 0; i < MAX_CHANNELS; ++i) {

        mcpwm_timer_handle_t h_timer;
        mcpwm_oper_handle_t  h_oper;

        ESP_LOGI(TAG, "Initialize CH%02d", i+1);

        //Save struct infos
        pwm_out &out = pwm_group_list[i];
        out.group_id = i/MCPWM_CNT;
        out.gpio_num = outputs_pins[i];
        out.chan = i;

        if (0 == i % MCPWM_CNT) {
            
            mcpwm_timer_config_t timer_config = {
                .group_id = out.group_id,
                .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
                .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
                .period_ticks = SERVO_TIMEBASE_PERIOD,
            };
    
            ESP_LOGI(TAG, "Initialize timer");
            ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &h_timer));

            out.freq = timer_config.resolution_hz/timer_config.period_ticks;

            ESP_LOGI(TAG, "Enable and start timer");
            ESP_ERROR_CHECK(mcpwm_timer_enable(h_timer));
            ESP_ERROR_CHECK(mcpwm_timer_start_stop(h_timer, MCPWM_TIMER_START_NO_STOP));
        }
        out.h_timer = h_timer;
         

        if (0 == i % SOC_MCPWM_GENERATORS_PER_OPERATOR) {

            ESP_LOGI(TAG, "Initialize operator");

            mcpwm_operator_config_t operator_config = {
                .group_id = out.group_id, // operator must be in the same group to the timer
            };
            ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &h_oper));
        }
        out.h_oper = h_oper;

        ESP_LOGI(TAG, "Connect timer and operator");
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(out.h_oper, out.h_timer));

        ESP_LOGI(TAG, "Create comparator and generator from the operator");
        mcpwm_comparator_config_t comparator_config = {};
        comparator_config.flags.update_cmp_on_tez = true;

        ESP_ERROR_CHECK(mcpwm_new_comparator(out.h_oper, &comparator_config, &out.h_cmpr));

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = out.gpio_num,
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(out.h_oper, &generator_config, &out.h_gen));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(out.h_cmpr, 1500));
        out.value = 1500;
        
        ESP_LOGI(TAG, "Set generator action on timer and compare event");
        // go high on counter empty
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(out.h_gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        // go low on compare threshold
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(out.h_gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, out.h_cmpr, MCPWM_GEN_ACTION_LOW)));

    }

    _initialized = true;
}



void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (!_initialized) {
        return;
    }

    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        if (chmask & 1 << i) {
            pwm_out &out = pwm_group_list[i];
            ESP_ERROR_CHECK(mcpwm_timer_set_period( out.h_timer, SERVO_TIMEBASE_RESOLUTION_HZ/freq_hz));
            out.freq = freq_hz;
        }
    }
}

void RCOutput::set_default_rate(uint16_t freq_hz)
{
    if (!_initialized) {
        return;
    }
    set_freq(0xFFFFFFFF, freq_hz);
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return 50;
    }

    pwm_out &out = pwm_group_list[chan];
    return out.freq;
}

void RCOutput::enable_ch(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }

    pwm_out &out = pwm_group_list[chan];
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(out.h_timer, MCPWM_TIMER_START_NO_STOP));
}

void RCOutput::disable_ch(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }

    write(chan, 0);
    pwm_out &out = pwm_group_list[chan];
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(out.h_timer, MCPWM_TIMER_STOP_EMPTY));
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }

    if (_corked) {
        _pending[chan] = period_us;
        _pending_mask |= (1U<<chan);
    } else {
        write_int(chan, period_us);
    }

}

uint16_t RCOutput::read(uint8_t chan)
{
    if (chan >= MAX_CHANNELS || !_initialized) {
        return 0;
    }

    pwm_out &out = pwm_group_list[chan];
    return out.value;
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    for (int i = 0; i < MIN(len, _max_channels); i++) {
        period_us[i] = read(i);
    }
}

void RCOutput::cork()
{
    _corked = true;
}

void RCOutput::push()
{
    if (!_corked) {
        return;
    }

    bool safety_on = hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED;

    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        if ((1U<<i) & _pending_mask) {
            uint32_t period_us = _pending[i];

            // If safety is on and safety mask not bypassing
            if (safety_on && !(safety_mask & (1U<<(i)))) {
                // safety is on, overwride pwm
                period_us = safe_pwm[i];
            }
            write_int(i, period_us);
        }
    }

    _corked = false;
}

void RCOutput::timer_tick(void)
{
    safety_update();
}

void RCOutput::write_int(uint8_t chan, uint16_t period_us)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }

    bool safety_on = hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED;
    if (safety_on && !(safety_mask & (1U<<(chan)))) {
        // safety is on, overwride pwm
        period_us = safe_pwm[chan];
    }

    pwm_out &out = pwm_group_list[chan];
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(out.h_cmpr, period_us));
    out.value = period_us;
}

/*
  get safety switch state for Util.cpp
 */
AP_HAL::Util::safety_state RCOutput::_safety_switch_state(void)
{
    if (!hal.util->was_watchdog_reset()) {
        hal.util->persistent_data.safety_state = safety_state;
    }
    return safety_state;
}

/*
  force the safety switch on, disabling PWM output from the IO board
*/
bool RCOutput::force_safety_on(void)
{
    safety_state = AP_HAL::Util::SAFETY_DISARMED;
    return true;
}

/*
  force the safety switch off, enabling PWM output from the IO board
*/
void RCOutput::force_safety_off(void)
{
    safety_state = AP_HAL::Util::SAFETY_ARMED;
}

/*
  set PWM to send to a set of channels when the safety switch is
  in the safe state
*/
void RCOutput::set_safety_pwm(uint32_t chmask, uint16_t period_us)
{
    for (uint8_t i=0; i<16; i++) {
        if (chmask & (1U<<i)) {
            safe_pwm[i] = period_us;
        }
    }
}

/*
  update safety state
 */
void RCOutput::safety_update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - safety_update_ms < 100) {
        // update safety at 10Hz
        return;
    }
    safety_update_ms = now;

    AP_BoardConfig *boardconfig = AP_BoardConfig::get_singleton();

    if (boardconfig) {
        // remember mask of channels to allow with safety on
        safety_mask = boardconfig->get_safety_mask();
    }

#ifdef HAL_GPIO_PIN_SAFETY_IN
    gpio_set_direction((gpio_num_t)HAL_GPIO_PIN_SAFETY_IN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)HAL_GPIO_PIN_SAFETY_IN, GPIO_PULLDOWN_ONLY);
    bool safety_pressed = gpio_get_level((gpio_num_t)HAL_GPIO_PIN_SAFETY_IN);
    if (safety_pressed) {
        AP_BoardConfig *brdconfig = AP_BoardConfig::get_singleton();
        if (safety_press_count < UINT8_MAX) {
            safety_press_count++;
        }
        if (brdconfig && brdconfig->safety_button_handle_pressed(safety_press_count)) {
            if (safety_state ==AP_HAL::Util::SAFETY_ARMED) {
                safety_state = AP_HAL::Util::SAFETY_DISARMED;
            } else {
                safety_state = AP_HAL::Util::SAFETY_ARMED;
            }
        }
    } else {
        safety_press_count = 0;
    }
#endif

#ifdef HAL_GPIO_PIN_LED_SAFETY
    led_counter = (led_counter+1) % 16;
    const uint16_t led_pattern = safety_state==AP_HAL::Util::SAFETY_DISARMED?0x5500:0xFFFF;
    gpio_set_level((gpio_num_t)HAL_GPIO_PIN_LED_SAFETY, (led_pattern & (1U << led_counter))?0:1);
#endif
}

/*
  set PWM to send to a set of channels if the FMU firmware dies
*/
void RCOutput::set_failsafe_pwm(uint32_t chmask, uint16_t period_us)
{
    //RIP (not the pointer)
}
