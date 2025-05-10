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
 * Code by Charles "Silvanosky" Villard, David "Buzz" Bussenschutt,
 * Andrey "ARg" Romanov, and Thomas "tpw_rules" Watson
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

#define TAG "RCOut"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

#ifdef HAL_ESP32_RCOUT

gpio_num_t outputs_pins[] = HAL_ESP32_RCOUT;

//If the RTC source is not required, then GPIO32/Pin12/32K_XP and GPIO33/Pin13/32K_XN can be used as digital GPIOs.

#else
gpio_num_t outputs_pins[] = {};

#endif

/*
 * The MCPWM (motor control PWM) peripheral is used to generate PWM signals for
 * RC output. It is divided up into the following blocks:
 *  * The chip has SOC_MCPWM_GROUPS groups
 *  * Each group has SOC_MCPWM_TIMERS_PER_GROUP timers and operators
 *  * Each operator has SOC_MCPWM_COMPARATORS_PER_OPERATOR comparators and
 *    generators
 *  * Each generator can drive one GPIO pin
 * Though there is more possible, we use the following capabilities:
 *  * Groups have an 8 bit integer prescaler from a 160MHz peripheral clock
 *    (the prescaler value defaults to 2)
 *  * Each timer has an 8 bit integer prescaler from the group clock, a 16 bit
 *    period, and is connected to exactly one operator
 *  * Each comparator in an operator acts on the corresponding timer's value and
 *    is connected to exactly one generator which drives exactly one GPIO pin
 *
 * Each MCPWM group (on ESP32/ESP32S3) gives us 3 independent "PWM groups"
 * (in the STM32 sense) which contain 2 GPIO pins. The pins are assigned
 * consecutively from the HAL_ESP32_RCOUT list. The frequency of each group can
 * be controlled independently by changing that timer's period.
 *  * For "normal" PWM output, running the timer at 1MHz allows 16-1000Hz with
 *    at least 1000 ticks per cycle and makes assigning the compare value easy
 *  * For brushed PWM output, running the timer at 40MHz allows 650-32000Hz with
 *    at least 1000 ticks per cycle and makes an easy divider setting
 *
 * MCPWM is only capable of PWM; DMA-based modes will require using the RMT
 * peripheral.
 */

// each of our PWM groups has its own timer
#define MAX_GROUPS (SOC_MCPWM_GROUPS*SOC_MCPWM_TIMERS_PER_GROUP)
// we connect one timer to one operator
static_assert(SOC_MCPWM_OPERATORS_PER_GROUP >= SOC_MCPWM_TIMERS_PER_GROUP);
// and one generator to one comparator
static_assert(SOC_MCPWM_GENERATORS_PER_OPERATOR >= SOC_MCPWM_COMPARATORS_PER_OPERATOR);

// default settings for PWM ("normal") and brushed mode. carefully understand
// the prescaler update logic in set_group_mode before changing resolution!
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1000ns per tick, 2x80 prescaler
#define BRUSH_TIMEBASE_RESOLUTION_HZ 40000000 // 40MHz, 25ns per tick, 2x2 prescaler

#define SERVO_DEFAULT_FREQ_HZ 50 // the rest of ArduPilot assumes this!

#define MAX_CHANNELS ARRAY_SIZE(outputs_pins)
static_assert(MAX_CHANNELS < 12, "overrunning _pending and safe_pwm"); // max for current chips
static_assert(MAX_CHANNELS < 32, "overrunning bitfields");

#define NEEDED_GROUPS ((MAX_CHANNELS+SOC_MCPWM_COMPARATORS_PER_OPERATOR-1)/SOC_MCPWM_COMPARATORS_PER_OPERATOR)
static_assert(NEEDED_GROUPS <= MAX_GROUPS, "not enough hardware PWM groups");

RCOutput::pwm_group RCOutput::pwm_group_list[NEEDED_GROUPS];
RCOutput::pwm_chan RCOutput::pwm_chan_list[MAX_CHANNELS];

void RCOutput::init()
{
#ifdef CONFIG_IDF_TARGET_ESP32
    // only on plain esp32
    // 32 and 33 are special as they dont default to gpio, but can be if u disable their rtc setup:
    rtc_gpio_deinit(GPIO_NUM_32);
    rtc_gpio_deinit(GPIO_NUM_33);
#endif

    printf("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo\n");
    printf("RCOutput::init() - channels available: %d \n",(int)MAX_CHANNELS);
    printf("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo\n");

    _initialized = true; // assume we are initialized, any error will call abort()

    RCOutput::pwm_group *curr_group = &pwm_group_list[0];
    RCOutput::pwm_chan *curr_ch = &pwm_chan_list[0];
    int chan = 0;

    // loop through all the hardware blocks and set them up. returns when we run
    // out of GPIO pins (each of which is assigned in order to a PWM channel)
    for (int mcpwm_group_id=0; mcpwm_group_id<SOC_MCPWM_GROUPS; mcpwm_group_id++) {
        for (int timer_num=0; timer_num<SOC_MCPWM_TIMERS_PER_GROUP; timer_num++) {
            RCOutput::pwm_group &group = *curr_group++;

            // set up the group to use the default frequency and mode
            group.mcpwm_group_id = mcpwm_group_id;
            group.h_timer = nullptr; // no timer yet
            group.rc_frequency = SERVO_DEFAULT_FREQ_HZ;
            group.ch_mask = 0;
            group.current_mode = MODE_PWM_NORMAL;

            // create the operator for this timer
            mcpwm_operator_config_t operator_config {
                .group_id = mcpwm_group_id,
            };
            ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &group.h_oper));

            // set the mode, which creates the timer and connects the operator
            set_group_mode(group);

            // set up comparators/generators
            for (int comparator_num=0; comparator_num<SOC_MCPWM_COMPARATORS_PER_OPERATOR; comparator_num++) {
                RCOutput::pwm_chan &ch = *curr_ch++;

                // set up the output to be a part of the current group
                ch.group = &group;
                ch.gpio_num = outputs_pins[chan];
                ch.value = 0;
                group.ch_mask |= (1U << chan);

                // create and connect comparator
                mcpwm_comparator_config_t comparator_config {
                    // grab new comparator value when timer is zero
                    .flags { .update_cmp_on_tez = true },
                };
                ESP_ERROR_CHECK(mcpwm_new_comparator(group.h_oper, &comparator_config, &ch.h_cmpr));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(ch.h_cmpr, 0)); // zero the output

                // create and connect generator
                mcpwm_generator_config_t generator_config {
                    .gen_gpio_num = outputs_pins[chan],
                };
                ESP_ERROR_CHECK(mcpwm_new_generator(group.h_oper, &generator_config, &ch.h_gen));

                // configure it to go low on compare threshold (takes priority over going high)
                ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(ch.h_gen,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                        ch.h_cmpr, MCPWM_GEN_ACTION_LOW)));
                // and go high on counter empty
                ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(ch.h_gen,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                        MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

                if (++chan == MAX_CHANNELS) {
                    return; // finished all channels; done setting up the hardware
                }
            }
        }
    }
}

// constrain the frequency for the group given its current mode and return the
// corresponding period for the timer
uint32_t RCOutput::constrain_freq(pwm_group &group) {
    // clamp frequency to keep period between roughly 1000 (preserving output
    // resolution) and UINT16_MAX (hardware limit)
    unsigned int curr_freq = group.rc_frequency;
    switch (group.current_mode) {
    case MODE_PWM_BRUSHED:
        group.rc_frequency = constrain_value(curr_freq, 650U, 32000U);
        return BRUSH_TIMEBASE_RESOLUTION_HZ/group.rc_frequency;

    case MODE_PWM_NORMAL:
    default: // i.e. MODE_PWM_NONE
        // greater than 400 doesn't leave enough time for the down edge
        group.rc_frequency = constrain_value(curr_freq, 16U, 400U);
        return SERVO_TIMEBASE_RESOLUTION_HZ/group.rc_frequency;
    }
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (!_initialized) {
        return;
    }

    for (auto &group : pwm_group_list) {
        if ((group.ch_mask & chmask) != 0) { // group has channels to set?
            group.rc_frequency = freq_hz; // set frequency and corresponding period
            ESP_ERROR_CHECK(mcpwm_timer_set_period(group.h_timer, constrain_freq(group)));

            // disallow changing frequency of this group if it is greater than the default
            if (group.rc_frequency > SERVO_DEFAULT_FREQ_HZ || group.current_mode > MODE_PWM_NORMAL) {
                fast_channel_mask |= group.ch_mask;
            }
        }
    }
}

void RCOutput::set_default_rate(uint16_t freq_hz)
{
    if (!_initialized) {
        return;
    }

    for (auto &group : pwm_group_list) {
        // only set frequency of groups without fast channels
        if (!(group.ch_mask & fast_channel_mask) && group.ch_mask) {
            set_freq(group.ch_mask, freq_hz);
            // setting a high default frequency mustn't make channels fast
            fast_channel_mask &= ~group.ch_mask;
        }
    }
}

/*
  setup output mode for a group, using group.current_mode.
 */
void RCOutput::set_group_mode(pwm_group &group)
{
    if (!_initialized) {
        return;
    }

    // calculate group timer resolution
    uint32_t resolution_hz;
    switch (group.current_mode) {
    case MODE_PWM_BRUSHED:
        resolution_hz = BRUSH_TIMEBASE_RESOLUTION_HZ;
        break;

    default:
        group.current_mode = MODE_PWM_NONE; // treat as 0 output normal
    // fallthrough
    case MODE_PWM_NONE:
    case MODE_PWM_NORMAL:
        resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ;
        break;
    }

    if (group.current_mode > MODE_PWM_NORMAL) {
        fast_channel_mask |= group.ch_mask;
    }

    // the timer prescaler is different in normal vs. brushed mode. the only way
    // to change it with the SDK is to completely destroy the timer, then
    // create a "new" one with the right "resolution" (which is converted
    // internally to the prescaler). fortunately we can keep the
    // operator/comparators/generators around. also fortunately the SDK
    // defaults the MCPWM group prescaler to 2, so we have wiggle room to set
    // each timer's prescaler independently without affecting the others.

    // the code to do this was written after experimenting and studying the SDK
    // code and chip manual. we hope it's applicable to future versions and
    // doesn't create enough output glitches to be a serious problem...

    // if allocated, disable and delete the timer (which, mostly due to hardware
    // limitations, doesn't stop it or disconnect it from the operator)
    if (group.h_timer) {
        ESP_ERROR_CHECK(mcpwm_timer_disable(group.h_timer));
        ESP_ERROR_CHECK(mcpwm_del_timer(group.h_timer));
    }

    // build new timer config with the correct resolution and period
    mcpwm_timer_config_t timer_config {
        .group_id = group.mcpwm_group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M,
        .resolution_hz = resolution_hz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = constrain_freq(group),
    };

    // create a "new" timer with the correct settings for this mode (if already
    // allocated this need not reuse the same hardware unit, but likely will)
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &group.h_timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(group.h_timer));

    // convincing the hardware unit, if reused, to use the new prescaler value
    // is yet another challenge on top of convincing the software to write it
    // to the register. quoth the technical reference manual (ESP32S3, Version
    // 1.6, section 36.3.2.3), "The moment of updating the clock prescaler’s
    // active register is at the time when the timer starts operating.". this
    // statement is backed up and enhanced here:
    // https://www.esp32.com/viewtopic.php?t=14210#p57277

    // stop the timer when its value equals 0 (though we don't need to start it)
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(group.h_timer, MCPWM_TIMER_STOP_EMPTY));

    // now use the sync mechanism to force the value to 0 so we don't have to
    // wait for it to roll around

    // create a software-activated sync source
    mcpwm_sync_handle_t h_sync;
    mcpwm_soft_sync_config_t sync_config {};
    ESP_ERROR_CHECK(mcpwm_new_soft_sync_src(&sync_config, &h_sync));

    // tell the timer to set its value to 0 on sync
    mcpwm_timer_sync_phase_config_t timer_sync_config {
        .sync_src = h_sync,
        .count_value = 0,
        .direction = MCPWM_TIMER_DIRECTION_UP,
    };
    ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(group.h_timer, &timer_sync_config));

    // activate the sync and so request the set
    ESP_ERROR_CHECK(mcpwm_soft_sync_activate(h_sync));

    // disconnect the sync source and delete it; that's all we needed it for
    timer_sync_config.sync_src = nullptr; // set timer to no source
    ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(group.h_timer, &timer_sync_config));
    ESP_ERROR_CHECK(mcpwm_del_sync_src(h_sync));

    // wait a few timer ticks (at the slowest prescale we use) for the set to
    // happen, the timer to stop, and the prescaler to update (these might each
    // take a full tick)
    hal.scheduler->delay_microseconds(10);

    // now, finally!, start it free-running with the right prescale and period
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(group.h_timer, MCPWM_TIMER_START_NO_STOP));

    // oh, and connect the operator, in case we are using a new timer (it can
    // only connect to one timer so this will clear out any old connection)
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(group.h_oper, group.h_timer));
}


void RCOutput::set_output_mode(uint32_t mask, const enum output_mode mode)
{
    while (mask) {
        uint8_t chan = __builtin_ffs(mask)-1;
        if (!_initialized || chan >= MAX_CHANNELS) {
            return;
        }

        pwm_group &group = *pwm_chan_list[chan].group;
        group.current_mode = mode;
        set_group_mode(group);

        // acknowledge the setting of any channels sharing this group
        for (chan=0; chan<MAX_CHANNELS; chan++) {
            if (pwm_chan_list[chan].group == &group) {
                mask &= ~(1U << chan);
            }
        }
    }
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return SERVO_DEFAULT_FREQ_HZ;
    }

    pwm_group &group = *pwm_chan_list[chan].group;
    return group.rc_frequency;
}

void RCOutput::enable_ch(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }

    pwm_chan &ch = pwm_chan_list[chan];
    // set output to high when timer == 0 like normal
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(ch.h_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
}

void RCOutput::disable_ch(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }

    write(chan, 0);
    pwm_chan &ch = pwm_chan_list[chan];
    // set output to low when timer == 0, so the output is always low (after
    // this cycle). conveniently avoids pulse truncation
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(ch.h_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
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

    pwm_chan &ch = pwm_chan_list[chan];
    return ch.value;
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    for (int i = 0; i < MIN(len, MAX_CHANNELS); i++) {
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
                // safety is on, override pwm
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
        // safety is on, override pwm
        period_us = safe_pwm[chan];
    }

    pwm_chan &ch = pwm_chan_list[chan];
    const uint16_t max_period_us = SERVO_TIMEBASE_RESOLUTION_HZ/SERVO_DEFAULT_FREQ_HZ;
    if (period_us > max_period_us) {
        period_us = max_period_us;
    }
    ch.value = period_us;

    uint16_t compare_value;
    switch(ch.group->current_mode) {
    case MODE_PWM_BRUSHED: {
        float duty = 0;
        if (period_us <= _esc_pwm_min) {
            duty = 0;
        } else if (period_us >= _esc_pwm_max) {
            duty = 1;
        } else {
            duty = ((float)(period_us - _esc_pwm_min))/(_esc_pwm_max - _esc_pwm_min);
        }
        compare_value = duty*BRUSH_TIMEBASE_RESOLUTION_HZ/ch.group->rc_frequency;
        break;
    }

    case MODE_PWM_NORMAL:
        compare_value = period_us;
        break;

    case MODE_PWM_NONE:
    default:
        compare_value = 0;
        break;
    }

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(ch.h_cmpr, compare_value));
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
    for (uint8_t i=0; i<ARRAY_SIZE(safe_pwm); i++) {
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
