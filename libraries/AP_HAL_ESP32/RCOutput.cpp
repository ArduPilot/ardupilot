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
// New RMT driver for DShot output. Included only here (not in RCOutput.h) to
// avoid a type clash with the legacy RMT driver used by RCInput/RmtSigReader.
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"      // bidirectional DShot: capture the ESC eRPM reply
#include "driver/rmt_encoder.h"
#include "esp_rom_sys.h"        // esp_rom_delay_us (bounded wait for the reply)
#include "soc/gpio_struct.h"    // toggle only the pad output-enable for the bidir turnaround
#include "soc/io_mux_reg.h"     // PIN_INPUT_ENABLE — keep the shared pad readable for RX
#include "soc/gpio_periph.h"    // GPIO_PIN_MUX_REG[]

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

// Bidirectional DShot: per-channel capture state for the ESC eRPM reply. The rcout
// task processes channels sequentially, so a single RX capture is in flight at a
// time, but each channel keeps its own buffer + count so the decode reads a stable one.
// A bidir GCR reply is ~21 transitions; 48 leaves margin (and matches the block size).
#define BDSHOT_RX_SYMBOLS 48
#define BDSHOT_INVALID_ERPM 0xffffU
static rmt_symbol_word_t bdshot_rx_buf[MAX_CHANNELS][BDSHOT_RX_SYMBOLS];
static volatile uint16_t bdshot_rx_nsym[MAX_CHANNELS];
static volatile bool     bdshot_rx_done[MAX_CHANNELS];

// RX-done ISR callback: user_data is the channel index. received_symbols points into
// bdshot_rx_buf[chan] (we passed it to rmt_receive), so we only record the count + flag.
static bool IRAM_ATTR bdshot_on_rx_done(rmt_channel_handle_t ch,
                                        const rmt_rx_done_event_data_t *ed, void *user)
{
    const uint32_t c = (uint32_t)(uintptr_t)user;
    if (c < MAX_CHANNELS) {
        bdshot_rx_nsym[c] = ed->num_symbols;
        __sync_synchronize(); // publish the count (and the driver-filled buffer) before the flag
        bdshot_rx_done[c] = true;
    }
    return false; // no higher-priority task woken
}

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
            // DShot output rate is driven by the RMT rcout task, not the MCPWM timer
            if (is_dshot_protocol(group.current_mode)) {
                continue;
            }
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

    // DShot is generated by the RMT peripheral, not MCPWM. Hand the group off to
    // the RMT backend and skip the MCPWM timer reconfiguration below (whose
    // resolutions/periods only make sense for analog PWM).
    if (is_dshot_protocol(group.current_mode)) {
        set_group_mode_dshot(group);
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

/*
  set up a group for DShot output.

  DShot (DSHOT150/300/600) is a digital protocol generated with the RMT
  peripheral; MCPWM cannot produce the bit pattern. This allocates one RMT TX
  channel per output in the group. We use the RMT in non-DMA mode: each of the
  S3's 4 TX channels owns a 48-symbol memory block and a DShot frame is only ~17
  symbols, so it fits with room to spare. (The S3's RMT-DMA is a single shared
  connection that can't back all 4 channels at once, so DMA is reserved for a
  future single-channel multiplexed >4-output path.) Creating the RMT channel
  re-routes the GPIO (via the matrix) away from the inert MCPWM generator made in
  init().

  The bit-pattern encoder (step 3) and the periodic rcout task that re-transmits
  each frame at the DShot rate (step 4) are added next; until then nothing is
  transmitted, so the line idles low (ESC disarmed). Throttle values written via
  write()/write_int() are stashed in pwm_chan::value, ready for the rcout task.

  Note: the ESP32-S3 has only 4 RMT TX channels, so at most 4 DShot outputs (a
  quad) can be allocated; a 5th+ fails gracefully and stays inactive.
 */
void RCOutput::set_group_mode_dshot(pwm_group &group)
{
    uint32_t bitrate;
    switch (group.current_mode) {
    case MODE_PWM_DSHOT150: bitrate = 150000; break;
    case MODE_PWM_DSHOT300: bitrate = 300000; break;
    case MODE_PWM_DSHOT600: bitrate = 600000; break;
    default:
        return; // not a DShot mode we support
    }

    // Run the RMT at the full 80 MHz APB clock (divider 1, 12.5 ns/tick). Deriving
    // the resolution as bitrate*8 would request e.g. 4.8 MHz for DShot600, which
    // 80 MHz cannot divide evenly — the driver rounds to 5 MHz ("resolution loss"),
    // making DShot600 ~4% fast. A fine fixed resolution keeps the error <0.3%.
    // Each DShot bit holds the line high for 0.75*bit (logical 1) or 0.375*bit
    // (logical 0), then low for the remainder.
    const uint32_t resolution_hz = 80 * 1000 * 1000;
    const uint32_t bit_ticks  = (resolution_hz + bitrate / 2) / bitrate; // ticks per bit
    const uint32_t t1h_ticks  = (bit_ticks * 3 + 2) / 4;  // 0.75 * bit
    const uint32_t t0h_ticks  = (bit_ticks * 3 + 4) / 8;  // 0.375 * bit

    // Hold _dshot_sem across the whole alloc/free loop: set_output_mode() is called
    // more than once during boot (AP_Motors, then SRV/BLHeli setup), so the transmit
    // task may already be running and reading these RMT handles. Freeing/reallocating
    // a channel under it without the lock is a use-after-free (observed as a boot crash).
    WITH_SEMAPHORE(_dshot_sem);

    // Bidirectional DShot reply timing, precomputed once so the rcout task doesn't
    // recompute per frame: the ESC replies at 5/4 the DShot bitrate, and frame_us is
    // how long to let a 16-bit frame finish before the TX->RX turnaround. Written
    // under _dshot_sem: the task reads these group fields during the turnaround.
    const uint32_t telem_rate = bitrate * 5 / 4;
    group.telem_bit_ticks = (resolution_hz + telem_rate / 2) / telem_rate;
    group.frame_us = (uint16_t)((16u * 1000000u) / bitrate + 5); // 16 bits + margin

    // Hold _dshot_sem across the whole alloc/free loop: set_output_mode() is called
    // more than once during boot (AP_Motors, then SRV/BLHeli setup), so the transmit
    // task may already be running and reading these RMT handles. Freeing/reallocating
    // a channel under it without the lock is a use-after-free (observed as a boot crash).
    WITH_SEMAPHORE(_dshot_sem);

    for (uint8_t chan = 0; chan < MAX_CHANNELS; chan++) {
        pwm_chan &ch = pwm_chan_list[chan];
        if (ch.group != &group) {
            continue;
        }

        // idempotent: release any RMT resources from a previous mode change
        dshot_free_chan(ch);
        ch.rmt_rx_chan = nullptr; // bidir RX channel is created below, AFTER the TX channel

        rmt_tx_channel_config_t cfg = {};
        cfg.gpio_num = (gpio_num_t)ch.gpio_num;
        cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        cfg.resolution_hz = resolution_hz;
        cfg.mem_block_symbols = 48; // full per-channel block (non-DMA); frame is ~17
        cfg.trans_queue_depth = 2;
        cfg.flags.with_dma = false;

        rmt_channel_handle_t rmt_chan = nullptr;
        esp_err_t err = rmt_new_tx_channel(&cfg, &rmt_chan);
        if (err != ESP_OK) {
            // most likely no free RMT channel (S3 has 4) — leave it inactive
            // rather than aborting the whole board.
            ch.rmt_chan = nullptr;
            printf("RCOut: DShot RMT alloc failed on chan %u (err 0x%x); S3 has 4 RMT channels\n",
                   (unsigned)chan, (int)err);
            continue;
        }
        err = rmt_enable(rmt_chan);
        if (err != ESP_OK) {
            // treat like the alloc failure above: leave this channel inactive
            // rather than aborting the whole board (ESP_ERROR_CHECK would).
            rmt_del_channel(rmt_chan);
            ch.rmt_chan = nullptr;
            printf("RCOut: DShot RMT enable failed on chan %u (err 0x%x)\n",
                   (unsigned)chan, (int)err);
            continue;
        }
        ch.rmt_chan = rmt_chan; // stored as void* (see RCOutput.h)

        // Bytes encoder: each frame bit -> one RMT symbol, a pulse then the rest of
        // the bit, in 12.5 ns ticks. Logical 1 pulses for t1h_ticks (0.75 bit),
        // logical 0 for t0h_ticks (0.375 bit). MSB first (DShot wire order).
        // Normal DShot idles LOW and pulses HIGH; BIDIRECTIONAL DShot inverts the
        // line (idles HIGH, pulses LOW) — a real bidir ESC detects the mode by this
        // inverted signalling — so the pulse/idle levels are flipped for bidir channels.
        const uint8_t pulse_lvl = ch.bidir ? 0 : 1;
        const uint8_t idle_lvl  = ch.bidir ? 1 : 0;
        rmt_bytes_encoder_config_t enc_cfg = {};
        enc_cfg.bit0.duration0 = t0h_ticks;             enc_cfg.bit0.level0 = pulse_lvl;
        enc_cfg.bit0.duration1 = bit_ticks - t0h_ticks; enc_cfg.bit0.level1 = idle_lvl;
        enc_cfg.bit1.duration0 = t1h_ticks;             enc_cfg.bit1.level0 = pulse_lvl;
        enc_cfg.bit1.duration1 = bit_ticks - t1h_ticks; enc_cfg.bit1.level1 = idle_lvl;
        enc_cfg.flags.msb_first = 1;
        rmt_encoder_handle_t enc = nullptr;
        err = rmt_new_bytes_encoder(&enc_cfg, &enc);
        if (err != ESP_OK) {
            printf("RCOut: DShot encoder alloc failed on chan %u (err 0x%x)\n",
                   (unsigned)chan, (int)err);
            dshot_free_chan(ch); // releases the RMT channel just created
            continue;
        }
        ch.rmt_encoder = enc; // stored as void*

        // Bidirectional DShot: NOW (after the TX channel) create the RX channel on the
        // SAME pad to capture the ESC's eRPM reply after the per-frame turnaround.
        // Creating it after TX avoids TX clobbering the RX input routing (HW-proven).
        // The S3 has 4 RX-capable RMT channels, so a bidir quad consumes all 8 (4TX+4RX).
        if (ch.bidir) {
            rmt_rx_channel_config_t rxcfg = {};
            rxcfg.gpio_num = (gpio_num_t)ch.gpio_num;
            rxcfg.clk_src = RMT_CLK_SRC_DEFAULT;
            rxcfg.resolution_hz = resolution_hz;
            rxcfg.mem_block_symbols = BDSHOT_RX_SYMBOLS;
            rmt_channel_handle_t rx = nullptr;
            esp_err_t rxerr = rmt_new_rx_channel(&rxcfg, &rx);
            if (rxerr == ESP_OK) {
                rmt_rx_event_callbacks_t rxcbs = {};
                rxcbs.on_recv_done = bdshot_on_rx_done;
                rmt_rx_register_event_callbacks(rx, &rxcbs, (void*)(uintptr_t)chan);
                ESP_ERROR_CHECK(rmt_enable(rx));
                ch.rmt_rx_chan = rx; // stored as void* (see RCOutput.h)
                // keep the pad readable: creating the TX channel leaves it output-only,
                // so force input-enable (a pad can hold IE+OE; we gate OE in the turnaround).
                PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[ch.gpio_num]);
            } else {
                // no RX channel: fall back to plain (non-inverted) DShot on this channel,
                // else we'd send inverted-CRC frames the ESC rejects with no turnaround.
                ch.bidir = false;
                printf("RCOut: bidir DShot RX alloc failed on chan %u (err 0x%x); S3 has 4 RX channels\n",
                       (unsigned)chan, (int)rxerr);
            }
        }
    }

    // start the periodic transmit task now that DShot output exists. It reads the
    // pwm_chan RMT handles under _dshot_sem, which this function also holds while
    // (re)allocating them, so a re-entrant set_output_mode() alongside the running
    // task is safe. The task blocks on the semaphore until this call returns.
    start_dshot_task();
}

/*
  release the RMT channel/encoder held by a channel, if any. Safe to call on a
  channel that is not RMT-backed.
 */
void RCOutput::dshot_free_chan(pwm_chan &ch)
{
    // Disable the TX channel BEFORE deleting the encoder: rmt_tx_disable() recycles
    // the pending transaction and calls rmt_encoder_reset() on the encoder it
    // references, so deleting the encoder first is a use-after-free through a
    // function pointer (HW-reproduced: InstructionFetchError with PC in data RAM
    // on a group re-setup while the transmit task had a frame in flight).
    if (ch.rmt_chan != nullptr) {
        rmt_disable((rmt_channel_handle_t)ch.rmt_chan);
    }
    if (ch.rmt_encoder != nullptr) {
        rmt_del_encoder((rmt_encoder_handle_t)ch.rmt_encoder);
        ch.rmt_encoder = nullptr;
    }
    if (ch.rmt_chan != nullptr) {
        rmt_del_channel((rmt_channel_handle_t)ch.rmt_chan);
        ch.rmt_chan = nullptr;
    }
    if (ch.rmt_rx_chan != nullptr) {
        rmt_channel_handle_t r = (rmt_channel_handle_t)ch.rmt_rx_chan;
        rmt_disable(r);
        rmt_del_channel(r);
        ch.rmt_rx_chan = nullptr;
    }
}

/*
  build a 16-bit DShot frame: 11-bit value, 1 telemetry-request bit, then a 4-bit
  CRC (XOR of the three nibbles). For bidirectional DShot the CRC is inverted.
  Matches AP_HAL_ChibiOS::create_dshot_packet().
 */
uint16_t RCOutput::create_dshot_packet(uint16_t value, bool telem_request, bool bidir)
{
    uint16_t packet = (value << 1) | (telem_request ? 1U : 0U);
    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (uint8_t i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    if (bidir) {
        // bidirectional DShot inverts the checksum nibble
        csum = ~csum;
    }
    csum &= 0xf;
    return (packet << 4) | csum;
}

/*
  encode and asynchronously transmit one DShot frame on a channel. The RMT driver
  reads from ch.dshot_buf during the (microseconds-long) transmission, so the
  buffer is per-channel and must not be reused until the frame completes — which
  holds because the rcout task re-sends a channel at most once per cycle.
 */
void RCOutput::dshot_send_chan(pwm_chan &ch, uint16_t value, bool telem_request)
{
    if (ch.rmt_chan == nullptr || ch.rmt_encoder == nullptr) {
        return;
    }
    const uint16_t packet = create_dshot_packet(value, telem_request, ch.bidir);
    ch.dshot_buf[0] = uint8_t(packet >> 8);
    ch.dshot_buf[1] = uint8_t(packet & 0xff);

    rmt_transmit_config_t tx_cfg = {};
    tx_cfg.loop_count = 0; // one frame per call; rcout task re-sends each cycle
    rmt_transmit((rmt_channel_handle_t)ch.rmt_chan, (rmt_encoder_handle_t)ch.rmt_encoder,
                 ch.dshot_buf, sizeof(ch.dshot_buf), &tx_cfg);
}

// Release / re-drive the shared pad for the bidir turnaround by toggling ONLY the
// GPIO output-enable, leaving the RMT->pad matrix routing intact. HW-proven (2026-07-03):
// gpio_set_direction() resets the pad's out signal to plain GPIO and breaks RMT TX,
// and disabling the TX channel does not release the pad either.
static inline void bdshot_pad_oe(uint8_t gpio, bool drive)
{
    if (gpio < 32) {
        if (drive) { ::GPIO.enable_w1ts = (1U << gpio); }
        else       { ::GPIO.enable_w1tc = (1U << gpio); }
    } else {
        if (drive) { ::GPIO.enable1_w1ts.val = (1U << (gpio - 32)); }
        else       { ::GPIO.enable1_w1tc.val = (1U << (gpio - 32)); }
    }
}

/*
  bidirectional DShot turnaround: after the frame is transmitted, release the shared
  pad (output-enable only) so the ESC can drive its eRPM reply, arm the RX channel to
  capture it, then re-drive the pad for the next frame.
  Raw symbols land in bdshot_rx_buf[chan]; bdshot_decode_erpm() turns them into eRPM.
  NOTE: we deliberately do NOT call rmt_tx_wait_all_done() here — it deadlocks
  ("flush timeout") once the pad is released. A timed wait for the frame duration is
  used instead (HW-proven 2026-07-03).
 */
void RCOutput::bdshot_capture_reply(pwm_chan &ch, uint8_t chan)
{
    rmt_channel_handle_t rx = (rmt_channel_handle_t)ch.rmt_rx_chan;

    // let the frame finish on the wire (timed, not rmt_tx_wait_all_done — that
    // deadlocks once the pad is released). frame_us is precomputed per mode.
    esp_rom_delay_us(ch.group->frame_us);

    // RELEASE the pad (OE off); the pull-up idles it high and the ESC pulls low
    bdshot_pad_oe(ch.gpio_num, false);

    rmt_receive_config_t rxcfg = {};
    rxcfg.signal_range_min_ns = 50;      // glitch filter
    rxcfg.signal_range_max_ns = 30000;   // reply is a short burst; a long idle ends it

    bdshot_rx_done[chan] = false;
    if (rmt_receive(rx, bdshot_rx_buf[chan], sizeof(bdshot_rx_buf[chan]), &rxcfg) == ESP_OK) {
        // The ESC replies ~30 us after the frame. Bounded busy-poll (~200 us max):
        // the RMT RX completion is delivered by ISR, but at this sub-tick timescale a
        // FreeRTOS block (1 ms tick) is far too coarse, so we spin briefly instead.
        for (int i = 0; i < 20 && !bdshot_rx_done[chan]; i++) {
            esp_rom_delay_us(10);
        }
        if (bdshot_rx_done[chan]) {
            // the RMT ISR (which fills bdshot_rx_buf) may run on a different core than
            // this pinned task; barrier so we see the completed symbol buffer + count,
            // not a partial one, after observing the done flag.
            __sync_synchronize();
            const uint32_t nsym = bdshot_rx_nsym[chan];
            const uint32_t erpm = bdshot_decode_erpm(bdshot_rx_buf[chan], nsym, ch.group->telem_bit_ticks);
            if (chan < 12) {
                if (erpm == BDSHOT_INVALID_ERPM) {
                    _bdshot.erpm_errors[chan]++;
                } else {
                    _bdshot.erpm_clean_frames[chan]++;
                    bdshot_store_erpm(erpm, chan);
                }
                // keep the error-rate window bounded (avoid uint16 overflow)
                if (_bdshot.erpm_errors[chan] + _bdshot.erpm_clean_frames[chan] >= 4000) {
                    _bdshot.erpm_errors[chan] /= 2;
                    _bdshot.erpm_clean_frames[chan] /= 2;
                }
            }
        } else {
            // No reply in the window (disarmed ESC / none attached). The receive is
            // still pending, and rmt_receive() can't be re-armed while pending, so
            // reset the channel with disable/enable. This runs every frame on an idle
            // ESC (~1 kHz) but is the only way to re-arm; it's cheap and only on the
            // no-telemetry path.
            rmt_disable(rx);
            rmt_enable(rx);
            if (chan < 12) {
                _bdshot.erpm_errors[chan]++;
            }
        }
    }

    // re-drive the pad (OE on) for the next frame's transmit
    bdshot_pad_oe(ch.gpio_num, true);
}

/*
  decode a captured bidirectional-DShot GCR reply into the 12-bit encoded eRPM word
  (0xffff = invalid). Ported from AP_HAL_ChibiOS::bdshot_decode_telemetry_packet
  (originally betaflight, https://github.com/betaflight/betaflight/pull/8554), but
  fed from RMT symbol run-durations instead of timer input-capture timestamps:
  each rmt_symbol_word_t is a high run (duration0) then a low run (duration1), so we
  reconstruct the 21-bit value directly from the run lengths (no timestamp round-trip).
 */
uint32_t RCOutput::bdshot_decode_erpm(const void *symbols, uint32_t nsym, uint32_t bit_ticks)
{
    const rmt_symbol_word_t *sym = (const rmt_symbol_word_t *)symbols;
    if (bit_ticks == 0) {
        return BDSHOT_INVALID_ERPM;
    }
    if (nsym > BDSHOT_RX_SYMBOLS) {
        nsym = BDSHOT_RX_SYMBOLS; // clamp: never index past this channel's buffer row
    }

    // reconstruct the 21-bit GCR value from run durations: a run of `len` telemetry-bit
    // periods encodes a 1 in its MSB position (GCR "toggle" encoding).
    uint32_t value = 0, bits = 0;
    bool ended = false;
    for (uint32_t i = 0; i < nsym && !ended; i++) {
        const uint16_t dur[2] = { sym[i].duration0, sym[i].duration1 };
        for (uint8_t k = 0; k < 2 && !ended; k++) {
            if (dur[k] == 0) { ended = true; break; } // 0-duration marks end of capture
            uint32_t len = ((uint32_t)dur[k] + bit_ticks / 2) / bit_ticks;
            if (len == 0) { len = 1; }
            if (len > 21U - bits) { len = 21U - bits; }
            value <<= len;
            value |= 1U << (len - 1);
            bits += len;
            if (bits >= 21U) { ended = true; }
        }
    }
    // the trailing idle isn't a real edge, so fill any remaining bits as one final run
    if (bits < 21U) {
        const uint32_t len = 21U - bits;
        value <<= len;
        value |= 1U << (len - 1);
        bits += len;
    }
    if (bits != 21U) {
        return BDSHOT_INVALID_ERPM;
    }

    // GCR 5-bit -> 4-bit nibble decode (0 entries are invalid quintets)
    static const uint32_t gcr[32] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };
    uint32_t dv = gcr[value & 0x1fU];
    dv |= gcr[(value >> 5U)  & 0x1fU] << 4U;
    dv |= gcr[(value >> 10U) & 0x1fU] << 8U;
    dv |= gcr[(value >> 15U) & 0x1fU] << 12U;

    // bidir DShot uses an inverted CRC: the folded xor must equal 0xf
    uint32_t csum = dv;
    csum ^= csum >> 8U;
    csum ^= csum >> 4U;
    if ((csum & 0xfU) != 0xfU) {
        return BDSHOT_INVALID_ERPM;
    }
    return dv >> 4U; // 12-bit encoded eRPM (3-bit exponent + 9-bit mantissa)
}

/*
  convert a decoded eRPM word (3-bit exponent + 9-bit mantissa period) to eRPM and
  store it for the channel. RPM = 60 * 1e6 / (period_us * 100), i.e. eRPM below.
 */
void RCOutput::bdshot_store_erpm(uint32_t encodederpm, uint8_t chan)
{
    if (encodederpm == BDSHOT_INVALID_ERPM || chan >= 12) {
        return;
    }
    const uint8_t expo = (encodederpm >> 9U) & 0x7U;
    const uint16_t mant = encodederpm & 0x1ffU;
    const uint32_t period = (uint32_t)mant << expo;
    if (period == 0) {
        return;
    }
    const uint32_t erpm = (1000000U * 60U / 100U + period / 2U) / period;
    if (erpm < BDSHOT_INVALID_ERPM) {
        _bdshot.erpm[chan] = (uint16_t)erpm;
        _bdshot.update_mask |= 1U << chan;
#if HAL_WITH_ESC_TELEM
        // feed the ESC telemetry frontend: mechanical RPM = eRPM * 200 / poles
        if (_bdshot.motor_poles > 0) {
            update_rpm(chan, erpm * 200U / _bdshot.motor_poles, get_erpm_error_rate(chan));
        }
#endif
    }
}

uint16_t RCOutput::get_erpm(uint8_t chan) const
{
    return chan < 12 ? _bdshot.erpm[chan] : 0;
}

bool RCOutput::new_erpm()
{
    return _bdshot.update_mask != 0;
}

uint32_t RCOutput::read_erpm(uint16_t* erpm, uint8_t len)
{
    const uint8_t n = len < 12 ? len : 12;
    memcpy(erpm, _bdshot.erpm, sizeof(uint16_t) * n);
    const uint32_t mask = _bdshot.update_mask;
    _bdshot.update_mask = 0;
    return mask;
}

/*
  scale an ArduPilot PWM value (microseconds) to a DShot throttle command.
  At or below 1000us (disarmed / safety / min / no signal) -> 0 (DShot motor-stop);
  1001..2000us maps linearly to 49..2047 (values 1..47 are reserved for commands,
  48 is the lowest throttle step). Sending 0 (not 48) at minimum matches the
  mainline ChibiOS backend: 48 is the lowest *throttle*, not motor-stop, and a
  BLHeli_S given a constant 48 at idle behaves unpredictably across power-ups
  (observed on hardware: either refuses to release motor output, or arms straight
  into an uncommanded idle spin). 0 is the explicit, safe motor-stop.
 */
uint16_t RCOutput::dshot_throttle_from_pwm(uint16_t period_us)
{
    if (period_us <= 1000) {
        return 0;
    }
    const uint16_t us = period_us > 2000 ? 2000 : period_us;
    // 1001..2000us -> 49..2047; 1000us (handled above) -> 0.
    uint16_t value = (uint16_t)(((uint32_t)(us - 1000) * (2047 - 48)) / 1000);
    return value == 0 ? 0 : value + 48;
}

/*
  start the periodic DShot transmit task once. Pinned to the APP CPU so motor
  output stays off the core that runs WiFi/lwIP (a non-issue on this FC since
  WiFi is disabled, but correct in general and helps timing determinism).
 */
void RCOutput::start_dshot_task()
{
    if (_dshot_task_started) {
        return;
    }
    _dshot_task_started = true;
#if defined(CONFIG_FREERTOS_NUMBER_OF_CORES) && CONFIG_FREERTOS_NUMBER_OF_CORES > 1
    const BaseType_t core = 1; // APP_CPU
#else
    const BaseType_t core = tskNO_AFFINITY;
#endif
    xTaskCreatePinnedToCore(dshot_task_entry, "dshot", 4096, this, 12, nullptr, core);
}

void RCOutput::dshot_task_entry(void *arg)
{
    static_cast<RCOutput *>(arg)->dshot_task();
}

/*
  re-transmit the latest throttle on every DShot channel, forever. DShot ESCs
  disarm if frames stop, so this runs continuously regardless of new writes; the
  main loop just updates pwm_chan::value via write()/push().
 */
void RCOutput::dshot_task()
{
    while (true) {
        {
            // Hold _dshot_sem only while touching the RMT handles, so a concurrent
            // set_group_mode_dshot() (which frees/reallocates them) can never pull a
            // channel out from under an in-flight transmit. Released before the delay
            // so the ~1 ms idle window is when any reconfiguration gets its turn.
            WITH_SEMAPHORE(_dshot_sem);
            for (uint8_t chan = 0; chan < MAX_CHANNELS; chan++) {
                pwm_chan &ch = pwm_chan_list[chan];
                if (ch.rmt_chan == nullptr) {
                    continue; // PWM channel, not DShot
                }
                if (ch.dshot_command_repeat > 0) {
                    // a special command is pending: send it with the telemetry bit set
                    // (DShot commands require it) and count down. Throttle output is
                    // suspended on this channel until the repeats are exhausted.
                    // Barrier pairs with send_dshot_command(): the count was stored
                    // after the command, so load the command only after the count.
                    __sync_synchronize();
                    dshot_send_chan(ch, ch.dshot_command, true);
                    ch.dshot_command_repeat--;
                } else {
                    dshot_send_chan(ch, dshot_throttle_from_pwm((uint16_t)ch.value), false);
                }
                // bidirectional DShot: turn the line around and capture the ESC reply
                if (ch.bidir && ch.rmt_rx_chan != nullptr) {
                    bdshot_capture_reply(ch, chan);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // ~1 kHz frame rate
    }
}

/*
  queue a DShot special command (arm, beep, spin direction, 3D mode, save
  settings, ...) on a single DShot channel or, with chan == ALL_CHANNELS, on all
  of them. DShot commands occupy the reserved value range 0..47 and must be sent
  with the telemetry bit set and repeated to be accepted; the rcout task does the
  repeating (see dshot_task). Throttle output on a channel is suspended while its
  command repeats are pending. command_timeout_ms and priority are accepted for
  API compatibility but unused here: pacing comes from the ~1 kHz rcout task, so
  repeat_count frames span ~repeat_count ms, and there is no separate command queue
  to prioritise. Caller (AP_Motors/AP_BLHeli) is responsible for only issuing
  commands such as direction/save while disarmed.
 */
void RCOutput::send_dshot_command(uint8_t command, uint8_t chan, uint32_t command_timeout_ms,
                                  uint16_t repeat_count, bool priority)
{
    (void)command_timeout_ms;
    (void)priority;
    if (command > 47 || repeat_count == 0) {
        return; // not a special command, or nothing to send
    }
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        if (chan != ALL_CHANNELS && i != chan) {
            continue;
        }
        pwm_chan &ch = pwm_chan_list[i];
        if (ch.rmt_chan == nullptr) {
            continue; // not a DShot channel
        }
        // command before count (see pwm_chan in the header): the rcout task gates
        // on dshot_command_repeat, so this ordering prevents it pairing a new count
        // with a stale command value. The full barrier makes the store order visible
        // to the task on the other core; volatile alone only constrains the compiler.
        ch.dshot_command = command;
        __sync_synchronize();
        ch.dshot_command_repeat = repeat_count;
    }
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

/*
  enable bidirectional DShot on the given channel mask by mirroring it into each
  channel's `bidir` flag; the DShot frame is then inverted (CRC + waveform) for those
  channels and the rcout task performs the TX->RX pad turnaround to read the ESC's eRPM
  reply. Gated upstream by SERVO_BLH_BDMASK via AP_BLHeli.

  The per-channel RX RMT channel and the inverted encoder are set up in
  set_group_mode_dshot() based on ch.bidir. AP_BLHeli calls set_output_mode() (which
  runs that) BEFORE this, so the mode was configured while ch.bidir was still false —
  we must re-run the DShot setup for the affected groups so the RX channels and inverted
  encoders actually get created. Called at boot, so this is not a live-reconfig hazard.
 */
void RCOutput::set_bidir_dshot_mask(uint32_t mask)
{
    bool changed = false;
    for (uint8_t chan = 0; chan < MAX_CHANNELS; chan++) {
        const bool b = (mask & (1U << chan)) != 0;
        if (pwm_chan_list[chan].bidir != b) {
            pwm_chan_list[chan].bidir = b;
            changed = true;
        }
    }
    if (!changed || !_initialized) {
        return;
    }
    // re-apply DShot setup so bidir channels get their RX channel + inverted encoder
    for (uint8_t g = 0; g < NEEDED_GROUPS; g++) {
        pwm_group &group = pwm_group_list[g];
        if (is_dshot_protocol(group.current_mode)) {
            set_group_mode_dshot(group);
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

    // DShot: keep the raw value for the RMT rcout task to encode and transmit;
    // the MCPWM comparator is not used for this channel.
    if (is_dshot_protocol(ch.group->current_mode)) {
        return;
    }

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
