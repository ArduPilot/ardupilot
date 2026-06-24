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
#include "driver/rmt_encoder.h"

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

    for (uint8_t chan = 0; chan < MAX_CHANNELS; chan++) {
        pwm_chan &ch = pwm_chan_list[chan];
        if (ch.group != &group) {
            continue;
        }

        // idempotent: release any RMT resources from a previous mode change
        dshot_free_chan(ch);

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
        ESP_ERROR_CHECK(rmt_enable(rmt_chan));
        ch.rmt_chan = rmt_chan; // stored as void* (see RCOutput.h)

        // Bytes encoder: each frame bit -> one RMT symbol, high then low, in
        // 12.5 ns ticks. Logical 1 is high for t1h_ticks (0.75 bit), logical 0
        // for t0h_ticks (0.375 bit); the rest of the bit is held low. MSB first,
        // matching the DShot wire order.
        rmt_bytes_encoder_config_t enc_cfg = {};
        enc_cfg.bit0.duration0 = t0h_ticks;             enc_cfg.bit0.level0 = 1;
        enc_cfg.bit0.duration1 = bit_ticks - t0h_ticks; enc_cfg.bit0.level1 = 0;
        enc_cfg.bit1.duration0 = t1h_ticks;             enc_cfg.bit1.level0 = 1;
        enc_cfg.bit1.duration1 = bit_ticks - t1h_ticks; enc_cfg.bit1.level1 = 0;
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
    }

    // start the periodic transmit task now that DShot output exists. Note: it
    // reads pwm_chan RMT handles without a lock, so DShot modes should be set at
    // boot (the normal case), not reconfigured live alongside an active task.
    // TODO(dshot): guard the channel list if runtime mode switching is needed.
    start_dshot_task();
}

/*
  release the RMT channel/encoder held by a channel, if any. Safe to call on a
  channel that is not RMT-backed.
 */
void RCOutput::dshot_free_chan(pwm_chan &ch)
{
    if (ch.rmt_encoder != nullptr) {
        rmt_del_encoder((rmt_encoder_handle_t)ch.rmt_encoder);
        ch.rmt_encoder = nullptr;
    }
    if (ch.rmt_chan != nullptr) {
        rmt_channel_handle_t c = (rmt_channel_handle_t)ch.rmt_chan;
        rmt_disable(c);
        rmt_del_channel(c);
        ch.rmt_chan = nullptr;
    }
}

/*
  build a 16-bit DShot frame: 11-bit value, 1 telemetry-request bit, then a 4-bit
  CRC (XOR of the three nibbles). Matches AP_HAL_ChibiOS::create_dshot_packet()
  for the non-bidirectional case.
 */
uint16_t RCOutput::create_dshot_packet(uint16_t value, bool telem_request)
{
    uint16_t packet = (value << 1) | (telem_request ? 1U : 0U);
    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (uint8_t i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
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
    const uint16_t packet = create_dshot_packet(value, telem_request);
    ch.dshot_buf[0] = uint8_t(packet >> 8);
    ch.dshot_buf[1] = uint8_t(packet & 0xff);

    rmt_transmit_config_t tx_cfg = {};
    tx_cfg.loop_count = 0; // one frame per call; rcout task re-sends each cycle
    rmt_transmit((rmt_channel_handle_t)ch.rmt_chan, (rmt_encoder_handle_t)ch.rmt_encoder,
                 ch.dshot_buf, sizeof(ch.dshot_buf), &tx_cfg);
}

/*
  scale an ArduPilot PWM value (microseconds) to a DShot throttle command.
  Below 1000us (disarmed / safety / no signal) -> 0; 1000..2000us maps linearly
  to 48..2047 (DShot min..max throttle; 1..47 are reserved for commands).
 */
uint16_t RCOutput::dshot_throttle_from_pwm(uint16_t period_us)
{
    if (period_us < 1000) {
        return 0;
    }
    const uint16_t us = period_us > 2000 ? 2000 : period_us;
    return 48 + (uint16_t)(((uint32_t)(us - 1000) * (2047 - 48)) / 1000);
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
        for (uint8_t chan = 0; chan < MAX_CHANNELS; chan++) {
            pwm_chan &ch = pwm_chan_list[chan];
            if (ch.rmt_chan == nullptr) {
                continue; // PWM channel, not DShot
            }
            dshot_send_chan(ch, dshot_throttle_from_pwm((uint16_t)ch.value), false);
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // ~1 kHz frame rate
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
