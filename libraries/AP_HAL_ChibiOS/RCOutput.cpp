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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 * Bi-directional dshot based on Betaflight, code by Andy Piper and Siddharth Bharat Purohit
 * 
 * There really is no dshot reference. For information try these resources:
 * https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
 * https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
 * https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs
 */

#include <hal.h>

#if defined(IOMCU_FW) && HAL_DSHOT_ENABLED
// need to give the little guy as much help as possible
#pragma GCC optimize("O2")
#endif

#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "GPIO.h"
#include "Util.h"
#include "Scheduler.h"
#include "hwdef/common/stm32_util.h"
#include "hwdef/common/watchdog.h"
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Common/ExpandingString.h>
#ifndef HAL_NO_UARTDRIVER
#include <GCS_MAVLink/GCS.h>
#endif

#if AP_SIM_ENABLED
#include <AP_HAL/SIMState.h>
#endif

#if HAL_USE_PWM == TRUE
#include <SRV_Channel/SRV_Channel.h>

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#define RCOU_SERIAL_TIMING_DEBUG 0
#define LED_THD_WA_SIZE 256
#ifndef HAL_NO_LED_THREAD
#if defined(HAL_NO_RCOUT_THREAD)
#define HAL_NO_LED_THREAD 1
#else
#define HAL_NO_LED_THREAD 0
#endif
#endif

#define TELEM_IC_SAMPLE 16

struct RCOutput::pwm_group RCOutput::pwm_group_list[] = { HAL_PWM_GROUPS };
#if HAL_SERIAL_ESC_COMM_ENABLED
struct RCOutput::irq_state RCOutput::irq;
#endif
const uint8_t RCOutput::NUM_GROUPS = ARRAY_SIZE(RCOutput::pwm_group_list);

// event mask for triggering a PWM send
// EVT_PWM_SEND  = 11
static const eventmask_t EVT_PWM_START  = EVENT_MASK(12);
// EVT_PWM_SYNTHETIC_SEND  = 13
static const eventmask_t EVT_PWM_SEND_NEXT  = EVENT_MASK(14);
static const eventmask_t EVT_LED_SEND  = EVENT_MASK(15);

// #pragma GCC optimize("Og")

/*
  initialise RC output driver
 */
void RCOutput::init()
{
    if (_initialised) {
        // cannot init RCOutput twice
        return;
    }

#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        // with IOMCU the local (FMU) channels start at 8
        chan_offset = 8;
        iomcu_enabled = true;
    }
    if (AP_BoardConfig::io_dshot()) {
        iomcu_dshot = true;
    }
#endif

    for (auto &group : pwm_group_list) {
        const uint8_t i = &group - pwm_group_list;
        //Start Pwm groups
        group.current_mode = MODE_PWM_NORMAL;
        group.dshot_event_mask = EVENT_MASK(i);

        for (uint8_t j = 0; j < 4; j++ ) {
#if !defined(IOMCU_FW)
            uint8_t chan = group.chan[j];
            if (SRV_Channels::is_GPIO(chan+chan_offset)) {
                group.chan[j] = CHAN_DISABLED;
            } else if (SRV_Channels::is_alarm(chan+chan_offset)
                || SRV_Channels::is_alarm_inverted(chan+chan_offset)) {
                // alarm takes the whole timer
                group.ch_mask = 0;
                group.current_mode = MODE_PWM_NONE;
                for (uint8_t k = 0; k < 4; k++) {
                    group.chan[k] = CHAN_DISABLED;
                    group.pwm_cfg.channels[k].mode = PWM_OUTPUT_DISABLED;
                }
                ChibiOS::Util::from(hal.util)->toneAlarm_init(group.pwm_cfg, group.pwm_drv, j,
                    SRV_Channels::is_alarm(chan+chan_offset));
                break;
            }
#endif
            if (group.chan[j] != CHAN_DISABLED) {
                num_fmu_channels = MAX(num_fmu_channels, group.chan[j]+1);
                group.ch_mask |= (1U<<group.chan[j]);
            }
#ifdef HAL_WITH_BIDIR_DSHOT
            group.bdshot.telem_tim_ch[j] = CHAN_DISABLED;
#endif
        }
        if (group.ch_mask != 0) {
            pwmStart(group.pwm_drv, &group.pwm_cfg);
            group.pwm_started = true;
        }
        chVTObjectInit(&group.dma_timeout);
    }

#if HAL_WITH_IO_MCU
    if (iomcu_enabled) {
        iomcu.init();
    }
#endif
    chMtxObjectInit(&trigger_mutex);
    chVTObjectInit(&_dshot_rate_timer);
    // setup default output rate of 50Hz
    set_freq(0xFFFF ^ ((1U<<chan_offset)-1), 50);

    safety_state = AP_HAL::Util::SAFETY_DISARMED;

#if RCOU_DSHOT_TIMING_DEBUG
    hal.gpio->pinMode(54, 1);
    hal.gpio->pinMode(55, 1);
    hal.gpio->pinMode(56, 1);
    hal.gpio->pinMode(57, 1);
#endif

    hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &RCOutput::safety_update, void));
    _initialised = true;
}

#if HAL_SERIALLED_ENABLED
// start the led thread
bool RCOutput::start_led_thread(void)
{
#if HAL_NO_LED_THREAD
    return false;
#else
    WITH_SEMAPHORE(led_thread_sem);

    if (led_thread_created) {
        return true;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&RCOutput::led_thread, void), "led", LED_THD_WA_SIZE, AP_HAL::Scheduler::PRIORITY_LED, 0)) {
        return false;
    }

    led_thread_created = true;
    return true;
#endif
}

/*
  thread for handling LED RCOutpu
 */
void RCOutput::led_thread()
{
    {
        WITH_SEMAPHORE(led_thread_sem);
        led_thread_ctx = chThdGetSelfX();
    }

    // don't start outputting until fully configured
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }

    while (true) {
        chEvtWaitOne(EVT_LED_SEND);
        // if DMA sharing is in effect there can be quite a delay between the request to begin the cycle and
        // actually sending out data - thus we need to work out how much time we have left to collect the locks

        // process any pending LED output requests
        led_timer_tick(LED_OUTPUT_PERIOD_US + AP_HAL::micros64());
    }
}
#endif // HAL_SERIAL_ENABLED

/*
  thread for handling RCOutput send on FMU
 */
#if !defined(IOMCU_FW)
void RCOutput::rcout_thread()
{
    uint64_t last_thread_run_us = 0; // last time we did a 1kHz run of rcout
    uint64_t last_cycle_run_us = 0;

    rcout_thread_ctx = chThdGetSelfX();

    // don't start outputting until fully configured
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }

    // dshot is quite sensitive to timing, it's important to output pulses as
    // regularly as possible at the correct bitrate
    while (true) {
        const auto mask = chEvtWaitOne(EVT_PWM_SEND | EVT_PWM_SYNTHETIC_SEND);
        const bool have_pwm_event = (mask & (EVT_PWM_SEND | EVT_PWM_SYNTHETIC_SEND)) != 0;
        // start the clock
        last_thread_run_us = AP_HAL::micros64();

        // this is when the cycle is supposed to start
        if (_dshot_cycle == 0 && have_pwm_event) {
            last_cycle_run_us = AP_HAL::micros64();
            // register a timer for the next tick if push() will not be providing it
            if (_dshot_rate != 1) {
                chVTSet(&_dshot_rate_timer, chTimeUS2I(_dshot_period_us), dshot_update_tick, this);
            }
        }

        // if DMA sharing is in effect there can be quite a delay between the request to begin the cycle and
        // actually sending out data - thus we need to work out how much time we have left to collect the locks
        uint64_t time_out_us = (_dshot_cycle + 1) * _dshot_period_us + last_cycle_run_us;
        if (!_dshot_rate) {
            time_out_us = last_thread_run_us + _dshot_period_us;
        }

        // main thread requested a new dshot send or we timed out - if we are not running
        // as a multiple of loop rate then ignore EVT_PWM_SEND events to preserve periodicity
        if (!in_soft_serial() && have_pwm_event) {
            dshot_send_groups(time_out_us);

            // now unlock everything
            dshot_collect_dma_locks(time_out_us);

            if (_dshot_rate > 0) {
                _dshot_cycle = (_dshot_cycle + 1) % _dshot_rate;
            }
        }

        // process any pending RC output requests
        timer_tick(time_out_us);
#if RCOU_DSHOT_TIMING_DEBUG
        static bool output_masks = true;
        if (AP_HAL::millis() > 5000 && output_masks) {
            output_masks = false;
            hal.console->printf("bdmask 0x%lx, en_mask 0x%lx, 3dmask 0x%lx:\n", _bdshot.mask, en_mask, _reversible_mask);
            for (auto &group : pwm_group_list) {
                hal.console->printf("  timer %u: ch_mask 0x%lx, en_mask 0x%lx\n", group.timer_id, group.ch_mask, group.en_mask);
            }
        }
#endif
    }
}
#endif

__RAMFUNC__ void RCOutput::dshot_update_tick(virtual_timer_t* vt, void* p)
{
    chSysLockFromISR();
    RCOutput* rcout = (RCOutput*)p;

    if (rcout->_dshot_cycle + 1 < rcout->_dshot_rate) {
        chVTSetI(&rcout->_dshot_rate_timer, chTimeUS2I(rcout->_dshot_period_us), dshot_update_tick, p);
    }
    chEvtSignalI(rcout->rcout_thread_ctx, EVT_PWM_SYNTHETIC_SEND);
    chSysUnlockFromISR();
}

#if AP_HAL_SHARED_DMA_ENABLED
// calculate how much time remains in the current cycle
sysinterval_t RCOutput::calc_ticks_remaining(pwm_group &group, uint64_t time_out_us, uint32_t output_period_us)
{
    // calculate how long we have left
    uint64_t now = AP_HAL::micros64();
    // if we have time left wait for the event
    const uint64_t pulse_elapsed_us = now - group.last_dmar_send_us;
    uint32_t wait_us = 0;
    if (now < time_out_us) {
        wait_us = time_out_us - now;
    }
    if (pulse_elapsed_us < group.dshot_pulse_send_time_us) {
        // better to let the burst write in progress complete rather than cancelling mid way through
        wait_us = MAX(wait_us, group.dshot_pulse_send_time_us - pulse_elapsed_us);
    }

    // waiting for a very short period of time can cause a
    // timer wrap with ChibiOS timers. Use CH_CFG_ST_TIMEDELTA
    // as minimum. Don't allow for a very long delay (over _dshot_period_us)
    // to prevent bugs in handling timer wrap
    const uint32_t max_delay_us = output_period_us;
    const uint32_t min_delay_us = 10; // matches our CH_CFG_ST_TIMEDELTA
    wait_us = constrain_uint32(wait_us, min_delay_us, max_delay_us);

    return MIN(TIME_MAX_INTERVAL, chTimeUS2I(wait_us));
}

// release locks on the groups that are pending in reverse order
void RCOutput::dshot_collect_dma_locks(uint64_t time_out_us, bool led_thread)
{
    if (NUM_GROUPS == 0) {
        return;
    }
    for (int8_t i = NUM_GROUPS - 1; i >= 0; i--) {
        pwm_group &group = pwm_group_list[i];

        if (led_thread != is_led_protocol(group.current_mode)) {
            continue;
        }

        // dma handle will only be unlocked if the send was aborted
        if (group.dma_handle != nullptr && group.dma_handle->is_locked()) {
            // if we have time left wait for the event
            const sysinterval_t wait_ticks = calc_ticks_remaining(group, time_out_us,
                                                                  led_thread ? LED_OUTPUT_PERIOD_US : _dshot_period_us);
            const eventmask_t mask = chEvtWaitOneTimeout(group.dshot_event_mask, wait_ticks);

            // no time left cancel and restart
            if (!mask) {
                dma_cancel(group);
            }
            group.dshot_waiter = nullptr;
#ifdef HAL_WITH_BIDIR_DSHOT
            // if using input capture DMA then clean up
            if (group.bdshot.enabled) {
                // only unlock if not shared
                if (group.bdshot.curr_ic_dma_handle != nullptr
                    && group.bdshot.curr_ic_dma_handle != group.dma_handle) {
                    group.bdshot.curr_ic_dma_handle->unlock();
                    group.bdshot.curr_ic_dma_handle = nullptr;
                }
            }
#endif
            group.dma_handle->unlock();
        }
    }
}
#endif // AP_HAL_SHARED_DMA_ENABLED

/*
  setup the output frequency for a group and start pwm output
 */
void RCOutput::set_freq_group(pwm_group &group)
{
    if (mode_requires_dma(group.current_mode)) {
        // speed setup in DMA handler
        return;
    }

    uint16_t freq_set = group.rc_frequency;
    uint32_t old_clock = group.pwm_cfg.frequency;
    uint32_t old_period = group.pwm_cfg.period;

    if (freq_set > 400 || group.current_mode == MODE_PWM_ONESHOT125) {
        // use a 8MHz clock for higher frequencies or for
        // oneshot125. Using 8MHz for oneshot125 results in the full
        // 1000 steps for smooth output
        group.pwm_cfg.frequency = 8000000;
    } else if (freq_set <= 400) {
        // use a 1MHz clock
        group.pwm_cfg.frequency = 1000000;
    }

    // check if the frequency is possible, and keep halving
    // down to 1MHz until it is OK with the hardware timer we
    // are using. If we don't do this we'll hit an assert in
    // the ChibiOS PWM driver on some timers
    PWMDriver *pwmp = group.pwm_drv;
    uint32_t psc = (pwmp->clock / pwmp->config->frequency) - 1;
    while ((psc > 0xFFFF || ((psc + 1) * pwmp->config->frequency) != pwmp->clock) &&
           group.pwm_cfg.frequency > 1000000) {
        group.pwm_cfg.frequency /= 2;
        psc = (pwmp->clock / pwmp->config->frequency) - 1;
    }

    if (group.current_mode == MODE_PWM_ONESHOT ||
        group.current_mode == MODE_PWM_ONESHOT125) {
        // force a period of 0, meaning no pulses till we trigger
        group.pwm_cfg.period = 0;
    } else {
        group.pwm_cfg.period = group.pwm_cfg.frequency/freq_set;
    }

    bool force_reconfig = false;
    for (uint8_t j=0; j<4; j++) {
        if (group.pwm_cfg.channels[j].mode == PWM_OUTPUT_ACTIVE_LOW) {
            group.pwm_cfg.channels[j].mode = PWM_OUTPUT_ACTIVE_HIGH;
            force_reconfig = true;
        }
        if (group.pwm_cfg.channels[j].mode == PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW) {
            group.pwm_cfg.channels[j].mode = PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH;
            force_reconfig = true;
        }

    }

    if (old_clock != group.pwm_cfg.frequency ||
        old_period != group.pwm_cfg.period ||
        !group.pwm_started ||
        force_reconfig) {
        // we need to stop and start to setup the new clock
        if (group.pwm_started) {
            pwmStop(group.pwm_drv);
        }
        pwmStart(group.pwm_drv, &group.pwm_cfg);
        group.pwm_started = true;
    }
    pwmChangePeriod(group.pwm_drv, group.pwm_cfg.period);
}

/*
  set output frequency in HZ for a set of channels given by a mask
 */
void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
#if HAL_WITH_IO_MCU
    if (iomcu_enabled) {
        // change frequency on IOMCU
        uint16_t io_chmask = chmask & 0xFF;
        if (io_chmask) {
            // disallow changing frequency of this group if it is greater than the default
            for (uint8_t i=0; i<ARRAY_SIZE(iomcu.ch_masks); i++) {
                const uint16_t mask = io_chmask & iomcu.ch_masks[i];
                if (mask != 0) {
                    if (freq_hz > 50) {
                        io_fast_channel_mask |= mask;
                    } else {
                        io_fast_channel_mask &= ~mask;
                    }
                }
            }
            iomcu.set_freq(io_fast_channel_mask, freq_hz);
        }
    }
#endif

    // convert to a local (FMU) channel mask
    chmask >>= chan_offset;
    if (chmask == 0) {
        return;
    }

    /*
      we enable the new frequency on all groups that have one
      of the requested channels. This means we may enable high
      speed on some channels that aren't requested, but that
      is needed in order to fly a vehicle such as a hex
      multicopter properly
    */
    for (auto &group : pwm_group_list) {
        // greater than 400 doesn't give enough room at higher periods for
        // the down pulse. This still allows for high rate with oneshot and dshot.
        uint16_t group_freq = freq_hz;
        if (group_freq > 400 && group.current_mode != MODE_PWM_BRUSHED) {
            group_freq = 400;
        }
        if ((group.ch_mask & chmask) != 0) {
            group.rc_frequency = group_freq;
            set_freq_group(group);
            // disallow changing frequency of this group if it is greater than the default
            if (group_freq > 50) {
                fast_channel_mask |= group.ch_mask;
            }
        }
    }
}

/*
  set default output rate
 */
void RCOutput::set_default_rate(uint16_t freq_hz)
{
#if HAL_WITH_IO_MCU
    if (iomcu_enabled) {
        iomcu.set_default_rate(freq_hz);
    }
#endif
    for (auto &group : pwm_group_list) {
        if ((group.ch_mask & fast_channel_mask) || group.ch_mask == 0) {
            // don't change fast channels
            continue;
        }
        group.pwm_cfg.period = group.pwm_cfg.frequency/freq_hz;
        if (group.pwm_started) {
            pwmChangePeriod(group.pwm_drv, group.pwm_cfg.period);
        }
    }
}

/*
  Set the dshot rate as a multiple of the loop rate.
  This is called late after init_ardupilot() so groups will have been setup
 */
void RCOutput::set_dshot_rate(uint8_t dshot_rate, uint16_t loop_rate_hz)
{
    // for low loop rates simply output at 1Khz on a timer
    if (loop_rate_hz <= 100 || dshot_rate == 0) {
        _dshot_period_us = 1000UL;
        _dshot_rate = 0;
#if HAL_WITH_IO_MCU
        if (iomcu_dshot) {
            iomcu.set_dshot_period(1000UL, 0);
        }
#endif
        return;
    }
    // if there are non-dshot channels then do likewise
    for (auto &group : pwm_group_list) {
        if (group.current_mode == MODE_PWM_ONESHOT ||
            group.current_mode == MODE_PWM_ONESHOT125 ||
            group.current_mode == MODE_PWM_BRUSHED) {
            _dshot_period_us = 1000UL;
            _dshot_rate = 0;
#if HAL_WITH_IO_MCU
            // this is not strictly neccessary since the iomcu could run at a different rate,
            // but there is only one parameter to control this
            if (iomcu_dshot) {
                iomcu.set_dshot_period(1000UL, 0);
            }
#endif
            return;
        }
    }

    uint16_t drate = dshot_rate * loop_rate_hz;
    _dshot_rate = dshot_rate;
    // BLHeli32 uses a 16 bit counter for input calibration which at 48Mhz will wrap
    // at 732Hz so never allow rates below 800hz
    while (drate < 800) {
        _dshot_rate++;
        drate = _dshot_rate * loop_rate_hz;
    }
    // prevent stupidly high rates, ideally should also prevent high rates
    // with slower dshot variants
    if (drate > 4000) {
        _dshot_rate = 4000 / loop_rate_hz;
        drate = _dshot_rate * loop_rate_hz;
    }
    _dshot_period_us = 1000000UL / drate;
#if HAL_WITH_IO_MCU
    if (iomcu_dshot) {
        iomcu.set_dshot_period(_dshot_period_us, _dshot_rate);
    }
#endif
}

#if HAL_DSHOT_ENABLED
/*
 Set/get the dshot esc_type
 */
void RCOutput::set_dshot_esc_type(DshotEscType dshot_esc_type)
{
    _dshot_esc_type = dshot_esc_type;
    switch (_dshot_esc_type) {
        case DSHOT_ESC_BLHELI_S:
        case DSHOT_ESC_BLHELI_EDT_S:
            DSHOT_BIT_WIDTH_TICKS = DSHOT_BIT_WIDTH_TICKS_S;
            DSHOT_BIT_0_TICKS = DSHOT_BIT_0_TICKS_S;
            DSHOT_BIT_1_TICKS = DSHOT_BIT_1_TICKS_S;
            break;
        default:
            DSHOT_BIT_WIDTH_TICKS = DSHOT_BIT_WIDTH_TICKS_DEFAULT;
            DSHOT_BIT_0_TICKS = DSHOT_BIT_0_TICKS_DEFAULT;
            DSHOT_BIT_1_TICKS = DSHOT_BIT_1_TICKS_DEFAULT;
            break;
    }
#if HAL_WITH_IO_MCU
    if (iomcu_dshot) {
        iomcu.set_dshot_esc_type(dshot_esc_type);
    }
#endif
}
#endif // #if HAL_DSHOT_ENABLED

/*
  find pwm_group and index in group given a channel number
 */
RCOutput::pwm_group *RCOutput::find_chan(uint8_t chan, uint8_t &group_idx)
{
    if (chan >= max_channels) {
        return nullptr;
    }
    if (chan < chan_offset) {
        return nullptr;
    }
    chan -= chan_offset;

    for (auto &group : pwm_group_list) {
        for (uint8_t j = 0; j < 4; j++) {
            if (group.chan[j] == chan) {
                group_idx = j;
                return &group;
            }
        }
    }
    return nullptr;
}

/*
 * return mask of channels that must be disabled because they share a group with a digital channel
 */
uint32_t RCOutput::get_disabled_channels(uint32_t digital_mask)
{
    uint32_t dmask = (digital_mask >> chan_offset);
    uint32_t disabled_chan_mask = 0;
    for (auto &group : pwm_group_list) {
        bool digital_group = false;
        for (uint8_t j = 0; j < 4; j++) {
            if ((1U << group.chan[j]) & dmask) {
                digital_group = true;
            }
        }
        if (digital_group) {
            for (uint8_t j = 0; j < 4; j++) {
                if (!((1U << group.chan[j]) & dmask)) {
                    disabled_chan_mask |= (1U << group.chan[j]);
                }
            }
        }
    }

    disabled_chan_mask <<= chan_offset;
    return disabled_chan_mask;
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
#if HAL_WITH_IO_MCU
    if (chan < chan_offset) {
        return iomcu.get_freq(chan);
    }
#endif
    uint8_t i;
    pwm_group *grp = find_chan(chan, i);
    if (grp) {
        return grp->pwm_drv->config->frequency / grp->pwm_drv->period;
    }
    // assume 50Hz default
    return 50;
}

void RCOutput::enable_ch(uint8_t chan)
{
#if HAL_WITH_IO_MCU
    if (chan < chan_offset && iomcu_enabled) {
        iomcu.enable_ch(chan);
        return;
    }
#endif
    uint8_t i;
    pwm_group *grp = find_chan(chan, i);
    if (grp) {
        en_mask |= 1U << (chan - chan_offset);
        grp->en_mask |= 1U << (chan - chan_offset);
    }
}

void RCOutput::disable_ch(uint8_t chan)
{
#if HAL_WITH_IO_MCU
    if (chan < chan_offset && iomcu_enabled) {
        iomcu.disable_ch(chan);
        return;
    }
#endif
    uint8_t i;
    pwm_group *grp = find_chan(chan, i);
    if (grp) {
        pwmDisableChannel(grp->pwm_drv, i);
        en_mask &= ~(1U<<(chan - chan_offset));
        grp->en_mask &= ~(1U << (chan - chan_offset));
    }
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{

    if (chan >= max_channels) {
        return;
    }
    last_sent[chan] = period_us;

#if AP_SIM_ENABLED
    hal.simstate->pwm_output[chan] = period_us;
    if (!(AP::sitl()->on_hardware_output_enable_mask & (1U<<chan))) {
        return;
    }
#endif

#if HAL_WITH_IO_MCU
    // handle IO MCU channels
    if (iomcu_enabled) {
        iomcu.write_channel(chan, period_us);
    }
#endif
    if (chan < chan_offset) {
        return;
    }

    if (safety_state == AP_HAL::Util::SAFETY_DISARMED && !(safety_mask & (1U<<chan))) {
        // implement safety pwm value
        period_us = 0;
    }

    chan -= chan_offset;

    period[chan] = period_us;

    if (chan < num_fmu_channels) {
        active_fmu_channels = MAX(chan+1, active_fmu_channels);
        if (!corked) {
            push_local();
        }
    }
}

/*
  push values to local channels from period[] array
 */
void RCOutput::push_local(void)
{
    if (active_fmu_channels == 0) {
        return;
    }
    uint32_t outmask = (1U<<active_fmu_channels)-1;
    outmask &= en_mask;

    uint16_t widest_pulse = 0;
    uint8_t need_trigger = 0;

    bool safety_on = hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED;
    for (auto &group : pwm_group_list) {
        if (in_soft_serial()) {
            continue;
        }
        if (!group.pwm_started) {
            continue;
        }
        for (uint8_t j = 0; j < 4; j++) {
            uint8_t chan = group.chan[j];
            if (!group.is_chan_enabled(j)) {
                continue;
            }
            if (outmask & (1UL<<chan)) {
                uint32_t period_us = period[chan];

                if (safety_on && !(safety_mask & (1U<<(chan+chan_offset)))) {
                    // safety is on, overwride pwm
                    period_us = 0;
                }

                if (group.current_mode == MODE_PWM_BRUSHED) {
                    if (period_us <= _esc_pwm_min) {
                        period_us = 0;
                    } else if (period_us >= _esc_pwm_max) {
                        period_us = PWM_FRACTION_TO_WIDTH(group.pwm_drv, 1, 1);
                    } else {
                        period_us = PWM_FRACTION_TO_WIDTH(group.pwm_drv,\
                               (_esc_pwm_max - _esc_pwm_min), (period_us - _esc_pwm_min));
                    }
                    pwmEnableChannel(group.pwm_drv, j, period_us);
                } else if (group.current_mode == MODE_PWM_ONESHOT125) {
                    // this gives us a width in 125 ns increments, giving 1000 steps over the 125 to 250 range
                    uint32_t width = ((group.pwm_cfg.frequency/1000000U) * period_us) / 8U;
                    pwmEnableChannel(group.pwm_drv, j, width);
                    // scale the period down so we don't delay for longer than we need to
                    period_us /= 8;
                }
                else if (group.current_mode < MODE_PWM_DSHOT150) {
                    uint32_t width = (group.pwm_cfg.frequency/1000000U) * period_us;
                    pwmEnableChannel(group.pwm_drv, j, width);
                }
#if HAL_DSHOT_ENABLED
                else if (is_dshot_protocol(group.current_mode) || is_led_protocol(group.current_mode)) {
                    // set period_us to time for pulse output, to enable very fast rates
                    period_us = group.dshot_pulse_time_us;
                }
#endif // HAL_DSHOT_ENABLED
                if (group.current_mode == MODE_PWM_ONESHOT ||
                    group.current_mode == MODE_PWM_ONESHOT125 ||
                    group.current_mode == MODE_NEOPIXEL ||
                    group.current_mode == MODE_NEOPIXELRGB ||
                    group.current_mode == MODE_PROFILED ||
                    is_dshot_protocol(group.current_mode)) {
                    // only control widest pulse for oneshot and dshot
                    if (period_us > widest_pulse) {
                        widest_pulse = period_us;
                    }
                    const uint8_t i = &group - pwm_group_list;
                    need_trigger |= (1U<<i);
                }
            }
        }
    }

    if (widest_pulse > 2300) {
        widest_pulse = 2300;
    }
    trigger_widest_pulse = widest_pulse;

    trigger_groupmask = need_trigger;

    if (trigger_groupmask) {
        trigger_groups();
    }
}

uint16_t RCOutput::read(uint8_t chan)
{
    if (chan >= max_channels) {
        return 0;
    }
#if HAL_WITH_IO_MCU
    if (chan < chan_offset) {
        return iomcu.read_channel(chan);
    }
#endif
    chan -= chan_offset;
    return period[chan];
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    if (len > max_channels) {
        len = max_channels;
    }
#if HAL_WITH_IO_MCU
    for (uint8_t i=0; i<MIN(len, chan_offset); i++) {
        period_us[i] = iomcu.read_channel(i);
    }
#endif
    if (len <= chan_offset) {
        return;
    }
    len -= chan_offset;
    period_us += chan_offset;

    memcpy(period_us, period, len*sizeof(uint16_t));
}

uint16_t RCOutput::read_last_sent(uint8_t chan)
{
    if (chan >= max_channels) {
        return 0;
    }
    return last_sent[chan];
}

void RCOutput::read_last_sent(uint16_t* period_us, uint8_t len)
{
    if (len > max_channels) {
        len = max_channels;
    }
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read_last_sent(i);
    }
}

/*
  does an output mode require the use of the UP DMA channel?
 */
bool RCOutput::mode_requires_dma(enum output_mode mode) const
{
#if !HAL_DSHOT_ENABLED
    return false;
#else
    return is_dshot_protocol(mode) || is_led_protocol(mode);
#endif //#if !HAL_DSHOT_ENABLED
}

void RCOutput::print_group_setup_error(pwm_group &group, const char* error_string)
{
#ifndef HAL_NO_UARTDRIVER
    uint8_t min_chan = UINT8_MAX;
    uint8_t max_chan = 0;
    for (uint8_t j = 0; j < 4; j++) {
        uint8_t chan = group.chan[j];
        if (chan == CHAN_DISABLED) {
            continue;
        }
        chan += chan_offset;
        min_chan = MIN(min_chan,chan);
        max_chan = MAX(max_chan,chan);
    }

    if (min_chan == max_chan) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Chan %i, %s: %s",min_chan+1,get_output_mode_string(group.current_mode),error_string);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Chan %i to %i, %s: %s",min_chan+1,max_chan+1,get_output_mode_string(group.current_mode),error_string);
    }
#endif
}

/*
  setup a group for DMA output at a given bitrate. The bit_width is
  the value for a pulse width in the DMA buffer for a full bit.

  This is used for both DShot and serial output
 */
bool RCOutput::setup_group_DMA(pwm_group &group, uint32_t bitrate, uint32_t bit_width, bool active_high, const uint16_t buffer_length,
                               uint32_t pulse_time_us, bool at_least_freq)
{
#if HAL_DSHOT_ENABLED
    // for dshot we setup for DMAR based output
#if !AP_HAL_SHARED_DMA_ENABLED
    if (group.dma == nullptr) {
        // we don't do dma sharing in non advanced mode
        chSysLock();
        group.dma = dmaStreamAllocI(group.dma_up_stream_id, 10, dma_up_irq_callback, &group);
        chSysUnlock();
        if (group.dma == nullptr) {
            print_group_setup_error(group, "failed to allocate DMA");
            return false;
        }
    }
#else
    if (!group.dma_handle) {
        group.dma_handle = new Shared_DMA(group.dma_up_stream_id, SHARED_DMA_NONE,
                                          FUNCTOR_BIND_MEMBER(&RCOutput::dma_allocate, void, Shared_DMA *),
                                          FUNCTOR_BIND_MEMBER(&RCOutput::dma_deallocate, void, Shared_DMA *));
        if (!group.dma_handle) {
            print_group_setup_error(group, "failed to allocate DMA");
            return false;
        }
    }
#endif

    // hold the lock during setup, to ensure there isn't a DMA operation ongoing
#if AP_HAL_SHARED_DMA_ENABLED
    group.dma_handle->lock();
#endif
    if (!group.dma_buffer || buffer_length != group.dma_buffer_len) {
        if (group.dma_buffer) {
            hal.util->free_type(group.dma_buffer, group.dma_buffer_len, AP_HAL::Util::MEM_DMA_SAFE);
            group.dma_buffer_len = 0;
        }
        group.dma_buffer = (dmar_uint_t *)hal.util->malloc_type(buffer_length, AP_HAL::Util::MEM_DMA_SAFE);
        if (!group.dma_buffer) {
#if AP_HAL_SHARED_DMA_ENABLED
            group.dma_handle->unlock();
#endif
            print_group_setup_error(group, "failed to allocate DMA buffer");
            return false;
        }
        group.dma_buffer_len = buffer_length;
    }
    // reset the pulse time inside the lock
    group.dshot_pulse_time_us = group.dshot_pulse_send_time_us = pulse_time_us;

#ifdef HAL_WITH_BIDIR_DSHOT
    // configure input capture DMA if required
    if (is_bidir_dshot_enabled(group)) {
        if (!bdshot_setup_group_ic_DMA(group)) {
            _bdshot.mask &= ~group.ch_mask; // only use dshot on this group
            _bdshot.disabled_mask |= group.ch_mask;
            active_high = true;
        }
    }
#endif
    // configure timer driver for DMAR at requested rate
    if (group.pwm_started) {
        pwmStop(group.pwm_drv);
        group.pwm_started = false;
    }
    const uint32_t target_frequency = bitrate * bit_width;

    const uint32_t prescaler = calculate_bitrate_prescaler(group.pwm_drv->clock, target_frequency, at_least_freq);
    if (prescaler == 0 || prescaler > 0x8000) {
#if AP_HAL_SHARED_DMA_ENABLED
        group.dma_handle->unlock();
#endif
        print_group_setup_error(group, "failed to match clock speed");
        return false;
    }

    const uint32_t freq = group.pwm_drv->clock / (prescaler + 1);
    // PSC is calculated by ChibiOS as (pwm_drv.clock / pwm_cfg.frequency) - 1;
    group.pwm_cfg.frequency = freq;
    group.pwm_cfg.dier = TIM_DIER_UDE;
    group.pwm_cfg.cr2 = 0;
    group.bit_width_mul = (freq + (target_frequency/2)) / target_frequency;
    // ARR is calculated by ChibiOS as pwm_cfg.period -1
    group.pwm_cfg.period = bit_width * group.bit_width_mul;
#if 0
    hal.console->printf("CLOCK=%u BW=%u FREQ=%u BR=%u MUL=%u PRE=%u\n", unsigned(group.pwm_drv->clock), unsigned(bit_width), unsigned(group.pwm_cfg.frequency),
        unsigned(bitrate), unsigned(group.bit_width_mul), unsigned(prescaler));
    static char clock_setup[64];
    hal.util->snprintf(clock_setup, 64, "CLOCK=%u BW=%u FREQ=%u BR=%u MUL=%u PRE=%u\n", unsigned(group.pwm_drv->clock), unsigned(bit_width), unsigned(group.pwm_cfg.frequency),
        unsigned(bitrate), unsigned(group.bit_width_mul), unsigned(prescaler));
#endif
    for (uint8_t j=0; j<4; j++) {
        pwmmode_t mode = group.pwm_cfg.channels[j].mode;
        if (mode != PWM_OUTPUT_DISABLED) {
            if(mode == PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW || mode == PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH) {
               group.pwm_cfg.channels[j].mode = active_high ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH : PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW;
            } else {
               group.pwm_cfg.channels[j].mode = active_high ? PWM_OUTPUT_ACTIVE_HIGH : PWM_OUTPUT_ACTIVE_LOW;
            }
        }
    }

    pwmStart(group.pwm_drv, &group.pwm_cfg);
    group.pwm_started = true;

    for (uint8_t j=0; j<4; j++) {
        if (group.is_chan_enabled(j)) {
            pwmEnableChannel(group.pwm_drv, j, 0);
        }
    }

#if AP_HAL_SHARED_DMA_ENABLED
    group.dma_handle->unlock();
#endif
    return true;
#else
    return false;
#endif // HAL_DSHOT_ENABLED
}

/*
  setup output mode for a group, using group.current_mode. Used to restore output
  after serial operations
 */
void RCOutput::set_group_mode(pwm_group &group)
{
    if (group.pwm_started) {
        pwmStop(group.pwm_drv);
        group.pwm_started = false;
    }
#ifdef HAL_WITH_BIDIR_DSHOT
    memset(group.bdshot.erpm, 0, 4*sizeof(uint16_t));
#endif
    switch (group.current_mode) {
    case MODE_PWM_BRUSHED:
        // force zero output initially
        for (uint8_t i=0; i<4; i++) {
            if (group.chan[i] == CHAN_DISABLED) {
                continue;
            }
            uint8_t chan = chan_offset + group.chan[i];
            write(chan, 0);
        }
        break;

    case MODE_NEOPIXEL:
    case MODE_NEOPIXELRGB:
    case MODE_PROFILED:
#if HAL_SERIALLED_ENABLED
    {
        uint8_t bits_per_pixel = 24;
        uint32_t bit_width = NEOP_BIT_WIDTH_TICKS;
        bool active_high = true;

        if (!start_led_thread()) {
            group.current_mode = MODE_PWM_NONE;
            break;
        }

        if (group.current_mode == MODE_PROFILED) {
            bits_per_pixel = 25;
            bit_width = PROFI_BIT_WIDTH_TICKS;
            active_high = false;
        }

        const uint32_t rate = protocol_bitrate(group.current_mode);

        // configure timer driver for DMAR at requested rate
        const uint8_t pad_end_bits = 8;
        const uint8_t pad_start_bits = 1;
        const uint8_t channels_per_group = 4;
        const uint16_t bit_length = bits_per_pixel * group.serial_nleds + pad_start_bits + pad_end_bits;
        const uint16_t buffer_length = bit_length * sizeof(uint32_t) * channels_per_group;
        // calculate min time between pulses taking into account the DMAR parallelism
        const uint32_t pulse_time_us = 1000000UL * bit_length / rate;

        if (!setup_group_DMA(group, rate, bit_width, active_high, buffer_length, pulse_time_us, false)) {
            group.current_mode = MODE_PWM_NONE;
            break;
        }
        break;
    }
#endif

    case MODE_PWM_DSHOT150 ... MODE_PWM_DSHOT1200: {
#if HAL_DSHOT_ENABLED
        const uint32_t rate = protocol_bitrate(group.current_mode);
        bool active_high = is_bidir_dshot_enabled(group) ? false : true;
        bool at_least_freq = false;
        // calculate min time between pulses
        const uint32_t pulse_send_time_us = 1000000UL * dshot_bit_length / rate;

        // BLHeli_S (and BlueJay) appears to always want the frequency above the target
        if (_dshot_esc_type == DSHOT_ESC_BLHELI_S || _dshot_esc_type == DSHOT_ESC_BLHELI_EDT_S) {
            at_least_freq = true;
        }

        // configure timer driver for DMAR at requested rate
        if (!setup_group_DMA(group, rate, DSHOT_BIT_WIDTH_TICKS, active_high,
                             MAX(DSHOT_BUFFER_LENGTH, GCR_TELEMETRY_BUFFER_LEN), pulse_send_time_us, at_least_freq)) {
            group.current_mode = MODE_PWM_NORMAL;
            break;
        }
        if (is_bidir_dshot_enabled(group)) {
            group.dshot_pulse_send_time_us = pulse_send_time_us;
            // to all intents and purposes the pulse time of send and receive are the same
            group.dshot_pulse_time_us = pulse_send_time_us + pulse_send_time_us + 30;
        }
#endif
        break;
    }

    case MODE_PWM_ONESHOT:
    case MODE_PWM_ONESHOT125:
        // for oneshot we set a period of 0, which results in no pulses till we trigger
        group.pwm_cfg.period = 0;
        group.rc_frequency = 1;
        if (group.pwm_started) {
            pwmChangePeriod(group.pwm_drv, group.pwm_cfg.period);
        }
        break;

    case MODE_PWM_NORMAL:
    case MODE_PWM_NONE:
        // nothing needed
        break;
    }

    set_freq_group(group);

    if (group.current_mode != MODE_PWM_NONE &&
        !group.pwm_started) {
        pwmStart(group.pwm_drv, &group.pwm_cfg);
        group.pwm_started = true;
        for (uint8_t j=0; j<4; j++) {
            if (group.is_chan_enabled(j)) {
                pwmEnableChannel(group.pwm_drv, j, 0);
            }
        }
    }
}

/*
  setup output mode
 */
void RCOutput::set_output_mode(uint32_t mask, const enum output_mode mode)
{
    for (auto &group : pwm_group_list) {
        enum output_mode thismode = mode;
        if (((group.ch_mask << chan_offset) & mask) == 0) {
            // this group is not affected
            continue;
        }
        if (mode_requires_dma(thismode) && !group.have_up_dma) {
            print_group_setup_error(group, "failed, no DMA");
            thismode = MODE_PWM_NORMAL;
        }
        if (mode > MODE_PWM_NORMAL) {
            fast_channel_mask |= group.ch_mask;
        }
        // setup of the group mode also sets up DMA which might have changed, so always
        // redo it if using DMA
        if (group.current_mode != thismode) {
            group.current_mode = thismode;
            set_group_mode(group);
        }
    }
#if HAL_WITH_IO_MCU
    const uint16_t iomcu_mask = (mask & ((1U<<chan_offset)-1));
    if ((mode == MODE_PWM_ONESHOT ||
         mode == MODE_PWM_ONESHOT125 ||
         mode == MODE_PWM_BRUSHED ||
         (mode >= MODE_PWM_DSHOT150 && mode <= MODE_PWM_DSHOT600)) &&
        iomcu_mask &&
        iomcu_enabled) {
        iomcu.set_output_mode(iomcu_mask, mode);
        return;
    }
#endif
}

/*
 Get current non-PWM output mode for all channels
 */
RCOutput::output_mode RCOutput::get_output_mode(uint32_t& mask)
{
    enum output_mode mode = MODE_PWM_NONE;

    for (auto &group : pwm_group_list) {
        if (group.current_mode == MODE_PWM_NONE) {
            continue;
        }

        if (group.current_mode != MODE_PWM_NORMAL) {
            mode = group.current_mode;
            mask |= group.en_mask;
        }
    }

    return mode;
}

/*
  enable telemetry request for a mask of channels. This is used
  with DShot to get telemetry feedback
  The mask uses servo channel numbering
 */
void RCOutput::set_telem_request_mask(uint32_t mask)
{
#if HAL_WITH_IO_MCU
    if (iomcu_dshot && (mask & ((1U<<chan_offset)-1))) {
        iomcu.set_telem_request_mask(mask);
    }
#endif
    telem_request_mask = (mask >> chan_offset);
}

/*
 * get output mode banner to inform user of how outputs are configured
 */
bool RCOutput::get_output_mode_banner(char banner_msg[], uint8_t banner_msg_len) const
{
    if (!hal.scheduler->is_system_initialized()) {
        hal.util->snprintf(banner_msg, banner_msg_len, "RCOut: Initialising");
        return true;
    }

    // create array of each channel's mode
    output_mode ch_mode[chan_offset + NUM_GROUPS * ARRAY_SIZE(pwm_group::chan)] = {};
    bool have_nonzero_modes = false;

#if HAL_WITH_IO_MCU
    // fill in ch_mode array for IOMCU channels
    if (iomcu_enabled) {
        uint8_t iomcu_mask;
        const output_mode iomcu_mode = iomcu.get_output_mode(iomcu_mask);
        for (uint8_t i = 0; i < chan_offset; i++ ) {
            if (iomcu_mask & 1U<<i) {
                ch_mode[i] = iomcu_mode;
            } else {
                ch_mode[i] = MODE_PWM_NORMAL;
            }
        }
        have_nonzero_modes = (chan_offset > 0) && (iomcu_mode != MODE_PWM_NONE);
    }
#endif

    // fill in ch_mode array for FMU channels
    for (auto &group : pwm_group_list) {
        if (group.current_mode != MODE_PWM_NONE) {
            for (uint8_t j = 0; j < ARRAY_SIZE(group.chan); j++) {
                if (group.chan[j] != CHAN_DISABLED) {
                    const uint8_t chan_num = group.chan[j] + chan_offset;
                    if (chan_num < ARRAY_SIZE(ch_mode)) {
                        ch_mode[chan_num] = group.current_mode;
                        have_nonzero_modes = true;
                    }
                }
            }
        }
    }

    // handle simple case
    if (!have_nonzero_modes) {
        hal.util->snprintf(banner_msg, banner_msg_len, "RCOut: None");
        return true;
    }

    // write banner to banner_msg
    hal.util->snprintf(banner_msg, banner_msg_len, "RCOut:");
    uint8_t curr_mode_lowest_ch = 0;
    for (uint8_t k = 1; k < ARRAY_SIZE(ch_mode); k++) {
        if (ch_mode[k-1] != ch_mode[k]) {
            if (ch_mode[k-1] != MODE_PWM_NONE) {
                append_to_banner(banner_msg, banner_msg_len, ch_mode[k-1], curr_mode_lowest_ch + 1, k);
            }
            curr_mode_lowest_ch = k;
        }
    }

    // add final few channel's mode to banner (won't have been done by above loop)
    const uint8_t final_index = ARRAY_SIZE(ch_mode)-1;
    if (ch_mode[final_index] != MODE_PWM_NONE) {
        append_to_banner(banner_msg, banner_msg_len, ch_mode[final_index], curr_mode_lowest_ch + 1, final_index + 1);
    }

    return true;
}

/*
  start corking output
 */
void RCOutput::cork(void)
{
    corked = true;
#if HAL_WITH_IO_MCU
    if (iomcu_enabled) {
        iomcu.cork();
    }
#endif
}

/*
  stop corking output
 */
void RCOutput::push(void)
{
    corked = false;
    push_local();
#if HAL_WITH_IO_MCU
    if (iomcu_enabled) {
        iomcu.push();
    }
#endif
}

/*
  enable sbus output
 */
bool RCOutput::enable_px4io_sbus_out(uint16_t rate_hz)
{
#if HAL_WITH_IO_MCU
    if (iomcu_enabled) {
        return iomcu.enable_sbus_out(rate_hz);
    }
#endif
    return false;
}

/*
  trigger output groups for oneshot or dshot modes
 */
void RCOutput::trigger_groups(void)
{
    if (!chMtxTryLock(&trigger_mutex)) {
        return;
    }
    uint64_t now = AP_HAL::micros64();
    if (now < min_pulse_trigger_us) {
        // guarantee minimum pulse separation
        hal.scheduler->delay_microseconds(min_pulse_trigger_us - now);
    }

    osalSysLock();
    for (auto &group : pwm_group_list) {
        if (soft_serial_waiting()) {
            // doing serial output, don't send pulses
            continue;
        }
        if (group.current_mode == MODE_PWM_ONESHOT ||
            group.current_mode == MODE_PWM_ONESHOT125) {
            const uint8_t i = &group - pwm_group_list;
            if (trigger_groupmask & (1U<<i)) {
                // this triggers pulse output for a channel group
                group.pwm_drv->tim->EGR = STM32_TIM_EGR_UG;
            }
        }
    }
    osalSysUnlock();
#if !defined(HAL_NO_RCOUT_THREAD) || HAL_DSHOT_ENABLED
    // trigger a PWM send
    if (!in_soft_serial() &&
        (hal.scheduler->in_main_thread() ||
         chThdGetPriorityX() > APM_MAIN_PRIORITY_BOOST) && rcout_thread_ctx) {
        chEvtSignal(rcout_thread_ctx, EVT_PWM_SEND);
    }
#endif
    /*
      calculate time that we are allowed to trigger next pulse
      to guarantee at least a 50us gap between pulses
    */
    min_pulse_trigger_us = AP_HAL::micros64() + trigger_widest_pulse + 50;

    chMtxUnlock(&trigger_mutex);
}

/*
  periodic timer. This is used for oneshot and dshot modes, plus for
  safety switch update. Runs every 1000us.
 */
void RCOutput::timer_tick(uint64_t time_out_us)
{
    if (in_soft_serial()) {
        return;
    }

    if (min_pulse_trigger_us == 0) {
        return;
    }

    uint64_t now = AP_HAL::micros64();

    if (now > min_pulse_trigger_us &&
        now - min_pulse_trigger_us > 4000) {
        // trigger at a minimum of 250Hz
        trigger_groups();
    }
}

/*
  periodic timer called from led thread. This is used for LED output
 */
void RCOutput::led_timer_tick(uint64_t time_out_us)
{
    if (in_soft_serial()) {
        return;
    }

    // if we have enough time left send out LED data
    if (serial_led_pending) {
        serial_led_pending = false;
        for (auto &group : pwm_group_list) {
            serial_led_pending |= !serial_led_send(group);
        }

        // release locks on the groups that are pending in reverse order
        dshot_collect_dma_locks(time_out_us, true);
    }
}

// send dshot for all groups that support it
void RCOutput::dshot_send_groups(uint64_t time_out_us)
{
#if HAL_DSHOT_ENABLED
    if (in_soft_serial()) {
        return;
    }

    bool command_sent = false;
    // queue up a command if there is one
    if (_dshot_current_command.cycle == 0
        && _dshot_command_queue.pop(_dshot_current_command)) {
        // got a new command
    }

    for (auto &group : pwm_group_list) {
        bool pulse_sent;
        // send a dshot command
        if (is_dshot_protocol(group.current_mode)
            && dshot_command_is_active(group)) {
            command_sent = dshot_send_command(group, _dshot_current_command.command, _dshot_current_command.chan);
            pulse_sent = true;
        // actually do a dshot send
        } else if (group.can_send_dshot_pulse()) {
            dshot_send(group, time_out_us);
            pulse_sent = true;
        }
#if defined(HAL_WITH_BIDIR_DSHOT) && defined(HAL_TIM_UP_SHARED)
        // prevent the next send going out until the previous send has released its DMA channel
        if (pulse_sent && group.shared_up_dma && group.bdshot.enabled) {
            chEvtWaitOneTimeout(DSHOT_CASCADE, calc_ticks_remaining(group, time_out_us, _dshot_period_us));
        }
#else
        (void)pulse_sent;
#endif
    }

    if (command_sent) {
        _dshot_current_command.cycle--;
    }
#endif // HAL_DSHOT_ENABLED
}

__RAMFUNC__ void RCOutput::dshot_send_next_group(void* p)
{
    chSysLockFromISR();
    RCOutput* rcout = (RCOutput*)p;

    chEvtSignalI(rcout->rcout_thread_ctx, EVT_PWM_SEND_NEXT);
    chSysUnlockFromISR();
}

/*
  allocate DMA channel
 */
#if AP_HAL_SHARED_DMA_ENABLED
void RCOutput::dma_allocate(Shared_DMA *ctx)
{
    for (auto &group : pwm_group_list) {
        if (group.dma_handle == ctx && group.dma == nullptr) {
            chSysLock();
            group.dma = dmaStreamAllocI(group.dma_up_stream_id, 10, dma_up_irq_callback, &group);
#if defined(STM32F1)
            if (group.pwm_started && group.dma_handle->is_shared()) {
                /* Timer configured and started.*/
                group.pwm_drv->tim->CR1   = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;
                group.pwm_drv->tim->DIER  = group.pwm_drv->config->dier & ~STM32_TIM_DIER_IRQ_MASK;
            }
#endif
#if STM32_DMA_SUPPORTS_DMAMUX
            if (group.dma) {
                dmaSetRequestSource(group.dma, group.dma_up_channel);
            }
#endif
            chSysUnlock();
        }
    }
}

/*
  deallocate DMA channel
 */
void RCOutput::dma_deallocate(Shared_DMA *ctx)
{
    for (auto &group : pwm_group_list) {
        if (group.dma_handle == ctx && group.dma != nullptr) {
            chSysLock();
#if defined(STM32F1)
            // leaving the peripheral running on IOMCU plays havoc with the UART that is
            // also sharing this channel, we only turn it off rather than resetting so
            // that we don't have to worry about line modes etc
            if (group.pwm_started && group.dma_handle->is_shared()) {
                group.pwm_drv->tim->CR1   = 0;
                group.pwm_drv->tim->DIER  = 0;
            }
#endif
            dmaStreamFreeI(group.dma);
            group.dma = nullptr;
            chSysUnlock();
        }
    }
}
#endif // AP_HAL_SHARED_DMA_ENABLED

/*
  create a DSHOT 16 bit packet. Based on prepareDshotPacket from betaflight
 */
uint16_t RCOutput::create_dshot_packet(const uint16_t value, bool telem_request, bool bidir_telem)
{
    uint16_t packet = (value << 1);

    if (telem_request) {
        packet |= 1;
    }

    // compute checksum
    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (uint8_t i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    // trigger bi-dir dshot telemetry
    if (bidir_telem) {
        csum = ~csum;
    }

    // append checksum
    csum &= 0xf;
    packet = (packet << 4) | csum;

    return packet;
}

/*
  fill in a DMA buffer for dshot
  the buffer format is (stride is always 4 for 4 channels):
  [ch0b0][ch1b0][ch2b0][ch3b0][ch0b1][ch1b1][ch2b1][ch3b1]...[ch0bN][ch1bN][ch2bN][ch3bN]
  where N = dshot_bit_length - 1
  each value is a number of beats for the DMA engine to send in burst via DMAR to the 4 CCR registers
 */
void RCOutput::fill_DMA_buffer_dshot(dmar_uint_t *buffer, uint8_t stride, uint16_t packet, uint16_t clockmul)
{
    const dmar_uint_t DSHOT_MOTOR_BIT_0 = DSHOT_BIT_0_TICKS * clockmul;
    const dmar_uint_t DSHOT_MOTOR_BIT_1 = DSHOT_BIT_1_TICKS * clockmul;
    uint16_t i = 0;
    for (; i < dshot_pre; i++) {
        buffer[i * stride] = 0;
    }
    for (; i < 16 + dshot_pre; i++) {
        buffer[i * stride] = (packet & 0x8000) ? DSHOT_MOTOR_BIT_1 : DSHOT_MOTOR_BIT_0;
        packet <<= 1;
    }
    for (; i<dshot_bit_length; i++) {
        buffer[i * stride] = 0;
    }
}

/*
  send a set of DShot packets for a channel group
  This call be called in blocking mode from the timer, in which case it waits for the DMA lock.
  In normal operation it doesn't wait for the DMA lock.
 */
void RCOutput::dshot_send(pwm_group &group, uint64_t time_out_us)
{
#if HAL_DSHOT_ENABLED
    if (soft_serial_waiting() || !is_dshot_send_allowed(group.dshot_state)) {
        // doing serial output or DMAR input, don't send DShot pulses
        return;
    }

#if AP_HAL_SHARED_DMA_ENABLED
    // first make sure we have the DMA channel before anything else
    osalDbgAssert(!group.dma_handle->is_locked(), "DMA handle is already locked");
    group.dma_handle->lock();
#endif
    // if we are sharing UP channels then it might have taken a long time to get here,
    // if there's not enough time to actually send a pulse then cancel
#if AP_HAL_SHARED_DMA_ENABLED
    if (AP_HAL::micros64() + group.dshot_pulse_time_us > time_out_us) {
        group.dma_handle->unlock();
        return;
    }
#endif

    // only the timer thread releases the locks
    group.dshot_waiter = rcout_thread_ctx;
#ifdef HAL_WITH_BIDIR_DSHOT
    bdshot_prepare_for_next_pulse(group);
#endif
    bool safety_on = hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED;
#if !defined(IOMCU_FW)
    bool armed = hal.util->get_soft_armed();
#endif
    memset((uint8_t *)group.dma_buffer, 0, DSHOT_BUFFER_LENGTH);

    for (uint8_t i=0; i<4; i++) {
        uint8_t chan = group.chan[i];
        if (group.is_chan_enabled(i)) {
#ifdef HAL_WITH_BIDIR_DSHOT
            if (group.bdshot.enabled) {
                bdshot_decode_telemetry_from_erpm(group.bdshot.erpm[i], chan);
            }
#endif
            if (safety_on && !(safety_mask & (1U<<(chan+chan_offset)))) {
                // safety is on, don't output anything
                continue;
            }

            uint16_t pwm = period[chan];

            if (pwm == 0) {
                // no pwm, don't output anything
                continue;
            }

            const uint32_t chan_mask = (1U<<chan);

            pwm = constrain_int16(pwm, 1000, 2000);
            uint16_t value = MIN(2 * (pwm - 1000), 1999);

            if ((chan_mask & _reversible_mask) != 0) {
                // this is a DShot-3D output, map so that 1500 PWM is zero throttle reversed
                if (value < 1000) {
                    value = 1999 - value;
                } else if (value > 1000) {
                    value = value - 1000;
                } else {
                    // mid-throttle is off
                    value = 0;
                }
            }

            // dshot values are from 48 to 2047. 48 means off.
            if (value != 0) {
                value += DSHOT_ZERO_THROTTLE;
            }
#if !defined(IOMCU_FW)
            if (!armed) {
                // when disarmed we always send a zero value
                value = 0;
            }
#endif
            // according to sskaug requesting telemetry while trying to arm may interfere with the good frame calc
            bool request_telemetry = telem_request_mask & chan_mask;
            uint16_t packet = create_dshot_packet(value, request_telemetry,
#ifdef HAL_WITH_BIDIR_DSHOT
             group.bdshot.enabled
#else
             false
#endif
             );
            if (request_telemetry) {
                telem_request_mask &= ~chan_mask;
            }
            fill_DMA_buffer_dshot(group.dma_buffer + i, 4, packet, group.bit_width_mul);
        }
    }

    chEvtGetAndClearEvents(group.dshot_event_mask | DSHOT_CASCADE);
    // start sending the pulses out
    send_pulses_DMAR(group, DSHOT_BUFFER_LENGTH);
#endif // HAL_DSHOT_ENABLED
}

/*
  send a set of Serial LED packets for a channel group
  return true if send was successful
  called from led thread
 */
#if HAL_SERIALLED_ENABLED
bool RCOutput::serial_led_send(pwm_group &group)
{
    if (!group.serial_led_pending || !is_led_protocol(group.current_mode)) {
        return true;
    }

#if HAL_DSHOT_ENABLED
    if (soft_serial_waiting() || !is_dshot_send_allowed(group.dshot_state)
        || AP_HAL::micros64() - group.last_dmar_send_us < (group.dshot_pulse_time_us + 50)) {
        // doing serial output or DMAR input, don't send DShot pulses
        return false;
    }

    // first make sure we have the DMA channel before anything else
    group.dma_handle->lock();

    {
        WITH_SEMAPHORE(group.serial_led_mutex);

        group.serial_led_pending = false;
        group.prepared_send = false;

        // fill the DMA buffer while we have the lock
        fill_DMA_buffer_serial_led(group);
    }

    group.dshot_waiter = led_thread_ctx;

    chEvtGetAndClearEvents(group.dshot_event_mask | DSHOT_CASCADE);

    // start sending the pulses out
    send_pulses_DMAR(group, group.dma_buffer_len);
#endif // HAL_DSHOT_ENABLED
    return true;
}
#endif // HAL_SERIALLED_ENABLED

/*
  send a series of pulses for a group using DMAR. Pulses must have
  been encoded into the group dma_buffer with interleaving for the 4
  channels in the group
 */
void RCOutput::send_pulses_DMAR(pwm_group &group, uint32_t buffer_length)
{
#if HAL_DSHOT_ENABLED
    osalDbgAssert(group.dma && group.dma_buffer, "DMA structures are corrupt");
    /*
      The DMA approach we are using is based on the DMAR method from
      betaflight. We use the TIMn_UP DMA channel for the timer, and
      setup an interleaved set of pulse durations, with a stride of 4
      (for the 4 channels). We use the DMAR register to point the DMA
      engine at the 4 CCR registers of the timer, so it fills in the
      pulse widths for each timer in turn. This means we only use a
      single DMA channel for groups of 4 timer channels. See the "DMA
      address for full transfer TIMx_DMAR" section of the
      datasheet. Many thanks to the betaflight developers for coming
      up with this great method.
     */
#ifdef HAL_GPIO_LINE_GPIO54
    TOGGLE_PIN_DEBUG(54);
#endif
#if STM32_DMA_SUPPORTS_DMAMUX
    dmaSetRequestSource(group.dma, group.dma_up_channel);
#endif
    dmaStreamSetPeripheral(group.dma, &(group.pwm_drv->tim->DMAR));
    stm32_cacheBufferFlush(group.dma_buffer, buffer_length);
    dmaStreamSetMemory0(group.dma, group.dma_buffer);
    dmaStreamSetTransactionSize(group.dma, buffer_length / sizeof(dmar_uint_t));
#if STM32_DMA_ADVANCED
    dmaStreamSetFIFO(group.dma, STM32_DMA_FCR_DMDIS | STM32_DMA_FCR_FTH_FULL);
#endif
    dmaStreamSetMode(group.dma,
                     STM32_DMA_CR_CHSEL(group.dma_up_channel) |
                     STM32_DMA_CR_DIR_M2P |
#if defined(STM32F1)
#ifdef HAL_WITH_BIDIR_DSHOT
                     STM32_DMA_CR_PSIZE_HWORD |
                     STM32_DMA_CR_MSIZE_HWORD |
#else
                     STM32_DMA_CR_PSIZE_WORD |
                     STM32_DMA_CR_MSIZE_BYTE |
#endif

#else
                     STM32_DMA_CR_PSIZE_WORD |
                     STM32_DMA_CR_MSIZE_WORD |
#endif
                     STM32_DMA_CR_MINC | STM32_DMA_CR_PL(3) |
                     STM32_DMA_CR_TEIE | STM32_DMA_CR_TCIE);

    // setup for burst strided transfers into the timers 4 CCR registers
    const uint8_t ccr_ofs = offsetof(stm32_tim_t, CCR)/4;
    // burst address (BA) of the CCR register, burst length (BL) of 4 (0b11)
    group.pwm_drv->tim->DCR = STM32_TIM_DCR_DBA(ccr_ofs) | STM32_TIM_DCR_DBL(3);
    group.dshot_state = DshotState::SEND_START;
#ifdef HAL_GPIO_LINE_GPIO54
    TOGGLE_PIN_DEBUG(54);
#endif

    dmaStreamEnable(group.dma);
    // record when the transaction was started
    group.last_dmar_send_us = AP_HAL::micros64();
#endif // HAL_DSHOT_ENABLED
}

/*
  unlock DMA channel after a dshot send completes and no return value is expected
 */
__RAMFUNC__ void RCOutput::dma_unlock(virtual_timer_t* vt, void *p)
{
    chSysLockFromISR();
    pwm_group *group = (pwm_group *)p;

    group->dshot_state = DshotState::IDLE;
    if (group->dshot_waiter != nullptr) {
        // tell the waiting process we've done the DMA. Note that
        // dshot_waiter can be null if we have cancelled the send
        chEvtSignalI(group->dshot_waiter, group->dshot_event_mask);
    }
    chSysUnlockFromISR();
}

#ifndef HAL_WITH_BIDIR_DSHOT
/*
  DMA interrupt handler. Used to mark DMA completed for DShot
 */
__RAMFUNC__ void RCOutput::dma_up_irq_callback(void *p, uint32_t flags)
{
    pwm_group *group = (pwm_group *)p;
    chSysLockFromISR();
    dmaStreamDisable(group->dma);
#if HAL_SERIAL_ESC_COMM_ENABLED
    if (group->in_serial_dma && soft_serial_waiting()) {
        // tell the waiting process we've done the DMA
        chEvtSignalI(irq.waiter, serial_event_mask);
    } else
#endif
    {
        // this prevents us ever having two dshot pulses too close together
        // dshot mandates a minimum pulse separation of 40us, WS2812 mandates 50us so we
        // pick the higher value
        chVTSetI(&group->dma_timeout, chTimeUS2I(50), dma_unlock, p);
    }
    chSysUnlockFromISR();
}
#endif

/*
  Cancel a DMA transaction in progress
 */
void RCOutput::dma_cancel(pwm_group& group)
{
    chSysLock();
    dmaStreamDisable(group.dma);
#ifdef HAL_WITH_BIDIR_DSHOT
    if (group.ic_dma_enabled() && !group.has_shared_ic_up_dma()) {
        dmaStreamDisable(group.bdshot.ic_dma[group.bdshot.curr_telem_chan]);
    }
#if STM32_DMA_SUPPORTS_DMAMUX
    // the DMA request source has been switched by the receive path, so reinstate the correct one
    if (group.dshot_state == DshotState::RECV_START && group.has_shared_ic_up_dma()) {
        dmaSetRequestSource(group.dma, group.dma_up_channel);
    }
#endif
#endif
    // normally the CCR registers are reset by the final 0 in the DMA buffer
    // since we are cancelling early they need to be reset to avoid infinite pulses
    for (uint8_t i = 0; i < 4; i++) {
        if (group.chan[i] != CHAN_DISABLED) {
            group.pwm_drv->tim->CCR[i] = 0;
        }
    }
    chVTResetI(&group.dma_timeout);
    chEvtGetAndClearEventsI(group.dshot_event_mask | DSHOT_CASCADE);

    group.dshot_state = DshotState::IDLE;
    chSysUnlock();
}

/*
  setup for serial output to an ESC using the given
  baudrate. Assumes 1 start bit, 1 stop bit, LSB first and 8
  databits. This is used for passthrough ESC configuration and
  firmware flashing

  While serial output is active normal output to the channel group is
  suspended.
*/
#if HAL_SERIAL_ESC_COMM_ENABLED
bool RCOutput::serial_setup_output(uint8_t chan, uint32_t baudrate, uint32_t chanmask)
{
    // account for IOMCU channels
    chan -= chan_offset;
    chanmask >>= chan_offset;
    pwm_group *new_serial_group = nullptr;

    // find the channel group
    for (auto &group : pwm_group_list) {
        if (group.current_mode == MODE_PWM_BRUSHED) {
            // can't do serial output with brushed motors
            continue;
        }
        if (group.ch_mask & (1U<<chan)) {
            new_serial_group = &group;
            for (uint8_t j=0; j<4; j++) {
                if (group.chan[j] == chan) {
                    group.serial.chan = j;
                }
            }
            break;
        }
    }

    if (!new_serial_group) {
        if (in_soft_serial()) {
            // shutdown old group
            serial_end();
        }
        return false;
    }

    // stop further dshot output before we reconfigure the DMA
    serial_group = new_serial_group;

    // setup the groups for serial output. We ask for a bit width of 1, which gets modified by the
    // we setup all groups so they all are setup with the right polarity, and to make switching between
    // channels in blheli pass-thru fast
    for (auto &group : pwm_group_list) {
        if (group.ch_mask & chanmask) {
            if (!setup_group_DMA(group, baudrate, 10, false, DSHOT_BUFFER_LENGTH, 10, false)) {
                serial_end();
                return false;
            }
        }
    }

    // run the thread doing serial IO at highest priority. This is needed to ensure we don't
    // lose bytes when we switch between output and input
    serial_thread = chThdGetSelfX();
    serial_priority  = chThdGetSelfX()->realprio;
    chThdSetPriority(HIGHPRIO);

    // remember the bit period for serial_read_byte()
    serial_group->serial.bit_time_us = 1000000UL / baudrate;

    // remember the thread that set things up. This is also used to
    // mark the group as doing serial output, so normal output is
    // suspended
    irq.waiter = chThdGetSelfX();

    return true;
}


/*
  fill in a DMA buffer for a serial byte, assuming 1 start bit and 1 stop bit
 */
void RCOutput::fill_DMA_buffer_byte(dmar_uint_t *buffer, uint8_t stride, uint8_t b, uint32_t bitval)
{
    const dmar_uint_t BIT_0 = bitval;
    const dmar_uint_t BIT_1 = 0;

    // start bit
    buffer[0] = BIT_0;

    // stop bit
    buffer[9*stride] = BIT_1;

    // 8 data bits
    for (uint8_t i = 0; i < 8; i++) {
        buffer[(1 + i) * stride] = (b & 1) ? BIT_1 : BIT_0;
        b >>= 1;
    }
}


/*
  send one serial byte, blocking call, should be called with the DMA lock held
*/
bool RCOutput::serial_write_byte(uint8_t b)
{
    chEvtGetAndClearEvents(serial_event_mask);

    fill_DMA_buffer_byte(serial_group->dma_buffer+serial_group->serial.chan, 4, b, serial_group->bit_width_mul*10);

    serial_group->in_serial_dma = true;

    // start sending the pulses out
    send_pulses_DMAR(*serial_group, 10*4*sizeof(uint32_t));

    // wait for the event
    eventmask_t mask = chEvtWaitAnyTimeout(serial_event_mask, chTimeMS2I(2));

    serial_group->in_serial_dma = false;

    return (mask & serial_event_mask) != 0;
}

/*
  send a set of serial bytes, blocking call
*/
bool RCOutput::serial_write_bytes(const uint8_t *bytes, uint16_t len)
{
#if HAL_DSHOT_ENABLED
    if (!in_soft_serial()) {
        return false;
    }
    serial_group->dma_handle->lock();
    memset(serial_group->dma_buffer, 0, DSHOT_BUFFER_LENGTH);
    while (len--) {
        if (!serial_write_byte(*bytes++)) {
            serial_group->dma_handle->unlock();
            return false;
        }
    }

    // add a small delay for last word of output to have completely
    // finished
    hal.scheduler->delay_microseconds(25);

    serial_group->dma_handle->unlock();
    return true;
#else
    return false;
#endif // DISABLE_DSHOT
}

/*
  irq handler for bit transition in serial_read_byte()
  This implements a one byte soft serial reader
 */
void RCOutput::serial_bit_irq(void)
{
    uint16_t now = AP_HAL::micros16();
    uint8_t bit = palReadLine(irq.line);
    bool send_signal = false;

#if RCOU_SERIAL_TIMING_DEBUG
    palWriteLine(HAL_GPIO_LINE_GPIO55, bit);
#endif

    if (irq.nbits == 0 || bit == irq.last_bit) {
        // start of byte, should be low
        if (bit != 0) {
            irq.byteval = 0x200;
            send_signal = true;
        } else {
            irq.nbits = 1;
            irq.byte_start_tick = now;
            irq.bitmask = 0;
        }
    } else {
        uint16_t dt = now - irq.byte_start_tick;
        uint8_t bitnum = (dt+(irq.bit_time_tick/2)) / irq.bit_time_tick;

        if (bitnum > 10) {
            bitnum = 10;
        }
        if (!bit) {
            // set the bits that we've processed
            irq.bitmask |= ((1U<<bitnum)-1) & ~((1U<<irq.nbits)-1);
        }
        irq.nbits = bitnum;

        if (irq.nbits == 10) {
            send_signal = true;
            irq.byteval = irq.bitmask & 0x3FF;
            irq.bitmask = 0;
            irq.nbits = 1;
            irq.byte_start_tick = now;
        }
    }
    irq.last_bit = bit;

    if (send_signal) {
        chSysLockFromISR();
        chVTResetI(&irq.serial_timeout);
        chEvtSignalI(irq.waiter, serial_event_mask);
        chSysUnlockFromISR();
    }
}

/*
  timeout a byte read
 */
void RCOutput::serial_byte_timeout(virtual_timer_t* vt, void *ctx)
{
    chSysLockFromISR();
    irq.timed_out = true;
    chEvtSignalI((thread_t *)ctx, serial_event_mask);
    chSysUnlockFromISR();
}

/*
  read a byte from a port, using serial parameters from serial_setup_output()
*/
bool RCOutput::serial_read_byte(uint8_t &b)
{
    irq.timed_out = false;
    chVTSet(&irq.serial_timeout, chTimeMS2I(10), serial_byte_timeout, irq.waiter);
    bool timed_out = ((chEvtWaitAny(serial_event_mask) & serial_event_mask) == 0) || irq.timed_out;

    uint16_t byteval = irq.byteval;

    if (timed_out) {
        // we can accept a byte with a timeout if the last bit was 1
        // and the start bit is set correctly
        if (irq.last_bit == 0) {
            return false;
        }
        byteval = irq.bitmask | 0x200;
    }

    if ((byteval & 0x201) != 0x200) {
        // wrong start/stop bits
        return false;
    }

    b = uint8_t(byteval>>1);
    return true;
}

/*
  read a byte from a port, using serial parameters from serial_setup_output()
*/
uint16_t RCOutput::serial_read_bytes(uint8_t *buf, uint16_t len)
{
    if (!in_soft_serial()) {
        return 0;
    }
    pwm_group &group = *serial_group;
    const ioline_t line = group.pal_lines[group.serial.chan];
    // keep speed low to avoid noise when switching between input and output
#ifndef PAL_STM32_OSPEED_LOWEST
    // for GPIOv3
    uint32_t gpio_mode = PAL_STM32_MODE_INPUT | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_PUPDR_PULLUP | PAL_STM32_OSPEED_LOW;
#else
    uint32_t gpio_mode = PAL_STM32_MODE_INPUT | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_PUPDR_PULLUP | PAL_STM32_OSPEED_LOWEST;
#endif
    // restore the line to what it was before
    iomode_t restore_mode = palReadLineMode(line);
    uint16_t i = 0;

#if RCOU_SERIAL_TIMING_DEBUG
    hal.gpio->pinMode(54, 1);
    hal.gpio->pinMode(55, 1);
#endif

    // assume GPIO mappings for PWM outputs start at 50
    palSetLineMode(line, gpio_mode);

    chVTObjectInit(&irq.serial_timeout);
    chEvtGetAndClearEvents(serial_event_mask);

    irq.line = group.pal_lines[group.serial.chan];
    irq.nbits = 0;
    irq.bitmask = 0;
    irq.byteval = 0;
    irq.bit_time_tick = serial_group->serial.bit_time_us;
    irq.last_bit = 0;
    irq.waiter = chThdGetSelfX();

#if RCOU_SERIAL_TIMING_DEBUG
    palWriteLine(HAL_GPIO_LINE_GPIO54, 1);
#endif

    if (!((GPIO *)hal.gpio)->_attach_interrupt(line, serial_bit_irq, AP_HAL::GPIO::INTERRUPT_BOTH)) {
#if RCOU_SERIAL_TIMING_DEBUG
        palWriteLine(HAL_GPIO_LINE_GPIO54, 0);
#endif
        return false;
    }

    for (i=0; i<len; i++) {
        if (!serial_read_byte(buf[i])) {
            break;
        }
    }

    ((GPIO *)hal.gpio)->_attach_interrupt(line, nullptr, 0);
    irq.waiter = nullptr;

    palSetLineMode(line, restore_mode);
#if RCOU_SERIAL_TIMING_DEBUG
    palWriteLine(HAL_GPIO_LINE_GPIO54, 0);
#endif
    return i;
}

/*
  end serial output
*/
void RCOutput::serial_end(void)
{
    if (in_soft_serial()) {
        if (serial_thread == chThdGetSelfX()) {
            chThdSetPriority(serial_priority);
            serial_thread = nullptr;
        }
        irq.waiter = nullptr;
        for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
            pwm_group &group = pwm_group_list[i];
            set_group_mode(group);
            set_freq_group(group);
        }
    }
    serial_group = nullptr;
}
#endif // HAL_SERIAL_ESC_COMM_ENABLED

/*
  get safety switch state for Util.cpp
 */
AP_HAL::Util::safety_state RCOutput::_safety_switch_state(void)
{
#if HAL_WITH_IO_MCU
    if (iomcu_enabled) {
        safety_state = iomcu.get_safety_switch_state();
    }
#endif
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
#if HAL_WITH_IO_MCU
    if (iomcu_enabled) {
        return iomcu.force_safety_on();
    }
#endif
    safety_state = AP_HAL::Util::SAFETY_DISARMED;
    return true;
}

/*
  force the safety switch off, enabling PWM output from the IO board
*/
void RCOutput::force_safety_off(void)
{
#if HAL_WITH_IO_MCU
    if (iomcu_enabled) {
        iomcu.force_safety_off();
        return;
    }
#endif
    safety_state = AP_HAL::Util::SAFETY_ARMED;
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
    // handle safety button
    bool safety_pressed = palReadLine(HAL_GPIO_PIN_SAFETY_IN);
    if (safety_pressed) {
        AP_BoardConfig *brdconfig = AP_BoardConfig::get_singleton();
        if (safety_press_count < 255) {
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
#elif HAL_WITH_IO_MCU
    safety_state = _safety_switch_state();
#endif

#if HAL_WITH_IO_MCU
    // regardless of if we have a FMU safety pin, if we have an IOMCU we need
    // to pass the BRD_SAFETY_MASK to the IOMCU
    iomcu.set_safety_mask(safety_mask);
#endif
    
#ifdef HAL_GPIO_PIN_LED_SAFETY
    led_counter = (led_counter+1) % 16;
    const uint16_t led_pattern = safety_state==AP_HAL::Util::SAFETY_DISARMED?0x5500:0xFFFF;
    palWriteLine(HAL_GPIO_PIN_LED_SAFETY, (led_pattern & (1U << led_counter))?0:1);
#endif
}

/*
  set PWM to send to a set of channels if the FMU firmware dies
*/
void RCOutput::set_failsafe_pwm(uint32_t chmask, uint16_t period_us)
{
#if HAL_WITH_IO_MCU
    if (iomcu_enabled) {
        iomcu.set_failsafe_pwm(chmask, period_us);
    }
#endif
}

/*
    returns the bitrate in Hz of the given output_mode
*/
uint32_t RCOutput::protocol_bitrate(const enum output_mode mode)
{
    switch (mode) {
    case MODE_PWM_DSHOT150:
        return 150000;
    case MODE_PWM_DSHOT300:
        return 300000;
    case MODE_PWM_DSHOT600:
        return 600000;
    case MODE_PWM_DSHOT1200:
        return 1200000;
    case MODE_NEOPIXEL:
    case MODE_NEOPIXELRGB:
        return 800000;
    case MODE_PROFILED:
        return 1500000; // experiment winding this up 3000000 max from data sheet
    default:
        // use 1 to prevent a possible divide-by-zero
        return 1;
    }
}

/*
  setup serial led output for a given channel number, with
  the given max number of LEDs in the chain.
*/
#if HAL_SERIALLED_ENABLED
bool RCOutput::set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode, uint32_t clock_mask)
{
    if (!_initialised || num_leds == 0) {
        return false;
    }

    uint8_t i = 0;
    pwm_group *grp = find_chan(chan, i);
    if (!grp) {
        return false;
    }

    WITH_SEMAPHORE(grp->serial_led_mutex);

    // group is already setup correctly
    if ((grp->serial_nleds >= num_leds) && (mode == grp->current_mode)) {
        return true;
    }

    // we cant add more or change the type after the first setup
    if (is_led_protocol(grp->current_mode)) {
        return false;
    }

    switch (mode) {

        case MODE_NEOPIXEL:
        case MODE_NEOPIXELRGB: {
            grp->serial_nleds = MAX(num_leds, grp->serial_nleds);
            grp->led_mode = mode;
            return true;
        }
        case MODE_PROFILED: {
            // ProfiLED requires two dummy LED's to mark end of transmission
            grp->serial_nleds = MAX(num_leds + 2, grp->serial_nleds);
            grp->led_mode = MODE_PROFILED;

            // Enable any clock channels in the same group
            grp->clock_mask = 0;
            for (uint8_t j = 0; j < 4; j++) {
                if ((clock_mask & (1U<<(grp->chan[j] + chan_offset))) != 0) {
                    grp->clock_mask |= 1U<<j;
                }
            }
            return true;
        }
        default:
            grp->serial_nleds = 0;
            grp->led_mode = MODE_PWM_NONE;
            return false;
    }

}

#pragma GCC push_options
#pragma GCC optimize("O2")
// Fill the group DMA buffer with data to be output
void RCOutput::fill_DMA_buffer_serial_led(pwm_group& group)
{
    memset(group.dma_buffer, 0, group.dma_buffer_len);
    for (uint8_t j = 0; j < 4; j++) {
        if (group.serial_led_data[j] == nullptr) {
            // something very bad has happended
            continue;
        }

        if (group.current_mode == MODE_PROFILED && (group.clock_mask & 1U<<j) != 0) {
            // output clock channel
            for (uint8_t i = 0; i < group.serial_nleds; i++) {
                _set_profiled_clock(&group, j, i);
            }
            continue;
        }

        for (uint8_t i = 0; i < group.serial_nleds; i++) {
            const SerialLed& led = group.serial_led_data[j][i];
            switch (group.current_mode) {
                case MODE_NEOPIXEL:
                    _set_neopixel_rgb_data(&group, j, i, led.red, led.green, led.blue);
                    break;
                case MODE_NEOPIXELRGB:
                    _set_neopixel_rgb_data(&group, j, i, led.green, led.red, led.blue);
                    break;
                case MODE_PROFILED: {
                    if (i < group.serial_nleds - 2) {
                        _set_profiled_rgb_data(&group, j, i, led.red, led.green, led.blue);
                    } else {
                        _set_profiled_blank_frame(&group, j, i);
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }
}

/*
  setup neopixel (WS2812B) output data for a given output channel
  and a LED number. LED -1 is all LEDs
*/
void RCOutput::_set_neopixel_rgb_data(pwm_group *grp, uint8_t idx, uint8_t led, uint8_t red, uint8_t green, uint8_t blue)
{
    const uint8_t pad_start_bits = 1;
    const uint8_t neopixel_bit_length = 24;
    const uint8_t stride = 4;
    dmar_uint_t *buf = grp->dma_buffer + (led * neopixel_bit_length + pad_start_bits) * stride + idx;
    uint32_t bits = (green<<16) | (red<<8) | blue;
    const uint32_t BIT_0 = NEOP_BIT_0_TICKS * grp->bit_width_mul;
    const uint32_t BIT_1 = NEOP_BIT_1_TICKS * grp->bit_width_mul;
    for (uint16_t b=0; b < 24; b++) {
        buf[b * stride] = (bits & 0x800000) ? BIT_1 : BIT_0;
        bits <<= 1;
    }
}

/*
  ProfiLED frame for a given output channel
  channel is active high and bits inverted to get clock rising edge away from data rising edge
*/
void RCOutput::_set_profiled_rgb_data(pwm_group *grp, uint8_t idx, uint8_t led, uint8_t red, uint8_t green, uint8_t blue)
{
    const uint8_t pad_start_bits = 1;
    const uint8_t bit_length = 25;
    const uint8_t stride = 4;
    dmar_uint_t *buf = grp->dma_buffer + (led * bit_length + pad_start_bits) * stride + idx;
    uint32_t bits = 0x1000000 | (blue<<16) | (red<<8) | green;
    const uint32_t BIT_1 = PROFI_BIT_1_TICKS * grp->bit_width_mul;
    for (uint16_t b=0; b < bit_length; b++) {
        buf[b * stride] = (bits & 0x1000000) ? 0 : BIT_1;
        bits <<= 1;
    }
}

/*
  ProfiLED blank frame for a given output channel
  channel is active high and bits inverted to get clock rising edge away from data rising edge
*/
void RCOutput::_set_profiled_blank_frame(pwm_group *grp, uint8_t idx, uint8_t led)
{
    const uint8_t pad_start_bits = 1;
    const uint8_t bit_length = 25;
    const uint8_t stride = 4;
    dmar_uint_t *buf = grp->dma_buffer + (led * bit_length + pad_start_bits) * stride + idx;
    const uint32_t BIT_1 = PROFI_BIT_1_TICKS * grp->bit_width_mul;
    for (uint16_t b=0; b < bit_length; b++) {
        buf[b * stride] = BIT_1;
    }
}

/*
  setup ProfiLED clock frame for a given output channel
*/
void RCOutput::_set_profiled_clock(pwm_group *grp, uint8_t idx, uint8_t led)
{
    const uint8_t pad_start_bits = 1;
    const uint8_t bit_length = 25;
    const uint8_t stride = 4;
    dmar_uint_t *buf = grp->dma_buffer + (led * bit_length + pad_start_bits) * stride + idx;
    const uint32_t BIT_1 = PROFI_BIT_0_TICKS * grp->bit_width_mul;
    for (uint16_t b=0; b < bit_length; b++) {
        buf[b * stride] = BIT_1;
    }
}
#pragma GCC pop_options

/*
  setup serial LED output data for a given output channel
  and a LED number. LED -1 is all LEDs
*/
bool RCOutput::set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue)
{
    if (!_initialised) {
        return false;
    }

    uint8_t i = 0;
    pwm_group *grp = find_chan(chan, i);

    if (!grp) {
        return false;
    }

    if (grp->serial_led_pending) {
        // dont allow setting new data if a send is pending
        // would result in a fight over the mutex
        return false;
    }

    WITH_SEMAPHORE(grp->serial_led_mutex);

    if (grp->serial_nleds == 0 || led >= grp->serial_nleds) {
        return false;
    }

    if ((grp->current_mode != grp->led_mode) && is_led_protocol(grp->led_mode)) {
        // Arrays have not yet been setup, do it now
        for (uint8_t j = 0; j < 4; j++) {
            delete[] grp->serial_led_data[j];
            grp->serial_led_data[j] = nullptr;
            grp->serial_led_data[j] = new SerialLed[grp->serial_nleds];
            if (grp->serial_led_data[j] == nullptr) {
                // if allocation failed clear all memory
                 for (uint8_t k = 0; k < 4; k++) {
                    delete[] grp->serial_led_data[k];
                    grp->serial_led_data[k] = nullptr;
                }
                grp->led_mode = MODE_PWM_NONE;
                grp->serial_nleds = 0;
                return false;
            }
        }

        // at this point the group led data is all setup but the dma buffer still needs to be resized
        set_output_mode(1U<<chan, grp->led_mode);

        if (grp->current_mode != grp->led_mode) {
            // Failed to set output mode
            grp->led_mode = MODE_PWM_NONE;
            grp->serial_nleds = 0;
            return false;
        }

    } else if (!is_led_protocol(grp->current_mode)) {
        return false;
    }

    if (led == -1) {
        grp->prepared_send = true;
        for (uint8_t n=0; n<grp->serial_nleds; n++) {
            serial_led_set_single_rgb_data(*grp, i, n, red, green, blue);
        }
        return true;
    }

    // if not ouput clock and trailing frames, run through all LED's to do it now
    if (!grp->prepared_send) {
        grp->prepared_send = true;
        for (uint8_t n=0; n<grp->serial_nleds; n++) {
            serial_led_set_single_rgb_data(*grp, i, n, 0, 0, 0);
        }
    }
    serial_led_set_single_rgb_data(*grp, i, uint8_t(led), red, green, blue);

    return true;
}

/*
  setup serial LED output data for a given output channel
  and a LED number. LED -1 is all LEDs
*/
void RCOutput::serial_led_set_single_rgb_data(pwm_group& group, uint8_t idx, uint8_t led, uint8_t red, uint8_t green, uint8_t blue)
{
    switch (group.current_mode) {
        case MODE_PROFILED:
        case MODE_NEOPIXEL:
        case MODE_NEOPIXELRGB:
            group.serial_led_data[idx][led].red = red;
            group.serial_led_data[idx][led].green = green;
            group.serial_led_data[idx][led].blue = blue;
            break;
        default:
            break;
    }
}

/*
  trigger send of serial led data for one group
*/
bool RCOutput::serial_led_send(const uint16_t chan)
{
    if (!_initialised) {
        return false;
    }

    if (led_thread_ctx == nullptr) {
        return false;
    }

    uint8_t i;
    pwm_group *grp = find_chan(chan, i);
    if (!grp) {
        return false;
    }

    WITH_SEMAPHORE(grp->serial_led_mutex);

    if (grp->serial_nleds == 0 || !is_led_protocol(grp->current_mode)) {
        return false;
    }

    if (grp->prepared_send) {
        grp->serial_led_pending = true;
        serial_led_pending = true;
        chEvtSignal(led_thread_ctx, EVT_LED_SEND);
    }

    return true;
}
#endif // HAL_SERIALLED_ENABLED

void RCOutput::timer_info(ExpandingString &str)
{
    // a header to allow for machine parsers to determine format
    str.printf("TIMERV1\n");
#if HAL_DSHOT_ENABLED
    for (auto &group : pwm_group_list) {
        uint32_t target_freq;
        bool at_least_freq;
#if HAL_SERIAL_ESC_COMM_ENABLED
        if (&group == serial_group) {
            target_freq = 19200 * 10;
        } else
#endif // HAL_SERIAL_ESC_COMM_ENABLED
        if (is_dshot_protocol(group.current_mode)) {
            target_freq = protocol_bitrate(group.current_mode) * DSHOT_BIT_WIDTH_TICKS;
            if (_dshot_esc_type == DSHOT_ESC_BLHELI_S || _dshot_esc_type == DSHOT_ESC_BLHELI_EDT_S) {
                at_least_freq = true;
            }
        } else {
            target_freq = protocol_bitrate(group.current_mode) * NEOP_BIT_WIDTH_TICKS;
        }
        const uint32_t prescaler = calculate_bitrate_prescaler(group.pwm_drv->clock, target_freq, at_least_freq);
        str.printf("TIM%-2u CLK=%4uMhz MODE=%5s FREQ=%8u TGT=%8u\n", group.timer_id, unsigned(group.pwm_drv->clock / 1000000),
            get_output_mode_string(group.current_mode),
            unsigned(group.pwm_drv->clock / (prescaler + 1)), unsigned(target_freq));
    }
#endif
}

#endif // HAL_USE_PWM
