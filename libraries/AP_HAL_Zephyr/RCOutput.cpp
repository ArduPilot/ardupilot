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
 * Code by @davidbuzz and Claude
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "RCOutput.h"
#include <AP_Common/ExpandingString.h>   // timer_info() writes into one
#include <stdlib.h>                      // strtoul() for the timer number

#ifdef __ZEPHYR__
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>   /* _report_pwm_set() */
#endif

#include <AP_BoardConfig/AP_BoardConfig.h>
#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#if AP_ZEPHYR_DSHOT_ENABLED
/* Pad switching lives in zephyr/src/ap_rcout_pinmux.c - a C file because
   the NXP pinctrl_soc.h initializers PINCTRL_DT_DEFINE expands don't
   compile as C++ (-Wnarrowing); see that file's header comment. */
#include <ap_rcout_pinmux.h>
#include <zephyr/drivers/misc/nxp_flexio_dshot/nxp_flexio_dshot.h>
#include <AP_Math/AP_Math.h>
#endif

using namespace Zephyr;

extern const AP_HAL::HAL &hal;

void RCOutput::init()
{
#if HAL_WITH_IO_MCU && AP_ZEPHYR_IOMCU_ENABLED
    /* Not gated on AP_BoardConfig::io_enabled(): this runs during early HAL
       init, before AP_Param::load_all(), so io_enable still holds its table
       default of 1 and BRD_IO_ENABLE cannot turn it off here. A compile-time
       gate is the only one that actually holds at this point. */
    _iomcu_enabled = true;
    _chan_offset = 8;   /* the IO co-processor owns SERVO1-8 */
    iomcu.init();
#endif

    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        _enabled[i] = false;
        _period_us[i] = 1000;  // 1ms = min servo pulse
        _freq_hz[i] = 50;
        _dirty[i] = false;
    }

#ifdef __ZEPHYR__
#if DT_NODE_HAS_STATUS(DT_NODELABEL(tim1_pwm), okay)
    /* STM32 (CubeOrangeZephyr): FMU AUX1-6, ChibiOS CubeOrange map —
       PWM1=TIM1_CH4, PWM2=TIM1_CH3, PWM3=TIM1_CH2, PWM4=TIM1_CH1,
       PWM5=TIM4_CH2, PWM6=TIM4_CH3. pwm_stm32 channels are 1-based. */
    const struct device *tim1 = DEVICE_DT_GET(DT_NODELABEL(tim1_pwm));
#if DT_NODE_HAS_STATUS(DT_NODELABEL(tim4_pwm), okay)
    const struct device *tim4 = DEVICE_DT_GET(DT_NODELABEL(tim4_pwm));
#else
    const struct device *tim4 = nullptr;
#endif
    /* These are the FMU's own AUX pins. On a board with an IOMCU they are
       SERVO9-14, because the IO co-processor owns SERVO1-8 - so they go in at
       _chan_offset, and channels below it are left with a null device and
       routed to the IOMCU by write(). Without an IOMCU _chan_offset is 0 and
       they are SERVO1-6, as before. */
#define ZTIM_PSC(label) \
    (DT_NODE_HAS_PROP(DT_NODELABEL(label), st_prescaler) \
        ? DT_PROP_OR(DT_NODELABEL(label), st_prescaler, 0) : 0)
    _map[_chan_offset + 0] = { tim1, 4, 1, ZTIM_PSC(timers1) };
    _map[_chan_offset + 1] = { tim1, 3, 1, ZTIM_PSC(timers1) };
    _map[_chan_offset + 2] = { tim1, 2, 1, ZTIM_PSC(timers1) };
    _map[_chan_offset + 3] = { tim1, 1, 1, ZTIM_PSC(timers1) };
    _map[_chan_offset + 4] = { tim4, 2, 4, ZTIM_PSC(timers4) };
    _map[_chan_offset + 5] = { tim4, 3, 4, ZTIM_PSC(timers4) };
#undef ZTIM_PSC

    /* channels are usable individually; _apply_channel() skips any
       with a missing/not-ready device */
    _map_ready = true;

#if defined(CONFIG_AP_RCIN_PWM_CAPTURE) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), pwms)
    /* RC PPM hardware capture claims one channel of one timer, but pwm_stm32.c gates
     * OUTPUT on the whole timer, so every channel on it is refused. */
    const struct device *cap_dev = DEVICE_DT_GET(DT_PWMS_CTLR(DT_PATH(zephyr_user)));
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (_map[i].dev == cap_dev) {
            _map[i] = { nullptr, 0 };
        }
    }
#endif
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(flexpwm1_pwm0), okay)
    /* NXP eFlexPWM (mr_vmu_rt1176): one FMU output per FlexPWM submodule, because
     * each FMU_CH net is on the *_a pad of a different submodule. */
#define ZPWM_DEV(label)                                                        \
    (DT_NODE_HAS_STATUS(DT_NODELABEL(label), okay)                             \
        ? DEVICE_DT_GET_OR_NULL(DT_NODELABEL(label)) : nullptr)

    _map[0]  = { ZPWM_DEV(flexpwm1_pwm0), 0, 1, 0 };   /* FMU_CH1  EMC_B1_23 */
    _map[1]  = { ZPWM_DEV(flexpwm1_pwm1), 0, 1, 0 };   /* FMU_CH2  EMC_B1_25 */
    _map[2]  = { ZPWM_DEV(flexpwm1_pwm2), 0, 1, 0 };   /* FMU_CH3  EMC_B1_27 */
    _map[3]  = { ZPWM_DEV(flexpwm2_pwm0), 0, 2, 0 };   /* FMU_CH4  EMC_B1_06 */
    _map[4]  = { ZPWM_DEV(flexpwm2_pwm1), 0, 2, 0 };   /* FMU_CH5  EMC_B1_08 */
    _map[5]  = { ZPWM_DEV(flexpwm2_pwm2), 0, 2, 0 };   /* FMU_CH6  EMC_B1_10 */
    _map[6]  = { ZPWM_DEV(flexpwm2_pwm3), 0, 2, 0 };   /* FMU_CH7  EMC_B1_19 */
    _map[7]  = { ZPWM_DEV(flexpwm3_pwm0), 0, 3, 0 };   /* FMU_CH8  EMC_B1_29 */
    _map[8]  = { ZPWM_DEV(flexpwm3_pwm1), 0, 3, 0 };   /* FMU_CH9  EMC_B1_31 */
    _map[9]  = { ZPWM_DEV(flexpwm3_pwm3), 0, 3, 0 };   /* FMU_CH10 EMC_B1_21 */
    _map[10] = { ZPWM_DEV(flexpwm4_pwm0), 0, 4, 0 };   /* FMU_CH11 EMC_B1_00 */
    _map[11] = { ZPWM_DEV(flexpwm4_pwm1), 0, 4, 0 };   /* FMU_CH12 EMC_B1_02 */
    for (uint8_t i = 12; i < NUM_CHANNELS; i++) {
        _map[i] = { nullptr, 0 };
    }
#undef ZPWM_DEV

    /* Channels are usable individually - _apply_channel() already skips any
     * entry whose device is null or not ready. The previous code cleared
     * _map_ready if ANY channel was missing, which would have disabled every
     * output just because CH4/CH6 have no pad yet. */
    _map_ready = true;
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(ledc0), okay)
    /* Espressif LEDC (ESP32-C6): PWM1-4 on GPIO1/2/3/23, LEDC channels
       0-3, one low-speed timer per channel (board DTS) so every output can
       run its own frequency. pwm_led_esp32.c channels are 0-based. */
    const struct device *ledc = DEVICE_DT_GET(DT_NODELABEL(ledc0));
    _map[0] = { ledc, 0 };   /* PWM1  GPIO1  */
    _map[1] = { ledc, 1 };   /* PWM2  GPIO2  */
    _map[2] = { ledc, 2 };   /* PWM3  GPIO3  */
    _map[3] = { ledc, 3 };   /* PWM4  GPIO23 */
    for (uint8_t i = 4; i < NUM_CHANNELS; i++) {
        _map[i] = { nullptr, 0 };
    }
    _map_ready = true;
#else
    /* No PWM output map for this SoC yet (e.g. ESP32-S3, whose LEDC/MCPWM
       map has not been designed; native_sim has no PWM hardware at all).
       All channels null - _apply_channel() skips them. */
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        _map[i] = { nullptr, 0 };
    }
    _map_ready = true;
#endif  /* tim1_pwm (STM32) vs flexpwm (NXP) vs ledc (ESP32) vs no map */

#if AP_ZEPHYR_DSHOT_ENABLED
    /* Deterministic pad ownership at boot: the FlexIO-DShot driver applies its own
     * pinctrl at init, so the last applier would otherwise win by init-order luck. */
    (void)ap_rcout_pinmux_apply_pwm();
#endif
#endif  /* __ZEPHYR__ */
}

#if AP_ZEPHYR_DSHOT_ENABLED

/* FMU_CH1-8 = AP channels 0-7: the only channels with a FlexIO alternate
   (docs/WIRING.md "8 DShot-capable"); CH9-12 are PWM-only. Must match the
   dshot DTS node's child list, whose order defines driver channels 0-7. */
static constexpr uint32_t DSHOT_BANK_MASK = 0xFFU;

void RCOutput::set_output_mode(uint32_t mask, enum output_mode mode)
{
    const uint32_t bank = mask & DSHOT_BANK_MASK;
    if (bank == 0) {
        return;
    }

    if (is_dshot_protocol(mode)) {
        /* Bank-granular, deliberately: the arbiter node's "dshot" state covers a whole
         * bank, so a single channel cannot be switched alone. */
        if (_dshot_mask == 0) {
            (void)ap_rcout_pinmux_apply_dshot();
        }
        _dshot_mask = DSHOT_BANK_MASK;
        _dshot_mode = mode;
    } else {
        /* Any PWM-family mode returns the bank to FlexPWM; the existing
           set_freq() path then provides the param-selected rate (50Hz
           servos, 400Hz fast PWM, oneshot's base rate, etc). */
        if (_dshot_mask != 0) {
            (void)ap_rcout_pinmux_apply_pwm();
            /* re-assert PWM period/duty on every bank channel: the PWM
               peripheral kept running while FlexIO owned the pads, but
               values may be stale */
            for (uint8_t i = 0; i < 8; i++) {
                _dirty[i] = true;
            }
        }
        _dshot_mask = 0;
        _dshot_mode = MODE_PWM_NORMAL;
    }
}

enum AP_HAL::RCOutput::output_mode RCOutput::get_output_mode(uint32_t &mask)
{
    if (_dshot_mask != 0) {
        mask = _dshot_mask;
        return _dshot_mode;
    }
    mask = 0;
    return MODE_PWM_NORMAL;
}

void RCOutput::dshot_tick()
{
    if (_dshot_mask == 0) {
        return;
    }

    const struct device *dshot_dev = DEVICE_DT_GET(DT_NODELABEL(dshot));
    if (!device_is_ready(dshot_dev)) {
        return;
    }

    /* When disarmed we always send a zero value, exactly like ChibiOS's
       dshot_send() - DShot 0 is the "disarmed/off" command, and streaming
       it keeps the ESCs' signal-loss timeout satisfied without any
       possibility of spin. */
    /* ChibiOS dshot_send(): zero while the safety is on, as well as while
       disarmed. */
    const bool armed = hal.util->get_soft_armed() &&
                       safety_state != AP_HAL::Util::SAFETY_DISARMED;

    const uint8_t nch = MIN((uint8_t)8, nxp_flexio_dshot_channel_count(dshot_dev));
    for (uint8_t i = 0; i < nch; i++) {
        uint16_t value = 0;
        if (armed && _enabled[i] && _period_us[i] != 0) {
            /* ChibiOS's exact conversion (RCOutput.cpp dshot_send()):
               1000-2000us -> 0-1999, then +48 into DShot's 48-2047
               throttle range (0-47 are reserved commands, 0 = off). */
            const uint16_t pwm = constrain_int16(_period_us[i], 1000, 2000);
            value = MIN(2 * (pwm - 1000), 1999);
            if (value != 0) {
                value += 48;
            }
        }
        nxp_flexio_dshot_data_set(dshot_dev, i, value, false);
    }
    nxp_flexio_dshot_trigger(dshot_dev);
}

#endif  /* AP_ZEPHYR_DSHOT_ENABLED */

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (freq_hz == 0) {
        return;
    }

    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if ((chmask & (1U << i)) != 0U) {
            _freq_hz[i] = freq_hz;
            _dirty[i] = true;
            if (!_corked) {
#ifdef __ZEPHYR__
                _apply_channel(i);
#endif
                _dirty[i] = false;
            }
        }
    }
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    if (chan >= NUM_CHANNELS) {
        return 0;
    }
    return _freq_hz[chan];
}

void RCOutput::enable_ch(uint8_t chan)
{
    if (chan >= NUM_CHANNELS) {
        return;
    }
    _enabled[chan] = true;
    _dirty[chan] = true;
}

void RCOutput::disable_ch(uint8_t chan)
{
    if (chan >= NUM_CHANNELS) {
        return;
    }
    _enabled[chan] = false;
    _dirty[chan] = true;
#ifdef __ZEPHYR__
    if (!_corked) {
        _apply_channel(chan);
        _dirty[chan] = false;
    }
#endif
}

/* AP_HAL_ChibiOS/RCOutput.cpp force_safety_on(): set the flag; the output
   path reads it. Applied to every channel at once, corked or not, because
   Scheduler::reboot() calls this and never push()es afterwards. */
bool RCOutput::force_safety_on(void)
{
    safety_state = AP_HAL::Util::SAFETY_DISARMED;
#ifdef __ZEPHYR__
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        _apply_channel(i);
        _dirty[i] = false;
    }
#if AP_ZEPHYR_DSHOT_ENABLED
    dshot_tick();   /* sends DShot 0 on every channel while safety is on */
#endif
#endif
    return true;
}

void RCOutput::force_safety_off(void)
{
    safety_state = AP_HAL::Util::SAFETY_ARMED;
}

AP_HAL::Util::safety_state RCOutput::_safety_switch_state(void)
{
    hal.util->persistent_data.safety_state = safety_state;
    return safety_state;
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan >= NUM_CHANNELS) {
        return;
    }

    _period_us[chan] = period_us;
    /* NOT _enabled[chan] = true. ChibiOS's RCOutput::write() never touches the
       enable mask - only enable_ch()/disable_ch() do. Setting it here meant a
       channel that had been deliberately disabled came back to life on the
       next write(), which for an output channel means a motor or servo
       driven after something asked for it to stop. */
    _dirty[chan] = true;

#if HAL_WITH_IO_MCU && AP_ZEPHYR_IOMCU_ENABLED
    if (_iomcu_enabled) {
        iomcu.write_channel(chan, period_us);
    }
#endif

    if (!_corked) {
#ifdef __ZEPHYR__
        _apply_channel(chan);
#endif
        _dirty[chan] = false;
    }
}

uint16_t RCOutput::read(uint8_t chan)
{
    if (chan >= NUM_CHANNELS) {
        return 0;
    }
    return _period_us[chan];
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    if (period_us == nullptr) {
        return;
    }

    if (len > NUM_CHANNELS) {
        len = NUM_CHANNELS;
    }

    memcpy(period_us, _period_us, len * sizeof(period_us[0]));
}

void RCOutput::cork()
{
    _corked = true;
}

void RCOutput::push()
{
#if HAL_WITH_IO_MCU && AP_ZEPHYR_IOMCU_ENABLED
    if (_iomcu_enabled) {
        iomcu.push();
    }
#endif
#ifdef __ZEPHYR__
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (_dirty[i]) {
            _apply_channel(i);
            _dirty[i] = false;
        }
    }
#endif
    _corked = false;
}

#ifdef __ZEPHYR__
/* Report a pwm_set() result once per channel per failure episode. Silence on
   the way back to working, so a channel that recovers says so. */
void RCOutput::_report_pwm_set(uint8_t chan, int rc)
{
    if (chan >= NUM_CHANNELS) {
        return;
    }
    if (rc != 0) {
        if (!_pwm_set_failed[chan]) {
            _pwm_set_failed[chan] = true;
            printk("AP_Zephyr RCOutput: pwm_set failed on channel %u (%d)\n",
                   (unsigned)chan, rc);
        }
    } else if (_pwm_set_failed[chan]) {
        _pwm_set_failed[chan] = false;
        printk("AP_Zephyr RCOutput: channel %u recovered\n", (unsigned)chan);
    }
}

void RCOutput::_apply_channel(uint8_t chan)
{
    if (!_map_ready || chan >= NUM_CHANNELS) {
        return;
    }

#if AP_ZEPHYR_DSHOT_ENABLED
    if (_dshot_mask & (1U << chan)) {
        /* FlexIO owns this pad; the staged _period_us is consumed by
           dshot_tick() instead. Writing the orphaned FlexPWM peripheral
           would be harmless but misleading in any register-level debug. */
        return;
    }
#endif

    const PWMChannelMap &m = _map[chan];
    /* device_is_ready() re-walked per channel per loop is the same disease
       spi_is_ready_dt() had (6.2% of PC samples); readiness never regresses. */
    if (!_dev_ready[chan]) {
        if (m.dev == nullptr || !device_is_ready(m.dev)) {
            return;
        }
        _dev_ready[chan] = true;
    }

    const uint32_t freq = (_freq_hz[chan] == 0U) ? 50U : _freq_hz[chan];
    const uint32_t period_us = 1000000U / freq;

    /* pwm_set() failures were discarded. A channel whose driver rejects the
       setting then sits at whatever it was last given while the vehicle
       believes the new value took, so report it - once per channel, because
       this runs on the output path and must not print per update. */
    if (!_enabled[chan]) {
        _report_pwm_set(chan, pwm_set(m.dev, m.hw_channel,
                                      PWM_USEC(period_us), PWM_USEC(0U), 0));
        return;
    }

    /* ChibiOS RCOutput::write(): with safety on the pulse is zero (no
       BRD_SAFETY_MASK here, so every channel). */
    uint16_t pulse_us = (safety_state == AP_HAL::Util::SAFETY_DISARMED) ? 0 : _period_us[chan];
    if (pulse_us > period_us) {
        pulse_us = period_us;
    }

    _report_pwm_set(chan, pwm_set(m.dev, m.hw_channel,
                                  PWM_USEC(period_us), PWM_USEC(pulse_us), 0));
}
#endif


/*
  @SYS/timers.txt, byte-for-byte in AP_HAL_ChibiOS/RCOutput.cpp::timer_info()'s
  TIMERV1 columns so a parser written for ChibiOS reads it unchanged. Where a
  quantity genuinely does not exist on this HAL the column is ZERO rather than
  absent or reformatted.

  ChibiOS walks pwm_group_list, one row per hardware timer. This HAL has no
  such list: Zephyr's PWM driver owns the timer and the devicetree hands out
  (device, channel) pairs, so the grouping is rebuilt by collecting the
  distinct devices out of _map[] - one Zephyr PWM device is one timer.

  Column by column:
    TIM   the timer number, from PWMChannelMap::timer_id. It cannot come from
          the device name: every STM32 PWM node is called "pwm", the number is
          on the parent, and reading the first digit out of the name gave
          "TIM0" for every row.
    CLK   the timer's INPUT clock in MHz, as ChibiOS prints. Zephyr reports
          the post-prescaler counter rate, so this multiplies back up by
          (prescaler + 1). Zero when the binding states no prescaler and the
          input clock is therefore not recoverable.
    MODE  the current output mode, same strings as ChibiOS.
    FREQ  the counter rate after prescaling, from pwm_get_cycles_per_sec() -
          the driver's own answer, so none of ChibiOS's prescaler arithmetic
          is reimplemented here.
    TGT   ChibiOS's target BIT clock, which only exists for a serial output
          protocol. Zero under ordinary PWM, where there is no bit clock -
          this column is not the servo update rate, and filling it with one
          would make the two HALs' files silently incomparable.
*/
void RCOutput::timer_info(ExpandingString &str)
{
    // a header to allow for machine parsers to determine format
    str.printf("TIMERV1\n");

#ifdef __ZEPHYR__
    if (!_map_ready) {
        return;
    }

    const struct device *seen[NUM_CHANNELS] = {};
    uint8_t num_seen = 0;

    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        const struct device *dev = _map[i].dev;
        if (dev == nullptr) {
            continue;
        }
        bool already = false;
        for (uint8_t j = 0; j < num_seen; j++) {
            if (seen[j] == dev) {
                already = true;
                break;
            }
        }
        if (already) {
            continue;
        }
        seen[num_seen++] = dev;

        uint64_t counter_hz = 0;
        if (pwm_get_cycles_per_sec(dev, _map[i].hw_channel, &counter_hz) != 0) {
            counter_hz = 0;
        }
        const uint32_t input_hz = (_map[i].prescaler != 0)
            ? (uint32_t)(counter_hz * (uint64_t)(_map[i].prescaler + 1))
            : 0;

        enum output_mode mode = MODE_PWM_NORMAL;
        uint32_t target_bitrate = 0;
#if AP_ZEPHYR_DSHOT_ENABLED
        uint32_t mode_mask = 0;
        mode = get_output_mode(mode_mask);
        /* ChibiOS reads this from its private protocol_bitrate(); the rates
           are fixed by the DShot spec, so mirror them rather than export
           that function just for a diagnostic. */
        switch (mode) {
        case MODE_PWM_DSHOT150:  target_bitrate =  150000U * DSHOT_BIT_WIDTH_TICKS; break;
        case MODE_PWM_DSHOT300:  target_bitrate =  300000U * DSHOT_BIT_WIDTH_TICKS; break;
        case MODE_PWM_DSHOT600:  target_bitrate =  600000U * DSHOT_BIT_WIDTH_TICKS; break;
        case MODE_PWM_DSHOT1200: target_bitrate = 1200000U * DSHOT_BIT_WIDTH_TICKS; break;
        default:                 target_bitrate = 0; break;   /* no bit clock under ordinary PWM */
        }
#endif
        str.printf("TIM%-2u CLK=%4uMhz MODE=%5s FREQ=%8u TGT=%8u\n",
                   unsigned(_map[i].timer_id),
                   unsigned(input_hz / 1000000U),
                   get_output_mode_string(mode),
                   unsigned(counter_hz),
                   unsigned(target_bitrate));
    }
#endif  // __ZEPHYR__
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
