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
 */
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#if HAL_USE_PWM == TRUE

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

struct RCOutput::pwm_group RCOutput::pwm_group_list[] = { HAL_PWM_GROUPS };

#define NUM_GROUPS ARRAY_SIZE_SIMPLE(pwm_group_list)

#define CHAN_DISABLED 255

void RCOutput::init()
{
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        //Start Pwm groups
        pwmStart(pwm_group_list[i].pwm_drv, &pwm_group_list[i].pwm_cfg);
        for (uint8_t j = 0; j < 4; j++ ) {
            if (pwm_group_list[i].chan[j] != CHAN_DISABLED) {
                total_channels = MAX(total_channels, pwm_group_list[i].chan[j]+1);
            }
        }
    }
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        iomcu.init();
        // with IOMCU the local channels start at 8
        chan_offset = 8;
        total_channels += chan_offset;
    }
#endif
    chMtxObjectInit(&trigger_mutex);
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    //check if the request spans accross any of the channel groups
    uint8_t update_mask = 0;
    // greater than 400 doesn't give enough room at higher periods for
    // the down pulse
    if (freq_hz > 400 && _output_mode != MODE_PWM_BRUSHED) {
        freq_hz = 400;
    }

#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        iomcu.set_freq(chmask, freq_hz);
    }
#endif

    chmask >>= chan_offset;
    if (chmask == 0) {
        return;
    }
    
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        uint16_t grp_ch_mask = 0;
        for (uint8_t j=0; j<4; j++) {
            if (pwm_group_list[i].chan[j] != CHAN_DISABLED) {
                grp_ch_mask |= (1U<<pwm_group_list[i].chan[j]);
            }
        }
        if ((grp_ch_mask & chmask) != 0) {
            /*
              we enable the new frequency on all groups that have one
              of the requested channels. This means we may enable high
              speed on some channels that aren't requested, but that
              is needed in order to fly a vehicle such a a hex
              multicopter properly
             */
            update_mask |= grp_ch_mask;
            uint16_t freq_set = freq_hz;
            uint32_t old_clock = pwm_group_list[i].pwm_cfg.frequency;
            
            if (freq_set > 400 && pwm_group_list[i].pwm_cfg.frequency == 1000000) {
                // need to change to an 8MHz clock
                pwm_group_list[i].pwm_cfg.frequency = 8000000;
            } else if (freq_set <= 400 && pwm_group_list[i].pwm_cfg.frequency == 8000000) {
                // need to change to an 1MHz clock
                pwm_group_list[i].pwm_cfg.frequency = 1000000;
            }

            // check if the frequency is possible, and keep halving
            // down to 1MHz until it is OK with the hardware timer we
            // are using. If we don't do this we'll hit an assert in
            // the ChibiOS PWM driver on some timers
            PWMDriver *pwmp = pwm_group_list[i].pwm_drv;
            uint32_t psc = (pwmp->clock / pwmp->config->frequency) - 1;
            while ((psc > 0xFFFF || ((psc + 1) * pwmp->config->frequency) != pwmp->clock) &&
                   pwm_group_list[i].pwm_cfg.frequency > 1000000) {
                pwm_group_list[i].pwm_cfg.frequency /= 2;
                psc = (pwmp->clock / pwmp->config->frequency) - 1;
            }
            
            if (old_clock != pwm_group_list[i].pwm_cfg.frequency) {
                // we need to stop and start to setup the new clock
                pwmStop(pwm_group_list[i].pwm_drv);
                pwmStart(pwm_group_list[i].pwm_drv, &pwm_group_list[i].pwm_cfg);
            }
            pwmChangePeriod(pwm_group_list[i].pwm_drv, 
                            pwm_group_list[i].pwm_cfg.frequency/freq_set);
        }
    }
    if (freq_hz > 50) {
        fast_channel_mask |= update_mask;
    }
    if (chmask != update_mask) {
        hal.console->printf("RCOutput: Failed to set PWM frequency req %x set %x\n", (unsigned)chmask, (unsigned)update_mask);
    }
}

/*
  set default output rate
 */
void RCOutput::set_default_rate(uint16_t freq_hz)
{
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        iomcu.set_default_rate(freq_hz);
    }
#endif
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        uint16_t grp_ch_mask = 0;
        for (uint8_t j=0; j<4; j++) {
            if (pwm_group_list[i].chan[j] != CHAN_DISABLED) {
                grp_ch_mask |= (1U<<pwm_group_list[i].chan[j]);
            }
        }
        if (grp_ch_mask & fast_channel_mask) {
            // don't change fast channels
            continue;
        }
        pwmChangePeriod(pwm_group_list[i].pwm_drv, 
                        pwm_group_list[i].pwm_cfg.frequency/freq_hz);
    }
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    if (chan >= total_channels) {
        return 0;
    }
#if HAL_WITH_IO_MCU
    if (chan < chan_offset) {
        return iomcu.get_freq(chan);
    }
#endif
    chan -= chan_offset;
    
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            if (pwm_group_list[i].chan[j] == chan) {
                return pwm_group_list[i].pwm_drv->config->frequency / pwm_group_list[i].pwm_drv->period;
            }
        }
    }
    // assume 50Hz default
    return 50;
}

void RCOutput::enable_ch(uint8_t chan)
{
    if (chan >= total_channels) {
        return;
    }
    if (chan < chan_offset) {
        return;
    }
    chan -= chan_offset;

    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            if ((pwm_group_list[i].chan[j] == chan) && !(en_mask & 1<<chan)) {
                en_mask |= 1<<chan;
            }
        }
    }
}

void RCOutput::disable_ch(uint8_t chan)
{
    if (chan >= total_channels) {
        return;
    }
    if (chan < chan_offset) {
        return;
    }
    chan -= chan_offset;

    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            if (pwm_group_list[i].chan[j] == chan) {
                pwmDisableChannel(pwm_group_list[i].pwm_drv, j);
                en_mask &= ~(1<<chan);
            }
        }
    }
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan >= total_channels) {
        return;
    }
    last_sent[chan] = period_us;
    
#if HAL_WITH_IO_MCU
    // handle IO MCU channels
    if (AP_BoardConfig::io_enabled()) {
        iomcu.write_channel(chan, period_us);
    }
#endif
    if (chan < chan_offset) {
        return;
    }
    chan -= chan_offset;
    
    period[chan] = period_us;
    num_channels = MAX(chan+1, num_channels);
    if (!corked) {
        push_local();
    }
}

/*
  push values to local channels from period[] array
 */
void RCOutput::push_local(void)
{
    if (num_channels == 0) {
        return;
    }
    uint16_t outmask = (1U<<num_channels)-1;
    outmask &= en_mask;

    uint16_t widest_pulse = 0;
    uint8_t need_trigger = 0;
    
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        for (uint8_t j = 0; j < 4; j++) {
            uint8_t chan = pwm_group_list[i].chan[j];
            if (chan == CHAN_DISABLED) {
                continue;
            }
            if (outmask & (1UL<<chan)) {
                uint32_t period_us = period[chan];
                if(_output_mode == MODE_PWM_BRUSHED && (fast_channel_mask & (1UL<<chan))) {
                    // note that we only use brushed signals on fast
                    // channels. This allows for ordinary PWM on
                    // servos attached to a brushed vehicle
                    if (period_us <= _esc_pwm_min) {
                        period_us = 0;
                    } else if (period_us >= _esc_pwm_max) {
                        period_us = PWM_FRACTION_TO_WIDTH(pwm_group_list[i].pwm_drv, 1, 1);
                    } else {
                        period_us = PWM_FRACTION_TO_WIDTH(pwm_group_list[i].pwm_drv,\
                               (_esc_pwm_max - _esc_pwm_min), (period_us - _esc_pwm_min));
                    }
                    pwmEnableChannel(pwm_group_list[i].pwm_drv, j, period_us);
                } else {
                    uint32_t width = (pwm_group_list[i].pwm_cfg.frequency/1000000) * period_us;
                    pwmEnableChannel(pwm_group_list[i].pwm_drv, j, width);
                }
                if (period_us > widest_pulse) {
                    widest_pulse = period_us;
                }
                need_trigger |= (1U<<i);
            }
        }
    }

    if (widest_pulse > 2300) {
        widest_pulse = 2300;
    }
    trigger_widest_pulse = widest_pulse;
    
    trigger_groups = need_trigger;
    
    if (trigger_groups && _output_mode == MODE_PWM_ONESHOT) {
        trigger_oneshot();
    }
}

uint16_t RCOutput::read(uint8_t chan)
{
    if (chan >= total_channels) {
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
    if (len > total_channels) {
        len = total_channels;
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
    if (chan >= total_channels) {
        return 0;
    }
    return last_sent[chan];
}

void RCOutput::read_last_sent(uint16_t* period_us, uint8_t len)
{
    if (len > total_channels) {
        len = total_channels;
    }
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read_last_sent(i);
    }
}

/*
  setup output mode
 */
void RCOutput::set_output_mode(enum output_mode mode)
{
    _output_mode = mode;
    if (_output_mode == MODE_PWM_BRUSHED) {
        // force zero output initially
        for (uint8_t i=chan_offset; i<chan_offset+num_channels; i++) {
            write(i, 0);
        }
    }
    if (_output_mode == MODE_PWM_ONESHOT) {
        // for oneshot we force 1Hz output and then trigger on each loop
        for (uint8_t i=0; i< NUM_GROUPS; i++) {
            pwmChangePeriod(pwm_group_list[i].pwm_drv, pwm_group_list[i].pwm_cfg.frequency);
        }
#if HAL_WITH_IO_MCU
        if (AP_BoardConfig::io_enabled()) {
            return iomcu.set_oneshot_mode();
        }
#endif
    }
}

/*
  force the safety switch on, disabling PWM output from the IO board
*/
bool RCOutput::force_safety_on(void)
{
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        return iomcu.force_safety_on();
    }
#endif
    return false;
}

/*
  force the safety switch off, enabling PWM output from the IO board
*/
void RCOutput::force_safety_off(void)
{
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        iomcu.force_safety_off();
    }
#endif
}

/*
  start corking output
 */
void RCOutput::cork(void)
{
    corked = true;
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
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
    if (AP_BoardConfig::io_enabled()) {
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
    if (AP_BoardConfig::io_enabled()) {
        return iomcu.enable_sbus_out(rate_hz);
    }
#endif
    return false;
}

/*
  trigger output groups for oneshot mode
 */
void RCOutput::trigger_oneshot(void)
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
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        if (trigger_groups & (1U<<i)) {
            // this triggers pulse output for a channel group
            pwm_group_list[i].pwm_drv->tim->EGR = STM32_TIM_EGR_UG;
        }
    }
    osalSysUnlock();
    /*
      calculate time that we are allowed to trigger next pulse
      to guarantee at least a 50us gap between pulses
    */
    min_pulse_trigger_us = AP_HAL::micros64() + trigger_widest_pulse + 50;
    chMtxUnlock(&trigger_mutex);
}

/*
  periodic timer. The only need for a periodic timer is in oneshot
  mode where we want to sustain a minimum output rate for when the
  main loop is busy doing something like gyro calibration

  A mininum output rate helps with some oneshot ESCs
 */
void RCOutput::timer_tick(void)
{
    if (_output_mode != MODE_PWM_ONESHOT ||
        trigger_groups == 0 ||
        min_pulse_trigger_us == 0) {
        return;
    }
    uint64_t now = AP_HAL::micros64();
    if (now - min_pulse_trigger_us > 10000) {
        // trigger at a minimum of 100Hz
        trigger_oneshot();
    }
}

#endif // HAL_USE_PWM
