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
        pwm_group &group = pwm_group_list[i];
        group.ch_mask = 0;
        group.current_mode = MODE_PWM_NORMAL;
        for (uint8_t j = 0; j < 4; j++ ) {
            if (group.chan[j] != CHAN_DISABLED) {
                total_channels = MAX(total_channels, group.chan[j]+1);
                group.ch_mask |= (1U<<group.chan[j]);
            }
        }
        if (group.ch_mask != 0) {
            pwmStart(group.pwm_drv, &group.pwm_cfg);
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
        // greater than 400 doesn't give enough room at higher periods for
        // the down pulse. This still allows for high rate with oneshot and dshot.
        pwm_group &group = pwm_group_list[i];
        uint16_t group_freq = freq_hz;
        if (group_freq > 400 && group.current_mode != MODE_PWM_BRUSHED) {
            group_freq = 400;
        }
        if ((group.ch_mask & chmask) != 0) {
            /*
              we enable the new frequency on all groups that have one
              of the requested channels. This means we may enable high
              speed on some channels that aren't requested, but that
              is needed in order to fly a vehicle such a a hex
              multicopter properly
             */
            update_mask |= group.ch_mask;
            uint16_t freq_set = group_freq;
            uint32_t old_clock = group.pwm_cfg.frequency;

            if (freq_set > 400 && group.pwm_cfg.frequency == 1000000) {
                // need to change to an 8MHz clock
                group.pwm_cfg.frequency = 8000000;
            } else if (freq_set <= 400 && group.pwm_cfg.frequency == 8000000) {
                // need to change to an 1MHz clock
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

            if (old_clock != group.pwm_cfg.frequency) {
                // we need to stop and start to setup the new clock
                pwmStop(group.pwm_drv);
                pwmStart(group.pwm_drv, &group.pwm_cfg);
            }
            pwmChangePeriod(group.pwm_drv,
                            group.pwm_cfg.frequency/freq_set);
        }
        if (group_freq > 50) {
            fast_channel_mask |= group.ch_mask;
        }
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
        pwm_group &group = pwm_group_list[i];
        if ((group.ch_mask & fast_channel_mask) || group.ch_mask == 0) {
            // don't change fast channels
            continue;
        }
        pwmChangePeriod(group.pwm_drv,
                        group.pwm_cfg.frequency/freq_hz);
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
        pwm_group &group = pwm_group_list[i];
        for (uint8_t j = 0; j < 4; j++) {
            if (group.chan[j] == chan) {
                return group.pwm_drv->config->frequency / group.pwm_drv->period;
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
        pwm_group &group = pwm_group_list[i];
        for (uint8_t j = 0; j < 4; j++) {
            if ((group.chan[j] == chan) && !(en_mask & 1<<chan)) {
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
        pwm_group &group = pwm_group_list[i];
        for (uint8_t j = 0; j < 4; j++) {
            if (group.chan[j] == chan) {
                pwmDisableChannel(group.pwm_drv, j);
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
        pwm_group &group = pwm_group_list[i];
        for (uint8_t j = 0; j < 4; j++) {
            uint8_t chan = group.chan[j];
            if (chan == CHAN_DISABLED) {
                continue;
            }
            if (outmask & (1UL<<chan)) {
                uint32_t period_us = period[chan];
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
                } else if (group.current_mode < MODE_PWM_DSHOT150) {
                    uint32_t width = (group.pwm_cfg.frequency/1000000) * period_us;
                    pwmEnableChannel(group.pwm_drv, j, width);
                } else if (group.current_mode >= MODE_PWM_DSHOT150 && group.current_mode <= MODE_PWM_DSHOT1200) {
                    // set period_us to time for pulse output, to enable very fast rates
                    period_us = dshot_pulse_time_us;
                }
                if (period_us > widest_pulse) {
                    widest_pulse = period_us;
                }
                if (group.current_mode == MODE_PWM_ONESHOT ||
                    (group.current_mode >= MODE_PWM_DSHOT150 &&
                     group.current_mode <= MODE_PWM_DSHOT1200)) {
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
void RCOutput::set_output_mode(uint16_t mask, enum output_mode mode)
{
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        pwm_group &group = pwm_group_list[i];
        if (((group.ch_mask << chan_offset) & mask) == 0) {
            // this group is not affected
            continue;
        }
        group.current_mode = mode;
        if (mode == MODE_PWM_BRUSHED) {
            // force zero output initially
            for (uint8_t i=chan_offset; i<16; i++) {
                if ((group.ch_mask << chan_offset) & (1U<<i)) {
                    write(i, 0);
                }
            }
        }
        if (mode >= MODE_PWM_DSHOT150 && mode <= MODE_PWM_DSHOT1200) {
            if (!group.have_up_dma) {
                // fallback to ONESHOT125
                hal.console->printf("DShot unavailable on mask 0x%04x\n", group.ch_mask);
                group.current_mode = MODE_PWM_NORMAL;
            } else {
                if (!group.dma_buffer) {
                    group.dma_buffer = (uint32_t *)hal.util->malloc_type(dshot_buffer_length, AP_HAL::Util::MEM_DMA_SAFE);
                    if (!group.dma_buffer) {
                        hal.console->printf("DShot setup no memory\n");
                        group.current_mode = MODE_PWM_NORMAL;
                        continue;
                    }
                }
                // for dshot we setup for DMAR based output
                if (!group.dma) {
                    group.dma = STM32_DMA_STREAM(group.dma_up_stream_id);
                    group.dma_handle = new Shared_DMA(group.dma_up_stream_id, SHARED_DMA_NONE,
                                                      FUNCTOR_BIND_MEMBER(&RCOutput::dma_allocate, void, Shared_DMA *),
                                                      FUNCTOR_BIND_MEMBER(&RCOutput::dma_deallocate, void, Shared_DMA *));
                    if (!group.dma_handle) {
                        hal.console->printf("DShot setup no memory\n");
                        group.current_mode = MODE_PWM_NORMAL;
                        continue;
                    }
                }
                const uint16_t rates[(1 + MODE_PWM_DSHOT1200) - MODE_PWM_DSHOT150] = { 150, 300, 600, 1200 };
                uint32_t rate = rates[uint8_t(mode - MODE_PWM_DSHOT150)] * 1000UL;
                const uint32_t bit_period = 19;
                // configure timer driver for DMAR at requested rate
                pwmStop(group.pwm_drv);
                group.pwm_cfg.frequency = rate * bit_period;
                group.pwm_cfg.period = bit_period;
                group.pwm_cfg.dier = TIM_DIER_UDE;
                group.pwm_cfg.cr2 = 0;
                pwmStart(group.pwm_drv, &group.pwm_cfg);
                for (uint8_t j=0; j<4; j++) {
                    if (group.chan[j] != CHAN_DISABLED) {
                        pwmEnableChannel(group.pwm_drv, j, 0);
                    }
                }
                // calculate min time between pulses
                dshot_pulse_time_us = 1000000UL * dshot_bit_length / rate;
            }
        }
        if (group.current_mode == MODE_PWM_ONESHOT) {
            // for oneshot we force 1Hz output and then trigger on each loop
            pwmChangePeriod(group.pwm_drv, group.pwm_cfg.frequency);
        }
    }
#if HAL_WITH_IO_MCU
    if (mode == MODE_PWM_ONESHOT &&
        (mask & ((1U<<chan_offset)-1)) &&
        AP_BoardConfig::io_enabled()) {
        return iomcu.set_oneshot_mode();
    }
#endif
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
    for (uint8_t i = 0; i < NUM_GROUPS; i++) {
        pwm_group &group = pwm_group_list[i];
        if (group.current_mode == MODE_PWM_ONESHOT) {
            if (trigger_groupmask & (1U<<i)) {
                // this triggers pulse output for a channel group
                group.pwm_drv->tim->EGR = STM32_TIM_EGR_UG;
            }
        } else if (group.current_mode >= MODE_PWM_DSHOT150 && group.current_mode <= MODE_PWM_DSHOT1200) {
            dshot_send(group, false);
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
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        pwm_group &group = pwm_group_list[i];
        if (dshot_delayed_trigger_mask & (1U<<i)) {
            // do a blocking send now
            dshot_send(group, true);
        }
    }
    if (trigger_groupmask == 0 ||
        min_pulse_trigger_us == 0) {
        return;
    }
    uint64_t now = AP_HAL::micros64();
    if (now > min_pulse_trigger_us &&
        now - min_pulse_trigger_us > 10000) {
        // trigger at a minimum of 100Hz
        trigger_groups();
    }    
}

/*
  allocate DMA channel
 */
void RCOutput::dma_allocate(Shared_DMA *ctx)
{
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        pwm_group &group = pwm_group_list[i];
        if (group.dma_handle == ctx) {
            dmaStreamAllocate(group.dma, 10, dma_irq_callback, &group);
        }
    }
}

/*
  deallocate DMA channel
 */
void RCOutput::dma_deallocate(Shared_DMA *ctx)
{
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        pwm_group &group = pwm_group_list[i];
        if (group.dma_handle == ctx) {
            dmaStreamRelease(group.dma);
        }
    }
}

/*
  create a DSHOT 16 bit packet. Based on prepareDshotPacket from betaflight
 */
uint16_t RCOutput::create_dshot_packet(const uint16_t value)
{
    uint16_t packet = (value << 1); // no telemetry request

    // compute checksum
    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (uint8_t i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    // append checksum
    packet = (packet << 4) | csum;

    return packet;
}

/*
  fill in a DMA buffer for dshot
 */
void RCOutput::fill_DMA_buffer_dshot(uint32_t *buffer, uint8_t stride, uint16_t packet)
{
    const uint8_t DSHOT_MOTOR_BIT_0 = 7;
    const uint8_t DSHOT_MOTOR_BIT_1 = 14;
    for (uint16_t i = 0; i < 16; i++) {
        buffer[i * stride] = (packet & 0x8000) ? DSHOT_MOTOR_BIT_1 : DSHOT_MOTOR_BIT_0;
        packet <<= 1;
    }
}

/*
  send a set of DShot packets for a channel group
  This call be called in blocking mode from the timer, in which case it waits for the DMA lock.
  In normal operation it doesn't wait for the DMA lock.
 */
void RCOutput::dshot_send(pwm_group &group, bool blocking)
{
    uint8_t groupidx = &group - pwm_group_list;
    if (blocking) {
        group.dma_handle->lock();
        dshot_delayed_trigger_mask &= ~(1U<<groupidx);
    } else {
        if (!group.dma_handle->lock_nonblock()) {
            dshot_delayed_trigger_mask |= 1U<<groupidx;
            return;
        }
    }
    for (uint8_t i=0; i<4; i++) {
        uint8_t chan = group.chan[i];
        if (chan != CHAN_DISABLED) {
            uint16_t pwm = period[chan];
            pwm = constrain_int16(pwm, _esc_pwm_min, _esc_pwm_max);
            uint16_t value = 2000UL * uint32_t(pwm - _esc_pwm_min) / uint32_t(_esc_pwm_max - _esc_pwm_min);
            if (value != 0) {
                // dshot values are from 48 to 2047. Zero means off.
                value += 47;
            }
            uint16_t packet = create_dshot_packet(value);
            fill_DMA_buffer_dshot(group.dma_buffer + i, 4, packet);
        }
    }

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
    dmaStreamSetPeripheral(group.dma, &(group.pwm_drv->tim->DMAR));
    dmaStreamSetMemory0(group.dma, group.dma_buffer);
    dmaStreamSetTransactionSize(group.dma, dshot_buffer_length/sizeof(uint32_t));
    dmaStreamSetFIFO(group.dma, STM32_DMA_FCR_DMDIS | STM32_DMA_FCR_FTH_FULL);
    dmaStreamSetMode(group.dma,
                     STM32_DMA_CR_CHSEL(group.dma_up_channel) |
                     STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD |
                     STM32_DMA_CR_MINC | STM32_DMA_CR_PL(3) |
                     STM32_DMA_CR_TEIE | STM32_DMA_CR_TCIE);

    // setup for 4 burst strided transfers. 0x0D is the register
    // address offset of the CCR registers in the timer peripheral
    group.pwm_drv->tim->DCR = 0x0D | STM32_TIM_DCR_DBL(3);

    dmaStreamEnable(group.dma);
}

/*
  DMA interrupt handler. Used to mark DMA completed for DShot
 */
void RCOutput::dma_irq_callback(void *p, uint32_t flags)
{
    pwm_group *group = (pwm_group *)p;
    dmaStreamDisable(group->dma);
    group->dma_handle->unlock();
}

#endif // HAL_USE_PWM
