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
 * Bi-directional dshot based on Betaflight, code by Andy Piper and Siddharth Bharat Purohit
 */

#include <hal.h>
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include "hwdef/common/stm32_util.h"
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#ifdef HAL_WITH_BIDIR_DSHOT

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

#if RCOU_DSHOT_TIMING_DEBUG
#define DEBUG_CHANNEL 1
#define TOGGLE_PIN_CH_DEBUG(pin, channel) do { if (channel == DEBUG_CHANNEL) palToggleLine(HAL_GPIO_LINE_GPIO ## pin); } while (0)
#else
#define TOGGLE_PIN_CH_DEBUG(pin, channel) do {} while (0)
#endif

#define TELEM_IC_SAMPLE 16

/*
 * enable bi-directional telemetry request for a mask of channels. This is used
 * with DShot to get telemetry feedback
 */
void RCOutput::set_bidir_dshot_mask(uint32_t mask)
{
    _bdshot.mask = (mask >> chan_offset);
    // we now need to reconfigure the DMA channels since they are affected by the value of the mask
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        pwm_group &group = pwm_group_list[i];
        if (((group.ch_mask << chan_offset) & mask) == 0) {
            // this group is not affected
            continue;
        }
        set_group_mode(group);
    }
}

bool RCOutput::bdshot_setup_group_ic_DMA(pwm_group &group)
{
    // check if already allocated
    if (group.has_ic_dma()) {
        return true;
    }

    // allocate input capture DMA handles
    for (uint8_t i = 0; i < 4; i++) {
        if (!group.is_chan_enabled(i) ||
            !group.dma_ch[i].have_dma || !(_bdshot.mask & (1 << group.chan[i]))) {
            continue;
        }
        pwmmode_t mode = group.pwm_cfg.channels[i].mode;
        if (mode == PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW ||
            mode == PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH) {
            // Complementary channels don't support input capture
            // Return error
            return false;
        }
        if (!group.bdshot.ic_dma_handle[i]) {
            // share up channel if required
            if (group.dma_ch[i].stream_id == group.dma_up_stream_id) {
                group.bdshot.ic_dma_handle[i] = group.dma_handle;
            } else {
                group.bdshot.ic_dma_handle[i] = new Shared_DMA(group.dma_ch[i].stream_id, SHARED_DMA_NONE,
                                                FUNCTOR_BIND_MEMBER(&RCOutput::bdshot_ic_dma_allocate, void, Shared_DMA *),
                                                FUNCTOR_BIND_MEMBER(&RCOutput::bdshot_ic_dma_deallocate, void, Shared_DMA *));
            }
            if (!group.bdshot.ic_dma_handle[i]) {
                return false;
            }
        }
    }

    // We might need to do sharing of timers for telemetry feedback
    // due to lack of available DMA channels
    for (uint8_t i = 0; i < 4; i++) {
        // we must pull all the allocated channels high to prevent them going low
        // when the pwm peripheral is stopped
        if (group.chan[i] != CHAN_DISABLED && _bdshot.mask & group.ch_mask) {
            // bi-directional dshot requires less than MID2 speed and PUSHPULL in order to avoid noise on the line
            // when switching from output to input
            palSetLineMode(group.pal_lines[i], PAL_MODE_ALTERNATE(group.alt_functions[i])
                | PAL_STM32_OTYPE_PUSHPULL | PAL_STM32_PUPDR_PULLUP | PAL_STM32_OSPEED_MID1);
        }

        if (!group.is_chan_enabled(i) || !(_bdshot.mask & (1 << group.chan[i]))) {
            continue;
        }
        uint8_t curr_chan = i;
        if (group.bdshot.ic_dma_handle[i]) {
            // we are all good just set and continue
            group.bdshot.telem_tim_ch[i] = curr_chan;
        } else {
            // I guess we have to share, but only channels 1 & 2 or 3 & 4
            if (curr_chan % 2 == 0) {
                curr_chan = curr_chan + 1;
            } else {
                curr_chan = curr_chan - 1;
            }
            if (!group.dma_ch[curr_chan].have_dma) {
                // We can't find a DMA channel to use so
                // return error
                return false;
            }
            if (group.bdshot.ic_dma_handle[i]) {
                INTERNAL_ERROR(AP_InternalError::error_t::dma_fail);
                return false;
            }
            // share up channel if required
            if (group.dma_ch[curr_chan].stream_id == group.dma_up_stream_id) {
                group.bdshot.ic_dma_handle[i] = group.dma_handle;
            } else {
                // we can use the next channel
                group.bdshot.ic_dma_handle[i] = new Shared_DMA(group.dma_ch[curr_chan].stream_id, SHARED_DMA_NONE,
                                            FUNCTOR_BIND_MEMBER(&RCOutput::bdshot_ic_dma_allocate, void, Shared_DMA *),
                                            FUNCTOR_BIND_MEMBER(&RCOutput::bdshot_ic_dma_deallocate, void, Shared_DMA *));
            }
            if (!group.bdshot.ic_dma_handle[i]) {
                return false;
            }
            group.bdshot.telem_tim_ch[i] = curr_chan;
            group.dma_ch[i] = group.dma_ch[curr_chan];
        }
    }

    // now allocate the starting channel
    for (uint8_t i = 0; i < 4; i++) {
        if (group.chan[i] != CHAN_DISABLED && group.bdshot.ic_dma_handle[i] != nullptr) {
            group.bdshot.curr_telem_chan = i;
            break;
        }
    }

    return true;
}

/*
  allocate DMA channel
 */
void RCOutput::bdshot_ic_dma_allocate(Shared_DMA *ctx)
{
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        pwm_group &group = pwm_group_list[i];
        for (uint8_t icuch = 0; icuch < 4; icuch++) {
            if (group.bdshot.ic_dma_handle[icuch] == ctx && group.bdshot.ic_dma[icuch] == nullptr) {
                chSysLock();
                group.bdshot.ic_dma[icuch] = dmaStreamAllocI(group.dma_ch[icuch].stream_id, 10, bdshot_dma_ic_irq_callback, &group);
                chSysUnlock();
#if STM32_DMA_SUPPORTS_DMAMUX
                if (group.bdshot.ic_dma[icuch]) {
                    dmaSetRequestSource(group.bdshot.ic_dma[icuch], group.dma_ch[icuch].channel);
                }
#endif
            }
        }
    }
}

/*
  deallocate DMA channel
 */
void RCOutput::bdshot_ic_dma_deallocate(Shared_DMA *ctx)
{
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        pwm_group &group = pwm_group_list[i];
        for (uint8_t icuch = 0; icuch < 4; icuch++) {
            if (group.bdshot.ic_dma_handle[icuch] == ctx && group.bdshot.ic_dma[icuch] != nullptr) {
                chSysLock();
                dmaStreamFreeI(group.bdshot.ic_dma[icuch]);
                group.bdshot.ic_dma[icuch] = nullptr;
                chSysUnlock();
            }
        }
    }
}

// see https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625
// called from the interrupt
#pragma GCC push_options
#pragma GCC optimize("O2")
void RCOutput::bdshot_receive_pulses_DMAR(pwm_group* group)
{
    // make sure the transaction finishes or times out, this function takes a little time to run so the most
    // accurate timing is from the beginning. the pulse time is slightly longer than we need so an extra 10U
    // should be plenty
    chVTSetI(&group->dma_timeout, chTimeUS2I(group->dshot_pulse_send_time_us + 30U + 10U),
        bdshot_finish_dshot_gcr_transaction, group);
    uint8_t curr_ch = group->bdshot.curr_telem_chan;

    group->pwm_drv->tim->CR1 = 0;
    group->pwm_drv->tim->CCER = 0;

    // Configure Timer
    group->pwm_drv->tim->SR = 0;
    group->pwm_drv->tim->CCMR1 = 0;
    group->pwm_drv->tim->CCMR2 = 0;
    group->pwm_drv->tim->DIER = 0;
    group->pwm_drv->tim->CR2 = 0;
    group->pwm_drv->tim->PSC = group->bdshot.telempsc;

    group->dshot_state = DshotState::RECV_START;

    //TOGGLE_PIN_CH_DEBUG(54, curr_ch);
    group->pwm_drv->tim->ARR = 0xFFFF;  // count forever
    group->pwm_drv->tim->CNT = 0;

    // Initialise ICU channels
    bdshot_config_icu_dshot(group->pwm_drv->tim, curr_ch, group->bdshot.telem_tim_ch[curr_ch]);

    // do a little DMA dance when sharing with UP
#if STM32_DMA_SUPPORTS_DMAMUX
    if (group->has_shared_ic_up_dma()) {
        dmaSetRequestSource(group->dma, group->dma_ch[curr_ch].channel);
    }
#endif
    const stm32_dma_stream_t *ic_dma =
        group->has_shared_ic_up_dma() ? group->dma : group->bdshot.ic_dma[curr_ch];

    // Configure DMA
    dmaStreamSetPeripheral(ic_dma, &(group->pwm_drv->tim->DMAR));
    dmaStreamSetMemory0(ic_dma, group->dma_buffer);
    dmaStreamSetTransactionSize(ic_dma, GCR_TELEMETRY_BIT_LEN);
    dmaStreamSetFIFO(ic_dma, STM32_DMA_FCR_DMDIS | STM32_DMA_FCR_FTH_FULL);
    dmaStreamSetMode(ic_dma,
                    STM32_DMA_CR_CHSEL(group->dma_ch[curr_ch].channel) |
                    STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD |
                    STM32_DMA_CR_MINC | STM32_DMA_CR_PL(3) |
                    STM32_DMA_CR_TEIE | STM32_DMA_CR_TCIE);

    // setup for transfers. 0x0D is the register
    // address offset of the CCR registers in the timer peripheral
    group->pwm_drv->tim->DCR = (0x0D + group->bdshot.telem_tim_ch[curr_ch]) | STM32_TIM_DCR_DBL(0);

    // Start Timer
    group->pwm_drv->tim->EGR |= STM32_TIM_EGR_UG;
    group->pwm_drv->tim->SR = 0;
    group->pwm_drv->tim->CR1 = TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_UDIS | STM32_TIM_CR1_CEN;
    dmaStreamEnable(ic_dma);
}


void RCOutput::bdshot_config_icu_dshot(stm32_tim_t* TIMx, uint8_t chan, uint8_t ccr_ch)
{
    switch(ccr_ch) {
    case 0: {
        /* Disable the Channel 1: Reset the CC1E Bit */
        TIMx->CCER &= (uint32_t)~TIM_CCER_CC1E;

        const uint32_t CCMR1_FILT = TIM_CCMR1_IC1F_1;   // 4 samples per output transition
        // Select the Input and set the filter and the prescaler value
        if (chan == 0) {
            MODIFY_REG(TIMx->CCMR1,
                        (TIM_CCMR1_CC1S | TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC),
                        (TIM_CCMR1_CC1S_0 | CCMR1_FILT));
        } else {
            MODIFY_REG(TIMx->CCMR1,
                        (TIM_CCMR1_CC1S | TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC),
                        (TIM_CCMR1_CC1S_1 | CCMR1_FILT));
        }
        // Select the Polarity as Both Edge and set the CC1E Bit
        MODIFY_REG(TIMx->CCER,
                    (TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC1E),
                    (TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC1E));
        MODIFY_REG(TIMx->DIER, TIM_DIER_CC1DE, TIM_DIER_CC1DE);
        break;
    }
    case 1: {
        // Disable the Channel 2: Reset the CC2E Bit
        TIMx->CCER &= (uint32_t)~TIM_CCER_CC2E;

        const uint32_t CCMR1_FILT = TIM_CCMR1_IC2F_1;
        // Select the Input and set the filter and the prescaler value
        if (chan == 0) {
            MODIFY_REG(TIMx->CCMR1,
                        (TIM_CCMR1_CC2S | TIM_CCMR1_IC2F | TIM_CCMR1_IC2PSC),
                        (TIM_CCMR1_CC2S_1 | CCMR1_FILT));
        } else {
            MODIFY_REG(TIMx->CCMR1,
                        (TIM_CCMR1_CC2S | TIM_CCMR1_IC2F | TIM_CCMR1_IC2PSC),
                        (TIM_CCMR1_CC2S_0 | CCMR1_FILT));
        }

        // Select the Polarity as Both Edge and set the CC2E Bit
        MODIFY_REG(TIMx->CCER,
                    TIM_CCER_CC2P | TIM_CCER_CC2NP | TIM_CCER_CC2E,
                    (TIM_CCER_CC2P | TIM_CCER_CC2NP | TIM_CCER_CC2E));
        MODIFY_REG(TIMx->DIER, TIM_DIER_CC2DE, TIM_DIER_CC2DE);
        break;
    }
    case 2: {
        // Disable the Channel 3: Reset the CC3E Bit
        TIMx->CCER &= (uint32_t)~TIM_CCER_CC3E;

        const uint32_t CCMR2_FILT = TIM_CCMR2_IC3F_1;
        // Select the Input and set the filter and the prescaler value
        if (chan == 2) {
            MODIFY_REG(TIMx->CCMR2,
                        (TIM_CCMR2_CC3S | TIM_CCMR2_IC3F | TIM_CCMR2_IC3PSC),
                        (TIM_CCMR2_CC3S_0 | CCMR2_FILT));
        } else {
            MODIFY_REG(TIMx->CCMR2,
                        (TIM_CCMR2_CC3S | TIM_CCMR2_IC3F | TIM_CCMR2_IC3PSC),
                        (TIM_CCMR2_CC3S_1 | CCMR2_FILT));
        }

        // Select the Polarity as Both Edge and set the CC3E Bit
        MODIFY_REG(TIMx->CCER,
                    (TIM_CCER_CC3P | TIM_CCER_CC3NP | TIM_CCER_CC3E),
                    (TIM_CCER_CC3P | TIM_CCER_CC3NP | TIM_CCER_CC3E));
        MODIFY_REG(TIMx->DIER, TIM_DIER_CC3DE, TIM_DIER_CC3DE);
        break;
    }
    case 3: {
        // Disable the Channel 4: Reset the CC4E Bit
        TIMx->CCER &= (uint32_t)~TIM_CCER_CC4E;

        const uint32_t CCMR2_FILT = TIM_CCMR2_IC4F_1;
        // Select the Input and set the filter and the prescaler value
        if (chan == 2) {
            MODIFY_REG(TIMx->CCMR2,
                        (TIM_CCMR2_CC4S | TIM_CCMR2_IC4F | TIM_CCMR2_IC4PSC),
                        (TIM_CCMR2_CC4S_1 | CCMR2_FILT));
        } else {
            MODIFY_REG(TIMx->CCMR2,
                        (TIM_CCMR2_CC4S | TIM_CCMR2_IC4F | TIM_CCMR2_IC4PSC),
                        (TIM_CCMR2_CC4S_0 | CCMR2_FILT));
        }

        // Select the Polarity as Both Edge and set the CC4E Bit
        MODIFY_REG(TIMx->CCER,
                    (TIM_CCER_CC4P | TIM_CCER_CC4NP | TIM_CCER_CC4E),
                    (TIM_CCER_CC4P | TIM_CCER_CC4NP | TIM_CCER_CC4E));

        MODIFY_REG(TIMx->DIER, TIM_DIER_CC4DE, TIM_DIER_CC4DE);
        break;
    }
    default:
        break;
    }
}

/*
  unlock DMA channel after a bi-directional dshot transaction completes
 */
__RAMFUNC__ void RCOutput::bdshot_finish_dshot_gcr_transaction(void *p)
{
    pwm_group *group = (pwm_group *)p;
    chSysLockFromISR();
#ifdef HAL_GPIO_LINE_GPIO56
    TOGGLE_PIN_DEBUG(56);
#endif
    uint8_t curr_telem_chan = group->bdshot.curr_telem_chan;

    // the DMA buffer is either the regular outbound one because we are sharing UP and CH
    // or the input channel buffer
    const stm32_dma_stream_t *dma =
        group->has_shared_ic_up_dma() ? group->dma : group->bdshot.ic_dma[curr_telem_chan];
    // record the transaction size before the stream is released
    dmaStreamDisable(dma);
    group->bdshot.dma_tx_size = MIN(uint16_t(GCR_TELEMETRY_BIT_LEN),
        GCR_TELEMETRY_BIT_LEN - dmaStreamGetTransactionSize(dma));
    stm32_cacheBufferInvalidate(group->dma_buffer, group->bdshot.dma_tx_size);
    memcpy(group->bdshot.dma_buffer_copy, group->dma_buffer, sizeof(uint32_t) * group->bdshot.dma_tx_size);

    group->dshot_state = DshotState::RECV_COMPLETE;

    // if using input capture DMA and sharing the UP and CH channels then clean up
    // by assigning the source back to UP
#if STM32_DMA_SUPPORTS_DMAMUX
    if (group->has_shared_ic_up_dma()) {
        dmaSetRequestSource(group->dma, group->dma_up_channel);
    }
#endif

    // rotate to the next input channel
    group->bdshot.prev_telem_chan = group->bdshot.curr_telem_chan;
    group->bdshot.curr_telem_chan = bdshot_find_next_ic_channel(*group);
    // tell the waiting process we've done the DMA
    chEvtSignalI(group->dshot_waiter, group->dshot_event_mask);
#ifdef HAL_GPIO_LINE_GPIO56
    TOGGLE_PIN_DEBUG(56);
#endif
    chSysUnlockFromISR();
}

/*
  decode returned data from bi-directional dshot
 */
bool RCOutput::bdshot_decode_dshot_telemetry(pwm_group& group, uint8_t chan)
{
    if (!group.is_chan_enabled(chan)) {
        return true;
    }

    // evaluate dshot telemetry
    group.bdshot.erpm[chan] = bdshot_decode_telemetry_packet(group.bdshot.dma_buffer_copy, group.bdshot.dma_tx_size);

    group.dshot_state = DshotState::IDLE;

#if RCOU_DSHOT_TIMING_DEBUG
    // Record Stats
    if (group.bdshot.erpm[chan] != 0xFFFF) {
        group.bdshot.telem_rate[chan]++;
    } else {
#ifdef HAL_GPIO_LINE_GPIO57
        TOGGLE_PIN_DEBUG(57);
#endif
        group.bdshot.telem_err_rate[chan]++;
#ifdef HAL_GPIO_LINE_GPIO57
        TOGGLE_PIN_DEBUG(57);
#endif
    }

    uint64_t now = AP_HAL::micros64();
    if (chan == DEBUG_CHANNEL && (now  - group.bdshot.last_print) > 1000000) {
        hal.console->printf("TELEM: %d <%d Hz, %.1f%% err>", group.bdshot.erpm[chan], group.bdshot.telem_rate[chan],
            100.0f * float(group.bdshot.telem_err_rate[chan]) / (group.bdshot.telem_err_rate[chan] + group.bdshot.telem_rate[chan]));
        hal.console->printf(" %ld ", group.bdshot.dma_buffer_copy[0]);
        for (uint8_t l = 1; l < group.bdshot.dma_tx_size; l++) {
            hal.console->printf(" +%ld ", group.bdshot.dma_buffer_copy[l] - group.bdshot.dma_buffer_copy[l-1]);
        }
        hal.console->printf("\n");

        group.bdshot.telem_rate[chan] = 0;
        group.bdshot.telem_err_rate[chan] = 0;
        group.bdshot.last_print = now;
    }
#endif
    return group.bdshot.erpm[chan] != 0xFFFF;
}

// Find next valid channel for dshot telem
uint8_t RCOutput::bdshot_find_next_ic_channel(const pwm_group& group)
{
    uint8_t chan = group.bdshot.curr_telem_chan;
    for (uint8_t i = 1; i < 4; i++) {
        const uint8_t next_chan = (chan + i) % 4;
        if (group.is_chan_enabled(next_chan) &&
            group.bdshot.ic_dma_handle[next_chan] != nullptr) {
            return next_chan;
        }
    }
    return chan;
}

/*
  DMA UP channel interrupt handler. Used to mark DMA send completed for DShot
 */
__RAMFUNC__ void RCOutput::dma_up_irq_callback(void *p, uint32_t flags)
{
    pwm_group *group = (pwm_group *)p;
    chSysLockFromISR();

    // there is a small chance the shared UP CH codepath will get here
    if (group->bdshot.enabled && group->dshot_state == DshotState::RECV_START) {
        chSysUnlockFromISR();
        return;
    }

    // check nothing bad happened
    if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
        INTERNAL_ERROR(AP_InternalError::error_t::dma_fail);
    }
    dmaStreamDisable(group->dma);

    if (group->in_serial_dma && irq.waiter) {
        // tell the waiting process we've done the DMA
        chEvtSignalI(irq.waiter, serial_event_mask);
    } else if (!group->in_serial_dma && group->bdshot.enabled) {
        group->dshot_state = DshotState::SEND_COMPLETE;
        // sending is done, in 30us the ESC will send telemetry
        bdshot_receive_pulses_DMAR(group);
    } else {
        // non-bidir case, this prevents us ever having two dshot pulses too close together
        if (is_dshot_protocol(group->current_mode)) {
            // since we could be sending a dshot command, wait the full telemetry pulse width
            // dshot mandates a minimum pulse separation of 40us
            chVTSetI(&group->dma_timeout, chTimeUS2I(group->dshot_pulse_send_time_us + 30U + 40U), dma_unlock, p);
        } else {
            // WS2812 mandates a minimum pulse separation of 50us
            chVTSetI(&group->dma_timeout, chTimeUS2I(50U), dma_unlock, p);
        }
    }

    chSysUnlockFromISR();
}

// DMA IC channel handler. Used to mark DMA receive completed for DShot
__RAMFUNC__ void RCOutput::bdshot_dma_ic_irq_callback(void *p, uint32_t flags)
{
    chSysLockFromISR();

    // check nothing bad happened
    if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
        INTERNAL_ERROR(AP_InternalError::error_t::dma_fail);
    }

    chSysUnlockFromISR();
}

/*
    returns the bitrate in Hz of the given output_mode
*/
uint32_t RCOutput::bdshot_get_output_rate_hz(const enum output_mode mode)
{
    switch (mode) {
    case MODE_PWM_DSHOT150:
        return 150000U * 5 / 4;
    case MODE_PWM_DSHOT300:
        return 300000U * 5 / 4;
    case MODE_PWM_DSHOT600:
        return 600000U * 5 / 4;
    case MODE_PWM_DSHOT1200:
        return 1200000U * 5 / 4;
    default:
        // use 1 to prevent a possible divide-by-zero
        return 1;
    }
}

// decode a telemetry packet from a GCR encoded stride buffer, take from betaflight decodeTelemetryPacket
// see https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625 for a description of the protocol
uint32_t RCOutput::bdshot_decode_telemetry_packet(uint32_t* buffer, uint32_t count)
{
    uint32_t value = 0;
    uint32_t oldValue = buffer[0];
    uint32_t bits = 0;
    uint32_t len;

    for (uint32_t i = 1; i <= count; i++) {
        if (i < count) {
            int32_t diff = buffer[i] - oldValue;
            if (bits >= 21) {
                break;
            }
            len = (diff + TELEM_IC_SAMPLE/2) / TELEM_IC_SAMPLE;
        } else {
            len = 21 - bits;
        }

        value <<= len;
        value |= 1 << (len - 1);
        oldValue = buffer[i];
        bits += len;
    }
    if (bits != 21) {
        return 0xffff;
    }

    static const uint32_t decode[32] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };

    uint32_t decodedValue = decode[value & 0x1f];
    decodedValue |= decode[(value >> 5) & 0x1f] << 4;
    decodedValue |= decode[(value >> 10) & 0x1f] << 8;
    decodedValue |= decode[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8); // xor bytes
    csum = csum ^ (csum >> 4); // xor nibbles

    if ((csum & 0xf) != 0xf) {
        return 0xffff;
    }
    decodedValue >>= 4;

    if (decodedValue == 0x0fff) {
        return 0;
    }
    decodedValue = (decodedValue & 0x000001ff) << ((decodedValue & 0xfffffe00) >> 9);
    if (!decodedValue) {
        return 0xffff;
    }
    uint32_t ret = (1000000 * 60 / 100 + decodedValue / 2) / decodedValue;
    return ret;
}
#pragma GCC pop_options


#endif // HAL_WITH_BIDIR_DSHOT
