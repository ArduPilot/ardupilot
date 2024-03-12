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
 * Code by Andy Piper and Siddharth Bharat Purohit
 * 
 * There really is no dshot reference. For information try these resources:
 * https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
 * https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs
 */

#include <hal.h>

#if defined(IOMCU_FW) && HAL_DSHOT_ENABLED
// need to give the little guy as much help as possible
#pragma GCC optimize("O2")

#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include "GPIO.h"
#include "Scheduler.h"

#if HAL_USE_PWM == TRUE

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

#ifdef HAL_WITH_BIDIR_DSHOT
THD_WORKING_AREA(dshot_thread_wa, 512);
#else
THD_WORKING_AREA(dshot_thread_wa, 64);
#endif
static const char* rcout_thread_name = "rcout";

void RCOutput::timer_tick()
{
    if (dshot_timer_setup) {
        return;
    }

    uint32_t dshot_mask;
    if (is_dshot_protocol(get_output_mode(dshot_mask))) {
        chThdCreateStatic(dshot_thread_wa, sizeof(dshot_thread_wa),
                            APM_RCOUT_PRIORITY, &RCOutput::dshot_send_trampoline, this);
        dshot_timer_setup = true;
    }
}

void RCOutput::dshot_send_trampoline(void *p)
{
    RCOutput *rcout = (RCOutput *)p;
    rcout->rcout_thread();
}

/*
  thread for handling RCOutput send on IOMCU
 */
void RCOutput::rcout_thread() {
    // don't start outputting until fully configured
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }

    rcout_thread_ctx = chThdGetSelfX();
    chRegSetThreadNameX(rcout_thread_ctx, rcout_thread_name);

    rcout_timer_t last_cycle_run_us = 0;

    while (true) {
        chEvtWaitOne(EVT_PWM_SEND | EVT_PWM_SYNTHETIC_SEND);

        // start the clock
        const rcout_timer_t last_thread_run_us = rcout_micros();

        // this is when the cycle is supposed to start
        if (_dshot_cycle == 0) {
            last_cycle_run_us = rcout_micros();
            // register a timer for the next tick if push() will not be providing it
            if (_dshot_rate != 1) {
                chVTSet(&_dshot_rate_timer, chTimeUS2I(_dshot_period_us), dshot_update_tick, this);
            }
        }

        // if DMA sharing is in effect there can be quite a delay between the request to begin the cycle and
        // actually sending out data - thus we need to work out how much time we have left to collect the locks
        const rcout_timer_t timeout_period_us = _dshot_rate ? (_dshot_cycle + 1) * _dshot_period_us : _dshot_period_us;
        // timeout is measured from the beginning of the push() that initiated it to preserve periodicity
        const rcout_timer_t cycle_start_us = _dshot_rate ? last_cycle_run_us : last_thread_run_us;

        // DMA channel sharing on F10x is complicated. The allocations are
        // TIM2_UP  - (1,2)
        // TIM4_UP  - (1,7)
        // TIM3_UP  - (1,3)
        // TIM2_CH2 - (1,7) - F103 only
        // TIM4_CH3 - (1,5) - F103 only
        // TIM3_CH4 - (1,3) - F103 only
        // and (1,7) is also shared with USART2_TX
        // locks have to be unlocked in reverse order, and shared CH locks do not need to be taken so the
        // ordering that will work follows. This relies on recursive lock behaviour that allows us to relock
        // a mutex without releasing it first:
        // TIM4_UP  - lock   (shared)
        // TIM4     - dshot send
        // TIM4_CH3 - lock
        // TIM2_UP  - lock
        // TIM2_CH2 - lock recursive (shared)
        // TIM2     - dshot send
        // TIM3_UP  - lock
        // [TIM3_CH4 - shared lock]
        // TIM3     - dshot send
        // [TIM3_CH4 - shared unlock]
        // TIM3_UP  - unlock
        // TIM2_CH2 - unlock recursive (shared)
        // TIM2_UP  - unlock
        // TIM4_CH3 - unlock
        // TIM4_UP  - unlock

        dshot_send_groups(cycle_start_us, timeout_period_us);
#if AP_HAL_SHARED_DMA_ENABLED
        dshot_collect_dma_locks(cycle_start_us, timeout_period_us);
#endif
        if (_dshot_rate > 0) {
            _dshot_cycle = (_dshot_cycle + 1) % _dshot_rate;
        }
    }
}

#if defined(HAL_WITH_BIDIR_DSHOT) && defined(STM32F1)
// reset pwm driver to output mode without resetting the clock or the peripheral
// the code here is the equivalent of pwmStart()/pwmStop()
void RCOutput::bdshot_reset_pwm_f1(pwm_group& group, uint8_t telem_channel)
{
    osalSysLock();

    stm32_tim_t* TIMx = group.pwm_drv->tim;
    // pwmStop sets these
    TIMx->CR1  = 0;                    /* Timer disabled.              */
    TIMx->DIER = 0;                    /* All IRQs disabled.           */
    TIMx->SR   = 0;                    /* Clear eventual pending IRQs. */
    TIMx->CNT = 0;
    TIMx->CCR[0] = 0;                  /* Comparator 1 disabled.       */
    TIMx->CCR[1] = 0;                  /* Comparator 2 disabled.       */
    TIMx->CCR[2] = 0;                  /* Comparator 3 disabled.       */
    TIMx->CCR[3] = 0;                  /* Comparator 4 disabled.       */
    // at the point this is called we will have done input capture on two CC channels
    // we need to switch those channels back to output and the default settings
    // all other channels will not have been modified
    switch (group.bdshot.telem_tim_ch[telem_channel]) {
    case 0: // CC1
    case 1: // CC2
        MODIFY_REG(TIMx->CCER, TIM_CCER_CC2E | TIM_CCER_CC1E, 0);   // disable CC so that it can be modified
        MODIFY_REG(TIMx->CCMR1, (TIM_CCMR1_CC1S | TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC),
            STM32_TIM_CCMR1_OC1M(6) | STM32_TIM_CCMR1_OC1PE);
        MODIFY_REG(TIMx->CCMR1, (TIM_CCMR1_CC2S | TIM_CCMR1_IC2F | TIM_CCMR1_IC2PSC),
            STM32_TIM_CCMR1_OC2M(6) | STM32_TIM_CCMR1_OC2PE);
        MODIFY_REG(TIMx->CCER, (TIM_CCER_CC1P | TIM_CCER_CC2P),
            (TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1E | TIM_CCER_CC2E));
        break;
    case 2: // CC3
    case 3: // CC4
        MODIFY_REG(TIMx->CCER, TIM_CCER_CC3E | TIM_CCER_CC4E, 0);   // disable CC so that it can be modified
        MODIFY_REG(TIMx->CCMR2, (TIM_CCMR2_CC3S | TIM_CCMR2_IC3F | TIM_CCMR2_IC3PSC),
            STM32_TIM_CCMR2_OC3M(6) | STM32_TIM_CCMR2_OC3PE);
        MODIFY_REG(TIMx->CCMR2, (TIM_CCMR2_CC4S | TIM_CCMR2_IC4F | TIM_CCMR2_IC4PSC),
            STM32_TIM_CCMR2_OC4M(6) | STM32_TIM_CCMR2_OC4PE);
        MODIFY_REG(TIMx->CCER, (TIM_CCER_CC3P | TIM_CCER_CC4P),
            (TIM_CCER_CC3P | TIM_CCER_CC4P | TIM_CCER_CC3E | TIM_CCER_CC4E));
        break;
    default:
        break;
    }
    // pwmStart sets these
    uint32_t psc = (group.pwm_drv->clock / group.pwm_drv->config->frequency) - 1;
    TIMx->PSC  = psc;
    TIMx->ARR  = group.pwm_drv->period - 1;
    TIMx->CR2  = group.pwm_drv->config->cr2;
    TIMx->EGR   = STM32_TIM_EGR_UG;      /* Update event.                */
    TIMx->SR    = 0;                     /* Clear pending IRQs.          */
    TIMx->DIER  = group.pwm_drv->config->dier &   /* DMA-related DIER settings.   */
                        ~STM32_TIM_DIER_IRQ_MASK;
    if (group.pwm_drv->has_bdtr) {
        TIMx->BDTR  = group.pwm_drv->config->bdtr | STM32_TIM_BDTR_MOE;
    }

    // we need to switch every output on the same input channel to avoid
    // spurious line changes
    for (uint8_t i = 0; i<4; i++) {
        if (group.chan[i] == CHAN_DISABLED) {
            continue;
        }
        if (group.bdshot.telem_tim_ch[telem_channel] == group.bdshot.telem_tim_ch[i]) {
            palSetLineMode(group.pal_lines[i], PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        }
    }

    /* Timer configured and started.*/
    TIMx->CR1   = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;

    osalSysUnlock();
}

// see https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625
// called from the interrupt
void RCOutput::bdshot_receive_pulses_DMAR_f1(pwm_group* group)
{
    // make sure the transaction finishes or times out, this function takes a little time to run so the most
    // accurate timing is from the beginning. the pulse time is slightly longer than we need so an extra 10U
    // should be plenty
    chVTSetI(&group->dma_timeout, chTimeUS2I(group->dshot_pulse_send_time_us + 30U + 10U),
        bdshot_finish_dshot_gcr_transaction, group);

    group->pwm_drv->tim->CR1 = 0;

    // Configure Timer
    group->pwm_drv->tim->SR = 0;
    // do NOT set CCER to 0 here - this pulls the line low on F103 (at least)
    // and since we are already doing bdshot the relevant options that are set for output
    // also apply to input and bdshot_config_icu_dshot() will disable any channels that need
    // disabling
    group->pwm_drv->tim->DIER = 0;
    group->pwm_drv->tim->CR2 = 0;
    group->pwm_drv->tim->PSC = group->bdshot.telempsc;

    group->dshot_state = DshotState::RECV_START;

    //TOGGLE_PIN_CH_DEBUG(54, curr_ch);
    group->pwm_drv->tim->ARR = 0xFFFF;  // count forever
    group->pwm_drv->tim->CNT = 0;
    uint8_t curr_ch = group->bdshot.curr_telem_chan;

    // we need to switch every input on the same input channel to allow
    // the ESCs to drive the lines
    for (uint8_t i = 0; i<4; i++) {
        if (group->chan[i] == CHAN_DISABLED) {
            continue;
        }
        if (group->bdshot.telem_tim_ch[curr_ch] == group->bdshot.telem_tim_ch[i]) {
            palSetLineMode(group->pal_lines[i], PAL_MODE_INPUT_PULLUP);
        }
    }

    // Initialise ICU channels
    bdshot_config_icu_dshot_f1(group->pwm_drv->tim, curr_ch, group->bdshot.telem_tim_ch[curr_ch]);

    const stm32_dma_stream_t *ic_dma =
        group->has_shared_ic_up_dma() ? group->dma : group->bdshot.ic_dma[curr_ch];

    // Configure DMA
    dmaStreamSetPeripheral(ic_dma, &(group->pwm_drv->tim->DMAR));
    dmaStreamSetMemory0(ic_dma, group->dma_buffer);
    dmaStreamSetTransactionSize(ic_dma, GCR_TELEMETRY_BIT_LEN);
    dmaStreamSetMode(ic_dma,
                    STM32_DMA_CR_CHSEL(group->dma_ch[curr_ch].channel) |
                    STM32_DMA_CR_DIR_P2M |
                    STM32_DMA_CR_PSIZE_HWORD |
                    STM32_DMA_CR_MSIZE_HWORD |
                    STM32_DMA_CR_MINC | STM32_DMA_CR_PL(3) |
                    STM32_DMA_CR_TEIE | STM32_DMA_CR_TCIE);

    // setup for transfers. 0x0D is the register
    // address offset of the CCR registers in the timer peripheral
    uint8_t telem_ch_pair = group->bdshot.telem_tim_ch[curr_ch] & ~1U; // round to the lowest of the channel pair
    const uint8_t ccr_ofs = offsetof(stm32_tim_t, CCR)/4 + telem_ch_pair;
    group->pwm_drv->tim->DCR = STM32_TIM_DCR_DBA(ccr_ofs) | STM32_TIM_DCR_DBL(1); // read two registers at a time

    // Start Timer
    group->pwm_drv->tim->EGR |= STM32_TIM_EGR_UG;
    group->pwm_drv->tim->SR = 0;
    group->pwm_drv->tim->CR1 = TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_UDIS | STM32_TIM_CR1_CEN;
    dmaStreamEnable(ic_dma);
}

void RCOutput::bdshot_config_icu_dshot_f1(stm32_tim_t* TIMx, uint8_t chan, uint8_t ccr_ch)
{
    // F103 does not support both edges input capture so we need to set up two channels
    // both pointing at the same input to capture the data. The triggered channel
    // needs to handle the second edge - so rising or falling - so that we get an
    // even number of half-words in the DMA buffer
    switch(ccr_ch) {
    case 0:
    case 1: {
        // Disable the IC1 and IC2: Reset the CCxE Bit
        MODIFY_REG(TIMx->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E, 0);
        // Select the Input and set the filter and the prescaler value
        if (chan == 0) {    // TI1
            MODIFY_REG(TIMx->CCMR1,
                        (TIM_CCMR1_CC1S | TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC),
                        (TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1F_1));// 4 samples per output transition
            MODIFY_REG(TIMx->CCMR1,
                        (TIM_CCMR1_CC2S | TIM_CCMR1_IC2F | TIM_CCMR1_IC2PSC),
                        (TIM_CCMR1_CC2S_1 | TIM_CCMR1_IC2F_1));
        } else {            // TI2
            MODIFY_REG(TIMx->CCMR1,
                        (TIM_CCMR1_CC1S | TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC),
                        (TIM_CCMR1_CC1S_1 | TIM_CCMR1_IC1F_1));
            MODIFY_REG(TIMx->CCMR1,
                        (TIM_CCMR1_CC2S | TIM_CCMR1_IC2F | TIM_CCMR1_IC2PSC),
                        (TIM_CCMR1_CC2S_0 | TIM_CCMR1_IC2F_1));
        }
        if (ccr_ch == 0) {
            // Select the Polarity as falling on IC2 and rising on IC1
            MODIFY_REG(TIMx->CCER, TIM_CCER_CC1P | TIM_CCER_CC2P, TIM_CCER_CC2P | TIM_CCER_CC1E | TIM_CCER_CC2E);
            MODIFY_REG(TIMx->DIER, TIM_DIER_CC1DE | TIM_DIER_CC2DE, TIM_DIER_CC1DE);
        } else {
            // Select the Polarity as falling on IC1 and rising on IC2
            MODIFY_REG(TIMx->CCER, TIM_CCER_CC1P | TIM_CCER_CC2P, TIM_CCER_CC1P | TIM_CCER_CC1E | TIM_CCER_CC2E);
            MODIFY_REG(TIMx->DIER, TIM_DIER_CC1DE | TIM_DIER_CC2DE, TIM_DIER_CC2DE);
        }
        break;
    }
    case 2:
    case 3: {
        MODIFY_REG(TIMx->CCER, TIM_CCER_CC3E | TIM_CCER_CC4E, 0);
        // Select the Input and set the filter and the prescaler value
        if (chan == 2) {    // TI3
            MODIFY_REG(TIMx->CCMR2,
                        (TIM_CCMR2_CC3S | TIM_CCMR2_IC3F | TIM_CCMR2_IC3PSC),
                        (TIM_CCMR2_CC3S_0 | TIM_CCMR2_IC3F_1));
            MODIFY_REG(TIMx->CCMR2,
                        (TIM_CCMR2_CC4S | TIM_CCMR2_IC4F | TIM_CCMR2_IC4PSC),
                        (TIM_CCMR2_CC4S_1 | TIM_CCMR2_IC4F_1));
        } else {            // TI4
            MODIFY_REG(TIMx->CCMR2,
                        (TIM_CCMR2_CC3S | TIM_CCMR2_IC3F | TIM_CCMR2_IC3PSC),
                        (TIM_CCMR2_CC3S_1 | TIM_CCMR2_IC3F_1));
            MODIFY_REG(TIMx->CCMR2,
                        (TIM_CCMR2_CC4S | TIM_CCMR2_IC4F | TIM_CCMR2_IC4PSC),
                        (TIM_CCMR2_CC4S_0 | TIM_CCMR2_IC4F_1));
        }
        if (ccr_ch == 2) {
            // Select the Polarity as falling on IC4 and rising on IC3
            MODIFY_REG(TIMx->CCER, TIM_CCER_CC3P | TIM_CCER_CC4P, TIM_CCER_CC4P | TIM_CCER_CC3E | TIM_CCER_CC4E);
            MODIFY_REG(TIMx->DIER, TIM_DIER_CC3DE | TIM_DIER_CC4DE, TIM_DIER_CC3DE);
        } else {
            // Select the Polarity as falling on IC3 and rising on IC4
            MODIFY_REG(TIMx->CCER, TIM_CCER_CC3P | TIM_CCER_CC4P, TIM_CCER_CC3P | TIM_CCER_CC3E | TIM_CCER_CC4E);
            MODIFY_REG(TIMx->DIER, TIM_DIER_CC3DE | TIM_DIER_CC4DE, TIM_DIER_CC4DE);
        }
        break;

    }
    default:
        break;
    }
}

// decode a telemetry packet from a GCR encoded stride buffer, take from betaflight decodeTelemetryPacket
// see https://github.com/betaflight/betaflight/pull/8554#issuecomment-512507625 for a description of the protocol
uint32_t RCOutput::bdshot_decode_telemetry_packet_f1(dmar_uint_t* buffer, uint32_t count, bool reversed)
{
    if (!reversed) {
        return bdshot_decode_telemetry_packet(buffer, count);
    }

    uint32_t value = 0;
    uint32_t bits = 0;
    uint32_t len;

    // on F103 we are reading one edge with ICn and the other with ICn+1, the DMA architecture only
    // allows to trigger on a single register dictated by the DMA input capture channel being used.
    // even though we are reading multiple registers per transfer we always cannot trigger on one or other
    // of the registers and if the one we trigger on is the one that is numerically first each register
    // pair that we read will be swapped in time. in this case we trigger on ICn and then read CCRn and CCRn+1
    // giving us the new value of ICn and the old value of ICn+1. in order to avoid reading garbage on the
    // first read we trigger ICn on the rising edge. this gives us all the data but with each pair of bytes
    // transposed. we thus need to untranspose as we decode
    dmar_uint_t oldValue = buffer[1];

    for (int32_t i = 0; i <= count+1; ) {
        if (i < count) {
            dmar_int_t diff = buffer[i] - oldValue;
            if (bits >= 21U) {
                break;
            }
            len = (diff + TELEM_IC_SAMPLE/2U) / TELEM_IC_SAMPLE;
        } else {
            len = 21U - bits;
        }

        value <<= len;
        value |= 1U << (len - 1U);
        oldValue = buffer[i];
        bits += len;

        i += (i%2 ? -1 : 3);
    }


    if (bits != 21U) {
        return INVALID_ERPM;
    }

    static const uint32_t decode[32] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };

    uint32_t decodedValue = decode[value & 0x1fU];
    decodedValue |= decode[(value >> 5U) & 0x1fU] << 4U;
    decodedValue |= decode[(value >> 10U) & 0x1fU] << 8U;
    decodedValue |= decode[(value >> 15U) & 0x1fU] << 12U;

    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8U); // xor bytes
    csum = csum ^ (csum >> 4U); // xor nibbles

    if ((csum & 0xfU) != 0xfU) {
        return INVALID_ERPM;
    }
    decodedValue >>= 4;

    return decodedValue;
}

#endif // HAL_WITH_BIDIR_DSHOT && STM32F1

#endif // HAL_USE_PWM

#endif // IOMCU_FW && HAL_DSHOT_ENABLED
