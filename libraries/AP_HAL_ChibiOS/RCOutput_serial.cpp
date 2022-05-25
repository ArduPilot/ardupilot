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
 */

#include <hal.h>
#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include "hwdef/common/stm32_util.h"
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#if HAL_USE_PWM == TRUE
#ifndef DISABLE_DSHOT

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

bool RCOutput::dshot_send_command(pwm_group& group, uint8_t command, uint8_t chan)
{
    if (!group.can_send_dshot_pulse()) {
        return false;
    }

    if (irq.waiter || (group.dshot_state != DshotState::IDLE && group.dshot_state != DshotState::RECV_COMPLETE)) {
        // doing serial output or DMAR input, don't send DShot pulses
        return false;
    }

#ifdef HAL_GPIO_LINE_GPIO81
    TOGGLE_PIN_DEBUG(81);
#endif
    // first make sure we have the DMA channel before anything else

    osalDbgAssert(!group.dma_handle->is_locked(), "DMA handle is already locked");
    group.dma_handle->lock();

    // only the timer thread releases the locks
    group.dshot_waiter = rcout_thread_ctx;
    bool bdshot_telem = false;
#ifdef HAL_WITH_BIDIR_DSHOT
    uint32_t active_channels = group.ch_mask & group.en_mask;
    // no need to get the input capture lock
    group.bdshot.enabled = false;
    if ((_bdshot.mask & active_channels) == active_channels) {
        bdshot_telem = true;
        // it's not clear why this is required, but without it we get no output
        if (group.pwm_started) {
            pwmStop(group.pwm_drv);
        }
        pwmStart(group.pwm_drv, &group.pwm_cfg);
        group.pwm_started = true;
    }
#endif    

    memset((uint8_t *)group.dma_buffer, 0, DSHOT_BUFFER_LENGTH);

    // keep the other ESCs armed rather than sending nothing
    const uint16_t zero_packet = create_dshot_packet(0, false, bdshot_telem);
    const uint16_t packet = create_dshot_packet(command, true, bdshot_telem);

    for (uint8_t i = 0; i < 4; i++) {
        if (!group.is_chan_enabled(i)) {
            continue;
        }

        if (group.chan[i] == chan || chan == RCOutput::ALL_CHANNELS) {
            fill_DMA_buffer_dshot(group.dma_buffer + i, 4, packet, group.bit_width_mul);
        } else {
            fill_DMA_buffer_dshot(group.dma_buffer + i, 4, zero_packet, group.bit_width_mul);
        }
    }

    chEvtGetAndClearEvents(group.dshot_event_mask);
    // start sending the pulses out
    send_pulses_DMAR(group, DSHOT_BUFFER_LENGTH);
#ifdef HAL_GPIO_LINE_GPIO81
    TOGGLE_PIN_DEBUG(81);
#endif

    return true;
}

// Send a dshot command, if command timout is 0 then 10 commands are sent
// chan is the servo channel to send the command to
void RCOutput::send_dshot_command(uint8_t command, uint8_t chan, uint32_t command_timeout_ms, uint16_t repeat_count, bool priority)
{
    // once armed only priority commands will be accepted
    if (hal.util->get_soft_armed() && !priority) {
        return;
    }
    // not an FMU channel
    if (chan < chan_offset) {
        return;
    }

    DshotCommandPacket pkt;
    pkt.command = command;
    pkt.chan = chan - chan_offset;
    if (command_timeout_ms == 0) {
        pkt.cycle = MAX(10, repeat_count);
    } else {
        pkt.cycle = MAX(command_timeout_ms * 1000 / _dshot_period_us, repeat_count);
    }

    // prioritize anything that is not an LED or BEEP command
    if (!_dshot_command_queue.push(pkt) && priority) {
        _dshot_command_queue.push_force(pkt);
    }
}

// Set the dshot outputs that should be reversed (as opposed to 3D)
// The chanmask passed is added (ORed) into any existing mask.
// The mask uses servo channel numbering
void RCOutput::set_reversed_mask(uint32_t chanmask) {
    _reversed_mask |= (chanmask >> chan_offset);
}

// Set the dshot outputs that should be reversible/3D
// The chanmask passed is added (ORed) into any existing mask.
// The mask uses servo channel numbering
void RCOutput::set_reversible_mask(uint32_t chanmask) {
    _reversible_mask |= (chanmask >> chan_offset);
}

// Update the dshot outputs that should be reversible/3D at 1Hz
void RCOutput::update_channel_masks() {

    // post arming dshot commands will not be accepted
    if (hal.util->get_soft_armed() || _disable_channel_mask_updates) {
        return;
    }

#if HAL_PWM_COUNT > 0
    for (uint8_t i=0; i<HAL_PWM_COUNT; i++) {
        switch (_dshot_esc_type) {
            case DSHOT_ESC_BLHELI:
            case DSHOT_ESC_BLHELI_S:
                if (_reversible_mask & (1U<<i)) {
                    send_dshot_command(DSHOT_3D_ON, i + chan_offset, 0, 10, true);
                }
                if (_reversed_mask & (1U<<i)) {
                    send_dshot_command(DSHOT_REVERSE, i + chan_offset, 0, 10, true);
                }
                break;
            default:
                break;
        }
    }
#endif
}

#endif // DISABLE_DSHOT
#endif // HAL_USE_PWM
