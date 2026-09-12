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

#include <hal.h>
#include "RCInput.h"
#include "hal.h"
#include "hwdef/common/ppm.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <AP_RCProtocol/AP_RCProtocol_config.h>

#include <AP_Math/AP_Math.h>

#include <GCS_MAVLink/GCS.h>

#define SIG_DETECT_TIMEOUT_US 500000
using namespace ChibiOS;
extern const AP_HAL::HAL& hal;
void RCInput::init()
{
#if AP_RCPROTOCOL_ENABLED
    AP::RC().init();
#endif

#if defined(HAL_USE_ICU) && (HAL_USE_ICU == TRUE)
    //attach timer channel on which the signal will be received
    sig_reader.attach_capture_timer(&RCIN_ICU_TIMER, RCIN_ICU_CHANNEL, STM32_RCIN_DMA_STREAM, STM32_RCIN_DMA_CHANNEL);
    pulse_input_enabled = true;
#endif

#if defined(HAL_USE_EICU) && (HAL_USE_EICU == TRUE)
    sig_reader.init(&RCININT_EICU_TIMER, RCININT_EICU_CHANNEL);
    pulse_input_enabled = true;
#endif

#if defined(HAL_RCIN_IS_GPIO)
    sig_reader.init(HAL_RCIN_GPIO_LINE);
    pulse_input_enabled = true;
#endif

    _init = true;
}

/*
  enable or disable pulse input for RC input. This is used to reduce
  load when we are decoding R/C via a UART
*/
void RCInput::pulse_input_enable(bool enable)
{
    pulse_input_enabled = enable;
#if (defined(HAL_USE_ICU) && (HAL_USE_ICU == TRUE)) || (defined(HAL_USE_EICU) && (HAL_USE_EICU == TRUE)) || defined(HAL_RCIN_IS_GPIO)
    if (!enable) {
        sig_reader.disable();
    }
#endif
}

bool RCInput::new_input()
{
    if (!_init) {
        return false;
    }
    bool valid;
    {
        WITH_SEMAPHORE(rcin_mutex);
        valid = _rcin_timestamp_last_signal != _last_read;
        _last_read = _rcin_timestamp_last_signal;
    }

    return valid;
}

uint8_t RCInput::num_channels()
{
    if (!_init) {
        return 0;
    }
    return _num_channels;
}

uint16_t RCInput::read(uint8_t channel)
{
    if (!_init || (channel >= MIN(RC_INPUT_MAX_CHANNELS, _num_channels))) {
        return 0;
    }
    uint16_t v;
    {
        WITH_SEMAPHORE(rcin_mutex);
        v = _rc_values[channel];
    }
    return v;
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (!_init) {
        return false;
    }

    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }
    {
        WITH_SEMAPHORE(rcin_mutex);
        memcpy(periods, _rc_values, len*sizeof(periods[0]));
    }
    return len;
}

// there is no point enabling the thread if you're not using the
// RCProtocol library - this catches regressions in our hwdefs.  It
// can be removed in the future when the chance of there being
// outstanding work with regressions in is negigible
#if HAL_RCIN_THREAD_ENABLED && !AP_RCPROTOCOL_ENABLED
#error Do not enable the RCIN thread when not doing AP_RCPROTOCOL_ENABLED
#endif

void RCInput::_timer_tick(void)
{
    if (!_init) {
        return;
    }
#if AP_RCPROTOCOL_ENABLED
    AP_RCProtocol &rcprot = AP::RC();

#if defined(HAL_USE_ICU) && (HAL_USE_ICU == TRUE)
    if (pulse_input_enabled) {
        const uint32_t *p;
        uint32_t n;
        while ((p = (const uint32_t *)sig_reader.sigbuf.readptr(n)) != nullptr) {
            rcprot.process_pulse_list(p, n*2, sig_reader.need_swap);
            sig_reader.sigbuf.advance(n);
        }
    }
#endif

#if defined(HAL_USE_EICU) && (HAL_USE_EICU == TRUE)
    if (pulse_input_enabled) {
        uint32_t width_s0, width_s1;
        while(sig_reader.read(width_s0, width_s1)) {
            rcprot.process_pulse(width_s0, width_s1);
        }
    }
#endif

#if defined(HAL_RCIN_IS_GPIO)
    if (pulse_input_enabled) {
        sig_reader.enable();
        uint32_t width_s0, width_s1;
        while (sig_reader.read(width_s0, width_s1)) {
            rcprot.process_pulse(width_s0, width_s1);
        }
    }
#endif

    if (rcprot.new_input()) {
        WITH_SEMAPHORE(rcin_mutex);
// This field is only used as a change marker for new_input().
// Using a local generation count avoids an expensive hrt_micros64() read in the RC polling path, which can starve Pico2 USB CDC under GPIO pulse bursts.
        _rcin_timestamp_last_signal++;
        _num_channels = rcprot.num_channels();
        _num_channels = MIN(_num_channels, RC_INPUT_MAX_CHANNELS);
        rcprot.read(_rc_values, _num_channels);
        _rssi = rcprot.get_RSSI();
        _rx_link_quality = rcprot.get_rx_link_quality();
    }

#endif  // AP_RCPROTOCOL_ENABLED

    // note, we rely on the vehicle code checking new_input()
    // and a timeout for the last valid input to handle failsafe
}

#endif //#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
