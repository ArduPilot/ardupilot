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

#if HAL_WITH_IO_MCU
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#include <AP_Math/AP_Math.h>

#ifndef HAL_NO_UARTDRIVER
#include <GCS_MAVLink/GCS.h>
#endif

#define SIG_DETECT_TIMEOUT_US 500000
using namespace ChibiOS;
extern const AP_HAL::HAL& hal;
void RCInput::init()
{
#ifndef HAL_BUILD_AP_PERIPH
    AP::RC().init();
#endif

#if HAL_USE_ICU == TRUE
    //attach timer channel on which the signal will be received
    sig_reader.attach_capture_timer(&RCIN_ICU_TIMER, RCIN_ICU_CHANNEL, STM32_RCIN_DMA_STREAM, STM32_RCIN_DMA_CHANNEL);
    pulse_input_enabled = true;
#endif

#if HAL_USE_EICU == TRUE
    sig_reader.init(&RCININT_EICU_TIMER, RCININT_EICU_CHANNEL);
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
#if HAL_USE_ICU == TRUE || HAL_USE_EICU == TRUE
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

#if HAL_RCINPUT_WITH_AP_RADIO
    if (!_radio_init) {
        _radio_init = true;
        radio = AP_Radio::get_singleton();
        if (radio) {
            radio->init();
        }
    }
#endif
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
#if HAL_RCINPUT_WITH_AP_RADIO
    if (radio && channel == 0) {
        // hook to allow for update of radio on main thread, for mavlink sends
        radio->update();
    }
#endif
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
#if HAL_RCINPUT_WITH_AP_RADIO
    if (radio) {
        // hook to allow for update of radio on main thread, for mavlink sends
        radio->update();
    }
#endif
    return len;
}

void RCInput::_timer_tick(void)
{
    if (!_init) {
        return;
    }
#ifndef HAL_NO_UARTDRIVER
    const char *rc_protocol = nullptr;
    RCSource source = last_source;
#endif

#ifndef HAL_BUILD_AP_PERIPH
    AP_RCProtocol &rcprot = AP::RC();

#if HAL_USE_ICU == TRUE
    if (pulse_input_enabled) {
        const uint32_t *p;
        uint32_t n;
        while ((p = (const uint32_t *)sig_reader.sigbuf.readptr(n)) != nullptr) {
            rcprot.process_pulse_list(p, n*2, sig_reader.need_swap);
            sig_reader.sigbuf.advance(n);
        }
    }
#endif

#if HAL_USE_EICU == TRUE
    if (pulse_input_enabled) {
        uint32_t width_s0, width_s1;
        while(sig_reader.read(width_s0, width_s1)) {
            rcprot.process_pulse(width_s0, width_s1);
        }
    }
#endif

#if HAL_WITH_IO_MCU
    uint32_t now = AP_HAL::millis();
    const bool have_iocmu_rc = (_rcin_last_iomcu_ms != 0 && now - _rcin_last_iomcu_ms < 400);
    if (!have_iocmu_rc) {
        _rcin_last_iomcu_ms = 0;
    }
#else
    const bool have_iocmu_rc = false;
#endif

    if (rcprot.new_input() && !have_iocmu_rc) {
        WITH_SEMAPHORE(rcin_mutex);
        _rcin_timestamp_last_signal = AP_HAL::micros();
        _num_channels = rcprot.num_channels();
        _num_channels = MIN(_num_channels, RC_INPUT_MAX_CHANNELS);
        rcprot.read(_rc_values, _num_channels);
        _rssi = rcprot.get_RSSI();
        _rx_link_quality = rcprot.get_rx_link_quality();
#ifndef HAL_NO_UARTDRIVER
        rc_protocol = rcprot.protocol_name();
        source = rcprot.using_uart() ? RCSource::RCPROT_BYTES : RCSource::RCPROT_PULSES;
#endif
    }
#endif // HAL_BUILD_AP_PERIPH

#if HAL_RCINPUT_WITH_AP_RADIO
    if (radio && radio->last_recv_us() != last_radio_us && !have_iocmu_rc) {
        last_radio_us = radio->last_recv_us();
        WITH_SEMAPHORE(rcin_mutex);
        _rcin_timestamp_last_signal = last_radio_us;
        _num_channels = radio->num_channels();
        _num_channels = MIN(_num_channels, RC_INPUT_MAX_CHANNELS);
        for (uint8_t i=0; i<_num_channels; i++) {
            _rc_values[i] = radio->read(i);
        }
#ifndef HAL_NO_UARTDRIVER
        source = RCSource::APRADIO;
#endif
    }
#endif

#if HAL_WITH_IO_MCU
    {
        WITH_SEMAPHORE(rcin_mutex);
        if (AP_BoardConfig::io_enabled() &&
            iomcu.check_rcinput(last_iomcu_us, _num_channels, _rc_values, RC_INPUT_MAX_CHANNELS)) {
            _rcin_timestamp_last_signal = last_iomcu_us;
            _rcin_last_iomcu_ms = now;
#ifndef HAL_NO_UARTDRIVER
            rc_protocol = iomcu.get_rc_protocol();
            _rssi = iomcu.get_RSSI();
            source = RCSource::IOMCU;
#endif
        }
    }
#endif

#ifndef HAL_NO_UARTDRIVER
    if (rc_protocol && (rc_protocol != last_protocol || source != last_source)) {
        last_protocol = rc_protocol;
        last_source = source;
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "RCInput: decoding %s(%u)", last_protocol, unsigned(source));
    }
#endif

    // note, we rely on the vehicle code checking new_input()
    // and a timeout for the last valid input to handle failsafe
}

/*
  start a bind operation, if supported
 */
bool RCInput::rc_bind(int dsmMode)
{
#if HAL_WITH_IO_MCU
    {
        WITH_SEMAPHORE(rcin_mutex);
        if (AP_BoardConfig::io_enabled()) {
            iomcu.bind_dsm(dsmMode);
        }
    }
#endif

#ifndef HAL_BUILD_AP_PERIPH
    // ask AP_RCProtocol to start a bind
    AP::RC().start_bind();
#endif

#if HAL_RCINPUT_WITH_AP_RADIO
    if (radio) {
        radio->start_recv_bind();
    }
#endif
    return true;
}
#endif //#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
