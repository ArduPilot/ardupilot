/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Initial protocol implementation by Amilcar Lucas, IAV GmbH */

#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_TMotorESC.h"
#if AP_TMOTOR_ESC_ENABLED

extern const AP_HAL::HAL& hal;

// Set to 0 when no ESC hardware is available and you want to test the UART send function
#ifndef HAL_AP_TMOTOR_CONFIGURE_ESCS
#define HAL_AP_TMOTOR_CONFIGURE_ESCS 1
#endif

#define HAL_AP_TMOTOR_HALF_DUPLEX 0

#if HAL_AP_TMOTOR_HALF_DUPLEX
static constexpr uint32_t HALF_DUPLEX_BAUDRATE = 2000000;
#endif
static constexpr uint32_t FULL_DUPLEX_BAUDRATE =  500000;

const AP_Param::GroupInfo AP_TMotorESC::var_info[] {

#if HAL_WITH_ESC_TELEM
    // @Param: POLES
    // @DisplayName: Nr. electrical poles
    // @Description: Number of motor electrical poles
    // @Range: 2 50
    // @User: Standard
    AP_GROUPINFO("POLES", 3, AP_TMotorESC, _pole_count_parameter, 14),
#endif

    AP_GROUPEND
};

AP_TMotorESC *AP_TMotorESC::_singleton;

AP_TMotorESC::AP_TMotorESC()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_TMotorESC must be singleton");
    }
#endif
    _singleton = this;
}

/**
  initialize the serial port

*/
void AP_TMotorESC::init_uart()
{
    if (_uart != nullptr) {
        return;
    }
    const AP_SerialManager& serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_TMotorESC, 0);
    if (_uart == nullptr) {
        return; // no serial port available, so nothing to do here
    }
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_unbuffered_writes(true);
    _uart->set_blocking_writes(false);

    uint32_t uart_baud { FULL_DUPLEX_BAUDRATE };
#if HAL_AP_TMOTOR_HALF_DUPLEX
    if (_uart->get_options() & _uart->OPTION_HDPLEX) { //Half-Duplex is enabled
        _use_hdplex = true;
        uart_baud = HALF_DUPLEX_BAUDRATE;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TME using Half-Duplex");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TME using Full-Duplex");
    }
#endif

    _uart->begin(uart_baud);
}

/// initialize the device driver: configure serial port, wake-up and configure ESCs
void AP_TMotorESC::init()
{
    init_uart();
    if (_uart == nullptr) {
        return; // no serial port available, so nothing to do here
    }

    // we have a uart and the desired ESC combination id valid, allocate some memory:
    _escs = new ESC[_esc_count];
    if (_escs == nullptr) {
        return;
    }

    // initialise ESC ids.  This enforces that the TMotor ESC ids
    // inside TMotor ESCs need to be contiguous and start at ID 1
    uint8_t esc_offset = 0;  // offset into our device-driver dynamically-allocated array of ESCs
    uint8_t esc_id = 1;      // ESC ids inside TMotor protocol are one-indexed
    uint8_t servo_chan_offset = 0;  // offset into _motor_mask_parameter array
    for (uint32_t mask = 0xff; mask != 0; mask >>= 1, servo_chan_offset++) {
        if (mask & 0x1) {
            _escs[esc_offset].servo_ofs = servo_chan_offset;
            _escs[esc_offset].id = esc_id++;
            esc_offset++;
        }
    }

    gcs().send_text(MAV_SEVERITY_INFO, "TMotor: allocated %u motors", _esc_count);

#if HAL_AP_TMOTOR_HALF_DUPLEX
    if (_use_hdplex == true) { //Half-Duplex is enabled
        uart_baud = HALF_DUPLEX_BAUDRATE;
    }
#endif

    _init_done = true;
}

/**
    transmits data to ESCs
    @param bytes  bytes to transmit
    @param length number of bytes to transmit
    @return false there's no space in the UART for this message
*/
bool AP_TMotorESC::transmit(const uint8_t* bytes, const uint8_t length)
{
    const uint32_t now = AP_HAL::micros();
    if (now - _last_transmit_us < 400) {
        // in case the SRV_Channels::push() is running at very high rates, limit the period
        // this function gets executed because TMotor needs a time gap between frames
        // this also prevents one loop to do multiple actions, like reinitialize an ESC and sending a fast-throttle command without a gap.
        _period_too_short++;
        return false;
    }
    _last_transmit_us = now;
    if (length > _uart->txspace()) {
        return false;
    }

    _uart->write(bytes, length);
#if HAL_AP_TMOTOR_HALF_DUPLEX
    if (_use_hdplex) {
        _ignore_own_bytes += length;
    }
#endif
    return true;
}

/**
    transmits a config request to ESCs
    @param bytes  bytes to transmit
    @param length number of bytes to transmit
    @return false if vehicle is armed or if transmit(bytes, length) would return false
*/
bool AP_TMotorESC::transmit_config_request(const uint8_t* bytes, const uint8_t length)
{
    if (hal.util->get_soft_armed()) {
        return false;
    }
    return transmit(bytes, length);
}

/// shifts data to start of buffer based on magic header bytes
void AP_TMotorESC::move_frame_source_in_receive_buffer(const uint8_t search_start_pos)
{
    uint8_t i;
    for (i=search_start_pos; i<_receive_buf_used; i++) {
        // FIXME: full-duplex should add MASTER here as we see our own data
        if ((FrameSource)u.receive_buf[i] == FrameSource::BOOTLOADER ||
            (FrameSource)u.receive_buf[i] == FrameSource::ESC) {
            break;
        }
    }
    consume_bytes(i);
}

/// cut n bytes from start of buffer
void AP_TMotorESC::consume_bytes(const uint8_t n)
{
    if (n == 0) {
        return;
    }
    // assure the length of the memmove is positive
    if (_receive_buf_used < n) {
        return;
    }
    memmove(u.receive_buf, &u.receive_buf[n], _receive_buf_used-n);
    _receive_buf_used = _receive_buf_used - n;
}

/// returns true if the first message in the buffer is OK
bool AP_TMotorESC::buffer_contains_ok(const uint8_t length)
{
    if (length != sizeof(u.packed_ok)) {
        _message_invalid_in_state_count++;
        return false;
    }
    if ((MsgType)u.packed_ok.msg.msgid != MsgType::OK) {
        return false;
    }
    return true;
}

void AP_TMotorESC::handle_message(ESC &esc, const uint8_t length)
{
    // only accept messages from the bootloader when we could
    // legitimately get a message from the bootloader.  Swipes the OK
    // message for convenience
    const FrameSource frame_source = (FrameSource)u.packed_ok.frame_source;
    if (frame_source != FrameSource::ESC) {
        if (esc.state != ESCState::WAITING_OK_FOR_RUNNING_SW_TYPE) {
            return;
        }
    }

    switch (esc.state) {
    case ESCState::UNINITIALISED:
    case ESCState::WANT_SEND_OK_TO_GET_RUNNING_SW_TYPE:
        return;
    case ESCState::WAITING_OK_FOR_RUNNING_SW_TYPE:
        // "OK" is the only valid response
        if (!buffer_contains_ok(length)) {
            return;
        }
        switch (frame_source) {
        case FrameSource::MASTER:
            // probably half-duplex; should be caught before we get here
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        case FrameSource::BOOTLOADER:
            esc.set_state(ESCState::WANT_SEND_START_FW);
            esc.is_awake = true;
            break;
        case FrameSource::ESC:
#if HAL_WITH_ESC_TELEM
            esc.set_state(ESCState::WANT_SEND_SET_TLM_TYPE);
#else
            esc.set_state(ESCState::WANT_SEND_SET_FAST_COM_LENGTH);
#endif
            esc.is_awake = true;
            break;
        }
        break;

    case ESCState::WANT_SEND_START_FW:
        return;
    case ESCState::WAITING_OK_FOR_START_FW:
        if (buffer_contains_ok(length)) {
#if HAL_WITH_ESC_TELEM
            esc.set_state(ESCState::WANT_SEND_SET_TLM_TYPE);
#else
            esc.set_state(ESCState::RUNNING);
#endif
        }
        break;

#if HAL_WITH_ESC_TELEM
    case ESCState::WANT_SEND_SET_TLM_TYPE:
        return;
    case ESCState::WAITING_SET_TLM_TYPE_OK:
        if (buffer_contains_ok(length)) {
            esc.set_state(ESCState::RUNNING);
        }
        break;
#endif

    case ESCState::RUNNING:
        // we only expect telemetry messages in this state
#if HAL_WITH_ESC_TELEM
        if (!esc.telem_expected) {
            esc.unexpected_telem++;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("unexpected telemetry");
#endif
            return;
        }
        esc.telem_expected = false;
        return handle_message_telem(esc);
#else
        return;
#endif  // HAL_WITH_ESC_TELEM

    }
}

#if HAL_WITH_ESC_TELEM
void AP_TMotorESC::handle_message_telem(ESC &esc)
{
    // the following two methods are coming from AP_ESC_Telem:
    const TLM &tlm = u.packed_tlm.msg;

    // update rpm and error rate
    float error_rate_pct = 0;
    update_rpm(esc.servo_ofs,
               tlm.rpm*(100*2/_pole_count_parameter),
               error_rate_pct);

    // update power and temperature telem data
    TelemetryData t {};
    t.temperature_cdeg = tlm.temp * 100;
    t.voltage = tlm.voltage * 0.01f;
    t.current = tlm.current * 0.01f;
    t.consumption_mah = tlm.consumption_mah;
    update_telem_data(
        esc.servo_ofs,
        t,
        TelemetryType::TEMPERATURE|
          TelemetryType::VOLTAGE|
          TelemetryType::CURRENT|
          TelemetryType::CONSUMPTION);

    esc.last_telem_us = AP_HAL::micros();
}
#endif  // HAL_WITH_ESC_TELEM

// reads data from the UART, calling handle_message on any message found
void AP_TMotorESC::read_data_from_uart()
{
    /*
    a frame looks like:
    byte 1 = frame header (0x02 = bootloader, 0x03 = ESC firmware)
    byte 2 = sender ID (5bit)
    byte 3 & 4 = frame type (always 0x00, 0x00 used for bootloader. here just for compatibility)
    byte 5 = frame length over all bytes
    byte 6 - X = answer type, followed by the payload
    byte X+1 = 8bit CRC
    */

#if HAL_AP_TMOTOR_HALF_DUPLEX
    //ignore own bytes
    if (_use_hdplex) {
        while (_ignore_own_bytes > 0 && _uart->available()) {
            _ignore_own_bytes--;
            _uart->read();
        }
    }
#endif

    uint32_t bytes_to_read = MIN(_uart->available(), 128U);
    uint32_t last_bytes_to_read = 0;
    while (bytes_to_read &&
           bytes_to_read != last_bytes_to_read) {
        last_bytes_to_read = bytes_to_read;

        // read as much from the uart as we can:
        const uint8_t space = ARRAY_SIZE(u.receive_buf) - _receive_buf_used;
        const uint32_t nbytes = _uart->read(&u.receive_buf[_receive_buf_used], space);
        _receive_buf_used += nbytes;
        bytes_to_read -= nbytes;

        move_frame_source_in_receive_buffer();

        // borrow the "OK" message to retrieve the frame length from the buffer:
        const uint8_t frame_length = u.packed_ok.frame_length;
        if (_receive_buf_used < frame_length) {
            continue;
        }

        if (crc8_dvb_update(0, u.receive_buf, frame_length-1) != u.receive_buf[frame_length-1]) {
            // bad message; shift away this frame_source byte to try to find
            // another message
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("bad message");
#endif
            crc_rec_err_cnt++;
            move_frame_source_in_receive_buffer(1);
            continue;
        }

        // borrow the "OK" message to retrieve the frame_source from the buffer:
        const FrameSource frame_source = (FrameSource)u.packed_ok.frame_source;
        if (frame_source == FrameSource::MASTER) {
            // this is our own message - we'd best be running in
            // half-duplex or we're in trouble!
            consume_bytes(frame_length);
            continue;
        }

        // borrow the "OK" message to retrieve the esc id from the buffer:
        const uint8_t esc_id = u.packed_ok.esc_id;
        bool handled = false;
        // FIXME: we could scribble down the last ESC we sent a
        // message to here and use it rather than doing this linear
        // search:
        for (uint8_t i=0; i<_esc_count; i++) {
            auto &esc = _escs[i];
            if (esc.id != esc_id) {
                continue;
            }
            handle_message(esc, frame_length);
            handled = true;
            break;
        }
        if (!handled) {
            _unknown_esc_message++;
        }

        consume_bytes(frame_length);
    }
}

bool AP_TMotorESC::pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const
{
    if (_uart == nullptr) {
        hal.util->snprintf(failure_msg, failure_msg_len, "No uart");
        return false;
    }
#if HAL_WITH_ESC_TELEM
    if (_pole_count_parameter < 2) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Invalid pole count %u", uint8_t(_pole_count_parameter));
        return false;
    }
    uint8_t no_telem = 0;
    const uint32_t now = AP_HAL::micros();
#endif

    uint8_t not_running = 0;
    for (uint8_t i=0; i<_esc_count; i++) {
        auto &esc = _escs[i];
        if (esc.state != ESCState::RUNNING) {
            not_running++;
            continue;
        }
#if HAL_WITH_ESC_TELEM
        if (now - esc.last_telem_us > max_telem_interval_us) {
            no_telem++;
        }
#endif
    }
    if (not_running != 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%u of %u ESCs are not running", not_running, _esc_count);
        return false;
    }
    if (!_init_done) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Not initialised");
        return false;
    }
#if HAL_WITH_ESC_TELEM
    if (no_telem != 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%u of %u ESCs are not sending telemetry", no_telem, _esc_count);
        return false;
    }
#endif

    return true;
}

/// periodically called from SRV_Channels::push()
void AP_TMotorESC::update()
{
    if (!_init_done) {
        init();
        return; // the rest of this function can only run after fully initted
    }

    // read all data from incoming serial:
    read_data_from_uart();

    const uint32_t now = AP_HAL::micros();

#if HAL_WITH_ESC_TELEM
    if (!hal.util->get_soft_armed()) {

        // if we haven't seen an ESC in a while, the user might
        // have power-cycled them.  Try re-initialising.
        for (uint8_t i=0; i<_esc_count; i++) {
            auto &esc = _escs[i];
            if (!esc.telem_requested || now - esc.last_telem_us < 1000000U ) {
                // telem OK
                continue;
            }
            _running_mask &= ~(1 << esc.servo_ofs);
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "No telem from esc id=%u. Resetting it.", esc.id);
            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "unknown %u, invalid %u, too short %u, unexpected: %u, crc_err %u", _unknown_esc_message, _message_invalid_in_state_count, _period_too_short, esc.unexpected_telem, crc_rec_err_cnt);
            esc.set_state(ESCState::WANT_SEND_OK_TO_GET_RUNNING_SW_TYPE);
            esc.telem_requested = false;
        }
    }
#endif  // HAL_WITH_ESC_TELEM

    if (now - _last_transmit_us < 700U) {
        // in case the SRV_Channels::push() is running at very high rates, limit the period
        // this function gets executed because TMotor needs a time gap between frames
        _period_too_short++;
        return;
    }

}

#endif  // AP_TMOTOR_ESC_ENABLED
