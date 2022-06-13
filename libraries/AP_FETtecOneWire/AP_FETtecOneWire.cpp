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

/* Initial protocol implementation was provided by FETtec */
/* Strongly modified by Amilcar Lucas, IAV GmbH */

#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_FETtecOneWire.h"
#if AP_FETTEC_ONEWIRE_ENABLED

extern const AP_HAL::HAL& hal;

// Set to 0 when no ESC hardware is available and you want to test the UART send function
#ifndef HAL_AP_FETTEC_CONFIGURE_ESCS
#define HAL_AP_FETTEC_CONFIGURE_ESCS 1
#endif

#if HAL_AP_FETTEC_HALF_DUPLEX
static constexpr uint32_t HALF_DUPLEX_BAUDRATE = 2000000;
#endif
static constexpr uint32_t FULL_DUPLEX_BAUDRATE =  500000;

const AP_Param::GroupInfo AP_FETtecOneWire::var_info[] {

    // @Param: MASK
    // @DisplayName: Servo channel output bitmask
    // @Description: Servo channel mask specifying FETtec ESC output.
    // @Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("MASK",  1, AP_FETtecOneWire, _motor_mask_parameter, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: RVMASK
    // @DisplayName: Servo channel reverse rotation bitmask
    // @Description: Servo channel mask to reverse rotation of FETtec ESC outputs.
    // @Bitmask: 0:SERVO1,1:SERVO2,2:SERVO3,3:SERVO4,4:SERVO5,5:SERVO6,6:SERVO7,7:SERVO8,8:SERVO9,9:SERVO10,10:SERVO11,11:SERVO12
    // @User: Standard
    AP_GROUPINFO("RVMASK",  2, AP_FETtecOneWire, _reverse_mask_parameter, 0),

#if HAL_WITH_ESC_TELEM
    // @Param: POLES
    // @DisplayName: Nr. electrical poles
    // @Description: Number of motor electrical poles
    // @Range: 2 50
    // @User: Standard
    AP_GROUPINFO("POLES", 3, AP_FETtecOneWire, _pole_count_parameter, 14),
#endif

    AP_GROUPEND
};

AP_FETtecOneWire *AP_FETtecOneWire::_singleton;

AP_FETtecOneWire::AP_FETtecOneWire()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_FETtecOneWire must be singleton");
    }
#endif
    _singleton = this;
}

/**
  initialize the serial port

*/
void AP_FETtecOneWire::init_uart()
{
    if (_uart != nullptr) {
        return;
    }
    const AP_SerialManager& serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FETtecOneWire, 0);
    if (_uart == nullptr) {
        return; // no serial port available, so nothing to do here
    }
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_unbuffered_writes(true);
    _uart->set_blocking_writes(false);

    uint32_t uart_baud { FULL_DUPLEX_BAUDRATE };
#if HAL_AP_FETTEC_HALF_DUPLEX
    if (_uart->get_options() & _uart->OPTION_HDPLEX) { //Half-Duplex is enabled
        _use_hdplex = true;
        uart_baud = HALF_DUPLEX_BAUDRATE;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FTW using Half-Duplex");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FTW using Full-Duplex");
    }
#endif

    _uart->begin(uart_baud);
}

/// initialize the device driver: configure serial port, wake-up and configure ESCs
void AP_FETtecOneWire::init()
{
    init_uart();
    if (_uart == nullptr) {
        return; // no serial port available, so nothing to do here
    }

    _motor_mask = uint32_t(_motor_mask_parameter); // take a copy that will not change after we leave this function
    _esc_count = __builtin_popcount(_motor_mask);
#if HAL_WITH_ESC_TELEM
    // OneWire supports telemetry in at most 15 ESCs, because of the 4 bit limitation
    // on the fast-throttle command.  But we are still limited to the
    // number of ESCs the telem library will collect data for.
    if (_esc_count == 0 || _motor_mask >= (1U << MIN(15, ESC_TELEM_MAX_ESCS))) {
#else
    // OneWire supports at most 24 ESCs without telemetry
    if (_esc_count == 0 || _motor_mask >= (1U << MIN(24, NUM_SERVO_CHANNELS))) {
#endif
        _invalid_mask = true;
        return;
    }

    // we have a uart and the desired ESC combination id valid, allocate some memory:
    _escs = new ESC[_esc_count];
    if (_escs == nullptr) {
        return;
    }

    // initialise ESC ids.  This enforces that the FETtec ESC ids
    // inside FETtec ESCs need to be contiguous and start at ID 1
    // which is required by fast-throttle commands.
    uint8_t esc_offset = 0;  // offset into our device-driver dynamically-allocated array of ESCs
    uint8_t esc_id = 1;      // ESC ids inside FETtec protocol are one-indexed
    uint8_t servo_chan_offset = 0;  // offset into _motor_mask_parameter array
    for (uint32_t mask = _motor_mask; mask != 0; mask >>= 1, servo_chan_offset++) {
        if (mask & 0x1) {
            _escs[esc_offset].servo_ofs = servo_chan_offset;
            _escs[esc_offset].id = esc_id++;
            esc_offset++;
        }
    }
    _invalid_mask = false;  // mask is good

    gcs().send_text(MAV_SEVERITY_INFO, "FETtec: allocated %u motors", _esc_count);

    // We expect to be able to send a fast-throttle command in each loop.
    // 8  bits - OneWire Header
    // 4  bits - telemetry request
    // 11 bits - throttle value per ESC
    // 8  bits - frame CRC
    const uint16_t net_bit_count = 8 + 4 + (_esc_count * 11) + 8;
    // 7  dummy for rounding up the division by 8
    const uint16_t fast_throttle_byte_count = (net_bit_count + 7)/8;
    uint16_t telemetry_byte_count { 0U };
#if HAL_WITH_ESC_TELEM
    // Telemetry is fetched from each loop in turn.
    telemetry_byte_count = sizeof(u.packed_tlm) + 1; // assume 9 pause bits between TX and RX
    _fast_throttle_byte_count = fast_throttle_byte_count;
#endif
    uint32_t uart_baud { FULL_DUPLEX_BAUDRATE };
#if HAL_AP_FETTEC_HALF_DUPLEX
    if (_use_hdplex == true) { //Half-Duplex is enabled
        uart_baud = HALF_DUPLEX_BAUDRATE;
    }
#endif
    _min_fast_throttle_period_us = (fast_throttle_byte_count + telemetry_byte_count) * 9 * 1000000 / uart_baud + 300; // 300us extra reserve

    // tell SRV_Channels about ESC capabilities
    // this is a bit soonish because we have not seen the ESCs on the bus yet,
    // but saves us having to use a state variable to ensure doing this latter just once
    SRV_Channels::set_digital_outputs(_motor_mask, 0);

    _init_done = true;
}

/**
    transmits data to ESCs
    @param bytes  bytes to transmit
    @param length number of bytes to transmit
    @return false there's no space in the UART for this message
*/
bool AP_FETtecOneWire::transmit(const uint8_t* bytes, const uint8_t length)
{
    const uint32_t now = AP_HAL::micros();
    if (now - _last_transmit_us < _min_fast_throttle_period_us) {
        // in case the SRV_Channels::push() is running at very high rates, limit the period
        // this function gets executed because FETtec needs a time gap between frames
        // this also prevents one loop to do multiple actions, like reinitialize an ESC and sending a fast-throttle command without a gap.
        _period_too_short++;
        return false;
    }
    _last_transmit_us = now;
    if (length > _uart->txspace()) {
        return false;
    }

    _uart->write(bytes, length);
#if HAL_AP_FETTEC_HALF_DUPLEX
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
bool AP_FETtecOneWire::transmit_config_request(const uint8_t* bytes, const uint8_t length)
{
    if (hal.util->get_soft_armed()) {
        return false;
    }
    return transmit(bytes, length);
}

/// shifts data to start of buffer based on magic header bytes
void AP_FETtecOneWire::move_frame_source_in_receive_buffer(const uint8_t search_start_pos)
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
void AP_FETtecOneWire::consume_bytes(const uint8_t n)
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
bool AP_FETtecOneWire::buffer_contains_ok(const uint8_t length)
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

void AP_FETtecOneWire::handle_message(ESC &esc, const uint8_t length)
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
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
            esc.set_state(ESCState::WANT_SEND_REQ_TYPE);
#else
#if HAL_WITH_ESC_TELEM
            esc.set_state(ESCState::WANT_SEND_SET_TLM_TYPE);
#else
            esc.set_state(ESCState::WANT_SEND_SET_FAST_COM_LENGTH);
#endif
#endif  // HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
            esc.is_awake = true;
            break;
        }
        break;

    case ESCState::WANT_SEND_START_FW:
        return;
    case ESCState::WAITING_OK_FOR_START_FW:
        if (buffer_contains_ok(length)) {
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
            esc.set_state(ESCState::WANT_SEND_REQ_TYPE);
#else
#if HAL_WITH_ESC_TELEM
            esc.set_state(ESCState::WANT_SEND_SET_TLM_TYPE);
#else
            esc.set_state(ESCState::WANT_SEND_SET_FAST_COM_LENGTH);
#endif
#endif  // HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
        }
        break;

#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
    case ESCState::WANT_SEND_REQ_TYPE:
        return;
    case ESCState::WAITING_ESC_TYPE:
        if (length != sizeof(u.packed_esc_type)) {
            _message_invalid_in_state_count++;
            return;
        }
        esc.type = u.packed_esc_type.msg.type;
        esc.set_state(ESCState::WANT_SEND_REQ_SW_VER);
        break;

    case ESCState::WANT_SEND_REQ_SW_VER:
        return;
    case ESCState::WAITING_SW_VER:
        if (length != sizeof(u.packed_sw_ver)) {
            _message_invalid_in_state_count++;
            return;
        }
        esc.firmware_version = u.packed_sw_ver.msg.version;
        esc.firmware_subversion = u.packed_sw_ver.msg.subversion;
        esc.set_state(ESCState::WANT_SEND_REQ_SN);
        break;

    case ESCState::WANT_SEND_REQ_SN:
        return;
    case ESCState::WAITING_SN:
        if (length != sizeof(u.packed_sn)) {
            _message_invalid_in_state_count++;
            return;
        }
        static_assert(ARRAY_SIZE(u.packed_sn.msg.sn) == ARRAY_SIZE(esc.serial_number), "Serial number array length missmatch");
        memcpy(esc.serial_number, u.packed_sn.msg.sn, ARRAY_SIZE(u.packed_sn.msg.sn));
#if HAL_WITH_ESC_TELEM
        esc.set_state(ESCState::WANT_SEND_SET_TLM_TYPE);
#else
        esc.set_state(ESCState::WANT_SEND_SET_FAST_COM_LENGTH);
#endif
        break;
#endif  // HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO

#if HAL_WITH_ESC_TELEM
    case ESCState::WANT_SEND_SET_TLM_TYPE:
        return;
    case ESCState::WAITING_SET_TLM_TYPE_OK:
        if (buffer_contains_ok(length)) {
            esc.set_state(ESCState::WANT_SEND_SET_FAST_COM_LENGTH);
        }
        break;
#endif

    case ESCState::WANT_SEND_SET_FAST_COM_LENGTH:
        return;
    case ESCState::WAITING_SET_FAST_COM_LENGTH_OK:
        if (buffer_contains_ok(length)) {
            esc.set_state(ESCState::RUNNING);
        }
        break;
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
void AP_FETtecOneWire::handle_message_telem(ESC &esc)
{
    // the following two methods are coming from AP_ESC_Telem:
    const TLM &tlm = u.packed_tlm.msg;

    // update rpm and error rate
    float error_rate_pct = 0;
    if (_fast_throttle_cmd_count > _esc_count) {
        error_rate_pct = (tlm.tx_err_count-esc.error_count_at_throttle_count_overflow)*(float)100/(float)_fast_throttle_cmd_count;
    } else {
        // the telemetry is requested in a round-robin, sequential fashion
        // so the in the first _esc_count times all ESCs get to initialize this
        esc.error_count_at_throttle_count_overflow = tlm.tx_err_count;
    }
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
void AP_FETtecOneWire::read_data_from_uart()
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

#if HAL_AP_FETTEC_HALF_DUPLEX
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

/**
    packs a single fast-throttle command frame containing the throttle for all configured OneWire ESCs.
    @param motor_values a 16bit array containing the throttle values that should be sent to the motors. 0-2000 where 1001-2000 is positive rotation and 0-999 reversed rotation
    @param esc_id_to_request_telem_from the ESC to request telemetry from
*/
void AP_FETtecOneWire::pack_fast_throttle_command(const uint16_t *motor_values, uint8_t *fast_throttle_command, const uint8_t length, const uint8_t esc_id_to_request_telem_from)
{
    // byte 1:
    // bit 0,1,2,3 = ESC ID, Bit 4 = MSB bit of first ESC (11bit) throttle value, bit 5,6,7 = frame header
    // so AAAABCCC
    // A = ID from the ESC telemetry is requested from. ESC ID == 0 means no request.
    // B = MSB from first throttle value
    // C = frame header
    fast_throttle_command[0] = esc_id_to_request_telem_from << 4; // 0 here means no telemetry request
    fast_throttle_command[0] |= ((motor_values[0] >> 10) & 0x01) << 3;
    fast_throttle_command[0] |= (uint8_t)FrameSource::MASTER;

    // byte 2:
    // AAABBBBB
    // A = next 3 bits from (11bit) throttle value
    // B = 5bit target ID
    fast_throttle_command[1] = (((motor_values[0] >> 7) & 0x07)) << 5;
    fast_throttle_command[1] |= 0x1F;      // All IDs

    // following bytes are the rest 7 bit of the first (11bit) throttle value,
    // and all bits from all other throttle values, followed by the CRC byte
    uint8_t mot = 0;
    uint8_t bits_remaining_in_this_pwm = 7;
    for (uint8_t out_byte_offset = 2; out_byte_offset<length; out_byte_offset++) {
        if (bits_remaining_in_this_pwm >= 8) {
            // const uint8_t mask = 0xFF << (11-bits_remaining_in_this_pwm);
            fast_throttle_command[out_byte_offset] = (motor_values[mot] >> (bits_remaining_in_this_pwm-8)) & 0xFF;
            bits_remaining_in_this_pwm -= 8;
        } else {
            const uint8_t mask = (1U<<bits_remaining_in_this_pwm)-1;
            const uint8_t bits_to_copy_from_second_pwm = 8-bits_remaining_in_this_pwm;
            fast_throttle_command[out_byte_offset] = (motor_values[mot] & mask) << bits_to_copy_from_second_pwm;
            // move on to the next motor output
            mot++;
            if (mot < _esc_count) {
                fast_throttle_command[out_byte_offset] |= motor_values[mot] >> (11-bits_to_copy_from_second_pwm);
            }
            bits_remaining_in_this_pwm = 11 - bits_to_copy_from_second_pwm;
        }
    }

    fast_throttle_command[length-1] =
        crc8_dvb_update(0, fast_throttle_command, length-1);
}

void AP_FETtecOneWire::escs_set_values(const uint16_t* motor_values)
{
    uint8_t esc_id_to_request_telem_from = 0;
#if HAL_WITH_ESC_TELEM
    ESC &esc_to_req_telem_from = _escs[_esc_ofs_to_request_telem_from++];
    if (_esc_ofs_to_request_telem_from >= _esc_count) {
        _esc_ofs_to_request_telem_from = 0;
    }
    esc_id_to_request_telem_from = esc_to_req_telem_from.id;
#endif

    uint8_t fast_throttle_command[_fast_throttle_byte_count];
    pack_fast_throttle_command(motor_values, fast_throttle_command, sizeof(fast_throttle_command), esc_id_to_request_telem_from);

#if HAL_AP_FETTEC_HALF_DUPLEX
    // last byte of signal can be used to make sure the first TLM byte is correct, in case of spike corruption
    // FIXME: use this somehow
    _last_crc = fast_throttle_command[_fast_throttle_byte_count - 1];
#endif

    // No command was yet sent, so no reply is expected and all information
    // on the receive buffer is either garbage or noise. Discard it
    _uart->discard_input();
    _receive_buf_used = 0;

    // send throttle commands to all configured ESCs in a single packet transfer
    if (transmit(fast_throttle_command, sizeof(fast_throttle_command))) {
#if HAL_WITH_ESC_TELEM
        esc_to_req_telem_from.telem_expected = true;    // used to make sure that the returned telemetry comes from this ESC and not another
        esc_to_req_telem_from.telem_requested = true;   // used to check if this ESC is periodically sending telemetry
        _fast_throttle_cmd_count++;
#endif
    }
}

bool AP_FETtecOneWire::pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const
{
    if (_motor_mask_parameter == 0) {
        return true;    // No FETtec ESCs are expected, no need to run further pre-arm checks
    }

    if (_uart == nullptr) {
        hal.util->snprintf(failure_msg, failure_msg_len, "No uart");
        return false;
    }
    if (_invalid_mask) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Invalid motor mask");
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

void AP_FETtecOneWire::configure_escs()
{
    if (_uart->available()) {
        // don't attempt to configure if we've unread data
        return;
    }

    // note that we return as soon as we've transmitted anything in
    // case we're in one-wire mode
    for (uint8_t i=0; i<_esc_count; i++) {
        auto &esc = _escs[i];
        switch (esc.state) {
        case ESCState::UNINITIALISED:
            esc.state = ESCState::WANT_SEND_OK_TO_GET_RUNNING_SW_TYPE;
            FALLTHROUGH;
        case ESCState::WANT_SEND_OK_TO_GET_RUNNING_SW_TYPE:
            // probe for bootloader or running firmware
            if (transmit_config_request(PackedMessage<OK>{esc.id, OK{}})) {
                esc.is_awake = false; // Assume the ESC is asleep or unavailable
                esc.set_state(ESCState::WAITING_OK_FOR_RUNNING_SW_TYPE);
            }
            return;
        case ESCState::WAITING_OK_FOR_RUNNING_SW_TYPE:
            if (!esc.is_awake) {
                esc.set_state(ESCState::WANT_SEND_OK_TO_GET_RUNNING_SW_TYPE); // go back to try to wake up the ESC
            }
            return;
        case ESCState::WANT_SEND_START_FW:
            if (transmit_config_request(PackedMessage<START_FW>{esc.id, START_FW{}})) {
                esc.set_state(ESCState::WAITING_OK_FOR_START_FW);
            }
            return;
        case ESCState::WAITING_OK_FOR_START_FW:
            return;
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
        case ESCState::WANT_SEND_REQ_TYPE:
            if (transmit_config_request(PackedMessage<REQ_TYPE>{esc.id, REQ_TYPE{}})) {
                esc.set_state(ESCState::WAITING_ESC_TYPE);
            }
            return;
        case ESCState::WAITING_ESC_TYPE:
            return;
        case ESCState::WANT_SEND_REQ_SW_VER:
            if (transmit_config_request(PackedMessage<REQ_SW_VER>{esc.id, REQ_SW_VER{}})) {
                esc.set_state(ESCState::WAITING_SW_VER);
            }
            return;
        case ESCState::WAITING_SW_VER:
            return;
        case ESCState::WANT_SEND_REQ_SN:
            if (transmit_config_request(PackedMessage<REQ_SN>{esc.id, REQ_SN{}})) {
                esc.set_state(ESCState::WAITING_SN);
            }
            return;
        case ESCState::WAITING_SN:
            return;
#endif  // HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
#if HAL_WITH_ESC_TELEM
        case ESCState::WANT_SEND_SET_TLM_TYPE:
            if (transmit_config_request(PackedMessage<SET_TLM_TYPE>{esc.id, SET_TLM_TYPE{1}})) {
                esc.set_state(ESCState::WAITING_SET_TLM_TYPE_OK);
            }
            return;
        case ESCState::WAITING_SET_TLM_TYPE_OK:
            return;
#endif
        case ESCState::WANT_SEND_SET_FAST_COM_LENGTH:
            if (transmit_config_request(PackedMessage<SET_FAST_COM_LENGTH>{esc.id,
                            SET_FAST_COM_LENGTH{
                            _fast_throttle_byte_count,
                                _escs[0].id,
                                _esc_count
                                }})) {
                esc.set_state(ESCState::WAITING_SET_FAST_COM_LENGTH_OK);
            }
            return;
        case ESCState::WAITING_SET_FAST_COM_LENGTH_OK:
            return;
        case ESCState::RUNNING:
            _running_mask |= (1U << esc.servo_ofs);
            break;
        }
    }
}

/// periodically called from SRV_Channels::push()
void AP_FETtecOneWire::update()
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

        _reverse_mask = _reverse_mask_parameter; // update this only when disarmed

        // if we haven't seen an ESC in a while, the user might
        // have power-cycled them.  Try re-initialising.
        for (uint8_t i=0; i<_esc_count; i++) {
            auto &esc = _escs[i];
            if (!esc.telem_requested || now - esc.last_telem_us < 1000000U ) {
                // telem OK
                continue;
            }
            _running_mask &= ~(1U << esc.servo_ofs);
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "No telem from esc id=%u. Resetting it.", esc.id);
            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "unknown %u, invalid %u, too short %u, unexpected: %u, crc_err %u", _unknown_esc_message, _message_invalid_in_state_count, _period_too_short, esc.unexpected_telem, crc_rec_err_cnt);
            esc.set_state(ESCState::WANT_SEND_OK_TO_GET_RUNNING_SW_TYPE);
            esc.telem_requested = false;
        }
    }
#endif  // HAL_WITH_ESC_TELEM

    if (now - _last_transmit_us < 700U) {
        // in case the SRV_Channels::push() is running at very high rates, limit the period
        // this function gets executed because FETtec needs a time gap between frames
        _period_too_short++;
        return;
    }

#if defined(STM32F4)
    if (_uart->tx_pending()) {
        // there is unsent data in the send buffer,
        // do not send more data because FETtec needs a time gap between frames
        _period_too_short++;
        return;
    }
#endif

#if HAL_AP_FETTEC_CONFIGURE_ESCS
    // run ESC configuration state machines if needed
    if (_running_mask != _motor_mask) {
        configure_escs();
        return; // do not send fast-throttle command if a configuration command just got sent
    }
#endif

    // get ESC set points
    uint16_t motor_pwm[_esc_count];
    for (uint8_t i = 0; i < _esc_count; i++) {
        const ESC &esc = _escs[i];
        const SRV_Channel* c = SRV_Channels::srv_channel(esc.servo_ofs);
        // check if safety switch has been pushed
        if (   (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED)
            || (c == nullptr)) {  // this should never ever happen, but just in case ...
            motor_pwm[i] = 1000;  // stop motor
            continue;
        }
        motor_pwm[i] = constrain_int16(c->get_output_pwm(), 1000, 2000);
        fet_debug("esc=%u in: %u", esc.id, motor_pwm[i]);
        if (_reverse_mask & (1U << i)) {
            motor_pwm[i] = 2000-motor_pwm[i];
        }
    }

    // send motor setpoints to ESCs, and request for telemetry data
    escs_set_values(motor_pwm);
}

#if HAL_AP_FETTEC_ESC_BEEP
/**
    makes all connected ESCs beep
    @param beep_frequency a 8 bit value from 0-255. higher make a higher beep
*/
void AP_FETtecOneWire::beep(const uint8_t beep_frequency)
{
    for (uint8_t i=0; i<_esc_count; i++) {
        auto &esc = _escs[i];
        if (esc.state != ESCState::RUNNING) {
            continue;
        }
        transmit_config_request(PackedMessage<Beep>{esc.id, Beep{beep_frequency}});
    }
}
#endif  // HAL_AP_FETTEC_ESC_BEEP

#if HAL_AP_FETTEC_ESC_LIGHT
/**
    sets the racewire color for all ESCs
    @param r red brightness
    @param g green brightness
    @param b blue brightness
*/
void AP_FETtecOneWire::led_color(const uint8_t r, const uint8_t g, const uint8_t b)
{
    for (uint8_t i=0; i<_esc_count; i++) {
        auto &esc = _escs[i];
        if (esc.state != ESCState::RUNNING) {
            continue;
        }
        transmit_config_request(PackedMessage<LEDColour>{esc.id, LEDColour{r, g, b}});
    }
}
#endif  // HAL_AP_FETTEC_ESC_LIGHT

#endif  // AP_FETTEC_ONEWIRE_ENABLED
