#include "AP_Mount_config.h"

#if HAL_MOUNT_TOPOTEK_ENABLED || HAL_MOUNT_SKYDROID_ENABLED

#include "AP_Mount_Backend_TPFrame.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_TPFRAME_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_TPFRAME_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mount: " fmt, ## args); } } while (0)

// reading incoming packets from gimbal and confirm they are of the correct format
void AP_Mount_Backend_TPFrame::read_incoming_packets()
{
    // check for bytes on the serial port
    const uint16_t nbytes = MIN(_uart->available(), 1024U);
    if (nbytes == 0) {
        return;
    }

    // flag to allow cases below to reset parser state
    bool reset_parser = false;

    // process bytes received
    for (uint16_t i = 0; i < nbytes; i++) {
        uint8_t b;
        if (!_uart->read(b)) {
            continue;
        }

        // add latest byte to buffer
        _msg_buff[_msg_buff_len++] = b;

        // protect against overly long messages
        if (_msg_buff_len >= packetlen_max()) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parser.state) {

        case ParseState::WAITING_FOR_HEADER1:
            if (b == '#') {
                _parser.state = ParseState::WAITING_FOR_HEADER2;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_HEADER2:
            // 't'/'T' (and 'p'/'P' below) distinguish HeaderType::VARIABLE_LEN from
            // FIXED_LEN - meaningful on transmit (see send_variablelen_packet()), but
            // deliberately not tracked here on receive: the packet's own Data_Len
            // nibble is authoritative regardless of which header case was used to
            // send it, so the parser has no need to remember which one it saw
            if (b == 't' || b == 'T') {
                _parser.state = ParseState::WAITING_FOR_HEADER3;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_HEADER3:
            if (b == 'p' || b == 'P') {
                _parser.state = ParseState::WAITING_FOR_ADDR1;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_ADDR1:
        case ParseState::WAITING_FOR_ADDR2:
            if (is_valid_address_byte(b)) {
                // advance to next state
                _parser.state = (ParseState)((uint8_t)_parser.state+1);
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_DATALEN: {
            // sanity check data length
            uint8_t data_len;
            if (hex_char_to_nibble(b, data_len) && data_len <= datalen_max()) {
                _parser.data_len = data_len;
                _parser.state = ParseState::WAITING_FOR_CONTROL;
                break;
            }
            reset_parser = true;
            break;
        }

        case ParseState::WAITING_FOR_CONTROL:
            // r or w
            if (b == 'r' || b == 'w') {
                _parser.state = ParseState::WAITING_FOR_ID1;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_ID1:
        case ParseState::WAITING_FOR_ID2:
            // check all uppercase letters and numbers.  eg 'GAC'
            if ((b >= 'A' && b <= 'Z') || (b >= '0' && b <= '9')) {
                // advance to next state
                _parser.state = (ParseState)((uint8_t)_parser.state+1);
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_ID3:
            if ((b >= 'A' && b <= 'Z') || (b >= '0' && b <= '9')) {
                // a zero-length data segment has no data bytes to wait for - the
                // WAITING_FOR_DATA case below can only advance once
                // data_bytes_received (which starts at 1 on the very next byte and
                // only increases) equals _parser.data_len, which can never happen
                // for data_len==0, so go straight to the CRC instead of getting
                // stuck consuming the CRC (and then the next packet) as fake data
                // until datalen_max() overflows and fires an avoidable INTERNAL_ERROR
                _parser.state = (_parser.data_len == 0) ? ParseState::WAITING_FOR_CRC_LOW : ParseState::WAITING_FOR_DATA;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_DATA: {
            // normally hex numbers in char form (e.g. '0A')
            const uint8_t data_bytes_received = _msg_buff_len - (AP_MOUNT_TPFRAME_PACKETLEN_MIN - 2);

            // sanity check to protect against programming errors
            if (data_bytes_received > datalen_max()) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                reset_parser = true;
                break;
            }

            // advance parser state once expected number of bytes have been received
            if (data_bytes_received == _parser.data_len) {
                _parser.state = ParseState::WAITING_FOR_CRC_LOW;
            }
            break;
        }

        case ParseState::WAITING_FOR_CRC_LOW:
            _parser.state = ParseState::WAITING_FOR_CRC_HIGH;
            break;

        case ParseState::WAITING_FOR_CRC_HIGH:
            // this is the last byte in the message so reset the parser
            reset_parser = true;

            // sanity check to protect against programming errors
            if (_msg_buff_len < AP_MOUNT_TPFRAME_PACKETLEN_MIN) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                break;
            }

            // calculate and check CRC
            const uint8_t crc_value = calculate_crc(_msg_buff, _msg_buff_len - 2);
            const char crc_char1 = hex2char((crc_value >> 4) & 0x0f);
            const char crc_char2 = hex2char((crc_value) & 0x0f);
            if (crc_char1 != _msg_buff[_msg_buff_len - 2] || crc_char2 != _msg_buff[_msg_buff_len-1]) {
                debug("CRC expected:%x got:%c%c", (int)crc_value, crc_char1, crc_char2);
                break;
            }

            // CRC is OK, dispatch on the 3-character command ID to the subclass
            handle_message((const char*)&_msg_buff[AP_MOUNT_TPFRAME_MSGOFS_ID]);
        }

        // handle reset of parser
        if (reset_parser) {
            _parser.state = ParseState::WAITING_FOR_HEADER1;
            _msg_buff_len = 0;
            reset_parser = false;
        }
    }
}

// calculate checksum
uint8_t AP_Mount_Backend_TPFrame::calculate_crc(const uint8_t *cmd, uint8_t len) const
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i<len; i++) {
        crc += cmd[i];
    }
    return(crc);
}

// hexadecimal to character conversion
uint8_t AP_Mount_Backend_TPFrame::hex2char(uint8_t data) const
{
    if ((9 >= data)) {
        return (data + '0');
    } else {
        return (data - 10 + 'A');
    }
}

// send a fixed length packet
bool AP_Mount_Backend_TPFrame::send_fixedlen_packet(uint8_t address, const Identifier id, bool write, uint8_t value)
{
    uint8_t databuff[3];
    hal.util->snprintf((char *)databuff, ARRAY_SIZE(databuff), "%02X", value);
    return send_variablelen_packet(HeaderType::FIXED_LEN, address, id, write, databuff, ARRAY_SIZE(databuff)-1);
}

// send variable length packet
bool AP_Mount_Backend_TPFrame::send_variablelen_packet(HeaderType header, uint8_t address, const Identifier id, bool write, const uint8_t* databuff, uint8_t databuff_len)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // calculate and sanity check packet size
    const uint16_t packet_size = AP_MOUNT_TPFRAME_PACKETLEN_MIN + databuff_len;
    if (packet_size > packetlen_max()) {
        debug("send_packet data buff too large");
        return false;
    }

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < packet_size) {
        debug("tx buffer full");
        return false;
    }

    // create buffer for holding outgoing packet
    uint8_t send_buff[packet_size];
    uint8_t send_buff_ofs = 0;

    // packet header (bytes 0 ~ 2)
    send_buff[send_buff_ofs++] = '#';
    send_buff[send_buff_ofs++] = (header == HeaderType::FIXED_LEN) ? 'T' : 't';
    send_buff[send_buff_ofs++] = (header == HeaderType::FIXED_LEN) ? 'P' : 'p';

    // address (bytes 3, 4)
    send_buff[send_buff_ofs++] = source_address_byte();
    send_buff[send_buff_ofs++] = address;

    // data length (byte 5)
    send_buff[send_buff_ofs++] = hex2char(databuff_len);

    // control byte (byte 6)
    send_buff[send_buff_ofs++] = write ? (uint8_t)ControlByte::WRITE : (uint8_t)ControlByte::READ;

    // identifier (bytes 7 ~ 9)
    send_buff[send_buff_ofs++] = id[0];
    send_buff[send_buff_ofs++] = id[1];
    send_buff[send_buff_ofs++] = id[2];

    // data
    if (databuff_len != 0) {
        memcpy(&send_buff[send_buff_ofs], databuff, databuff_len);
        send_buff_ofs += databuff_len;
    }

    // crc
    uint8_t crc = calculate_crc(send_buff, send_buff_ofs);
    send_buff[send_buff_ofs++] = hex2char((crc >> 4) & 0x0f);
    send_buff[send_buff_ofs++] = hex2char(crc & 0x0f);

    // send packet
    _uart->write(send_buff, send_buff_ofs);
    return true;
}

#endif // HAL_MOUNT_TOPOTEK_ENABLED || HAL_MOUNT_SKYDROID_ENABLED
