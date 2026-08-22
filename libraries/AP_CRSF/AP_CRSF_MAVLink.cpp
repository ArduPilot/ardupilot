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

#include "AP_CRSF_MAVLink.h"

#if AP_CRSF_MAVLINK_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

// ----- VirtualUARTDriver -----

AP_CRSF_MAVLink::VirtualUARTDriver::VirtualUARTDriver() :
    _rx_buf(_rx_backing, BUF_SIZE),
    _tx_buf(_tx_backing, BUF_SIZE),
    _initialized(false)
{
}

void AP_CRSF_MAVLink::VirtualUARTDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    _initialized = true;
}

size_t AP_CRSF_MAVLink::VirtualUARTDriver::_write(const uint8_t *buffer, size_t size)
{
    return _tx_buf.write(buffer, size);
}

ssize_t AP_CRSF_MAVLink::VirtualUARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    return _rx_buf.read(buffer, count);
}

void AP_CRSF_MAVLink::VirtualUARTDriver::_end()
{
    _rx_buf.clear();
    _tx_buf.clear();
    _initialized = false;
}

uint32_t AP_CRSF_MAVLink::VirtualUARTDriver::_available()
{
    return _rx_buf.available();
}

bool AP_CRSF_MAVLink::VirtualUARTDriver::_discard_input()
{
    _rx_buf.clear();
    return true;
}

uint32_t AP_CRSF_MAVLink::VirtualUARTDriver::inject_rx(const uint8_t *data, uint32_t len)
{
    return _rx_buf.write(data, len);
}

uint32_t AP_CRSF_MAVLink::VirtualUARTDriver::drain_tx(uint8_t *data, uint32_t len)
{
    return _tx_buf.read(data, len);
}

uint32_t AP_CRSF_MAVLink::VirtualUARTDriver::peek_tx(uint8_t *data, uint32_t len)
{
    return _tx_buf.peekbytes(data, len);
}

uint32_t AP_CRSF_MAVLink::VirtualUARTDriver::tx_available() const
{
    return _tx_buf.available();
}

// ----- Reassembly / TxState helpers -----

void AP_CRSF_MAVLink::Reassembly::reset()
{
    write_offset = 0;
    expected_total = 0;
    next_chunk = 0;
    start_time_ms = 0;
    active = false;
}

void AP_CRSF_MAVLink::TxState::reset()
{
    msg_len = 0;
    total_chunks = 0;
    next_chunk = 0;
    active = false;
}

// ----- AP_CRSF_MAVLink -----

bool AP_CRSF_MAVLink::init()
{
    if (_initialized) {
        return true;
    }

    _reassembly.reset();
    _tx_state.reset();

    gcs().create_virtual_backend(_uart);

    _initialized = true;
    return true;
}

// handle an incoming 0xAA frame
void AP_CRSF_MAVLink::process_frame(const uint8_t *payload, uint8_t len)
{
    if (!_initialized || len < 2) {
        return;
    }

    const uint8_t chunk_info = payload[0];
    const uint8_t data_size = payload[1];
    const uint8_t *data = &payload[2];
    // per CRSF 0xAA envelope spec: chunk fields are zero-based. High nibble
    // is the index of the LAST chunk (not a 1-based count); a single
    // unchunked message is therefore chunk_info == 0x00.
    const uint8_t last_chunk = (chunk_info >> 4) & 0x0F;
    const uint8_t current_chunk = chunk_info & 0x0F;
    const uint8_t total_chunks = last_chunk + 1;

    // validate
    if (current_chunk > last_chunk) {
        _reassembly.reset();
        return;
    }
    if (data_size > MAX_CHUNK_DATA || (data_size + 2) > len) {
        _reassembly.reset();
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // check timeout on in-progress reassembly
    if (_reassembly.active &&
        (now_ms - _reassembly.start_time_ms) > REASSEMBLY_TIMEOUT_MS) {
        _reassembly.reset();
    }

    // start of a new message
    if (current_chunk == 0) {
        _reassembly.reset();
        _reassembly.active = true;
        _reassembly.expected_total = total_chunks;
        _reassembly.start_time_ms = now_ms;
    }

    // validate sequencing
    if (!_reassembly.active ||
        current_chunk != _reassembly.next_chunk ||
        total_chunks != _reassembly.expected_total) {
        _reassembly.reset();
        return;
    }

    // bounds check
    if ((_reassembly.write_offset + data_size) > MAVLINK_MAX_FRAME) {
        _reassembly.reset();
        return;
    }

    // append chunk data
    memcpy(&_reassembly.buf[_reassembly.write_offset], data, data_size);
    _reassembly.write_offset += data_size;
    _reassembly.next_chunk++;

    // if all chunks received, inject into virtual UART
    if (_reassembly.next_chunk == _reassembly.expected_total) {
        _uart.inject_rx(_reassembly.buf, _reassembly.write_offset);
        _reassembly.reset();
    }
}

bool AP_CRSF_MAVLink::tx_pending() const
{
    if (!_initialized) {
        return false;
    }
    return _tx_state.active || _uart.tx_available() > 0;
}

// produce the next outbound 0xAA chunk
bool AP_CRSF_MAVLink::get_telem_frame(uint8_t *payload, uint8_t &len)
{
    if (!_initialized) {
        return false;
    }

    // if no message currently being chunked, pull exactly one MAVLink
    // message from the TX buffer so one envelope carries one message.
    if (!_tx_state.active) {
        uint8_t hdr[3];
        const uint32_t peeked = _uart.peek_tx(hdr, sizeof(hdr));
        if (peeked < 2) {
            return false;
        }

        uint16_t msg_len = 0;
        if (hdr[0] == 0xFE) {
            // MAVLink v1: STX + LEN + SEQ + SYS + COMP + MSGID + payload + 2-byte CRC
            msg_len = 8 + hdr[1];
        } else if (hdr[0] == 0xFD) {
            if (peeked < 3) {
                return false;
            }
            // MAVLink v2: 10-byte header + payload + 2-byte CRC + optional 13-byte signature
            msg_len = 12 + hdr[1] + ((hdr[2] & 0x01) ? 13 : 0);
        } else {
            // not a valid STX; discard one byte to resync
            uint8_t discard;
            _uart.drain_tx(&discard, 1);
            return false;
        }

        if (msg_len > MAVLINK_MAX_FRAME) {
            // malformed length; discard one byte and resync
            uint8_t discard;
            _uart.drain_tx(&discard, 1);
            return false;
        }
        if (_uart.tx_available() < msg_len) {
            // not the whole message yet
            return false;
        }

        _tx_state.msg_len = _uart.drain_tx(_tx_state.buf, msg_len);
        if (_tx_state.msg_len != msg_len) {
            return false;
        }
        _tx_state.total_chunks = (_tx_state.msg_len + MAX_CHUNK_DATA - 1) / MAX_CHUNK_DATA;
        _tx_state.next_chunk = 0;
        _tx_state.active = true;
    }

    // build chunk
    const uint8_t chunk_idx = _tx_state.next_chunk;
    const uint16_t offset = static_cast<uint16_t>(chunk_idx) * MAX_CHUNK_DATA;
    uint8_t data_size = MIN(static_cast<uint16_t>(_tx_state.msg_len - offset), static_cast<uint16_t>(MAX_CHUNK_DATA));

    // per CRSF 0xAA envelope spec: high nibble = last chunk index (count-1),
    // low nibble = current chunk index. Single-chunk messages emit 0x00.
    const uint8_t last_chunk = _tx_state.total_chunks - 1;
    payload[0] = ((last_chunk & 0x0F) << 4) | (chunk_idx & 0x0F);
    payload[1] = data_size;
    memcpy(&payload[2], &_tx_state.buf[offset], data_size);
    len = data_size + 2;

    _tx_state.next_chunk++;
    if (_tx_state.next_chunk >= _tx_state.total_chunks) {
        _tx_state.reset();
    }

    return true;
}

#endif  // AP_CRSF_MAVLINK_ENABLED
