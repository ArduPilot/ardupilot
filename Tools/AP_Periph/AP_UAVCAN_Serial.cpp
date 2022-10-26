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
 * Code by Siddharth Bharat Purohit, Cubepilot Pty Ltd
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Periph.h"
#include "AP_UAVCAN_Serial.h"

#if AP_SERIAL_EXTENSION_ENABLED
#ifndef HAL_UART_MIN_TX_SIZE
#define HAL_UART_MIN_TX_SIZE 512
#endif

#ifndef HAL_UART_MIN_RX_SIZE
#define HAL_UART_MIN_RX_SIZE 512
#endif

#define IFACE_ALL ((1U<<(HAL_NUM_CAN_IFACES+1U))-1U)

extern const AP_HAL::HAL& hal;

#define DEBUG 0

#if DEBUG
#define debug(fmt, args...) can_printf(fmt, ##args)
#else
#define debug(fmt, args...)
#endif

void AP_UAVCAN_Serial::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    rxS = MAX(rxS, HAL_UART_MIN_RX_SIZE);
    txS = MAX(txS, HAL_UART_MIN_TX_SIZE);

    _readbuf.set_size(rxS);
    _writebuf.set_size(txS);
    _initialized = true;
}

void AP_UAVCAN_Serial::end()
{
    _initialized = false;
    _readbuf.set_size(0);
    _writebuf.set_size(0);
}

int16_t AP_UAVCAN_Serial::read()
{
    uint8_t c;
    if (_readbuf.read_byte(&c)) {
        return c;
    } else {
        return -1;
    }
}

ssize_t AP_UAVCAN_Serial::read(uint8_t *buffer, uint16_t count)
{
    debug("read %d bytes", count);
    return _readbuf.read(buffer, count);
}

size_t AP_UAVCAN_Serial::write(uint8_t c)
{
    return _writebuf.write(&c, 1);
}

size_t AP_UAVCAN_Serial::write(const uint8_t *buffer, size_t size)
{
    // check if space in can buffer
    size_t data_len = _writebuf.write(buffer, size);
    debug("write %d bytes", data_len);
    send_data();
    return data_len;
}

void AP_UAVCAN_Serial::send_data()
{
    if (_writebuf.available() == 0) {
        return;
    }
    int16_t ret;
    while (_writebuf.available()) {
        uavcan_tunnel_Broadcast msg;
        uint8_t buffer[UAVCAN_TUNNEL_BROADCAST_MAX_SIZE];
        msg.buffer.len = _writebuf.peekbytes(msg.buffer.data, sizeof(msg.buffer.data));
        msg.protocol.protocol = _protocol;
        msg.channel_id = _channel_id;
        // encode the message
        uint16_t total_size = uavcan_tunnel_Broadcast_encode(&msg, buffer, !periph.canfdout());
        ret = canard_broadcast(UAVCAN_TUNNEL_BROADCAST_SIGNATURE,
                                        UAVCAN_TUNNEL_BROADCAST_ID,
                                        CANARD_TRANSFER_PRIORITY_LOW,
                                        &buffer[0], total_size);
        if (ret <= 0) {
            // we failed to send, so stop trying
            break;
        }
        _writebuf.advance(msg.buffer.len);
    }
}

AP_SerialManager::SerialProtocol AP_UAVCAN_Serial::tunnel_protocol_to_ap_protocol(uint8_t tunnel_protocol)
{
    switch (tunnel_protocol) {
    case UAVCAN_TUNNEL_PROTOCOL_MAVLINK:
        return AP_SerialManager::SerialProtocol_MAVLink;
    case UAVCAN_TUNNEL_PROTOCOL_MAVLINK2:
        return AP_SerialManager::SerialProtocol_MAVLink2;
    case UAVCAN_TUNNEL_PROTOCOL_GPS_GENERIC:
        return AP_SerialManager::SerialProtocol_GPS;
    }
    return AP_SerialManager::SerialProtocol_None;
}


bool AP_UAVCAN_Serial::handle_tunnel_broadcast(CanardInstance &ins, CanardRxTransfer &transfer, const uavcan_tunnel_Broadcast &msg)
{
    if (_target_node_id == -1 || transfer.source_node_id != _target_node_id) {
        return false;
    }
    // is this message targeted at us
    if (_channel_id != msg.channel_id) {
        // we are already connected to a different channel
        return false;
    }
    debug("UAVCAN Serial: got %u bytes", msg.buffer.len);
    _protocol = msg.protocol.protocol;
    // put the data into the read buffer
    _readbuf.write(msg.buffer.data, msg.buffer.len);
    return true;
}
#endif
