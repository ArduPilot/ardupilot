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
#include "AP_DroneCAN_Serial.h"

#if AP_SERIAL_EXTENSION_ENABLED || HAL_ENABLE_SERIAL_TUNNEL
#ifndef HAL_UART_MIN_TX_SIZE
#define HAL_UART_MIN_TX_SIZE 512
#endif

#ifndef HAL_UART_MIN_RX_SIZE
#define HAL_UART_MIN_RX_SIZE 512
#endif

#define IFACE_ALL ((1U<<(HAL_NUM_CAN_IFACES+1U))-1U)

extern const AP_HAL::HAL& hal;
extern AP_Periph_FW periph;

#define DEBUG 0

#if DEBUG
#define debug(fmt, args...) can_printf(fmt, ##args)
#else
#define debug(fmt, args...)
#endif

void AP_DroneCAN_Serial::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    rxS = MAX(rxS, HAL_UART_MIN_RX_SIZE);
    txS = MAX(txS, HAL_UART_MIN_TX_SIZE);

    _readbuf.set_size(rxS);
    _writebuf.set_size(txS);

    if (b > 0) {
        _baudrate = b;
    }
    _initialized = true;
}

void AP_DroneCAN_Serial::end()
{
    _initialized = false;
    _readbuf.set_size(0);
    _writebuf.set_size(0);
}

int16_t AP_DroneCAN_Serial::read()
{
    uint8_t c;
    if (_readbuf.read_byte(&c)) {
        return c;
    } else {
        return -1;
    }
}

ssize_t AP_DroneCAN_Serial::read(uint8_t *buffer, uint16_t count)
{
    debug("read %d bytes", count);
    return _readbuf.read(buffer, count);
}

/* Implementations of Stream virtual methods */
uint32_t AP_DroneCAN_Serial::available()
{
    const uint32_t avail = _readbuf.available();

    // don't allow the UART to read any data from us until we have seen
    // at least a _buffer_time_us gap
    if (avail > 0 && (AP_HAL::micros64() - last_read_us) > _buffer_time_us) {
        return avail;
    }

    return 0;
}

size_t AP_DroneCAN_Serial::write(uint8_t c)
{
    return _writebuf.write(&c, 1);
}

size_t AP_DroneCAN_Serial::write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(send_sem);
    // we call send data here to split packets based on idle time
    size_t data_len = _writebuf.write(buffer, size);
    // we call here again to send immediately if idle time is zero
    if (_buffer_time_us == 0) {
        send_data();
    }
    if (last_write_us == 0) {
        last_write_us = AP_HAL::micros64();
    }
    debug("write %d bytes", data_len);
    return data_len;
}

void AP_DroneCAN_Serial::send_data()
{
    // we are busy doing write, no need to block here,
    // we will send data in next iteration, or directly from
    // write call
    if (!send_sem.take_nonblocking()) {
        return;
    }
    uavcan_tunnel_Broadcast msg;
    int16_t ret;
    if ((AP_HAL::micros64() - last_write_us) < _buffer_time_us &&
        _writebuf.available() < sizeof(msg.buffer.data)) {
        send_sem.give();
        return;
    }
    last_write_us = 0;
    while (_writebuf.available()) {
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
    send_sem.give();
}

AP_SerialManager::SerialProtocol AP_DroneCAN_Serial::tunnel_protocol_to_ap_protocol(uint8_t tunnel_protocol)
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


bool AP_DroneCAN_Serial::handle_tunnel_broadcast(CanardInstance &ins, CanardRxTransfer &transfer, const uavcan_tunnel_Broadcast &msg)
{
    // is this message targeted at us
    if (_channel_id != msg.channel_id) {
        // we are already connected to a different channel
        return false;
    }
    debug("UAVCAN Serial: got %u bytes", msg.buffer.len);
    _protocol = msg.protocol.protocol;
    // put the data into the read buffer
    _readbuf.write(msg.buffer.data, msg.buffer.len);

    last_read_us = AP_HAL::micros64();

    return true;
}

bool AP_DroneCAN_Serial::handle_tunnel_config(CanardInstance &ins, CanardRxTransfer &transfer, const uavcan_tunnel_SerialConfig &msg)
{
    set_options(msg.options);
    // port is pointing at the FC so our rx buffer must match its tx buffer
    begin(msg.baud, msg.tx_bufsize, msg.rx_bufsize);

    debug("UAVCAN SerialConfig: baud %u options %u rxsize %u txsize %u", msg.baud, msg.options, msg.rx_bufsize, msg.tx_bufsize);

    return true;
}
#endif
