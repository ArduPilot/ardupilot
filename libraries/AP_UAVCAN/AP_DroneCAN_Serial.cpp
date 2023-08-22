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
 * Code by Siddharth Bharat Purohit, Cubepilot Pty.
 */

#include <AP_HAL/AP_HAL.h>
#if HAL_ENABLE_LIBUAVCAN_DRIVERS && AP_SERIAL_EXTENSION_ENABLED

#include <AP_Math/AP_Math.h>
#include "AP_DroneCAN_Serial.h"
#include "AP_UAVCAN.h"
#include <uavcan/uavcan.hpp>
#include <uavcan/tunnel/Broadcast.hpp>
#include <uavcan/tunnel/SerialConfig.hpp>
#include <AP_BoardConfig/AP_BoardConfig.h>

#ifndef HAL_UART_MIN_TX_SIZE
#define HAL_UART_MIN_TX_SIZE 512
#endif

#ifndef HAL_UART_MIN_RX_SIZE
#define HAL_UART_MIN_RX_SIZE 512
#endif

extern const AP_HAL::HAL& hal;

UC_REGISTRY_BINDER(BroadcastCb, uavcan::tunnel::Broadcast);
static uavcan::Publisher<uavcan::tunnel::Broadcast>* broadcast_pub[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static uavcan::Publisher<uavcan::tunnel::SerialConfig>* serial_config_pub[HAL_MAX_CAN_PROTOCOL_DRIVERS];

#define DEBUG 1

#if DEBUG
#define debug(fmt, args...) do { hal.console->printf("AP_DroneCAN_Serial: " fmt, ##args); } while (0)
#else
#define debug(fmt, args...)
#endif

void AP_DroneCAN_Serial::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    auto* node = ap_uavcan->get_node();
    if (node == nullptr) {
        return;
    }
    uavcan::Subscriber<uavcan::tunnel::Broadcast, BroadcastCb> *call_sub;
    call_sub = new uavcan::Subscriber<uavcan::tunnel::Broadcast, BroadcastCb>(*node);
    const int call_sub_res = call_sub->start(BroadcastCb(ap_uavcan, &trampoline_handleBroadcast));
    if (call_sub_res < 0) {
        AP_HAL::panic("DroneCAN Serial subscriber start problem, code: %d", call_sub_res);
    }
}

void AP_DroneCAN_Serial::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    rxS = MAX(rxS, HAL_UART_MIN_RX_SIZE);
    txS = MAX(txS, HAL_UART_MIN_TX_SIZE);

    _readbuf.set_size(rxS);
    _writebuf.set_size(txS);

    // Initialise publishers
    for (uint8_t i = 0; i < AP::can().get_num_drivers(); i++) {
        AP_UAVCAN* ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }
        if (broadcast_pub[i] == nullptr) {
            broadcast_pub[i] = new uavcan::Publisher<uavcan::tunnel::Broadcast>(*ap_uavcan->get_node());
            if (broadcast_pub[i] == nullptr) {
                AP_BoardConfig::allocation_error("AP_DroneCAN_Serial: broadcast_pub[%d]", i);
            }
        }
        if (serial_config_pub[i] == nullptr) {
            serial_config_pub[i] = new uavcan::Publisher<uavcan::tunnel::SerialConfig>(*ap_uavcan->get_node());
            if (serial_config_pub[i] == nullptr) {
                AP_BoardConfig::allocation_error("AP_DroneCAN_Serial: serial_config_pub[%d]", i);
            }
        }
    }
    _initialized = true;

    if (b > 0) {
        _baudrate = b;
    }
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
    return _readbuf.read(buffer, count);
}

size_t AP_DroneCAN_Serial::write(uint8_t c)
{
    _last_write_us = AP_HAL::micros64();
    return _writebuf.write(&c, 1);
}

size_t AP_DroneCAN_Serial::write(const uint8_t *buffer, size_t size)
{
    _last_write_us = AP_HAL::micros64();
    return _writebuf.write(buffer, size);
}

void AP_DroneCAN_Serial::handleBroadcast(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BroadcastCb& resp)
{
    // finally consume the msg data
    _readbuf.write(resp.msg->buffer.begin(), resp.msg->buffer.size());
}

void AP_DroneCAN_Serial::trampoline_handleBroadcast(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BroadcastCb& resp)
{
    // find matching serial driver with channel id
    const uint8_t num_dronecan_serials = AP::serialmanager().get_num_phy_serials(AP_SerialManager::SerialPhysical_DroneCAN);
    for (uint8_t i = 0; i < num_dronecan_serials; i++) {
        AP_DroneCAN_Serial* serial = (AP_DroneCAN_Serial*)AP::serialmanager().find_serial_by_phy(AP_SerialManager::SerialPhysical_DroneCAN, i);
        if (serial == nullptr) {
            continue;
        }
        if (serial->_channel_id == resp.msg->channel_id) {
            serial->handleBroadcast(ap_uavcan, node_id, resp);
            break;
        }
    }
}

void AP_DroneCAN_Serial::dronecan_loop(AP_SerialManager::SerialProtocol protocol_id, uint32_t buffer_us)
{
    if (!_initialized) {
        return;
    }
    uavcan::tunnel::Protocol protocol;
    switch (protocol_id) {
        case AP_SerialManager::SerialProtocol_MAVLink:
            protocol.protocol = uavcan::tunnel::Protocol::MAVLINK;
            break;
        case AP_SerialManager::SerialProtocol_MAVLink2:
            protocol.protocol = uavcan::tunnel::Protocol::MAVLINK2;
            break;
        case AP_SerialManager::SerialProtocol_GPS:
        case AP_SerialManager::SerialProtocol_GPS2:
            protocol.protocol = uavcan::tunnel::Protocol::GPS_GENERIC;
            break;
        default:
            protocol.protocol = uavcan::tunnel::Protocol::UNDEFINED;
            break;
    }

    uavcan::tunnel::SerialConfig serial_config;
    // push out config changes every 1s
    if (AP_HAL::millis() - _last_serial_config_ms > 1000 || _baudrate != _last_baudrate) {
        serial_config.channel_id = _channel_id;
        serial_config.baud = _baudrate;
        serial_config.options = get_options();
        serial_config.rx_bufsize = get_rx_buffer_size();
        serial_config.tx_bufsize = get_tx_buffer_size();
        for (uint8_t i = 0; i < AP::can().get_num_drivers(); i++) {
            if (serial_config_pub[i] != nullptr) {
                serial_config_pub[i]->broadcast(serial_config);
            }
        }
        _last_serial_config_ms = AP_HAL::millis();
        _last_baudrate = _baudrate;
    }

    if (!_flush && AP_HAL::micros64() - _last_write_us < buffer_us) {
        return;
    }

    // Broadcast remaining data
    // the general problem is that CAN is slow compared to a UART and the packets are small
    // this means that if we are not careful a stream of data can be split in transmission and
    // end up being sent to the end device in pieces, which it doesn't understand.
    // the CAN payload is 60 bytes and the loop below can send one of these every 1-2ms. It can
    // then take another 10ms for these to actually get processed on the receiving node. What we want is
    // for gaps in the original stream to appear as gaps in the ultimate stream. So:
    // 1. On the sender (here) we wait for a gap of at least buffer_us before attempting to write. This does
    //    not need to be very long (1ms is usually enough) but makes sure that we have all the data we are 
    //    going to likely need. We then send all of this data in a loop.
    // 2. On the receiver we similarly wait for the absence of packets before trying to send to the UART. This 
    //    delay needs to be longer to take account of transmission delays.
    // 3. We need to make sure that the sender UART does not try and push out a packet until the last one is done
    //    otherwise we get packets smushed together. Generally this requires that buffer sizes match through the
    //    whole chain.
    uavcan::tunnel::Broadcast bcast_msg;
    uint32_t data_count = _writebuf.available();

    while (data_count > 0) {
        bcast_msg.buffer.clear();
        const uint16_t packet_size = MIN(data_count, bcast_msg.buffer.capacity());
        for (uint16_t i = 0; i < packet_size; i++) {
            uint8_t byte;
            if (_writebuf.read_byte(&byte)) {
                bcast_msg.buffer.push_back(byte);
                data_count--;
            } else {
                break;
            }
        }
        // set channel id
        bcast_msg.channel_id = _channel_id;
        // set protocol_id
        bcast_msg.protocol = protocol;
        // find and publish on all interfaces
        for (uint8_t i = 0; i < AP::can().get_num_drivers(); i++) {
            if (broadcast_pub[i] != nullptr) {
                broadcast_pub[i]->broadcast(bcast_msg);
            }
        }
    }

    _flush = false;
}
#endif
