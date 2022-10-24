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

#include "AP_UAVCAN_Serial.h"
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
#define debug(fmt, args...) do { hal.console->printf("AP_UAVCAN_Serial: " fmt, ##args); } while (0)
#else
#define debug(fmt, args...)
#endif

void AP_UAVCAN_Serial::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    auto* node = ap_uavcan->get_node();
    if (node == nullptr) {
        return;
    }
    uavcan::Subscriber<uavcan::tunnel::Broadcast, BroadcastCb> *call_sub;
    call_sub = new uavcan::Subscriber<uavcan::tunnel::Broadcast, BroadcastCb>(*node);
    const int call_sub_res = call_sub->start(BroadcastCb(ap_uavcan, &trampoline_handleBroadcast));
    if (call_sub_res < 0) {
        AP_HAL::panic("UAVCAN Serial subscriber start problem, code: %d", call_sub_res);
    }
}

void AP_UAVCAN_Serial::begin(uint32_t b, uint16_t rxS, uint16_t txS)
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
                AP_BoardConfig::allocation_error("AP_UAVCAN_Serial: broadcast_pub[%d]", i);
            }
        }
        if (serial_config_pub[i] == nullptr) {
            serial_config_pub[i] = new uavcan::Publisher<uavcan::tunnel::SerialConfig>(*ap_uavcan->get_node());
            if (serial_config_pub[i] == nullptr) {
                AP_BoardConfig::allocation_error("AP_UAVCAN_Serial: serial_config_pub[%d]", i);
            }
        }
    }
    _initialized = true;
    _baudrate = b;
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
    return _readbuf.read(buffer, count);
}

size_t AP_UAVCAN_Serial::write(uint8_t c)
{
    return _writebuf.write(&c, 1);
}

size_t AP_UAVCAN_Serial::write(const uint8_t *buffer, size_t size)
{
    return _writebuf.write(buffer, size);
}

void AP_UAVCAN_Serial::handleBroadcast(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BroadcastCb& resp)
{
    // finally consume the msg data
    _readbuf.write(resp.msg->buffer.begin(), resp.msg->buffer.size());
}

void AP_UAVCAN_Serial::trampoline_handleBroadcast(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BroadcastCb& resp)
{
    // find maching serial driver with channel id
    const uint8_t num_uavcan_serials = AP::serialmanager().get_num_phy_serials(AP_SerialManager::SerialPhysical_UAVCAN);
    for (uint8_t i = 0; i < num_uavcan_serials; i++) {
        AP_UAVCAN_Serial* serial = (AP_UAVCAN_Serial*)AP::serialmanager().find_serial_by_phy(AP_SerialManager::SerialPhysical_UAVCAN, i);
        if (serial == nullptr) {
            continue;
        }
        if (serial->_channel_id == resp.msg->channel_id) {
            serial->handleBroadcast(ap_uavcan, node_id, resp);
            break;
        }
    }
}

void AP_UAVCAN_Serial::uavcan_loop(AP_SerialManager::SerialProtocol protocol_id)
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
    if (AP_HAL::millis() - _last_serial_config_ms > 1000 || _baudrate != _last_baudrate) {
        serial_config.channel_id = _channel_id;
        serial_config.baud = _baudrate;
        for (uint8_t i = 0; i < AP::can().get_num_drivers(); i++) {
            if (serial_config_pub[i] != nullptr) {
                serial_config_pub[i]->broadcast(serial_config);
            }
        }
        _last_serial_config_ms = AP_HAL::millis();
        _last_baudrate = _baudrate;
    }

    // check if broadcast or call
    // broadcast
    uavcan::tunnel::Broadcast bcast_msg;
    uint8_t data_count = _writebuf.available();
    if (data_count == 0) {
        // nothing to send
        return;
    }
    for (uint8_t i = 0; i < MIN(data_count, bcast_msg.buffer.capacity()); i++) {
        uint8_t byte;
        if (_writebuf.read_byte(&byte)) {
            bcast_msg.buffer.push_back(byte);
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
#endif
