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
/*
  handle tunnelling of serial data over DroneCAN
 */

#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"

#if AP_UART_MONITOR_ENABLED

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

#define TUNNEL_LOCK_KEY 0xf2e460e4U

#ifndef TUNNEL_DEBUG
#define TUNNEL_DEBUG 0
#endif

#if TUNNEL_DEBUG
# define debug(fmt, args...) can_printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)
#endif

/*
  get the default port to tunnel if the client requests port -1
 */
int8_t AP_Periph_FW::get_default_tunnel_serial_port(void) const
{
    int8_t uart_num = -1;
#if HAL_PERIPH_ENABLE_GPS
    if (uart_num == -1) {
        uart_num = g.gps_port;
    }
#endif
#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    if (uart_num == -1) {
        uart_num = g.rangefinder_port[0];
    }
#endif
#ifdef HAL_PERIPH_ENABLE_ADSB
    if (uart_num == -1) {
        uart_num = g.adsb_port;
    }
#endif
#ifdef HAL_PERIPH_ENABLE_PROXIMITY
    if (uart_num == -1) {
        uart_num = g.proximity_port;
    }
#endif
    return uart_num;
}

/*
  handle tunnel data
 */
void AP_Periph_FW::handle_tunnel_Targetted(CanardInstance* canard_ins, CanardRxTransfer* transfer)
{
    uavcan_tunnel_Targetted pkt;
    if (uavcan_tunnel_Targetted_decode(transfer, &pkt)) {
        return;
    }
    if (pkt.target_node != canardGetLocalNodeID(canard_ins)) {
        return;
    }
    if (uart_monitor.buffer == nullptr) {
        uart_monitor.buffer = NEW_NOTHROW ByteBuffer(1024);
        if (uart_monitor.buffer == nullptr) {
            return;
        }
    }
    int8_t uart_num = pkt.serial_id;
    if (uart_num == -1) {
        uart_num = get_default_tunnel_serial_port();
    }
    if (uart_num < 0) {
        return;
    }
    auto *uart = hal.serial(uart_num);
    if (uart == nullptr) {
        return;
    }
    if (uart_monitor.uart_num != uart_num && uart_monitor.uart != nullptr) {
        // remove monitor from previous uart
        hal.serial(uart_monitor.uart_num)->set_monitor_read_buffer(nullptr);
    }
    uart_monitor.uart_num = uart_num;
    if (uart != uart_monitor.uart) {
        // change of uart or expired, clear old data
        uart_monitor.buffer->clear();
        uart_monitor.uart = uart;
        uart_monitor.baudrate = 0;
    }
    if (uart_monitor.uart == nullptr) {
        return;
    }
    /*
      allow for locked state to change at any time, so users can
      switch between locked and unlocked while connected
     */
    const bool was_locked = uart_monitor.locked;
    uart_monitor.locked = (pkt.options & UAVCAN_TUNNEL_TARGETTED_OPTION_LOCK_PORT) != 0;
    if (uart_monitor.locked) {
        uart_monitor.uart->lock_port(TUNNEL_LOCK_KEY, TUNNEL_LOCK_KEY);
    } else {
        uart_monitor.uart->lock_port(0,0);
    }
    uart_monitor.node_id = transfer->source_node_id;
    uart_monitor.protocol = pkt.protocol.protocol;
    if (pkt.baudrate != uart_monitor.baudrate || !was_locked) {
        if (uart_monitor.locked && pkt.baudrate != 0) {
            // ensure we have enough buffer space for a uBlox fw update and fast uCenter data
            uart_monitor.uart->begin_locked(pkt.baudrate, 2048, 2048, TUNNEL_LOCK_KEY);
            debug("begin_locked %u", unsigned(pkt.baudrate));
        }
        uart_monitor.baudrate = pkt.baudrate;
    }
    uart_monitor.uart->set_monitor_read_buffer(uart_monitor.buffer);
    uart_monitor.last_request_ms = AP_HAL::millis();

    // write to device
    if (pkt.buffer.len > 0) {
        if (uart_monitor.locked) {
            debug("write_locked %u", unsigned(pkt.buffer.len));
            uart_monitor.uart->write_locked(pkt.buffer.data, pkt.buffer.len, TUNNEL_LOCK_KEY);
        } else {
            uart_monitor.uart->write(pkt.buffer.data, pkt.buffer.len);
        }
    } else {
        debug("locked keepalive");
    }
}

/*
  send tunnelled serial data
 */
void AP_Periph_FW::send_serial_monitor_data()
{
    if (uart_monitor.uart == nullptr ||
        uart_monitor.node_id == 0 ||
        uart_monitor.buffer == nullptr) {
        return;
    }
    const uint32_t last_req_ms = uart_monitor.last_request_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_req_ms >= 3000) {
        // stop sending and unlock, but don't release the buffer
        if (uart_monitor.locked) {
            debug("unlock");
            uart_monitor.uart->lock_port(0, 0);
        }
        uart_monitor.uart = nullptr;
        return;
    }
    if (uart_monitor.locked) {
        /*
          when the port is locked nobody is reading the uart so the
          monitor doesn't fill. We read here to ensure it fills
         */
        uint8_t buf[120];
        for (uint8_t i=0; i<8; i++) {
            if (uart_monitor.uart->read_locked(buf, sizeof(buf), TUNNEL_LOCK_KEY) <= 0) {
                break;
            }
        }
    }
    uint8_t sends = 8;
    while (uart_monitor.buffer->available() > 0 && sends-- > 0) {
        uint32_t n;
        const uint8_t *buf = uart_monitor.buffer->readptr(n);
        if (n == 0) {
            return;
        }
        // broadcast data as tunnel packets, can be used for uCenter debug and device fw update
        uavcan_tunnel_Targetted pkt {};
        n = MIN(n, sizeof(pkt.buffer.data));
        pkt.target_node = uart_monitor.node_id;
        pkt.protocol.protocol = uart_monitor.protocol;
        pkt.buffer.len = n;
        pkt.baudrate = uart_monitor.baudrate;
        pkt.serial_id = uart_monitor.uart_num;
        memcpy(pkt.buffer.data, buf, n);

        uint8_t buffer[UAVCAN_TUNNEL_TARGETTED_MAX_SIZE];
        const uint16_t total_size = uavcan_tunnel_Targetted_encode(&pkt, buffer, !canfdout());

        debug("read %u", unsigned(n));

        if (!canard_broadcast(UAVCAN_TUNNEL_TARGETTED_SIGNATURE,
                              UAVCAN_TUNNEL_TARGETTED_ID,
                              CANARD_TRANSFER_PRIORITY_MEDIUM,
                              &buffer[0],
                              total_size)) {
            break;
        }
        uart_monitor.buffer->advance(n);
    }
}
#endif // AP_UART_MONITOR_ENABLED
