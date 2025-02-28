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

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_CANManager_config.h"
#include <AP_HAL/utility/RingBuffer.h>

#if AP_CAN_RAW_SERIAL_ENABLED

// Raw CAN Serial protocol configuration
#define RAW_CAN_SERIAL_MAX_PKT_LEN 74  // 64 byte data + 10 byte header
#define RAW_CAN_SERIAL_MAGIC 0x2934U
#define RAW_CAN_SERIAL_FLAG_CANFD 0x0001
#define RAW_CAN_SERIAL_SIZE_MASK 0xFC00  // 6 MSB bits of flags for size
#define RAW_CAN_SERIAL_SIZE_SHIFT 10     // Bits to shift size value

// Structure for raw CAN serial packet
struct PACKED raw_can_serial_pkt {
    uint16_t magic;
    uint16_t crc;
    uint16_t flags;
    uint32_t message_id;
    uint8_t data[RAW_CAN_SERIAL_MAX_PKT_LEN-10];
};

class AP_CANManager_RawSerial {
public:
    AP_CANManager_RawSerial(AP_HAL::UARTDriver* uart, uint32_t baudrate, uint32_t can_bus) :
        _can_bus(can_bus),
        _uart(uart),
        _baudrate(baudrate)
    {}

    // Initialize with uart port and associated CAN bus
    bool init();

    // check if setup and initialize if so
    static void check_and_init(uint8_t can_bus, AP_HAL::UARTDriver* uart = nullptr, uint32_t baud = 0);

private:
    uint8_t _can_bus;           // Associated CAN bus
    uint8_t _callback_id;       // CAN callback ID for forwarding
    AP_HAL::UARTDriver* _uart;  // UART for communication
    uint32_t _baudrate;         // Baudrate for UART
    
    // Process incoming bytes for RawSerialCAN protocol
    void update();
    void update_trampoline();

    // Common method to wait for and read a specified number of bytes
    bool wait_and_read(uint8_t *buffer, uint16_t count, uint32_t timeout_ms);

    // Callback for frame received that should be sent over serial
    void frame_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags);

    static AP_CANManager_RawSerial *instances[HAL_NUM_CAN_IFACES];

    // Buffer to store bytes from UART
    ByteBuffer _rx_buffer{RAW_CAN_SERIAL_MAX_PKT_LEN * 4}; // Larger buffer to handle multiple packets
};

#endif // AP_CAN_RAW_SERIAL_ENABLED
