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

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_HAL/CANIface.h>
#include "AP_CANManager.h"
#include "AP_CANManager_RawSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>

#if AP_CAN_RAW_SERIAL_ENABLED

extern const AP_HAL::HAL& hal;

#define RAW_CAN_HEADER_SIZE offsetof(raw_can_serial_pkt, data)

AP_CANManager_RawSerial* AP_CANManager_RawSerial::instances[HAL_NUM_CAN_IFACES] = {};

// Common method to wait for and read a specified number of bytes
bool AP_CANManager_RawSerial::wait_and_read(uint8_t *buffer, uint16_t count, uint32_t timeout_ms)
{
    if (!_uart || buffer == nullptr || count == 0) {
        return false;
    }
    
    // Wait for the requested number of bytes to be available
    if (!_uart->wait_timeout(count, timeout_ms)) {
        return false;
    }
    
    // Read the bytes in one go
    return (_uart->read(buffer, count) == count);
}

// Initialize with uart port and associated CAN bus
bool AP_CANManager_RawSerial::init()
{
    if (_uart == nullptr) {
        return false;
    }

    // Register frame callback to receive CAN frames
    AP_HAL::CANIface *can_iface = hal.can[_can_bus];
    if (can_iface) {
        can_iface->register_frame_callback(
            FUNCTOR_BIND_MEMBER(&AP_CANManager_RawSerial::frame_callback, void, uint8_t, const AP_HAL::CANFrame &, AP_HAL::CANIface::CanIOFlags),
            _callback_id);
        char thread_name[] = "RawSerialCANx";
        thread_name[sizeof(thread_name)-2] = '0' + _can_bus;
        // Create a thread to handle updates
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_CANManager_RawSerial::update_trampoline, void), 
                                     thread_name, 2048, AP_HAL::Scheduler::PRIORITY_CAN, -1);
    }

    return true;
}

void AP_CANManager_RawSerial::check_and_init(uint8_t can_bus, AP_HAL::UARTDriver* uart, uint32_t baud)
{
    if (can_bus >= HAL_NUM_CAN_IFACES) {
        return;
    }

#if AP_SERIALMANAGER_ENABLED
    AP_SerialManager::SerialProtocol protocol = AP_SerialManager::SerialProtocol(AP_SerialManager::SerialProtocol_RawSerialCAN0 + can_bus);
    if (uart == nullptr) {
        // check if AP_SerialManager has a port for use with RawSerialCAN
        uart = AP::serialmanager().find_serial(protocol, 0);
        baud = AP::serialmanager().find_baudrate(protocol, 0);
    }
#endif

    if (uart == nullptr) {
        return;
    }

    if (instances[can_bus] == nullptr) {
        instances[can_bus] = new AP_CANManager_RawSerial(uart, baud, can_bus);
        if (instances[can_bus] == nullptr) {
            return;
        }
    }

    if (!instances[can_bus]->init()) {
        delete instances[can_bus];
        instances[can_bus] = nullptr;
        return;
    }
}

// Thread for handling RawSerialCAN updates
void AP_CANManager_RawSerial::update_trampoline()
{
    _uart->begin(_baudrate);
    while (true) {
        update();
        hal.scheduler->delay_microseconds(100); // Small delay to prevent CPU hogging
    }
}

// Process incoming bytes for RawSerialCAN protocol
void AP_CANManager_RawSerial::update()
{
    if (_uart == nullptr) {
        return;
    }

    uint16_t available_bytes = _rx_buffer.available();
    // Check if we have enough bytes for the magic number
    if (available_bytes < 2) {
        uint8_t temp_buffer[2];
        // Not enough data for the magic number yet
        if (!wait_and_read(temp_buffer, 2 - available_bytes, 1000)) {
            return;
        }
        _rx_buffer.write(temp_buffer, 2 - available_bytes);
    }

    uint8_t magic_bytes[2];
    _rx_buffer.peekbytes(magic_bytes, 2);
    
    uint16_t magic = magic_bytes[0] | (magic_bytes[1] << 8);
    if (magic != RAW_CAN_SERIAL_MAGIC) {
        _rx_buffer.advance(1); // Discard the first byte and try again
        return; // Continue checking with next byte
    }
    
    // We found the magic number, now check if we have a complete header
    available_bytes = _rx_buffer.available();
    if (available_bytes < RAW_CAN_HEADER_SIZE) {
        // Not enough data for the complete header yet
        // Try to wait for the rest of the header
        uint8_t needed_bytes = RAW_CAN_HEADER_SIZE - available_bytes;
        uint8_t temp_buffer[needed_bytes];
        if (wait_and_read(temp_buffer, needed_bytes, 1000)) {
            _rx_buffer.write(temp_buffer, needed_bytes);
        } else {
            return;
        }
    }
    
    // Peek at the full header
    uint8_t header[RAW_CAN_HEADER_SIZE];
    _rx_buffer.peekbytes(header, RAW_CAN_HEADER_SIZE);

    // Parse the header fields
    raw_can_serial_pkt packet;
    packet.magic = magic;
    packet.crc = header[2] | (header[3] << 8);
    packet.flags = header[4] | (header[5] << 8);
    packet.message_id = header[6] | (header[7] << 8) | (header[8] << 16) | (header[9] << 24);
    
    // Extract and validate data length
    uint8_t data_length = (packet.flags & RAW_CAN_SERIAL_SIZE_MASK) >> RAW_CAN_SERIAL_SIZE_SHIFT;
    if (data_length > 64) {
        // Invalid data length, discard the first byte and try again
        _rx_buffer.advance(2); // Discard the first two bytes and try again
        return;
    }
    
    // Check if we have the complete packet (header + data)
    if (_rx_buffer.available() < RAW_CAN_HEADER_SIZE + data_length) {
        // Not enough data for the complete packet yet
        // Try to wait for the data portion
        uint8_t needed_bytes = RAW_CAN_HEADER_SIZE + data_length - _rx_buffer.available();
        uint8_t temp_buffer[needed_bytes];
        if (wait_and_read(temp_buffer, needed_bytes, 1000)) {
            _rx_buffer.write(temp_buffer, needed_bytes);
        } else {
            return;
        }
    }
    
    // Read data portion directly into our packet structure
    if (data_length > 0) {
        for (uint8_t i = 0; i < data_length; i++) {
            packet.data[i] = _rx_buffer.peek(i + RAW_CAN_HEADER_SIZE);
        }
    }
    
    // Verify CRC
    uint16_t calculated_crc = crc16_ccitt((uint8_t*)&packet.flags, data_length + 6, 0xFFFFU);
    if (packet.crc != calculated_crc) {
        // CRC mismatch - this means the packet is corrupted
        // Since we already consumed the packet, we just return
        _rx_buffer.advance(2); // Discard the first two bytes and try again
        return;
    }

    _rx_buffer.advance(RAW_CAN_HEADER_SIZE + data_length); // Consume the packet

    // Extract flags for the frame
    bool is_canfd = (packet.flags & RAW_CAN_SERIAL_FLAG_CANFD) != 0;
    
    // Create a CAN frame and send it to the bus
    AP_HAL::CANFrame frame(packet.message_id, packet.data, data_length, is_canfd);
    
    // Get CAN interface for this bus
    AP_HAL::CANIface *can_iface = hal.can[_can_bus];
    if (can_iface != nullptr) {
        // Send the frame to the CAN interface with the IsForwardedFrame flag
        can_iface->send(frame, AP_HAL::micros64() + 100000, AP_HAL::CANIface::IsForwardedFrame);
    }
    
    return; // Successfully processed a packet
}

// Callback for frame received that should be sent over serial
void AP_CANManager_RawSerial::frame_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags)
{
    if (_uart == nullptr) {
        return;
    }

    struct raw_can_serial_pkt pkt {};
    pkt.magic = RAW_CAN_SERIAL_MAGIC;
    pkt.flags = 0;
    
    const uint8_t data_len = AP_HAL::CANFrame::dlcToDataLength(frame.dlc);
    if (data_len > 64) {  // Maximum CAN-FD data length
        return;
    }
    
#if HAL_CANFD_SUPPORTED
    if (frame.isCanFDFrame()) {
        pkt.flags |= RAW_CAN_SERIAL_FLAG_CANFD;
    }
#endif
    
    // Embed size in the 6 MSB bits of flags
    pkt.flags |= ((uint16_t)data_len << RAW_CAN_SERIAL_SIZE_SHIFT) & RAW_CAN_SERIAL_SIZE_MASK;
    
    pkt.message_id = frame.id;
    memcpy(pkt.data, frame.data, data_len);
    
    // Calculate CRC16 of the packet (excluding magic and CRC fields)
    const auto crc = crc16_ccitt((uint8_t*)&pkt.flags, data_len + 6, 0xFFFFU);
    pkt.crc = crc;
    
    // Send the complete packet
    const size_t total_len = offsetof(raw_can_serial_pkt, data) + data_len;
    _uart->write((const uint8_t*)&pkt, total_len);
    return;
}

#endif // AP_CAN_RAW_SERIAL_ENABLED
