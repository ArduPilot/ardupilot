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

#include "AP_HAL/utility/sparse-endian.h"
#include "AP_RangeFinder_config.h"
#include <endian.h>

#if AP_RANGEFINDER_DFROBOT_LIDAR07_ENABLED

#include "AP_RangeFinder_DFRobot_Lidar07.h"

// DFRobot Lidar07 support driver
// Also sold as VSemi Atom
// Documentation: https://dfimg.dfrobot.com/nobody/wiki/1840a7b7b14e02f3566e0cef5b51e9ba.pdf
// https://wiki.dfrobot.com/TOF_IR_Distance_Sensor_0.2_12m_SKU_SEN0413
// 
// Current implementation only is limited to:
// - UART interface
// - Basic distance reading in continuous mode without an inbuilt filter

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
extern const AP_HAL::HAL& hal;

#define LIDAR07_SOF_SENSOR      (0xfa)
#define LIDAR07_SOF_CONTROLLER  (0xf5)
#define LIDAR07_WRITE_MASK      (0x80)
#define LIDAR07_REG_VERSION     (0x43)
#define LIDAR07_REG_FILTER      (0x59)
#define LIDAR07_REG_MEASUREMENT (0x60)
#define LIDAR07_REG_MODE        (0x61)
#define LIDAR07_REG_INTERVAL    (0x62)
#define LIDAR07_REG_ERROR       (0x65)

// the sensor supports rates up to 100 Hz
// use the lowest common rate between Copter and Plane: 20 Hz = 50 ms
#define LIDAR07_FRAME_INTERVAL_MS       (50)

// Invalidate the temperature and other readings after 2 measurement intervals
#define LIDAR07_DATA_EXPIRATION_MS      (2*LIDAR07_FRAME_INTERVAL_MS)

// Retry initialization if inactive for 30 seconds
#define LIDAR07_STALE_CONNECTION_MS     (30*1000)

// DFRobot's CRC32 implementation uses unreflected bit order
// apparently it's not the way CRC32 is used elsewhere so it makes sense to hide
// this utility under the define block
template <typename T> T reflect(const T& p)
{
    // NOTE: straightforward bit reflection, can probably be optimized if used in high frequency code
    T res = 0;
    size_t bit_len = sizeof(T) * 8;
    for (size_t i = 0; i < bit_len; i++) {
        res |= bool(p & (1<<i)) << (bit_len - i - 1);
    }
    return res;
}

uint32_t AP_RangeFinder_DFRobot_Lidar07::crc32_unreflected(uint8_t *buf, size_t n)
{
    uint8_t ubuf[n];
    for (size_t i = 0; i < n; i++)
        ubuf[i] = reflect<uint8_t>(buf[i]);
    return reflect<uint32_t>(crc_crc32(0xffffffff, ubuf, n));
}

void AP_RangeFinder_DFRobot_Lidar07::prepare_command(uint8_t reg, uint32_t value)
{
    _packet.start = LIDAR07_SOF_CONTROLLER;
    _packet.reg = reg;
    _packet.command.value = htole32(value);
    _packet.command.checksum = htole32(crc32_unreflected(_buf, offsetof(Packet, command.checksum)));
    _buf_pos = 0;
    _writing = true;
    _exp_reg = reg;
}

bool AP_RangeFinder_DFRobot_Lidar07::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // are we coming out of reset?
    if (!_exp_reg) {
        // no low noise filter to speed things up
        prepare_command(LIDAR07_REG_FILTER | LIDAR07_WRITE_MASK, 0);
        _last_read_ms = AP_HAL::millis();
    }

    // we have data to write, send what we can and return to get rescheduled
    if (_writing) {
        size_t left = offsetof(Packet, command.checksum) + sizeof(Packet::command.checksum) - _buf_pos;
        size_t written = uart->write(&_buf[_buf_pos], left);
        if (written < left) {
            _buf_pos += written;
        } else {
            _buf_pos = 0;
            _writing = false;
        }

        return false;
    }

    // invalidate connection and restart the setup if no data received in a while
    if (AP_HAL::millis() - _last_read_ms > LIDAR07_STALE_CONNECTION_MS) {
        _exp_reg = 0;
        return false;
    }

    // try reading a whole message from the serial
    for (size_t i = 0; i < sizeof(_buf); i++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        _last_read_ms = AP_HAL::millis();

        // restart if we are somehow out of buffer
        if (_buf_pos == sizeof(_buf)) {
            _buf_pos = 0;
            continue;
        }

        // reject spurious fragments
        if (_buf_pos == offsetof(Packet, start) && c != LIDAR07_SOF_SENSOR) {
            continue;
        }

        // reject things we aren't expecting to get
        if (_buf_pos == offsetof(Packet, reg) && c != _exp_reg) {
            _buf_pos = 0;
            continue;
        }

        _buf[_buf_pos++] = c;

        // pull the next byte if we haven't read the length yet
        constexpr size_t data_offset = offsetof(Packet, len) + sizeof(Packet::len);
        if (_buf_pos < data_offset) {
            continue;
        }

        // check if data length makes sense after we read all of the value
        bool measuring = _exp_reg == (LIDAR07_REG_MEASUREMENT | LIDAR07_WRITE_MASK);
        size_t exp_len = measuring? sizeof(Packet::measurement.data) : sizeof(Packet::responce.data);
        if (_buf_pos == data_offset && le16toh(_packet.len) != exp_len) {
            _buf_pos = 0;
            continue;
        }

        size_t crc32_offset = measuring? offsetof(Packet, measurement.checksum) : offsetof(Packet, responce.checksum);
        // looks like we have a full message, reset the counter and verify it
        if (_buf_pos >= crc32_offset + sizeof(uint32_t)) {
            _buf_pos = 0;

            // match the checksums
            // incoming value on the wire is little endian
            uint32_t message_crc32 = crc32_unreflected(_buf, crc32_offset);
            uint32_t incoming_crc32 = measuring? _packet.measurement.checksum : _packet.responce.checksum;
            if (le32toh(incoming_crc32) == message_crc32) {
                // We have a valid message, act in accordance to what we are expecting
                // NOTE: we aren't actually checking that the responce values match what we demanded
                switch (_exp_reg) {
                    case LIDAR07_REG_FILTER | LIDAR07_WRITE_MASK:
                        // continuous mode
                        prepare_command(LIDAR07_REG_MODE | LIDAR07_WRITE_MASK, 1);
                        return false;

                    case LIDAR07_REG_MODE | LIDAR07_WRITE_MASK:
                        // frame rate
                        prepare_command(LIDAR07_REG_INTERVAL | LIDAR07_WRITE_MASK, LIDAR07_FRAME_INTERVAL_MS);
                        return false;

                    case LIDAR07_REG_INTERVAL | LIDAR07_WRITE_MASK:
                        // start measuring
                        prepare_command(LIDAR07_REG_MEASUREMENT | LIDAR07_WRITE_MASK, 1);
                        return false;

                    case LIDAR07_REG_MEASUREMENT | LIDAR07_WRITE_MASK:
                        // got a valid measurement, capture the fields
                        reading_m = le16toh(_packet.measurement.data.distance)/1000.0;
                        _temperature = le16toh(_packet.measurement.data.temperature)/100.0;

                        // NOTE: other fields are unused right now
                        // we can probably figure out signal quality by light level/amplitude
                        return true;
                }
            }

            // wrong checksum or messed up register?
            _buf_pos = 0;
        }
    }

    // we've got nothing at this time
    return false;
}

bool AP_RangeFinder_DFRobot_Lidar07::get_temp(float &temp) const
{
    if (AP_HAL::millis() - _last_read_ms > LIDAR07_DATA_EXPIRATION_MS) {
        return false;
    }

    temp = _temperature;
    return true;
}

#endif  // AP_RANGEFINDER_DFROBOT_LIDAR07_ENABLED
