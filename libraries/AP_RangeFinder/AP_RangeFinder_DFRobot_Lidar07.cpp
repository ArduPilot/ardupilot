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

#include "AP_RangeFinder_DFRobot_Lidar07.h"

#if AP_RANGEFINDER_DFROBOT_LIDAR07_ENABLED

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
#include <AP_HAL/utility/sparse-endian.h>
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

#define LIDAR07_SOF_OFFSET              (0)
#define LIDAR07_REG_OFFSET              (LIDAR07_SOF_OFFSET + sizeof(uint8_t))
// value in the controller to sensor command
#define LIDAR07_VALUE_OFFSET            (LIDAR07_REG_OFFSET + sizeof(uint8_t))
// size of data in the sensor to controller reply
#define LIDAR07_LEN_OFFSET              (LIDAR07_REG_OFFSET + sizeof(uint8_t))
// start of data in the sensor to controller reply
#define LIDAR07_DATA_OFFSET             (LIDAR07_LEN_OFFSET + sizeof(uint16_t))
// measurement message: distance
#define LIDAR07_DIST_OFFSET             (LIDAR07_DATA_OFFSET)
// measurement message: temperature
#define LIDAR07_TEMP_OFFSET             (LIDAR07_DIST_OFFSET + sizeof(uint16_t))
// measurement message: amplitude
#define LIDAR07_AMP_OFFSET              (LIDAR07_TEMP_OFFSET + sizeof(uint16_t))
// measurement message: ambient light
#define LIDAR07_AMB_OFFSET              (LIDAR07_AMP_OFFSET + sizeof(uint16_t))
// measurement message: ToF phase
#define LIDAR07_TOF_OFFSET              (LIDAR07_AMB_OFFSET + sizeof(uint16_t))
// crc32 for controller to sensor
#define LIDAR07_CRC_OFFSET              (LIDAR07_VALUE_OFFSET + sizeof(uint32_t))
// crc32 for sensor to given payload size
#define LIDAR07_CRC_SENSOR_OFFSET(n)    (LIDAR07_DATA_OFFSET + n)
// Total length for controller to sensor
#define LIDAR07_CONTROLLER_LEN          (LIDAR07_CRC_OFFSET + sizeof(uint32_t))
// Total length for sensor to controller
#define LIDAR07_SENSOR_LEN(n)           (LIDAR07_CRC_SENSOR_OFFSET(n) + sizeof(uint32_t))
// Longest payload length
#define LIDAR07_MAX_LEN                 (16)

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

void AP_RangeFinder_DFRobot_Lidar07::send_command(uint8_t reg, uint32_t value)
{
    if (uart == nullptr) {
        return;
    }

    uint8_t cmd_buf[LIDAR07_CONTROLLER_LEN];
    cmd_buf[LIDAR07_SOF_OFFSET] = LIDAR07_SOF_CONTROLLER;
    cmd_buf[LIDAR07_REG_OFFSET] = reg;
    *reinterpret_cast<uint32_t*>(&cmd_buf[LIDAR07_VALUE_OFFSET]) = htole32(value);
    *reinterpret_cast<uint32_t*>(&cmd_buf[LIDAR07_CRC_OFFSET]) = htole32(crc32_unreflected(cmd_buf, LIDAR07_CRC_OFFSET));
    uart->write(cmd_buf, sizeof(cmd_buf));

    // the sensor doesn't quite like when we lump commands together, give it piecewise input
    uart->flush();

    // as well as yield to the scheduler to give the sensor some pause to think
    hal.scheduler->delay(50);
}

bool AP_RangeFinder_DFRobot_Lidar07::scan_input(uint8_t reg, size_t read_max)
{
    if (uart == nullptr) {
        return false;
    }

    // try reading a whole message from the serial
    for (size_t i = 0; i < read_max; i++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }

        // reject spurious fragments
        if (_buf_pos == LIDAR07_SOF_OFFSET && c != LIDAR07_SOF_SENSOR) {
            continue;
        }
        if (_buf_pos == LIDAR07_REG_OFFSET && c != reg) {
            _buf_pos = 0;
            continue;
        }

        _buf[_buf_pos++] = c;

        // pull the next byte if we haven't read the length yet
        if (_buf_pos < LIDAR07_DATA_OFFSET) {
            continue;
        }

        size_t data_len = *reinterpret_cast<uint16_t*>(&_buf[LIDAR07_LEN_OFFSET]);
        // check if data length makes sense after we read all of the value
        if (_buf_pos == LIDAR07_DATA_OFFSET && data_len > LIDAR07_MAX_LEN) {
            _buf_pos = 0;
            continue;
        }

        // looks like we have a full message, reset the counter and verify it
        if (_buf_pos >= LIDAR07_SENSOR_LEN(data_len)) {
            _buf_pos = 0;

            // crc32 offset depending on the data length
            uint32_t crc32_offset = LIDAR07_CRC_SENSOR_OFFSET(data_len);

            // match the checksums
            // incoming value on the wire is little endian
            uint32_t message_crc32 = crc32_unreflected(_buf, crc32_offset);
            const uint32_t *incoming_crc32 = reinterpret_cast<const uint32_t*>(&_buf[crc32_offset]);
            if (le32toh(*incoming_crc32) == message_crc32) {
                return true;
            }
        }
    }

    // we've got nothing at this time
    return false;
}

bool AP_RangeFinder_DFRobot_Lidar07::get_reading(float &reading_m)
{
    // invalidate connection if no data received in a while
    if (_initialized && AP_HAL::millis() - _last_read_ms > LIDAR07_STALE_CONNECTION_MS) {
        _initialized = false;
    }

    // set things up if not initialized yet
    if (!_initialized) {
        // no low noise filter to speed things up
        send_command(LIDAR07_REG_FILTER | LIDAR07_WRITE_MASK, 0);

        // continuous mode
        send_command(LIDAR07_REG_MODE | LIDAR07_WRITE_MASK, 1);

        // frame rate
        send_command(LIDAR07_REG_INTERVAL | LIDAR07_WRITE_MASK, LIDAR07_FRAME_INTERVAL_MS);

        // start measuring
        send_command(LIDAR07_REG_MEASUREMENT | LIDAR07_WRITE_MASK, 1);

        // NOTE: we could wait and validate the responses here
        _initialized = true;
    }
    // otherwise do a normal input scan
    else if (scan_input(LIDAR07_REG_MEASUREMENT | LIDAR07_WRITE_MASK, _buf_max)) {
        uint16_t *p_distance = reinterpret_cast<uint16_t*>(&_buf[LIDAR07_DIST_OFFSET]);
        uint16_t *p_temperature = reinterpret_cast<uint16_t*>(&_buf[LIDAR07_TEMP_OFFSET]);
        uint16_t *p_amplitude = reinterpret_cast<uint16_t*>(&_buf[LIDAR07_AMP_OFFSET]);
        uint16_t *p_ambient_light = reinterpret_cast<uint16_t*>(&_buf[LIDAR07_AMB_OFFSET]);
        uint64_t *p_tof_phase = reinterpret_cast<uint64_t*>(&_buf[LIDAR07_TOF_OFFSET]);

        reading_m = le16toh(*p_distance)/1000.0;
        _temperature = le16toh(*p_temperature)/100.0;

        // NOTE: we can probably figure out signal quality by light level/amplitude
        _amplitude = *p_amplitude;
        _ambient_light = *p_ambient_light;
        _tof_phase = *p_tof_phase;

        // mark the reception time to know when the temperature and side values have expired
        _last_read_ms = AP_HAL::millis();
        return true;
    }

    // no reading
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
