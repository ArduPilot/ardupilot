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
   driver for Cheerson CX-OF optical flow sensor

   CXOF serial packet description
   byte0: header (0xFE)
   byte1: reserved
   byte2: x-motion low byte;
   byte3: x-motion high byte;
   byte4: y-motion low byte;
   byte5: y-motion high byte;
   byte6: t-motion
   byte7: surface quality
   byte8: footer (0xAA)

   sensor sends packets at 25hz
 */

#include "AP_OpticalFlow_CXOF.h"

#if AP_OPTICALFLOW_CXOF_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <utility>
#include "AP_OpticalFlow.h"
#include <stdio.h>

#define CXOF_HEADER         (uint8_t)0xFE
#define CXOF_FOOTER         (uint8_t)0xAA
#define CXOF_FRAME_LENGTH               9
#define CXOF_PIXEL_SCALING      (1.76e-3)
#define CXOF_TIMEOUT_SEC             0.3f

extern const AP_HAL::HAL& hal;

// constructor
AP_OpticalFlow_CXOF::AP_OpticalFlow_CXOF(AP_OpticalFlow &_frontend, AP_HAL::UARTDriver *_uart) :
    OpticalFlow_backend(_frontend),
    uart(_uart)
{
}

// detect the device
AP_OpticalFlow_CXOF *AP_OpticalFlow_CXOF::detect(AP_OpticalFlow &_frontend)
{
    AP_SerialManager *serial_manager = AP::serialmanager().get_singleton();
    if (serial_manager == nullptr) {
        return nullptr;
    }

    // look for first serial driver with protocol defined as OpticalFlow
    AP_HAL::UARTDriver *uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_OpticalFlow, 0);
    if (uart == nullptr) {
        return nullptr;
    }

    // we have found a serial port so use it
    AP_OpticalFlow_CXOF *sensor = NEW_NOTHROW AP_OpticalFlow_CXOF(_frontend, uart);
    return sensor;
}

// initialise the sensor
void AP_OpticalFlow_CXOF::init()
{
    // sanity check uart
    if (uart == nullptr) {
        return;
    }

    // open serial port with baud rate of 19200
    uart->begin(19200);

    last_frame_us = AP_HAL::micros();
}

// read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_CXOF::update(void)
{
    // sanity check uart
    if (uart == nullptr) {
        return;
    }

    // record gyro values as long as they are being used
    // the sanity check of dt below ensures old gyro values are not used
    if (gyro_sum_count < 1000) {
        const Vector3f& gyro = AP::ahrs().get_gyro();
        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum_count++;
    }

    // sensor values
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    uint16_t qual_sum = 0;
    uint16_t count = 0;

    // read any available characters in the serial buffer
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c;
        if (!uart->read(c)) {
            continue;
        }
        // if buffer is empty and this byte is header, add to buffer
        if (buf_len == 0) {
            if (c == CXOF_HEADER) {
                buf[buf_len++] = c;
            }
        } else {
            // add character to buffer
            buf[buf_len++] = c;

            // if buffer has 9 items try to decode it
            if (buf_len >= CXOF_FRAME_LENGTH) {
                // check last character matches footer
                if (buf[buf_len-1] != CXOF_FOOTER) {
                    buf_len = 0;
                    continue;
                }

                // decode package
                int16_t x_raw = (int16_t)((uint16_t)buf[3] << 8) | buf[2];
                int16_t y_raw = (int16_t)((uint16_t)buf[5] << 8) | buf[4];

                // add to sum of all readings from sensor this iteration
                count++;
                x_sum += x_raw;
                y_sum += y_raw;
                qual_sum += buf[7];

                // clear buffer
                buf_len = 0;
            }
        }
    }

    // return without updating state if no readings
    if (count == 0) {
        return;
    }

    struct AP_OpticalFlow::OpticalFlow_state state {};

    // average surface quality scaled to be between 0 and 255
    state.surface_quality = (constrain_int16(qual_sum / count, 64, 78) - 64) * 255 / 14;

    // calculate dt
    uint64_t this_frame_us = uart->receive_time_constraint_us(CXOF_FRAME_LENGTH);
    if (this_frame_us == 0) {
        // for HAL that cannot estimate arrival time in serial buffer use current time
        this_frame_us = AP_HAL::micros();
    }
    float dt = (this_frame_us - last_frame_us) * 1.0e-6;
    last_frame_us = this_frame_us;

    // sanity check dt
    if (is_positive(dt) && (dt < CXOF_TIMEOUT_SEC)) {
        // calculate flow values
        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;

        // copy flow rates to state structure
        state.flowRate = Vector2f(((float)x_sum / count) * flowScaleFactorX,
                                  ((float)y_sum / count) * flowScaleFactorY);
        state.flowRate *= CXOF_PIXEL_SCALING / dt;

        // copy average body rate to state structure
        state.bodyRate = Vector2f(gyro_sum.x / gyro_sum_count, gyro_sum.y / gyro_sum_count);

        // we only apply yaw to flowRate as body rate comes from AHRS
        _applyYaw(state.flowRate);
    } else {
        // first frame received in some time so cannot calculate flow values
        state.flowRate.zero();
        state.bodyRate.zero();
    }

    _update_frontend(state);

    // reset gyro sum
    gyro_sum.zero();
    gyro_sum_count = 0;
}

#endif  // AP_OPTICALFLOW_CXOF_ENABLED
