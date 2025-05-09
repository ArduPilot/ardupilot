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
   driver for UPixel UPFLOW optical flow sensor

   UPFLOW serial packet description
   byte0: header0 (0xFE)
   byte1: header1 (0x0A)
   byte2: x-motion low byte;
   byte3: x-motion high byte;
   byte4: y-motion low byte;
   byte5: y-motion high byte;
   byte6: dt low byte;
   byte7: dt high byte;
   byte8: tof low byte;
   byte9: tof high byte;
   byte10: opt quality
   byte11: tof quality
   byte12: checksum
   byte13: footer (0x55)
 */

#include "AP_OpticalFlow_UPFLOW_Tx.h"

#include <AP_Common/AP_UpFlow_TOF.h>

#if AP_OPTICALFLOW_UPFLOW_Tx_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <utility>
#include "AP_OpticalFlow.h"
#include <stdio.h>
//#include <GCS_MAVLink/GCS.h>

#define UPFLOW_HEADER0         (uint8_t)0xFE
#define UPFLOW_HEADER1         (uint8_t)0x0A
#define UPFLOW_FOOTER         (uint8_t)0x55
#define UPFLOW_PIXEL_SCALING         (1e-4)
#define UPFLOW_TIMEOUT_SEC         0.3f

extern const AP_HAL::HAL& hal;

static const char* init_msg = "<set protocol upixels>";
static bool upflow_tx_message_sent = false;



// constructor
AP_OpticalFlow_UPFLOW_Tx::AP_OpticalFlow_UPFLOW_Tx(AP_OpticalFlow &_frontend, AP_HAL::UARTDriver *_uart) :
    OpticalFlow_backend(_frontend),
    uart(_uart)
{
}

// detect the device
AP_OpticalFlow_UPFLOW_Tx *AP_OpticalFlow_UPFLOW_Tx::detect(AP_OpticalFlow &_frontend)
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
    AP_OpticalFlow_UPFLOW_Tx *sensor = new AP_OpticalFlow_UPFLOW_Tx(_frontend, uart);
    return sensor;
}

// initialise the sensor
void AP_OpticalFlow_UPFLOW_Tx::init()
{
    // sanity check uart
    if (uart == nullptr) {
        return;
    }
    // open serial port with baud rate of 115200
    uart->begin(115200);
}

// read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_UPFLOW_Tx::update(void)
{
    // sanity check uart
    if (uart == nullptr) {
        return;
    }

    static UPFLOW_TOF UPFLOW_TOF_DATA = get_upflow_tof();

    // setup upflow link, only once
    if (!upflow_tx_message_sent) {
        //hal.scheduler->delay(200);
        uart->write((const uint8_t*)init_msg, strlen(init_msg));
        upflow_tx_message_sent = true; 
    }

    // record gyro values as long as they are being used
    // the sanity check of dt below ensures old gyro values are not used
    if (gyro_sum_count >= 1000) {
        gyro_sum.zero();
        gyro_sum_count = 0;
    }
    const Vector3f& gyro = AP::ahrs().get_gyro();
    gyro_sum.x += gyro.x;
    gyro_sum.y += gyro.y;
    gyro_sum_count++;

    bool phrased = false;
    // read any available characters in the serial buffer
    uint32_t nbytes = MIN(uart->available(), 1024u);
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            break;
        }
        uint8_t c = (uint8_t)r;
        if (recv_count == 0) {
            //Header0
            if (c == UPFLOW_HEADER0) {
                recv_count++;
                sum = 0;
            }
        } else if (recv_count == 1) {
            //Header1
            if (c != UPFLOW_HEADER1) {
                recv_count = 0;
            } else {
                recv_count++;
            }
        } else if (recv_count < 12) {
            //actual data
            ((uint8_t*)&updata)[recv_count - 2] = c;
            sum ^= c;
            recv_count++;
        } else if (recv_count == 12) {
            //checksum
            if (sum != c) {
                recv_count = 0;
            } else {
                recv_count++;
            }
        } else {	//footer
            if (c == UPFLOW_FOOTER) {
                phrased=true;
            }
            recv_count = 0;
        }
    }

    // return without updating state if no readings
    if (phrased == false) {
        return;
    }

    struct AP_OpticalFlow::OpticalFlow_state state {};

    state.surface_quality = updata.opt_valid;

    float dt = updata.integration_timespan * 1.0e-6;

    // sanity check dt
    if (is_positive(dt) && (dt < UPFLOW_TIMEOUT_SEC)) {
        // calculate flow values
        const Vector2f flowScaler = _flowScaler();
        Vector2f flowScaleFactor = Vector2f(1.0f, 1.0f) +  flowScaler * 0.001f;

        // copy flow rates to state structure
        state.flowRate = Vector2f((float)(-updata.flow_x_integral) * flowScaleFactor.x,
                                  (float)(-updata.flow_y_integral) * flowScaleFactor.y);
        state.flowRate *= UPFLOW_PIXEL_SCALING / dt;

        // copy average body rate to state structure
        state.bodyRate = Vector2f{gyro_sum.x / gyro_sum_count, gyro_sum.y / gyro_sum_count};

        // we only apply yaw to flowRate as body rate comes from AHRS
        _applyYaw(state.flowRate);

        //update tof
        UPFLOW_TOF_DATA.if_opt_ok = true;
        UPFLOW_TOF_DATA.ground_distance = updata.ground_distance;
        UPFLOW_TOF_DATA.tof_valid = updata.tof_valid;

        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UPFLOW_Tx: OK");

    } else {
        // first frame received in some time so cannot calculate flow values
        state.flowRate.zero();
        state.bodyRate.zero();

        //update tof
        UPFLOW_TOF_DATA.if_opt_ok = false;
        UPFLOW_TOF_DATA.ground_distance = 0;
        UPFLOW_TOF_DATA.tof_valid = 0;
    }

    // update the rangefinder state
    set_upflow_tof(UPFLOW_TOF_DATA);

    _update_frontend(state);

    // reset gyro sum
    gyro_sum.zero();
    gyro_sum_count = 0;
}

#endif  // AP_OPTICALFLOW_UPFLOW_Tx_ENABLED
