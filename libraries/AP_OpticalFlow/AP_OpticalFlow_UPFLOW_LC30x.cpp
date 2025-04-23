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
   byte8: reserved;
   byte9: reserved;
   byte10: surface quality
   byte11: hardware version
   byte12:checksum
   byte13:footer (0x55)
 */

#include "AP_OpticalFlow_UPFLOW_LC30x.h"

#if AP_OPTICALFLOW_UPFLOW_LC30x_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <utility>
#include "AP_OpticalFlow.h"
#include <stdio.h>

#define UPFLOW_HEADER0         (uint8_t)0xFE
#define UPFLOW_HEADER1         (uint8_t)0x0A
#define UPFLOW_FOOTER         (uint8_t)0x55
#define UPFLOW_SENSOR_IIC_ADDR (uint8_t)0xDC
#define UPFLOW_PIXEL_SCALING         (1e-4)
#define UPFLOW_TIMEOUT_SEC         0.3f

extern const AP_HAL::HAL& hal;
static bool upflow_lc30x_message_sent = false;

// internal parameter configuration instructions
static const uint8_t upflow_internal_para[4] = {0x96, 0x26, 0xbc, 0x50};

// sensor parameter configuration
static const uint8_t upflow_sensor_cfg[]= {
    //Address, Data
    0x12, 0x80, 0x11, 0x30, 0x1b, 0x06, 0x6b, 0x43, 0x12, 0x20, 0x3a, 0x00, 0x15, 0x02, 0x62, 0x81, 0x08, 0xa0, 0x06, 0x68,
    0x2b, 0x20, 0x92, 0x25, 0x27, 0x97, 0x17, 0x01, 0x18, 0x79, 0x19, 0x00, 0x1a, 0xa0, 0x03, 0x00, 0x13, 0x00, 0x01, 0x13,
    0x02, 0x20, 0x87, 0x16, 0x8c, 0x01, 0x8d, 0xcc, 0x13, 0x07, 0x33, 0x10, 0x34, 0x1d, 0x35, 0x46, 0x36, 0x40, 0x37, 0xa4,
    0x38, 0x7c, 0x65, 0x46, 0x66, 0x46, 0x6e, 0x20, 0x9b, 0xa4, 0x9c, 0x7c, 0xbc, 0x0c, 0xbd, 0xa4, 0xbe, 0x7c, 0x20, 0x09,
    0x09, 0x03, 0x72, 0x2f, 0x73, 0x2f, 0x74, 0xa7, 0x75, 0x12, 0x79, 0x8d, 0x7a, 0x00, 0x7e, 0xfa, 0x70, 0x0f, 0x7c, 0x84,
    0x7d, 0xba, 0x5b, 0xc2, 0x76, 0x90, 0x7b, 0x55, 0x71, 0x46, 0x77, 0xdd, 0x13, 0x0f, 0x8a, 0x10, 0x8b, 0x20, 0x8e, 0x21,
    0x8f, 0x40, 0x94, 0x41, 0x95, 0x7e, 0x96, 0x7f, 0x97, 0xf3, 0x13, 0x07, 0x24, 0x58, 0x97, 0x48, 0x25, 0x08, 0x94, 0xb5,
    0x95, 0xc0, 0x80, 0xf4, 0x81, 0xe0, 0x82, 0x1b, 0x83, 0x37, 0x84, 0x39, 0x85, 0x58, 0x86, 0xff, 0x89, 0x15, 0x8a, 0xb8,
    0x8b, 0x99, 0x39, 0x98, 0x3f, 0x98, 0x90, 0xa0, 0x91, 0xe0, 0x40, 0x20, 0x41, 0x28, 0x42, 0x26, 0x43, 0x25, 0x44, 0x1f,
    0x45, 0x1a, 0x46, 0x16, 0x47, 0x12, 0x48, 0x0f, 0x49, 0x0d, 0x4b, 0x0b, 0x4c, 0x0a, 0x4e, 0x08, 0x4f, 0x06, 0x50, 0x06,
    0x5a, 0x56, 0x51, 0x1b, 0x52, 0x04, 0x53, 0x4a, 0x54, 0x26, 0x57, 0x75, 0x58, 0x2b, 0x5a, 0xd6, 0x51, 0x28, 0x52, 0x1e,
    0x53, 0x9e, 0x54, 0x70, 0x57, 0x50, 0x58, 0x07, 0x5c, 0x28, 0xb0, 0xe0, 0xb1, 0xc0, 0xb2, 0xb0, 0xb3, 0x4f, 0xb4, 0x63,
    0xb4, 0xe3, 0xb1, 0xf0, 0xb2, 0xa0, 0x55, 0x00, 0x56, 0x40, 0x96, 0x50, 0x9a, 0x30, 0x6a, 0x81, 0x23, 0x33, 0xa0, 0xd0,
    0xa1, 0x31, 0xa6, 0x04, 0xa2, 0x0f, 0xa3, 0x2b, 0xa4, 0x0f, 0xa5, 0x2b, 0xa7, 0x9a, 0xa8, 0x1c, 0xa9, 0x11, 0xaa, 0x16,
    0xab, 0x16, 0xac, 0x3c, 0xad, 0xf0, 0xae, 0x57, 0xc6, 0xaa, 0xd2, 0x78, 0xd0, 0xb4, 0xd1, 0x00, 0xc8, 0x10, 0xc9, 0x12,
    0xd3, 0x09, 0xd4, 0x2a, 0xee, 0x4c, 0x7e, 0xfa, 0x74, 0xa7, 0x78, 0x4e, 0x60, 0xe7, 0x61, 0xc8, 0x6d, 0x70, 0x1e, 0x39,
    0x98, 0x1a
};


// constructor
AP_OpticalFlow_UPFLOW_LC30x::AP_OpticalFlow_UPFLOW_LC30x(AP_OpticalFlow &_frontend, AP_HAL::UARTDriver *_uart) :
    OpticalFlow_backend(_frontend),
    uart(_uart)
{
}

// detect the device
AP_OpticalFlow_UPFLOW_LC30x *AP_OpticalFlow_UPFLOW_LC30x::detect(AP_OpticalFlow &_frontend)
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
    AP_OpticalFlow_UPFLOW_LC30x *sensor = NEW_NOTHROW AP_OpticalFlow_UPFLOW_LC30x(_frontend, uart);
    return sensor;
}

// initialise the sensor
void AP_OpticalFlow_UPFLOW_LC30x::init()
{
    // sanity check uart
    if (uart == nullptr) {
        return;
    }
    // open serial port with baud rate of 19200
    uart->begin(19200);
}

// read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_UPFLOW_LC30x::update(void)
{
    // sanity check uart
    if (uart == nullptr) {
        return;
    }

    // init upflow,only once
    if (!upflow_lc30x_message_sent) {
        
        uint8_t buf[30];
		uint16_t len = sizeof(upflow_sensor_cfg);
		
		buf[0] = 0xAA;
		uart->write( buf, 1);
			
		buf[0] = 0xAB;
		uart->write( buf, 1 );
		uart->write( upflow_internal_para, 4 );
		buf[0] = upflow_internal_para[0]^upflow_internal_para[1]^upflow_internal_para[2]^upflow_internal_para[3];
		uart->write( buf, 1 );		
									
		for(uint16_t i=0; i<len;i+=2 )
		{
			buf[0] = 0xBB;
			buf[1] = UPFLOW_SENSOR_IIC_ADDR;
			buf[2] = upflow_sensor_cfg[i];
			buf[3] = upflow_sensor_cfg[i+1];
			buf[4] = UPFLOW_SENSOR_IIC_ADDR^upflow_sensor_cfg[i]^upflow_sensor_cfg[i+1];
			uart->write( buf, 5 );								 
		}
		 		 
		buf[0] = 0xDD;
		uart->write( buf, 1 );	

        upflow_lc30x_message_sent = true; 
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

    state.surface_quality = updata.quality;

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

#endif  // AP_OPTICALFLOW_UPFLOW_LC30x_ENABLED
