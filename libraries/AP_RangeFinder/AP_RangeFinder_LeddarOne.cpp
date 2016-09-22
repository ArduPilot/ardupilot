// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_LeddarOne.h"
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LeddarOne::AP_RangeFinder_LeddarOne(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}

/*
   detect if a LeddarOne rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_LeddarOne::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_LeddarOne::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // send a request message for Modbus function 4
    if (send_request() != LEDDARONE_OK) {
        // TODO: handle LEDDARONE_ERR_SERIAL_PORT
        return false;
    }

    uint32_t start_ms = AP_HAL::millis();
    while (!uart->available()) {
        // wait up to 200ms
        if (AP_HAL::millis() - start_ms > 200) {
            return false;
        }
    }

    // parse a response message, set detections and sum_distance
    // must be signed to handle errors
    int8_t number_detections = parse_response();

    if (number_detections <= 0) {
        // TODO: when (number_detections < 0) handle LEDDARONE_ERR_
        return false;
    }

    // calculate average distance
    reading_cm = sum_distance / (uint8_t)number_detections;

    return true;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LeddarOne::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

/*
   CRC16
   CRC-16-IBM(x16+x15+x2+1)
*/
bool AP_RangeFinder_LeddarOne::CRC16(uint8_t *aBuffer, uint8_t aLength, bool aCheck)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i=0; i<aLength; i++) {
        crc ^= aBuffer[i];
        for (uint32_t j=0; j<8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    uint8_t lCRCLo = LOWBYTE(crc);
    uint8_t lCRCHi = HIGHBYTE(crc);

    if (aCheck) {
        return (aBuffer[aLength] == lCRCLo) && (aBuffer[aLength+1] == lCRCHi);
    } else {
        aBuffer[aLength] = lCRCLo;
        aBuffer[aLength+1] = lCRCHi;
        return true;
    }
}

/*
   send a request message to execute ModBus function 0x04
 */
int8_t AP_RangeFinder_LeddarOne::send_request(void)
{
    uint8_t data_buffer[10] = {0};
    uint8_t i = 0;

    uint32_t nbytes = uart->available();

    // clear buffer
    while (nbytes-- > 0) {
        uart->read();
        if (++i > 250) {
            return LEDDARONE_ERR_SERIAL_PORT;
        }
    }

    // Modbus read input register (function code 0x04)
    data_buffer[0] = LEDDARONE_DEFAULT_ADDRESS;
    data_buffer[1] = 0x04;
    data_buffer[2] = 0;
    data_buffer[3] = 20;
    data_buffer[4] = 0;
    data_buffer[5] = 10;

    // CRC16
    CRC16(data_buffer, 6, false);

    // write buffer data with CRC16 bits
    for (i=0; i<8; i++) {
        uart->write(data_buffer[i]);
    }
    uart->flush();

    return LEDDARONE_OK;
}

 /*
    parse a response message from Modbus
  */
int8_t AP_RangeFinder_LeddarOne::parse_response(void)
{
    uint8_t data_buffer[25] = {0};
    uint32_t start_ms = AP_HAL::millis();
    uint32_t len = 0;
    uint8_t i;
    uint8_t index_offset = 11;

    // read serial
    while (AP_HAL::millis() - start_ms < 10) {
        uint32_t nbytes = uart->available();
        if (len == 25 && nbytes == 0) {
            break;
        } else {
            for (i=len; i<nbytes+len; i++) {
                if (i >= 25) {
                    return LEDDARONE_ERR_BAD_RESPONSE;
                }
                data_buffer[i] = uart->read();
            }
            start_ms = AP_HAL::millis();
            len += nbytes;
        }
    }

    if (len != 25) {
    	return LEDDARONE_ERR_BAD_RESPONSE;
    }

    // CRC16
    if (!CRC16(data_buffer, len-2, true)) {
        return LEDDARONE_ERR_BAD_CRC;
    }

    // number of detections
    uint8_t number_detections = data_buffer[10];

    // if the number of detection is over , it is false
    if (number_detections > LEDDARONE_DETECTIONS_MAX) {
        return LEDDARONE_ERR_NUMBER_DETECTIONS;
    }

    memset(detections, 0, sizeof(detections));
    sum_distance = 0;
    for (i=0; i<number_detections; i++) {
        // construct data word from two bytes and convert mm to cm
        detections[i] =  (((uint16_t)data_buffer[index_offset])*256 + data_buffer[index_offset+1]) / 10;
        sum_distance += detections[i];
        index_offset += 4;
    }

    return (int8_t)number_detections;
}
