/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
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
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_RangeFinder_NRA24.h"

//#define MAXSONAR_SERIAL_LV_BAUD_RATE 115200

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_NRA24::AP_RangeFinder_NRA24(RangeFinder &_ranger, uint8_t instance,
                                                                 RangeFinder::RangeFinder_State &_state,
                                                                 AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state, MAV_DISTANCE_SENSOR_ULTRASOUND)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}

/* 
   detect if a MaxSonar rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_NRA24::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_NRA24::reading(uint16_t &cm)
{
    const unsigned char head[] = {0xAA, 0xAA, 0x0C, 0x07};

    if (uart == nullptr) {
        return false;
    }

    while (uart->available()){
        memcpy(buff, buff + 1, sizeof(head));
        buff[sizeof(head) - 1] = uart->read();

        if (memcmp(head, buff, sizeof(head)) == 0){
            for (volatile int i = sizeof(head); i < sizeof(buff); i++){
                buff[i] = uart->read();
            }

            if (!(buff[8] == 0x2D && buff[9] == 0x02)) return false;

            makeshort sh;
            sh.ch[1] = buff[6];
            sh.ch[0] = buff[7];

            cm = sh.sh;

            uart->printf("NRA24: %3d\t %02X %02X %02X %02X %02X %02X \r\n", cm, buff[4], buff[5], buff[6], buff[7], buff[8], buff[9]);

            return true;
        }
    }

    return false;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_NRA24::update(void)
{
    if (reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 500) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
