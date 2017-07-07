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
#include "AP_Proximity_TeraRangerTower.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_TeraRangerTower::AP_Proximity_TeraRangerTower(AP_Proximity &_frontend,
                                                         AP_Proximity::Proximity_State &_state,
                                                         AP_SerialManager &serial_manager) :
    AP_Proximity_Backend(_frontend, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0));
    }
}

// detect if a TeraRanger Tower proximity sensor is connected by looking for a configured serial port
bool AP_Proximity_TeraRangerTower::detect(AP_SerialManager &serial_manager)
{
    AP_HAL::UARTDriver *uart = nullptr;
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);
    return uart != nullptr;
}

// update the state of the sensor
void AP_Proximity_TeraRangerTower::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // process incoming messages
    read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_TRTOWER_TIMEOUT_MS)) {
        set_status(AP_Proximity::Proximity_NoData);
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_TeraRangerTower::distance_max() const
{
    return 4.5f;
}
float AP_Proximity_TeraRangerTower::distance_min() const
{
    return 0.20f;
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_TeraRangerTower::read_sensor_data()
{
    if (uart == nullptr) {
        return false;
    }

    uint16_t message_count = 0;
    int16_t nbytes = uart->available();

    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == 'T' ) {
            buffer_count = 0;
        }

        buffer[buffer_count++] = c;

        // we should always read 19 bytes THxxxxxxxxxxxxxxxxC
        if (buffer_count >= 19){
            buffer_count = 0;

            // check if message has right CRC
            if (crc_crc8(buffer, 18) == buffer[18]){
                uint16_t d1 = process_distance(buffer[2], buffer[3]);
                uint16_t d2 = process_distance(buffer[4], buffer[5]);
                uint16_t d3 = process_distance(buffer[6], buffer[7]);
                uint16_t d4 = process_distance(buffer[8], buffer[9]);
                uint16_t d5 = process_distance(buffer[10], buffer[11]);
                uint16_t d6 = process_distance(buffer[12], buffer[13]);
                uint16_t d7 = process_distance(buffer[14], buffer[15]);
                uint16_t d8 = process_distance(buffer[16], buffer[17]);

                update_sector_data(0, d1);
                update_sector_data(45, d8);
                update_sector_data(90, d7);
                update_sector_data(135, d6);
                update_sector_data(180, d5);
                update_sector_data(225, d4);
                update_sector_data(270, d3);
                update_sector_data(315, d2);

                message_count++;
            }
        }
    }
    return (message_count > 0);
}

uint16_t AP_Proximity_TeraRangerTower::process_distance(uint8_t buf1, uint8_t buf2)
{
    return (buf1 << 8) + buf2;
}

// process reply
void AP_Proximity_TeraRangerTower::update_sector_data(int16_t angle_deg, uint16_t distance_cm)
{
    uint8_t sector;
    if (convert_angle_to_sector(angle_deg, sector)) {
        _angle[sector] = angle_deg;
        _distance[sector] = ((float) distance_cm) / 1000;
        _distance_valid[sector] = distance_cm != 0xffff;
        _last_distance_received_ms = AP_HAL::millis();
        // update boundary used for avoidance
        update_boundary_for_sector(sector);
    }
}
