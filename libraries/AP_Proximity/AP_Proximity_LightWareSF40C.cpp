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
#include "AP_Proximity_LightWareSF40C.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_LightWareSF40C::AP_Proximity_LightWareSF40C(AP_Proximity &_frontend,
                                                         AP_Proximity::Proximity_State &_state,
                                                         AP_SerialManager &serial_manager) :
    AP_Proximity_Backend(_frontend, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0));
    }
}

// detect if a Lightware proximity sensor is connected by looking for a configured serial port
bool AP_Proximity_LightWareSF40C::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0) != nullptr;
}

// update the state of the sensor
void AP_Proximity_LightWareSF40C::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // initialise sensor if necessary
    bool initialised = initialise();

    // process incoming messages
    check_for_reply();

    // request new data from sensor
    if (initialised) {
        request_new_data();
    }

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_SF40C_TIMEOUT_MS)) {
        set_status(AP_Proximity::Proximity_NoData);
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_LightWareSF40C::distance_max() const
{
    return 100.0f;
}
float AP_Proximity_LightWareSF40C::distance_min() const
{
    return 0.20f;
}

// initialise sensor (returns true if sensor is succesfully initialised)
bool AP_Proximity_LightWareSF40C::initialise()
{
    // set motor direction once per second
    if (_motor_direction > 1) {
        if ((_last_request_ms == 0) || AP_HAL::millis() - _last_request_ms > 1000) {
            set_motor_direction();
        }
    }
    // set forward direction once per second
    if (_forward_direction != frontend.get_yaw_correction(state.instance)) {
        if ((_last_request_ms == 0) || AP_HAL::millis() - _last_request_ms > 1000) {
            set_forward_direction();
        }
    }
    // request motors turn on once per second
    if (_motor_speed == 0) {
        if ((_last_request_ms == 0) || AP_HAL::millis() - _last_request_ms > 1000) {
            set_motor_speed(true);
        }
        return false;
    }
    // initialise sectors
    if (!_sector_initialised) {
        init_sectors();
        return false;
    }
    return true;
}

// initialise sector angles using user defined ignore areas
void AP_Proximity_LightWareSF40C::init_sectors()
{
    // use defaults if no ignore areas defined
    uint8_t ignore_area_count = get_ignore_area_count();
    if (ignore_area_count == 0) {
        _sector_initialised = true;
        return;
    }

    uint8_t sector = 0;

    for (uint8_t i=0; i<ignore_area_count; i++) {

        // get ignore area info
        uint16_t ign_area_angle;
        uint8_t ign_area_width;
        if (get_ignore_area(i, ign_area_angle, ign_area_width)) {

            // calculate how many degrees of space we have between this end of this ignore area and the start of the end
            int16_t start_angle, end_angle;
            get_next_ignore_start_or_end(1, ign_area_angle, start_angle);
            get_next_ignore_start_or_end(0, start_angle, end_angle);
            int16_t degrees_to_fill = wrap_360(end_angle - start_angle);

            // divide up the area into sectors
            while ((degrees_to_fill > 0) && (sector < PROXIMITY_SECTORS_MAX)) {
                uint16_t sector_size;
                if (degrees_to_fill >= 90) {
                    // set sector to maximum of 45 degrees
                    sector_size = 45;
                } else if (degrees_to_fill > 45) {
                    // use half the remaining area to optimise size of this sector and the next
                    sector_size = degrees_to_fill / 2.0f;
                } else  {
                    // 45 degrees or less are left so put it all into the next sector
                    sector_size = degrees_to_fill;
                }
                // record the sector middle and width
                _sector_middle_deg[sector] = wrap_360(start_angle + sector_size / 2.0f);
                _sector_width_deg[sector] = sector_size;

                // move onto next sector
                start_angle += sector_size;
                sector++;
                degrees_to_fill -= sector_size;
            }
        }
    }

    // set num sectors
    _num_sectors = sector;

    // re-initialise boundary because sector locations have changed
    init_boundary();

    // record success
    _sector_initialised = true;
}

// set speed of rotating motor
void AP_Proximity_LightWareSF40C::set_motor_speed(bool on_off)
{
    // exit immediately if no uart
    if (uart == nullptr) {
        return;
    }

    // set motor update speed
    if (on_off) {
        uart->write("#MBS,3\r\n");  // send request to spin motor at 4.5hz
    } else {
        uart->write("#MBS,0\r\n");  // send request to stop motor
    }

    // request update motor speed
    uart->write("?MBS\r\n");
    _last_request_type = RequestType_MotorSpeed;
    _last_request_ms = AP_HAL::millis();
}

// set spin direction of motor
void AP_Proximity_LightWareSF40C::set_motor_direction()
{
    // exit immediately if no uart
    if (uart == nullptr) {
        return;
    }

    // set motor update speed
    if (frontend.get_orientation(state.instance) == 0) {
        uart->write("#MBD,0\r\n");  // spin clockwise
    } else {
        uart->write("#MBD,1\r\n");  // spin counter clockwise
    }

    // request update on motor direction
    uart->write("?MBD\r\n");
    _last_request_type = RequestType_MotorDirection;
    _last_request_ms = AP_HAL::millis();
}

// set forward direction (to allow rotating lidar)
void AP_Proximity_LightWareSF40C::set_forward_direction()
{
    // exit immediately if no uart
    if (uart == nullptr) {
        return;
    }

    // set forward direction
    char request_str[15];
    int16_t yaw_corr = frontend.get_yaw_correction(state.instance);
    snprintf(request_str, sizeof(request_str), "#MBF,%d\r\n", (int)yaw_corr);
    uart->write(request_str);

    // request update on motor direction
    uart->write("?MBF\r\n");
    _last_request_type = RequestType_ForwardDirection;
    _last_request_ms = AP_HAL::millis();
}

// request new data if required
void AP_Proximity_LightWareSF40C::request_new_data()
{
    if (uart == nullptr) {
        return;
    }

    // after timeout assume no reply will ever come
    uint32_t now = AP_HAL::millis();
    if ((_last_request_type != RequestType_None) && ((now - _last_request_ms) > PROXIMITY_SF40C_TIMEOUT_MS)) {
        _last_request_type = RequestType_None;
        _last_request_ms = 0;
    }

    // if we are not waiting for a reply, ask for something
    if (_last_request_type == RequestType_None) {
        _request_count++;
        if (_request_count >= 5) {
            send_request_for_health();
            _request_count = 0;
        } else {
            // request new distance measurement
            send_request_for_distance();
        }
        _last_request_ms = now;
    }
}

// send request for sensor health
void AP_Proximity_LightWareSF40C::send_request_for_health()
{
    if (uart == nullptr) {
        return;
    }

    uart->write("?GS\r\n");
    _last_request_type = RequestType_Health;
    _last_request_ms = AP_HAL::millis();
}

// send request for distance from the next sector
bool AP_Proximity_LightWareSF40C::send_request_for_distance()
{
    if (uart == nullptr) {
        return false;
    }

    // increment sector
    _last_sector++;
    if (_last_sector >= _num_sectors) {
        _last_sector = 0;
    }

    // prepare request
    char request_str[15];
    snprintf(request_str, sizeof(request_str), "?TS,%d,%d\r\n", (int)(_sector_width_deg[_last_sector]), (int)(_sector_middle_deg[_last_sector]));
    uart->write(request_str);


    // record request for distance
    _last_request_type = RequestType_DistanceMeasurement;
    _last_request_ms = AP_HAL::millis();

    return true;
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_LightWareSF40C::check_for_reply()
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    //    if CR (i.e. \r), LF (\n) it means we have received a full packet so send for processing
    //    lines starting with # are ignored because this is the echo of a set-motor request which has no reply
    //    lines starting with ? are the echo back of our distance request followed by the sensed distance
    //        distance data appears after a <space>
    //    distance data is comma separated so we put into separate elements (i.e. <space>angle,distance)
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        // check for end of packet
        if (c == '\r' || c == '\n') {
            if ((element_len[0] > 0)) {
                if (process_reply()) {
                    count++;
                }
            }
            // clear buffers after processing
            clear_buffers();
            ignore_reply = false;
            wait_for_space = false;

        // if message starts with # ignore it
        } else if (c == '#' || ignore_reply) {
            ignore_reply = true;

        // if waiting for <space>
        } else if (c == '?') {
            wait_for_space = true;

        } else if (wait_for_space) {
            if (c == ' ') {
                wait_for_space = false;
            }

        // if comma, move onto filling in 2nd element
        } else if (c == ',') {
            if ((element_num == 0) && (element_len[0] > 0)) {
                element_num++;
            } else {
                // don't support 3rd element so clear buffers
                clear_buffers();
                ignore_reply = true;
            }

        // if part of a number, add to element buffer
        } else if (isdigit(c) || c == '.' || c == '-') {
            element_buf[element_num][element_len[element_num]] = c;
            element_len[element_num]++;
            if (element_len[element_num] >= sizeof(element_buf[element_num])-1) {
                // too long, discard the line
                clear_buffers();
                ignore_reply = true;
            }
        }
    }

    return (count > 0);
}

// process reply
bool AP_Proximity_LightWareSF40C::process_reply()
{
    if (uart == nullptr) {
        return false;
    }

    bool success = false;

    switch (_last_request_type) {
        case RequestType_None:
            break;

        case RequestType_Health:
            // expect result in the form "0xhhhh"
            if (element_len[0] > 0) {
                int result;
                if (sscanf(element_buf[0], "%x", &result) > 0) {
                    _sensor_status.value = result;
                    success = true;
                }
            }
            break;

        case RequestType_MotorSpeed:
            _motor_speed = atoi(element_buf[0]);
            success = true;
            break;

        case RequestType_MotorDirection:
            _motor_direction = atoi(element_buf[0]);
            success = true;
            break;

        case RequestType_ForwardDirection:
            _forward_direction = atoi(element_buf[0]);
            success = true;
            break;

        case RequestType_DistanceMeasurement:
        {
            float angle_deg = (float)atof(element_buf[0]);
            float distance_m = (float)atof(element_buf[1]);
            uint8_t sector;
            if (convert_angle_to_sector(angle_deg, sector)) {
                _angle[sector] = angle_deg;
                _distance[sector] = distance_m;
                _distance_valid[sector] = true;
                _last_distance_received_ms = AP_HAL::millis();
                success = true;
                // update boundary used for avoidance
                update_boundary_for_sector(sector);
            }
            break;
        }

        default:
            break;
    }

    // mark request as cleared
    if (success) {
        _last_request_type = RequestType_None;
    }

    return success;
}

// clear buffers ahead of processing next message
void AP_Proximity_LightWareSF40C::clear_buffers()
{
    element_len[0] = 0;
    element_len[1] = 0;
    element_num = 0;
    memset(element_buf, 0, sizeof(element_buf));
}
