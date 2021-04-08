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

#include "AP_Proximity_LightWareSF40C_v09.h"

#if HAL_PROXIMITY_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// update the state of the sensor
void AP_Proximity_LightWareSF40C_v09::update(void)
{
    if (_uart == nullptr) {
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
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_LightWareSF40C_v09::distance_max() const
{
    return 100.0f;
}
float AP_Proximity_LightWareSF40C_v09::distance_min() const
{
    return 0.20f;
}

// initialise sensor (returns true if sensor is successfully initialised)
bool AP_Proximity_LightWareSF40C_v09::initialise()
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

    return true;
}

// set speed of rotating motor
void AP_Proximity_LightWareSF40C_v09::set_motor_speed(bool on_off)
{
    // exit immediately if no uart
    if (_uart == nullptr) {
        return;
    }

    // set motor update speed
    if (on_off) {
        _uart->write("#MBS,3\r\n");  // send request to spin motor at 4.5hz
    } else {
        _uart->write("#MBS,0\r\n");  // send request to stop motor
    }

    // request update motor speed
    _uart->write("?MBS\r\n");
    _last_request_type = RequestType_MotorSpeed;
    _last_request_ms = AP_HAL::millis();
}

// set spin direction of motor
void AP_Proximity_LightWareSF40C_v09::set_motor_direction()
{
    // exit immediately if no uart
    if (_uart == nullptr) {
        return;
    }

    // set motor update speed
    if (frontend.get_orientation(state.instance) == 0) {
        _uart->write("#MBD,0\r\n");  // spin clockwise
    } else {
        _uart->write("#MBD,1\r\n");  // spin counter clockwise
    }

    // request update on motor direction
    _uart->write("?MBD\r\n");
    _last_request_type = RequestType_MotorDirection;
    _last_request_ms = AP_HAL::millis();
}

// set forward direction (to allow rotating lidar)
void AP_Proximity_LightWareSF40C_v09::set_forward_direction()
{
    // exit immediately if no uart
    if (_uart == nullptr) {
        return;
    }

    // set forward direction
    char request_str[15];
    int16_t yaw_corr = frontend.get_yaw_correction(state.instance);
    yaw_corr = constrain_int16(yaw_corr, -999, 999);
    snprintf(request_str, sizeof(request_str), "#MBF,%d\r\n", yaw_corr);
    _uart->write(request_str);

    // request update on motor direction
    _uart->write("?MBF\r\n");
    _last_request_type = RequestType_ForwardDirection;
    _last_request_ms = AP_HAL::millis();
}

// request new data if required
void AP_Proximity_LightWareSF40C_v09::request_new_data()
{
    if (_uart == nullptr) {
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
void AP_Proximity_LightWareSF40C_v09::send_request_for_health()
{
    if (_uart == nullptr) {
        return;
    }

    _uart->write("?GS\r\n");
    _last_request_type = RequestType_Health;
    _last_request_ms = AP_HAL::millis();
}

// send request for distance from the next sector
bool AP_Proximity_LightWareSF40C_v09::send_request_for_distance()
{
    if (_uart == nullptr) {
        return false;
    }

    // increment sector
    _last_sector++;
    if (_last_sector >= PROXIMITY_NUM_SECTORS) {
        _last_sector = 0;
    }

    // prepare request
    char request_str[16];
    snprintf(request_str, sizeof(request_str), "?TS,%u,%u\r\n",
             (unsigned int)PROXIMITY_SECTOR_WIDTH_DEG,
             boundary._sector_middle_deg[_last_sector]);
    _uart->write(request_str);


    // record request for distance
    _last_request_type = RequestType_DistanceMeasurement;
    _last_request_ms = AP_HAL::millis();

    return true;
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_LightWareSF40C_v09::check_for_reply()
{
    if (_uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    //    if CR (i.e. \r), LF (\n) it means we have received a full packet so send for processing
    //    lines starting with # are ignored because this is the echo of a set-motor request which has no reply
    //    lines starting with ? are the echo back of our distance request followed by the sensed distance
    //        distance data appears after a <space>
    //    distance data is comma separated so we put into separate elements (i.e. <space>angle,distance)
    uint16_t count = 0;
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        char c = _uart->read();
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
bool AP_Proximity_LightWareSF40C_v09::process_reply()
{
    if (_uart == nullptr) {
        return false;
    }

    bool success = false;

    switch (_last_request_type) {
        case RequestType_None:
            break;

        case RequestType_Health:
            // expect result in the form "0xhhhh"
            if (element_len[0] > 0) {
                long int result = strtol(element_buf[0], nullptr, 16);
                if (result > 0) {
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
            float angle_deg = strtof(element_buf[0], NULL);
            float distance_m = strtof(element_buf[1], NULL);
            if (!ignore_reading(angle_deg, distance_m)) {
                _last_distance_received_ms = AP_HAL::millis();
                success = true;
                // Get location on 3-D boundary based on angle to the object
                const AP_Proximity_Boundary_3D::Face face = boundary.get_face(angle_deg);
                if (is_positive(distance_m)) {
                    boundary.set_face_attributes(face, angle_deg, distance_m);
                    // update OA database
                    database_push(angle_deg, distance_m);
                } else {
                    // invalidate distance of face
                    boundary.reset_face(face);
                }
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
void AP_Proximity_LightWareSF40C_v09::clear_buffers()
{
    element_len[0] = 0;
    element_len[1] = 0;
    element_num = 0;
    memset(element_buf, 0, sizeof(element_buf));
}

#endif // HAL_PROXIMITY_ENABLED
