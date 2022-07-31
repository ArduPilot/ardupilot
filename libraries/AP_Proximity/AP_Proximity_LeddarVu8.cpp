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

#include "AP_Proximity_LeddarVu8.h"
#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#if HAL_PROXIMITY_ENABLED && AP_PROXIMITY_LEDDARVU8_ENABLED

// update the state of the sensor
void AP_Proximity_LeddarVu8::update()
{
    if (!_initialized) {
        request_distances();
        // gcs().send_text(MAV_SEVERITY_WARNING,"requested distances");
        _temp_boundary.reset();
        _initialized = true;
        last_distance_ms = AP_HAL::millis();
    }

    if ((AP_HAL::millis() - last_distance_ms) < LEDDARVU8_TIMEOUT_MS) {
        // just initialized
        set_status(AP_Proximity::Status::Good);
    } else {
        set_status(AP_Proximity::Status::NoData);
        // gcs().send_text(MAV_SEVERITY_WARNING,"no data");
    }

    // read data
    read_sensor_data();
    // gcs().send_text(MAV_SEVERITY_WARNING,"reading data");

    if (AP_HAL::millis() - last_distance_ms < LEDDARVU8_TIMEOUT_MS) {
        set_status(AP_Proximity::Status::Good);
        // gcs().send_text(MAV_SEVERITY_WARNING,"good data");
    } else {
        // long time since we received any valid sensor data
        // try sending the sensor the "send data" message
        _initialized = false;
        set_status(AP_Proximity::Status::NoData);
        // gcs().send_text(MAV_SEVERITY_WARNING,"no data in the else");
    }
}
// extern const AP_HAL::HAL& hal;

// LeddarVu8 uses the modbus RTU protocol
// https://autonomoustuff.com/product/leddartech-vu8/
// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
void AP_Proximity_LeddarVu8::read_sensor_data()
{
    if (_uart == nullptr) {
        return;
    }

    // check for timeout receiving messages
    uint32_t now_ms = AP_HAL::millis();
    if (((now_ms - last_distance_ms) > LEDDARVU8_TIMEOUT_MS) && ((now_ms - last_distance_request_ms) > LEDDARVU8_TIMEOUT_MS)) {
        request_distances();
    }

    // read any available characters from the lidar
  const uint16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        const uint8_t byte = _uart->read();
        if (r < 0) {
            continue;
        }
        bool valid_reading;
        uint16_t dist_cm = 0;
        if (parse_byte((uint8_t)r, valid_reading, dist_cm)) {
            // reset 
            // reset();
            last_distance_ms = AP_HAL::millis();
            // _temp_boundary.update_3D_boundary(boundary);
            // gcs().send_text(MAV_SEVERITY_WARNING,"distance read:%d" , dist_cm);
        }
        
        
    }
    
}

// TODO: Get sensor address, currently using default hardcoded
// get sensor address from RNGFNDx_ADDR parameter
/*uint8_t AP_Proximity_LeddarVu8::get_sensor_address() const
{
    if (params.address == 0) {
        return LEDDARVU8_ADDR_DEFAULT;
    }
    return params.address;
}*/

// send request to device to provide distances
void AP_Proximity_LeddarVu8::request_distances()
{
    uint8_t req_buf[] = {
            LEDDARVU8_ADDR_DEFAULT,                       // address
            (uint8_t)FunctionCode::READ_INPUT_REGISTER, // function code low
            0,                                          // function code high
            (uint8_t)RegisterNumber::FIRST_DISTANCE0,   // register number low
            0,                                          // register number high
            8,                                          // register count
            0,                                          // crc low
            0                                           // crc high
    };
    const uint8_t req_buf_len = sizeof(req_buf);

    // fill in crc bytes
    uint16_t crc = calc_crc_modbus(req_buf, req_buf_len - 2);
    req_buf[req_buf_len - 2] = LOWBYTE(crc);
    req_buf[req_buf_len - 1] = HIGHBYTE(crc);

    // send request to device
    _uart->write(req_buf, req_buf_len);

    // record time of request
    last_distance_request_ms = AP_HAL::millis();
    
}

// process one byte received on serial port
// returns true if successfully parsed a message
// if distances are valid, valid_readings is set to true and distance is stored in reading_cm
bool AP_Proximity_LeddarVu8::parse_byte(uint8_t b, bool &valid_reading, uint16_t &reading_cm)
{
    // process byte depending upon current state
    switch (parsed_msg.state) {

    case ParseState::WAITING_FOR_ADDRESS: {
        if (b == LEDDARVU8_ADDR_DEFAULT) {
            parsed_msg.address = b;
            parsed_msg.state = ParseState::WAITING_FOR_FUNCTION_CODE;
        }
        break;
    }

    case ParseState::WAITING_FOR_FUNCTION_CODE:
        if (b == (uint8_t)FunctionCode::READ_INPUT_REGISTER) {
            parsed_msg.function_code = b;
            parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD_LEN;
        } else {
            parsed_msg.state = ParseState::WAITING_FOR_ADDRESS;
        }
        break;

    case ParseState::WAITING_FOR_PAYLOAD_LEN:
        // only parse messages of the expected length
        if (b == LEDDARVU8_PAYLOAD_LENGTH) {
            parsed_msg.payload_len = b;
            parsed_msg.payload_recv = 0;
            parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD;
        } else {
            parsed_msg.state = ParseState::WAITING_FOR_ADDRESS;
        }
        break;

    case ParseState::WAITING_FOR_PAYLOAD:
        if (parsed_msg.payload_recv < parsed_msg.payload_len) {
            if (parsed_msg.payload_recv < ARRAY_SIZE(parsed_msg.payload)) {
                parsed_msg.payload[parsed_msg.payload_recv] = b;
            }
            parsed_msg.payload_recv++;
        }
        if (parsed_msg.payload_recv == parsed_msg.payload_len) {
            parsed_msg.state = ParseState::WAITING_FOR_CRC_LOW;
        }
        break;

    case ParseState::WAITING_FOR_CRC_LOW:
        parsed_msg.crc = b;
        parsed_msg.state = ParseState::WAITING_FOR_CRC_HIGH;
        break;

    case ParseState::WAITING_FOR_CRC_HIGH: {
        parsed_msg.crc |= ((uint16_t)b << 8);
        parsed_msg.state = ParseState::WAITING_FOR_ADDRESS;

        // check crc
        uint16_t expected_crc = calc_crc_modbus(&parsed_msg.address, 3+parsed_msg.payload_recv);
        if (expected_crc == parsed_msg.crc) {
            valid_reading = true;
            // parse payload, pick out distances, and send to correct faces
            // only works for 48 deg FOV hardware, mounted with the laser on top
            // reading left to right or channel 8 to 1
            valid_reading = false;
            // current horizontal angle in the payload
            float sampled_angle = LEDDARVU8_START_ANGLE;
            // gcs().send_text(MAV_SEVERITY_WARNING,"READING DATA, or so anthony thinks");
            for (uint8_t i=0; i<8; i++) {
                uint8_t ix2 = i*2;
                const uint16_t dist_cm = (uint16_t)parsed_msg.payload[ix2] << 8 | (uint16_t)parsed_msg.payload[ix2+1];
                float dist_m = dist_cm * 0.01f;
                // gcs().send_text(MAV_SEVERITY_WARNING,"valid reading www %d %f", dist_cm, sampled_angle);
                if ((dist_m > distance_min()) && (!valid_reading || dist_m < distance_max())) {
                    if (ignore_reading(sampled_angle, dist_m)) {
                        // ignore this angle
                        sampled_angle += LEDDARVU8_ANGLE_STEP;
                        continue;
                    }
                    // convert angle to face
                    const AP_Proximity_Boundary_3D::Face face = boundary.get_face(sampled_angle);
                    // set face attributes
                    boundary.set_face_attributes(face, sampled_angle, dist_m);
                    // push face to temp boundary
                    _temp_boundary.add_distance(face, sampled_angle, dist_m);
                    // push to OA_DB
                    database_push(sampled_angle, dist_m);
                    
                    reading_cm = dist_cm;
                    valid_reading = true;
                }
                // increment sampled angle
                sampled_angle += LEDDARVU8_ANGLE_STEP;
            }
            _temp_boundary.update_3D_boundary(boundary);
            reset();
            return true;
        }

        break;
    }
    }

    valid_reading = false;
    return false;
}

// reset all variables and flags
void AP_Proximity_LeddarVu8::reset()
{
    _temp_boundary.reset();
}

#endif
