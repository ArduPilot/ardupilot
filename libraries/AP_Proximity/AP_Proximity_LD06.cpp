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
 * ArduPilot device driver for Inno-Maker LD06 LiDAR
 *
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM InnoMaker DATASHEET:
 *
 * http://wiki.inno-maker.com/display/HOMEPAGE/LD06?preview=/6949506/6949511/LDROBOT_LD06_Development%20manual_v1.0_en.pdf
 *
 * Author: Adithya Patil, Georgia Institute of Technology
 * Based on the SLAMTEC RPLiDAR code written by Steven Josefs, IAV GmbH and CYGBOT D1 LiDAR code
 *
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LD06_ENABLED
#include "AP_Proximity_LD06.h"

#define LD_START_CHAR       0x54
#define PROXIMITY_LD06_TIMEOUT_MS 50

// Indices in data array where each value starts being recorded
// See comment below about data payload for more info about formatting
#define START_DATA_LENGTH           1
#define START_BEGIN_ANGLE           4
#define START_PAYLOAD               6
#define START_END_ANGLE             42
#define START_CHECK_SUM             46
#define MEASUREMENT_PAYLOAD_LENGTH  3
#define PAYLOAD_COUNT               12

// confidence for each measurement must be this value or higher
#define CONFIDENCE_THRESHOLD 20

 /* ------------------------------------------
    Data Packet Structure:
    Start Character : 1 Byte
    Data Length : 1 Byte
    Radar Speed : 2 Bytes
    Start Angle : 2 Bytes
    Data Measurements : 36 Bytes
        Contains 12 measurements of 3 Bytes each
        Each measurement has 2 Bytes for distance to closest object
        Each measurement has the 3rd Byte as measurement Confidence
    End Angle : 2 Bytes
    Timestamp : 2 Bytes
    Checksum : 1 Byte
    ------------------------------------------ */
// ----> 47 data bytes in total for one packet

// Update the sensor readings
void AP_Proximity_LD06::update(void)
{
    // Escape if no connection detected/supported while running
    if (_uart == nullptr) {
        return;
    }

    // Begin getting sensor readings
    // Calls method that repeatedly reads through UART channel
    get_readings();

    // Check if the data is being received correctly and sets Proximity Status
    if (_last_distance_received_ms == 0 || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_LD06_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// Called repeatedly to get the readings at the current instant
void AP_Proximity_LD06::get_readings()
{
    if (_uart == nullptr) {
        return;
    }

    // Store the number of bytes available on the UART input
    uint32_t nbytes = MIN((uint16_t) 4000,  _uart->available());

    // Loops through all bytes that were received
    while (nbytes-- > 0) {

        uint8_t c;
        if (!_uart->read(c)) {
            break;
        }

        // Stores the byte in an array if the byte is a start byte or we have already read a start byte
        if (c == LD_START_CHAR || _byte_count) {
            // Stores the next byte in an array
            _response[_byte_count] = c;
            if (_byte_count < START_DATA_LENGTH) {
                _byte_count++;
                continue;
            }

            // total_packet_length = sizeof(header) + datalength + sizeof(footer):
            const uint32_t total_packet_length = 6 + 3*(_response[START_DATA_LENGTH] & 0x1F) + 5;
            if ((_response[START_DATA_LENGTH] & 0x1F) != PAYLOAD_COUNT ||
                total_packet_length > ARRAY_SIZE(_response)) {
                // invalid packet received; throw away all data and
                // start again.
                _byte_count = 0;
                _uart->discard_input();
                break;
            }

            _byte_count++;

            if (_byte_count == total_packet_length) {

                const uint32_t current_ms = AP_HAL::millis();

                _last_distance_received_ms =  current_ms;

                // Updates the temporary boundary and passes off the completed data
                parse_response_data();

                // Resets the bytes read and whether or not we are reading data to accept a new payload
                _byte_count = 0;
            }
        }
    }
}

// Parses the data packet received from the LiDAR
void AP_Proximity_LD06::parse_response_data()
{

    // Data interpretation based on:
    // http://wiki.inno-maker.com/display/HOMEPAGE/LD06?preview=/6949506/6949511/LDROBOT_LD06_Development%20manual_v1.0_en.pdf

    // Second byte in array stores length of data - not used but stored for debugging
    // const uint8_t data_length = _response[START_DATA_LENGTH];

    // Verify the checksum that is stored in the last element of the response array
    // Return if checksum is incorrect - i.e. bad data, bad readings, etc.
    const uint8_t check_sum = _response[START_CHECK_SUM];
    if (check_sum != crc8_generic(&_response[0], sizeof(_response) / sizeof(_response[0]) - 1, 0x4D)) {
        return;
    }

    // Respective bits store the radar speed, start/end angles
    // Use bitwise operations to correctly obtain correct angles
    // Divide angles by 100 as per manual
    const float start_angle = float(UINT16_VALUE(_response[START_BEGIN_ANGLE + 1], _response[START_BEGIN_ANGLE])) * 0.01;
    const float end_angle = float(UINT16_VALUE(_response[START_END_ANGLE + 1], _response[START_END_ANGLE])) * 0.01;

    float angle_step;
    if (start_angle < end_angle) {
        angle_step = (end_angle - start_angle) / (PAYLOAD_COUNT - 1);
    } else {
        angle_step = (end_angle + 360 - start_angle) / (PAYLOAD_COUNT - 1);
    }
    // Each recording point is three bytes long, goes through all of that and updates database
    for (uint16_t i = START_PAYLOAD; i < START_PAYLOAD + MEASUREMENT_PAYLOAD_LENGTH * PAYLOAD_COUNT; i += MEASUREMENT_PAYLOAD_LENGTH) {

        // Gets the distance recorded and converts to meters
        const float angle_deg = correct_angle_for_orientation(start_angle + angle_step * (i / MEASUREMENT_PAYLOAD_LENGTH));
        const float distance_m = _dist_filt_mm.apply(UINT16_VALUE(_response[i + 1], _response[i])) * 0.001;
        const float confidence = _response[i + 2];

        // ignore distance that are out-of-range or have low confidence
        if (distance_m < distance_min_m() || distance_m > distance_max_m() || confidence < CONFIDENCE_THRESHOLD) {
            continue;
        }

        // ignore distances that are out-of-range (based on user parameters) or within ignore areas
        if (ignore_reading(angle_deg, distance_m)) {
            continue;
        }

        // use the shortest distance within 2 degree sectors
        const uint16_t a2d = (int)(angle_deg / 2.0) * 2;
        if (_angle_2deg == a2d) {
            if (distance_m < _dist_2deg_m) {
                _dist_2deg_m = distance_m;
            }
        } else {
            // new 2 degree sector, process the old one

            // check if new sector is also a new face
            const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face((float)_angle_2deg);
            if (face != _last_face) {
                // distance is for a new face, the previous one can be updated now
                if (_last_distance_valid) {
                    frontend.boundary.set_face_attributes(_last_face, _last_angle_deg, _last_distance_m, state.instance);
                } else {
                    // reset distance from last face
                    frontend.boundary.reset_face(face, state.instance);
                }

                // initialize the new face
                _last_face = face;
                _last_distance_valid = false;
            }

            // update face's shortest distance
            if (!_last_distance_valid || (_dist_2deg_m < _last_distance_m)) {
                _last_distance_m = _dist_2deg_m;
                _last_distance_valid = true;
                _last_angle_deg = (float)_angle_2deg;
            }

            // update OA database with the 2 degree sectors distance
            database_push(_angle_2deg, _dist_2deg_m);

            // advance to the next 2 degree sector
            _angle_2deg = a2d;
            _dist_2deg_m = distance_m;
        }
    }
}
#endif // AP_PROXIMITY_LD06_ENABLED
