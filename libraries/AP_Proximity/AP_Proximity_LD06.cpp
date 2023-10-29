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
#define START_BEGIN_CHARACTER       0
#define START_DATA_LENGTH           1
#define START_RADAR_SPEED           2
#define START_BEGIN_ANGLE           4
#define START_PAYLOAD               6
#define START_END_ANGLE             42
#define START_CHECK_SUM             46
#define MEASUREMENT_PAYLOAD_LENGTH  3
#define PAYLOAD_COUNT               12

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

        // Gets and logs the current byte being read
        const uint8_t c = _uart->read();

        // Stores the byte in an array if the byte is a start byte or we have already read a start byte
        if (c == LD_START_CHAR || _response_data) {

            // Sets to true if a start byte has been read, default false otherwise
            _response_data = true;

            // Stores the next byte in an array
            _response[_byte_count] = c;
            _byte_count++;

            if (_byte_count == _response[START_DATA_LENGTH] + 3) {
                
                const uint32_t current_ms = AP_HAL::millis();

                // Stores the last distance taken, used to reduce number of readings taken
                if (_last_distance_received_ms != current_ms) {
                    _last_distance_received_ms =  current_ms;
                }

                // Updates the temporary boundary and passes off the completed data
                parse_response_data();
                _temp_boundary.update_3D_boundary(state.instance, frontend.boundary);
                _temp_boundary.reset();

                // Resets the bytes read and whether or not we are reading data to accept a new payload
                _byte_count = 0;
                _response_data = false;
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

    // Respective bits store the radar speed, start/end angles
    // Use bitwise operations to correctly obtain correct angles
    // Divide angles by 100 as per manual
    const float start_angle = float(UINT16_VALUE(_response[START_BEGIN_ANGLE + 1], _response[START_BEGIN_ANGLE])) * 0.01;
    const float end_angle = float(UINT16_VALUE(_response[START_END_ANGLE + 1], _response[START_END_ANGLE])) * 0.01;

    // Verify the checksum that is stored in the last element of the response array
    // Return if checksum is incorrect - i.e. bad data, bad readings, etc.
    const uint8_t check_sum = _response[START_CHECK_SUM];
    if (check_sum != crc8_generic(&_response[0], sizeof(_response) / sizeof(_response[0]) - 1, 0x4D)) {
        return;
    }

    // Calculates the angle that this point was sampled at
    float sampled_counts = 0;
    const float angle_step = (end_angle - start_angle) /  (PAYLOAD_COUNT - 1);
    float uncorrected_angle = start_angle + (end_angle - start_angle) * 0.5;

    // Handles the case that the angles read went from 360 to 0 (jumped)
    if (angle_step < 0) {
        uncorrected_angle = wrap_360(start_angle + (end_angle + 360 - start_angle) * 0.5);
    }

    // Takes the angle in the middle of the readings to be pushed to the database
    const float push_angle = correct_angle_for_orientation(uncorrected_angle);

    float distance_avg = 0.0;

    // Each recording point is three bytes long, goes through all of that and updates database
    for (uint16_t i = START_PAYLOAD; i < START_PAYLOAD + MEASUREMENT_PAYLOAD_LENGTH * PAYLOAD_COUNT; i += MEASUREMENT_PAYLOAD_LENGTH) {

        // Gets the distance recorded and converts to meters
        const float distance_meas = UINT16_VALUE(_response[i + 1], _response[i]) * 0.001;

        // Validates data and checks if it should be included
        if (distance_meas > distance_min() && distance_meas < distance_max()) {
            if (ignore_reading(push_angle, distance_meas)) {
                continue;
            }

            sampled_counts ++;
            distance_avg += distance_meas;
        }
    }

    // Convert angle to appropriate face and adds to database
    // Since angle increments are only about 3 degrees, ignore readings if there were only 1 or 2 measurements
    //    (likely outliers) recorded in the range
    if (sampled_counts > 2) {
        // Gets the average distance read
        distance_avg /= sampled_counts;

        // Pushes the average distance and angle to the obstacle avoidance database
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(push_angle);
        _temp_boundary.add_distance(face, push_angle, distance_avg);
        database_push(push_angle, distance_avg);
    }
}
#endif // AP_PROXIMITY_LD06_ENABLED
