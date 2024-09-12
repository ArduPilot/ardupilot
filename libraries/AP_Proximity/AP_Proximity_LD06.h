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
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM LD06 DATASHEET:
 *
 * http://wiki.inno-maker.com/display/HOMEPAGE/LD06?preview=/6949506/6949511/LDROBOT_LD06_Development%20manual_v1.0_en.pdf
 *
 * Author: Adithya Patil, Georgia Institute of Technology
 * Based on the SLAMTEC RPLiDAR code written by Steven Josefs, IAV GmbH
 *
 */

#pragma once
#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LD06_ENABLED

#include "AP_Proximity_Backend_Serial.h"

#define MESSAGE_LENGTH_LD06         47

// Minimum and maximum distance that the sensor can read in meters
#define MAX_READ_DISTANCE_LD06          12.0f
#define MIN_READ_DISTANCE_LD06           0.02f

class AP_Proximity_LD06 : public AP_Proximity_Backend_Serial
{
public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // Update the state of the sensor
    void update(void) override;

    // Get the max and min distances for the sensor being used
    float distance_max() const override { return MAX_READ_DISTANCE_LD06; }
    float distance_min() const override { return MIN_READ_DISTANCE_LD06; }

private:

    // Get and parse the sensor data
    void parse_response_data();
    void get_readings();

    // Store and keep track of the bytes being read from the sensor
    uint8_t _response[MESSAGE_LENGTH_LD06];
    bool _response_data;
    uint16_t _byte_count;

    // Store for error-tracking purposes
    uint32_t  _last_distance_received_ms;

    // Boundary to store the measurements
    AP_Proximity_Temp_Boundary _temp_boundary;
};
#endif // AP_PROXIMITY_LD06_ENABLED
