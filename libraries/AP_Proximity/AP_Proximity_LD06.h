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

// points buffered for one rotation (about 450 at the default 10Hz)
#define LD06_ROTATION_POINTS_MAX        720

class AP_Proximity_LD06 : public AP_Proximity_Backend_Serial
{
public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // Update the state of the sensor
    void update(void) override;

    // Get the max and min distances for the sensor being used
    float distance_max_m() const override { return MAX_READ_DISTANCE_LD06; }
    float distance_min_m() const override { return MIN_READ_DISTANCE_LD06; }

    // one measurement of the current rotation
    struct ScanPoint {
        float angle_deg;        // sensor frame, not wrapped (may exceed 360 at the end of a rotation)
        uint16_t distance_mm;   // 0 means no return
        uint8_t intensity;
        bool keep;              // set by group_filter()
    };

    // LDROBOT LD06 near-point filter (ldlidar_stl_ros2 tofbf.cpp). Points
    // must be in scan order and cover at most one rotation. Sets keep on
    // every point. Has no state, so it can be tested on its own.
    static void group_filter(ScanPoint *points, uint16_t count, uint16_t speed_dps);

private:

    // Get and parse the sensor data
    void parse_response_data();
    void get_readings();

    // apply range, confidence and ignore-area checks, then update the 2 degree sectors and faces
    void process_reading(float angle_deg, float distance_m, uint8_t confidence);

    // Store and keep track of the bytes being read from the sensor
    uint8_t _response[MESSAGE_LENGTH_LD06];
    uint16_t _byte_count;

    // Store for error-tracking purposes
    uint32_t  _last_distance_received_ms;

    // filter the buffered rotation and pass the surviving points on
    void process_rotation();
    static void set_group_keep(ScanPoint *points, uint16_t first, uint16_t last, uint16_t size, bool keep);

    ScanPoint _scan[LD06_ROTATION_POINTS_MAX];  // current rotation
    uint16_t _scan_count;                       // number of points in _scan
    uint16_t _scan_speed_dps;                   // latest non-zero rotation speed
    float _last_start_angle_deg;                // start angle of the previous packet

    // face related variables
    AP_Proximity_Boundary_3D::Face _last_face;///< last face requested
    float _last_angle_deg;                    ///< yaw angle (in degrees) of _last_distance_m
    float _last_distance_m;                   ///< shortest distance for _last_face
    bool _last_distance_valid;                ///< true if _last_distance_m is valid

    // angle and distance for the latest 2 degree sector
    uint16_t _angle_2deg;
    float _dist_2deg_m;
};
#endif // AP_PROXIMITY_LD06_ENABLED
