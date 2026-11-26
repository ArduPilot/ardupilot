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
#define START_SPEED                 2
#define START_BEGIN_ANGLE           4
#define START_PAYLOAD               6
#define START_END_ANGLE             42
#define START_CHECK_SUM             46
#define MEASUREMENT_PAYLOAD_LENGTH  3
#define PAYLOAD_COUNT               12

// confidence for each measurement must be this value or higher
#define CONFIDENCE_THRESHOLD 20

// LDROBOT's LD06 filter settings (ldlidar_stl_ros2, tofbf.cpp)
#define LD06_FILTER_MAX_DIST_MM         12000   // only points closer than this are filtered (LDROBOT use 5000)
#define LD06_FILTER_POINTS_PER_SEC      4500.0f // measurement rate
#define LD06_FILTER_DIST_RATIO          0.03f   // neighbours in a group agree within 3%
#define LD06_FILTER_GROUP_SMALL         3       // groups smaller than this need high intensity
#define LD06_FILTER_GROUP_LARGE         15      // groups larger than this are always kept
#define LD06_FILTER_INTENSITY_SINGLE    220     // minimum average intensity of a small group
#define LD06_FILTER_INTENSITY_LOW       15      // groups at or below this average intensity are dropped
// groups smaller than this must be seen again on the next sweep (not part of LDROBOT's filter)
#define LD06_FILTER_CONFIRM_GROUP       5
#define LD06_FILTER_CONFIRM_DEG         4       // within this many degrees
#define LD06_FILTER_CONFIRM_MIN_MM      300     // and within 10% or this distance, whichever is larger
#define LD06_SWEEP_UNSEEN               UINT16_MAX

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

    // the previous rotation is complete when the start angle wraps around
    if (_scan_count > 0 && start_angle < _last_start_angle_deg - 180.0f) {
        process_rotation();
    }
    if (uint32_t(_scan_count) + PAYLOAD_COUNT > ARRAY_SIZE(_scan)) {
        // only happens if the sensor turns slower than 6.25Hz; filter what we have
        process_rotation();
    }
    _last_start_angle_deg = start_angle;
    const uint16_t speed_dps = UINT16_VALUE(_response[START_SPEED + 1], _response[START_SPEED]);
    if (speed_dps > 0) {
        _scan_speed_dps = speed_dps;
    }

    // Each recording point is three bytes long, goes through all of that and updates database
    for (uint16_t i = START_PAYLOAD; i < START_PAYLOAD + MEASUREMENT_PAYLOAD_LENGTH * PAYLOAD_COUNT; i += MEASUREMENT_PAYLOAD_LENGTH) {

        const float angle_deg = start_angle + angle_step * (i / MEASUREMENT_PAYLOAD_LENGTH);
        const uint16_t distance_mm = UINT16_VALUE(_response[i + 1], _response[i]);
        const uint8_t confidence = _response[i + 2];

        // buffer the point until the rotation is complete
        ScanPoint &p = _scan[_scan_count++];
        p.angle_deg = angle_deg;
        p.distance_mm = distance_mm;
        p.intensity = confidence;
        p.keep = true;
        p.small_group = false;
    }
}

// filter the buffered rotation and pass the points that survive to the sectors and faces
void AP_Proximity_LD06::process_rotation()
{
    group_filter(_scan, _scan_count, _scan_speed_dps);
    confirm_small_groups(_scan, _scan_count, _last_sweep_mm, _sweep_scratch_mm);
    for (uint16_t i = 0; i < _scan_count; i++) {
        const ScanPoint &p = _scan[i];
        if (p.keep) {
            process_reading(correct_angle_for_orientation(p.angle_deg), p.distance_mm * 0.001, p.intensity);
        }
    }
    _scan_count = 0;
}

// keep or drop the filtered points (those closer than LD06_FILTER_MAX_DIST_MM) between first and last
void AP_Proximity_LD06::set_group_keep(ScanPoint *points, uint16_t first, uint16_t last, uint16_t size, bool keep)
{
    for (uint16_t i = first; i <= last; i++) {
        if (points[i].distance_mm < LD06_FILTER_MAX_DIST_MM) {
            points[i].keep = keep;
            points[i].small_group = size < LD06_FILTER_CONFIRM_GROUP;
        }
    }
}

// true if a group of points is a real object rather than noise
static bool group_is_valid(uint16_t size, uint32_t intensity_sum)
{
    if (size > LD06_FILTER_GROUP_LARGE) {
        return true;
    }
    if (size < LD06_FILTER_GROUP_SMALL) {
        // isolated points must have a strong return
        return (intensity_sum / size) >= LD06_FILTER_INTENSITY_SINGLE;
    }
    // average intensity above LD06_FILTER_INTENSITY_LOW
    return intensity_sum > uint32_t(LD06_FILTER_INTENSITY_LOW) * size;
}

/*
  LDROBOT's LD06 near-point filter. Points closer than
  LD06_FILTER_MAX_DIST_MM are grouped (LDROBOT use 5m, here it is the
  whole range of the sensor): a new group starts when the angle to the
  previous point is more than two measurement steps or the distance
  differs by more than 3%. The first and last groups are joined if they
  meet across 0/360 degrees.
  Groups of fewer than 3 points are dropped unless their returns are
  strong, larger groups are dropped only if their returns are very weak.
  Random false returns rarely agree with their neighbours, so they mostly
  end up in groups of 1 or 2 points.
 */
void AP_Proximity_LD06::group_filter(ScanPoint *points, uint16_t count, uint16_t speed_dps)
{
    // angle between groups: twice the angle between two measurements
    const float limit_deg = speed_dps / LD06_FILTER_POINTS_PER_SEC * 2.0f;

    // the first group is classified last as it may join the last group
    bool first_done = false;
    uint16_t first_start = 0, first_end = 0, first_size = 0;
    uint32_t first_sum = 0;

    // group being built
    uint16_t grp_start = 0, grp_end = 0, grp_size = 0;
    uint32_t grp_sum = 0;

    for (uint16_t i = 0; i < count; i++) {
        ScanPoint &p = points[i];
        p.keep = true;
        p.small_group = false;
        if (p.distance_mm >= LD06_FILTER_MAX_DIST_MM) {
            // far points are not filtered and do not split groups
            continue;
        }
        if (grp_size > 0) {
            const ScanPoint &prev = points[grp_end];
            if (fabsf(p.angle_deg - prev.angle_deg) > limit_deg ||
                abs(int32_t(p.distance_mm) - int32_t(prev.distance_mm)) > prev.distance_mm * LD06_FILTER_DIST_RATIO) {
                // this point starts a new group, close the current one
                if (!first_done) {
                    first_done = true;
                    first_start = grp_start;
                    first_end = grp_end;
                    first_size = grp_size;
                    first_sum = grp_sum;
                } else {
                    set_group_keep(points, grp_start, grp_end, grp_size, group_is_valid(grp_size, grp_sum));
                }
                grp_size = 0;
                grp_sum = 0;
            }
        }
        if (grp_size == 0) {
            grp_start = i;
        }
        grp_end = i;
        grp_size++;
        grp_sum += p.intensity;
    }

    if (grp_size == 0) {
        // no points closer than LD06_FILTER_MAX_DIST_MM
        return;
    }
    if (!first_done) {
        // a single group
        set_group_keep(points, grp_start, grp_end, grp_size, group_is_valid(grp_size, grp_sum));
        return;
    }

    // join the first and last groups if they meet across 0/360 degrees
    const ScanPoint &first = points[first_start];
    const ScanPoint &last = points[grp_end];
    if (fabsf((first.angle_deg + 360.0f) - last.angle_deg) < limit_deg &&
        abs(int32_t(first.distance_mm) - int32_t(last.distance_mm)) < last.distance_mm * LD06_FILTER_DIST_RATIO) {
        const bool keep = group_is_valid(first_size + grp_size, first_sum + grp_sum);
        set_group_keep(points, first_start, first_end, first_size + grp_size, keep);
        set_group_keep(points, grp_start, grp_end, first_size + grp_size, keep);
    } else {
        set_group_keep(points, first_start, first_end, first_size, group_is_valid(first_size, first_sum));
        set_group_keep(points, grp_start, grp_end, grp_size, group_is_valid(grp_size, grp_sum));
    }
}

/*
  Outdoors the LD06 returns random short distances where there is
  nothing to see (open sky). Now and then three or four of them agree
  well enough to pass group_filter(), giving an obstacle that is there for
  one rotation only. A real object is still there on the next sweep, so
  points of small groups are kept only if the previous sweep had a kept
  point within LD06_FILTER_CONFIRM_DEG degrees and 10% (at least
  LD06_FILTER_CONFIRM_MIN_MM) of their distance. Points of larger groups
  are used at once.
 */
void AP_Proximity_LD06::confirm_small_groups(ScanPoint *points, uint16_t count, uint16_t *last_sweep_mm, uint16_t *scratch_mm)
{
    // nearest kept point per degree of this sweep, before confirmation
    for (uint16_t b = 0; b < LD06_SWEEP_BINS; b++) {
        scratch_mm[b] = LD06_SWEEP_UNSEEN;
    }
    for (uint16_t i = 0; i < count; i++) {
        const ScanPoint &p = points[i];
        const uint16_t b = uint16_t(p.angle_deg) % LD06_SWEEP_BINS;
        if (scratch_mm[b] == LD06_SWEEP_UNSEEN) {
            scratch_mm[b] = 0;
        }
        if (p.keep && p.distance_mm > 0 && (scratch_mm[b] == 0 || p.distance_mm < scratch_mm[b])) {
            scratch_mm[b] = p.distance_mm;
        }
    }

    for (uint16_t i = 0; i < count; i++) {
        ScanPoint &p = points[i];
        if (!p.keep || !p.small_group) {
            continue;
        }
        const uint16_t b = uint16_t(p.angle_deg) % LD06_SWEEP_BINS;
        const int32_t tolerance_mm = MAX(LD06_FILTER_CONFIRM_MIN_MM, p.distance_mm / 10);
        bool confirmed = false;
        for (int16_t k = -LD06_FILTER_CONFIRM_DEG; k <= LD06_FILTER_CONFIRM_DEG && !confirmed; k++) {
            const uint16_t prev_mm = last_sweep_mm[(b + LD06_SWEEP_BINS + k) % LD06_SWEEP_BINS];
            confirmed = prev_mm > 0 && abs(int32_t(p.distance_mm) - int32_t(prev_mm)) <= tolerance_mm;
        }
        p.keep = confirmed;
    }

    // this sweep is the previous one for the next rotation (only the degrees it covered)
    for (uint16_t b = 0; b < LD06_SWEEP_BINS; b++) {
        if (scratch_mm[b] != LD06_SWEEP_UNSEEN) {
            last_sweep_mm[b] = scratch_mm[b];
        }
    }
}

// check a single reading and update the 2 degree sectors and faces with it
void AP_Proximity_LD06::process_reading(float angle_deg, float distance_m, uint8_t confidence)
{
    // ignore distance that are out-of-range or have low confidence (a distance of 0 means no return)
    if (distance_m < distance_min_m() || distance_m > distance_max_m() || confidence < CONFIDENCE_THRESHOLD) {
        return;
    }

    // ignore distances that are out-of-range (based on user parameters) or within ignore areas
    if (ignore_reading(angle_deg, distance_m)) {
        return;
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
#endif // AP_PROXIMITY_LD06_ENABLED
