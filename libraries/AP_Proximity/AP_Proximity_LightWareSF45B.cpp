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

   The Lightware SF45B serial interface is described on this wiki page
   http://support.lightware.co.za/sf45/#/commands
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIGHTWARE_SF45B_ENABLED

#include "AP_Proximity_LightWareSF45B.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

static const uint32_t PROXIMITY_SF45B_TIMEOUT_MS = 200;
static const uint32_t PROXIMITY_SF45B_REINIT_INTERVAL_MS = 5000;    // re-initialise sensor after this many milliseconds
static const float PROXIMITY_SF45B_COMBINE_READINGS_DEG = 5.0f;     // combine readings from within this many degrees to improve efficiency
static const uint32_t PROXIMITY_SF45B_STREAM_DISTANCE_DATA_CM = 5;
static const uint8_t PROXIMITY_SF45B_DESIRED_UPDATE_RATE = 6;       // 1:48hz, 2:55hz, 3:64hz, 4:77hz, 5:97hz, 6:129hz, 7:194hz, 8:388hz
static const uint32_t PROXIMITY_SF45B_DESIRED_FIELDS = ((uint32_t)1 << 0 | (uint32_t)1 << 8);   // first return (unfiltered), yaw angle
static const uint16_t PROXIMITY_SF45B_DESIRED_FIELD_COUNT = 2;      // DISTANCE_DATA_CM message should contain two fields

// update the state of the sensor
void AP_Proximity_LightWareSF45B::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // initialise sensor if necessary
    initialise();

    // process incoming messages
    process_replies();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || ((AP_HAL::millis() - _last_distance_received_ms) > PROXIMITY_SF45B_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// initialise sensor
void AP_Proximity_LightWareSF45B::initialise()
{
    // check sensor is configured correctly
    _init_complete = (_sensor_state.stream_data_type == PROXIMITY_SF45B_STREAM_DISTANCE_DATA_CM) &&
                     (_sensor_state.update_rate == PROXIMITY_SF45B_DESIRED_UPDATE_RATE) &&
                     (_sensor_state.streaming_fields == PROXIMITY_SF45B_DESIRED_FIELDS);

    // exit if initialisation requests have been sent within the last few seconds
    uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_init_ms) < PROXIMITY_SF45B_REINIT_INTERVAL_MS) {
        return;
    }
    _last_init_ms = now_ms;

    // request stream rate and contents
    request_stream_start();
}

// request start of streaming of distances
void AP_Proximity_LightWareSF45B::request_stream_start()
{
    // request output rate
    send_message((uint8_t)MessageID::UPDATE_RATE, true, &PROXIMITY_SF45B_DESIRED_UPDATE_RATE, sizeof(PROXIMITY_SF45B_DESIRED_UPDATE_RATE));

    // request first return (unfiltered), and yaw angle
    send_message((uint8_t)MessageID::DISTANCE_OUTPUT, true, (const uint8_t*)&PROXIMITY_SF45B_DESIRED_FIELDS, sizeof(PROXIMITY_SF45B_DESIRED_FIELDS));

    // request start streaming of DISTANCE_DATA_CM messages
    send_message((uint8_t)MessageID::STREAM, true, (const uint8_t*)&PROXIMITY_SF45B_STREAM_DISTANCE_DATA_CM, sizeof(PROXIMITY_SF45B_STREAM_DISTANCE_DATA_CM));
}

// check for replies from sensor
void AP_Proximity_LightWareSF45B::process_replies()
{
    if (_uart == nullptr) {
        return;
    }

    // process up to 1K of characters per iteration
    uint32_t nbytes = MIN(_uart->available(), 1024U);
    while (nbytes-- > 0) {
        uint8_t c;
        if (!_uart->read(c)) {
            continue;
        }
        if (parse_byte(c)) {
            process_message();
        }
    }
}

// process the latest message held in the _msg structure
void AP_Proximity_LightWareSF45B::process_message()
{
    // process payload
    switch ((MessageID)_msg.msgid) {

    case MessageID::DISTANCE_OUTPUT:
        if (_payload_recv == sizeof(uint32_t)) {
            _sensor_state.streaming_fields = UINT32_VALUE(_msg.payload[3], _msg.payload[2], _msg.payload[1], _msg.payload[0]);
        }
        break;

    case MessageID::STREAM:
        if (_payload_recv == sizeof(uint32_t)) {
            _sensor_state.stream_data_type = UINT32_VALUE(_msg.payload[3], _msg.payload[2], _msg.payload[1], _msg.payload[0]);
        }
        break;

    case MessageID::UPDATE_RATE:
        if (_payload_recv == 1) {
            _sensor_state.update_rate = _msg.payload[0];
        }
        break;

    case MessageID::DISTANCE_DATA_CM: {
        // ignore distance messages until initialisation is complete
        if (!_init_complete || (_payload_recv != (PROXIMITY_SF45B_DESIRED_FIELD_COUNT * 2))) {
            break;
        }
        _last_distance_received_ms = AP_HAL::millis();
        const float distance_m = _distance_filt.apply((int16_t)UINT16_VALUE(_msg.payload[1], _msg.payload[0])) * 0.01f;
        const float angle_deg = correct_angle_for_orientation((int16_t)UINT16_VALUE(_msg.payload[3], _msg.payload[2]) * 0.01f);

        // if distance is from a new face then update distance, angle and boundary for previous face
        // get face from 3D boundary based on yaw angle to the object
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);
        if (face != _face) {
            if (_face_distance_valid) {
                frontend.boundary.set_face_attributes(_face, _face_yaw_deg, _face_distance, state.instance);
            } else {
                // mark previous face invalid
                frontend.boundary.reset_face(_face, state.instance);
            }
            // record updated face
            _face = face;
            _face_yaw_deg = 0;
            _face_distance = INT16_MAX;
            _face_distance_valid = false;
        }

        // if distance is from a new minisector then update obstacle database using angle and distance from previous minisector
        const uint8_t minisector = convert_angle_to_minisector(angle_deg);
        if (minisector != _minisector) {
            if ((_minisector != UINT8_MAX) && _minisector_distance_valid) {
                database_push(_minisector_angle, _minisector_distance);
            }
            // init mini sector
            _minisector = minisector;
            _minisector_angle = 0;
            _minisector_distance = INT16_MAX;
            _minisector_distance_valid = false;
        }

        // check reading is valid
        if (!ignore_reading(angle_deg, distance_m) && (distance_m >= distance_min_m()) && (distance_m <= distance_max_m())) {
            // update shortest distance for this face
            if (!_face_distance_valid || (distance_m < _face_distance)) {
                _face_yaw_deg = angle_deg;
                _face_distance = distance_m;
                _face_distance_valid = true;
            }

            // update shortest distance for this mini sector
            if (distance_m < _minisector_distance) {
                _minisector_angle = angle_deg;
                _minisector_distance = distance_m;
                _minisector_distance_valid = true;
            }
        }
        break;
    }

    default:
        // ignore unsupported messages
        break;
    }
}

// convert an angle (in degrees) to a mini sector number
uint8_t AP_Proximity_LightWareSF45B::convert_angle_to_minisector(float angle_deg) const
{
    return wrap_360(angle_deg + (PROXIMITY_SF45B_COMBINE_READINGS_DEG * 0.5f)) / PROXIMITY_SF45B_COMBINE_READINGS_DEG;
}

#endif // AP_PROXIMITY_LIGHTWARE_SF45B_ENABLED
