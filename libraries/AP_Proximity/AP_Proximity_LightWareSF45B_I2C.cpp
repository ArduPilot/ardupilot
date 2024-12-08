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

#if AP_PROXIMITY_LIGHTWARE_SF45B_I2C_ENABLED

#include "AP_Proximity_LightWareSF45B_I2C.h"
#include <utility>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/Semaphores.h>

#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Proximity_LightWareSF45B_I2C::AP_Proximity_LightWareSF45B_I2C(AP_Proximity &_frontend,
                                                              AP_Proximity::Proximity_State &_state,
                                                              AP_Proximity_Params &_params,
                                                              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_Proximity_Backend(_frontend, _state, _params), _dev(std::move(dev))
{
}

AP_Proximity_Backend* AP_Proximity_LightWareSF45B_I2C::detect(AP_Proximity &_frontend,
                                                              AP_Proximity::Proximity_State &_state,
                                                              AP_Proximity_Params &_params,
                                                              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_Proximity_LightWareSF45B_I2C *sensor = new AP_Proximity_LightWareSF45B_I2C(_frontend, _state, _params, std::move(dev));
    if (!sensor || !sensor->initialise()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

// configure the sensor and check if it is saved
bool AP_Proximity_LightWareSF45B_I2C::initialise(void)
{
    _dev->get_semaphore()->take_blocking();

    if (!configure_sensor()) {
        _dev->get_semaphore()->give();
        return false;
    }

    hal.scheduler->delay(100);

    if (!get_configuration()) {
        _dev->get_semaphore()->give();
        return false;
    }

    float reading_m, yaw;
    if (!get_reading(reading_m, yaw)) {
        _dev->get_semaphore()->give();
        return false;
    }

    _dev->get_semaphore()->give();
    _dev->register_periodic_callback(2500, FUNCTOR_BIND_MEMBER(&AP_Proximity_LightWareSF45B_I2C::timer, void));

    return true;
}

// configure sensor to send distances and servo angle only
bool AP_Proximity_LightWareSF45B_I2C::configure_sensor()
{
    struct PACKED {
        uint8_t message_id = uint8_t(MessageID::DISTANCE_OUTPUT);
        uint32_t data = PROXIMITY_SF45B_DESIRED_FIELDS;
    } cmd;

    if (!_dev->transfer((uint8_t *) &cmd, sizeof(cmd), nullptr, 0)) {
        return false;
    }

    return true;
}

// get configuration from sensor
bool AP_Proximity_LightWareSF45B_I2C::get_configuration()
{
    const uint8_t cmd = uint8_t(MessageID::DISTANCE_OUTPUT);
    if (!_dev->transfer(&cmd, sizeof(cmd), nullptr, 0)) {
        return false;
    }

    hal.scheduler->delay(100);

    uint8_t response[4];
    if (_dev->transfer(nullptr, 0, response, sizeof(response))) {
        const uint32_t configuration = le32toh_ptr(response);
        if (configuration != PROXIMITY_SF45B_DESIRED_FIELDS) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "SF45B: Unexpected configuration %08x", (unsigned int)configuration);
            return false;
        }
        return true;
    }

    return false;
}

// trigger a distance reading
bool AP_Proximity_LightWareSF45B_I2C::get_reading(float &reading_m, float &yaw_angle_deg)
{
    const uint8_t cmd = uint8_t(MessageID::DISTANCE_DATA_CM);
    _dev->transfer(&cmd, sizeof(cmd), nullptr, 0);

    struct PACKED {
        int16_t first_distance_cm;
        int16_t yaw_angle_cdeg;
    } packet;

    const bool ret = _dev->transfer(nullptr, 0, reinterpret_cast<uint8_t*>(&packet), sizeof(packet));

    if (ret) {
        reading_m = _distance_filt.apply(packet.first_distance_cm*0.01f);
        yaw_angle_deg = packet.yaw_angle_cdeg * 0.01f;
    }

    return ret;
}

// timer function
void AP_Proximity_LightWareSF45B_I2C::timer(void)
{
    float dist_m, yaw_angle_deg;
    if (get_reading(dist_m, yaw_angle_deg)) {
        WITH_SEMAPHORE(_sem);
        distance_m = dist_m;
        yaw_deg = yaw_angle_deg;
        new_distance = true;
        _last_distance_received_ms = AP_HAL::millis();
    }
}

void AP_Proximity_LightWareSF45B_I2C::update(void)
{
    WITH_SEMAPHORE(_sem);

    if (new_distance) {
        process_message(distance_m, yaw_deg);
        new_distance = false;
    }

    if ((_last_distance_received_ms == 0) || ((AP_HAL::millis() - _last_distance_received_ms) > PROXIMITY_SF45B_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

void AP_Proximity_LightWareSF45B_I2C::process_message(float dist_m, float angle)
{
    _last_distance_received_ms = AP_HAL::millis();
    const float angle_deg = correct_angle_for_orientation(angle);
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

    if (!ignore_reading(angle_deg, dist_m) && (dist_m >= distance_min()) && (dist_m <= distance_max())) {
        if (!_face_distance_valid || (dist_m < _face_distance)) {
            _face_yaw_deg = angle_deg;
            _face_distance = dist_m;
            _face_distance_valid = true;
        }

        if (dist_m < _minisector_distance) {
            _minisector_angle = angle_deg;
            _minisector_distance = dist_m;
            _minisector_distance_valid = true;
        }
    }
}

#endif // AP_PROXIMITY_LIGHTWARE_SF45B_I2C_ENABLED
