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

//
//  UAVCAN GPS driver
//
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN
#include "AP_GPS_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Common/Semaphore.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>

extern const AP_HAL::HAL& hal;

#define debug_gps_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { printf(fmt, ##args); }} while (0)

UC_REGISTRY_BINDER(FixCb, uavcan::equipment::gnss::Fix);
UC_REGISTRY_BINDER(AuxCb, uavcan::equipment::gnss::Auxiliary);

AP_GPS_UAVCAN::DetectedModules AP_GPS_UAVCAN::_detected_modules[] = {0};
HAL_Semaphore AP_GPS_UAVCAN::_sem_registry;

// Member Methods
AP_GPS_UAVCAN::AP_GPS_UAVCAN(AP_GPS &_gps, AP_GPS::GPS_State &_state) :
    AP_GPS_Backend(_gps, _state, nullptr)
{}

AP_GPS_UAVCAN::~AP_GPS_UAVCAN()
{
    if (take_registry()) {
        _detected_modules[_detected_module].driver = nullptr;
        give_registry();
    }
}

void AP_GPS_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::gnss::Fix, FixCb> *gnss_fix;
    gnss_fix = new uavcan::Subscriber<uavcan::equipment::gnss::Fix, FixCb>(*node);
    const int gnss_fix_start_res = gnss_fix->start(FixCb(ap_uavcan, &handle_fix_msg_trampoline));
    if (gnss_fix_start_res < 0) {
        AP_HAL::panic("UAVCAN GNSS subscriber start problem\n\r");
        return;
    }

    uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary, AuxCb> *gnss_aux;
    gnss_aux = new uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary, AuxCb>(*node);
    const int gnss_aux_start_res = gnss_aux->start(AuxCb(ap_uavcan, &handle_aux_msg_trampoline));
    if (gnss_aux_start_res < 0) {
        AP_HAL::panic("UAVCAN GNSS subscriber start problem\n\r");
        return;
    }
}

bool AP_GPS_UAVCAN::take_registry()
{
    return _sem_registry.take(HAL_SEMAPHORE_BLOCK_FOREVER);
}

void AP_GPS_UAVCAN::give_registry()
{
    _sem_registry.give();
}

AP_GPS_Backend* AP_GPS_UAVCAN::probe(AP_GPS &_gps, AP_GPS::GPS_State &_state)
{
    if (!take_registry()) {
        return nullptr;
    }
    AP_GPS_UAVCAN* backend = nullptr;
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            backend = new AP_GPS_UAVCAN(_gps, _state);
            if (backend == nullptr) {
                debug_gps_uavcan(2,
                                 _detected_modules[i].ap_uavcan->get_driver_index(),
                                 "Failed to register UAVCAN GPS Node %d on Bus %d\n",
                                 _detected_modules[i].node_id,
                                 _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                backend->_detected_module = i;
                debug_gps_uavcan(2,
                                 _detected_modules[i].ap_uavcan->get_driver_index(),
                                 "Registered UAVCAN GPS Node %d on Bus %d\n",
                                 _detected_modules[i].node_id,
                                 _detected_modules[i].ap_uavcan->get_driver_index());
            }
            break;
        }
    }
    give_registry();
    return backend;
}

AP_GPS_UAVCAN* AP_GPS_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan && 
            _detected_modules[i].node_id == node_id) {
            return _detected_modules[i].driver;
        }
    }

    bool already_detected = false;
    // Check if there's an empty spot for possible registeration
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
            // Already Detected
            already_detected = true;
            break;
        }
    }
    if (!already_detected) {
        for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
            if (_detected_modules[i].ap_uavcan == nullptr) {
                _detected_modules[i].ap_uavcan = ap_uavcan;
                _detected_modules[i].node_id = node_id;
                break;
            }
        }
    }
    return nullptr;
}

void AP_GPS_UAVCAN::handle_fix_msg(const FixCb &cb)
{
    bool process = false;

    WITH_SEMAPHORE(sem);

    if (cb.msg->status == uavcan::equipment::gnss::Fix::STATUS_NO_FIX) {
        interim_state.status = AP_GPS::GPS_Status::NO_FIX;
    } else {
        if (cb.msg->status == uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY) {
            interim_state.status = AP_GPS::GPS_Status::NO_FIX;
        } else if (cb.msg->status == uavcan::equipment::gnss::Fix::STATUS_2D_FIX) {
            interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;
            process = true;
        } else if (cb.msg->status == uavcan::equipment::gnss::Fix::STATUS_3D_FIX) {
            interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
            process = true;
        }

        if (cb.msg->gnss_time_standard == uavcan::equipment::gnss::Fix::GNSS_TIME_STANDARD_UTC) {
            uint64_t epoch_ms = uavcan::UtcTime(cb.msg->gnss_timestamp).toUSec();
            epoch_ms /= 1000;
            uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
            interim_state.time_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
            interim_state.time_week_ms = (uint32_t)(gps_ms - (interim_state.time_week) * AP_MSEC_PER_WEEK);
        }
    }

    if (process) {
        Location loc = { };
        loc.lat = cb.msg->latitude_deg_1e8 / 10;
        loc.lng = cb.msg->longitude_deg_1e8 / 10;
        loc.alt = cb.msg->height_msl_mm / 10;
        interim_state.location = loc;

        if (!uavcan::isNaN(cb.msg->ned_velocity[0])) {
            Vector3f vel(cb.msg->ned_velocity[0], cb.msg->ned_velocity[1], cb.msg->ned_velocity[2]);
            interim_state.velocity = vel;
            interim_state.ground_speed = norm(vel.x, vel.y);
            interim_state.ground_course = wrap_360(degrees(atan2f(vel.y, vel.x)));
            interim_state.have_vertical_velocity = true;
        } else {
            interim_state.have_vertical_velocity = false;
        }

        float pos_cov[9];
        cb.msg->position_covariance.unpackSquareMatrix(pos_cov);
        if (!uavcan::isNaN(pos_cov[8])) {
            if (pos_cov[8] > 0) {
                interim_state.vertical_accuracy = sqrtf(pos_cov[8]);
                interim_state.have_vertical_accuracy = true;
            } else {
                interim_state.have_vertical_accuracy = false;
            }
        } else {
            interim_state.have_vertical_accuracy = false;
        }

        const float horizontal_pos_variance = MAX(pos_cov[0], pos_cov[4]);
        if (!uavcan::isNaN(horizontal_pos_variance)) {
            if (horizontal_pos_variance > 0) {
                interim_state.horizontal_accuracy = sqrtf(horizontal_pos_variance);
                interim_state.have_horizontal_accuracy = true;
            } else {
                interim_state.have_horizontal_accuracy = false;
            }
        } else {
            interim_state.have_horizontal_accuracy = false;
        }

        float vel_cov[9];
        cb.msg->velocity_covariance.unpackSquareMatrix(vel_cov);
        if (!uavcan::isNaN(vel_cov[0])) {
            interim_state.speed_accuracy = sqrtf((vel_cov[0] + vel_cov[4] + vel_cov[8]) / 3.0);
            interim_state.have_speed_accuracy = true;
        } else {
            interim_state.have_speed_accuracy = false;
        }

        interim_state.num_sats = cb.msg->sats_used;
    } else {
        interim_state.have_vertical_velocity = false;
        interim_state.have_vertical_accuracy = false;
        interim_state.have_horizontal_accuracy = false;
        interim_state.have_speed_accuracy = false;
        interim_state.num_sats = 0;
    }

    interim_state.last_gps_time_ms = AP_HAL::millis();

    _new_data = true;
}

void AP_GPS_UAVCAN::handle_aux_msg(const AuxCb &cb)
{
    WITH_SEMAPHORE(sem);

    if (!uavcan::isNaN(cb.msg->hdop)) {
        interim_state.hdop = cb.msg->hdop * 100.0;
    }

    if (!uavcan::isNaN(cb.msg->vdop)) {
        interim_state.vdop = cb.msg->vdop * 100.0;
    }
}

void AP_GPS_UAVCAN::handle_fix_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const FixCb &cb)
{
    if (take_registry()) {
        AP_GPS_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);
        if (driver != nullptr) {
            driver->handle_fix_msg(cb);
        }
        give_registry();
    }
}

void AP_GPS_UAVCAN::handle_aux_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const AuxCb &cb)
{
    if (take_registry()) {
        AP_GPS_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);
        if (driver != nullptr) {
            driver->handle_aux_msg(cb);
        }
        give_registry();
    }
}

// Consume new data and mark it received
bool AP_GPS_UAVCAN::read(void)
{
    WITH_SEMAPHORE(sem);
    if (_new_data) {
        _new_data = false;

        state = interim_state;

        return true;
    }

    return false;
}

#endif // HAL_WITH_UAVCAN
