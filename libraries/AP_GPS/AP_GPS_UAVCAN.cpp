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

#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include "AP_GPS_UAVCAN.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_Logger/AP_Logger.h>

#include <uavcan/equipment/gnss/Fix2.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>
#include <ardupilot/gnss/Heading.hpp>
#include <ardupilot/gnss/Status.hpp>
#if GPS_MOVING_BASELINE
#include <ardupilot/gnss/MovingBaselineData.hpp>
#include <ardupilot/gnss/RelPosHeading.hpp>
#endif

#include <AP_BoardConfig/AP_BoardConfig.h>

#define GPS_PPS_EMULATION 0

extern const AP_HAL::HAL& hal;

#define GPS_UAVCAN_DEBUGGING 0

#if GPS_UAVCAN_DEBUGGING
#if defined(HAL_BUILD_AP_PERIPH)
 extern "C" {
   void can_printf(const char *fmt, ...);
 }
 # define Debug(fmt, args ...)  do {can_printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args);} while(0)
#else
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#endif
#else
 # define Debug(fmt, args ...)
#endif

#define LOG_TAG "GPS"

UC_REGISTRY_BINDER(Fix2Cb, uavcan::equipment::gnss::Fix2);
UC_REGISTRY_BINDER(AuxCb, uavcan::equipment::gnss::Auxiliary);
UC_REGISTRY_BINDER(HeadingCb, ardupilot::gnss::Heading);
UC_REGISTRY_BINDER(StatusCb, ardupilot::gnss::Status);
#if GPS_MOVING_BASELINE
UC_REGISTRY_BINDER(MovingBaselineDataCb, ardupilot::gnss::MovingBaselineData);
UC_REGISTRY_BINDER(RelPosHeadingCb, ardupilot::gnss::RelPosHeading);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define NATIVE_TIME_OFFSET (AP_HAL::micros64() - AP_HAL::native_micros64())
#else
#define NATIVE_TIME_OFFSET 0
#endif
AP_GPS_UAVCAN::DetectedModules AP_GPS_UAVCAN::_detected_modules[];
HAL_Semaphore AP_GPS_UAVCAN::_sem_registry;

// Member Methods
AP_GPS_UAVCAN::AP_GPS_UAVCAN(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_GPS::GPS_Role _role) :
    AP_GPS_Backend(_gps, _state, nullptr),
    interim_state(_state),
    role(_role)
{
    param_int_cb = FUNCTOR_BIND_MEMBER(&AP_GPS_UAVCAN::handle_param_get_set_response_int, bool, AP_UAVCAN*, const uint8_t, const char*, int32_t &);
    param_float_cb = FUNCTOR_BIND_MEMBER(&AP_GPS_UAVCAN::handle_param_get_set_response_float, bool, AP_UAVCAN*, const uint8_t, const char*, float &);
    param_save_cb = FUNCTOR_BIND_MEMBER(&AP_GPS_UAVCAN::handle_param_save_response, void, AP_UAVCAN*, const uint8_t, bool);
}

AP_GPS_UAVCAN::~AP_GPS_UAVCAN()
{
    WITH_SEMAPHORE(_sem_registry);

    _detected_modules[_detected_module].driver = nullptr;

#if GPS_MOVING_BASELINE
    if (rtcm3_parser != nullptr) {
        delete rtcm3_parser;
    }
#endif
}

void AP_GPS_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::gnss::Fix2, Fix2Cb> *gnss_fix2;
    gnss_fix2 = new uavcan::Subscriber<uavcan::equipment::gnss::Fix2, Fix2Cb>(*node);
    if (gnss_fix2 == nullptr) {
        AP_BoardConfig::allocation_error("gnss_fix2");
    }
    const int gnss_fix2_start_res = gnss_fix2->start(Fix2Cb(ap_uavcan, &handle_fix2_msg_trampoline));
    if (gnss_fix2_start_res < 0) {
        AP_HAL::panic("UAVCAN GNSS subscriber start problem\n\r");
    }
    
    uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary, AuxCb> *gnss_aux;
    gnss_aux = new uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary, AuxCb>(*node);
    if (gnss_aux == nullptr) {
        AP_BoardConfig::allocation_error("gnss_aux");
    }
    const int gnss_aux_start_res = gnss_aux->start(AuxCb(ap_uavcan, &handle_aux_msg_trampoline));
    if (gnss_aux_start_res < 0) {
        AP_HAL::panic("UAVCAN GNSS subscriber start problem\n\r");
    }

    uavcan::Subscriber<ardupilot::gnss::Heading, HeadingCb> *gnss_heading;
    gnss_heading = new uavcan::Subscriber<ardupilot::gnss::Heading, HeadingCb>(*node);
    if (gnss_heading == nullptr) {
        AP_BoardConfig::allocation_error("gnss_heading");
    }
    const int gnss_heading_start_res = gnss_heading->start(HeadingCb(ap_uavcan, &handle_heading_msg_trampoline));
    if (gnss_heading_start_res < 0) {
        AP_HAL::panic("UAVCAN GNSS subscriber start problem\n\r");
    }

    uavcan::Subscriber<ardupilot::gnss::Status, StatusCb> *gnss_status;
    gnss_status = new uavcan::Subscriber<ardupilot::gnss::Status, StatusCb>(*node);
    if (gnss_status == nullptr) {
        AP_BoardConfig::allocation_error("gnss_status");
    }
    const int gnss_status_start_res = gnss_status->start(StatusCb(ap_uavcan, &handle_status_msg_trampoline));
    if (gnss_status_start_res < 0) {
        AP_HAL::panic("UAVCAN GNSS subscriber start problem\n\r");
    }

#if GPS_MOVING_BASELINE
    uavcan::Subscriber<ardupilot::gnss::MovingBaselineData, MovingBaselineDataCb> *gnss_moving_baseline;
    gnss_moving_baseline = new uavcan::Subscriber<ardupilot::gnss::MovingBaselineData, MovingBaselineDataCb>(*node);
    if (gnss_moving_baseline == nullptr) {
        AP_BoardConfig::allocation_error("gnss_moving_baseline");
    }
    const int gnss_moving_baseline_start_res = gnss_moving_baseline->start(MovingBaselineDataCb(ap_uavcan, &handle_moving_baseline_msg_trampoline));
    if (gnss_moving_baseline_start_res < 0) {
        AP_HAL::panic("UAVCAN GNSS subscriber start problem\n\r");
    }

    uavcan::Subscriber<ardupilot::gnss::RelPosHeading, RelPosHeadingCb> *gnss_relposheading;
    gnss_relposheading = new uavcan::Subscriber<ardupilot::gnss::RelPosHeading, RelPosHeadingCb>(*node);
    if (gnss_relposheading == nullptr) {
        AP_BoardConfig::allocation_error("gnss_relposheading");
    }
    const int gnss_relposheading_start_res = gnss_relposheading->start(RelPosHeadingCb(ap_uavcan, &handle_relposheading_msg_trampoline));
    if (gnss_relposheading_start_res < 0) {
        AP_HAL::panic("UAVCAN GNSS subscriber start problem\n\r");
    }
#endif
}

AP_GPS_Backend* AP_GPS_UAVCAN::probe(AP_GPS &_gps, AP_GPS::GPS_State &_state)
{
    WITH_SEMAPHORE(_sem_registry);
    int8_t found_match = -1, last_match = -1;
    AP_GPS_UAVCAN* backend = nullptr;
    bool bad_override_config = false;
    for (int8_t i = GPS_MAX_RECEIVERS - 1; i >= 0; i--) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            if (_gps._override_node_id[_state.instance] != 0 &&
                _gps._override_node_id[_state.instance] != _detected_modules[i].node_id) {
                continue; // This device doesn't match the correct node
            }
            last_match = found_match;
            for (uint8_t j = 0; j < GPS_MAX_RECEIVERS; j++) {
                if (_detected_modules[i].node_id == _gps._override_node_id[j] &&
                    (j != _state.instance)) {
                    //wrong instance
                    found_match = -1;
                    break;
                }
                found_match = i;
            }

            // Handle Duplicate overrides
            for (uint8_t j = 0; j < GPS_MAX_RECEIVERS; j++) {
                if (_gps._override_node_id[i] != 0 && (i != j) &&
                    _gps._override_node_id[i] == _gps._override_node_id[j]) {
                    bad_override_config = true;
                }
            }
            if (bad_override_config) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Same Node Id %lu set for multiple GPS", (unsigned long int)_gps._override_node_id[i].get());
                last_match = i;
            }

            if (found_match == -1) {
                found_match = last_match;
                continue;
            }
            break;
        }
    }

    if (found_match == -1) {
        return NULL;
    }
    // initialise the backend based on the UAVCAN Moving baseline selection
    switch (_gps.get_type(_state.instance)) {
        case AP_GPS::GPS_TYPE_UAVCAN:
            backend = new AP_GPS_UAVCAN(_gps, _state, AP_GPS::GPS_ROLE_NORMAL);
            break;
#if GPS_MOVING_BASELINE
        case AP_GPS::GPS_TYPE_UAVCAN_RTK_BASE:
            backend = new AP_GPS_UAVCAN(_gps, _state, AP_GPS::GPS_ROLE_MB_BASE);
            break;
        case AP_GPS::GPS_TYPE_UAVCAN_RTK_ROVER:
            backend = new AP_GPS_UAVCAN(_gps, _state, AP_GPS::GPS_ROLE_MB_ROVER);
            break;
#endif
        default:
            return NULL;
    }
    if (backend == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR,
                            LOG_TAG,
                            "Failed to register UAVCAN GPS Node %d on Bus %d\n",
                            _detected_modules[found_match].node_id,
                            _detected_modules[found_match].ap_uavcan->get_driver_index());
    } else {
        _detected_modules[found_match].driver = backend;
        backend->_detected_module = found_match;
        AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
                            "Registered UAVCAN GPS Node %d on Bus %d as instance %d\n",
                            _detected_modules[found_match].node_id,
                            _detected_modules[found_match].ap_uavcan->get_driver_index(),
                            _state.instance);
        snprintf(backend->_name, ARRAY_SIZE(backend->_name), "UAVCAN%u-%u", _detected_modules[found_match].ap_uavcan->get_driver_index()+1, _detected_modules[found_match].node_id);
        _detected_modules[found_match].instance = _state.instance;
        for (uint8_t i=0; i < GPS_MAX_RECEIVERS; i++) {
            if (_detected_modules[found_match].node_id == AP::gps()._node_id[i]) {
                if (i == _state.instance) {
                    // Nothing to do here
                    break;
                }
                // else swap
                uint8_t tmp = AP::gps()._node_id[_state.instance].get();
                AP::gps()._node_id[_state.instance].set_and_notify(_detected_modules[found_match].node_id);
                AP::gps()._node_id[i].set_and_notify(tmp);
            }
        }
#if GPS_MOVING_BASELINE
        if (backend->role == AP_GPS::GPS_ROLE_MB_BASE) {
            backend->rtcm3_parser = new RTCM3_Parser;
            if (backend->rtcm3_parser == nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UAVCAN%u-%u: failed RTCMv3 parser allocation", _detected_modules[found_match].ap_uavcan->get_driver_index()+1, _detected_modules[found_match].node_id);
            }
        }
#endif // GPS_MOVING_BASELINE
    }

    return backend;
}

bool AP_GPS_UAVCAN::backends_healthy(char failure_msg[], uint16_t failure_msg_len)
{
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        bool overriden_node_found = false;
        bool bad_override_config = false;
        if (AP::gps()._override_node_id[i] == 0) {
            //anything goes
            continue;
        }
        for (uint8_t j = 0; j < GPS_MAX_RECEIVERS; j++) {
            if (AP::gps()._override_node_id[i] == AP::gps()._override_node_id[j] && (i != j)) {
                bad_override_config = true;
                break;
            }
            if (i == _detected_modules[j].instance && _detected_modules[j].driver) {
                if (AP::gps()._override_node_id[i] == _detected_modules[j].node_id) {
                    overriden_node_found = true;
                    break;
                }
            }
        }
        if (bad_override_config) {
            snprintf(failure_msg, failure_msg_len, "Same Node Id %lu set for multiple GPS", (unsigned long int)AP::gps()._override_node_id[i].get());
            return false;
        }

        if (!overriden_node_found) {
            snprintf(failure_msg, failure_msg_len, "Selected GPS Node %lu not set as instance %d", (unsigned long int)AP::gps()._override_node_id[i].get(), i + 1);
            return false;
        }
    }

    return true;
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
                // Just set the Node ID in order of appearance
                // This will be used to set select ids
                AP::gps()._node_id[i].set_and_notify(node_id);
                break;
            }
        }
    }
    struct DetectedModules tempslot;
    // Sort based on the node_id, larger values first
    // we do this, so that we have repeatable GPS
    // registration
    for (uint8_t i = 1; i < GPS_MAX_RECEIVERS; i++) {
        for (uint8_t j = i; j > 0; j--) {
            if (_detected_modules[j].node_id > _detected_modules[j-1].node_id) {
                tempslot = _detected_modules[j];
                _detected_modules[j] = _detected_modules[j-1];
                _detected_modules[j-1] = tempslot;
                // also fix the _detected_module in the driver so that RTCM injection
                // can determine if it has the bus to itself
                if (_detected_modules[j].driver) {
                    _detected_modules[j].driver->_detected_module = j;
                }
                if (_detected_modules[j-1].driver) {
                    _detected_modules[j-1].driver->_detected_module = j-1;
                }
            }
        }
    }
    return nullptr;
}

/*
  handle velocity element of message
 */
void AP_GPS_UAVCAN::handle_velocity(const float vx, const float vy, const float vz)
{
    if (!uavcan::isNaN(vx)) {
        const Vector3f vel(vx, vy, vz);
        interim_state.velocity = vel;
        velocity_to_speed_course(interim_state);
        // assume we have vertical velocity if we ever get a non-zero Z velocity
        if (!isnanf(vel.z) && !is_zero(vel.z)) {
            interim_state.have_vertical_velocity = true;
        } else {
            interim_state.have_vertical_velocity = state.have_vertical_velocity;
        }
    } else {
        interim_state.have_vertical_velocity = false;
    }
}

void AP_GPS_UAVCAN::handle_fix2_msg(const Fix2Cb &cb)
{
    bool process = false;
    seen_fix2 = true;

    WITH_SEMAPHORE(sem);

    if (cb.msg->status == uavcan::equipment::gnss::Fix2::STATUS_NO_FIX) {
        interim_state.status = AP_GPS::GPS_Status::NO_FIX;
    } else {
        if (cb.msg->status == uavcan::equipment::gnss::Fix2::STATUS_TIME_ONLY) {
            interim_state.status = AP_GPS::GPS_Status::NO_FIX;
        } else if (cb.msg->status == uavcan::equipment::gnss::Fix2::STATUS_2D_FIX) {
            interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;
            process = true;
        } else if (cb.msg->status == uavcan::equipment::gnss::Fix2::STATUS_3D_FIX) {
            interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
            process = true;
        }

        if (cb.msg->gnss_time_standard == uavcan::equipment::gnss::Fix2::GNSS_TIME_STANDARD_UTC) {
            uint64_t epoch_ms = uavcan::UtcTime(cb.msg->gnss_timestamp).toUSec();
            if (epoch_ms != 0) {
                epoch_ms /= 1000;
                uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
                interim_state.time_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
                interim_state.time_week_ms = (uint32_t)(gps_ms - (interim_state.time_week) * AP_MSEC_PER_WEEK);
            }
        }

        if (interim_state.status == AP_GPS::GPS_Status::GPS_OK_FIX_3D) {
            if (cb.msg->mode == uavcan::equipment::gnss::Fix2::MODE_DGPS) {
                interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS;
            } else if (cb.msg->mode == uavcan::equipment::gnss::Fix2::MODE_RTK) {
                if (cb.msg->sub_mode == uavcan::equipment::gnss::Fix2::SUB_MODE_RTK_FLOAT) {
                    interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT;
                } else if (cb.msg->sub_mode == uavcan::equipment::gnss::Fix2::SUB_MODE_RTK_FIXED) {
                    interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED;
                }
            }
        }
    }

    if (process) {
        Location loc = { };
        loc.lat = cb.msg->latitude_deg_1e8 / 10;
        loc.lng = cb.msg->longitude_deg_1e8 / 10;
        loc.alt = cb.msg->height_msl_mm / 10;
        interim_state.have_undulation = true;
        interim_state.undulation = (cb.msg->height_msl_mm - cb.msg->height_ellipsoid_mm) * 0.001;
        interim_state.location = loc;

        handle_velocity(cb.msg->ned_velocity[0], cb.msg->ned_velocity[1], cb.msg->ned_velocity[2]);

        if (cb.msg->covariance.size() == 6) {
            if (!uavcan::isNaN(cb.msg->covariance[0])) {
                interim_state.horizontal_accuracy = sqrtf(cb.msg->covariance[0]);
                interim_state.have_horizontal_accuracy = true;
            } else {
                interim_state.have_horizontal_accuracy = false;
            }
            if (!uavcan::isNaN(cb.msg->covariance[2])) {
                interim_state.vertical_accuracy = sqrtf(cb.msg->covariance[2]);
                interim_state.have_vertical_accuracy = true;
            } else {
                interim_state.have_vertical_accuracy = false;
            }
            if (!uavcan::isNaN(cb.msg->covariance[3]) &&
                !uavcan::isNaN(cb.msg->covariance[4]) &&
                !uavcan::isNaN(cb.msg->covariance[5])) {
                interim_state.speed_accuracy = sqrtf((cb.msg->covariance[3] + cb.msg->covariance[4] + cb.msg->covariance[5])/3);
                interim_state.have_speed_accuracy = true;
            } else {
                interim_state.have_speed_accuracy = false;
            }
        }

        interim_state.num_sats = cb.msg->sats_used;
    } else {
        interim_state.have_vertical_velocity = false;
        interim_state.have_vertical_accuracy = false;
        interim_state.have_horizontal_accuracy = false;
        interim_state.have_speed_accuracy = false;
        interim_state.num_sats = 0;
    }

    if (!seen_aux) {
        // if we haven't seen an Aux message then populate vdop and
        // hdop from pdop. Some GPS modules don't provide the Aux message
        interim_state.hdop = interim_state.vdop = cb.msg->pdop * 100.0;
    }

    if ((cb.msg->timestamp.usec > cb.msg->gnss_timestamp.usec) && (cb.msg->gnss_timestamp.usec > 0)) {
        // we have a valid timestamp based on gnss_timestamp timescale, we can use that to correct our gps message time
        interim_state.last_corrected_gps_time_us = jitter_correction.correct_offboard_timestamp_usec(cb.msg->timestamp.usec, (cb.msg->getUtcTimestamp().toUSec() + NATIVE_TIME_OFFSET));
        interim_state.last_gps_time_ms = interim_state.last_corrected_gps_time_us/1000U;
        interim_state.last_corrected_gps_time_us -= cb.msg->timestamp.usec - cb.msg->gnss_timestamp.usec;
        // this is also the time the message was received on the UART on other end.
        interim_state.corrected_timestamp_updated = true;
    } else {
        interim_state.last_gps_time_ms = jitter_correction.correct_offboard_timestamp_usec(cb.msg->timestamp.usec, cb.msg->getUtcTimestamp().toUSec() + NATIVE_TIME_OFFSET)/1000U;
    }

#if GPS_PPS_EMULATION
    // Emulates a PPS signal, can be used to check how close are we to real GPS time
    static virtual_timer_t timeout_vt;
    hal.gpio->pinMode(51, 1);
    auto handle_timeout = [](void *arg)
    {
        (void)arg;
        //we are called from ISR context
        chSysLockFromISR();
        hal.gpio->toggle(51);
        chSysUnlockFromISR();
    };

    static uint64_t next_toggle, last_toggle;
    
    next_toggle = (cb.msg->timestamp.usec) + (1000000ULL - ((cb.msg->timestamp.usec) % 1000000ULL));

    next_toggle += jitter_correction.get_link_offset_usec();
    if (next_toggle != last_toggle) {
        chVTSet(&timeout_vt, chTimeUS2I(next_toggle - AP_HAL::micros64()), handle_timeout, nullptr);
        last_toggle = next_toggle;
    }
#endif

    _new_data = true;
    if (!seen_message) {
        if (interim_state.status == AP_GPS::GPS_Status::NO_GPS) {
            // the first time we see a fix message we change from
            // NO_GPS to NO_FIX, indicating to user that a UAVCAN GPS
            // has been seen
            interim_state.status = AP_GPS::GPS_Status::NO_FIX;
        }
        seen_message = true;
    }
}

void AP_GPS_UAVCAN::handle_aux_msg(const AuxCb &cb)
{
    WITH_SEMAPHORE(sem);

    if (!uavcan::isNaN(cb.msg->hdop)) {
        seen_aux = true;
        interim_state.hdop = cb.msg->hdop * 100.0;
    }

    if (!uavcan::isNaN(cb.msg->vdop)) {
        seen_aux = true;
        interim_state.vdop = cb.msg->vdop * 100.0;
    }
}

void AP_GPS_UAVCAN::handle_heading_msg(const HeadingCb &cb)
{
#if GPS_MOVING_BASELINE
    if (seen_relposheading && gps.mb_params[interim_state.instance].type.get() != 0) {
        // we prefer to use the relposheading to get yaw as it allows
        // the user to more easily control the relative antenna positions
        return;
    }
#endif

    WITH_SEMAPHORE(sem);

    if (interim_state.gps_yaw_configured == false) {
        interim_state.gps_yaw_configured = cb.msg->heading_valid;
    }

    interim_state.have_gps_yaw = cb.msg->heading_valid;
    interim_state.gps_yaw = degrees(cb.msg->heading_rad);
    if (interim_state.have_gps_yaw) {
        interim_state.gps_yaw_time_ms = AP_HAL::millis();
    }

    interim_state.have_gps_yaw_accuracy = cb.msg->heading_accuracy_valid;
    interim_state.gps_yaw_accuracy = degrees(cb.msg->heading_accuracy_rad);
}

void AP_GPS_UAVCAN::handle_status_msg(const StatusCb &cb)
{
    WITH_SEMAPHORE(sem);

    seen_status = true;

    healthy = cb.msg->healthy;
    status_flags = cb.msg->status;
    if (error_code != cb.msg->error_codes) {
        AP::logger().Write_MessageF("GPS %d: error changed (0x%08x/0x%08x)",
                                    (unsigned int)(state.instance + 1),
                                    error_code,
                                    cb.msg->error_codes);
        error_code = cb.msg->error_codes;
    }
}

#if GPS_MOVING_BASELINE
/*
  handle moving baseline data.
  */
void AP_GPS_UAVCAN::handle_moving_baseline_msg(const MovingBaselineDataCb &cb, uint8_t node_id)
{
    WITH_SEMAPHORE(sem);
    if (role != AP_GPS::GPS_ROLE_MB_BASE) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Incorrect Role set for UAVCAN GPS, %d should be Base", node_id);
        return;
    }

    if (rtcm3_parser == nullptr) {
        return;
    }
    for (const auto &c : cb.msg->data) {
        rtcm3_parser->read(c);
    }
}

/*
    handle relposheading message
*/
void AP_GPS_UAVCAN::handle_relposheading_msg(const RelPosHeadingCb &cb, uint8_t node_id)
{
    WITH_SEMAPHORE(sem);

    interim_state.gps_yaw_configured = true;
    seen_relposheading = true;
    // push raw heading data to calculate moving baseline heading states
    if (calculate_moving_base_yaw(interim_state,
                                cb.msg->reported_heading_deg,
                                cb.msg->relative_distance_m,
                                cb.msg->relative_down_pos_m)) {
        if (cb.msg->reported_heading_acc_available) {
            interim_state.gps_yaw_accuracy = cb.msg->reported_heading_acc_deg;
        }
        interim_state.have_gps_yaw_accuracy = cb.msg->reported_heading_acc_available;
    }
}

// support for retrieving RTCMv3 data from a moving baseline base
bool AP_GPS_UAVCAN::get_RTCMV3(const uint8_t *&bytes, uint16_t &len)
{
    WITH_SEMAPHORE(sem);
    if (rtcm3_parser != nullptr) {
        len = rtcm3_parser->get_len(bytes);
        return len > 0;
    }
    return false;
}

// clear previous RTCM3 packet
void AP_GPS_UAVCAN::clear_RTCMV3(void)
{
    WITH_SEMAPHORE(sem);
    if (rtcm3_parser != nullptr) {
        rtcm3_parser->clear_packet();
    }
}

#endif // GPS_MOVING_BASELINE

void AP_GPS_UAVCAN::handle_fix2_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const Fix2Cb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_GPS_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);
    if (driver != nullptr) {
        driver->handle_fix2_msg(cb);
    }
}

void AP_GPS_UAVCAN::handle_aux_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const AuxCb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_GPS_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);
    if (driver != nullptr) {
        driver->handle_aux_msg(cb);
    }
}

void AP_GPS_UAVCAN::handle_heading_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const HeadingCb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_GPS_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);
    if (driver != nullptr) {
        driver->handle_heading_msg(cb);
    }
}

void AP_GPS_UAVCAN::handle_status_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const StatusCb &cb)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_GPS_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);
    if (driver != nullptr) {
        driver->handle_status_msg(cb);
    }
}

#if GPS_MOVING_BASELINE
// Moving Baseline msg trampoline
void AP_GPS_UAVCAN::handle_moving_baseline_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MovingBaselineDataCb &cb)
{
    WITH_SEMAPHORE(_sem_registry);
    AP_GPS_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);
    if (driver != nullptr) {
        driver->handle_moving_baseline_msg(cb, node_id);
    }
}

// RelPosHeading msg trampoline
void AP_GPS_UAVCAN::handle_relposheading_msg_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const RelPosHeadingCb &cb)
{
    WITH_SEMAPHORE(_sem_registry);
    AP_GPS_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);
    if (driver != nullptr) {
        driver->handle_relposheading_msg(cb, node_id);
    }
}
#endif

bool AP_GPS_UAVCAN::do_config()
{
    AP_UAVCAN *ap_uavcan = _detected_modules[_detected_module].ap_uavcan;
    if (ap_uavcan == nullptr) {
        return false;
    }
    uint8_t node_id = _detected_modules[_detected_module].node_id;
    
    switch(cfg_step) {
        case STEP_SET_TYPE:
            ap_uavcan->get_parameter_on_node(node_id, "GPS_TYPE", &param_int_cb);
            break;
        case STEP_SET_MB_CAN_TX:
            if (role != AP_GPS::GPS_Role::GPS_ROLE_NORMAL) {
                ap_uavcan->get_parameter_on_node(node_id, "GPS_MB_ONLY_PORT", &param_int_cb);
            } else {
                cfg_step++;
            }
            break;
        case STEP_SAVE_AND_REBOOT:
            if (requires_save_and_reboot) {
                ap_uavcan->save_parameters_on_node(node_id, &param_save_cb);
            } else {
                cfg_step++;
            }
            break;
        case STEP_FINISHED:
            return true;
        default:
            break;
    }
    return false;
}

// Consume new data and mark it received
bool AP_GPS_UAVCAN::read(void)
{
    if (gps._auto_config >= AP_GPS::GPS_AUTO_CONFIG_ENABLE_ALL) {
        if (!do_config()) {
            return false;
        }
    }

    WITH_SEMAPHORE(sem);
    if (_new_data) {
        _new_data = false;

        // the encoding of accuracies in UAVCAN can result in infinite
        // values. These cause problems with blending. Use 1000m and 1000m/s instead
        interim_state.horizontal_accuracy = MIN(interim_state.horizontal_accuracy, 1000.0);
        interim_state.vertical_accuracy = MIN(interim_state.vertical_accuracy, 1000.0);
        interim_state.speed_accuracy = MIN(interim_state.speed_accuracy, 1000.0);

        state = interim_state;
        if (interim_state.last_corrected_gps_time_us) {
            // If we were able to get a valid last_corrected_gps_time_us
            // we have had a valid GPS message time, from which we calculate
            // the time of week.
            _last_itow_ms = interim_state.time_week_ms;
            _have_itow = true;
        }
        return true;
    }
    if (!seen_message) {
        // start with NO_GPS until we get first packet
        state.status = AP_GPS::GPS_Status::NO_GPS;
    }

    return false;
}

bool AP_GPS_UAVCAN::is_healthy(void) const
{
    // if we don't have any health reports, assume it's healthy
    if (!seen_status) {
        return true;
    }
    return healthy;
}

bool AP_GPS_UAVCAN::logging_healthy(void) const
{
    // if we don't have status, assume it's valid
    if (!seen_status) {
        return true;
    }

    return (status_flags & ardupilot::gnss::Status::STATUS_LOGGING) != 0;
}

bool AP_GPS_UAVCAN::is_configured(void) const
{
    // if we don't have status assume it's configured
    if (!seen_status) {
        return true;
    }

    return (status_flags & ardupilot::gnss::Status::STATUS_ARMABLE) != 0;
}

/*
  handle RTCM data from MAVLink GPS_RTCM_DATA, forwarding it over MAVLink
 */
void AP_GPS_UAVCAN::inject_data(const uint8_t *data, uint16_t len)
{
    // we only handle this if we are the first UAVCAN GPS or we are
    // using a different uavcan instance than the first GPS, as we
    // send the data as broadcast on all UAVCAN devive ports and we
    // don't want to send duplicates
    if (_detected_module == 0 ||
        _detected_modules[_detected_module].ap_uavcan != _detected_modules[0].ap_uavcan) {
        _detected_modules[_detected_module].ap_uavcan->send_RTCMStream(data, len);
    }
}

/*
    handle param get/set response
*/
bool AP_GPS_UAVCAN::handle_param_get_set_response_int(AP_UAVCAN* ap_uavcan, uint8_t node_id, const char* name, int32_t &value)
{
    Debug("AP_GPS_UAVCAN: param set/get response from %d %s %ld\n", node_id, name, value);
    if (strcmp(name, "GPS_TYPE") == 0 && cfg_step == STEP_SET_TYPE) {
        if (role == AP_GPS::GPS_ROLE_MB_BASE && value != AP_GPS::GPS_TYPE_UBLOX_RTK_BASE) {
            value = (int32_t)AP_GPS::GPS_TYPE_UBLOX_RTK_BASE;
            requires_save_and_reboot = true;
            return true;
        } else if (role == AP_GPS::GPS_ROLE_MB_ROVER && value != AP_GPS::GPS_TYPE_UBLOX_RTK_ROVER) {
            value = (int32_t)AP_GPS::GPS_TYPE_UBLOX_RTK_ROVER;
            requires_save_and_reboot = true;
            return true;
        } else {
            cfg_step++;
        }
    }

    if (strcmp(name, "GPS_MB_ONLY_PORT") == 0 && cfg_step == STEP_SET_MB_CAN_TX) {
        if (option_set(AP_GPS::UAVCAN_MBUseDedicatedBus) && !value) {
            // set up so that another CAN port is used for the Moving Baseline Data
            // setting this value will allow another CAN port to be used as dedicated
            // line for the Moving Baseline Data
            value = 1;
            requires_save_and_reboot = true;
            return true;
        } else if (!option_set(AP_GPS::UAVCAN_MBUseDedicatedBus) && value) {
            // set up so that all CAN ports are used for the Moving Baseline Data
            value = 0;
            requires_save_and_reboot = true;
            return true;
        } else {
            cfg_step++;
        }
    }
    return false;
}

bool AP_GPS_UAVCAN::handle_param_get_set_response_float(AP_UAVCAN* ap_uavcan, uint8_t node_id, const char* name, float &value)
{
    Debug("AP_GPS_UAVCAN: param set/get response from %d %s %f\n", node_id, name, value);
    return false;
}

void AP_GPS_UAVCAN::handle_param_save_response(AP_UAVCAN* ap_uavcan, const uint8_t node_id, bool success)
{
    Debug("AP_GPS_UAVCAN: param save response from %d %s\n", node_id, success ? "success" : "failure");

    if (cfg_step != STEP_SAVE_AND_REBOOT) {
        return;
    }

    if (success) {
        cfg_step++;
    }
    // Also send reboot command
    // this is ok as we are sending from UAVCAN thread context
    Debug("AP_GPS_UAVCAN: sending reboot command %d\n", node_id);
    ap_uavcan->send_reboot_request(node_id);
}

#endif // HAL_ENABLE_LIBUAVCAN_DRIVERS
