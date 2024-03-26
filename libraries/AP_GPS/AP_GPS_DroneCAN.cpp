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
#include "AP_GPS_config.h"

#if AP_GPS_DRONECAN_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "AP_GPS_DroneCAN.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_Logger/AP_Logger.h>

#include <stdio.h>
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

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define NATIVE_TIME_OFFSET (AP_HAL::micros64() - AP_HAL::micros64())
#else
#define NATIVE_TIME_OFFSET 0
#endif
AP_GPS_DroneCAN::DetectedModules AP_GPS_DroneCAN::_detected_modules[];
HAL_Semaphore AP_GPS_DroneCAN::_sem_registry;

// Member Methods
AP_GPS_DroneCAN::AP_GPS_DroneCAN(AP_GPS &_gps,
                                 AP_GPS::Params &_params,
                                 AP_GPS::GPS_State &_state,
                                 AP_GPS::GPS_Role _role) :
    AP_GPS_Backend(_gps, _params, _state, nullptr),
    interim_state(_state),
    role(_role)
{
    param_int_cb = FUNCTOR_BIND_MEMBER(&AP_GPS_DroneCAN::handle_param_get_set_response_int, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    param_float_cb = FUNCTOR_BIND_MEMBER(&AP_GPS_DroneCAN::handle_param_get_set_response_float, bool, AP_DroneCAN*, const uint8_t, const char*, float &);
    param_save_cb = FUNCTOR_BIND_MEMBER(&AP_GPS_DroneCAN::handle_param_save_response, void, AP_DroneCAN*, const uint8_t, bool);
}

AP_GPS_DroneCAN::~AP_GPS_DroneCAN()
{
    WITH_SEMAPHORE(_sem_registry);

    _detected_modules[_detected_module].driver = nullptr;

#if GPS_MOVING_BASELINE
    if (rtcm3_parser != nullptr) {
        delete rtcm3_parser;
    }
#endif
}

void AP_GPS_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_fix2_msg_trampoline, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("status_sub");
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_aux_msg_trampoline, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("status_sub");
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_heading_msg_trampoline, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("status_sub");
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_status_msg_trampoline, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("status_sub");
    }
#if GPS_MOVING_BASELINE
    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_moving_baseline_msg_trampoline, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("moving_baseline_sub");
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_relposheading_msg_trampoline, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("relposheading_sub");
    }
#endif
}

AP_GPS_Backend* AP_GPS_DroneCAN::probe(AP_GPS &_gps, AP_GPS::GPS_State &_state)
{
    WITH_SEMAPHORE(_sem_registry);
    int8_t found_match = -1, last_match = -1;
    AP_GPS_DroneCAN* backend = nullptr;
    bool bad_override_config = false;
    for (int8_t i = GPS_MAX_RECEIVERS - 1; i >= 0; i--) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_dronecan != nullptr) {
            if (_gps.params[_state.instance].override_node_id != 0 &&
                _gps.params[_state.instance].override_node_id != _detected_modules[i].node_id) {
                continue; // This device doesn't match the correct node
            }
            last_match = found_match;
            for (uint8_t j = 0; j < GPS_MAX_RECEIVERS; j++) {
                if (_detected_modules[i].node_id == _gps.params[j].override_node_id &&
                    (j != _state.instance)) {
                    //wrong instance
                    found_match = -1;
                    break;
                }
                found_match = i;
            }

            // Handle Duplicate overrides
            for (uint8_t j = 0; j < GPS_MAX_RECEIVERS; j++) {
                if (_gps.params[i].override_node_id != 0 && (i != j) &&
                    _gps.params[i].override_node_id == _gps.params[j].override_node_id) {
                    bad_override_config = true;
                }
            }
            if (bad_override_config) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Same Node Id %lu set for multiple GPS", (unsigned long int)_gps.params[i].override_node_id.get());
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
            backend = new AP_GPS_DroneCAN(_gps, _gps.params[_state.instance], _state, AP_GPS::GPS_ROLE_NORMAL);
            break;
#if GPS_MOVING_BASELINE
        case AP_GPS::GPS_TYPE_UAVCAN_RTK_BASE:
            backend = new AP_GPS_DroneCAN(_gps, _gps.params[_state.instance], _state, AP_GPS::GPS_ROLE_MB_BASE);
            break;
        case AP_GPS::GPS_TYPE_UAVCAN_RTK_ROVER:
            backend = new AP_GPS_DroneCAN(_gps, _gps.params[_state.instance], _state, AP_GPS::GPS_ROLE_MB_ROVER);
            break;
#endif
        default:
            return NULL;
    }
    if (backend == nullptr) {
        AP::can().log_text(AP_CANManager::LOG_ERROR,
                            LOG_TAG,
                            "Failed to register DroneCAN GPS Node %d on Bus %d\n",
                            _detected_modules[found_match].node_id,
                            _detected_modules[found_match].ap_dronecan->get_driver_index());
    } else {
        _detected_modules[found_match].driver = backend;
        backend->_detected_module = found_match;
        AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
                            "Registered DroneCAN GPS Node %d on Bus %d as instance %d\n",
                            _detected_modules[found_match].node_id,
                            _detected_modules[found_match].ap_dronecan->get_driver_index(),
                            _state.instance);
        snprintf(backend->_name, ARRAY_SIZE(backend->_name), "DroneCAN%u-%u", _detected_modules[found_match].ap_dronecan->get_driver_index()+1, _detected_modules[found_match].node_id);
        _detected_modules[found_match].instance = _state.instance;
        for (uint8_t i=0; i < GPS_MAX_RECEIVERS; i++) {
            if (_detected_modules[found_match].node_id == AP::gps().params[i].node_id) {
                if (i == _state.instance) {
                    // Nothing to do here
                    break;
                }
                // else swap
                uint8_t tmp = AP::gps().params[_state.instance].node_id.get();
                AP::gps().params[_state.instance].node_id.set_and_notify(_detected_modules[found_match].node_id);
                AP::gps().params[i].node_id.set_and_notify(tmp);
            }
        }
#if GPS_MOVING_BASELINE
        if (backend->role == AP_GPS::GPS_ROLE_MB_BASE) {
            backend->rtcm3_parser = new RTCM3_Parser;
            if (backend->rtcm3_parser == nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DroneCAN%u-%u: failed RTCMv3 parser allocation", _detected_modules[found_match].ap_dronecan->get_driver_index()+1, _detected_modules[found_match].node_id);
            }
        }
#endif // GPS_MOVING_BASELINE
    }

    return backend;
}

bool AP_GPS_DroneCAN::backends_healthy(char failure_msg[], uint16_t failure_msg_len)
{
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        const auto &params_i = AP::gps().params[i];
        bool overriden_node_found = false;
        bool bad_override_config = false;
        if (params_i.override_node_id == 0) {
            //anything goes
            continue;
        }
        for (uint8_t j = 0; j < GPS_MAX_RECEIVERS; j++) {
            const auto &params_j = AP::gps().params[j];
            if (params_i.override_node_id == params_j.override_node_id && (i != j)) {
                bad_override_config = true;
                break;
            }
            if (i == _detected_modules[j].instance && _detected_modules[j].driver) {
                if (params_i.override_node_id == _detected_modules[j].node_id) {
                    overriden_node_found = true;
                    break;
                }
            }
        }
        if (bad_override_config) {
            snprintf(failure_msg, failure_msg_len, "Same Node Id %lu set for multiple GPS", (unsigned long int)params_i.override_node_id.get());
            return false;
        }

        if (!overriden_node_found) {
            snprintf(failure_msg, failure_msg_len, "Selected GPS Node %lu not set as instance %d", (unsigned long int)params_i.override_node_id.get(), i + 1);
            return false;
        }
    }

    return true;
}

AP_GPS_DroneCAN* AP_GPS_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_dronecan == ap_dronecan && 
            _detected_modules[i].node_id == node_id) {
            return _detected_modules[i].driver;
        }
    }

    bool already_detected = false;
    // Check if there's an empty spot for possible registeration
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        if (_detected_modules[i].ap_dronecan == ap_dronecan && _detected_modules[i].node_id == node_id) {
            // Already Detected
            already_detected = true;
            break;
        }
    }
    if (!already_detected) {
        for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
            if (_detected_modules[i].ap_dronecan == nullptr) {
                _detected_modules[i].ap_dronecan = ap_dronecan;
                _detected_modules[i].node_id = node_id;
                // Just set the Node ID in order of appearance
                // This will be used to set select ids
                AP::gps().params[i].node_id.set_and_notify(node_id);
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
void AP_GPS_DroneCAN::handle_velocity(const float vx, const float vy, const float vz)
{
    if (!isnan(vx)) {
        const Vector3f vel(vx, vy, vz);
        interim_state.velocity = vel;
        velocity_to_speed_course(interim_state);
        // assume we have vertical velocity if we ever get a non-zero Z velocity
        if (!isnan(vel.z) && !is_zero(vel.z)) {
            interim_state.have_vertical_velocity = true;
        } else {
            interim_state.have_vertical_velocity = state.have_vertical_velocity;
        }
    } else {
        interim_state.have_vertical_velocity = false;
    }
}

void AP_GPS_DroneCAN::handle_fix2_msg(const uavcan_equipment_gnss_Fix2& msg, uint64_t timestamp_usec)
{
    bool process = false;
    seen_fix2 = true;

    WITH_SEMAPHORE(sem);

    if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX) {
        interim_state.status = AP_GPS::GPS_Status::NO_FIX;
    } else {
        if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_TIME_ONLY) {
            interim_state.status = AP_GPS::GPS_Status::NO_FIX;
        } else if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX) {
            interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;
            process = true;
        } else if (msg.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX) {
            interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
            process = true;
        }

        if (msg.gnss_time_standard == UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC) {
            uint64_t epoch_ms = msg.gnss_timestamp.usec;
            if (epoch_ms != 0) {
                epoch_ms /= 1000;
                uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
                interim_state.time_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
                interim_state.time_week_ms = (uint32_t)(gps_ms - (interim_state.time_week) * AP_MSEC_PER_WEEK);
            }
        }

        if (interim_state.status == AP_GPS::GPS_Status::GPS_OK_FIX_3D) {
            if (msg.mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS) {
                interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS;
            } else if (msg.mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK) {
                if (msg.sub_mode == UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT) {
                    interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT;
                } else if (msg.sub_mode == UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED) {
                    interim_state.status = AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED;
                }
            }
        }
    }

    if (process) {
        Location loc = { };
        loc.lat = msg.latitude_deg_1e8 / 10;
        loc.lng = msg.longitude_deg_1e8 / 10;
        const int32_t alt_amsl_cm = msg.height_msl_mm / 10;
        interim_state.have_undulation = true;
        interim_state.undulation = (msg.height_msl_mm - msg.height_ellipsoid_mm) * 0.001;
        interim_state.location = loc;
        set_alt_amsl_cm(interim_state, alt_amsl_cm);

        handle_velocity(msg.ned_velocity[0], msg.ned_velocity[1], msg.ned_velocity[2]);

        if (msg.covariance.len == 6) {
            if (!isnan(msg.covariance.data[0])) {
                interim_state.horizontal_accuracy = sqrtf(msg.covariance.data[0]);
                interim_state.have_horizontal_accuracy = true;
            } else {
                interim_state.have_horizontal_accuracy = false;
            }
            if (!isnan(msg.covariance.data[2])) {
                interim_state.vertical_accuracy = sqrtf(msg.covariance.data[2]);
                interim_state.have_vertical_accuracy = true;
            } else {
                interim_state.have_vertical_accuracy = false;
            }
            if (!isnan(msg.covariance.data[3]) &&
                !isnan(msg.covariance.data[4]) &&
                !isnan(msg.covariance.data[5])) {
                interim_state.speed_accuracy = sqrtf((msg.covariance.data[3] + msg.covariance.data[4] + msg.covariance.data[5])/3);
                interim_state.have_speed_accuracy = true;
            } else {
                interim_state.have_speed_accuracy = false;
            }
        }

        interim_state.num_sats = msg.sats_used;
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
        interim_state.hdop = interim_state.vdop = msg.pdop * 100.0;
    }

    if ((msg.timestamp.usec > msg.gnss_timestamp.usec) && (msg.gnss_timestamp.usec > 0)) {
        // we have a valid timestamp based on gnss_timestamp timescale, we can use that to correct our gps message time
        interim_state.last_corrected_gps_time_us = jitter_correction.correct_offboard_timestamp_usec(msg.timestamp.usec, (timestamp_usec + NATIVE_TIME_OFFSET));
        interim_state.last_gps_time_ms = interim_state.last_corrected_gps_time_us/1000U;
        interim_state.last_corrected_gps_time_us -= msg.timestamp.usec - msg.gnss_timestamp.usec;
        // this is also the time the message was received on the UART on other end.
        interim_state.corrected_timestamp_updated = true;
    } else {
        interim_state.last_gps_time_ms = jitter_correction.correct_offboard_timestamp_usec(msg.timestamp.usec, timestamp_usec + NATIVE_TIME_OFFSET)/1000U;
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
    
    next_toggle = (msg.timestamp.usec) + (1000000ULL - ((msg.timestamp.usec) % 1000000ULL));

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
            // NO_GPS to NO_FIX, indicating to user that a DroneCAN GPS
            // has been seen
            interim_state.status = AP_GPS::GPS_Status::NO_FIX;
        }
        seen_message = true;
    }
}

void AP_GPS_DroneCAN::handle_aux_msg(const uavcan_equipment_gnss_Auxiliary& msg)
{
    WITH_SEMAPHORE(sem);

    if (!isnan(msg.hdop)) {
        seen_aux = true;
        interim_state.hdop = msg.hdop * 100.0;
    }

    if (!isnan(msg.vdop)) {
        seen_aux = true;
        interim_state.vdop = msg.vdop * 100.0;
    }
}

void AP_GPS_DroneCAN::handle_heading_msg(const ardupilot_gnss_Heading& msg)
{
#if GPS_MOVING_BASELINE
    if (seen_relposheading && gps.params[interim_state.instance].mb_params.type.get() != 0) {
        // we prefer to use the relposheading to get yaw as it allows
        // the user to more easily control the relative antenna positions
        return;
    }
#endif

    WITH_SEMAPHORE(sem);

    if (interim_state.gps_yaw_configured == false) {
        interim_state.gps_yaw_configured = msg.heading_valid;
    }

    interim_state.have_gps_yaw = msg.heading_valid;
    interim_state.gps_yaw = degrees(msg.heading_rad);
    if (interim_state.have_gps_yaw) {
        interim_state.gps_yaw_time_ms = AP_HAL::millis();
    }

    interim_state.have_gps_yaw_accuracy = msg.heading_accuracy_valid;
    interim_state.gps_yaw_accuracy = degrees(msg.heading_accuracy_rad);
}

void AP_GPS_DroneCAN::handle_status_msg(const ardupilot_gnss_Status& msg)
{
    WITH_SEMAPHORE(sem);

    seen_status = true;

    healthy = msg.healthy;
    status_flags = msg.status;
    if (error_code != msg.error_codes) {
#if HAL_LOGGING_ENABLED
        AP::logger().Write_MessageF("GPS %d: error changed (0x%08x/0x%08x)",
                                    (unsigned int)(state.instance + 1),
                                    error_code,
                                    msg.error_codes);
#endif
        error_code = msg.error_codes;
    }
}

#if GPS_MOVING_BASELINE
/*
  handle moving baseline data.
  */
void AP_GPS_DroneCAN::handle_moving_baseline_msg(const ardupilot_gnss_MovingBaselineData& msg, uint8_t node_id)
{
    WITH_SEMAPHORE(sem);
    if (role != AP_GPS::GPS_ROLE_MB_BASE) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Incorrect Role set for DroneCAN GPS, %d should be Base", node_id);
        return;
    }

    if (rtcm3_parser == nullptr) {
        return;
    }
    for (int i=0; i < msg.data.len; i++) {
        rtcm3_parser->read(msg.data.data[i]);
    }
}

/*
    handle relposheading message
*/
void AP_GPS_DroneCAN::handle_relposheading_msg(const ardupilot_gnss_RelPosHeading& msg, uint8_t node_id)
{
    WITH_SEMAPHORE(sem);

    interim_state.gps_yaw_configured = true;
    seen_relposheading = true;
    // push raw heading data to calculate moving baseline heading states
    if (calculate_moving_base_yaw(interim_state,
                                msg.reported_heading_deg,
                                msg.relative_distance_m,
                                msg.relative_down_pos_m)) {
        if (msg.reported_heading_acc_available) {
            interim_state.gps_yaw_accuracy = msg.reported_heading_acc_deg;
        }
        interim_state.have_gps_yaw_accuracy = msg.reported_heading_acc_available;
    }
}

// support for retrieving RTCMv3 data from a moving baseline base
bool AP_GPS_DroneCAN::get_RTCMV3(const uint8_t *&bytes, uint16_t &len)
{
    WITH_SEMAPHORE(sem);
    if (rtcm3_parser != nullptr) {
        len = rtcm3_parser->get_len(bytes);
        return len > 0;
    }
    return false;
}

// clear previous RTCM3 packet
void AP_GPS_DroneCAN::clear_RTCMV3(void)
{
    WITH_SEMAPHORE(sem);
    if (rtcm3_parser != nullptr) {
        rtcm3_parser->clear_packet();
    }
}

#endif // GPS_MOVING_BASELINE

void AP_GPS_DroneCAN::handle_fix2_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_gnss_Fix2& msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_GPS_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);
    if (driver != nullptr) {
        driver->handle_fix2_msg(msg, transfer.timestamp_usec);
    }
}

void AP_GPS_DroneCAN::handle_aux_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_gnss_Auxiliary& msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_GPS_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);
    if (driver != nullptr) {
        driver->handle_aux_msg(msg);
    }
}

void AP_GPS_DroneCAN::handle_heading_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_gnss_Heading& msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_GPS_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);
    if (driver != nullptr) {
        driver->handle_heading_msg(msg);
    }
}

void AP_GPS_DroneCAN::handle_status_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_gnss_Status& msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_GPS_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);
    if (driver != nullptr) {
        driver->handle_status_msg(msg);
    }
}

#if GPS_MOVING_BASELINE
// Moving Baseline msg trampoline
void AP_GPS_DroneCAN::handle_moving_baseline_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_gnss_MovingBaselineData& msg)
{
    WITH_SEMAPHORE(_sem_registry);
    AP_GPS_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);
    if (driver != nullptr) {
        driver->handle_moving_baseline_msg(msg, transfer.source_node_id);
    }
}

// RelPosHeading msg trampoline
void AP_GPS_DroneCAN::handle_relposheading_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_gnss_RelPosHeading& msg)
{
    WITH_SEMAPHORE(_sem_registry);
    AP_GPS_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);
    if (driver != nullptr) {
        driver->handle_relposheading_msg(msg, transfer.source_node_id);
    }
}
#endif

bool AP_GPS_DroneCAN::do_config()
{
    AP_DroneCAN *ap_dronecan = _detected_modules[_detected_module].ap_dronecan;
    if (ap_dronecan == nullptr) {
        return false;
    }
    uint8_t node_id = _detected_modules[_detected_module].node_id;
    
    switch(cfg_step) {
        case STEP_SET_TYPE:
            // GPS_TYPE was renamed GPS1_TYPE.  Request both and
            // handle whichever we receive.
            ap_dronecan->get_parameter_on_node(node_id, "GPS_TYPE", &param_int_cb);
            ap_dronecan->get_parameter_on_node(node_id, "GPS1_TYPE", &param_int_cb);
            break;
        case STEP_SET_MB_CAN_TX:
            if (role != AP_GPS::GPS_Role::GPS_ROLE_NORMAL) {
                ap_dronecan->get_parameter_on_node(node_id, "GPS_MB_ONLY_PORT", &param_int_cb);
            } else {
                cfg_step++;
            }
            break;
        case STEP_SAVE_AND_REBOOT:
            if (requires_save_and_reboot) {
                ap_dronecan->save_parameters_on_node(node_id, &param_save_cb);
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
bool AP_GPS_DroneCAN::read(void)
{
    if (gps._auto_config >= AP_GPS::GPS_AUTO_CONFIG_ENABLE_ALL) {
        if (!do_config()) {
            return false;
        }
    }

    WITH_SEMAPHORE(sem);

    send_rtcm();

    if (_new_data) {
        _new_data = false;

        // the encoding of accuracies in DroneCAN can result in infinite
        // values. These cause problems with blending. Use 1000m and 1000m/s instead
        interim_state.horizontal_accuracy = MIN(interim_state.horizontal_accuracy, 1000.0);
        interim_state.vertical_accuracy = MIN(interim_state.vertical_accuracy, 1000.0);
        interim_state.speed_accuracy = MIN(interim_state.speed_accuracy, 1000.0);

        // prevent announcing multiple times
        interim_state.announced_detection = state.announced_detection;

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

bool AP_GPS_DroneCAN::is_healthy(void) const
{
    // if we don't have any health reports, assume it's healthy
    if (!seen_status) {
        return true;
    }
    return healthy;
}

bool AP_GPS_DroneCAN::logging_healthy(void) const
{
    // if we don't have status, assume it's valid
    if (!seen_status) {
        return true;
    }

    return (status_flags & ARDUPILOT_GNSS_STATUS_STATUS_LOGGING) != 0;
}

bool AP_GPS_DroneCAN::is_configured(void) const
{
    // if we don't have status assume it's configured
    if (!seen_status) {
        return true;
    }

    return (status_flags & ARDUPILOT_GNSS_STATUS_STATUS_ARMABLE) != 0;
}

/*
  send pending RTCM data
 */
void AP_GPS_DroneCAN::send_rtcm(void)
{
    if (_rtcm_stream.buf == nullptr) {
        return;
    }
    WITH_SEMAPHORE(sem);

    const uint32_t now = AP_HAL::millis();
    if (now - _rtcm_stream.last_send_ms < 20) {
        // don't send more than 50 per second
        return;
    }
    uint32_t outlen = 0;
    const uint8_t *ptr = _rtcm_stream.buf->readptr(outlen);
    if (ptr == nullptr || outlen == 0) {
        return;
    }
    uavcan_equipment_gnss_RTCMStream msg {};
    outlen = MIN(outlen, sizeof(msg.data.data));
    msg.protocol_id = UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_PROTOCOL_ID_RTCM3;
    memcpy(msg.data.data, ptr, outlen);
    msg.data.len = outlen;
    if (_detected_modules[_detected_module].ap_dronecan->rtcm_stream.broadcast(msg)) {
        _rtcm_stream.buf->advance(outlen);
        _rtcm_stream.last_send_ms = now;
    }
}

/*
  handle RTCM data from MAVLink GPS_RTCM_DATA, forwarding it over MAVLink
 */
void AP_GPS_DroneCAN::inject_data(const uint8_t *data, uint16_t len)
{
    // we only handle this if we are the first DroneCAN GPS or we are
    // using a different uavcan instance than the first GPS, as we
    // send the data as broadcast on all DroneCAN device ports and we
    // don't want to send duplicates
    const uint32_t now_ms = AP_HAL::millis();
    if (_detected_module == 0 ||
        _detected_modules[_detected_module].ap_dronecan != _detected_modules[0].ap_dronecan ||
        now_ms - _detected_modules[0].last_inject_ms > 2000) {
        if (_rtcm_stream.buf == nullptr) {
            // give enough space for a full round from a NTRIP server with all
            // constellations
            _rtcm_stream.buf = new ByteBuffer(2400);
            if (_rtcm_stream.buf == nullptr) {
                return;
            }
        }
        _detected_modules[_detected_module].last_inject_ms = now_ms;
        _rtcm_stream.buf->write(data, len);
        send_rtcm();
    }
}

/*
    handle param get/set response
*/
bool AP_GPS_DroneCAN::handle_param_get_set_response_int(AP_DroneCAN* ap_dronecan, uint8_t node_id, const char* name, int32_t &value)
{
    Debug("AP_GPS_DroneCAN: param set/get response from %d %s %ld\n", node_id, name, value);
    if (((strcmp(name, "GPS_TYPE") == 0) || (strcmp(name, "GPS1_TYPE") == 0)) && (cfg_step == STEP_SET_TYPE)) {
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

bool AP_GPS_DroneCAN::handle_param_get_set_response_float(AP_DroneCAN* ap_dronecan, uint8_t node_id, const char* name, float &value)
{
    Debug("AP_GPS_DroneCAN: param set/get response from %d %s %f\n", node_id, name, value);
    return false;
}

void AP_GPS_DroneCAN::handle_param_save_response(AP_DroneCAN* ap_dronecan, const uint8_t node_id, bool success)
{
    Debug("AP_GPS_DroneCAN: param save response from %d %s\n", node_id, success ? "success" : "failure");

    if (cfg_step != STEP_SAVE_AND_REBOOT) {
        return;
    }

    if (success) {
        cfg_step++;
    }
    // Also send reboot command
    // this is ok as we are sending from DroneCAN thread context
    Debug("AP_GPS_DroneCAN: sending reboot command %d\n", node_id);
    ap_dronecan->send_reboot_request(node_id);
}

#if AP_DRONECAN_SEND_GPS
bool AP_GPS_DroneCAN::instance_exists(const AP_DroneCAN* ap_dronecan)
{
    for (uint8_t i=0; i<ARRAY_SIZE(_detected_modules); i++) {
        if (ap_dronecan == _detected_modules[i].ap_dronecan) {
            return true;
        }
    }
    return false;
}
#endif // AP_DRONECAN_SEND_GPS

#endif // AP_GPS_DRONECAN_ENABLED
