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
  Support for SensAItion serial connected INS and IMU
  Implements SensAItion protocol with ArduPilot-specific adaptations
*/

#include "AP_ExternalAHRS_SensAItion.h"

#if AP_EXTERNAL_AHRS_SENSAITION_ENABLED

#include <float.h>
#include <math.h>

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

namespace
{
const float MINIMUM_INTERESTING_BAROMETER_CHANGE_p = 1.0f;
const float MINIMUM_INTERESTING_TEMP_CHANGE_degc = 0.1f;
}

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_SensAItion::AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state),
    parser(AP_ExternalAHRS_SensAItion_Parser::ConfigMode::IMU)
{
    _ins_mode_enabled = option_is_set(static_cast<AP_ExternalAHRS::OPTIONS>(1U << 1));

    auto mode = _ins_mode_enabled ?
                AP_ExternalAHRS_SensAItion_Parser::ConfigMode::INTERLEAVED_INS :
                AP_ExternalAHRS_SensAItion_Parser::ConfigMode::IMU;

    if (_ins_mode_enabled) {
        parser = AP_ExternalAHRS_SensAItion_Parser(mode);
    }

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart || baudrate == 0 || port_num == -1) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "KEBNI: Serial Port Not Found!");
        return;
    }

    if (_ins_mode_enabled) {
        set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::IMU) |
                            uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                            uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                            uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));
    } else {
        set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::IMU));
    }

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SensAItion::update_thread, void),
            "AHRS_SensAItion", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "KEBNI: Failed to create thread!");
    }
}

int8_t AP_ExternalAHRS_SensAItion::get_port() const
{
    return uart ? port_num : -1;
}

const char* AP_ExternalAHRS_SensAItion::get_name() const
{
    return "Kebni SensAItion";
}

uint8_t AP_ExternalAHRS_SensAItion::num_gps_sensors() const
{
    return _ins_mode_enabled ? 1 : 0;
}

bool AP_ExternalAHRS_SensAItion::healthy() const
{
    WITH_SEMAPHORE(sem_handle);

    uint32_t now_ms = AP_HAL::millis();
    bool is_healthy = true;

    if ((now_ms - _last_imu_pkt_ms) > 160) {
        is_healthy = false;
    }

    if (is_healthy && _ins_mode_enabled) {
        const bool imu_available = _last_sensor_valid & 0x01;
        if ((now_ms - _last_ins_pkt_ms) > 400) {
            is_healthy = false;
        } else if (!imu_available) {
            is_healthy = false;
        } else if (_last_gnss1_fix < 3) {
            is_healthy = false;
        }
    }

    return is_healthy;
}

bool AP_ExternalAHRS_SensAItion::initialised() const
{
    WITH_SEMAPHORE(sem_handle);
    return setup_complete;
}

bool AP_ExternalAHRS_SensAItion::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion Unhealthy");
        return false;
    }

    {
        WITH_SEMAPHORE(sem_handle);
        if (_ins_mode_enabled) {
            if (_last_alignment_status != 1) {
                hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion Aligning");
                return false;
            }
        }
    }

    return true;
}

void AP_ExternalAHRS_SensAItion::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));

    status.flags.initalized = initialised();

    if (healthy()) {
        WITH_SEMAPHORE(sem_handle);
        if (_ins_mode_enabled && _last_alignment_status == 1) {
            status.flags.attitude = true;
            status.flags.horiz_pos_abs = true;
            status.flags.vert_pos = true;
            status.flags.horiz_vel = true;
            status.flags.vert_vel = true;
            status.flags.using_gps = true;
            status.flags.horiz_pos_rel = true;
            status.flags.pred_horiz_pos_abs = true;
            status.flags.pred_horiz_pos_rel = true;
        }
    }
}

// ---------------------------------------------------------------------------
// THREAD & PROFILER
// ---------------------------------------------------------------------------
void AP_ExternalAHRS_SensAItion::update_thread()
{
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay_microseconds(100);
        }
    }
}

bool AP_ExternalAHRS_SensAItion::check_uart()
{
    WITH_SEMAPHORE(sem_handle);

    if (!uart) {
        return false;
    }

    if (!setup_complete) {
        uart->begin(baudrate);
        setup_complete = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "KEBNI: INIT. Mode:%d Baud:%u",
                      (int)_ins_mode_enabled, (unsigned)baudrate);
    }
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }

    n = MIN(n, sizeof(buffer));
    ssize_t nread = uart->read(buffer, n);

    bool parsed_any = false;

    auto handler = [&](const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas) {
        uint32_t now_ms = AP_HAL::millis();

        if (meas.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::UNINITIALIZED) {
            return;
        }
        parsed_any = true;
        //
        if (meas.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::IMU) {
            handle_imu(meas, now_ms);
        } else if (meas.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::AHRS) {
            handle_ahrs(meas, now_ms);
        } else if (meas.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::INS) {
            handle_ins(meas, now_ms);
        }
    };

    if (nread > 0) {
        parser.parse_stream(buffer, nread, handler);
    }

    return parsed_any;
}

void AP_ExternalAHRS_SensAItion::handle_imu(const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas, uint32_t now_ms)
{
    // Time tag
    _last_imu_pkt_ms = now_ms;

    // STATE
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = meas.acceleration_mss;
        state.gyro = meas.angular_velocity_rads;
    }
    // INS
    {
        _ins.accel = meas.acceleration_mss;
        _ins.gyro = meas.angular_velocity_rads;
        _ins.temperature = meas.temperature_degc;
        //
        AP::ins().handle_external(_ins);
    }
    // COMPASS
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    {
        _mag.field = meas.magnetic_field_mgauss;
        //
        AP::compass().handle_external(_mag);
    }
#endif
    // BARO
#if AP_BARO_EXTERNALAHRS_ENABLED

    // ArduPlane has an internal check that triggers an error if there are too many barometer
    // readings with the same value. Therefore, we don't send them again unless there
    // has been a relevant change.
    const bool pressure_changed = fabsf(_baro.pressure_pa - meas.air_pressure_p) > MINIMUM_INTERESTING_BAROMETER_CHANGE_p;
    const bool temp_changed = fabsf(_baro.temperature - meas.temperature_degc) > MINIMUM_INTERESTING_TEMP_CHANGE_degc;
    if (pressure_changed || temp_changed) {
        _baro.instance = 0;
        _baro.pressure_pa = meas.air_pressure_p;
        _baro.temperature = meas.temperature_degc;
        //
        AP::baro().handle_external(_baro);
    }
#endif
}

void AP_ExternalAHRS_SensAItion::handle_ahrs(const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas, uint32_t now_ms)
{
    // Time tag
    _last_quat_pkt_ms = now_ms;
    // STATE
    {
        WITH_SEMAPHORE(state.sem);
        state.quat = meas.orientation;
        state.have_quaternion = true;
    }
}

void AP_ExternalAHRS_SensAItion::handle_ins(const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas, uint32_t now_ms)
{
    // Local data
    _last_ins_pkt_ms = now_ms;
    _last_alignment_status = meas.alignment_status;
    _last_sensor_valid = meas.sensor_valid;
    _last_gnss1_fix = meas.gnss1_fix;
    _last_gnss2_fix = meas.gnss2_fix;
    _last_error_flags = meas.error_flags;
    _last_h_pos_quality = meas.pos_accuracy.xy().length();
    _last_v_pos_quality = meas.pos_accuracy.z;
    _last_vel_quality = meas.vel_accuracy.length();
    // Log
    log_ins_status(meas);
    // STATE
    {
        WITH_SEMAPHORE(state.sem);
        state.location = Location(
                             meas.location.lat,
                             meas.location.lng,
                             meas.location.alt,
                             Location::AltFrame::ABSOLUTE
                         );
        state.velocity = meas.velocity_ned;
        state.have_location = true;
        state.have_velocity = true;
        state.last_location_update_us = AP_HAL::micros();

        if (!state.have_origin && meas.alignment_status) {
            state.origin = Location(
                               meas.location.lat,
                               meas.location.lng,
                               meas.location.alt,
                               Location::AltFrame::ABSOLUTE
                           );
            state.have_origin = true;
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "KEBNI: Origin Set.");
        }
    }
    // GPS
    {
        _gps.gps_week = meas.gps_week;
        _gps.ms_tow = meas.time_itow_ms;
        _gps.fix_type = AP_GPS_FixType(meas.gnss1_fix);
        _gps.satellites_in_view = meas.num_sats_gnss1;
        _gps.horizontal_pos_accuracy = _last_h_pos_quality;
        _gps.vertical_pos_accuracy = _last_v_pos_quality;
        _gps.horizontal_vel_accuracy = meas.vel_accuracy.xy().length();

        _gps.latitude = meas.location.lat;
        _gps.longitude = meas.location.lng;

        // Note: SensAItion reports altitude relative to WGS84, not MSL.
        // But we expect the user to reset the altitude to 0 at start,
        // so the absolute reference should not matter.
        _gps.msl_altitude = meas.location.alt;
        _gps.ned_vel_north = meas.velocity_ned.x;
        _gps.ned_vel_east = meas.velocity_ned.y;
        _gps.ned_vel_down = meas.velocity_ned.z;

        // 3. Estimate DOPs (Unitless) using assumed UERE of 3.0m
        // This answers "What is HDOP/VDOP?"
        const float ASSUMED_UERE = 3.0f;

        float est_hdop = _last_h_pos_quality / ASSUMED_UERE;
        float est_vdop = _last_v_pos_quality / ASSUMED_UERE;

        // 4. Sanity Clamping (DOP cannot be 0, and rarely < 0.6)
        if (est_hdop < 0.7f) {
            est_hdop = 0.7f;
        }
        if (est_vdop < 0.7f) {
            est_vdop = 0.7f;
        }
        _gps.hdop = est_hdop;
        _gps.vdop = est_vdop;

        // Handle
        uint8_t instance;
        if (AP::gps().get_first_external_instance(instance)) {
            AP::gps().handle_external(_gps, instance);
        }
    }
}

bool AP_ExternalAHRS_SensAItion::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    WITH_SEMAPHORE(sem_handle);
    if (_ins_mode_enabled && _last_alignment_status == 1) {
        posVar = _last_h_pos_quality * pos_gate_scale;
        velVar = _last_vel_quality * vel_gate_scale;
        hgtVar = _last_v_pos_quality * hgt_gate_scale;
        tasVar = 0; //not used
        return true;
    }

    return false;
}

void AP_ExternalAHRS_SensAItion::log_ins_status(const AP_ExternalAHRS_SensAItion_Parser::Measurement &meas)
{
#if HAL_LOGGING_ENABLED
    if (meas.type != AP_ExternalAHRS_SensAItion_Parser::MeasurementType::INS) {
        return;
    }

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    {
        WITH_SEMAPHORE(state.sem);
        if (state.have_quaternion) {
            state.quat.to_euler(roll_rad, pitch_rad, yaw_rad);
        }
    }
#endif
}

#endif // AP_EXTERNAL_AHRS_SENSAITION_ENABLED
