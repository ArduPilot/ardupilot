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
const uint32_t BARO_UPDATE_TIMEOUT_ms = 100U;
}

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_SensAItion::AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state),
    parser(AP_ExternalAHRS_SensAItion_Parser::ConfigMode::IMU)
{
    ins_mode_enabled = option_is_set(AP_ExternalAHRS::OPTIONS::SENSAITION_INS);

    auto mode = ins_mode_enabled ?
                AP_ExternalAHRS_SensAItion_Parser::ConfigMode::INTERLEAVED_INS :
                AP_ExternalAHRS_SensAItion_Parser::ConfigMode::IMU;

    if (ins_mode_enabled) {
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

    if (ins_mode_enabled) {
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
    return ins_mode_enabled ? 1 : 0;
}

bool AP_ExternalAHRS_SensAItion::healthy() const
{
    WITH_SEMAPHORE(sem_handle);

    const uint32_t now_ms = AP_HAL::millis();

    if ((now_ms - last_imu_pkt_ms) > 160) {
        // IMU is required at high rate for health
        return false;
    }

    if (ins_mode_enabled) {
        if ((now_ms - last_ins_pkt_ms) > 400) {
            // INS packets must also have high enough rate
            return false;
        }

        if (!(last_sensor_valid & 0x01)) {
            // IMU not available
            return false;
        }

        if (last_gnss1_fix < 3) {
            // No 3D satellite fix
            return false;
        }
    }

    return true;
}

bool AP_ExternalAHRS_SensAItion::initialised() const
{
    return setup_complete;
}

bool AP_ExternalAHRS_SensAItion::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion Unhealthy");
        return false;
    }

    if (ins_mode_enabled) {
        WITH_SEMAPHORE(sem_handle);
        if (last_alignment_status != 1) {
            hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion Aligning");
            return false;
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
        if (ins_mode_enabled && last_alignment_status == 1) {
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
            hal.scheduler->delay_microseconds(500);
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
                      (int)ins_mode_enabled, (unsigned)baudrate);
    }

    const auto nread = uart->read(buffer, sizeof(buffer));
    if (nread <= 0) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();

    AP_ExternalAHRS_SensAItion_Parser::Measurement meas;
    bool found_measurement = false;
    ssize_t parsed_bytes = 0;
    while (parsed_bytes < nread) {
        const size_t max_bytes_to_parse = nread - parsed_bytes;
        parsed_bytes += parser.parse_stream(&buffer[parsed_bytes], max_bytes_to_parse, meas);

        switch (meas.type) {
        case AP_ExternalAHRS_SensAItion_Parser::MeasurementType::UNINITIALIZED:
            break;
        case AP_ExternalAHRS_SensAItion_Parser::MeasurementType::IMU:
            handle_imu(meas, now_ms);
            found_measurement = true;
            break;
        case AP_ExternalAHRS_SensAItion_Parser::MeasurementType::AHRS:
            handle_ahrs(meas, now_ms);
            found_measurement = true;
            break;
        case AP_ExternalAHRS_SensAItion_Parser::MeasurementType::INS:
            handle_ins(meas, now_ms);
            found_measurement = true;
            break;
        }
    }

    return found_measurement;
}

void AP_ExternalAHRS_SensAItion::handle_imu(const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas, uint32_t now_ms)
{
    last_imu_pkt_ms = now_ms;

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = meas.acceleration_mss;
        state.gyro = meas.angular_velocity_rads;
    }

    // INS
    ins.accel = meas.acceleration_mss;
    ins.gyro = meas.angular_velocity_rads;
    ins.temperature = meas.temperature_degc;
    AP::ins().handle_external(ins);

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    // COMPASS
    mag.field = meas.magnetic_field_mgauss;
    AP::compass().handle_external(mag);
#endif

#if AP_BARO_EXTERNALAHRS_ENABLED
    // BARO
    // ArduPlane has an internal check that triggers an error if there are too many barometer
    // readings with the same value. At high sampling rates, we triggered that check because
    // the barometer is internally sampled at a lower rate. To avoid that, we only update the value
    // if it changes OR a certain minimum time has passed.
    const bool pressure_changed = fabsf(baro.pressure_pa - meas.air_pressure_p) > MINIMUM_INTERESTING_BAROMETER_CHANGE_p;
    const bool temp_changed = fabsf(baro.temperature - meas.temperature_degc) > MINIMUM_INTERESTING_TEMP_CHANGE_degc;
    const bool timeout = now_ms > last_baro_update_ms + BARO_UPDATE_TIMEOUT_ms;
    if (pressure_changed || temp_changed || timeout) {
        last_baro_update_ms = now_ms;
        baro.instance = 0;
        baro.pressure_pa = meas.air_pressure_p;
        baro.temperature = meas.temperature_degc;
        AP::baro().handle_external(baro);
    }
#endif
}

void AP_ExternalAHRS_SensAItion::handle_ahrs(const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas, uint32_t now_ms)
{
    last_quat_pkt_ms = now_ms;

    {
        WITH_SEMAPHORE(state.sem);
        state.quat = meas.orientation;
        state.have_quaternion = true;
    }
}

void AP_ExternalAHRS_SensAItion::handle_ins(const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas, uint32_t now_ms)
{
#if HAL_LOGGING_ENABLED
    log_ins_status(meas);
#endif

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

    // Local data
    last_ins_pkt_ms = now_ms;
    last_alignment_status = meas.alignment_status;
    last_sensor_valid = meas.sensor_valid;
    last_gnss1_fix = meas.gnss1_fix;
    last_gnss2_fix = meas.gnss2_fix;
    last_error_flags = meas.error_flags;
    last_h_pos_quality = meas.pos_accuracy.xy().length();
    last_v_pos_quality = meas.pos_accuracy.z;
    last_vel_quality = meas.vel_accuracy.length();

    // GPS
    gps.gps_week = meas.gps_week;
    gps.ms_tow = meas.time_itow_ms;
    gps.fix_type = AP_GPS_FixType(meas.gnss1_fix);
    gps.satellites_in_view = meas.num_sats_gnss1;
    gps.horizontal_pos_accuracy = last_h_pos_quality;
    gps.vertical_pos_accuracy = last_v_pos_quality;
    gps.horizontal_vel_accuracy = meas.vel_accuracy.xy().length();

    gps.latitude = meas.location.lat;
    gps.longitude = meas.location.lng;

    // Note: SensAItion reports altitude relative to WGS84, not MSL.
    // But we expect the user to reset the altitude to 0 at start,
    // so the absolute reference should not matter.
    gps.msl_altitude = meas.location.alt;
    gps.ned_vel_north = meas.velocity_ned.x;
    gps.ned_vel_east = meas.velocity_ned.y;
    gps.ned_vel_down = meas.velocity_ned.z;

    // 3. Estimate DOPs (Unitless) using assumed UERE of 3.0m
    // This answers "What is HDOP/VDOP?"
    const float ASSUMED_UERE = 3.0f;

    float est_hdop = last_h_pos_quality / ASSUMED_UERE;
    float est_vdop = last_v_pos_quality / ASSUMED_UERE;

    // 4. Sanity Clamping (DOP cannot be 0, and rarely < 0.6)
    if (est_hdop < 0.7f) {
        est_hdop = 0.7f;
    }
    if (est_vdop < 0.7f) {
        est_vdop = 0.7f;
    }
    gps.hdop = est_hdop;
    gps.vdop = est_vdop;

    // Handle
    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps, instance);
    }
}

bool AP_ExternalAHRS_SensAItion::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    WITH_SEMAPHORE(sem_handle);
    if (ins_mode_enabled && last_alignment_status == 1) {
        posVar = last_h_pos_quality * pos_gate_scale;
        velVar = last_vel_quality * vel_gate_scale;
        hgtVar = last_v_pos_quality * hgt_gate_scale;
        tasVar = 0; //not used
        return true;
    }

    return false;
}

#if HAL_LOGGING_ENABLED
void AP_ExternalAHRS_SensAItion::log_ins_status(const AP_ExternalAHRS_SensAItion_Parser::Measurement &meas)
{
    if (meas.type != AP_ExternalAHRS_SensAItion_Parser::MeasurementType::INS) {
        return;
    }

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    WITH_SEMAPHORE(state.sem);
    if (state.have_quaternion) {
        state.quat.to_euler(roll_rad, pitch_rad, yaw_rad);
    }
}
#endif

#endif // AP_EXTERNAL_AHRS_SENSAITION_ENABLED
