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

//   Usage in SITL with hardware for debugging:
//     $ sim_vehicle.py -v Plane --console --map -DG
//     param set AHRS_EKF_TYPE 11
//     param set EAHRS_TYPE 6
//     For EAHRS as a GPS:
//     param set GPS1_TYPE 21
//   Configure GSOF 49,50,70 on UDP port 44444, "UDP Mode" and "UDP Broadcast Transmit"
//   Consider setting EK3_SRC1_YAW to 2 on the bench...

// Usage with NET parameters and ethernet in SITL with hardware:
//     param set NET_ENABLE 1
//     param set NET_P1_TYPE 2
//     # Set up AHRS input
//     param set NET_P1_PROTOCOL 36
//     param set SIM_GPS1_TYPE 0
//     param set NET_P1_PORT 44444
//     param set EAHRS_TYPE 6
//
// Usage with serial in SITL with hardware:
//     $ sim_vehicle.py -v Plane -A "--serial3=uart:/dev/ttyUSB0" --console --map -DG
//     param set SERIAL3_PROTOCOL 36
//     param set SERIAL3_BAUD 115200
//     param set EAHRS_TYPE 6
//     # ensure NET_* if off if you were using ethernet.
//     # To enable the EAHRS to provide GPS:
//     param set GPS2_TYPE 21

// On most hardware, you must enable EAHRS:
//     ./waf configure --board Pixhawk6X --enable-AHRS_EXT

// GPS ride-along with EARHS as the 2nd GPS
//      param set GPS_AUTO_SWITCH 0
//      param set GPS2_TYPE 21
//      param set GPS_PRIMARY 0

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_GSOF_ENABLED

#include "AP_ExternalAHRS_GSOF.h"
#include "AP_Compass/AP_Compass_config.h"
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/Socket.h>

static const char* LOG_FMT = "%s ExternalAHRS: %s";

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_GSOF::AP_ExternalAHRS_GSOF(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, LOG_FMT, get_name(), "no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    pktbuf = NEW_NOTHROW uint8_t[AP_GSOF::MAX_PACKET_SIZE];

    if (!pktbuf) {
        AP_BoardConfig::allocation_error("GSOF ExternalAHRS");
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_GSOF::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("GSOF ExternalAHRS failed to allocate ExternalAHRS update thread");
    }

    // Offer GPS even through it's a tightly coupled EKF.
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS));

    hal.scheduler->delay(5000);
    if (!initialised()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, LOG_FMT, get_name(), "missing data within 5s.");
    }
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_GSOF::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

void AP_ExternalAHRS_GSOF::update_thread(void)
{
    // TODO configure receiver to output expected data.

    auto last_debug = AP_HAL::millis();
    size_t pps = 0;

    uart->begin(baudrate);
    
    while (true) {
        // TODO should we ever call begin() again?
        while (uart->available() > 0) {
            const uint8_t c = uart->read();

            AP_GSOF::MsgTypes parsed;
            const auto parse_res = parse(c, parsed);
            if (parse_res != PARSED_GSOF_DATA) {
                continue;
            }

            auto const now = AP_HAL::millis();
            pps++;

            if (now - last_debug > 1000) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "GSOF PPS: %lu, ins %u",
                    static_cast<unsigned long>(pps),
                    static_cast<uint16_t>(ins_full_nav.gnss_status));
                last_debug = now;
                pps = 0;
            }
            if (parsed.get(AP_GSOF::POS_TIME)) {
                last_pos_time_ms = now;

                gps_data.satellites_in_view = pos_time.num_sats;
            }

            if (parsed.get(AP_GSOF::INS_FULL_NAV)) {
                last_ins_full_nav_ms = now;

                gps_data.gps_week = ins_full_nav.gps_week;
                gps_data.ms_tow = ins_full_nav.gps_time_ms;
                gps_data.ned_vel_north = ins_full_nav.vel_n;
                gps_data.ned_vel_east = ins_full_nav.vel_e;
                gps_data.ned_vel_down = ins_full_nav.vel_d;
                switch(ins_full_nav.gnss_status) {
                    case AP_GSOF::GnssStatus::FIX_NOT_AVAILABLE:
                    case AP_GSOF::GnssStatus::GNSS_SPS_MODE: // TODO Is this right?
                        gps_data.fix_type = AP_GPS_FixType::NONE;
                        break;
                    case AP_GSOF::GnssStatus::DGPS_SPS_MODE:
                        gps_data.fix_type = AP_GPS_FixType::DGPS;
                        break;
                    case AP_GSOF::GnssStatus::GNSS_PPS_MODE:
                        gps_data.fix_type = AP_GPS_FixType::PPP;
                        break;
                    case AP_GSOF::GnssStatus::FIXED_RTK_MODE:
                        gps_data.fix_type = AP_GPS_FixType::RTK_FIXED;
                        break;
                    case AP_GSOF::GnssStatus::FLOAT_RTK_MODE:
                        gps_data.fix_type = AP_GPS_FixType::RTK_FLOAT;
                        break;
                    case AP_GSOF::GnssStatus::DR_MODE:
                        gps_data.fix_type = AP_GPS_FixType::NONE;
                        break;
                }
                if (ins_full_nav.gnss_status != AP_GSOF::GnssStatus::FIX_NOT_AVAILABLE) {
                    // If fix is unavailble, the lat/lon may be zero, so don't post the data.
                    // To reproduce this condition, disconnect the GPS antenna and reboot the receiver.
                    post_filter();
                }
            }
            if (parsed.get(AP_GSOF::INS_RMS)) {
                last_ins_rms_ms = now;

                gps_data.horizontal_pos_accuracy = Vector2d(ins_rms.pos_rms_n, ins_rms.pos_rms_e).length();
                gps_data.vertical_pos_accuracy = ins_rms.pos_rms_d;
                gps_data.horizontal_vel_accuracy = Vector2d(ins_rms.vel_rms_n, ins_rms.vel_rms_e).length();
            }
            if (parsed.get(AP_GSOF::LLH_MSL)) {
                last_llh_msl_ms = now;

                gps_data.longitude = static_cast<int32_t>(llh_msl.longitude * 1E7);
                gps_data.latitude = static_cast<int32_t>(llh_msl.latitude * 1E7);
                gps_data.msl_altitude = static_cast<int32_t>(llh_msl.altitude_msl * 1E2);
            }

            if (parsed.get(AP_GSOF::LLH_MSL) && parsed.get(AP_GSOF::INS_FULL_NAV)) {
                undulation = ins_full_nav.altitude - llh_msl.altitude_msl;
            }

            uint8_t instance;
            AP_GSOF::MsgTypes expected_gps;
            expected_gps.set(AP_GSOF::POS_TIME);
            expected_gps.set(AP_GSOF::INS_FULL_NAV);
            expected_gps.set(AP_GSOF::INS_RMS);
            expected_gps.set(AP_GSOF::LLH_MSL);
            if (AP::gps().get_first_external_instance(instance) && parsed == expected_gps) {
                AP::gps().handle_external(gps_data, instance);
            }
        }
        hal.scheduler->delay_microseconds(100);
        check_initialise_state();
    }
}

void AP_ExternalAHRS_GSOF::check_initialise_state(void)
{
    const auto new_init_state = initialised();
    // Only send the message after fully booted up, otherwise it gets dropped.
    if (!last_init_state && new_init_state && AP_HAL::millis() > 5000) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, LOG_FMT, get_name(), "initialised.");
        last_init_state = new_init_state;
    }
}

void AP_ExternalAHRS_GSOF::post_filter() const
{
    WITH_SEMAPHORE(state.sem);
    state.velocity = Vector3f{ins_full_nav.vel_n, ins_full_nav.vel_e, ins_full_nav.vel_d};
    state.have_velocity = true;

    state.location = Location(
        ins_full_nav.latitude * 1E7,
        ins_full_nav.longitude * 1E7,
        (ins_full_nav.altitude - undulation) * 1E2,
        Location::AltFrame::ABSOLUTE);
    state.have_location = true;

    state.quat.from_euler(
        radians(ins_full_nav.roll_deg), 
        radians(ins_full_nav.pitch_deg),
        radians(ins_full_nav.heading_deg));
    state.have_quaternion = true;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (!state.location.initialised()) {
            AP_HAL::panic("Uninitialized location.");
        }
#endif

    if (!state.have_origin && filter_healthy()) {
        state.origin = state.location;
        state.have_origin = true;
    }

    state.last_location_update_us = AP_HAL::micros();
}

// Get model/type name
const char* AP_ExternalAHRS_GSOF::get_name() const
{
    return "GSOF";
}

bool AP_ExternalAHRS_GSOF::healthy(void) const
{
    return times_healthy() && filter_healthy();
}

bool AP_ExternalAHRS_GSOF::initialised(void) const
{
    return last_pos_time_ms != 0 && last_ins_full_nav_ms != 0 && last_ins_rms_ms != 0 && last_llh_msl_ms != 0;
}

bool AP_ExternalAHRS_GSOF::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!initialised()) {
        hal.util->snprintf(failure_msg, failure_msg_len, LOG_FMT, get_name(), "not initialised");
        return false;
    }
    if (!times_healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, LOG_FMT, get_name(), "data is stale");
        return false;
    }
    if (!filter_healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, LOG_FMT, get_name(), "filter is unhealthy");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, LOG_FMT, get_name(), "unhealthy");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_GSOF::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.initalized = initialised();
}

bool AP_ExternalAHRS_GSOF::times_healthy() const
{
    auto const now = AP_HAL::millis();
    
    auto const TIMES_FOS = 2.0;

    // All messages must be at minimum 5Hz to pass the AP_GPS 4hz rate as logged in dataflash GPA.Delta logs.

    // 5Hz = 200mS.
    auto const GSOF_1_EXPECTED_DELAY_MS = 200;
    auto const pos_time_healthy = now - last_pos_time_ms <= TIMES_FOS * GSOF_1_EXPECTED_DELAY_MS;

    // 50Hz = 20mS.
    auto const GSOF_49_EXPECTED_DELAY_MS = 20;
    auto const ins_full_nav_healthy = now - last_ins_full_nav_ms <= TIMES_FOS * GSOF_49_EXPECTED_DELAY_MS;

    // 5Hz = 200mS.
    auto const GSOF_50_EXPECTED_DELAY_MS = 200;
    auto const ins_rms_healthy = now - last_ins_rms_ms < TIMES_FOS * GSOF_50_EXPECTED_DELAY_MS;

    // 5Hz = 200mS.
    auto const GSOF_70_EXPECTED_DELAY_MS = 200;
    auto const llh_msl_healthy = now - last_llh_msl_ms < TIMES_FOS * GSOF_70_EXPECTED_DELAY_MS;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!pos_time_healthy) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s: GSOF pos time delayed by %lu ms", get_name(), uint64_t(now - last_pos_time_ms));
    }
    if (!ins_full_nav_healthy) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s: INS Full nav delayed by %lu ms", get_name(), uint64_t(now - last_ins_full_nav_ms));
    }
    if (!ins_rms_healthy) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s: INS rms delayed by %lu ms", get_name(), uint64_t(now - last_ins_rms_ms));
    }
    if (!llh_msl_healthy) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s: LLH MSL delayed by %lu ms", get_name(), uint64_t(now - last_llh_msl_ms));
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

    return pos_time_healthy && ins_full_nav_healthy && ins_rms_healthy && llh_msl_healthy;
}

bool AP_ExternalAHRS_GSOF::filter_healthy() const
{
    // TODO get the right threshold from Trimble for arming vs in flight.
    // Fow now, assume an aligned IMU is sufficient for flight.
    auto const imu_alignment_healthy = (
        ins_rms.imu_alignment_status == ImuAlignmentStatus::DEGRADED ||
        ins_rms.imu_alignment_status == ImuAlignmentStatus::ALIGNED ||
        ins_rms.imu_alignment_status == ImuAlignmentStatus::FULL_NAV
    );

    auto const gnss_healthy = ins_rms.gnss_status != GnssStatus::FIX_NOT_AVAILABLE;

    // TODO check RMS errors.
    return imu_alignment_healthy && gnss_healthy;
}

#endif // AP_EXTERNAL_AHRS_GSOF_ENABLED
