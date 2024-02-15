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
  support for MicroStrain CX5/GX5-45 serially connected AHRS Systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED

#include "AP_ExternalAHRS_MicroStrain5.h"
#include "AP_Compass/AP_Compass_config.h"
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL &hal;

static constexpr uint8_t gnss_instance = 0;

AP_ExternalAHRS_MicroStrain5::AP_ExternalAHRS_MicroStrain5(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "MicroStrain5 ExternalAHRS no UART");
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_MicroStrain5::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("MicroStrain5 failed to allocate ExternalAHRS update thread");
    }

    hal.scheduler->delay(5000);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MicroStrain5 ExternalAHRS initialised");
}

void AP_ExternalAHRS_MicroStrain5::update_thread(void)
{
    if (!port_open) {
        port_open = true;
        uart->begin(baudrate);
    }

    while (true) {
        build_packet();
        hal.scheduler->delay_microseconds(100);
    }
}



// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_MicroStrain5::build_packet()
{
    if (uart == nullptr) {
        return;
    }
    
    WITH_SEMAPHORE(sem);
    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes--> 0) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        DescriptorSet descriptor;
        if (handle_byte(b, descriptor)) {
            switch (descriptor) {
            case DescriptorSet::IMUData:
                post_imu();
                break;
            case DescriptorSet::GNSSData:
            case DescriptorSet::GNSSRecv1:
            case DescriptorSet::GNSSRecv2:
                break;
            case DescriptorSet::FilterData:
                post_filter();
                break;
            case DescriptorSet::BaseCommand:
            case DescriptorSet::DMCommand:
            case DescriptorSet::SystemCommand:
                break;
            }
        }
    }
}



// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_MicroStrain5::post_imu() const
{
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = imu_data.accel;
        state.gyro = imu_data.gyro;

        state.quat = imu_data.quat;
        state.have_quaternion = true;
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins {
            accel: imu_data.accel,
            gyro: imu_data.gyro,
            temperature: -300
        };
        AP::ins().handle_external(ins);
    }

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    {
        AP_ExternalAHRS::mag_data_message_t mag {
            field: imu_data.mag
        };
        AP::compass().handle_external(mag);
    }
#endif

#if AP_BARO_EXTERNALAHRS_ENABLED
    {
        const AP_ExternalAHRS::baro_data_message_t baro {
            instance: 0,
            pressure_pa: imu_data.pressure,
            // setting temp to 25 effectively disables barometer temperature calibrations - these are already performed by MicroStrain
            temperature: 25,
        };        
        AP::baro().handle_external(baro);
    }
#endif
}

void AP_ExternalAHRS_MicroStrain5::post_filter() const
{
    {
        WITH_SEMAPHORE(state.sem);
        state.velocity = Vector3f{filter_data.ned_velocity_north, filter_data.ned_velocity_east, filter_data.ned_velocity_down};
        state.have_velocity = true;

        state.location = Location{filter_data.lat, filter_data.lon, gnss_data[gnss_instance].msl_altitude, Location::AltFrame::ABSOLUTE};
        state.have_location = true;
        state.last_location_update_us = AP_HAL::micros();
    }

    AP_ExternalAHRS::gps_data_message_t gps {
        gps_week: filter_data.week,
        ms_tow: filter_data.tow_ms,
        fix_type: (uint8_t) gnss_data[gnss_instance].fix_type,
        satellites_in_view: gnss_data[gnss_instance].satellites,

        horizontal_pos_accuracy: gnss_data[gnss_instance].horizontal_position_accuracy,
        vertical_pos_accuracy: gnss_data[gnss_instance].vertical_position_accuracy,
        horizontal_vel_accuracy: gnss_data[gnss_instance].speed_accuracy,

        hdop: gnss_data[gnss_instance].hdop,
        vdop: gnss_data[gnss_instance].vdop,

        longitude: filter_data.lon,
        latitude: filter_data.lat,
        msl_altitude: gnss_data[gnss_instance].msl_altitude,

        ned_vel_north: filter_data.ned_velocity_north,
        ned_vel_east: filter_data.ned_velocity_east,
        ned_vel_down: filter_data.ned_velocity_down,
    };

    if (gps.fix_type >= 3 && !state.have_origin) {
        WITH_SEMAPHORE(state.sem);
        state.origin = Location{int32_t(filter_data.lat),
                                int32_t(filter_data.lon),
                                int32_t(gnss_data[gnss_instance].msl_altitude),
                                Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }

    uint8_t gps_instance;
    if (AP::gps().get_first_external_instance(gps_instance)) {
        AP::gps().handle_external(gps, gps_instance);
    }
}

int8_t AP_ExternalAHRS_MicroStrain5::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

// Get model/type name
const char* AP_ExternalAHRS_MicroStrain5::get_name() const
{
    return "MicroStrain5";
}

bool AP_ExternalAHRS_MicroStrain5::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_imu_pkt < 40 && now - last_gps_pkt < 500 && now - last_filter_pkt < 500);
}

bool AP_ExternalAHRS_MicroStrain5::initialised(void) const
{
    return last_imu_pkt != 0 && last_gps_pkt != 0 && last_filter_pkt != 0;
}

bool AP_ExternalAHRS_MicroStrain5::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "MicroStrain5 unhealthy");
        return false;
    }
    if (gnss_data[gnss_instance].fix_type < 3) {
        hal.util->snprintf(failure_msg, failure_msg_len, "MicroStrain5 no GPS lock");
        return false;
    }
    if (filter_status.state != 0x02) {
        hal.util->snprintf(failure_msg, failure_msg_len, "MicroStrain5 filter not running");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_MicroStrain5::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (last_imu_pkt != 0 && last_gps_pkt != 0) {
        status.flags.initalized = true;
    }
    if (healthy() && last_imu_pkt != 0) {
        status.flags.attitude = true;
        status.flags.vert_vel = true;
        status.flags.vert_pos = true;

        if (gnss_data[gnss_instance].fix_type >= 3) {
            status.flags.horiz_vel = true;
            status.flags.horiz_pos_rel = true;
            status.flags.horiz_pos_abs = true;
            status.flags.pred_horiz_pos_rel = true;
            status.flags.pred_horiz_pos_abs = true;
            status.flags.using_gps = true;
        }
    }
}

void AP_ExternalAHRS_MicroStrain5::send_status_report(GCS_MAVLINK &link) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    const float vel_gate = 4; // represents hz value data is posted at
    const float pos_gate = 4; // represents hz value data is posted at
    const float hgt_gate = 4; // represents hz value data is posted at
    const float mag_var = 0; //we may need to change this to be like the other gates, set to 0 because mag is ignored by the ins filter in vectornav
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       gnss_data[gnss_instance].speed_accuracy/vel_gate, gnss_data[gnss_instance].horizontal_position_accuracy/pos_gate, gnss_data[gnss_instance].vertical_position_accuracy/hgt_gate,
                                       mag_var, 0, 0);

}


#endif // AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED
