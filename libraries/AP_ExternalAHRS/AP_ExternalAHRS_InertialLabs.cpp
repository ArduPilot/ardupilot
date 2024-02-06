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
  support for serial connected InertialLabs INS
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED

#include "AP_ExternalAHRS_InertialLabs.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Common/Bitmask.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

// unit status bits
#define ILABS_UNIT_STATUS_ALIGNMENT_FAIL   0x0001
#define ILABS_UNIT_STATUS_OPERATION_FAIL   0x0002
#define ILABS_UNIT_STATUS_GYRO_FAIL        0x0004
#define ILABS_UNIT_STATUS_ACCEL_FAIL       0x0008
#define ILABS_UNIT_STATUS_MAG_FAIL         0x0010
#define ILABS_UNIT_STATUS_ELECTRONICS_FAIL 0x0020
#define ILABS_UNIT_STATUS_GNSS_FAIL        0x0040
#define ILABS_UNIT_STATUS_RUNTIME_CAL      0x0080
#define ILABS_UNIT_STATUS_VOLTAGE_LOW      0x0100
#define ILABS_UNIT_STATUS_VOLTAGE_HIGH     0x0200
#define ILABS_UNIT_STATUS_X_RATE_HIGH      0x0400
#define ILABS_UNIT_STATUS_Y_RATE_HIGH      0x0800
#define ILABS_UNIT_STATUS_Z_RATE_HIGH      0x1000
#define ILABS_UNIT_STATUS_MAG_FIELD_HIGH   0x2000
#define ILABS_UNIT_STATUS_TEMP_RANGE_ERR   0x4000
#define ILABS_UNIT_STATUS_RUNTIME_CAL2     0x8000

// unit status2 bits
#define ILABS_UNIT_STATUS2_ACCEL_X_HIGH           0x0001
#define ILABS_UNIT_STATUS2_ACCEL_Y_HIGH           0x0002
#define ILABS_UNIT_STATUS2_ACCEL_Z_HIGH           0x0004
#define ILABS_UNIT_STATUS2_BARO_FAIL              0x0008
#define ILABS_UNIT_STATUS2_DIFF_PRESS_FAIL        0x0010
#define ILABS_UNIT_STATUS2_MAGCAL_2D_ACT          0x0020
#define ILABS_UNIT_STATUS2_MAGCAL_3D_ACT          0x0020
#define ILABS_UNIT_STATUS2_GNSS_FUSION_OFF        0x0040
#define ILABS_UNIT_STATUS2_DIFF_PRESS_FUSION_OFF  0x0080
#define ILABS_UNIT_STATUS2_MAG_FUSION_OFF         0x0100
#define ILABS_UNIT_STATUS2_GNSS_POS_VALID         0x0200

// air data status bits
#define ILABS_AIRDATA_INIT_FAIL                   0x0001
#define ILABS_AIRDATA_DIFF_PRESS_INIT_FAIL        0x0002
#define ILABS_AIRDATA_STATIC_PRESS_FAIL           0x0004
#define ILABS_AIRDATA_DIFF_PRESS_FAIL             0x0008
#define ILABS_AIRDATA_STATIC_PRESS_RANGE_ERR      0x0010
#define ILABS_AIRDATA_DIFF_PRESS_RANGE_ERR        0x0020
#define ILABS_AIRDATA_PRESS_ALT_FAIL              0x0040
#define ILABS_AIRDATA_AIRSPEED_FAIL               0x0080
#define ILABS_AIRDATA_BELOW_THRESHOLD             0x0100


// constructor
AP_ExternalAHRS_InertialLabs::AP_ExternalAHRS_InertialLabs(AP_ExternalAHRS *_frontend,
                                                           AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "InertialLabs ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    // don't offer IMU by default, at 200Hz it is too slow for many aircraft
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::IMU) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));
    
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_InertialLabs::update_thread, void), "ILabs", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("InertialLabs Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs ExternalAHRS initialised");
}

/*
  re-sync buffer on parse failure
 */
void AP_ExternalAHRS_InertialLabs::re_sync(void)
{
    if (buffer_ofs > 3) {
        /*
          look for the 2 byte header and try to sync to that
         */
        const uint16_t header = 0x55AA;
        const uint8_t *p = (const uint8_t *)memmem(&buffer[1], buffer_ofs-3, &header, sizeof(header));
        if (p != nullptr) {
            const uint16_t n = p - &buffer[0];
            memmove(&buffer[0], p, buffer_ofs - n);
            buffer_ofs -= n;
        } else {
            buffer_ofs = 0;
        }
    } else {
        buffer_ofs = 0;
    }
}

// macro for checking we don't run past end of message buffer
#define CHECK_SIZE(d) need_re_sync = (message_ofs + (msg_len=sizeof(d)) > buffer_end); if (need_re_sync) break

// lookup a message in the msg_types bitmask to see if we received it in this packet
#define GOT_MSG(mtype) msg_types.get(unsigned(MessageType::mtype))

/*
  check header is valid
 */
bool AP_ExternalAHRS_InertialLabs::check_header(const ILabsHeader *h) const
{
    return h->magic == 0x55AA &&
        h->msg_type == 1 &&
        h->msg_id == 0x95 &&
        h->msg_len <= sizeof(buffer)-2;
}

/*
  check the UART for more data
  returns true if we have consumed potentially valid bytes
 */
bool AP_ExternalAHRS_InertialLabs::check_uart()
{
    WITH_SEMAPHORE(state.sem);

    if (!setup_complete) {
        return false;
    }
    // ensure we own the uart
    uart->begin(0);
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }
    if (n + buffer_ofs > sizeof(buffer)) {
        n = sizeof(buffer) - buffer_ofs;
    }
    const ILabsHeader *h = (ILabsHeader *)&buffer[0];

    if (buffer_ofs < sizeof(ILabsHeader)) {
        n = MIN(n, sizeof(ILabsHeader)-buffer_ofs);
    } else {
        if (!check_header(h)) {
            re_sync();
            return false;
        }
        if (buffer_ofs > h->msg_len+8) {
            re_sync();
            return false;
        }
        n = MIN(n, uint32_t(h->msg_len + 2 - buffer_ofs));
    }

    const ssize_t nread = uart->read(&buffer[buffer_ofs], n);
    if (nread != ssize_t(n)) {
        re_sync();
        return false;
    }
    buffer_ofs += n;

    if (buffer_ofs < sizeof(ILabsHeader)) {
        return true;
    }

    if (!check_header(h)) {
        re_sync();
        return false;
    }

    if (buffer_ofs < h->msg_len+2) {
        /*
          see if we can read the rest immediately
         */
        const uint16_t needed = h->msg_len+2 - buffer_ofs;
        if (uart->available() < needed) {
            // need more data
            return true;
        }
        const ssize_t nread2 = uart->read(&buffer[buffer_ofs], needed);
        if (nread2 != needed) {
            re_sync();
            return false;
        }
        buffer_ofs += nread2;
    }

    // check checksum
    const uint16_t crc1 = crc_sum_of_bytes_16(&buffer[2], buffer_ofs-4);
    const uint16_t crc2 = le16toh_ptr(&buffer[buffer_ofs-2]);
    if (crc1 != crc2) {
        re_sync();
        return false;
    }

    const uint8_t *buffer_end = &buffer[buffer_ofs];
    const uint16_t payload_size = h->msg_len - 6;
    const uint8_t *payload = &buffer[6];
    if (payload_size < 3) {
        re_sync();
        return false;
    }
    const uint8_t num_messages = payload[0];
    if (num_messages == 0 ||
        num_messages > payload_size-1) {
        re_sync();
        return false;
    }
    const uint8_t *message_ofs = &payload[num_messages+1];
    bool need_re_sync = false;

    // bitmask for what types we get
    Bitmask<256> msg_types;
    uint32_t now_ms = AP_HAL::millis();

    for (uint8_t i=0; i<num_messages; i++) {
        if (message_ofs >= buffer_end) {
            re_sync();
            return false;
        }
        MessageType mtype = (MessageType)payload[1+i];
        ILabsData &u = *(ILabsData*)message_ofs;
        uint8_t msg_len = 0;

        msg_types.set(unsigned(mtype));

        switch (mtype) {
        case MessageType::GPS_INS_TIME_MS: {
            // this is the GPS tow timestamp in ms for when the IMU data was sampled
            CHECK_SIZE(u.gps_time_ms);
            state2.gnss_ins_time_ms = u.gps_time_ms;
            break;
        }
        case MessageType::GPS_WEEK: {
            CHECK_SIZE(u.gps_week);
            gps_data.gps_week = u.gps_week;
            break;
        }
        case MessageType::ACCEL_DATA_HR: {
            CHECK_SIZE(u.accel_data_hr);
            ins_data.accel = u.accel_data_hr.tofloat().rfu_to_frd()*9.8106f*1.0e-6;      
            break;
        }
        case MessageType::GYRO_DATA_HR: {
            CHECK_SIZE(u.gyro_data_hr);
            ins_data.gyro = u.gyro_data_hr.tofloat().rfu_to_frd()*DEG_TO_RAD*1.0e-5;
            break;
        }
        case MessageType::BARO_DATA: {
            CHECK_SIZE(u.baro_data);
            baro_data.pressure_pa = u.baro_data.pressure_pa2*2;
            break;
        }
        case MessageType::MAG_DATA: {
            CHECK_SIZE(u.mag_data);
            mag_data.field = u.mag_data.tofloat().rfu_to_frd()*(10*NTESLA_TO_MGAUSS);
            break;
        }
        case MessageType::ORIENTATION_ANGLES: {
            CHECK_SIZE(u.orientation_angles);
            state.quat.from_euler(radians(u.orientation_angles.roll*0.01),
                                  radians(u.orientation_angles.pitch*0.01),
                                  radians(u.orientation_angles.yaw*0.01));
            state.have_quaternion = true;
            if (last_att_ms == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: got link");
            }
            last_att_ms = now_ms;
            break;
        }
        case MessageType::VELOCITIES: {
            CHECK_SIZE(u.velocity);
            state.velocity = u.velocity.tofloat().rfu_to_frd()*0.01;
            state.have_velocity = true;
            last_vel_ms = now_ms;
            break;
        }
        case MessageType::POSITION: {
            CHECK_SIZE(u.position);
            state.location.lat = u.position.lat;
            state.location.lng = u.position.lon;
            state.location.alt = u.position.alt;
            state.have_location = true;
            state.last_location_update_us = AP_HAL::micros();
            last_pos_ms = now_ms;
            break;
        }
        case MessageType::KF_VEL_COVARIANCE: {
            CHECK_SIZE(u.kf_vel_covariance);
            state2.kf_vel_covariance = u.kf_vel_covariance.tofloat() * 0.001;
            break;
        }
        case MessageType::KF_POS_COVARIANCE: {
            CHECK_SIZE(u.kf_pos_covariance);
            state2.kf_pos_covariance = u.kf_pos_covariance.tofloat() * 0.001;
            break;
        }
        case MessageType::UNIT_STATUS: {
            CHECK_SIZE(u.unit_status);
            state2.unit_status = u.unit_status;
            break;
        }
        case MessageType::GNSS_EXTENDED_INFO: {
            CHECK_SIZE(u.gnss_extended_info);
            gps_data.fix_type = u.gnss_extended_info.fix_type+1;
            state2.gnss_extended_info = u.gnss_extended_info;
            break;
        }
        case MessageType::NUM_SATS: {
            CHECK_SIZE(u.num_sats);
            gps_data.satellites_in_view = u.num_sats;
            break;
        }
        case MessageType::GNSS_POSITION: {
            CHECK_SIZE(u.position);
            gps_data.latitude = u.position.lat;
            gps_data.longitude = u.position.lon;
            gps_data.msl_altitude = u.position.alt;
            break;
        }
        case MessageType::GNSS_VEL_TRACK: {
            CHECK_SIZE(u.gnss_vel_track);
            gps_data.ned_vel_north = cosF(radians(u.gnss_vel_track.track_over_ground*0.01))*u.gnss_vel_track.hor_speed*0.01;
            gps_data.ned_vel_east = sinF(radians(u.gnss_vel_track.track_over_ground*0.01))*u.gnss_vel_track.hor_speed*0.01;
            gps_data.ned_vel_down = -u.gnss_vel_track.ver_speed*0.01;
            break;
        }
        case MessageType::GNSS_POS_TIMESTAMP: {
            CHECK_SIZE(u.gnss_pos_timestamp);
            gps_data.ms_tow = u.gnss_pos_timestamp;
            break;
        }
        case MessageType::GNSS_INFO_SHORT: {
            CHECK_SIZE(u.gnss_info_short);
            state2.gnss_info_short = u.gnss_info_short;
            break;
        }
        case MessageType::GNSS_NEW_DATA: {
            CHECK_SIZE(u.gnss_new_data);
            state2.gnss_new_data = u.gnss_new_data;
            break;
        }
        case MessageType::GNSS_JAM_STATUS: {
            CHECK_SIZE(u.gnss_jam_status);
            state2.gnss_jam_status = u.gnss_jam_status;
            break;
        }
        case MessageType::DIFFERENTIAL_PRESSURE: {
            CHECK_SIZE(u.differential_pressure);
            airspeed_data.differential_pressure = u.differential_pressure*1.0e-2; //  Pa
            break;
        }
        case MessageType::TRUE_AIRSPEED: {
            CHECK_SIZE(u.true_airspeed);
            state2.true_airspeed = u.true_airspeed*0.01;
            break;
        }
        case MessageType::WIND_SPEED: {
            CHECK_SIZE(u.wind_speed);
            state2.wind_speed = u.wind_speed.tofloat().rfu_to_frd()*0.01;
            break;
        }
        case MessageType::AIR_DATA_STATUS: {
            CHECK_SIZE(u.air_data_status);
            state2.air_data_status = u.air_data_status;
            break;
        }
        case MessageType::SUPPLY_VOLTAGE: {
            CHECK_SIZE(u.supply_voltage);
            state2.supply_voltage = u.supply_voltage*0.01;
            break;
        }
        case MessageType::TEMPERATURE: {
            CHECK_SIZE(u.temperature);
            // assume same temperature for baro and airspeed
            baro_data.temperature = u.temperature*0.1;
            airspeed_data.temperature = u.temperature*0.1;
            break;
        }
        case MessageType::UNIT_STATUS2: {
            CHECK_SIZE(u.unit_status2);
            state2.unit_status2 = u.unit_status2;
            break;
        }
        case MessageType::GNSS_ANGLES: {
            CHECK_SIZE(u.gnss_angles);
            state2.gnss_heading = u.gnss_angles.gnss_heading * 1.0e-2;
            state2.gnss_pitch = u.gnss_angles.gnss_pitch * 1.0e-2;
            break;
        }
        case MessageType::GNSS_ANGLE_POS_TYPE: {
            CHECK_SIZE(u.gnss_angle_pos_type);
            state2.gnss_angle_pos_type = u.gnss_angle_pos_type;
            break;
        }
        case MessageType::GNSS_HEADING_TIMESTAMP: {
            CHECK_SIZE(u.gnss_heading_timestamp);
            state2.gnss_heading_timestamp = u.gnss_heading_timestamp;
            break;
        }
        case MessageType::GNSS_DOP: {
            CHECK_SIZE(u.gnss_dop);
            state2.gnss_gdop = u.gnss_dop.gnss_gdop * 1.0e-1;
            state2.gnss_pdop = u.gnss_dop.gnss_pdop * 1.0e-1;
            gps_data.hdop = u.gnss_dop.gnss_hdop * 1.0e-1;
            gps_data.vdop = u.gnss_dop.gnss_vdop * 1.0e-1;
            state2.gnss_tdop = u.gnss_dop.gnss_tdop * 1.0e-1;
            break;
        }
        }

        if (msg_len == 0) {
            // got an unknown message
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: unknown msg 0x%02x", unsigned(mtype));
            buffer_ofs = 0;
            return false;
        }
        message_ofs += msg_len;

        if (msg_len == 0 || need_re_sync) {
            re_sync();
            return false;
        }
    }

    if (h->msg_len != message_ofs-buffer) {
        // we had stray bytes at the end of the message
        re_sync();
        return false;
    }

    if (GOT_MSG(ACCEL_DATA_HR) &&
        GOT_MSG(GYRO_DATA_HR)) {
        AP::ins().handle_external(ins_data);
        state.accel = ins_data.accel;
        state.gyro = ins_data.gyro;
    }
    if (GOT_MSG(GPS_INS_TIME_MS) &&
        GOT_MSG(NUM_SATS) &&
        GOT_MSG(GNSS_POSITION) &&
        GOT_MSG(GNSS_NEW_DATA) &&
        GOT_MSG(GNSS_EXTENDED_INFO) &&
        state2.gnss_new_data != 0) {
        uint8_t instance;
        if (AP::gps().get_first_external_instance(instance)) {
            AP::gps().handle_external(gps_data, instance);
        }
        if (gps_data.satellites_in_view > 3) {
            if (last_gps_ms == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: got GPS lock");
                if (!state.have_origin) {
                    state.origin = Location{
                        gps_data.latitude,
                        gps_data.longitude,
                        gps_data.msl_altitude,
                        Location::AltFrame::ABSOLUTE};
                    state.have_origin = true;
                }
            }
            last_gps_ms = now_ms;
        }
    }
#if AP_BARO_EXTERNALAHRS_ENABLED
    if (GOT_MSG(BARO_DATA) &&
        GOT_MSG(TEMPERATURE)) {
        AP::baro().handle_external(baro_data);
    }
#endif
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    if (GOT_MSG(MAG_DATA)) {
        AP::compass().handle_external(mag_data);
    }
#endif
#if AP_AIRSPEED_EXTERNAL_ENABLED && (APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane))
    // only on plane and copter as others do not link AP_Airspeed
    if (GOT_MSG(DIFFERENTIAL_PRESSURE) &&
        GOT_MSG(TEMPERATURE)) {
        auto *arsp = AP::airspeed();
        if (arsp != nullptr) {
            arsp->handle_external(airspeed_data);
        }
    }
#endif // AP_AIRSPEED_EXTERNAL_ENABLED
    buffer_ofs = 0;

    if (GOT_MSG(POSITION) &&
        GOT_MSG(ORIENTATION_ANGLES) &&
        GOT_MSG(VELOCITIES)) {

        float roll, pitch, yaw;
        state.quat.to_euler(roll, pitch, yaw);
        uint64_t now_us = AP_HAL::micros64();

        // @LoggerMessage: ILB1
        // @Description: InertialLabs AHRS data1
        // @Field: TimeUS: Time since system startup
        // @Field: Roll: euler roll
        // @Field: Pitch: euler pitch
        // @Field: Yaw: euler yaw
        // @Field: VN: velocity north
        // @Field: VE: velocity east
        // @Field: VD: velocity down
        // @Field: Lat: latitude
        // @Field: Lon: longitude
        // @Field: Alt: altitude AMSL
        // @Field: TimeINS: GPS INS time (round)
        // @Field: INSvolt: Supply voltage

        AP::logger().WriteStreaming("ILB1", "TimeUS,Roll,Pitch,Yaw,VN,VE,VD,Lat,Lon,Alt,TimeINS,INSvolt",
                                    "sdddnnnDUmsv",
                                    "F000000GG000",
                                    "QffffffLLfIf",
                                    now_us,
                                    degrees(roll), degrees(pitch), degrees(yaw),
                                    state.velocity.x, state.velocity.y, state.velocity.z,
                                    state.location.lat, state.location.lng, state.location.alt*0.01,
                                    state2.gnss_ins_time_ms, state2.supply_voltage);

        // @LoggerMessage: ILB2
        // @Description: InertialLabs AHRS data2
        // @Field: TimeUS: Time since system startup
        // @Field: PosVarN: position variance north
        // @Field: PosVarE: position variance east
        // @Field: PosVarD: position variance down
        // @Field: VelVarN: velocity variance north
        // @Field: VelVarE: velocity variance east
        // @Field: VelVarD: velocity variance down

        AP::logger().WriteStreaming("ILB2", "TimeUS,PosVarN,PosVarE,PosVarD,VelVarN,VelVarE,VelVarD",
                                    "smmmnnn",
                                    "F000000",
                                    "Qffffff",
                                    now_us,
                                    state2.kf_pos_covariance.x, state2.kf_pos_covariance.y, state2.kf_pos_covariance.z,
                                    state2.kf_vel_covariance.x, state2.kf_vel_covariance.y, state2.kf_vel_covariance.z);

        // @LoggerMessage: ILB3
        // @Description: InertialLabs AHRS data3
        // @Field: TimeUS: Time since system startup
        // @Field: Stat1: unit status1
        // @Field: Stat2: unit status2
        // @Field: FType: fix type
        // @Field: SpStat: spoofing status
        // @Field: GI1: GNSS Info1
        // @Field: GI2: GNSS Info2
        // @Field: GJS: GNSS jamming status
        // @Field: TAS: true airspeed
        // @Field: WVN: Wind velocity north
        // @Field: WVE: Wind velocity east
        // @Field: WVD: Wind velocity down
        // @Field: ADU: Air Data Unit status

        AP::logger().WriteStreaming("ILB3", "TimeUS,Stat1,Stat2,FType,SpStat,GI1,GI2,GJS,TAS,WVN,WVE,WVD,ADU",
                                    "s------------",
                                    "F------------",
                                    "QHHBBBBBffffH",
                                    now_us,
                                    state2.unit_status, state2.unit_status2,
                                    state2.gnss_extended_info.fix_type, state2.gnss_extended_info.spoofing_status,
                                    state2.gnss_info_short.info1, state2.gnss_info_short.info2,
                                    state2.gnss_jam_status,
                                    state2.true_airspeed,
                                    state2.wind_speed.x, state2.wind_speed.y, state2.wind_speed.z,
                                    state2.air_data_status);

        // @LoggerMessage: ILB4
        // @Description: InertialLabs AHRS data4
        // @Field: TimeUS: Time since system startup
        // @Field: GpsYaw: GNSS Heading
        // @Field: GpsPitch: GNSS Pitch
        // @Field: GpsHTS: GNSS Heading timestamp        
        // @Field: GpsAType: GNSS Angles position type
        // @Field: GDOP: GNSS GDOP
        // @Field: PDOP: GNSS PDOP
        // @Field: HDOP: GNSS HDOP
        // @Field: VDOP: GNSS VDOP
        // @Field: TDOP: GNSS TDOP

        AP::logger().WriteStreaming("ILB4", "TimeUS,GpsYaw,GpsPitch,GpsHTS,GpsAType,GDOP,PDOP,HDOP,VDOP,TDOP",
                                    "sdds------",
                                    "F00I------",
                                    "QffIBfffff",
                                    now_us,
                                    state2.gnss_heading, state2.gnss_pitch,
                                    state2.gnss_heading_timestamp, state2.gnss_angle_pos_type,
                                    state2.gnss_gdop * 1.0e-2, state2.gnss_pdop * 1.0e-2, gps_data.hdop * 1.0e-2,
                                    gps_data.vdop * 1.0e-2, state2.gnss_tdop * 1.0e-2);
    }
        
    return true;
}

void AP_ExternalAHRS_InertialLabs::update_thread()
{
    // Open port in the thread
    uart->begin(baudrate, 1024, 512);

    /*
      we assume the user has already configured the device
     */

    setup_complete = true;
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay_microseconds(250);
        }
    }
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_InertialLabs::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS_InertialLabs::healthy(void) const
{
    WITH_SEMAPHORE(state.sem);
    return AP_HAL::millis() - last_att_ms < 100;
}

bool AP_ExternalAHRS_InertialLabs::initialised(void) const
{
    if (!setup_complete) {
        return false;
    }
    return true;
}

bool AP_ExternalAHRS_InertialLabs::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs unhealthy");
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    uint32_t now = AP_HAL::millis();
    if (now - last_att_ms > 10 ||
        now - last_pos_ms > 10 ||
        now - last_vel_ms > 10) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs not up to date");
        return false;
    }
    return true;
}

/*
  get filter status. We don't know the meaning of the status bits yet,
  so assume all OK if we have GPS lock
 */
void AP_ExternalAHRS_InertialLabs::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);
    uint32_t now = AP_HAL::millis();
    const uint32_t dt_limit = 200;
    const uint32_t dt_limit_gps = 500;
    memset(&status, 0, sizeof(status));
    const bool init_ok = (state2.unit_status & (ILABS_UNIT_STATUS_ALIGNMENT_FAIL|ILABS_UNIT_STATUS_OPERATION_FAIL))==0;
    status.flags.initalized = init_ok;
    status.flags.attitude = init_ok && (now - last_att_ms < dt_limit) && init_ok;
    status.flags.vert_vel = init_ok && (now - last_vel_ms < dt_limit);
    status.flags.vert_pos = init_ok && (now - last_pos_ms < dt_limit);
    status.flags.horiz_vel = status.flags.vert_vel;
    status.flags.horiz_pos_abs = status.flags.vert_pos;
    status.flags.horiz_pos_rel = status.flags.horiz_pos_abs;
    status.flags.pred_horiz_pos_rel = status.flags.horiz_pos_abs;
    status.flags.pred_horiz_pos_abs = status.flags.horiz_pos_abs;
    status.flags.using_gps = (now - last_gps_ms < dt_limit_gps) &&
        (state2.unit_status & (ILABS_UNIT_STATUS_GNSS_FAIL|ILABS_UNIT_STATUS2_GNSS_FUSION_OFF)) == 0;
    status.flags.gps_quality_good = (now - last_gps_ms < dt_limit_gps) &&
        (state2.unit_status2 & ILABS_UNIT_STATUS2_GNSS_POS_VALID) != 0 &&
        (state2.unit_status & ILABS_UNIT_STATUS_GNSS_FAIL) == 0;
    status.flags.rejecting_airspeed = (state2.air_data_status & ILABS_AIRDATA_AIRSPEED_FAIL);
}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS_InertialLabs::send_status_report(GCS_MAVLINK &link) const
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
    const float vel_gate = 5;
    const float pos_gate = 5;
    const float hgt_gate = 5;
    const float mag_var = 0;
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       state2.kf_vel_covariance.length()/vel_gate,
                                       state2.kf_pos_covariance.xy().length()/pos_gate,
                                       state2.kf_pos_covariance.z/hgt_gate,
                                       mag_var, 0, 0);
}

#endif  // AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED

