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

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

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
#define ILABS_UNIT_STATUS2_MAGCAL_3D_ACT          0x0040
#define ILABS_UNIT_STATUS2_GNSS_FUSION_OFF        0x0080
#define ILABS_UNIT_STATUS2_DIFF_PRESS_FUSION_OFF  0x0100
#define ILABS_UNIT_STATUS2_MAG_FUSION_OFF         0x0200
#define ILABS_UNIT_STATUS2_GNSS_POS_VALID         0x0400

// air data status bits
#define ILABS_AIRDATA_INIT_FAIL                   0x0001
#define ILABS_AIRDATA_DIFF_PRESS_INIT_FAIL        0x0002
#define ILABS_AIRDATA_STATIC_PRESS_FAIL           0x0004
#define ILABS_AIRDATA_DIFF_PRESS_FAIL             0x0008
#define ILABS_AIRDATA_STATIC_PRESS_RANGE_ERR      0x0010
#define ILABS_AIRDATA_DIFF_PRESS_RANGE_ERR        0x0020
#define ILABS_AIRDATA_PRESS_ALT_FAIL              0x0100
#define ILABS_AIRDATA_AIRSPEED_FAIL               0x0200
#define ILABS_AIRDATA_BELOW_THRESHOLD             0x0400


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
            CHECK_SIZE(u.gnss_time_ms);
            gps_data.ms_tow = u.gnss_time_ms;
            break;
        }
        case MessageType::GPS_WEEK: {
            CHECK_SIZE(u.gnss_week);
            gps_data.gps_week = u.gnss_week;
            break;
        }
        case MessageType::ACCEL_DATA_HR: {
            CHECK_SIZE(u.accel_data_hr);
            // should use 9.8106 instead of GRAVITY_MSS-constant in accordance with the device-documentation
            ins_data.accel = u.accel_data_hr.tofloat().rfu_to_frd()*9.8106f*1.0e-6; // m/s^2
            break;
        }
        case MessageType::GYRO_DATA_HR: {
            CHECK_SIZE(u.gyro_data_hr);
            ins_data.gyro = u.gyro_data_hr.tofloat().rfu_to_frd()*DEG_TO_RAD*1.0e-5; // rad/s
            break;
        }
        case MessageType::BARO_DATA: {
            CHECK_SIZE(u.baro_data);
            baro_data.pressure_pa = u.baro_data.pressure_pa2*2; // Pa
            state2.baro_alt = u.baro_data.baro_alt*0.01; // m
            break;
        }
        case MessageType::MAG_DATA: {
            CHECK_SIZE(u.mag_data);
            mag_data.field = u.mag_data.tofloat().rfu_to_frd()*(10*NTESLA_TO_MGAUSS); // milligauss
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
            gps_data.ned_vel_north = state.velocity.x; // m/s
            gps_data.ned_vel_east = state.velocity.y; // m/s
            gps_data.ned_vel_down = state.velocity.z; // m/s
            state.have_velocity = true;
            last_vel_ms = now_ms;
            break;
        }
        case MessageType::POSITION: {
            CHECK_SIZE(u.position);
            state.location.lat = u.position.lat; // deg*1.0e7
            state.location.lng = u.position.lon; // deg*1.0e7
            state.location.alt = u.position.alt; // m*100

            gps_data.latitude = u.position.lat; // deg*1.0e7
            gps_data.longitude = u.position.lon; // deg*1.0e7
            gps_data.msl_altitude = u.position.alt; // m*100

            state.have_location = true;
            state.last_location_update_us = AP_HAL::micros();
            last_pos_ms = now_ms;
            break;
        }
        case MessageType::KF_VEL_COVARIANCE: {
            CHECK_SIZE(u.kf_vel_covariance);
            state2.kf_vel_covariance = u.kf_vel_covariance.tofloat().rfu_to_frd(); // mm/s
            break;
        }
        case MessageType::KF_POS_COVARIANCE: {
            CHECK_SIZE(u.kf_pos_covariance);
            state2.kf_pos_covariance = u.kf_pos_covariance.tofloat(); // mm
            break;
        }
        case MessageType::UNIT_STATUS: {
            CHECK_SIZE(u.unit_status);
            state2.unit_status = u.unit_status;
            break;
        }
        case MessageType::GNSS_EXTENDED_INFO: {
            CHECK_SIZE(u.gnss_extended_info);
            gps_data.fix_type = AP_GPS_FixType(u.gnss_extended_info.fix_type+1);
            gnss_data.spoof_status = u.gnss_extended_info.spoofing_status;
            break;
        }
        case MessageType::NUM_SATS: {
            CHECK_SIZE(u.num_sats);
            gps_data.satellites_in_view = u.num_sats;
            break;
        }
        case MessageType::GNSS_POSITION: {
            CHECK_SIZE(u.gnss_position);
            gnss_data.lat = u.gnss_position.lat; // deg*1.0e7
            gnss_data.lng = u.gnss_position.lon; // deg*1.0e7
            gnss_data.alt = u.gnss_position.alt; // mm
            break;
        }
        case MessageType::GNSS_VEL_TRACK: {
            CHECK_SIZE(u.gnss_vel_track);
            gnss_data.hor_speed = u.gnss_vel_track.hor_speed*0.01; // m/s
            gnss_data.ver_speed = u.gnss_vel_track.ver_speed*0.01; // m/s
            gnss_data.track_over_ground = u.gnss_vel_track.track_over_ground*0.01; // deg
            break;
        }
        case MessageType::GNSS_POS_TIMESTAMP: {
            CHECK_SIZE(u.gnss_pos_timestamp);
            gnss_data.pos_timestamp = u.gnss_pos_timestamp;
            break;
        }
        case MessageType::GNSS_INFO_SHORT: {
            CHECK_SIZE(u.gnss_info_short);
            gnss_data.info_short = u.gnss_info_short;
            break;
        }
        case MessageType::GNSS_NEW_DATA: {
            CHECK_SIZE(u.gnss_new_data);
            gnss_data.new_data = u.gnss_new_data;
            break;
        }
        case MessageType::GNSS_JAM_STATUS: {
            CHECK_SIZE(u.gnss_jam_status);
            gnss_data.jam_status = u.gnss_jam_status;
            break;
        }
        case MessageType::DIFFERENTIAL_PRESSURE: {
            CHECK_SIZE(u.differential_pressure);
            airspeed_data.differential_pressure = u.differential_pressure*1.0e-4*100; // 100: mbar to Pa
            break;
        }
        case MessageType::TRUE_AIRSPEED: {
            CHECK_SIZE(u.true_airspeed);
            state2.true_airspeed = u.true_airspeed*0.01; // m/s
            break;
        }
        case MessageType::WIND_SPEED: {
            CHECK_SIZE(u.wind_speed);
            state2.wind_speed = u.wind_speed.tofloat().rfu_to_frd()*0.01; // m/s
            break;
        }
        case MessageType::AIR_DATA_STATUS: {
            CHECK_SIZE(u.air_data_status);
            state2.air_data_status = u.air_data_status;
            break;
        }
        case MessageType::SUPPLY_VOLTAGE: {
            CHECK_SIZE(u.supply_voltage);
            state2.supply_voltage = u.supply_voltage*0.01; // V
            break;
        }
        case MessageType::TEMPERATURE: {
            CHECK_SIZE(u.temperature);
            // assume same temperature for baro and airspeed
            baro_data.temperature = u.temperature*0.1; // degC
            airspeed_data.temperature = u.temperature*0.1; // degC
            ins_data.temperature = u.temperature*0.1;
            break;
        }
        case MessageType::UNIT_STATUS2: {
            CHECK_SIZE(u.unit_status2);
            state2.unit_status2 = u.unit_status2;
            break;
        }
        case MessageType::GNSS_ANGLES: {
            CHECK_SIZE(u.gnss_angles);
            gnss_data.heading = u.gnss_angles.heading*0.01; // deg
            gnss_data.pitch = u.gnss_angles.pitch*0.01; // deg
            break;
        }
        case MessageType::GNSS_ANGLE_POS_TYPE: {
            CHECK_SIZE(u.gnss_angle_pos_type);
            gnss_data.angle_pos_type = u.gnss_angle_pos_type;
            break;
        }
        case MessageType::GNSS_HEADING_TIMESTAMP: {
            CHECK_SIZE(u.gnss_heading_timestamp);
            gnss_data.heading_timestamp = u.gnss_heading_timestamp;
            break;
        }
        case MessageType::GNSS_DOP: {
            CHECK_SIZE(u.gnss_dop);
            gnss_data.gdop = u.gnss_dop.gdop*0.1;
            gnss_data.pdop = u.gnss_dop.pdop*0.1;
            gnss_data.tdop = u.gnss_dop.tdop*0.1;

            gps_data.hdop = u.gnss_dop.hdop*0.1;
            gps_data.vdop = u.gnss_dop.vdop*0.1;
            break;
        }
        case MessageType::INS_SOLUTION_STATUS: {
            CHECK_SIZE(u.ins_sol_status);
            state2.ins_sol_status = u.ins_sol_status;
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

#if HAL_LOGGING_ENABLED
        uint64_t now_us = AP_HAL::micros64();

        // @LoggerMessage: ILB1
        // @Description: InertialLabs AHRS data1
        // @Field: TimeUS: Time since system startup
        // @Field: GMS: GPS INS time (round)
        // @Field: GyrX: Gyro X
        // @Field: GyrY: Gyro Y
        // @Field: GyrZ: Gyro z
        // @Field: AccX: Accelerometer X
        // @Field: AccY: Accelerometer Y
        // @Field: AccZ: Accelerometer Z

        AP::logger().WriteStreaming("ILB1", "TimeUS,GMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ",
                                    "s-kkkooo",
                                    "F-------",
                                    "QIffffff",
                                    now_us, gps_data.ms_tow,
                                    ins_data.gyro.x, ins_data.gyro.y, ins_data.gyro.z,
                                    ins_data.accel.x, ins_data.accel.y, ins_data.accel.z);
#endif // HAL_LOGGING_ENABLED
    }

    if (GOT_MSG(GPS_INS_TIME_MS) &&
        GOT_MSG(NUM_SATS) &&
        GOT_MSG(GNSS_POSITION) &&
        GOT_MSG(GNSS_NEW_DATA) &&
        GOT_MSG(GNSS_EXTENDED_INFO) &&
        gnss_data.new_data != 0) {
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

#if HAL_LOGGING_ENABLED
        uint64_t now_us = AP_HAL::micros64();

        // @LoggerMessage: ILB4
        // @Description: InertialLabs AHRS data4
        // @Field: TimeUS: Time since system startup
        // @Field: GMS: GNSS Position timestamp
        // @Field: GWk: GPS Week
        // @Field: NSat: Number of satellites
        // @Field: NewGPS: Indicator of new update of GPS data
        // @Field: Lat: GNSS Latitude
        // @Field: Lng: GNSS Longitude
        // @Field: Alt: GNSS Altitude
        // @Field: GCrs: GNSS Track over ground
        // @Field: Spd: GNSS Horizontal speed
        // @Field: VZ: GNSS Vertical speed

        AP::logger().WriteStreaming("ILB4", "TimeUS,GMS,GWk,NSat,NewGPS,Lat,Lng,Alt,GCrs,Spd,VZ",
                                    "s----DUmhnn",
                                    "F----------",
                                    "QIHBBffffff",
                                    now_us, gnss_data.pos_timestamp, gps_data.gps_week,
                                    gps_data.satellites_in_view, gnss_data.new_data,
                                    gnss_data.lat*1.0e-7, gnss_data.lng*1.0e-7, gnss_data.alt*0.01,
                                    gnss_data.track_over_ground, gnss_data.hor_speed, gnss_data.ver_speed
                                    );

        // @LoggerMessage: ILB5
        // @Description: InertialLabs AHRS data5
        // @Field: TimeUS: Time since system startup
        // @Field: GMS: GNSS Position timestamp
        // @Field: FType: fix type
        // @Field: GSS: GNSS spoofing status
        // @Field: GJS: GNSS jamming status
        // @Field: GI1: GNSS Info1
        // @Field: GI2: GNSS Info2
        // @Field: GAPS: GNSS Angles position type

        AP::logger().WriteStreaming("ILB5", "TimeUS,GMS,FType,GSS,GJS,GI1,GI2,GAPS",
                                    "s-------",
                                    "F-------",
                                    "QIBBBBBB",
                                    now_us, gnss_data.pos_timestamp, gps_data.fix_type,
                                    gnss_data.spoof_status, gnss_data.jam_status,
                                    gnss_data.info_short.info1, gnss_data.info_short.info2,
                                    gnss_data.angle_pos_type);

        // @LoggerMessage: ILB6
        // @Description: InertialLabs AHRS data6
        // @Field: TimeUS: Time since system startup
        // @Field: GMS: GNSS Position timestamp
        // @Field: GpsHTS: GNSS Heading timestamp
        // @Field: GpsYaw: GNSS Heading
        // @Field: GpsPitch: GNSS Pitch
        // @Field: GDOP: GNSS GDOP
        // @Field: PDOP: GNSS PDOP
        // @Field: HDOP: GNSS HDOP
        // @Field: VDOP: GNSS VDOP
        // @Field: TDOP: GNSS TDOP

        AP::logger().WriteStreaming("ILB6", "TimeUS,GMS,GpsHTS,GpsYaw,GpsPitch,GDOP,PDOP,HDOP,VDOP,TDOP",
                                    "s--hd-----",
                                    "F---------",
                                    "QIIfffffff",
                                    now_us, gnss_data.pos_timestamp, gnss_data.heading_timestamp,
                                    gnss_data.heading, gnss_data.pitch, gnss_data.gdop, gnss_data.pdop,
                                    gps_data.hdop, gps_data.vdop, gnss_data.tdop);
#endif // HAL_LOGGING_ENABLED
    }

#if AP_BARO_EXTERNALAHRS_ENABLED
    if (GOT_MSG(BARO_DATA) &&
        GOT_MSG(TEMPERATURE)) {
        AP::baro().handle_external(baro_data);

#if HAL_LOGGING_ENABLED
        uint64_t now_us = AP_HAL::micros64();

        // @LoggerMessage: ILB3
        // @Description: InertialLabs AHRS data3
        // @Field: TimeUS: Time since system startup
        // @Field: GMS: GPS INS time (round)
        // @Field: Press: Static pressure
        // @Field: Diff: Differential pressure
        // @Field: Temp: Temperature
        // @Field: Alt: Baro altitude
        // @Field: TAS: true airspeed
        // @Field: VWN: Wind velocity north
        // @Field: VWE: Wind velocity east
        // @Field: VWD: Wind velocity down
        // @Field: ADU: Air Data Unit status

        AP::logger().WriteStreaming("ILB3", "TimeUS,GMS,Press,Diff,Temp,Alt,TAS,VWN,VWE,VWD,ADU",
                                    "s-PPOmnnnn-",
                                    "F----------",
                                    "QIffffffffH",
                                    now_us, gps_data.ms_tow,
                                    baro_data.pressure_pa, airspeed_data.differential_pressure, baro_data.temperature,
                                    state2.baro_alt, state2.true_airspeed,
                                    state2.wind_speed.x, state2.wind_speed.y, state2.wind_speed.z,
                                    state2.air_data_status);
#endif // HAL_LOGGING_ENABLED
    }
#endif // AP_BARO_EXTERNALAHRS_ENABLED

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    if (GOT_MSG(MAG_DATA)) {
        AP::compass().handle_external(mag_data);

#if HAL_LOGGING_ENABLED
        uint64_t now_us = AP_HAL::micros64();

        // @LoggerMessage: ILB2
        // @Description: InertialLabs AHRS data2
        // @Field: TimeUS: Time since system startup
        // @Field: GMS: GPS INS time (round)
        // @Field: MagX: Magnetometer X
        // @Field: MagY: Magnetometer Y
        // @Field: MagZ: Magnetometer Z

        AP::logger().WriteStreaming("ILB2", "TimeUS,GMS,MagX,MagY,MagZ",
                                    "s----",
                                    "F----",
                                    "QIfff",
                                    now_us, gps_data.ms_tow,
                                    mag_data.field.x, mag_data.field.y, mag_data.field.z);
#endif // HAL_LOGGING_ENABLED
    }
#endif // AP_COMPASS_EXTERNALAHRS_ENABLED

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

        float roll, pitch, yaw_deg;
        state.quat.to_euler(roll, pitch, yaw_deg);

        yaw_deg = fmodf(degrees(yaw_deg), 360.0f);
        if (yaw_deg < 0.0f) {
            yaw_deg += 360.0f;
        }

#if HAL_LOGGING_ENABLED
        uint64_t now_us = AP_HAL::micros64();

        // @LoggerMessage: ILB7
        // @Description: InertialLabs AHRS data7
        // @Field: TimeUS: Time since system startup
        // @Field: GMS: GPS INS time (round)
        // @Field: Roll: euler roll
        // @Field: Pitch: euler pitch
        // @Field: Yaw: euler yaw
        // @Field: VN: velocity north
        // @Field: VE: velocity east
        // @Field: VD: velocity down
        // @Field: Lat: latitude
        // @Field: Lng: longitude
        // @Field: Alt: altitude MSL
        // @Field: USW: USW1
        // @Field: USW2: USW2
        // @Field: Vdc: Supply voltage

        AP::logger().WriteStreaming("ILB7", "TimeUS,GMS,Roll,Pitch,Yaw,VN,VE,VD,Lat,Lng,Alt,USW,USW2,Vdc",
                                    "s-dddnnnDUm--v",
                                    "F-------------",
                                    "QIfffffffffHHf",
                                    now_us, gps_data.ms_tow,
                                    degrees(roll), degrees(pitch), yaw_deg,
                                    state.velocity.x, state.velocity.y, state.velocity.z,
                                    state.location.lat*1.0e-7, state.location.lng*1.0e-7, state.location.alt*0.01,
                                    state2.unit_status, state2.unit_status2,
                                    state2.supply_voltage);

        // @LoggerMessage: ILB8
        // @Description: InertialLabs AHRS data8
        // @Field: TimeUS: Time since system startup
        // @Field: GMS: GPS INS time (round)
        // @Field: PVN: position variance north
        // @Field: PVE: position variance east
        // @Field: PVD: position variance down
        // @Field: VVN: velocity variance north
        // @Field: VVE: velocity variance east
        // @Field: VVD: velocity variance down

        AP::logger().WriteStreaming("ILB8", "TimeUS,GMS,PVN,PVE,PVD,VVN,VVE,VVD",
                                    "s-mmmnnn",
                                    "F-------",
                                    "QIffffff",
                                    now_us, gps_data.ms_tow,
                                    state2.kf_pos_covariance.x, state2.kf_pos_covariance.y, state2.kf_pos_covariance.z,
                                    state2.kf_vel_covariance.x, state2.kf_vel_covariance.y, state2.kf_vel_covariance.z);
#endif  // HAL_LOGGING_ENABLED
    }

    const uint32_t dt_critical_usw = 10000;
    uint32_t now_usw = AP_HAL::millis();

    // InertialLabs critical messages to GCS (sending messages once every 10 seconds)
    if ((last_unit_status != state2.unit_status) ||
        (last_unit_status2 != state2.unit_status2) ||
        (last_air_data_status != state2.air_data_status) ||
        (now_usw - last_critical_msg_ms > dt_critical_usw)) {

        // Critical USW message
        if (state2.unit_status & ILABS_UNIT_STATUS_ALIGNMENT_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: Unsuccessful initial alignment");
        }
        if (state2.unit_status & ILABS_UNIT_STATUS_OPERATION_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: IMU data are incorrect");
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_GYRO_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: Gyros failure");
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_ACCEL_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: Accelerometers failure");
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_MAG_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: Magnetometers failure");
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_ELECTRONICS_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: Electronics failure");
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_GNSS_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: GNSS receiver failure");
        }

        // Critical USW2 message
        if (state2.unit_status2 & ILABS_UNIT_STATUS2_BARO_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: Baro altimeter failure");
        }

        if (state2.unit_status2 & ILABS_UNIT_STATUS2_DIFF_PRESS_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: Diff. pressure sensor failure");
        }

        // Critical ADU message
        if (state2.air_data_status & ILABS_AIRDATA_INIT_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: Static pressure sensor unsuccessful initialization");
        }

        if (state2.air_data_status & ILABS_AIRDATA_DIFF_PRESS_INIT_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: Diff. pressure sensor unsuccessful initialization");
        }

        if (state2.air_data_status & ILABS_AIRDATA_STATIC_PRESS_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ILAB: Static pressure sensor failure is detect");
        }

        if (state2.air_data_status & ILABS_AIRDATA_DIFF_PRESS_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ILAB: Diff. pressure sensor failure is detect");
        }

        last_critical_msg_ms = AP_HAL::millis();
    }

    if (last_unit_status != state2.unit_status) {
        if (state2.unit_status & ILABS_UNIT_STATUS_RUNTIME_CAL) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: On-the-fly calibration is in progress");
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_VOLTAGE_LOW) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Low input voltage");
        } else {
            if (last_unit_status & ILABS_UNIT_STATUS_VOLTAGE_LOW) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Input voltage is in range");
            }
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_VOLTAGE_HIGH) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: High input voltage");
        } else {
            if (last_unit_status & ILABS_UNIT_STATUS_VOLTAGE_HIGH) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Input voltage is in range");
            }
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_X_RATE_HIGH) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Y-axis angular rate is exceeded");
        } else {
            if (last_unit_status & ILABS_UNIT_STATUS_X_RATE_HIGH) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Y-axis angular rate is in range");
            }
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_Y_RATE_HIGH) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: X-axis angular rate is exceeded");
        } else {
            if (last_unit_status & ILABS_UNIT_STATUS_Y_RATE_HIGH) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: X-axis angular rate is in range");
            }
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_Z_RATE_HIGH) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Z-axis angular rate is exceeded");
        } else {
            if (last_unit_status & ILABS_UNIT_STATUS_Z_RATE_HIGH) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Z-axis angular rate is in range");
            }
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_MAG_FIELD_HIGH) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Large total magnetic field");
        } else {
            if (last_unit_status & ILABS_UNIT_STATUS_MAG_FIELD_HIGH) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Total magnetic field is in range");
            }
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_TEMP_RANGE_ERR) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Temperature is out of range");
        } else {
            if (last_unit_status & ILABS_UNIT_STATUS_TEMP_RANGE_ERR) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Temperature is in range");
            }
        }

        if (state2.unit_status & ILABS_UNIT_STATUS_RUNTIME_CAL2) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: On-the-fly calibration successful");
        }

        last_unit_status = state2.unit_status;
    }

    // InertialLabs INS Unit Status Word 2 (USW2) messages to GCS
    if (last_unit_status2 != state2.unit_status2) {
        if (state2.unit_status2 & ILABS_UNIT_STATUS2_ACCEL_X_HIGH) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Y-acceleration is out of range");
        } else {
            if (last_unit_status2 & ILABS_UNIT_STATUS2_ACCEL_X_HIGH) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Y-acceleration is in range");
            }
        }

        if (state2.unit_status2 & ILABS_UNIT_STATUS2_ACCEL_Y_HIGH) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: X-acceleration is out of range");
        } else {
            if (last_unit_status2 & ILABS_UNIT_STATUS2_ACCEL_Y_HIGH) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: X-acceleration is in range");
            }
        }

        if (state2.unit_status2 & ILABS_UNIT_STATUS2_ACCEL_Z_HIGH) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Z-acceleration is out of range");
        } else {
            if (last_unit_status2 & ILABS_UNIT_STATUS2_ACCEL_Z_HIGH) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Z-acceleration is in range");
            }
        }

        if (state2.unit_status2 & ILABS_UNIT_STATUS2_MAGCAL_2D_ACT) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Automatic 2D calibration is in progress");
        }

        if (state2.unit_status2 & ILABS_UNIT_STATUS2_MAGCAL_3D_ACT) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Automatic 3D calibration is in progress");
        }

        if (state2.unit_status2 & ILABS_UNIT_STATUS2_GNSS_FUSION_OFF) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS input switched off");
        } else {
            if (last_unit_status2 & ILABS_UNIT_STATUS2_GNSS_FUSION_OFF) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS input switched on");
            }
        }

        if (state2.unit_status2 & ILABS_UNIT_STATUS2_DIFF_PRESS_FUSION_OFF) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Diff. pressure input switched off");
        } else {
            if (last_unit_status2 & ILABS_UNIT_STATUS2_DIFF_PRESS_FUSION_OFF) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Diff. pressure input switched on");
            }
        }

        if (state2.unit_status2 & ILABS_UNIT_STATUS2_MAG_FUSION_OFF) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Magnetometer input switched off");
        } else {
            if (last_unit_status2 & ILABS_UNIT_STATUS2_MAG_FUSION_OFF) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Magnetometer input switched on");
            }
        }

        if (state2.unit_status2 & ILABS_UNIT_STATUS2_GNSS_POS_VALID) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Incorrect GNSS position");
        } else {
            if (last_unit_status2 & ILABS_UNIT_STATUS2_GNSS_POS_VALID) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS position is correct");
            }
        }

        last_unit_status2 = state2.unit_status2;
    }

    // InertialLabs INS Air Data Unit (ADU) status messages to GCS
    if (last_air_data_status != state2.air_data_status) {
        if (state2.air_data_status & ILABS_AIRDATA_STATIC_PRESS_RANGE_ERR) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Static pressure is out of range");
        } else {
            if (last_air_data_status & ILABS_AIRDATA_STATIC_PRESS_RANGE_ERR) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Static pressure is in range");
            }
        }

        if (state2.air_data_status & ILABS_AIRDATA_DIFF_PRESS_RANGE_ERR) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Diff. pressure is out of range");
        } else {
            if (last_air_data_status & ILABS_AIRDATA_DIFF_PRESS_RANGE_ERR) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Diff. pressure is in range");
            }
        }

        if (state2.air_data_status & ILABS_AIRDATA_PRESS_ALT_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Pressure altitude is incorrect");
        } else {
            if (last_air_data_status & ILABS_AIRDATA_PRESS_ALT_FAIL) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Pressure altitude is correct");
            }
        }

        if (state2.air_data_status & ILABS_AIRDATA_AIRSPEED_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Air speed is incorrect");
        } else {
            if (last_air_data_status & ILABS_AIRDATA_AIRSPEED_FAIL) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Air speed is correct");
            }
        }

        if (state2.air_data_status & ILABS_AIRDATA_AIRSPEED_FAIL) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Air speed is below the threshold");
        } else {
            if (last_air_data_status & ILABS_AIRDATA_AIRSPEED_FAIL) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Air speed is above the threshold");
            }
        }

        last_air_data_status = state2.air_data_status;
    }

    // InertialLabs INS spoofing detection messages to GCS
    if (last_spoof_status != gnss_data.spoof_status) {
        if ((last_spoof_status == 2 || last_spoof_status == 3) && (gnss_data.spoof_status == 1)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS no spoofing");
        }

        if (last_spoof_status == 2) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: GNSS spoofing indicated");
        }

        if (last_spoof_status == 3) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: GNSS multiple spoofing indicated");
        }

        last_spoof_status = gnss_data.spoof_status;
    }

    // InertialLabs INS jamming detection messages to GCS
    if (last_jam_status != gnss_data.jam_status) {
        if ((last_jam_status == 2 || last_jam_status == 3) && (gnss_data.jam_status == 1)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS no jamming");
        }

        if (gnss_data.jam_status == 3) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: GNSS jamming indicated and no fix");
        }

        last_jam_status = gnss_data.jam_status;
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
        ((state2.unit_status & ILABS_UNIT_STATUS_GNSS_FAIL) | (state2.unit_status2 & ILABS_UNIT_STATUS2_GNSS_FUSION_OFF)) == 0;
    status.flags.gps_quality_good = (now - last_gps_ms < dt_limit_gps) &&
        (state2.unit_status2 & ILABS_UNIT_STATUS2_GNSS_POS_VALID) != 0 &&
        (state2.unit_status & ILABS_UNIT_STATUS_GNSS_FAIL) == 0;
    status.flags.rejecting_airspeed = (state2.air_data_status & ILABS_AIRDATA_AIRSPEED_FAIL);
}

// get variances
bool AP_ExternalAHRS_InertialLabs::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = state2.kf_vel_covariance.length() * vel_gate_scale;
    posVar = state2.kf_pos_covariance.xy().length() * pos_gate_scale;
    hgtVar = state2.kf_pos_covariance.z * hgt_gate_scale;
    tasVar = 0;
    return true;
}

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

