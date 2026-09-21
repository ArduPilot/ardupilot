/*
   Copyright (C) 2025 Kraus Hamdani Aerospace Inc. All rights reserved.

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
  support for serial connected SBG INS
 */

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SBG_ENABLED

#include "AP_ExternalAHRS_SBG.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Common/time.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

// constructor
AP_ExternalAHRS_SBG::AP_ExternalAHRS_SBG(AP_ExternalAHRS *_frontend,
                                                           AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SBG ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);


    // don't offer IMU by default, at 200Hz it is too slow for many aircraft
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));
    
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SBG::update_thread, void), "SBG", 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("SBG Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG ExternalAHRS initialised");
}

void AP_ExternalAHRS_SBG::update_thread()
{
    hal.scheduler->delay(1000);
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
    hal.scheduler->delay(1000);

    if (uart == nullptr) {
        return;
    }

    uart->begin(baudrate, 1024, 1024);

    setup_complete = true;

    while (true) {
        
        if (check_uart()) {
            // we've parsed something. There might be more so lets come back quickly
            hal.scheduler->delay_microseconds(100);
            continue;
        }

        // uart is idle, lets snooze a little more and then do some housekeeping
        hal.scheduler->delay_microseconds(250);

        const uint32_t now_ms = AP_HAL::millis();
        if (cached.sbg.deviceInfo.firmwareRev == 0 && now_ms - version_check_ms >= 5000) {
            // request Device Info every few seconds until we get a response
            version_check_ms = now_ms;

            // static uint32_t count = 1;
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG Requesting DeviceInfo %u", count++);

            const sbgMessage msg = sbgMessage(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INFO);
            UNUSED_RESULT(send_sbgMessage(uart, msg)); // don't care about any error, just retry at 1Hz

#if AP_COMPASS_ENABLED
        } else if (now_ms - send_MagData_ms >= 100) {
            send_MagData_ms = now_ms;

            if (!send_MagData(uart)) {
                // TODO: if it fails maybe we should figure out why and retry?
                // possible causes:
                // 1) uart == nullptr
                // 2) msg.len > sizeof(data)
                // 3) did not write all the bytes out the uart: zero/fail is treated same as packet_len != bytes_sent
                if (now_ms - send_mag_error_last_ms >= 5000) {
                    // throttle the error to no faster than 5Hz because if you get one error you'll likely get a lot
                    send_mag_error_last_ms = now_ms;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Error sending Mag data");
                }
            }
#endif // AP_COMPASS_ENABLED

#if AP_BARO_ENABLED || AP_AIRSPEED_ENABLED
        } else if (now_ms - send_AirData_ms >= 100) {
            send_AirData_ms = now_ms;

            if (!send_AirData(uart)) {
                // TODO: if it fails maybe we should figure out why and retry?
                // possible causes:
                // 1) uart == nullptr
                // 2) msg.len > sizeof(data)
                // 3) did not write all the bytes out the uart: zero/fail is treated same as packet_len != bytes_sent
                if (now_ms - send_air_error_last_ms >= 5000) {
                    // throttle the error to no faster than 5Hz because if you get one error you'll likely get a lot
                    send_air_error_last_ms = now_ms;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Error sending Air data");
                }
            }
#endif // AP_BARO_ENABLED || AP_AIRSPEED_ENABLED
        }
    } // while
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
// returns true if any data was found in the UART buffer which was then processed
bool AP_ExternalAHRS_SBG::check_uart()
{
    uint32_t nbytes = MIN(uart->available(), 512u);
    if (nbytes == 0) {
        return false;
    }
    while (nbytes--> 0) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }

        if (parse_byte(b, _inbound_state.msg, _inbound_state)) {
            handle_msg(_inbound_state.msg);
        }
    }
    return true;
}


bool AP_ExternalAHRS_SBG::send_sbgMessage(AP_HAL::UARTDriver *_uart, const sbgMessage &msg)
{
    if (_uart == nullptr || msg.len > ARRAY_SIZE(msg.data)) {
        // invalid _uart or msg.len is out of range
        return false;
    }

    const uint16_t buffer_len = (SBG_PACKET_OVERHEAD + msg.len);
    uint8_t buffer[buffer_len];

    buffer[0] = SBG_PACKET_SYNC1;
    buffer[1] = SBG_PACKET_SYNC2;
    buffer[2] = msg.msgid;
    buffer[3] = msg.msgclass;
    buffer[4] = msg.len & 0xFF; // LSB first
    buffer[5] = msg.len >> 8;

    for (uint16_t i=0; i<msg.len; i++) {
        buffer[6+i] = msg.data[i];
    }

    const uint16_t crc = crc16_ccitt_r((const uint8_t*)&msg, msg.len+4, 0, 0);

    buffer[buffer_len-3] = crc & 0xFF; // LSB First
    buffer[buffer_len-2] = crc >> 8;
    buffer[buffer_len-1] = SBG_PACKET_ETX;

    const uint32_t bytes_sent = _uart->write(buffer, buffer_len);
    _uart->flush();
    return (bytes_sent == buffer_len);
}

bool AP_ExternalAHRS_SBG::parse_byte(const uint8_t data, sbgMessage &msg, SBG_PACKET_INBOUND_STATE &inbound_state)
{
    switch (inbound_state.parser) {
        case SBG_PACKET_PARSE_STATE::SYNC1:
            inbound_state.parser = (data == SBG_PACKET_SYNC1) ? SBG_PACKET_PARSE_STATE::SYNC2 : SBG_PACKET_PARSE_STATE::SYNC1;
            break;
            
        case SBG_PACKET_PARSE_STATE::SYNC2:
            inbound_state.parser = (data == SBG_PACKET_SYNC2) ? SBG_PACKET_PARSE_STATE::MSG : SBG_PACKET_PARSE_STATE::SYNC1;
            break;

        case SBG_PACKET_PARSE_STATE::MSG:
            msg.msgid = data;
            inbound_state.parser = SBG_PACKET_PARSE_STATE::CLASS;
            break;

        case SBG_PACKET_PARSE_STATE::CLASS:
            msg.msgclass = data;
            inbound_state.parser = SBG_PACKET_PARSE_STATE::LEN1;
            break;

        case SBG_PACKET_PARSE_STATE::LEN1:
            msg.len = data;
            inbound_state.parser = SBG_PACKET_PARSE_STATE::LEN2;
            break;

        case SBG_PACKET_PARSE_STATE::LEN2:
            msg.len |= uint16_t(data) << 8;
            if (msg.len > sizeof(msg.data)) {
                // we can't handle this packet, it's larger than the rx buffer which is larger than the largest packet we care about
                inbound_state.data_count_skip = msg.len;
                inbound_state.parser = SBG_PACKET_PARSE_STATE::DROP_THIS_PACKET;
            } else {
                inbound_state.data_count = 0;
                inbound_state.parser = (msg.len > 0) ? SBG_PACKET_PARSE_STATE::DATA : SBG_PACKET_PARSE_STATE::CRC1;
            }
            break;

        case SBG_PACKET_PARSE_STATE::DATA:
            msg.data[inbound_state.data_count++] = data;
            if (inbound_state.data_count >= sizeof(msg.data)) {
                inbound_state.parser = SBG_PACKET_PARSE_STATE::SYNC1;
            } else if (inbound_state.data_count >= msg.len) {
                inbound_state.parser = SBG_PACKET_PARSE_STATE::CRC1;
            }
            break;

        case SBG_PACKET_PARSE_STATE::CRC1:
            inbound_state.crc = data;
            inbound_state.parser = SBG_PACKET_PARSE_STATE::CRC2;
            break;

        case SBG_PACKET_PARSE_STATE::CRC2:
            inbound_state.crc |= uint16_t(data) << 8;
            inbound_state.parser = SBG_PACKET_PARSE_STATE::SYNC1; // skip ETX and go directly to SYNC1. Do not pass Go.
            {
                // CRC field is computed on [MSG(1), CLASS(1), LEN(2), DATA(msg.len)] fields
                const uint16_t crc = crc16_ccitt_r((const uint8_t*)&msg, msg.len+4, 0, 0);
                if (crc == inbound_state.crc) {
                    return true;
                }
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Failed CRC. Received 0x%04X, Expected 0x%04X", (unsigned)inbound_state.crc, (unsigned)crc);
            }
            break;

        case SBG_PACKET_PARSE_STATE::ETX:
        // we can just skip this state and let SYNC1 fail once when it gets the ETX byte every time... same amount of work
            inbound_state.parser = SBG_PACKET_PARSE_STATE::SYNC1;
            break;

        default:
        case SBG_PACKET_PARSE_STATE::DROP_THIS_PACKET:
            // we're currently parsing a packet that is very large and is not one we care
            // about. This is not the packet you're looking for... drop all those bytes
            if (inbound_state.data_count_skip > 0) {
                inbound_state.data_count_skip--;
            }
            if (inbound_state.data_count_skip == 0) {
                inbound_state.parser = SBG_PACKET_PARSE_STATE::SYNC1;
            }
            break;
    }

    return false;
}

uint16_t AP_ExternalAHRS_SBG::make_gps_week(const SbgEComLogUtc *utc_data)
{
    const struct tm tm {
        .tm_sec = utc_data->second,
        .tm_min = utc_data->minute,
        .tm_hour = utc_data->hour,
        .tm_mday = utc_data->day,
        .tm_mon  = utc_data->month - 1,
        .tm_year = utc_data->year - 1900,
    };


    // convert from time structure to unix time
    const time_t unix_time = ap_mktime(&tm);

    // convert to time since GPS epoch
    const uint32_t gps_time = unix_time - ((utc_data->gpsTimeOfWeek + UNIX_OFFSET_MSEC) / 1000);

    // get GPS week
    const uint16_t gps_week = gps_time / AP_SEC_PER_WEEK;

    return gps_week;
}

void AP_ExternalAHRS_SBG::handle_msg(const sbgMessage &msg)
{
    const uint32_t now_ms = AP_HAL::millis();
    const bool valid_class1_msg = ((SbgEComClass)msg.msgclass == SBG_ECOM_CLASS_LOG_ECOM_1 && (_SbgEComLog1MsgId)msg.msgid == SBG_ECOM_LOG_FAST_IMU_DATA);

    if ((SbgEComClass)msg.msgclass == SBG_ECOM_CLASS_LOG_CMD_0) {
        switch ((SbgEComCmd)msg.msgid) {
            case SBG_ECOM_CMD_ACK: // 0
                break;

            case SBG_ECOM_CMD_INFO: // 4
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.deviceInfo, sizeof(cached.sbg.deviceInfo), msg.data, msg.len);

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: %s", cached.sbg.deviceInfo.productCode);
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: %u, %u, %u, %u, %u, %u, %u: %u.%u.%u",
                //                 (unsigned)cached.sbg.deviceInfo.serialNumber,
                //                 (unsigned)cached.sbg.deviceInfo.calibationRev,
                //                 (unsigned)cached.sbg.deviceInfo.calibrationYear,
                //                 (unsigned)cached.sbg.deviceInfo.calibrationMonth,
                //                 (unsigned)cached.sbg.deviceInfo.calibrationDay,
                //                 (unsigned)cached.sbg.deviceInfo.hardwareRev,
                //                 (unsigned)cached.sbg.deviceInfo.firmwareRev,
                //                 (cached.sbg.deviceInfo.firmwareRev >> 22) & 0x3F,
                //                 (cached.sbg.deviceInfo.firmwareRev >> 16) & 0x3F,
                //                 cached.sbg.deviceInfo.firmwareRev & 0xFFFF

                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Serial Number: %u",(unsigned)cached.sbg.deviceInfo.serialNumber);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Version HW v%u.%u   FW v%u.%u.%u",
                                (unsigned)(cached.sbg.deviceInfo.hardwareRev >> 24) & 0x00FF,
                                (unsigned)(cached.sbg.deviceInfo.hardwareRev >> 16) & 0x00FF,

                                (unsigned) (cached.sbg.deviceInfo.firmwareRev >> 22) & 0x003F,
                                (unsigned)(cached.sbg.deviceInfo.firmwareRev >> 16) & 0x003F,
                                (unsigned)cached.sbg.deviceInfo.firmwareRev & 0xFFFF);
                break;

                case SBG_ECOM_CMD_COMPUTE_MAG_CALIB: // 15
                break;

            default:
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Unknown ID=%u, CLASS=%u, LEN=%u", (unsigned)msg.msgid, (unsigned)msg.msgclass, (unsigned)msg.len);
                break;
        }
        return;
    }



    if (((SbgEComClass)msg.msgclass != SBG_ECOM_CLASS_LOG_ECOM_0) && !valid_class1_msg) {
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Unknown ID=%u, CLASS=%u, LEN=%u", (unsigned)msg.msgid, (unsigned)msg.msgclass, (unsigned)msg.len);
        return;
    }

    const bool use_ekf_as_gnss = option_is_set(AP_ExternalAHRS::OPTIONS::SBG_EKF_AS_GNSS);

    bool updated_gps = false;
    bool updated_baro = false;
    bool updated_ins = false;
    bool updated_mag = false;
    bool updated_airspeed = false;

    {
        WITH_SEMAPHORE(state.sem);

        switch (msg.msgid) {  // (_SbgEComLog)
            case SBG_ECOM_LOG_FAST_IMU_DATA: // 0
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.imuFastLegacy, sizeof(cached.sbg.imuFastLegacy), msg.data, msg.len);

                state.accel = Vector3f(cached.sbg.imuFastLegacy.accelerometers[0], cached.sbg.imuFastLegacy.accelerometers[1], cached.sbg.imuFastLegacy.accelerometers[2]);
                state.gyro = Vector3f(cached.sbg.imuFastLegacy.gyroscopes[0], cached.sbg.imuFastLegacy.gyroscopes[1], cached.sbg.imuFastLegacy.gyroscopes[2]);
                updated_ins = true;
                break;

            case SBG_ECOM_LOG_UTC_TIME: // 2
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.utc, sizeof(cached.sbg.utc), msg.data, msg.len);

                if (sbgEComLogUtcGetClockStatus(cached.sbg.utc.status)    != SBG_ECOM_CLOCK_VALID ||
                    sbgEComLogUtcGetClockUtcStatus(cached.sbg.utc.status) != SBG_ECOM_UTC_VALID)
                {
                    // Data is not valid, ignore it.
                    // This will happen even with a GPS fix = 3. A 3D lock is not enough, a valid
                    // clock has a higher threshold of quiality needed. It also needs time to sync
                    // its internal timing and PPS outputs before valid.
                    break;
                }

                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "------------------");
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: %u-%u-%u %02u:%02u:%02u",
                //     cached.sbg.utc.year,
                //     (unsigned)cached.sbg.utc.month,
                //     (unsigned)cached.sbg.utc.day,
                //     (unsigned)cached.sbg.utc.hour,
                //     (unsigned)cached.sbg.utc.minute,
                //     (unsigned)cached.sbg.utc.second);

#if AP_RTC_ENABLED
                {
                    const uint32_t utc_epoch_sec = AP::rtc().date_fields_to_clock_s(
                        cached.sbg.utc.year,
                        cached.sbg.utc.month - 1,
                        cached.sbg.utc.day,
                        cached.sbg.utc.hour,
                        cached.sbg.utc.minute,
                        cached.sbg.utc.second);

                    const uint64_t utc_epoch_usec = ((uint64_t)utc_epoch_sec) * 1E6 + (cached.sbg.utc.nanoSecond * 1E-3);
                    AP::rtc().set_utc_usec(utc_epoch_usec, AP_RTC::SOURCE_GPS);

                }
#endif // AP_RTC_ENABLED

                cached.sensors.gps_data.ms_tow = cached.sbg.utc.gpsTimeOfWeek;
                cached.sensors.gps_data.gps_week = make_gps_week(&cached.sbg.utc);
                updated_gps = true;
                break;
            
            case SBG_ECOM_LOG_IMU_SHORT: // 44
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.imuShort, sizeof(cached.sbg.imuShort), msg.data, msg.len);

                {
                    Vector3f temp;
                    /*!< X, Y, Z delta velocity. Unit is 1048576 LSB for 1 m.s^-2. */
                    temp = Vector3f(cached.sbg.imuShort.deltaVelocity[0], cached.sbg.imuShort.deltaVelocity[1], cached.sbg.imuShort.deltaVelocity[2]);
                    state.accel = temp / SBG_ECOM_LOG_IMU_ACCEL_SCALE_STD;

                    /*!< X, Y, Z delta angle. Unit is either 67108864 LSB for 1 rad.s^-1 (standard) or 12304174 LSB for 1 rad.s^-1 (high range), managed automatically. */
                    temp = Vector3f(cached.sbg.imuShort.deltaAngle[0], cached.sbg.imuShort.deltaAngle[1], cached.sbg.imuShort.deltaAngle[2]);
                    const float scaleFactor = (cached.sbg.imuShort.status & SBG_ECOM_IMU_GYROS_USE_HIGH_SCALE) ? SBG_ECOM_LOG_IMU_GYRO_SCALE_HIGH : SBG_ECOM_LOG_IMU_GYRO_SCALE_STD;
                    state.gyro = temp / scaleFactor;
                }
                cached.sensors.ins_data.temperature = (cached.sbg.imuShort.temperature / SBG_ECOM_LOG_IMU_TEMP_SCALE_STD);
                updated_ins = true;
                break;

            case SBG_ECOM_LOG_IMU_DATA: // 3
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.imuLegacy, sizeof(cached.sbg.imuLegacy), msg.data, msg.len);

                state.accel = Vector3f(cached.sbg.imuLegacy.accelerometers[0], cached.sbg.imuLegacy.accelerometers[1], cached.sbg.imuLegacy.accelerometers[2]);
                state.gyro = Vector3f(cached.sbg.imuLegacy.gyroscopes[0], cached.sbg.imuLegacy.gyroscopes[1], cached.sbg.imuLegacy.gyroscopes[2]);
                cached.sensors.ins_data.temperature = cached.sbg.imuLegacy.temperature;
                updated_ins = true;
                break;

            case SBG_ECOM_LOG_MAG: // 4
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.mag, sizeof(cached.sbg.mag), msg.data, msg.len);

                state.accel = Vector3f(cached.sbg.mag.accelerometers[0], cached.sbg.mag.accelerometers[1], cached.sbg.mag.accelerometers[2]);
                updated_ins = true;

                cached.sensors.mag_data.field = Vector3f(cached.sbg.mag.magnetometers[0], cached.sbg.mag.magnetometers[1], cached.sbg.mag.magnetometers[2]);
                updated_mag = true;
                break;

            case SBG_ECOM_LOG_EKF_EULER: // 6
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.ekfEuler, sizeof(cached.sbg.ekfEuler), msg.data, msg.len);

                // float	euler[3];				/*!< Roll, Pitch and Yaw angles in rad. */
                // float	eulerStdDev[3];			/*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
                state.quat.from_euler(cached.sbg.ekfEuler.euler[0], cached.sbg.ekfEuler.euler[1], cached.sbg.ekfEuler.euler[2]);
                state.have_quaternion = true;
                break;
            
            case SBG_ECOM_LOG_EKF_QUAT: // 7
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.ekfQuat, sizeof(cached.sbg.ekfQuat), msg.data, msg.len);

                // float	quaternion[4];			/*!< Orientation quaternion stored in W, X, Y, Z form. */
                // float	eulerStdDev[3];			/*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
                memcpy(&state.quat, cached.sbg.ekfQuat.quaternion, sizeof(state.quat));
                state.have_quaternion = true;
                break;

            case SBG_ECOM_LOG_EKF_NAV: // 8
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.ekfNav, sizeof(cached.sbg.ekfNav), msg.data, msg.len);

                state.velocity = Vector3f(cached.sbg.ekfNav.velocity[0], cached.sbg.ekfNav.velocity[1], cached.sbg.ekfNav.velocity[2]);
                state.have_velocity = true;

                state.location = Location(cached.sbg.ekfNav.position[0]*1e7, cached.sbg.ekfNav.position[1]*1e7, cached.sbg.ekfNav.position[2]*1e2, Location::AltFrame::ABSOLUTE);
                state.last_location_update_us = AP_HAL::micros();

                ekf_is_full_nav = SbgEkfStatus_is_fullNav(cached.sbg.ekfNav.status);

                if (!state.have_location && ekf_is_full_nav) {
                    state.have_location = true;
                } else if (!state.have_origin && cached.sensors.gps_data.fix_type >= AP_GPS_FixType::FIX_3D && ekf_is_full_nav) {
                    // this is in an else so that origin doesn't get set on the very very first sample, do it on the second one just to give us a tiny bit more chance of a better origin
                    state.origin = state.location;
                    state.have_origin = true;
                }

                if (ekf_is_full_nav && use_ekf_as_gnss) {
                    cached.sensors.gps_data.latitude = cached.sbg.ekfNav.position[0] * 1E7;
                    cached.sensors.gps_data.longitude = cached.sbg.ekfNav.position[1] * 1E7;
                    cached.sensors.gps_data.msl_altitude = cached.sbg.ekfNav.position[2] * 100;

                    cached.sensors.gps_data.horizontal_pos_accuracy = Vector2f(cached.sbg.ekfNav.positionStdDev[0], cached.sbg.ekfNav.positionStdDev[1]).length();
                    cached.sensors.gps_data.hdop = cached.sensors.gps_data.horizontal_pos_accuracy;
                    cached.sensors.gps_data.vertical_pos_accuracy = cached.sbg.ekfNav.positionStdDev[2];
                    cached.sensors.gps_data.vdop =  cached.sensors.gps_data.vertical_pos_accuracy;

                    cached.sensors.gps_data.fix_type = AP_GPS_FixType::FIX_3D;

                    cached.sensors.gps_data.ned_vel_north = cached.sbg.ekfNav.velocity[0];
                    cached.sensors.gps_data.ned_vel_east = cached.sbg.ekfNav.velocity[1];
                    cached.sensors.gps_data.ned_vel_down = cached.sbg.ekfNav.velocity[2];
                    cached.sensors.gps_data.horizontal_vel_accuracy = Vector2f(cached.sbg.ekfNav.velocityStdDev[0], cached.sbg.ekfNav.velocityStdDev[1]).length();

                    updated_gps = true;
                }
                break;

            case SBG_ECOM_LOG_GPS1_VEL: // 13
            case SBG_ECOM_LOG_GPS2_VEL: // 16
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.gnssVel, sizeof(cached.sbg.gnssVel), msg.data, msg.len);

                if ((!use_ekf_as_gnss) || (use_ekf_as_gnss && !ekf_is_full_nav)) {
                    cached.sensors.gps_data.ms_tow = cached.sbg.gnssVel.timeOfWeek;
                    cached.sensors.gps_data.ned_vel_north = cached.sbg.gnssVel.velocity[0];
                    cached.sensors.gps_data.ned_vel_east = cached.sbg.gnssVel.velocity[1];
                    cached.sensors.gps_data.ned_vel_down = cached.sbg.gnssVel.velocity[2];
                    cached.sensors.gps_data.horizontal_vel_accuracy = Vector2f(cached.sbg.gnssVel.velocityAcc[0], cached.sbg.gnssVel.velocityAcc[1]).length();
                    // unused - cached.sbg.gnssVel.course
                    // unused - cached.sbg.gnssVel.courseAcc
                    updated_gps = true;
                }
                break;

            case SBG_ECOM_LOG_GPS1_POS: // 14
            case SBG_ECOM_LOG_GPS2_POS: // 17
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.gnssPos, sizeof(cached.sbg.gnssPos), msg.data, msg.len);

                if ((!use_ekf_as_gnss) || (use_ekf_as_gnss && !ekf_is_full_nav)) {
                    cached.sensors.gps_data.ms_tow = cached.sbg.gnssPos.timeOfWeek;
                    cached.sensors.gps_data.latitude = cached.sbg.gnssPos.latitude * 1E7;
                    cached.sensors.gps_data.longitude = cached.sbg.gnssPos.longitude * 1E7;
                    cached.sensors.gps_data.msl_altitude = cached.sbg.gnssPos.altitude * 100;
                    // unused - cached.sbg.gnssPos.undulation
                    cached.sensors.gps_data.horizontal_pos_accuracy = Vector2f(cached.sbg.gnssPos.latitudeAccuracy, cached.sbg.gnssPos.longitudeAccuracy).length();
                    cached.sensors.gps_data.hdop = cached.sensors.gps_data.horizontal_pos_accuracy;
                    cached.sensors.gps_data.vertical_pos_accuracy = cached.sbg.gnssPos.altitudeAccuracy;
                    cached.sensors.gps_data.vdop =  cached.sensors.gps_data.vertical_pos_accuracy;
                    cached.sensors.gps_data.satellites_in_view = cached.sbg.gnssPos.numSvUsed;
                    // unused - cached.sbg.gnssPos.baseStationId
                    // unused - cached.sbg.gnssPos.differentialAge
                    cached.sensors.gps_data.fix_type = SbgGpsPosStatus_to_GpsFixType(cached.sbg.gnssPos.status);
                    updated_gps = true;
                }
                break;

            case SBG_ECOM_LOG_AIR_DATA: // 36
                safe_copy_msg_to_object((uint8_t*)&cached.sbg.airData, sizeof(cached.sbg.airData), msg.data, msg.len);
                
                if (cached.sbg.airData.status & SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID) {
                    cached.sensors.baro_data.pressure_pa = cached.sbg.airData.pressureAbs;
                    updated_baro = true;
                }
                if (cached.sbg.airData.status & SBG_ECOM_AIR_DATA_ALTITUDE_VALID) {
                    cached.sensors.baro_height = cached.sbg.airData.altitude;
                    updated_baro = true;
                }
                if (cached.sbg.airData.status & SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID) {
                    cached.sensors.airspeed_data.differential_pressure = cached.sbg.airData.pressureDiff;
                    updated_airspeed = true;
                }
                // if (cached.sbg.airData.status & SBG_ECOM_AIR_DATA_AIRPSEED_VALID) {
                //     // we don't accept airspeed directly, we compute it ourselves in AP_Airspeed via diff pressure
                // }
                
                if ((cached.sbg.airData.status & SBG_ECOM_AIR_DATA_TEMPERATURE_VALID) && (updated_baro || updated_airspeed)) {
                    cached.sensors.airspeed_data.temperature = cached.sbg.airData.airTemperature;
                    cached.sensors.baro_data.temperature = cached.sbg.airData.airTemperature;
                }
                break;

            default:
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: unhandled ID=%u, CLASS=%u, LEN=%u", (unsigned)msg.msgid, (unsigned)msg.msgclass, (unsigned)msg.len);
                return;
        } // switch msgid
    } // semaphore

#if AP_GPS_ENABLED
    if (updated_gps) {
        cached.sensors.gps_ms = now_ms;
        uint8_t instance;
        if (AP::gps().get_first_external_instance(instance)) {
            AP::gps().handle_external(cached.sensors.gps_data, instance);
        }
    }
#else
    (void)updated_gps;
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    if (updated_mag) {
        cached.sensors.mag_ms = now_ms;
        AP::compass().handle_external(cached.sensors.mag_data);
    }
#else
    (void)updated_mag;
#endif

#if AP_BARO_EXTERNALAHRS_ENABLED
    if (updated_baro) {
        cached.sensors.baro_ms = now_ms;
        cached.sensors.baro_data.instance = 0;
        AP::baro().handle_external(cached.sensors.baro_data);
    }
#else
    (void)updated_baro;
#endif

#if AP_AIRSPEED_EXTERNAL_ENABLED && (APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane))
    if (updated_airspeed && AP::airspeed() != nullptr) {
        cached.sensors.airspeed_ms = now_ms;
        AP::airspeed()->handle_external(cached.sensors.airspeed_data);
    }
#else
    (void)updated_airspeed;
#endif


    if (updated_ins) {
        cached.sensors.ins_data.accel = state.accel;
        cached.sensors.ins_data.gyro = state.gyro;
        cached.sensors.ins_ms = now_ms;
        AP::ins().handle_external(cached.sensors.ins_data);
    }

    last_received_ms = now_ms;
}

void AP_ExternalAHRS_SBG::safe_copy_msg_to_object(uint8_t* dest, const uint16_t dest_len, const uint8_t* src, const uint16_t src_len)
{
    // To allow for future changes of the SBG protocol the protocol allows extending messages
    // which can be detected by a length mismatch, usually longer. To allow for this you assume unused data is zero
    // but instead of zeroing the packet every time before populating it it's best to only zero when necessary.
    if (dest_len != src_len) {
        memset(dest, 0, dest_len);
    }
    memcpy(dest, src, MIN(dest_len,src_len));
}

bool AP_ExternalAHRS_SBG::SbgEkfStatus_is_fullNav(const uint32_t ekfStatus)
{
    SbgEComSolutionMode solutionMode = (SbgEComSolutionMode)(ekfStatus & SBG_ECOM_LOG_EKF_SOLUTION_MODE_MASK);

    return (solutionMode == SBG_ECOM_SOL_MODE_NAV_POSITION);
}

AP_GPS_FixType AP_ExternalAHRS_SBG::SbgGpsPosStatus_to_GpsFixType(const uint32_t gpsPosStatus)
{
    const uint32_t fix = (gpsPosStatus >> SBG_ECOM_GPS_POS_TYPE_SHIFT) & SBG_ECOM_GPS_POS_TYPE_MASK;
    switch ((SbgEComGpsPosType)fix) {
        case SBG_ECOM_POS_NO_SOLUTION:      /*!< No valid solution available. */
        case SBG_ECOM_POS_UNKNOWN_TYPE:     /*!< An unknown solution type has been computed. */
            return AP_GPS_FixType::NONE;

        case SBG_ECOM_POS_SINGLE:           /*!< Single point solution position. */
        case SBG_ECOM_POS_FIXED:            /*!< Fixed location solution position. */
            return AP_GPS_FixType::FIX_3D;

        case SBG_ECOM_POS_PSRDIFF:          /*!< Standard Pseudorange Differential Solution (DGPS). */
        case SBG_ECOM_POS_SBAS:             /*!< SBAS satellite used for differential corrections. */
            return AP_GPS_FixType::DGPS;

        case SBG_ECOM_POS_RTK_FLOAT:        /*!< Floating RTK ambiguity solution (20 cms RTK). */
        case SBG_ECOM_POS_PPP_FLOAT:        /*!< Precise Point Positioning with float ambiguities. */
        case SBG_ECOM_POS_OMNISTAR:         /*!< Omnistar VBS Position (L1 sub-meter). */
            return AP_GPS_FixType::RTK_FLOAT;

        case SBG_ECOM_POS_RTK_INT:          /*!< Integer RTK ambiguity solution (2 cms RTK). */
        case SBG_ECOM_POS_PPP_INT:          /*!< Precise Point Positioning with fixed ambiguities. */
            return AP_GPS_FixType::RTK_FIXED;
    }
    return AP_GPS_FixType::NONE;
}

void AP_ExternalAHRS_SBG::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);
    memset(&status, 0, sizeof(status));

    if (cached.sensors.ins_ms != 0 && cached.sensors.gps_ms != 0) {
        status.flags.initalized = true;
    }
    if (!healthy()) {
        return;
    }

    if (state.have_location) {
        status.flags.vert_pos = true;
        status.flags.horiz_pos_rel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = true;
    }

    if (state.have_quaternion) {
        status.flags.attitude = true;
    }

    if (state.have_velocity) {
        status.flags.vert_vel = true;
        status.flags.horiz_vel = true;
    }
}

bool AP_ExternalAHRS_SBG::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SBG setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SBG unhealthy");
        return false;
    }
    return true;
}

#if AP_COMPASS_ENABLED
bool AP_ExternalAHRS_SBG::send_MagData(AP_HAL::UARTDriver *_uart)
{
    SbgEComLogMag mag_data_log {};
    mag_data_log.timeStamp = AP_HAL::micros();

    const auto &compass = AP::compass();
    if (compass.available() || compass.healthy()) {
        // TODO: consider also checking compass.last_update_usec() to only send when we have new data

        const Vector3f mag_field = compass.get_field();
        mag_data_log.magnetometers[0] = mag_field[0];
        mag_data_log.magnetometers[1] = mag_field[1];
        mag_data_log.magnetometers[2] = mag_field[2];

        mag_data_log.status |= (SBG_ECOM_MAG_MAG_X_BIT | SBG_ECOM_MAG_MAG_Y_BIT | SBG_ECOM_MAG_MAG_Z_BIT | SBG_ECOM_MAG_MAGS_IN_RANGE | SBG_ECOM_MAG_CALIBRATION_OK);
    }

    const sbgMessage msg = sbgMessage(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, (uint8_t*)&mag_data_log, sizeof(mag_data_log));
    return send_sbgMessage(_uart, msg);
}
#endif // AP_COMPASS_ENABLED

bool AP_ExternalAHRS_SBG::send_AirData(AP_HAL::UARTDriver *_uart)
{
    SbgEComLogAirData air_data_log {};
    air_data_log.timeStamp = 0;
    air_data_log.status |= SBG_ECOM_AIR_DATA_TIME_IS_DELAY;

#if AP_BARO_ENABLED
    const auto &baro = AP::baro();
    if (baro.healthy()) {
        air_data_log.pressureAbs = baro.get_pressure();
        air_data_log.altitude = baro.get_altitude();
        air_data_log.airTemperature = baro.get_temperature();

        air_data_log.status |= (SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID | SBG_ECOM_AIR_DATA_ALTITUDE_VALID | SBG_ECOM_AIR_DATA_TEMPERATURE_VALID);
    }
#endif // AP_BARO_ENABLED

#if AP_AIRSPEED_ENABLED
    auto *airspeed = AP::airspeed();
    if (airspeed != nullptr && airspeed->healthy()) {
        float airTemperature;
        if (airspeed->get_temperature(airTemperature)) {
            air_data_log.airTemperature = airTemperature;
            air_data_log.status |= SBG_ECOM_AIR_DATA_TEMPERATURE_VALID;
        }

        air_data_log.pressureDiff = airspeed->get_differential_pressure();
        air_data_log.trueAirspeed = airspeed->get_airspeed();
        air_data_log.status |= (SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID | SBG_ECOM_AIR_DATA_AIRPSEED_VALID);
    }
#endif // AP_AIRSPEED_ENABLED

    const sbgMessage msg = sbgMessage(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_AIR_DATA, (uint8_t*)&air_data_log, sizeof(air_data_log));
    return send_sbgMessage(_uart, msg);
}

// get variances
bool AP_ExternalAHRS_SBG::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = cached.sensors.gps_data.horizontal_vel_accuracy * vel_gate_scale;
    posVar = cached.sensors.gps_data.horizontal_pos_accuracy * pos_gate_scale;
    hgtVar = cached.sensors.gps_data.vertical_pos_accuracy * hgt_gate_scale;
    magVar.zero(); // Not provided, set to 0.
    tasVar = 0;
    return true;
}

#endif  // AP_EXTERNAL_AHRS_SBG_ENABLED

