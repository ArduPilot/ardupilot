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
  Simulate SBG serial INS
  Converts SITL flight dynamics to sbgECom protocol frames
*/

#include "SIM_config.h"

#if AP_SIM_EAHRS_SBG_ENABLED

#include "SIM_SBG.h"
#include <SITL/SITL.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS_SBG_structs.h>
#include <sys/time.h>
#include <time.h>

#define GPS_LEAPSECONDS_MILLIS 18000ULL

using namespace SITL;

SBG::SBG() : SerialDevice::SerialDevice()
{
}

void SBG::simulation_timeval(struct timeval *tv)
{
    uint64_t now = AP_HAL::micros64();
    static uint64_t first_usec;
    static struct timeval first_tv;
    if (first_usec == 0) {
        first_usec = now;
        first_tv.tv_sec = AP::sitl()->start_time_UTC;
    }
    *tv = first_tv;
    tv->tv_sec += now / 1000000ULL;
    uint64_t new_usec = tv->tv_usec + (now % 1000000ULL);
    tv->tv_sec += new_usec / 1000000ULL;
    tv->tv_usec = new_usec % 1000000ULL;
}

uint32_t SBG::gps_tow_ms(const struct timeval &tv)
{
    const uint32_t epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - (GPS_LEAPSECONDS_MILLIS / 1000ULL);
    const uint32_t epoch_seconds = tv.tv_sec - epoch;
    return (epoch_seconds % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC + tv.tv_usec / 1000ULL;
}

// frame layout: SYNC1 SYNC2 MSG CLASS LEN_LSB LEN_MSB DATA CRC_LSB CRC_MSB ETX
// CRC is computed over the [MSG, CLASS, LEN, DATA] fields
void SBG::send_frame(uint8_t msgclass, uint8_t msgid, const void *payload, uint16_t len)
{
    uint8_t buffer[9 + len];
    buffer[0] = 0xFF;
    buffer[1] = 0x5A;
    buffer[2] = msgid;
    buffer[3] = msgclass;
    buffer[4] = len & 0xFF;
    buffer[5] = len >> 8;
    memcpy(&buffer[6], payload, len);
    const uint16_t crc = crc16_ccitt_r(&buffer[2], len + 4, 0, 0);
    buffer[6 + len] = crc & 0xFF;
    buffer[7 + len] = crc >> 8;
    buffer[8 + len] = 0x33;

    if (to_autopilot == nullptr || to_autopilot->space() < sizeof(buffer)) {
        // drop the whole frame: a partially written frame corrupts the
        // stream and desyncs the driver parser for a long time. This
        // happens when the autopilot drains slower than we produce, e.g.
        // during boot or at high sim speedups
        return;
    }
    write_to_autopilot((char *)buffer, sizeof(buffer));
}

void SBG::send_device_info()
{
    SbgEComDeviceInfo info {};
    strncpy((char *)info.productCode, "ELLIPSE-N-SITL", sizeof(info.productCode) - 1);
    info.serialNumber = 12345;
    info.calibationRev = 1;
    info.calibrationYear = 2025;
    info.calibrationMonth = 1;
    info.calibrationDay = 1;
    info.hardwareRev = 1U << 24;    // v1.0
    info.firmwareRev = 1U << 22;    // v1.0.0

    send_frame(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INFO, &info, sizeof(info));
}

void SBG::send_utc_time()
{
    struct timeval tv;
    simulation_timeval(&tv);

    struct tm tm {};
    struct tm *tm_ptr = gmtime_r(&tv.tv_sec, &tm);
    if (tm_ptr == nullptr) {
        return;
    }

    SbgEComLogUtc utc {};
    utc.timeStamp = AP_HAL::micros();
    utc.status = (SBG_ECOM_CLOCK_VALID << SBG_ECOM_CLOCK_STATUS_SHIFT) |
                 (SBG_ECOM_UTC_VALID << SBG_ECOM_CLOCK_UTC_STATUS_SHIFT);
    utc.year = tm.tm_year + 1900;
    utc.month = tm.tm_mon + 1;
    utc.day = tm.tm_mday;
    utc.hour = tm.tm_hour;
    utc.minute = tm.tm_min;
    utc.second = tm.tm_sec;
    utc.nanoSecond = tv.tv_usec * 1000;
    utc.gpsTimeOfWeek = gps_tow_ms(tv);

    send_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, &utc, sizeof(utc));
}

void SBG::send_imu_short()
{
    const auto &fdm = _sitl->state;

    SbgEComLogImuShort imu {};
    imu.timeStamp = AP_HAL::micros();
    imu.status = SBG_ECOM_IMU_COM_OK | SBG_ECOM_IMU_STATUS_BIT |
                 SBG_ECOM_IMU_ACCEL_X_BIT | SBG_ECOM_IMU_ACCEL_Y_BIT | SBG_ECOM_IMU_ACCEL_Z_BIT |
                 SBG_ECOM_IMU_GYRO_X_BIT | SBG_ECOM_IMU_GYRO_Y_BIT | SBG_ECOM_IMU_GYRO_Z_BIT |
                 SBG_ECOM_IMU_ACCELS_IN_RANGE | SBG_ECOM_IMU_GYROS_IN_RANGE;
    imu.deltaVelocity[0] = fdm.xAccel * SBG_ECOM_LOG_IMU_ACCEL_SCALE_STD;
    imu.deltaVelocity[1] = fdm.yAccel * SBG_ECOM_LOG_IMU_ACCEL_SCALE_STD;
    imu.deltaVelocity[2] = fdm.zAccel * SBG_ECOM_LOG_IMU_ACCEL_SCALE_STD;
    imu.deltaAngle[0] = radians(fdm.rollRate) * SBG_ECOM_LOG_IMU_GYRO_SCALE_STD;
    imu.deltaAngle[1] = radians(fdm.pitchRate) * SBG_ECOM_LOG_IMU_GYRO_SCALE_STD;
    imu.deltaAngle[2] = radians(fdm.yawRate) * SBG_ECOM_LOG_IMU_GYRO_SCALE_STD;
    imu.temperature = 25 * SBG_ECOM_LOG_IMU_TEMP_SCALE_STD;

    send_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_SHORT, &imu, sizeof(imu));
}

void SBG::send_mag()
{
    const auto &fdm = _sitl->state;

    SbgEComLogMag mag {};
    mag.timeStamp = AP_HAL::micros();
    mag.status = SBG_ECOM_MAG_MAG_X_BIT | SBG_ECOM_MAG_MAG_Y_BIT | SBG_ECOM_MAG_MAG_Z_BIT |
                 SBG_ECOM_MAG_ACCEL_X_BIT | SBG_ECOM_MAG_ACCEL_Y_BIT | SBG_ECOM_MAG_ACCEL_Z_BIT |
                 SBG_ECOM_MAG_MAGS_IN_RANGE | SBG_ECOM_MAG_ACCELS_IN_RANGE | SBG_ECOM_MAG_CALIBRATION_OK;
    // the driver passes the field through unscaled, so send milliGauss
    mag.magnetometers[0] = fdm.bodyMagField.x;
    mag.magnetometers[1] = fdm.bodyMagField.y;
    mag.magnetometers[2] = fdm.bodyMagField.z;
    mag.accelerometers[0] = fdm.xAccel;
    mag.accelerometers[1] = fdm.yAccel;
    mag.accelerometers[2] = fdm.zAccel;

    send_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, &mag, sizeof(mag));
}

void SBG::send_ekf_quat()
{
    const auto &fdm = _sitl->state;

    SbgEComLogEkfQuat quat {};
    quat.timeStamp = AP_HAL::micros();
    quat.quaternion[0] = fdm.quaternion.q1; // W
    quat.quaternion[1] = fdm.quaternion.q2; // X
    quat.quaternion[2] = fdm.quaternion.q3; // Y
    quat.quaternion[3] = fdm.quaternion.q4; // Z
    quat.eulerStdDev[0] = 0.001f;
    quat.eulerStdDev[1] = 0.001f;
    quat.eulerStdDev[2] = 0.001f;
    quat.status = SBG_ECOM_SOL_MODE_NAV_POSITION;

    send_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, &quat, sizeof(quat));
}

void SBG::send_ekf_nav()
{
    const auto &fdm = _sitl->state;

    SbgEComLogEkfNav nav {};
    nav.timeStamp = AP_HAL::micros();
    nav.velocity[0] = fdm.speedN;
    nav.velocity[1] = fdm.speedE;
    nav.velocity[2] = fdm.speedD;
    nav.velocityStdDev[0] = 0.1f;
    nav.velocityStdDev[1] = 0.1f;
    nav.velocityStdDev[2] = 0.1f;
    nav.position[0] = fdm.latitude;
    nav.position[1] = fdm.longitude;
    nav.position[2] = fdm.altitude;
    nav.undulation = 0;
    nav.positionStdDev[0] = 0.3f;
    nav.positionStdDev[1] = 0.3f;
    nav.positionStdDev[2] = 0.4f;
    nav.status = SBG_ECOM_SOL_MODE_NAV_POSITION;

    send_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, &nav, sizeof(nav));
}

void SBG::send_gps()
{
    const auto &fdm = _sitl->state;

    struct timeval tv;
    simulation_timeval(&tv);
    const uint32_t tow_ms = gps_tow_ms(tv);

    SbgEComLogGnssPos pos {};
    pos.timeStamp = AP_HAL::micros();
    pos.status = (uint32_t)SBG_ECOM_POS_SINGLE << SBG_ECOM_GPS_POS_TYPE_SHIFT;
    pos.timeOfWeek = tow_ms;
    pos.latitude = fdm.latitude;
    pos.longitude = fdm.longitude;
    pos.altitude = fdm.altitude;
    pos.undulation = 0;
    pos.latitudeAccuracy = 0.5f;
    pos.longitudeAccuracy = 0.5f;
    pos.altitudeAccuracy = 0.7f;
    pos.numSvUsed = 19;
    pos.baseStationId = 0xFFFF;
    pos.differentialAge = 0xFFFF;

    send_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, &pos, sizeof(pos));

    SbgEComLogGnssVel vel {};
    vel.timeStamp = AP_HAL::micros();
    vel.status = 0;
    vel.timeOfWeek = tow_ms;
    vel.velocity[0] = fdm.speedN;
    vel.velocity[1] = fdm.speedE;
    vel.velocity[2] = fdm.speedD;
    vel.velocityAcc[0] = 0.25f;
    vel.velocityAcc[1] = 0.25f;
    vel.velocityAcc[2] = 0.25f;
    vel.course = wrap_360(degrees(atan2f(fdm.speedE, fdm.speedN)));
    vel.courseAcc = 5.0f;

    send_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_VEL, &vel, sizeof(vel));
}

void SBG::send_air_data()
{
    const auto &fdm = _sitl->state;

    float pressure_pa;
    float temp_K;
    // a little noise is needed: AP_Baro declares a barometer whose
    // readings never change unhealthy
    AP_Baro::get_pressure_temperature_for_alt_amsl(fdm.altitude + rand_float() * 0.25f, pressure_pa, temp_K);

    SbgEComLogAirData air {};
    air.timeStamp = AP_HAL::micros();
    air.status = SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID | SBG_ECOM_AIR_DATA_ALTITUDE_VALID |
                 SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID | SBG_ECOM_AIR_DATA_AIRPSEED_VALID |
                 SBG_ECOM_AIR_DATA_TEMPERATURE_VALID;
    air.pressureAbs = pressure_pa;
    air.altitude = fdm.altitude;
    air.pressureDiff = fdm.airspeed_raw_pressure[0];
    air.trueAirspeed = fdm.airspeed;
    air.airTemperature = KELVIN_TO_C(temp_K);

    send_frame(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_AIR_DATA, &air, sizeof(air));
}

void SBG::check_incoming()
{
    uint8_t buf[64];
    while (true) {
        const ssize_t n = read_from_autopilot((char *)buf, sizeof(buf));
        if (n <= 0) {
            return;
        }
        seen_autopilot_traffic = true;
        for (ssize_t i = 0; i < n; i++) {
            const uint8_t b = buf[i];
            switch (rx.state) {
                case ParseState::SYNC1:
                    rx.state = (b == 0xFF) ? ParseState::SYNC2 : ParseState::SYNC1;
                    break;
                case ParseState::SYNC2:
                    rx.state = (b == 0x5A) ? ParseState::MSG : ParseState::SYNC1;
                    break;
                case ParseState::MSG:
                    rx.msgid = b;
                    rx.state = ParseState::CLASS;
                    break;
                case ParseState::CLASS:
                    rx.msgclass = b;
                    rx.state = ParseState::LEN1;
                    break;
                case ParseState::LEN1:
                    rx.len = b;
                    rx.state = ParseState::LEN2;
                    break;
                case ParseState::LEN2:
                    rx.len |= uint16_t(b) << 8;
                    rx.count = 0;
                    rx.state = (rx.len > 0) ? ParseState::DATA : ParseState::CRC1;
                    break;
                case ParseState::DATA:
                    // payload is not needed to answer requests, just skip it
                    if (++rx.count >= rx.len) {
                        rx.state = ParseState::CRC1;
                    }
                    break;
                case ParseState::CRC1:
                    rx.state = ParseState::CRC2;
                    break;
                case ParseState::CRC2:
                    rx.state = ParseState::ETX;
                    break;
                case ParseState::ETX:
                    rx.state = ParseState::SYNC1;
                    if (b != 0x33) {
                        break;
                    }
                    if ((SbgEComClass)rx.msgclass == SBG_ECOM_CLASS_LOG_CMD_0 &&
                        (SbgEComCmd)rx.msgid == SBG_ECOM_CMD_INFO) {
                        send_device_info();
                    }
                    break;
            }
        }
    }
}

/*
  send SBG data
 */
void SBG::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }

    check_incoming();

    const uint32_t ms_between_ins_packets = 40;
    // AP_GPS requires an average delta below 215ms to report a healthy
    // GPS, which a 5Hz rate only meets marginally: use 10Hz
    const uint32_t ms_between_gps_packets = 100;

    const uint32_t now = AP_HAL::millis();

    // GPS logs are sent first: when the buffer can't hold everything the
    // dropped frames must not be GPS ones, or AP_GPS reports an unhealthy
    // (too slow) GPS
    if (now - last_gps_pkt_ms >= ms_between_gps_packets) {
        last_gps_pkt_ms = now;
        send_gps();
        // the driver pushes GPS data to the AP_GPS frontend as soon as it
        // receives a valid UTC log. Hold UTC back until the autopilot is
        // known to be up so a position log is always parsed first,
        // otherwise SITL panics on a zero lat/lon push
        if (seen_autopilot_traffic) {
            send_utc_time();
        }
        send_ekf_nav();
        send_mag();
        send_air_data();
    }

    if (now - last_ins_pkt_ms >= ms_between_ins_packets) {
        last_ins_pkt_ms = now;
        send_ekf_quat();
        send_imu_short();
    }
}

#endif  // AP_SIM_EAHRS_SBG_ENABLED
