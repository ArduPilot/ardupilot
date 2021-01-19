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
  suppport for serial connected AHRS systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <stdio.h>

#if HAL_EXTERNAL_AHRS_ENABLED

extern const AP_HAL::HAL &hal;
HAL_Semaphore AP_ExternalAHRS::sem;

/*
  header for pre-configured 50Hz data
  assumes the following config for VN-300:
    $VNWRG,75,3,8,35,0003,0F2C,0147,0613*2642
*/
static const uint8_t vn_pkt1_header[] { 0x35, 0x03, 0x00, 0x2c, 0x0f, 0x47, 0x01, 0x13, 0x06 };
#define VN_PKT1_LENGTH 194 // includes header

struct PACKED VN_packet1 {
    uint64_t timeStartup;
    uint64_t timeGPS;
    float uncompAccel[3];
    float uncompAngRate[3];
    float pressure;
    float mag[3];
    float accel[3];
    float gyro[3];
    uint16_t sensSat;
    uint16_t AHRSStatus;
    float ypr[3];
    float quaternion[4];
    float linAccBody[3];
    float yprU[3];
    uint16_t INSStatus;
    double positionLLA[3];
    float velNED[3];
    float posU;
    float velU;
};

// check packet size for 4 groups
static_assert(sizeof(VN_packet1)+2+4*2+2 == VN_PKT1_LENGTH, "incorrect VN_packet1 length");

/*
  header for pre-configured 5Hz data
  assumes the following VN-300 config:
    $VNWRG,76,3,80,4E,0002,0010,20B8,2018*A66B
*/
static const uint8_t vn_pkt2_header[] { 0x4e, 0x02, 0x00, 0x10, 0x00, 0xb8, 0x20, 0x18, 0x20 };
#define VN_PKT2_LENGTH 120 // includes header

struct PACKED VN_packet2 {
    uint64_t timeGPS;
    float temp;
    uint8_t numGPS1Sats;
    uint8_t GPS1Fix;
    double GPS1posLLA[3];
    float GPS1velNED[3];
    float GPS1DOP[7];
    uint8_t numGPS2Sats;
    uint8_t GPS2Fix;
    float GPS2DOP[7];
};

// check packet size for 4 groups
static_assert(sizeof(VN_packet2)+2+4*2+2 == VN_PKT2_LENGTH, "incorrect VN_packet2 length");

AP_ExternalAHRS *AP_ExternalAHRS::_singleton;

// constructor
AP_ExternalAHRS::AP_ExternalAHRS()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

#ifndef HAL_EXTERNAL_AHRS_DEFAULT
#define HAL_EXTERNAL_AHRS_DEFAULT 0
#endif


// table of user settable parameters
const AP_Param::GroupInfo AP_ExternalAHRS::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: AHRS type
    // @Description: Type of AHRS device
    // @Values: 0:None,1:VectorNav
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_ExternalAHRS, devtype, HAL_EXTERNAL_AHRS_DEFAULT, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};


void AP_ExternalAHRS::init(void)
{
    if (devtype != DevType::VecNav) {
        return;
    }
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    bufsize = MAX(VN_PKT1_LENGTH, VN_PKT2_LENGTH);
    pktbuf = new uint8_t[bufsize];
    last_pkt1 = new VN_packet1;
    last_pkt2 = new VN_packet2;

    if (!pktbuf || !last_pkt1 || !last_pkt2) {
        AP_HAL::panic("Failed to allocate ExternalAHRS");
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS initialised");
}

/*
  check the UART for more data
  returns true if the function should be called again straight away
 */
bool AP_ExternalAHRS::check_uart()
{
    if (!port_opened) {
        return false;
    }
    WITH_SEMAPHORE(sem);

    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }
    if (pktoffset < bufsize) {
        ssize_t nread = uart->read(&pktbuf[pktoffset], MIN(n, unsigned(bufsize-pktoffset)));
        if (nread <= 0) {
            return false;
        }
        pktoffset += nread;
    }

    bool match_header1, match_header2;

    if (pktbuf[0] != 0xFA) {
        goto reset;
    }

    match_header1 = (0 == memcmp(&pktbuf[1], vn_pkt1_header, MIN(sizeof(vn_pkt1_header), unsigned(pktoffset-1))));
    match_header2 = (0 == memcmp(&pktbuf[1], vn_pkt2_header, MIN(sizeof(vn_pkt2_header), unsigned(pktoffset-1))));
    if (!match_header1 && !match_header2) {
        goto reset;
    }

    if (match_header1 && pktoffset >= VN_PKT1_LENGTH) {
        uint16_t crc = crc16_ccitt(&pktbuf[1], VN_PKT1_LENGTH-1, 0);
        if (crc == 0) {
            // got pkt1
            process_packet1(&pktbuf[sizeof(vn_pkt1_header)+1]);
            memmove(&pktbuf[0], &pktbuf[VN_PKT1_LENGTH], pktoffset-VN_PKT1_LENGTH);
            pktoffset -= VN_PKT1_LENGTH;
        } else {
            goto reset;
        }
    } else if (match_header2 && pktoffset >= VN_PKT2_LENGTH) {
        uint16_t crc = crc16_ccitt(&pktbuf[1], VN_PKT2_LENGTH-1, 0);
        if (crc == 0) {
            // got pkt2
            process_packet2(&pktbuf[sizeof(vn_pkt2_header)+1]);
            memmove(&pktbuf[0], &pktbuf[VN_PKT2_LENGTH], pktoffset-VN_PKT2_LENGTH);
            pktoffset -= VN_PKT2_LENGTH;
        } else {
            goto reset;
        }
    }
    return true;

reset:
    uint8_t *p = (uint8_t *)memchr(&pktbuf[1], (char)0xFA, pktoffset-1);
    if (p) {
        uint8_t newlen = pktoffset - (p - pktbuf);
        memmove(&pktbuf[0], p, newlen);
        pktoffset = newlen;
    } else {
        pktoffset = 0;
    }
    return true;
}

void AP_ExternalAHRS::update_thread()
{
    if (!port_opened) {
        // open port in the thread
        port_opened = true;
        uart->begin(baudrate, 1024, 512);
    }

    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay(1);
        }
    }
}

/*
  process packet type 1
 */
void AP_ExternalAHRS::process_packet1(const uint8_t *b)
{
    const struct VN_packet1 &pkt1 = *(struct VN_packet1 *)b;
    const struct VN_packet2 &pkt2 = *last_pkt2;

    last_pkt1_ms = AP_HAL::millis();
    *last_pkt1 = pkt1;

    {
        baro_data_message_t baro;
        baro.instance = 0;
        baro.pressure_pa = pkt1.pressure*1e3;
        baro.temperature = pkt2.temp;

        AP::baro().handle_external(baro);
    }

    {
        mag_data_message_t mag;
        mag.field = Vector3f{pkt1.mag[0], pkt1.mag[1], pkt1.mag[2]};
        mag.field *= 1000; // to mGauss

        AP::compass().handle_external(mag);
    }

    {
        ins_data_message_t ins;

        ins.accel = Vector3f{pkt1.accel[0], pkt1.accel[1], pkt1.accel[2]};
        ins.gyro = Vector3f{pkt1.gyro[0], pkt1.gyro[1], pkt1.gyro[2]};
        ins.temperature = pkt2.temp;

        AP::ins().handle_external(ins);
    }

    // @LoggerMessage: EAH1
    // @Description: External AHRS data
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
    // @Field: UXY: uncertainty in XY position
    // @Field: UV: uncertainty in velocity
    // @Field: UR: uncertainty in roll
    // @Field: UP: uncertainty in pitch
    // @Field: UY: uncertainty in yaw

    AP::logger().Write("EAH1", "TimeUS,Roll,Pitch,Yaw,VN,VE,VD,Lat,Lon,Alt,UXY,UV,UR,UP,UY",
                       "sdddnnnDUmmnddd", "F000000GG000000",
                       "QffffffLLffffff",
                       AP_HAL::micros64(),
                       pkt1.ypr[2], pkt1.ypr[1], pkt1.ypr[0],
                       pkt1.velNED[0], pkt1.velNED[1], pkt1.velNED[2],
                       int32_t(pkt1.positionLLA[0]*1.0e7), int32_t(pkt1.positionLLA[1]*1.0e7),
                       float(pkt1.positionLLA[2]),
                       pkt1.posU, pkt1.velU,
                       pkt1.yprU[2], pkt1.yprU[1], pkt1.yprU[0]);
}

/*
  process packet type 2
 */
void AP_ExternalAHRS::process_packet2(const uint8_t *b)
{
    const struct VN_packet2 &pkt2 = *(struct VN_packet2 *)b;
    const struct VN_packet1 &pkt1 = *last_pkt1;

    last_pkt2_ms = AP_HAL::millis();
    *last_pkt2 = pkt2;

    gps_data_message_t gps;

    // get ToW in milliseconds
    gps.gps_week = pkt2.timeGPS / (AP_MSEC_PER_WEEK * 1000000ULL);
    gps.ms_tow = (pkt2.timeGPS / 1000000ULL) % (60*60*24*7*1000ULL);
    gps.fix_type = pkt2.GPS1Fix;
    gps.satellites_in_view = pkt2.numGPS1Sats;

    gps.horizontal_pos_accuracy = pkt1.posU;
    gps.vertical_pos_accuracy = pkt1.posU;
    gps.horizontal_vel_accuracy = pkt1.velU;

    gps.hdop = pkt2.GPS1DOP[4];
    gps.vdop = pkt2.GPS1DOP[3];

    gps.latitude = pkt2.GPS1posLLA[0] * 1.0e7;
    gps.longitude = pkt2.GPS1posLLA[1] * 1.0e7;
    gps.msl_altitude = pkt2.GPS1posLLA[2] * 1.0e2;

    gps.ned_vel_north = pkt2.GPS1velNED[0];
    gps.ned_vel_east = pkt2.GPS1velNED[1];
    gps.ned_vel_down = pkt2.GPS1velNED[2];

    if (gps.fix_type >= 3 && !origin_set) {
        origin.lat = gps.latitude;
        origin.lng = gps.longitude;
        origin.alt = gps.msl_altitude;
        origin_set = true;
    }

    AP::gps().handle_external(gps);
}

// get serial port number for the uart
int8_t AP_ExternalAHRS::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_pkt1_ms < 40 && now - last_pkt2_ms < 500);
}

bool AP_ExternalAHRS::initialised(void)
{
    return last_pkt1_ms != 0 && last_pkt2_ms != 0;
}

bool AP_ExternalAHRS::get_quaternion(Quaternion &quat)
{
    if (!last_pkt1) {
        return false;
    }
    WITH_SEMAPHORE(sem);
    const struct VN_packet1 &pkt1 = *last_pkt1;
    quat(pkt1.quaternion[3], pkt1.quaternion[0], pkt1.quaternion[1], pkt1.quaternion[2]);
    return true;
}

bool AP_ExternalAHRS::get_origin(Location &loc)
{
    WITH_SEMAPHORE(sem);
    loc = origin;
    return origin_set;
}

bool AP_ExternalAHRS::get_location(Location &loc)
{
    if (!last_pkt2) {
        return false;
    }
    const struct VN_packet2 &pkt2 = *last_pkt2;
    WITH_SEMAPHORE(sem);
    loc = Location{int32_t(pkt2.GPS1posLLA[0] * 1.0e7),
                   int32_t(pkt2.GPS1posLLA[1] * 1.0e7),
                   int32_t(pkt2.GPS1posLLA[2] * 1.0e2),
                   Location::AltFrame::ABSOLUTE};
    return true;
}

Vector2f AP_ExternalAHRS::get_groundspeed_vector()
{
    if (!last_pkt1) {
        return Vector2f{};
    }
    const struct VN_packet1 &pkt1 = *last_pkt1;
    WITH_SEMAPHORE(sem);
    return Vector2f{pkt1.velNED[0], pkt1.velNED[1]};
}

bool AP_ExternalAHRS::get_velocity_NED(Vector3f &vel)
{
    if (!last_pkt1) {
        return false;
    }
    const struct VN_packet1 &pkt1 = *last_pkt1;
    WITH_SEMAPHORE(sem);
    vel.x = pkt1.velNED[0];
    vel.y = pkt1.velNED[1];
    vel.z = pkt1.velNED[2];
    return true;
}

bool AP_ExternalAHRS::get_speed_down(float &speedD)
{
    if (!last_pkt1) {
        return false;
    }
    const struct VN_packet1 &pkt1 = *last_pkt1;
    WITH_SEMAPHORE(sem);
    speedD = pkt1.velNED[2];
    return true;
}

bool AP_ExternalAHRS::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "VectorNav unhealthy");
        return false;
    }
    if (last_pkt2->GPS1Fix < 3) {
        hal.util->snprintf(failure_msg, failure_msg_len, "VectorNav no GPS1 lock");
        return false;
    }
    if (last_pkt2->GPS2Fix < 3) {
        hal.util->snprintf(failure_msg, failure_msg_len, "VectorNav no GPS2 lock");
        return false;
    }
    return true;
}

/*
  get filter status. We don't know the meaning of the status bits yet,
  so assume all OK if we have GPS lock
 */
void AP_ExternalAHRS::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (last_pkt1 && last_pkt2) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_pkt2) {
        status.flags.attitude = 1;
        status.flags.vert_vel = 1;
        status.flags.vert_pos = 1;

        const struct VN_packet2 &pkt2 = *last_pkt2;
        if (pkt2.GPS1Fix >= 3) {
            status.flags.horiz_vel = 1;
            status.flags.horiz_pos_rel = 1;
            status.flags.horiz_pos_abs = 1;
            status.flags.pred_horiz_pos_rel = 1;
            status.flags.pred_horiz_pos_abs = 1;
            status.flags.using_gps = 1;
        }
    }
}

Vector3f AP_ExternalAHRS::get_gyro(void)
{
    if (!last_pkt1) {
        return Vector3f{};
    }
    const struct VN_packet1 &pkt1 = *last_pkt1;
    WITH_SEMAPHORE(sem);
    return Vector3f(pkt1.gyro[0], pkt1.gyro[1], pkt1.gyro[2]);
}

Vector3f AP_ExternalAHRS::get_accel(void)
{
    if (!last_pkt1) {
        return Vector3f{};
    }
    const struct VN_packet1 &pkt1 = *last_pkt1;
    WITH_SEMAPHORE(sem);
    return Vector3f{pkt1.accel[0], pkt1.accel[1], pkt1.accel[2]};
}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS::send_status_report(mavlink_channel_t chan) const
{
    if (!last_pkt1) {
        return;
    }
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
    const struct VN_packet1 &pkt1 = *(struct VN_packet1 *)last_pkt1;
    const float vel_gate = 5;
    const float pos_gate = 5;
    const float hgt_gate = 5;
    const float mag_var = 0;
    mavlink_msg_ekf_status_report_send(chan, flags,
                                       pkt1.velU/vel_gate, pkt1.posU/pos_gate, pkt1.posU/hgt_gate,
                                       mag_var, 0, 0);
}

namespace AP {

AP_ExternalAHRS &externalAHRS()
{
    return *AP_ExternalAHRS::get_singleton();
}

};

#endif  // HAL_EXTERNAL_AHRS_ENABLED

