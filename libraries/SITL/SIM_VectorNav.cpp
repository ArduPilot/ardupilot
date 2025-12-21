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
  simulate VectorNav serial AHRS
*/

#include "SIM_VectorNav.h"
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <AP_Common/NMEA.h>

using namespace SITL;

VectorNav::VectorNav(VNModel VN_Name) :
    SerialDevice::SerialDevice()
{
    model = VN_Name;
}

struct PACKED VN_IMU_packet_sim {
    static constexpr uint8_t header[]{0x01, 0x21, 0x07};
    uint64_t timeStartup;
    float gyro[3];
    float accel[3];
    float uncompAccel[3];
    float uncompAngRate[3];
    float mag[3];
    float temp;
    float pressure;
};
constexpr uint8_t VN_IMU_packet_sim::header[];

struct PACKED VN_INS_ekf_packet_sim {
    static constexpr uint8_t header[]{0x31, 0x01, 0x00, 0x06, 0x01, 0x13, 0x06};
    uint64_t timeStartup;
    float ypr[3];
    float quaternion[4];
    float yprU[3];
    uint16_t insStatus;
    double posLla[3];
    float velNed[3];
    float posU;
    float velU;
};
constexpr uint8_t VN_INS_ekf_packet_sim::header[];

struct PACKED VN_INS_gnss_packet_sim {
    static constexpr uint8_t header[]{0x49, 0x03, 0x00, 0xB8, 0x26, 0x18, 0x00};
    uint64_t timeStartup;
    uint64_t timeGps;
    uint8_t numSats1;
    uint8_t fix1;
    double posLla1[3];
    float velNed1[3];
    float posU1[3];
    float velU1;
    float dop1[7];
    uint8_t numSats2;
    uint8_t fix2;
};
constexpr uint8_t VN_INS_gnss_packet_sim::header[];

struct PACKED VN_AHRS_ekf_packet_sim {
    static constexpr uint8_t header[]{0x11, 0x01, 0x00, 0x06, 0x01};
    uint64_t timeStartup;
    float ypr[3];
    float quaternion[4];
    float yprU[3];
};
constexpr uint8_t VN_AHRS_ekf_packet_sim::header[];

/*
  get timeval using simulation time
 */
static void simulation_timeval(struct timeval *tv)
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

void VectorNav::send_imu_packet(void)
{
    const auto &fdm = _sitl->state;

    struct VN_IMU_packet_sim pkt {};

    pkt.timeStartup = AP_HAL::micros() * 1e3;
    
    
    const float gyro_noise = 0.05;

    pkt.gyro[0] = radians(fdm.rollRate + rand_float() * gyro_noise);
    pkt.gyro[1] = radians(fdm.pitchRate + rand_float() * gyro_noise);
    pkt.gyro[2] = radians(fdm.yawRate + rand_float() * gyro_noise);
    
    pkt.accel[0] = fdm.xAccel;
    pkt.accel[1] = fdm.yAccel;
    pkt.accel[2] = fdm.zAccel;

    pkt.uncompAccel[0] = fdm.xAccel;
    pkt.uncompAccel[1] = fdm.yAccel;
    pkt.uncompAccel[2] = fdm.zAccel;
    
    pkt.uncompAngRate[0] = radians(fdm.rollRate + gyro_noise * rand_float());
    pkt.uncompAngRate[1] = radians(fdm.pitchRate + gyro_noise * rand_float());
    pkt.uncompAngRate[2] = radians(fdm.yawRate + gyro_noise * rand_float());

    pkt.mag[0] = fdm.bodyMagField.x*0.001;
    pkt.mag[1] = fdm.bodyMagField.y*0.001;
    pkt.mag[2] = fdm.bodyMagField.z*0.001;

    pkt.temp = AP_Baro::get_temperatureC_for_alt_amsl(fdm.altitude);

    const float pressure_Pa = AP_Baro::get_pressure_for_alt_amsl(fdm.altitude);
    pkt.pressure = pressure_Pa*0.001 + rand_float() * 0.01;

    const uint8_t sync_byte = 0xFA;
    write_to_autopilot((const char *)&sync_byte, 1);
    write_to_autopilot((const char *)&VN_IMU_packet_sim::header, sizeof(VN_IMU_packet_sim::header));
    write_to_autopilot((const char *)&pkt, sizeof(pkt));

    uint16_t crc = crc16_ccitt(&VN_IMU_packet_sim::header[0], sizeof(VN_IMU_packet_sim::header), 0);
    crc = crc16_ccitt((const uint8_t *)&pkt, sizeof(pkt), crc);
    uint16_t crc2;
    swab(&crc, &crc2, 2);

    write_to_autopilot((const char *)&crc2, sizeof(crc2));
}

void VectorNav::send_ins_ekf_packet(void)
{
    const auto &fdm = _sitl->state;

    struct VN_INS_ekf_packet_sim pkt {};

    pkt.timeStartup = AP_HAL::micros() * 1e3;

    pkt.ypr[0] = fdm.yawDeg;
    pkt.ypr[1] = fdm.pitchDeg;
    pkt.ypr[2] = fdm.rollDeg;

    pkt.quaternion[0] = fdm.quaternion.q2;
    pkt.quaternion[1] = fdm.quaternion.q3;
    pkt.quaternion[2] = fdm.quaternion.q4;
    pkt.quaternion[3] = fdm.quaternion.q1;

    pkt.yprU[0] = 0.03;
    pkt.yprU[1] = 0.03;
    pkt.yprU[2] = 0.15;

    pkt.insStatus = 0x0306;

    pkt.posLla[0] = fdm.latitude;
    pkt.posLla[1] = fdm.longitude;
    pkt.posLla[2] = fdm.altitude;
    pkt.velNed[0] = fdm.speedN;
    pkt.velNed[1] = fdm.speedE;
    pkt.velNed[2] = fdm.speedD;
    pkt.posU = 0.5;
    pkt.velU = 0.25;

    const uint8_t sync_byte = 0xFA;
    write_to_autopilot((const char *)&sync_byte, 1);
    write_to_autopilot((const char *)&VN_INS_ekf_packet_sim::header, sizeof(VN_INS_ekf_packet_sim::header));
    write_to_autopilot((const char *)&pkt, sizeof(pkt));

    uint16_t crc = crc16_ccitt(&VN_INS_ekf_packet_sim::header[0], sizeof(VN_INS_ekf_packet_sim::header), 0);
    crc = crc16_ccitt((const uint8_t *)&pkt, sizeof(pkt), crc);

    uint16_t crc2;
    swab(&crc, &crc2, 2);

    write_to_autopilot((const char *)&crc2, sizeof(crc2));
}

void VectorNav::send_ins_gnss_packet(void)
{
    const auto &fdm = _sitl->state;

    struct VN_INS_gnss_packet_sim pkt {};

    pkt.timeStartup = AP_HAL::micros() * 1e3;

    struct timeval tv;
    simulation_timeval(&tv);

    pkt.timeGps = tv.tv_usec * 1000ULL;

    pkt.numSats1 = 19;
    pkt.fix1 = 3;
    pkt.posLla1[0] = fdm.latitude;
    pkt.posLla1[1] = fdm.longitude;
    pkt.posLla1[2] = fdm.altitude;
    pkt.velNed1[0] = fdm.speedN;
    pkt.velNed1[1] = fdm.speedE;
    pkt.velNed1[2] = fdm.speedD;

    pkt.posU1[0] = 1;
    pkt.posU1[1] = 1;
    pkt.posU1[2] = 1.5;

    pkt.velNed1[0] = 0.05;
    pkt.velNed1[1] = 0.05;
    pkt.velNed1[2] = 0.05;
    // pkt.dop1 =
    pkt.numSats2 = 18;
    pkt.fix2 = 3;

    const uint8_t sync_byte = 0xFA;
    write_to_autopilot((const char *)&sync_byte, 1);
    write_to_autopilot((const char *)&VN_INS_gnss_packet_sim::header, sizeof(VN_INS_gnss_packet_sim::header));
    write_to_autopilot((const char *)&pkt, sizeof(pkt));

    uint16_t crc = crc16_ccitt(&VN_INS_gnss_packet_sim::header[0], sizeof(VN_INS_gnss_packet_sim::header), 0);
    crc = crc16_ccitt((const uint8_t *)&pkt, sizeof(pkt), crc);

    uint16_t crc2;
    swab(&crc, &crc2, 2);

    write_to_autopilot((const char *)&crc2, sizeof(crc2));
}

void VectorNav::send_ahrs_packet(void)
{
    const auto &fdm = _sitl->state;

    struct VN_AHRS_ekf_packet_sim pkt{};
    pkt.timeStartup = AP_HAL::micros() * 1e3;

    pkt.ypr[0] = fdm.yawDeg;
    pkt.ypr[1] = fdm.pitchDeg;
    pkt.ypr[2] = fdm.rollDeg;

    pkt.quaternion[0] = fdm.quaternion.q2;
    pkt.quaternion[1] = fdm.quaternion.q3;
    pkt.quaternion[2] = fdm.quaternion.q4;
    pkt.quaternion[3] = fdm.quaternion.q1;

    pkt.yprU[0] = 0.03;
    pkt.yprU[1] = 0.03;
    pkt.yprU[2] = 0.15;

    const uint8_t sync = 0xFA;
    write_to_autopilot((const char*)&sync, 1);
    write_to_autopilot((const char*)VN_AHRS_ekf_packet_sim::header,
                       sizeof(VN_AHRS_ekf_packet_sim::header));
    write_to_autopilot((const char*)&pkt, sizeof(pkt));

    uint16_t crc = crc16_ccitt(VN_AHRS_ekf_packet_sim::header,
                               sizeof(VN_AHRS_ekf_packet_sim::header), 0);
    crc = crc16_ccitt((const uint8_t*)&pkt, sizeof(pkt), crc);

    uint16_t crc_swapped;
    swab(&crc, &crc_swapped, 2);

    write_to_autopilot((const char*)&crc_swapped, sizeof(crc_swapped));
}

void VectorNav::nmea_printf(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    char *s = nmea_vaprintf(fmt, ap);
    va_end(ap);
    if (s != nullptr) {
        write_to_autopilot((const char*)s, strlen(s));
        free(s);
    }
}

/*
  send VectorNav data
 */
void VectorNav::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }

    uint32_t now = AP_HAL::micros();
    if (now - last_imu_pkt_us >= 20000) {
        last_imu_pkt_us = now;
        send_imu_packet();
    }

    if (model == VNModel::VN300) {
        if (now - last_ekf_pkt_us >= 20000) {
            last_ekf_pkt_us = now;
            send_ins_ekf_packet();
        }
        if (now - last_gnss_pkt_us >= 200000) {
            last_gnss_pkt_us = now;
            send_ins_gnss_packet();
        }
    } else {
        if (now - last_ahrs_pkt_us >= 20000) {
            last_ahrs_pkt_us = now;
            send_ahrs_packet();
        }
    }

    char receive_buf[50];
    ssize_t n = read_from_autopilot(&receive_buf[0], ARRAY_SIZE(receive_buf));
    if (n <= 0) {
        return;
    }

    // avoid parsing the NMEA stream here by making assumptions about
    // how we receive configuration strings.  Generally we can just
    // echo back the configuration string to make the driver happy.
    if (n >= 9) {
        // intercept device-version query, respond with simulated version:
        const char *ver_query_string = "$VNRRG,01";
        if (strncmp(receive_buf, ver_query_string, strlen(ver_query_string)) == 0) {
            if (model == VNModel::VN100) {
                nmea_printf("$VNRRG,01,VN-100-SITL");
            } else {
                nmea_printf("$VNRRG,01,VN-300-SITL");
            }
            // consume the query so we don't "respond" twice:
            memmove(&receive_buf[0], &receive_buf[strlen(ver_query_string)], n - strlen(ver_query_string));
            n -= strlen(ver_query_string);
        }
    }
    write_to_autopilot(receive_buf, n);
}

