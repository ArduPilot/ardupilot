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
#include <unistd.h>
#include <fcntl.h>
#include <AP_Common/NMEA.h>

using namespace SITL;

VectorNav::VectorNav() :
    SerialDevice::SerialDevice()
{
}

struct PACKED VN_packet1 {
    float uncompMag[3];
    float uncompAccel[3];
    float uncompAngRate[3];
    float pressure;
    float mag[3];
    float accel[3];
    float gyro[3];
    float ypr[3];
    float quaternion[4];
    float yprU[3];
    double positionLLA[3];
    float velNED[3];
    float posU;
    float velU;
};

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
};

#define VN_PKT1_HEADER { 0xFA, 0x34, 0x2E, 0x07, 0x06, 0x01, 0x12, 0x06 }
#define VN_PKT2_HEADER { 0xFA, 0x4E, 0x02, 0x00, 0x10, 0x00, 0xB8, 0x20, 0x18, 0x00 }

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

void VectorNav::send_packet1(void)
{
    const auto &fdm = _sitl->state;

    struct VN_packet1 pkt {};

    pkt.uncompAccel[0] = fdm.xAccel;
    pkt.uncompAccel[1] = fdm.yAccel;
    pkt.uncompAccel[2] = fdm.zAccel;
    const float gyro_noise = 0.05;
    pkt.uncompAngRate[0] = radians(fdm.rollRate + gyro_noise * rand_float());
    pkt.uncompAngRate[1] = radians(fdm.pitchRate + gyro_noise * rand_float());
    pkt.uncompAngRate[2] = radians(fdm.yawRate + gyro_noise * rand_float());

    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(fdm.altitude * 0.001f, sigma, delta, theta);
    pkt.pressure = SSL_AIR_PRESSURE * delta * 0.001 + rand_float() * 0.01;

    pkt.mag[0] = fdm.bodyMagField.x*0.001;
    pkt.mag[1] = fdm.bodyMagField.y*0.001;
    pkt.mag[2] = fdm.bodyMagField.z*0.001;
    pkt.uncompMag[0] = pkt.mag[0];
    pkt.uncompMag[1] = pkt.mag[1];
    pkt.uncompMag[2] = pkt.mag[2];

    pkt.accel[0] = fdm.xAccel;
    pkt.accel[1] = fdm.yAccel;
    pkt.accel[2] = fdm.zAccel;
    pkt.gyro[0] = radians(fdm.rollRate + rand_float() * gyro_noise);
    pkt.gyro[1] = radians(fdm.pitchRate + rand_float() * gyro_noise);
    pkt.gyro[2] = radians(fdm.yawRate + rand_float() * gyro_noise);

    pkt.ypr[0] = fdm.yawDeg;
    pkt.ypr[1] = fdm.pitchDeg;
    pkt.ypr[2] = fdm.rollDeg;

    pkt.quaternion[0] = fdm.quaternion.q2;
    pkt.quaternion[1] = fdm.quaternion.q3;
    pkt.quaternion[2] = fdm.quaternion.q4;
    pkt.quaternion[3] = fdm.quaternion.q1;

    pkt.positionLLA[0] = fdm.latitude;
    pkt.positionLLA[1] = fdm.longitude;
    pkt.positionLLA[2] = fdm.altitude;
    pkt.velNED[0] = fdm.speedN;
    pkt.velNED[1] = fdm.speedE;
    pkt.velNED[2] = fdm.speedD;
    pkt.posU = 0.5;
    pkt.velU = 0.25;

    const uint8_t header[] VN_PKT1_HEADER;

    write_to_autopilot((char *)&header, sizeof(header));
    write_to_autopilot((char *)&pkt, sizeof(pkt));

    uint16_t crc = crc16_ccitt(&header[1], sizeof(header)-1, 0);
    crc = crc16_ccitt((const uint8_t *)&pkt, sizeof(pkt), crc);
    uint16_t crc2;
    swab(&crc, &crc2, 2);

    write_to_autopilot((char *)&crc2, sizeof(crc2));
}

void VectorNav::send_packet2(void)
{
    const auto &fdm = _sitl->state;

    struct VN_packet2 pkt {};

    struct timeval tv;
    simulation_timeval(&tv);

    pkt.timeGPS = tv.tv_usec * 1000ULL;
    pkt.temp = 23.5;
    pkt.numGPS1Sats = 19;
    pkt.GPS1Fix = 3;
    pkt.GPS1posLLA[0] = fdm.latitude;
    pkt.GPS1posLLA[1] = fdm.longitude;
    pkt.GPS1posLLA[2] = fdm.altitude;
    pkt.GPS1velNED[0] = fdm.speedN;
    pkt.GPS1velNED[1] = fdm.speedE;
    pkt.GPS1velNED[2] = fdm.speedD;
    // pkt.GPS1DOP =
    pkt.numGPS2Sats = 18;
    pkt.GPS2Fix = 3;

    const uint8_t header[] VN_PKT2_HEADER;

    write_to_autopilot((char *)&header, sizeof(header));
    write_to_autopilot((char *)&pkt, sizeof(pkt));

    uint16_t crc = crc16_ccitt(&header[1], sizeof(header)-1, 0);
    crc = crc16_ccitt((const uint8_t *)&pkt, sizeof(pkt), crc);

    uint16_t crc2;
    swab(&crc, &crc2, 2);

    write_to_autopilot((char *)&crc2, sizeof(crc2));
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
    if (now - last_pkt1_us >= 20000) {
        last_pkt1_us = now;
        send_packet1();
    }

    if (now - last_pkt2_us >= 200000) {
        last_pkt2_us = now;
        send_packet2();
    }

    // Strictly we should send this in responce to the request
    // but sending it occasionally acheaves the same thing
    if (now - last_type_us >= 1000000) {
        last_type_us = now;
        nmea_printf("$VNRRG,01,VN-300-SITL");
    }

}

