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
    simulate LORD Microstrain serial device
*/
#include "SIM_LORD.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

using namespace SITL;

LORD::LORD() :SerialDevice::SerialDevice()
{
}


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

void LORD::generate_checksum(LORD_Packet& packet)
{
    uint8_t checksumByte1 = 0;
    uint8_t checksumByte2 = 0;

    for (int i = 0; i < 4; i++) {
        checksumByte1 += packet.header[i];
        checksumByte2 += checksumByte1;
    }

    for (int i = 0; i < packet.header[3]; i++) {
        checksumByte1 += packet.payload[i];
        checksumByte2 += checksumByte1;
    }

    packet.checksum[0] = checksumByte1;
    packet.checksum[1] = checksumByte2;
}

void LORD::send_packet(LORD_Packet packet)
{
    generate_checksum(packet);

    write_to_autopilot((char *)&packet.header, sizeof(packet.header));
    write_to_autopilot((char *)&packet.payload, packet.payload_size);
    write_to_autopilot((char *)&packet.checksum, sizeof(packet.checksum));
}


void LORD::send_imu_packet(void)
{
    const auto &fdm = _sitl->state;
    LORD_Packet packet;

    struct timeval tv;
    simulation_timeval(&tv);

    if (start_us == 0) {
        start_us = tv.tv_usec * 1000;
    }

    packet.header[0] = 0x75;
    packet.header[1] = 0x65;
    packet.header[2] = 0x80;

    // Add ambient pressure field
    packet.payload[packet.payload_size++] = 0x06;
    packet.payload[packet.payload_size++] = 0x17;
    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(fdm.altitude * 0.001f, sigma, delta, theta);
    put_float(packet, SSL_AIR_PRESSURE * delta * 0.001 + rand_float() * 0.1);

    // Add scaled magnetometer field
    packet.payload[packet.payload_size++] = 0x0E;
    packet.payload[packet.payload_size++] = 0x06;
    put_float(packet, fdm.bodyMagField.x*0.001);
    put_float(packet, fdm.bodyMagField.y*0.001);
    put_float(packet, fdm.bodyMagField.z*0.001);

    // Add scaled acceletometer field
    packet.payload[packet.payload_size++] = 0x0E;
    packet.payload[packet.payload_size++] = 0x04;
    put_float(packet, fdm.xAccel / GRAVITY_MSS);
    put_float(packet, fdm.yAccel / GRAVITY_MSS);
    put_float(packet, fdm.zAccel / GRAVITY_MSS);

    // Add scaled gyro field
    const float gyro_noise = 0.05;
    packet.payload[packet.payload_size++] = 0x0E;
    packet.payload[packet.payload_size++] = 0x05;
    put_float(packet, radians(fdm.rollRate + rand_float() * gyro_noise));
    put_float(packet, radians(fdm.pitchRate + rand_float() * gyro_noise));
    put_float(packet, radians(fdm.yawRate + rand_float() * gyro_noise));

    // Add CF Quaternion field
    packet.payload[packet.payload_size++] = 0x12;
    packet.payload[packet.payload_size++] = 0x0A;
    put_float(packet, fdm.quaternion.q1);
    put_float(packet, fdm.quaternion.q2);
    put_float(packet, fdm.quaternion.q3);
    put_float(packet, fdm.quaternion.q4);

    packet.header[3] = packet.payload_size;

    send_packet(packet);
}


void LORD::send_gnss_packet(void)
{
    const auto &fdm = _sitl->state;
    LORD_Packet packet;

    struct timeval tv;
    simulation_timeval(&tv);

    packet.header[0] = 0x75;
    packet.header[1] = 0x65;
    packet.header[2] = 0x81;

    // Add ambient GPS Time
    packet.payload[packet.payload_size++] = 0x0E;
    packet.payload[packet.payload_size++] = 0x09;
    put_double(packet, (double) tv.tv_sec);
    put_int(packet, tv.tv_usec / (AP_MSEC_PER_WEEK * 1000000ULL));
    put_int(packet, 0);

    // Add GNSS Fix Information
    packet.payload[packet.payload_size++] =  0x08;
    packet.payload[packet.payload_size++] = 0x0B;
    packet.payload[packet.payload_size++] = 0x00; // Fix type
    packet.payload[packet.payload_size++] = 19; // Sat count
    put_int(packet, 0); // Fix flags
    put_int(packet, 0); // Valid flags

    // Add GNSS LLH position
    packet.payload[packet.payload_size++] = 0x2C;
    packet.payload[packet.payload_size++] = 0x03;
    put_double(packet, fdm.latitude);
    put_double(packet, fdm.longitude);
    put_double(packet, 0);   // Height above ellipsoid - unused
    put_double(packet, fdm.altitude);
    put_float(packet, 0.5f); // Horizontal accuracy
    put_float(packet, 0.5f); // Vertical accuracy
    put_int(packet, 31);       // Valid flags

    // Add DOP Data
    packet.payload[packet.payload_size++] = 0x20;
    packet.payload[packet.payload_size++] = 0x07;
    put_float(packet, 0); // GDOP
    put_float(packet, 0); // PDOP
    put_float(packet, 0); // HDOP
    put_float(packet, 0); // VDOP
    put_float(packet, 0); // TDOP
    put_float(packet, 0); // NDOP
    put_float(packet, 0); // EDOP
    put_int(packet, 127);

    // Add GNSS NED velocity
    packet.payload[packet.payload_size++] = 0x24;
    packet.payload[packet.payload_size++] = 0x05;
    put_float(packet, fdm.speedN);
    put_float(packet, fdm.speedE);
    put_float(packet, fdm.speedD);
    put_float(packet, 0);       //speed - unused
    put_float(packet, 0);       //ground speed - unused
    put_float(packet, 0);       //heading - unused
    put_float(packet, 0.25f);   //speed accuracy
    put_float(packet, 0);       //heading accuracy - unused
    put_int(packet, 31);        //valid flags

    packet.header[3] = packet.payload_size;


    send_packet(packet);
}

void LORD::send_filter_packet(void)
{
    const auto &fdm = _sitl->state;
    LORD_Packet packet;

    struct timeval tv;
    simulation_timeval(&tv);

    packet.header[0] = 0x75;
    packet.header[1] = 0x65;
    packet.header[2] = 0x82;

    // Add ambient Filter Time
    packet.payload[packet.payload_size++] = 0x0E;
    packet.payload[packet.payload_size++] = 0x11;
    put_double(packet, (double) tv.tv_usec / 1e6);
    put_int(packet, tv.tv_usec / (AP_MSEC_PER_WEEK * 1000000ULL));
    put_int(packet, 0x0001);

    // Add GNSS Filter velocity
    packet.payload[packet.payload_size++] = 0x10;
    packet.payload[packet.payload_size++] = 0x02;
    put_float(packet, fdm.speedN);
    put_float(packet, fdm.speedE);
    put_float(packet, fdm.speedD);
    put_int(packet, 0x0001);


    // Add Filter LLH position
    packet.payload[packet.payload_size++] = 0x1C;
    packet.payload[packet.payload_size++] = 0x01;
    put_double(packet, fdm.latitude);
    put_double(packet, fdm.longitude);
    put_double(packet, 0);   // Height above ellipsoid - unused
    put_int(packet, 0x0001);       // Valid flags

    packet.payload[packet.payload_size++] = 0x08;
    packet.payload[packet.payload_size++] = 0x10;
    put_int(packet, 0x02); // Filter state
    put_int(packet, 0x03); // Dynamics mode
    put_int(packet, 0); // Filter flags

    packet.header[3] = packet.payload_size;


    send_packet(packet);
}

/*
  send LORD data
 */
void LORD::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }

    uint32_t us_between_imu_packets = 20000;
    uint32_t us_between_gnss_packets = 250000;
    uint32_t us_between_filter_packets = 100000;

    uint32_t now = AP_HAL::micros();
    if (now - last_imu_pkt_us >= us_between_imu_packets) {
        last_imu_pkt_us = now;
        send_imu_packet();
    }

    if (now - last_gnss_pkt_us >= us_between_gnss_packets) {
        last_gnss_pkt_us = now;
        send_gnss_packet();
    }

    if (now - last_filter_pkt_us >= us_between_filter_packets) {
        last_filter_pkt_us = now;
        send_filter_packet();
    }
}

void LORD::put_float(LORD_Packet &packet, float f)
{
    uint32_t fbits = 0;
    memcpy(&fbits, &f, sizeof(fbits));
    put_be32_ptr(&packet.payload[packet.payload_size], fbits);
    packet.payload_size += sizeof(float);
}

void LORD::put_double(LORD_Packet &packet, double d)
{
    uint64_t dbits = 0;
    memcpy(&dbits, &d, sizeof(dbits));
    put_be64_ptr(&packet.payload[packet.payload_size], dbits);
    packet.payload_size += sizeof(double);
}

void LORD::put_int(LORD_Packet &packet, uint16_t t)
{
    put_be16_ptr(&packet.payload[packet.payload_size], t);
    packet.payload_size += sizeof(uint16_t);
}