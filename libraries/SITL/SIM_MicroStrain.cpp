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
    Simulate MicroStrain CX5 GNSS-INS devices
    
    Usage:
    PARAMS:
        param set AHRS_EKF_TYPE 11
        param set EAHRS_TYPE 2
        param set SERIAL3_PROTOCOL 36
        param set SERIAL3_BAUD 115
    sim_vehicle.py -v Plane -A "--serial3=sim:MicroStrain5" --console --map -DG
*/
#include "SIM_MicroStrain.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

using namespace SITL;

MicroStrain::MicroStrain() :SerialDevice::SerialDevice()
{
}


void MicroStrain::simulation_timeval(struct timeval *tv)
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

void MicroStrain::generate_checksum(MicroStrain_Packet& packet)
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

void MicroStrain::send_packet(MicroStrain_Packet packet)
{
    generate_checksum(packet);

    write_to_autopilot((char *)&packet.header, sizeof(packet.header));
    write_to_autopilot((char *)&packet.payload, packet.payload_size);
    write_to_autopilot((char *)&packet.checksum, sizeof(packet.checksum));
}


void MicroStrain::send_imu_packet(void)
{
    const auto &fdm = _sitl->state;
    MicroStrain_Packet packet;

    struct timeval tv;
    simulation_timeval(&tv);

    if (start_us == 0) {
        start_us = tv.tv_usec * 1000;
    }

    packet.header[0] = 0x75; // Sync One
    packet.header[1] = 0x65; // Sync Two
    packet.header[2] = 0x80; // INS Descriptor

    // Add ambient pressure field
    packet.payload[packet.payload_size++] = 0x06; // Ambient Pressure Field Size
    packet.payload[packet.payload_size++] = 0x17; // Descriptor
    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(fdm.altitude * 0.001f, sigma, delta, theta);
    put_float(packet, SSL_AIR_PRESSURE * delta * 0.001 + rand_float() * 0.1);

    // Add scaled magnetometer field
    packet.payload[packet.payload_size++] = 0x0E; // Scaled Magnetometer Field Size
    packet.payload[packet.payload_size++] = 0x06; // Descriptor
    put_float(packet, fdm.bodyMagField.x*0.001);
    put_float(packet, fdm.bodyMagField.y*0.001);
    put_float(packet, fdm.bodyMagField.z*0.001);

    // Add scaled accelerometer field
    packet.payload[packet.payload_size++] = 0x0E; // Scaled Accelerometer Field Size
    packet.payload[packet.payload_size++] = 0x04; // Descriptor
    put_float(packet, fdm.xAccel / GRAVITY_MSS);
    put_float(packet, fdm.yAccel / GRAVITY_MSS);
    put_float(packet, fdm.zAccel / GRAVITY_MSS);

    // Add scaled gyro field
    const float gyro_noise = 0.05;
    packet.payload[packet.payload_size++] = 0x0E; // Scaled Gyro Field Size
    packet.payload[packet.payload_size++] = 0x05; // Descriptor
    put_float(packet, radians(fdm.rollRate + rand_float() * gyro_noise));
    put_float(packet, radians(fdm.pitchRate + rand_float() * gyro_noise));
    put_float(packet, radians(fdm.yawRate + rand_float() * gyro_noise));

    // Add CF Quaternion field
    packet.payload[packet.payload_size++] = 0x12; // CF Quaternion Field Size
    packet.payload[packet.payload_size++] = 0x0A; // Descriptor
    put_float(packet, fdm.quaternion.q1);
    put_float(packet, fdm.quaternion.q2);
    put_float(packet, fdm.quaternion.q3);
    put_float(packet, fdm.quaternion.q4);

    packet.header[3] = packet.payload_size;

    send_packet(packet);
}

void MicroStrain5::send_gnss_packet(void)
{
    const auto &fdm = _sitl->state;
    MicroStrain_Packet packet;

    struct timeval tv;
    simulation_timeval(&tv);

    packet.header[0] = 0x75; // Sync One
    packet.header[1] = 0x65; // Sync Two
    packet.header[2] = 0x81; // GNSS Descriptor

    // Add GPS Timestamp
    packet.payload[packet.payload_size++] = 0x0E; // GPS Time Field Size
    packet.payload[packet.payload_size++] = 0xD3; // Descriptor
    put_double(packet, (double) tv.tv_sec);
    put_int(packet, tv.tv_usec / (AP_MSEC_PER_WEEK * 1000000ULL));
    put_int(packet, 0);

    // Add GNSS Fix Information
    packet.payload[packet.payload_size++] =  0x08; // GNSS Fix Field Size
    packet.payload[packet.payload_size++] = 0x0B; // Descriptor
    packet.payload[packet.payload_size++] = 0x00; // Fix type
    packet.payload[packet.payload_size++] = 19; // Sat count
    put_int(packet, 0); // Fix flags
    put_int(packet, 0); // Valid flags

    // Add GNSS LLH position
    packet.payload[packet.payload_size++] = 0x2C; // GNSS LLH Field Size
    packet.payload[packet.payload_size++] = 0x03; // Descriptor
    put_double(packet, fdm.latitude);
    put_double(packet, fdm.longitude);
    put_double(packet, 0);   // Height above ellipsoid - unused
    put_double(packet, fdm.altitude);
    put_float(packet, 0.5f); // Horizontal accuracy
    put_float(packet, 0.5f); // Vertical accuracy
    put_int(packet, 31); // Valid flags

    // Add DOP Data
    packet.payload[packet.payload_size++] = 0x20; // DOP Field Size
    packet.payload[packet.payload_size++] = 0x07; // Descriptor
    put_float(packet, 0); // GDOP
    put_float(packet, 0); // PDOP
    put_float(packet, 0); // HDOP
    put_float(packet, 0); // VDOP
    put_float(packet, 0); // TDOP
    put_float(packet, 0); // NDOP
    put_float(packet, 0); // EDOP
    put_int(packet, 127);

    // Add GNSS NED velocity
    packet.payload[packet.payload_size++] = 0x24; // GNSS NED Velocity Field Size
    packet.payload[packet.payload_size++] = 0x05; // Descriptor
    put_float(packet, fdm.speedN);
    put_float(packet, fdm.speedE);
    put_float(packet, fdm.speedD);
    put_float(packet, 0); //speed - unused
    put_float(packet, 0); //ground speed - unused
    put_float(packet, 0); //heading - unused
    put_float(packet, 0.25f); //speed accuracy
    put_float(packet, 0); //heading accuracy - unused
    put_int(packet, 31); //valid flags

    packet.header[3] = packet.payload_size;


    send_packet(packet);
}



void MicroStrain5::send_filter_packet(void)
{
    const auto &fdm = _sitl->state;
    MicroStrain_Packet packet;

    struct timeval tv;
    simulation_timeval(&tv);

    packet.header[0] = 0x75; // Sync One
    packet.header[1] = 0x65; // Sync Two
    packet.header[2] = 0x82; // Filter Descriptor

    // Add GPS Timestamp Shared Data
    packet.payload[packet.payload_size++] = 0x0E; // GPS Timestamp Field Size
    packet.payload[packet.payload_size++] = 0xD3; // Descriptor
    put_double(packet, (double) tv.tv_usec / 1e6);
    put_int(packet, tv.tv_usec / (AP_MSEC_PER_WEEK * 1000000ULL));
    put_int(packet, 0x0001);

    // Add GNSS Filter velocity
    packet.payload[packet.payload_size++] = 0x10; // GNSS Velocity Field Size
    packet.payload[packet.payload_size++] = 0x02; // Descriptor
    put_float(packet, fdm.speedN);
    put_float(packet, fdm.speedE);
    put_float(packet, fdm.speedD);
    put_int(packet, 0x0001);

    // Add Filter LLH position
    packet.payload[packet.payload_size++] = 0x1C; // Filter LLH Field Size
    packet.payload[packet.payload_size++] = 0x01; // Descriptor
    put_double(packet, fdm.latitude);
    put_double(packet, fdm.longitude);
    put_double(packet, 0); // Height above ellipsoid - unused
    put_int(packet, 0x0001); // Valid flags

    // Add Filter State
    packet.payload[packet.payload_size++] = 0x08; // Filter State Field Size
    packet.payload[packet.payload_size++] = 0x10; // Descriptor
    put_int(packet, 0x02); // Filter state (Running, Solution Valid)
    put_int(packet, 0x03); // Dynamics mode (Airborne)
    put_int(packet, 0); // Filter flags (None, no warnings)

    packet.header[3] = packet.payload_size;


    send_packet(packet);
}

/*
  send MicroStrain data
 */
void MicroStrain::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }

    uint32_t ms_between_imu_packets = 40;
    uint32_t ms_between_gnss_packets = 500;
    uint32_t ms_between_filter_packets = 40;

    uint32_t now = AP_HAL::millis();
    if (now - last_imu_pkt_ms >= ms_between_imu_packets) {
        last_imu_pkt_ms = now;
        send_imu_packet();
    }

    if (now - last_gnss_pkt_ms >= ms_between_gnss_packets) {
        last_gnss_pkt_ms = now;
        send_gnss_packet();
    }

    if (now - last_filter_pkt_ms >= ms_between_filter_packets) {
        last_filter_pkt_ms = now;
        send_filter_packet();
    }
}

void MicroStrain::put_float(MicroStrain_Packet &packet, float f)
{
    uint32_t fbits = 0;
    memcpy(&fbits, &f, sizeof(fbits));
    put_be32_ptr(&packet.payload[packet.payload_size], fbits);
    packet.payload_size += sizeof(float);
}

void MicroStrain::put_double(MicroStrain_Packet &packet, double d)
{
    uint64_t dbits = 0;
    memcpy(&dbits, &d, sizeof(dbits));
    put_be64_ptr(&packet.payload[packet.payload_size], dbits);
    packet.payload_size += sizeof(double);
}

void MicroStrain::put_int(MicroStrain_Packet &packet, uint16_t t)
{
    put_be16_ptr(&packet.payload[packet.payload_size], t);
    packet.payload_size += sizeof(uint16_t);
}

