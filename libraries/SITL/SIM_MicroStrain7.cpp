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
    Simulate MicroStrain 7-series GNSS-INS devices

    Usage:
    PARAMS:
        param set AHRS_EKF_TYPE 11
        param set EAHRS_TYPE 7
        param set SERIAL3_PROTOCOL 36
        param set SERIAL3_BAUD 115
    sim_vehicle.py -v Plane -A "--serial3=sim:MicroStrain7" --console --map -DG
*/

#include "SIM_MicroStrain.h"

using namespace SITL;

void MicroStrain7::send_gnss_packet(void)
{
    const auto &fdm = _sitl->state;

    constexpr uint8_t descriptors[2] = {0x91, 0x92};
    for (uint8_t i = 0; i < ARRAY_SIZE(descriptors); i++) {
        MicroStrain_Packet packet;

        struct timeval tv;
        simulation_timeval(&tv);

        packet.header[0] = 0x75; // Sync One
        packet.header[1] = 0x65; // Sync Two
        packet.header[2] = descriptors[i]; // GNSS Descriptor

        // Add GPS Timestamp
        // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/shared_data/data/mip_field_shared_gps_timestamp.htm
        packet.payload[packet.payload_size++] = 0x0E; // GPS Time Field Size
        packet.payload[packet.payload_size++] = 0xD3; // Descriptor
        put_double(packet, (double) tv.tv_sec);
        put_int(packet, tv.tv_usec / (AP_MSEC_PER_WEEK * 1000000ULL));
        put_int(packet, 0);

        // Add GNSS Fix Information
        // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_recv_1/data/mip_field_gnss_fix_info.htm
        packet.payload[packet.payload_size++] =  0x08; // GNSS Fix Field Size
        packet.payload[packet.payload_size++] = 0x0B; // Descriptor
        packet.payload[packet.payload_size++] = 0x00; // Fix type FIX_3D
        packet.payload[packet.payload_size++] = 19; // Sat count
        put_int(packet, 0); // Fix flags
        put_int(packet, 0); // Valid flags

        // Add GNSS LLH position
        // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_recv_1/data/mip_field_gnss_llh_pos.htm
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
        // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_recv_1/data/mip_field_gnss_dop.htm
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

}

void MicroStrain7::send_filter_packet(void)
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
    // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_status.htm
    packet.payload[packet.payload_size++] = 0x08; // Filter State Field Size
    packet.payload[packet.payload_size++] = 0x10; // Descriptor
    put_int(packet, 0x04); // Filter state (GQ7_FULL_NAV)
    put_int(packet, 0x03); // Dynamics mode (Airborne)
    put_int(packet, 0); // Filter flags (None, no warnings)

    // Add Attitude Quaternion
    // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_attitude_quaternion.htm
    packet.payload[packet.payload_size++] = 0x14; // Attitude Quaternion Field Size
    packet.payload[packet.payload_size++] = 0x03; // Descriptor
    put_float(packet, fdm.quaternion.q1);
    put_float(packet, fdm.quaternion.q2);
    put_float(packet, fdm.quaternion.q3);
    put_float(packet, fdm.quaternion.q4);
    put_int(packet, 0x0001); // Valid flags

    packet.header[3] = packet.payload_size;

    send_packet(packet);
}
