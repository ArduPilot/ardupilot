// Created by Asa Davis and Davis Schenkenberger on 23rd September 21.

//usage:
//PARAMS:
// param set AHRS_EKF_TYPE 11
// param set EAHRS_TYPE 2
// param set SERIAL4_PROTOCOL 36
// param set SERIAL4_BAUD 115
// sim_vehicle.py -v ArduPlane -D --console --map -A --uartE=sim:MicroStrain
#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL
{

class MicroStrain5 : public SerialDevice
{
public:

    MicroStrain5();

    // update state
    void update(void);

private:
    struct MicroStrain_Packet {
        uint8_t header[4];
        uint8_t payload[256];
        uint8_t checksum[2];

        size_t payload_size = 0;
    };

    uint32_t last_imu_pkt_us;
    uint32_t last_gnss_pkt_us;
    uint32_t last_filter_pkt_us;

    void generate_checksum(MicroStrain_Packet&);

    void send_packet(MicroStrain_Packet);
    void send_imu_packet();
    void send_gnss_packet();
    void send_filter_packet();

    void put_float(MicroStrain_Packet&, float);
    void put_double(MicroStrain_Packet&, double);
    void put_int(MicroStrain_Packet&, uint16_t);

    uint64_t start_us;
};

}

