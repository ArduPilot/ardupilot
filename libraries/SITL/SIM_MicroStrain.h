// Created by Asa Davis and Davis Schenkenberger on 23rd September 21.

//usage:
//PARAMS:
// param set AHRS_EKF_TYPE 11
// param set EAHRS_TYPE 2
// param set SERIAL3_PROTOCOL 36
// param set SERIAL3_BAUD 115
// sim_vehicle.py -v Plane -A "--serial3=sim:MicroStrain7" --console --map -DG
#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL
{

class MicroStrain : public SerialDevice
{
    // This class implements the common MicroStrain driver support.
public:

    MicroStrain();

    // update state
    void update(void);

protected:
    struct MicroStrain_Packet {
        uint8_t header[4];
        uint8_t payload[256];
        uint8_t checksum[2];

        size_t payload_size = 0;
    };

    uint32_t last_imu_pkt_ms;
    uint32_t last_gnss_pkt_ms;
    uint32_t last_filter_pkt_ms;

    void generate_checksum(MicroStrain_Packet&);

    void send_packet(MicroStrain_Packet);
    void send_imu_packet();
    virtual void send_gnss_packet() = 0;
    virtual void send_filter_packet() = 0;

    void put_float(MicroStrain_Packet&, float);
    void put_double(MicroStrain_Packet&, double);
    void put_int(MicroStrain_Packet&, uint16_t);

    // get timeval using simulation time
    static void simulation_timeval(struct timeval *tv);

    uint64_t start_us;
};

class MicroStrain5 : public MicroStrain
{
    // This is a specialization for the 3DM-GX5-GNSS/INS
private:
    void send_gnss_packet() override;
    void send_filter_packet() override;

};

class MicroStrain7 : public MicroStrain
{
    // This is a specialization for the 3DM-GQ7-GNSS/INS
private:
    void send_gnss_packet() override;
    void send_filter_packet() override;

};

}

