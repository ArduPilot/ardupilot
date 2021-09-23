//
// Created by asa on 9/23/21.
//

//usage:
//PARAMS:
//  AHRS_EKF_TYPE = 11
//  EAHRS_TYPE = 2
//  SERIAL4_PROTOCOL = 36
//  SERIAL4_BAUD = 115
//  sim_vehicle.py -v ArduPlane -D --console --map -A --uartE=sim:LORD

//debugging:
//echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope
//sim_vehicle.py -v ArduPlane -D --console --map -A --uartE=sim:LORD
//then just attach to process and set breakpoints in AP_InertialSensor_SITL to look at raw stat variables

#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL
{

class LORD : public SerialDevice
{
public:

    LORD();

    // update state
    void update(void);

private:
    struct LORD_Packet {
        uint8_t header[4];
        uint8_t payload[256];
        uint8_t checksum[2];

        size_t payload_size = 0;
    };

    uint32_t last_imu_pkt_us;
    uint32_t last_gnss_pkt_us;
    uint32_t last_filter_pkt_us;

    void generate_checksum(LORD_Packet&);

    void send_packet(LORD_Packet);
    void send_imu_packet();
    void send_gnss_packet();
    void send_filter_packet();

    void put_float(LORD_Packet&, float);
    void put_double(LORD_Packet&, double);
    void put_int(LORD_Packet&, uint16_t);

    uint64_t start_us;
};

}