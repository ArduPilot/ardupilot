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
 Original C Code by Marvelmind (https://github.com/MarvelmindRobotics/marvelmind.c)
 Adapted into Ardupilot by Karthik Desai, Amilcar Lucas
 April 2017
 */

#pragma once

#include "AP_Beacon_Backend.h"

#define AP_BEACON_MARVELMIND_BUF_SIZE 255

class AP_Beacon_Marvelmind : public AP_Beacon_Backend
{
public:

    // constructor
    AP_Beacon_Marvelmind(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy();

    // update
    void update();

private:
    // Variables for Marvelmind
    struct PositionValue
    {
        uint8_t address;
        uint32_t timestamp;
        int32_t x__mm, y__mm, z__mm;
        bool high_resolution;
    };

    struct StationaryBeaconPosition
    {
        uint8_t address;
        int32_t x__mm, y__mm, z__mm;
        bool high_resolution;
        float distance__m;  // Distance between beacon and hedge
    };

    struct StationaryBeaconsPositions
    {
        uint8_t num_beacons;
        StationaryBeaconPosition beacons[AP_BEACON_MAX_BEACONS];
        bool updated;
    };

    struct MarvelmindHedge
    {
        StationaryBeaconsPositions positions_beacons;
        PositionValue cur_position;
        bool _have_new_values;
    };

    enum {
        RECV_HDR,
        RECV_DGRAM
    } parse_state; // current state of receive data

    MarvelmindHedge hedge;
    uint8_t input_buffer[AP_BEACON_MARVELMIND_BUF_SIZE];
    uint16_t num_bytes_in_block_received;
    uint16_t data_id;

    uint16_t calc_crc_modbus(uint8_t *buf, uint16_t len);
    StationaryBeaconPosition* get_or_alloc_beacon(uint8_t address);
    void process_beacons_positions_datagram();
    void process_beacons_positions_highres_datagram();
    void process_position_highres_datagram();
    void process_position_datagram();
    void process_beacons_distances_datagram();
    void set_stationary_beacons_positions();
    void order_stationary_beacons();
    int8_t find_beacon_instance(uint8_t address) const;

    // Variables for Ardupilot
    AP_HAL::UARTDriver *uart;
    uint32_t last_update_ms;

    // cache the vehicle position in NED coordinates [m]
    Vector3f vehicle_position_NED__m;
    bool vehicle_position_initialized;

    // cache the beacon positions in NED coordinates [m]
    Vector3f beacon_position_NED__m[AP_BEACON_MAX_BEACONS];
    bool beacon_position_initialized;
};

