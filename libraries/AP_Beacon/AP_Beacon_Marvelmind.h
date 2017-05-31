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
 Original C Code by Marvelmind (https://bitbucket.org/marvelmind_robotics/)
 Adapted into Ardupilot by Karthik Desai, Amilcar Lucas
 April 2017
 */

#ifndef AP_BEACON_MARVELMIND_H_
#define AP_BEACON_MARVELMIND_H_

#pragma once

#define AP_BEACON_MARVELMIND_POSITION_DATAGRAM_ID 0x0001
#define AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_ID 0x0002
#define AP_BEACON_MARVELMIND_POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_HIGHRES_ID 0x0012
#define AP_BEACON_MARVELMIND_BUF_SIZE 255

#include "AP_Beacon_Backend.h"

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
        int32_t x, y, z; // coordinates in millimeters
        bool high_resolution;
        bool ready;
        bool processed;
    };

    struct StationaryBeaconPosition
    {
        uint8_t address;
        int32_t x, y, z;// coordinates in millimeters
        bool high_resolution;
    };

    struct StationaryBeaconsPositions
    {
        uint8_t num_beacons;
        StationaryBeaconPosition beacons[AP_BEACON_MAX_BEACONS];
        bool updated;
    };

    struct MarvelmindHedge
    {
        uint8_t max_buffered_positions;   // maximum count of measurements of coordinates stored in buffer, default: 3
        PositionValue * position_buffer;  // buffer of measurements
        StationaryBeaconsPositions positions_beacons;
        bool verbose;   // verbose flag which activate console output, default: False
        bool pause;     //  pause flag. If True, class would not read serial data
        bool termination_required;  //  If True, thread would exit from main loop and stop
        void (*receive_data_callback)(PositionValue position); //  receive_data_callback is callback function to receive data

        uint8_t _last_values_count;
        uint8_t _last_values_next;
        bool _have_new_values;
    };

    enum {
        RECV_HDR,
        RECV_DGRAM
    } parse_state; // current state of receive data

    MarvelmindHedge *hedge;
    PositionValue cur_position;
    uint8_t input_buffer[AP_BEACON_MARVELMIND_BUF_SIZE];
    uint16_t num_bytes_in_block_received;
    uint16_t data_id;

    uint16_t calc_crc_modbus(uint8_t *buf, uint16_t len);
    StationaryBeaconPosition* get_or_alloc_beacon(uint8_t address);
    uint8_t mark_position_ready();
    void process_beacons_positions_datagram();
    void process_beacons_positions_highres_datagram();
    void process_position_highres_datagram(PositionValue &p);
    void process_position_datagram(PositionValue &p);
    void create_marvelmind_hedge();
    void start_marvelmind_hedge();
    void set_stationary_beacons_positions_and_distances();
    void order_stationary_beacons();

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

#endif /* AP_BEACON_MARVELMIND_H_ */
