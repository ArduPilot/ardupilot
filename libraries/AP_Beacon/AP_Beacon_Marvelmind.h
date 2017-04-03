/*
 * AP_Beacon_Marvelmind.h
 *
 *  Created on: 21.03.2017
 */

#ifndef AP_BEACON_MARVELMIND_H_
#define AP_BEACON_MARVELMIND_H_

#pragma once

#define AP_BEACON_MARVELMIND_MAX_STATIONARY_BEACONS 30
#define AP_BEACON_MARVELMIND_POSITION_DATAGRAM_ID 0x0001
#define AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_ID 0x0002
#define AP_BEACON_MARVELMIND_POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_HIGHRES_ID 0x0012
#define AP_BEACON_MARVELMIND_BUF_SIZE 255

#include "AP_Beacon_Backend.h"

class AP_Beacon_Marvelmind : public AP_Beacon_Backend
{
public:

    struct PositionValue
    {
        uint8_t address;
        uint32_t timestamp;
        int32_t x, y, z;// coordinates in millimeters
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
        struct StationaryBeaconPosition beacons[AP_BEACON_MARVELMIND_MAX_STATIONARY_BEACONS];
        bool updated;
    };

    struct MarvelmindHedge
    {
        uint8_t max_buffered_positions;   // maximum count of measurements of coordinates stored in buffer, default: 3
        struct PositionValue * position_buffer;  // buffer of measurements
        struct StationaryBeaconsPositions positions_beacons;
        bool verbose;   // verbose flag which activate console output, default: False
        bool pause;     //  pause flag. If True, class would not read serial data
        bool terminationRequired;   //  If True, thread would exit from main loop and stop
        void (*receive_data_callback)(struct PositionValue position); //  receive_data_callback is callback function to recieve data

        // private variables
        uint8_t _last_values_count;
        uint8_t _last_values_next;
        bool _have_new_values;
    };

    // constructor
    AP_Beacon_Marvelmind(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy();

    // update
    void update();

private:
    enum {
        RECV_HDR,
        RECV_DGRAM
    } parse_state; // current state of receive data

    struct MarvelmindHedge *hedge;
    struct PositionValue cur_position;
    struct StationaryBeaconPosition cur_beacon;
    uint8_t input_buffer[AP_BEACON_MARVELMIND_BUF_SIZE];
    uint16_t num_bytes_in_block_received;
    uint16_t data_id;

    struct MarvelmindHedge* m_MarvelmindHedge;
    uint16_t calc_crc_modbus(uint8_t *buf, uint16_t len);
    bool get_or_alloc_beacon(struct StationaryBeaconPosition &b, uint8_t address);
    uint8_t mark_position_ready();
    void process_beacons_positions_datagram(struct StationaryBeaconPosition &b);
    void process_beacons_positions_highres_datagram(struct StationaryBeaconPosition &b);
    void process_position_highres_datagram(struct PositionValue &p);
    void process_position_datagram(struct PositionValue &p);
    void create_marvelmind_hedge();
    void start_marvelmind_hedge();
    bool get_position_from_marvelmind_hedge(struct PositionValue *position);
    void stop_marvelmind_hedge();

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_update_ms = 0;
};

#endif /* AP_BEACON_MARVELMIND_H_ */
