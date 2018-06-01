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

#include <AP_HAL/AP_HAL.h>

#include "AP_Beacon_Marvelmind.h"

#define AP_BEACON_MARVELMIND_POSITION_DATAGRAM_ID 0x0001
#define AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_ID 0x0002
#define AP_BEACON_MARVELMIND_DISTANCES_DATAGRAM_ID 0x0004
#define AP_BEACON_MARVELMIND_POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_HIGHRES_ID 0x0012

extern const AP_HAL::HAL& hal;

#define MM_DEBUG_LEVEL 0

#if MM_DEBUG_LEVEL
  #include <GCS_MAVLink/GCS.h>
  #define Debug(level, fmt, args ...)  do { if (level <= MM_DEBUG_LEVEL) { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
  #define Debug(level, fmt, args ...)
#endif

AP_Beacon_Marvelmind::AP_Beacon_Marvelmind(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    AP_Beacon_Backend(frontend)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Beacon, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Beacon, 0));
        last_update_ms = 0;
        parse_state = RECV_HDR; // current state of receive data
        num_bytes_in_block_received = 0; // bytes received
        data_id = 0;
        hedge._have_new_values = false;
        hedge.positions_beacons.num_beacons = 0;
        hedge.positions_beacons.updated = false;

    }
}

//////////////////////////////////////////////////////////////////////////////
// Calculate Modbus CRC16 for array of bytes
// buf: input buffer
// len: size of buffer
// returncode: CRC value
//////////////////////////////////////////////////////////////////////////////
uint16_t AP_Beacon_Marvelmind::calc_crc_modbus(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t) buf[pos]; // XOR byte into least sig. byte of crc
        for (uint8_t i = 8; i != 0; i--) { // Loop over each bit
            if ((crc & 0x0001) != 0) { // If the LSB is set
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else {
                // Else LSB is not set
                crc >>= 1; // Just shift right
            }
        }
    }
    return crc;
}

void AP_Beacon_Marvelmind::process_position_datagram()
{
    hedge.cur_position.address = input_buffer[16];
    hedge.cur_position.timestamp = input_buffer[5]
                  | (((uint32_t) input_buffer[6]) << 8)
                  | (((uint32_t) input_buffer[7]) << 16)
                  | (((uint32_t) input_buffer[8]) << 24);
    const int16_t vx = input_buffer[9] | (((uint16_t) input_buffer[10]) << 8);
    hedge.cur_position.x__mm = vx * 10; // centimeters -> millimeters
    const int16_t vy = input_buffer[11] | (((uint16_t) input_buffer[12]) << 8);
    hedge.cur_position.y__mm = vy * 10; // centimeters -> millimeters
    const int16_t vz = input_buffer[13] | (((uint16_t) input_buffer[14]) << 8);
    hedge.cur_position.z__mm = vz * 10; // centimeters -> millimeters
    hedge.cur_position.high_resolution = false;
    hedge._have_new_values = true;
}

void AP_Beacon_Marvelmind::process_position_highres_datagram()
{
    hedge.cur_position.address = input_buffer[22];
    hedge.cur_position.timestamp = input_buffer[5]
                  | (((uint32_t) input_buffer[6]) << 8)
                  | (((uint32_t) input_buffer[7]) << 16)
                  | (((uint32_t) input_buffer[8]) << 24);
    hedge.cur_position.x__mm = input_buffer[9] | (((uint32_t) input_buffer[10]) << 8)
                  | (((uint32_t) input_buffer[11]) << 16)
                  | (((uint32_t) input_buffer[12]) << 24);
    hedge.cur_position.y__mm = input_buffer[13] | (((uint32_t) input_buffer[14]) << 8)
                  | (((uint32_t) input_buffer[15]) << 16)
                  | (((uint32_t) input_buffer[16]) << 24);
    hedge.cur_position.z__mm = input_buffer[17] | (((uint32_t) input_buffer[18]) << 8)
                  | (((uint32_t) input_buffer[19]) << 16)
                  | (((uint32_t) input_buffer[20]) << 24);
    hedge.cur_position.high_resolution = true;
    hedge._have_new_values = true;
}

AP_Beacon_Marvelmind::StationaryBeaconPosition* AP_Beacon_Marvelmind::get_or_alloc_beacon(uint8_t address)
{
    const uint8_t n_used = hedge.positions_beacons.num_beacons;
    for (uint8_t i = 0; i < n_used; i++) {
        if (hedge.positions_beacons.beacons[i].address == address) {
            return &hedge.positions_beacons.beacons[i];
        }
    }
    if (n_used >= AP_BEACON_MAX_BEACONS) {
        return nullptr;
    }
    hedge.positions_beacons.num_beacons = (n_used + 1);
    return &hedge.positions_beacons.beacons[n_used];
}

void AP_Beacon_Marvelmind::process_beacons_positions_datagram()
{
    const uint8_t n = input_buffer[5]; // number of beacons in packet
    StationaryBeaconPosition *stationary_beacon;
    if ((1 + n * 8) != input_buffer[4]) {
        Debug(1, "beacon pos lo pkt size %d != %d", input_buffer[4], (1 + n * 8));
        return; // incorrect size
    }
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t ofs = 6 + i * 8;
        const uint8_t address = input_buffer[ofs];
        const int16_t x = input_buffer[ofs + 1]
                | (((uint16_t) input_buffer[ofs + 2]) << 8);
        const int16_t y = input_buffer[ofs + 3]
                | (((uint16_t) input_buffer[ofs + 4]) << 8);
        const int16_t z = input_buffer[ofs + 5]
                | (((uint16_t) input_buffer[ofs + 6]) << 8);
        stationary_beacon = get_or_alloc_beacon(address);
        if (stationary_beacon != nullptr) {
            stationary_beacon->address = address; //The instance and the address are the same
            stationary_beacon->x__mm = x * 10; // centimeters -> millimeters
            stationary_beacon->y__mm = y * 10; // centimeters -> millimeters
            stationary_beacon->z__mm = z * 10; // centimeters -> millimeters
            stationary_beacon->high_resolution = false;
            hedge.positions_beacons.updated = true;
        }
    }
    order_stationary_beacons();
}

void AP_Beacon_Marvelmind::process_beacons_positions_highres_datagram()
{
    const uint8_t n = input_buffer[5]; // number of beacons in packet
    StationaryBeaconPosition *stationary_beacon;
    if ((1 + n * 14) != input_buffer[4]) {
        Debug(1, "beacon pos hi pkt size %d != %d", input_buffer[4], (1 + n * 14));
        return; // incorrect size
    }
    for (uint8_t i = 0; i < n; i++) {
        const uint8_t ofs = 6 + i * 14;
        const uint8_t address = input_buffer[ofs];
        const int32_t x = input_buffer[ofs + 1]
                | (((uint32_t) input_buffer[ofs + 2]) << 8)
                | (((uint32_t) input_buffer[ofs + 3]) << 16)
                | (((uint32_t) input_buffer[ofs + 4]) << 24);
        const int32_t y = input_buffer[ofs + 5]
                | (((uint32_t) input_buffer[ofs + 6]) << 8)
                | (((uint32_t) input_buffer[ofs + 7]) << 16)
                | (((uint32_t) input_buffer[ofs + 8]) << 24);
        const int32_t z = input_buffer[ofs + 9]
                | (((uint32_t) input_buffer[ofs + 10]) << 8)
                | (((uint32_t) input_buffer[ofs + 11]) << 16)
                | (((uint32_t) input_buffer[ofs + 12]) << 24);
        stationary_beacon = get_or_alloc_beacon(address);
        if (stationary_beacon != nullptr) {
            stationary_beacon->address = address; //The instance and the address are the same
            stationary_beacon->x__mm = x; // millimeters
            stationary_beacon->y__mm = y; // millimeters
            stationary_beacon->z__mm = z; // millimeters
            stationary_beacon->high_resolution = true;
            hedge.positions_beacons.updated = true;
        }
    }
    order_stationary_beacons();
}

void AP_Beacon_Marvelmind::process_beacons_distances_datagram()
{
    if (32 != input_buffer[4]) {
        Debug(1, "beacon dist pkt size %d != 32", input_buffer[4]);
        return; // incorrect size
    }
    bool set = false;
    for (uint8_t i = 0; i < hedge.positions_beacons.num_beacons; i++) {
        const uint8_t ofs = 6 + i * 6;
        const uint8_t address = input_buffer[ofs];
        const int8_t instance = find_beacon_instance(address);
        if (instance != -1) {
            const uint32_t distance = input_buffer[ofs + 1]
                                      | (((uint32_t) input_buffer[ofs + 2]) << 8)
                                      | (((uint32_t) input_buffer[ofs + 3]) << 16)
                                      | (((uint32_t) input_buffer[ofs + 4]) << 24);
            hedge.positions_beacons.beacons[instance].distance__m = distance * 0.001f; // millimeters -> meters
            set_beacon_distance(instance, hedge.positions_beacons.beacons[instance].distance__m);
            set = true;
            Debug(2, "Beacon %d is %.2fm", instance, hedge.positions_beacons.beacons[instance].distance__m);
        }
    }
    if (set) {
        last_update_ms = AP_HAL::millis();
    }
}

int8_t AP_Beacon_Marvelmind::find_beacon_instance(uint8_t address) const
{
    for (uint8_t i = 0; i < hedge.positions_beacons.num_beacons; i++) {
        if (hedge.positions_beacons.beacons[i].address == address) {
            return i;
        }
    }
    return -1;
}

void AP_Beacon_Marvelmind::update(void)
{
    if (uart == nullptr) {
        return;
    }
    // read any available characters
    int32_t num_bytes_read = uart->available();
    uint8_t received_char = 0;
    if (num_bytes_read < 0) {
        return;
    }
    while (num_bytes_read-- > 0) {
        bool good_byte = false;
        received_char = uart->read();
        input_buffer[num_bytes_in_block_received] = received_char;
        switch (parse_state) {
        case RECV_HDR:
            switch (num_bytes_in_block_received) {
            case 0:
                good_byte = (received_char == 0xff);
                break;
            case 1:
                good_byte = (received_char == 0x47);
                break;
            case 2:
                good_byte = true;
                break;
            case 3:
                data_id = (((uint16_t)received_char) << 8) + input_buffer[2];
                good_byte = (data_id == AP_BEACON_MARVELMIND_POSITION_DATAGRAM_ID)
                         || (data_id == AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_ID)
                         || (data_id == AP_BEACON_MARVELMIND_DISTANCES_DATAGRAM_ID)
                         || (data_id == AP_BEACON_MARVELMIND_POSITION_DATAGRAM_HIGHRES_ID)
                         || (data_id == AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_HIGHRES_ID);
                break;
            case 4: {
                switch (data_id) {
                case AP_BEACON_MARVELMIND_POSITION_DATAGRAM_ID: {
                    good_byte = (received_char == 0x10);
                    break;
                }
                case AP_BEACON_MARVELMIND_DISTANCES_DATAGRAM_ID:
                case AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_ID:
                case AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_HIGHRES_ID:
                    good_byte = true;
                    break;
                case AP_BEACON_MARVELMIND_POSITION_DATAGRAM_HIGHRES_ID: {
                    good_byte = (received_char == 0x16);
                    break;
                }
                }
                if (good_byte) {
                    parse_state = RECV_DGRAM;
                }
                break;
            }
            }
            if (good_byte) {
                // correct header byte
                num_bytes_in_block_received++;
            } else {
                // ...or incorrect
                parse_state = RECV_HDR;
                num_bytes_in_block_received = 0;
            }
            break;

        case RECV_DGRAM:
            num_bytes_in_block_received++;
            if (num_bytes_in_block_received >= 7 + input_buffer[4]) {
                // parse dgram
                uint16_t block_crc = calc_crc_modbus(input_buffer, num_bytes_in_block_received);
                if (block_crc == 0) {
                    switch (data_id) {
                        case AP_BEACON_MARVELMIND_POSITION_DATAGRAM_ID:
                        {
                            // add to position_buffer
                            process_position_datagram();
                            vehicle_position_initialized = true;
                            set_stationary_beacons_positions();
                            break;
                        }

                        case AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_ID:
                        {
                            process_beacons_positions_datagram();
                            beacon_position_initialized = true;
                            set_stationary_beacons_positions();
                            break;
                        }

                        case AP_BEACON_MARVELMIND_DISTANCES_DATAGRAM_ID:
                        {
                            process_beacons_distances_datagram();
                            break;
                        }

                        case AP_BEACON_MARVELMIND_POSITION_DATAGRAM_HIGHRES_ID:
                        {
                            process_position_highres_datagram();
                            vehicle_position_initialized = true;
                            set_stationary_beacons_positions();
                            break;
                        }

                        case AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_HIGHRES_ID:
                        {
                            process_beacons_positions_highres_datagram();
                            beacon_position_initialized = true;
                            set_stationary_beacons_positions();
                            break;
                        }
                    }
                }
                // and repeat
                parse_state = RECV_HDR;
                num_bytes_in_block_received = 0;
            }
            break;
        }
    }
}

bool AP_Beacon_Marvelmind::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

void AP_Beacon_Marvelmind::set_stationary_beacons_positions()
{
    bool set = false;
    if (vehicle_position_initialized && beacon_position_initialized) {
        if (hedge._have_new_values) {
            vehicle_position_NED__m = Vector3f(hedge.cur_position.y__mm * 0.001f,
                                               hedge.cur_position.x__mm * 0.001f,
                                              -hedge.cur_position.z__mm * 0.001f); //Transform Marvelmind ENU to Ardupilot NED
            //TODO: Calculate Accuracy of the received signal. Marvelmind *advertises* +/- 2cms
            // But we are conservative here and use 20cm instead (until MM provides us with a proper accuracy value)
            set_vehicle_position(vehicle_position_NED__m, 0.2f);
            set = true;
            Debug(2,
                  "Hedge is at N%.2f, E%.2f, D%.2f",
                  vehicle_position_NED__m[0],
                  vehicle_position_NED__m[1],
                  vehicle_position_NED__m[2]);
        }
        hedge._have_new_values = false;
        for (uint8_t i = 0; i < hedge.positions_beacons.num_beacons; ++i) {
            if (hedge.positions_beacons.updated) {
                beacon_position_NED__m[i] = Vector3f(hedge.positions_beacons.beacons[i].y__mm * 0.001f,
                                                     hedge.positions_beacons.beacons[i].x__mm * 0.001f,
                                                    -hedge.positions_beacons.beacons[i].z__mm * 0.001f); //Transform Marvelmind ENU to Ardupilot NED
                set_beacon_position(i, beacon_position_NED__m[i]);
                set = true;
                Debug(2,
                      "Beacon %d is at N%.2f, E%.2f, D%.2f",
                      i,
                      beacon_position_NED__m[i][0],
                      beacon_position_NED__m[i][1],
                      beacon_position_NED__m[i][2]);
            }
        }
        hedge.positions_beacons.updated = false;

    }
    if (set) {
        last_update_ms = AP_HAL::millis();
    }
}

void AP_Beacon_Marvelmind::order_stationary_beacons()
{
    if (hedge.positions_beacons.updated) {
        bool swapped = false;
        uint8_t j = hedge.positions_beacons.num_beacons;
        do
        {
            swapped = false;
            StationaryBeaconPosition beacon_to_swap;
            for (uint8_t i = 1; i < j; i++) {
                if (hedge.positions_beacons.beacons[i-1].address > hedge.positions_beacons.beacons[i].address) {
                    beacon_to_swap = hedge.positions_beacons.beacons[i];
                    hedge.positions_beacons.beacons[i] = hedge.positions_beacons.beacons[i-1];
                    hedge.positions_beacons.beacons[i-1] = beacon_to_swap;
                    swapped = true;
                }
            }
            j--;
        } while(swapped);
    }
}
