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

#include <AP_HAL/AP_HAL.h>

#include "AP_Beacon_Marvelmind.h"

extern const AP_HAL::HAL& hal;

AP_Beacon_Marvelmind::AP_Beacon_Marvelmind(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    AP_Beacon_Backend(frontend)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Beacon, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Beacon, 0));
        hedge = new MarvelmindHedge();
        last_update_ms = 0;
        if (hedge) {
            create_marvelmind_hedge();
            parse_state = RECV_HDR; // current state of receive data
            num_bytes_in_block_received = 0; // bytes received
            data_id = 0;
            start_marvelmind_hedge();
        } else {
            // initialising beacon failed
        }
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

uint8_t AP_Beacon_Marvelmind::mark_position_ready()
{
    uint8_t ind = hedge->_last_values_next;
    const uint8_t ind_cur = ind;
    hedge->position_buffer[ind].ready = true;
    hedge->position_buffer[ind].processed = false;
    ind++;
    if (ind >= hedge->max_buffered_positions) {
        ind = 0;
    }
    if (hedge->_last_values_count < hedge->max_buffered_positions) {
        hedge->_last_values_count++;
    }
    hedge->_have_new_values = true;
    hedge->_last_values_next = ind;
    return ind_cur;
}

void AP_Beacon_Marvelmind::process_position_datagram(AP_Beacon_Marvelmind::PositionValue &p)
{
    uint8_t ind = hedge->_last_values_next;
    hedge->position_buffer[ind].address = input_buffer[16];
    hedge->position_buffer[ind].timestamp = input_buffer[5]
            | (((uint32_t) input_buffer[6]) << 8)
            | (((uint32_t) input_buffer[7]) << 16)
            | (((uint32_t) input_buffer[8]) << 24);
    const int16_t vx = input_buffer[9] | (((uint16_t) input_buffer[10]) << 8);
    hedge->position_buffer[ind].x = vx * 10; // centimeters -> millimeters
    const int16_t vy = input_buffer[11] | (((uint16_t) input_buffer[12]) << 8);
    hedge->position_buffer[ind].y = vy * 10; // centimeters -> millimeters
    const int16_t vz = input_buffer[13] | (((uint16_t) input_buffer[14]) << 8);
    hedge->position_buffer[ind].z = vz * 10; // centimeters -> millimeters
    hedge->position_buffer[ind].high_resolution = false;
    ind = mark_position_ready();
    p = hedge->position_buffer[ind];
}

void AP_Beacon_Marvelmind::process_position_highres_datagram(AP_Beacon_Marvelmind::PositionValue &p)
{
    uint8_t ind = hedge->_last_values_next;
    hedge->position_buffer[ind].address = input_buffer[22];
    hedge->position_buffer[ind].timestamp = input_buffer[5]
            | (((uint32_t) input_buffer[6]) << 8)
            | (((uint32_t) input_buffer[7]) << 16)
            | (((uint32_t) input_buffer[8]) << 24);
    const int32_t vx = input_buffer[9] | (((uint32_t) input_buffer[10]) << 8)
            | (((uint32_t) input_buffer[11]) << 16)
            | (((uint32_t) input_buffer[12]) << 24);
    hedge->position_buffer[ind].x = vx;
    const int32_t vy = input_buffer[13] | (((uint32_t) input_buffer[14]) << 8)
            | (((uint32_t) input_buffer[15]) << 16)
            | (((uint32_t) input_buffer[16]) << 24);
    hedge->position_buffer[ind].y = vy;
    const int32_t vz = input_buffer[17] | (((uint32_t) input_buffer[18]) << 8)
            | (((uint32_t) input_buffer[19]) << 16)
            | (((uint32_t) input_buffer[20]) << 24);
    hedge->position_buffer[ind].z = vz;
    hedge->position_buffer[ind].high_resolution = true;
    ind = mark_position_ready();
    p = hedge->position_buffer[ind];
}

AP_Beacon_Marvelmind::StationaryBeaconPosition* AP_Beacon_Marvelmind::get_or_alloc_beacon(uint8_t address)
{
    const uint8_t n_used = hedge->positions_beacons.num_beacons;
    if (n_used != 0) {
        for (uint8_t i = 0; i < n_used; i++) {
            if (hedge->positions_beacons.beacons[i].address == address) {
                return &hedge->positions_beacons.beacons[i];
            }
        }
    }
    if (n_used >= AP_BEACON_MAX_BEACONS) {
        return nullptr;
    }
    hedge->positions_beacons.num_beacons = (n_used + 1);
    return &hedge->positions_beacons.beacons[n_used];
}

void AP_Beacon_Marvelmind::process_beacons_positions_datagram()
{
    const uint8_t n = input_buffer[5]; // number of beacons in packet
    StationaryBeaconPosition *stationary_beacon;
    if ((1 + n * 8) != input_buffer[4]) {
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
            stationary_beacon->x = x * 10; // centimeters -> millimeters
            stationary_beacon->y = y * 10; // centimeters -> millimeters
            stationary_beacon->z = z * 10; // centimeters -> millimeters
            stationary_beacon->high_resolution = false;
            hedge->positions_beacons.updated = true;
        }
    }
    order_stationary_beacons();
}

void AP_Beacon_Marvelmind::process_beacons_positions_highres_datagram()
{
    const uint8_t n = input_buffer[5]; // number of beacons in packet
    StationaryBeaconPosition *stationary_beacon;
    if ((1 + n * 14) != input_buffer[4]) {
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
            stationary_beacon->x = x; // millimeters
            stationary_beacon->y = y; // millimeters
            stationary_beacon->z = z; // millimeters
            stationary_beacon->high_resolution = true;
            hedge->positions_beacons.updated = true;
        }
    }
    order_stationary_beacons();
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
                         || (data_id == AP_BEACON_MARVELMIND_POSITION_DATAGRAM_HIGHRES_ID)
                         || (data_id == AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_HIGHRES_ID);
                break;
            case 4: {
                switch (data_id) {
                case AP_BEACON_MARVELMIND_POSITION_DATAGRAM_ID: {
                    good_byte = (received_char == 0x10);
                    break;
                }
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
                            process_position_datagram(cur_position);
                            vehicle_position_initialized = true;
                            set_stationary_beacons_positions_and_distances();
                            break;
                        }

                        case AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_ID:
                        {
                            process_beacons_positions_datagram();
                            beacon_position_initialized = true;
                            set_stationary_beacons_positions_and_distances();
                            break;
                        }

                        case AP_BEACON_MARVELMIND_POSITION_DATAGRAM_HIGHRES_ID:
                        {
                            process_position_highres_datagram(cur_position);
                            vehicle_position_initialized = true;
                            set_stationary_beacons_positions_and_distances();
                            break;
                        }

                        case AP_BEACON_MARVELMIND_POSITIONS_DATAGRAM_HIGHRES_ID:
                        {
                            process_beacons_positions_highres_datagram();
                            beacon_position_initialized = true;
                            set_stationary_beacons_positions_and_distances();
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

//////////////////////////////////////////////////////////////////////////////
// Create and initialize MarvelmindHedge structure
//////////////////////////////////////////////////////////////////////////////
void AP_Beacon_Marvelmind::create_marvelmind_hedge()
{
    hedge->max_buffered_positions = 3;
    hedge->position_buffer = nullptr;
    hedge->verbose = false;
    hedge->receive_data_callback = nullptr;
    hedge->_last_values_count = 0;
    hedge->_last_values_next = 0;
    hedge->_have_new_values = false;
    hedge->termination_required = false;
}

//////////////////////////////////////////////////////////////////////////////
// Initialize and start work
//////////////////////////////////////////////////////////////////////////////
void AP_Beacon_Marvelmind::start_marvelmind_hedge()
{
    hedge->position_buffer = (PositionValue*) malloc(sizeof(struct PositionValue) * hedge->max_buffered_positions);
    if (hedge->position_buffer == nullptr) {
        if (hedge->verbose) {
            hal.console->printf("MarvelMind: Not enough memory");
        }
        hedge->termination_required = true;
        return;
    }
    for (uint8_t i = 0; i < hedge->max_buffered_positions; i++) {
        hedge->position_buffer[i].ready = false;
        hedge->position_buffer[i].processed = false;
    }
    hedge->positions_beacons.num_beacons = 0;
    hedge->positions_beacons.updated = false;
}

bool AP_Beacon_Marvelmind::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

void AP_Beacon_Marvelmind::set_stationary_beacons_positions_and_distances()
{
    if (vehicle_position_initialized && beacon_position_initialized) {
        if (hedge->_have_new_values) {
            vehicle_position_NED__m = Vector3f(cur_position.y / 1000.0f,
                                               cur_position.x / 1000.0f,
                                              -cur_position.z / 1000.0f); //Transform Marvelmind ENU to Ardupilot NED
            //TODO: Calculate Accuracy of the received signal. Marvelmind *advertises* +/- 2cms
            set_vehicle_position(vehicle_position_NED__m, 0.02f);
            last_update_ms = AP_HAL::millis();
        }
        for (uint8_t i=0; i < hedge->positions_beacons.num_beacons; ++i) {
            if (hedge->positions_beacons.updated) {
                beacon_position_NED__m[i] = Vector3f(hedge->positions_beacons.beacons[i].y / 1000.0f,
                                                     hedge->positions_beacons.beacons[i].x / 1000.0f,
                                                    -hedge->positions_beacons.beacons[i].z / 1000.0f); //Transform Marvelmind ENU to Ardupilot NED
                set_beacon_position(i, beacon_position_NED__m[i]);
            }
            if (hedge->_have_new_values) {
                // this is a big hack:
                // The distances measured in the hedgehog to each beacon are not available in the Marvelmind serial protocol
                // As a workaround we use the triangulated position calculated by the Marvelmind hardware and calculate the distances to the beacons
                // as a result the EKF will not have to resolve ambiguities
                set_beacon_distance(i, (beacon_position_NED__m[i] - vehicle_position_NED__m).length());
            }
        }
        hedge->positions_beacons.updated = false;
        hedge->_have_new_values = false;
    }
}

void AP_Beacon_Marvelmind::order_stationary_beacons()
{
    if (hedge->positions_beacons.updated) {
        bool swapped = false;
        uint8_t j = hedge->positions_beacons.num_beacons;
        do
        {
            swapped = false;
            StationaryBeaconPosition beacon_to_swap;
            for(uint8_t i = 1; i < j; i++) {
                if (hedge->positions_beacons.beacons[i-1].address > hedge->positions_beacons.beacons[i].address) {
                    beacon_to_swap = hedge->positions_beacons.beacons[i];
                    hedge->positions_beacons.beacons[i] = hedge->positions_beacons.beacons[i-1];
                    hedge->positions_beacons.beacons[i-1] = beacon_to_swap;
                    swapped = true;
                }
            }
            j--;
        } while(swapped);
    }
}
