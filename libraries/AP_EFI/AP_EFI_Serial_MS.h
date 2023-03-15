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
#pragma once

#include "AP_EFI_config.h"

#if AP_EFI_SERIAL_MS_ENABLED

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

// RPM Threshold for fuel consumption estimator
#define RPM_THRESHOLD                100

class AP_EFI_Serial_MS: public AP_EFI_Backend {
    
public:
    // Constructor with initialization
    AP_EFI_Serial_MS(AP_EFI &_frontend);

    // Update the state structure
    void update() override;

private:
    AP_HAL::UARTDriver *port;
    void parse_realtime_data();
    bool read_incoming_realtime_data();
    void send_request(uint8_t table, uint16_t first_offset, uint16_t last_offset);
    uint8_t read_byte_CRC32();
    uint32_t CRC32_compute_byte(uint32_t inCrc32, uint8_t data);

    // Serial Protocol Variables
    uint32_t checksum;
    uint8_t step;
    uint8_t response_flag;
    uint16_t message_counter;
    uint32_t last_response_ms;

    // confirmed that last command was ok
    bool last_command_confirmed;

    // Command Response Codes
    enum response_codes {
        RESPONSE_WRITE_OK =0x00,
        RESPONSE_REALTIME_DATA,
        RESPONSE_PAGE_DATA,
        RESPONSE_CONFIG_ERROR,
        RESPONSE_PAGE10_OK,
        RESPONSE_CAN_DATA,
        // Error Responses
        ERR_UNDERRUN = 0X80,
        ERR_OVERRUN,
        ERR_CRC_FAILURE,
        ERR_UNRECOGNIZED_COMMAND,
        ERR_OUT_OF_RANGE,
        ERR_SERIAL_BUSY,
        ERR_FLASH_LOCKED,
        ERR_SEQ_FAIL_1,
        ERR_SEQ_FAIL_2,
        ERR_CAN_QUEUE_FULL,
        ERR_CAN_TIMEOUT,
        ERR_CAN_FAILURE,
        ERR_PARITY,
        ERR_FRAMING,
        ERR_SERIAL_NOISE,
        ERR_TXMODE_RANGE,
        ERR_UNKNOWN
    };
    
    // Realtime Data Table Locations
    enum realtime_data {
        PW1_MSB = 2,
        PW1_LSB,
        RPM_MSB = 6,
        RPM_LSB,
        ADVANCE_MSB,
        ADVANCE_LSB,
        ENGINE_BM = 11,
        BAROMETER_MSB = 16,
        BAROMETER_LSB,
        MAP_MSB,
        MAP_LSB,
        MAT_MSB,
        MAT_LSB,
        CHT_MSB,
        CHT_LSB,
        TPS_MSB,
        TPS_LSB,
        AFR1_MSB = 28,
        AFR1_LSB,
        AFR2_MSB,
        AFR2_LSB,
        DWELL_MSB = 62,
        DWELL_LSB,
        LOAD = 66,
        FUEL_PRESSURE_MSB = 128,
        FUEL_PRESSURE_LSB, 
        // Helpers used when sending request
        RT_FIRST_OFFSET = PW1_MSB,
        RT_LAST_OFFSET = FUEL_PRESSURE_LSB
    };
};

#endif  // AP_EFI_SERIAL_MS_ENABLED
