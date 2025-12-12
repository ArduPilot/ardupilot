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

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SENSAITION_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include "AP_ExternalAHRS_SensAItion_Parser.h"

// Driver for a Kebni SensAItion sensor that provides external sensor data to the EKF
class AP_ExternalAHRS_SensAItion : public AP_ExternalAHRS_backend
{
public:
    // Constructor
    AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &_state);

    // Get serial port number for the uart, or -1 if not applicable
    int8_t get_port() const override;

    // Get model/type name
    const char* get_name() const override;

    // Accessors for AP_AHRS
    bool healthy() const override;
    bool initialised() const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;

    // Check for new data. Not used since all processing happens in a thread.
    void update() override {};

    // Return the number of GPS sensors sharing data to AP_GPS.
    uint8_t num_gps_sensors() const override
    {
        // The SensAItion IMU/AHRS models do not have GPS input
        return 0;
    }

private:
    AP_ExternalAHRS_SensAItion_Parser parser;

    // Pre-allocated measurement from parser
    AP_ExternalAHRS_SensAItion_Parser::Measurement sensor_measurement;

    //Buffer for reading from UART
    uint8_t buffer[AP_ExternalAHRS_SensAItion_Parser::MAX_PACKET_SIZE];

    // UART driver and configuration
    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t baudrate = 460800;
    int8_t port_num = -1;

    // Thread-shared variables (setup_complete, last_valid_packet_ms, valid_packets)
    // No semaphore needed - uint32_t/bool operations are atomic on ARM
    bool setup_complete = false;
    uint32_t last_valid_packet_ms = 0;
    uint32_t valid_packets = 0;

    // Thread processing
    void update_thread();

    // Check UART for available data and process it
    // Returns: true if data was read and processed, false if no data available or UART not ready
    bool check_uart();
};

#endif  // AP_EXTERNAL_AHRS_SENSAITION_ENABLED