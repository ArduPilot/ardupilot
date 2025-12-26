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

/*
This class is the interface to Kebni's SensAItion range of inertial navigation
sensors, which can provide raw sensor data and/or a sensor fusion solution.
*/
class AP_ExternalAHRS_SensAItion : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &_state);

    // Hardware Identification
    int8_t get_port() const override;
    const char* get_name() const override;

    // Health & Status Interface
    bool healthy() const override;
    bool initialised() const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // GPS Interface
    uint8_t num_gps_sensors() const override;

    // Main Loop
    void update() override
    {
        check_uart();
    }

private:
    mutable HAL_Semaphore sem_handle;

    // All member variables below are accessed from inside the thread
    // and should be protected by the semaphore above!
    // =======================================================
    AP_ExternalAHRS::ins_data_message_t _ins;
    AP_ExternalAHRS::mag_data_message_t _mag;
    AP_ExternalAHRS::baro_data_message_t _baro;
    AP_ExternalAHRS::gps_data_message_t _gps;

    AP_ExternalAHRS_SensAItion_Parser parser;

    // UART
    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t baudrate = 460800;
    int8_t port_num = -1;
    uint8_t buffer[AP_ExternalAHRS_SensAItion_Parser::MAX_PACKET_SIZE];

    bool setup_complete = false;

    // Persistent State
    uint32_t _last_imu_pkt_ms = 0;
    uint32_t _last_ins_pkt_ms = 0; // Only used in INS mode
    uint32_t _last_quat_pkt_ms = 0; // Only used in INS mode

    // Last known values from INS packet (Packet 2)
    uint8_t  _last_alignment_status = 0;
    uint8_t  _last_gnss1_fix = 0;
    uint8_t  _last_gnss2_fix = 0;
    uint8_t  _last_sensor_valid = 0;
    float    _last_h_pos_quality = 999.9f;
    float    _last_v_pos_quality = 999.9f;
    float    _last_vel_quality = 999.9f;
    uint32_t _last_error_flags = 0;

    // End of member variables that should be protected by semaphore
    // ==========================================================

    // Only read from inside the thread, does not need semaphore
    bool _ins_mode_enabled = false;

    void update_thread();
    bool check_uart();

    void handle_imu(const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas, uint32_t now_ms);
    void handle_ahrs(const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas, uint32_t now_ms);
    void handle_ins(const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas, uint32_t now_ms);

    void log_ins_status(const AP_ExternalAHRS_SensAItion_Parser::Measurement &meas);
};
#endif  // AP_EXTERNAL_AHRS_SENSAITION_ENABLED
