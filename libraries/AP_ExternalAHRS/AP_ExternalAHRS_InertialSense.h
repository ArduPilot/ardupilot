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
  support for Inertial Sense INS
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_ExternalAHRS/data_sets.h"
#include "ISComm.h"

class AP_ExternalAHRS_InertialSense: public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_InertialSense(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override;

protected:
    uint8_t num_gps_sensors(void) const override;

private:
    int initialize();
    void update_thread();
    bool check_uart();

    int stop_message_broadcasting();
    int enable_message_broadcasting();

    void handleIns1Message(ins_1_t* ins);
    void handleIns2Message(ins_2_t* ins);
    void handleIns3Message(ins_3_t* ins);
    void handleGpsPosMessage(gps_pos_t* pos);
    void handleGpsVelMessage(gps_vel_t* vel);
    void handleGpsRtkPosMiscMessage(gps_rtk_misc_t* misc);
    void handlePimuMessage(pimu_t* pimu);
    void handleMagnetometerMessage(magnetometer_t* mag);
    void handleBarometerMessage(barometer_t* bar);
    void handleInl2NedSigmaMessage(inl2_ned_sigma_t *sigmas);
    void handleDevInfoMessage(dev_info_t *dev_info);
    void handleBitMessage(bit_t* bit);
    int parseIsbData(void* ctx, p_data_t* data, port_handle_t port);

    // callback helper
    static AP_ExternalAHRS_InertialSense *instance;
    static int isbDataHandler(void* ctx, p_data_t* data, port_handle_t port) {
        return instance->parseIsbData(ctx, data, port);
    }
    int ppd_fd;

    bool initialized = false;
    bool _healthy = false;
    AP_GPS_FixType _fix_type = AP_GPS_FixType::NONE;

    uint32_t baudrate;
    int8_t port_num;
    bool port_open = false;
    uint8_t buffer[1024];

    int imu_sample_duration = 20;

    uint32_t last_imu_pkt;
    uint32_t last_gps_pkt;
    uint32_t last_filter_pkt;

    uint8_t comm_buf[2048];
    is_comm_instance_t comm;

    float vel_cov;
    float pos_cov;
    float hgt_cov;

    AP_ExternalAHRS::gps_data_message_t gps_data_msg;

    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;
};

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
