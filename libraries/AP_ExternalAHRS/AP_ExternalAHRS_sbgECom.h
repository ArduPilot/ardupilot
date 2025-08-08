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
  support for serial connected AHRS systems
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SBGECOM_ENABLED

#include "AP_ExternalAHRS_backend.h"

#include <sbgEComLib.h>

class AP_ExternalAHRS_sbgECom : public AP_ExternalAHRS_backend
{

public:
    AP_ExternalAHRS_sbgECom(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override;

    // Get model/type name
    const char *get_name() const override;

    AP_ExternalAHRS::gps_data_message_t gnss_data;

protected:
    uint8_t num_gps_sensors(void) const override
    {
        return 1;
    }

private:
    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    bool setup_complete;
    uint32_t baudrate;

    bool check_uart();
    bool send_uart();

    void initialize();

    // void run_command(const char *fmt, ...);

    static void process_imu_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg);
    static void process_ekf_quat_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg);
    static void process_ekf_nav_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg);
    static void process_mag_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg);
    static void process_gnss_pos_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg);
    static void process_gnss_vel_packet(const SbgEComLogUnion *ref_sbg_data, void *user_arg);

    static SbgErrorCode sendAirDataLog(SbgEComHandle *handle);
    static SbgErrorCode sendMagDataLog(SbgEComHandle *handle);

    static void printLogCallBack(const char *file_name, const char *function_name, uint32_t line, const char *category,
                                 SbgDebugLogType log_type, SbgErrorCode error_code, const char *message);
    static void getAndPrintProductInfo(SbgEComHandle *handle);
    static SbgErrorCode onLogReceived(SbgEComHandle *handle, SbgEComClass msg_class, SbgEComMsgId msg,
                                      const SbgEComLogUnion *ref_sbg_data, void *user_arg);
    static SbgErrorCode readCallback(SbgInterface *p_interface, void *p_buffer, size_t *p_read_bytes, size_t bytes_to_read);
    static SbgErrorCode destroyCallback(SbgInterface *p_interface);
    static SbgErrorCode writeCallback(SbgInterface *p_interface, const void *p_buffer, size_t bytes_to_write);
    static SbgErrorCode flushCallback(SbgInterface *p_interface, uint32_t flags);
    static SbgErrorCode setSpeedCallback(SbgInterface *p_interface, uint32_t speed);
    static uint32_t getSpeedCallback(const SbgInterface *p_interface);
    static uint32_t delayCallback(const SbgInterface *p_interface, size_t numBytes);

    // SBG interface and state variables
    SbgInterface _sbg_interface;
    SbgEComHandle _com_handle;
    SbgEComLogUnion _log_data;
};

#endif // AP_EXTERNAL_AHRS_SBGECOM_ENABLED
