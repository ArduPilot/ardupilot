/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANDriver.h>

#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#include "AP_PiccoloCAN_config.h"
#include "AP_PiccoloCAN_Device.h"
#include "AP_PiccoloCAN_Device.h"
#include "AP_PiccoloCAN_ESC.h"
#include "AP_PiccoloCAN_ECU.h"
#include "AP_PiccoloCAN_Servo.h"
#include <AP_EFI/AP_EFI_Currawong_ECU.h>

#if HAL_PICCOLO_CAN_ENABLE

#define PICCOLO_MSG_RATE_HZ_MIN 1
#define PICCOLO_MSG_RATE_HZ_MAX 500
#define PICCOLO_MSG_RATE_HZ_DEFAULT 50

class AP_PiccoloCAN : public AP_CANDriver, public AP_ESC_Telem_Backend
{
public:
    AP_PiccoloCAN();
    ~AP_PiccoloCAN();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PiccoloCAN);

    static const struct AP_Param::GroupInfo var_info[];

    // Return PiccoloCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_PiccoloCAN *get_pcan(uint8_t driver_index);

    // initialize PiccoloCAN bus
    void init(uint8_t driver_index) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint32_t timeout_us);

    // called from SRV_Channels
    void update();

    // return true if a particular servo is 'active' on the Piccolo interface
    bool is_servo_channel_active(uint8_t chan);

    // return true if a particular ESC is 'active' on the Piccolo interface
    bool is_esc_channel_active(uint8_t chan);

    // return true if a particular servo has been detected on the CAN interface
    bool is_servo_present(uint8_t chan, uint32_t timeout_us = 2000000);

    // return true if a particular ESC has been detected on the CAN interface
    bool is_esc_present(uint8_t chan, uint32_t timeout_us = 2000000);

    // return true if a particular servo is enabled
    bool is_servo_enabled(uint8_t chan);

    // return true if a particular ESC is enabled
    bool is_esc_enabled(uint8_t chan);

    // test if the Piccolo CAN driver is ready to be armed
    bool pre_arm_check(char* reason, uint8_t reason_len);

private:

    // loop to send output to ESCs in background thread
    void loop();

    // read frame on CAN bus, returns true on succses
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint32_t timeout_us);

    // send ESC commands over CAN
    void send_esc_messages(void);

    // interpret an ESC message received over CAN
    bool handle_esc_message(AP_HAL::CANFrame &frame);

    // send servo commands over CAN
    void send_servo_messages(void);

    // interpret a servo message received over CAN
    bool handle_servo_message(AP_HAL::CANFrame &frame);

#if AP_EFI_CURRAWONG_ECU_ENABLED
    void send_ecu_messages(void);

    // interpret an ECU message received over CAN
    bool handle_ecu_message(AP_HAL::CANFrame &frame);
#endif

    bool handle_cortex_message(AP_HAL::CANFrame &frame);

    bool _initialized;
    char _thread_name[16];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_BinarySemaphore sem_handle;

    AP_PiccoloCAN_Servo _servos[PICCOLO_CAN_MAX_NUM_SERVO];
    AP_PiccoloCAN_ESC _escs[PICCOLO_CAN_MAX_NUM_ESC];

    struct CurrawongECU_Info_t {
        float command;
        bool newCommand;
    } _ecu_info;

    // Piccolo CAN parameters
    AP_Int32 _esc_bm;       //!< ESC selection bitmask
    AP_Int16 _esc_hz;       //!< ESC update rate (Hz)

    AP_Int32 _srv_bm;       //!< Servo selection bitmask
    AP_Int16 _srv_hz;       //!< Servo update rate (Hz)

    AP_Int16 _ecu_id;       //!< ECU Node ID
    AP_Int16 _ecu_hz;       //!< ECU update rate (Hz)

};

#endif // HAL_PICCOLO_CAN_ENABLE
