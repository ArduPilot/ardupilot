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

#include <AP_EFI/AP_EFI_Currawong_ECU.h>

#include "AP_PiccoloCAN_Device.h"
#include "AP_PiccoloCAN_ESC.h"
#include "AP_PiccoloCAN_Servo.h"

#if HAL_PICCOLO_CAN_ENABLE

#define PICCOLO_MSG_RATE_HZ_MIN 1
#define PICCOLO_MSG_RATE_HZ_MAX 500
#define PICCOLO_MSG_RATE_HZ_DEFAULT 50

#define PICCOLO_CAN_ECU_ID_DEFAULT 0

class AP_PiccoloCAN : public AP_CANDriver
{
public:
    AP_PiccoloCAN();
    ~AP_PiccoloCAN();

    // Piccolo message groups form part of the CAN ID of each frame
    enum class MessageGroup : uint8_t {
        SIMULATOR = 0x00,       // Simulator messages
        SENSOR = 0x04,          // External sensors
        ACTUATOR = 0x07,        // Actuators (e.g. ESC / servo)
        ECU_OUT = 0x08,         // Messages *from* an ECU
        ECU_IN = 0x09,          // Message *to* an ECU

        SYSTEM = 0x19,          // System messages (e.g. bootloader)
    };

    // Piccolo device types differentiate between devices
    enum class DeviceType : uint8_t {
        SERVO = 0x00,
        ESC = 0x20,
    };

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PiccoloCAN);

    static const struct AP_Param::GroupInfo var_info[];

    // Return PiccoloCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_PiccoloCAN *get_pcan(uint8_t driver_index);

    // initialize PiccoloCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // called from SRV_Channels
    void update();

    // send ESC telemetry messages over MAVLink
    void send_esc_telemetry_mavlink(uint8_t mav_chan);

    // return true if a particular servo is 'active' on the Piccolo interface
    bool is_servo_channel_active(uint8_t chan);

    // return true if a particular ESC is 'active' on the Piccolo interface
    bool is_esc_channel_active(uint8_t chan);

    // test if the Piccolo CAN driver is ready to be armed
    bool pre_arm_check(char* reason, uint8_t reason_len);

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on succses
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);

    // decode a single CAN frame
    bool decode_frame(AP_HAL::CANFrame &frame);

    // send ESC commands over CAN
    void send_esc_messages(void);

    // send servo commands over CAN
    void send_servo_messages(void);
    
#if AP_EFI_CURRAWONG_ECU_ENABLED
    void send_ecu_messages(void);
#endif

    bool _initialized;
    char _thread_name[16];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_EventHandle _event_handle;

    // Keep track of multiple connected servos
    AP_PiccoloCAN_Servo servo_devices[PICCOLO_CAN_MAX_NUM_SERVO];

    // Keep track of multiple connected ESCs
    AP_PiccoloCAN_ESC esc_devices[PICCOLO_CAN_MAX_NUM_ESC];

    struct CurrawongECU_Info_t {
        float command;
        bool newCommand;
    } _ecu_info;

    // Piccolo CAN parameters
    AP_Int32 _esc_bm;       //! ESC selection bitmask
    AP_Int16 _esc_hz;       //! ESC update rate (Hz)

    AP_Int32 _srv_bm;       //! Servo selection bitmask
    AP_Int16 _srv_hz;       //! Servo update rate (Hz)

    AP_Int16 _ecu_id;        //! ECU Node ID
    AP_Int16 _ecu_hz;       //! ECU update rate (Hz)

    HAL_Semaphore _telem_sem;
};

#endif // HAL_PICCOLO_CAN_ENABLE
