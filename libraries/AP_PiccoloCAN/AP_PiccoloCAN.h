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
 * Author: Oliver Walters
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>

#include "piccolo_protocol/ESCPackets.h"

// maximum number of ESC allowed on CAN bus simultaneously
#define PICCOLO_CAN_MAX_NUM_ESC 12
#define PICCOLO_CAN_MAX_GROUP_ESC (PICCOLO_CAN_MAX_NUM_ESC / 4)

#ifndef HAL_PICCOLO_CAN_ENABLE
#define HAL_PICCOLO_CAN_ENABLE (HAL_WITH_UAVCAN && !HAL_MINIMIZE_FEATURES)
#endif

#if HAL_PICCOLO_CAN_ENABLE

class AP_PiccoloCAN : public AP_HAL::CANProtocol
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

    // Piccolo actuator types differentiate between actuator frames
    enum class ActuatorType : uint8_t {
        SERVO = 0x00,
        ESC = 0x20,
    };

    /* Do not allow copies */
    AP_PiccoloCAN(const AP_PiccoloCAN &other) = delete;
    AP_PiccoloCAN &operator=(const AP_PiccoloCAN&) = delete;

    // Return PiccoloCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_PiccoloCAN *get_pcan(uint8_t driver_index);

    // initialize PiccoloCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;

    // called from SRV_Channels
    void update();

    // send ESC telemetry messages over MAVLink
    void send_esc_telemetry_mavlink(uint8_t mav_chan);

    // return true if a particular ESC has been detected
    bool is_esc_present(uint8_t chan, uint64_t timeout_ms = 2000);

    // return true if a particular ESC is enabled
    bool is_esc_enabled(uint8_t chan);

    // test if the Piccolo CAN driver is ready to be armed
    bool pre_arm_check(char* reason, uint8_t reason_len);

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(uavcan::CanFrame &out_frame, uavcan::MonotonicTime timeout);

    // read frame on CAN bus, returns true on succses
    bool read_frame(uavcan::CanFrame &recv_frame, uavcan::MonotonicTime timeout);

    // send ESC commands over CAN
    void send_esc_messages(void);

    // interpret an ESC message received over CAN
    bool handle_esc_message(uavcan::CanFrame &frame);

    bool _initialized;
    char _thread_name[16];
    uint8_t _driver_index;
    uavcan::ICanDriver* _can_driver;
    const uavcan::CanFrame* _select_frames[uavcan::MaxCanIfaces] { };

    HAL_Semaphore _telem_sem;

    struct PiccoloESC_Info_t {

        // ESC telemetry information
        ESC_StatusA_t statusA;          //! Telemetry data
        ESC_StatusB_t statusB;          //! Telemetry data
        ESC_Firmware_t firmware;        //! Firmware / checksum information
        ESC_Address_t address;          //! Serial number
        ESC_EEPROMSettings_t eeprom;    //! Non-volatile settings info

        // Output information

        int16_t command;    //! Raw command to send to each ESC
        bool newCommand;    //! Is the command "new"?
        bool newTelemetry;  //! Is there new telemetry data available?

        uint64_t last_rx_msg_timestamp = 0;    //! Time of most recently received message

    } _esc_info[PICCOLO_CAN_MAX_NUM_ESC];

};

#endif // HAL_PICCOLO_CAN_ENABLE
