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
#include <AP_CANManager/AP_CANDriver.h>

#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#include "piccolo_protocol/ESCPackets.h"
#include "piccolo_protocol/LegacyESCPackets.h"

// maximum number of ESC allowed on CAN bus simultaneously
#define PICCOLO_CAN_MAX_NUM_ESC 12
#define PICCOLO_CAN_MAX_GROUP_ESC (PICCOLO_CAN_MAX_NUM_ESC / 4)

#ifndef HAL_PICCOLO_CAN_ENABLE
#define HAL_PICCOLO_CAN_ENABLE (HAL_NUM_CAN_IFACES && !HAL_MINIMIZE_FEATURES)
#endif

#if HAL_PICCOLO_CAN_ENABLE

#define PICCOLO_MSG_RATE_HZ_MIN 1
#define PICCOLO_MSG_RATE_HZ_MAX 500
#define PICCOLO_MSG_RATE_HZ_DEFAULT 50

class AP_PiccoloCAN : public AP_CANDriver
#if HAL_WITH_ESC_TELEM
 , public AP_ESC_Telem_Backend
#endif
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

    static const struct AP_Param::GroupInfo var_info[];

    // Return PiccoloCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_PiccoloCAN *get_pcan(uint8_t driver_index);

    // initialize PiccoloCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // called from SRV_Channels
    void update();

    // return true if a particular ESC is 'active' on the Piccolo interface
    bool is_esc_channel_active(uint8_t chan);

    // return true if a particular ESC has been detected
    bool is_esc_present(uint8_t chan, uint64_t timeout_ms = 2000) const;

    // return true if a particular ESC is enabled
    bool is_esc_enabled(uint8_t chan);

    // test if the Piccolo CAN driver is ready to be armed
    bool pre_arm_check(char* reason, uint8_t reason_len);

private:

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on succses
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);

    // send ESC commands over CAN
    void send_esc_messages(void);

    // interpret an ESC message received over CAN
    bool handle_esc_message(AP_HAL::CANFrame &frame);

    bool _initialized;
    char _thread_name[16];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_EventHandle _event_handle;
    HAL_Semaphore _telem_sem;

    struct PiccoloESC_Info_t {

        /* Telemetry data provided in the PKT_ESC_STATUS_A packet */
        uint8_t mode;                   //! ESC operational mode
        ESC_StatusBits_t status;        //! ESC status information
        uint16_t setpoint;              //!< ESC operational command - value depends on 'mode' available in this packet. If the ESC is disabled, data reads 0x0000. If the ESC is in open-loop PWM mode, this value is the PWM command in units of 1us, in the range 1000us to 2000us. If the ESC is in closed-loop RPM mode, this value is the RPM command in units of 1RPM
        uint16_t rpm;                   //!< Motor speed

        /* Telemetry data provided in the PKT_ESC_STATUS_B packet */
        uint16_t voltage;          //!< ESC Rail Voltage
        int16_t  current;          //!< ESC Current. Current IN to the ESC is positive. Current OUT of the ESC is negative
        uint16_t dutyCycle;        //!< ESC Motor Duty Cycle
        int8_t   escTemperature;   //!< ESC Logic Board Temperature
        uint8_t  motorTemperature; //!< ESC Motor Temperature

        /* Telemetry data provided in the PKT_ESC_STATUS_C packet */
        float    fetTemperature; //!< ESC Phase Board Temperature
        uint16_t pwmFrequency;   //!< Current motor PWM frequency (10 Hz per bit)
        uint16_t timingAdvance;  //!< Current timing advance (0.1 degree per bit)
        
        /* ESC status information provided in the PKT_ESC_WARNINGS_ERRORS packet */
        ESC_WarningBits_t warnings;     //! ESC warning information
        ESC_ErrorBits_t errors;         //! ESC error information

        ESC_Firmware_t firmware;        //! Firmware / checksum information
        ESC_Address_t address;          //! Serial number
        ESC_EEPROMSettings_t eeprom;    //! Non-volatile settings info

        // Output information

        int16_t command;    //! Raw command to send to each ESC
        bool newCommand;    //! Is the command "new"?
        bool newTelemetry;  //! Is there new telemetry data available?

        uint64_t last_rx_msg_timestamp = 0;    //! Time of most recently received message

    } _esc_info[PICCOLO_CAN_MAX_NUM_ESC];

    // Piccolo CAN parameters
    AP_Int32 _esc_bm;       //! ESC selection bitmask
    AP_Int16 _esc_hz;       //! ESC update rate (Hz)

};

#endif // HAL_PICCOLO_CAN_ENABLE
