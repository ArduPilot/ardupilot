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
   This driver supports communicating with the Iris Orca linear actuator via RS485
   ModBus protocol.
 */

#pragma once

#include "AP_IrisOrca_config.h"

#if HAL_IRISORCA_ENABLED

#include <AP_Param/AP_Param.h>

#define IRISORCA_MESSAGE_LEN_MAX    35  // messages are no more than 35 bytes

class AP_IrisOrca {
public:
    AP_IrisOrca();

    CLASS_NO_COPY(AP_IrisOrca);

    static AP_IrisOrca* get_singleton();

    // initialise driver
    void init();

    // consume incoming messages from actuator, reply with latest actuator position
    // runs in background thread
    void thread_main();

    // returns true if communicating with the actuator
    bool healthy();

    static const struct AP_Param::GroupInfo var_info[];

private:

    // message addresses
    enum class MsgAddress : uint8_t {
        BUS_MASTER = 0x00,
        DEVICE = 0x01
    };

    // function codes
    enum class FunctionCode : uint8_t {
        MOTOR_COMMAND_STREAM = 0x64
    };

    // sub codes for MOTOR_COMMAND_STREAM
    enum class MotorCommandStreamSubCode : uint8_t {
        FORCE_CONTROL_STREAM = 0x1C,
        POSITION_CONTROL_STREAM = 0x1E,
        SLEEP_DATA_STREAM = 0x00 // sleep data stream is everything else
    };

    // initialise serial port (run from background thread)
    // returns true on success
    bool init_internals();

    // returns true if it is safe to send a message
    bool safe_to_send() const { return ((_send_delay_us == 0) && (_reply_wait_start_ms == 0)); }

    // set pin to enable sending a message
    void send_start();

    // check for timeout after sending a message and unset pin if required
    void check_for_send_end();

    // calculate delay required to allow message to be completely sent
    uint32_t calc_send_delay_us(uint8_t num_bytes);

    // record msgid of message to wait for and set timer for reply timeout handling
    void set_expected_reply_msgid(uint8_t msg_id);

    // check for timeout waiting for reply
    void check_for_reply_timeout();

    // mark reply received. should be called whenever a message is received regardless of whether we are actually waiting for a reply
    void set_reply_received();

    // send a 100/0x64 Motor Command Stream message to the actuator
    // returns true on success
    bool send_motor_command_stream(uint8_t sub_code, uint32_t data);

    // send a actuator position command as a value from 0 to the maximum travel
    // value is taken directly from the steering servo channel
    void send_actuator_position_cmd();

    // parameters
    AP_Int8 _pin_de;        // Pin number connected to RS485 to Serial converter's DE pin. -1 to disable sending commands to actuator
    AP_Int16 _max_travel_mm;// maximum travel of actuator in millimeters

    // members
    AP_HAL::UARTDriver *_uart;          // serial port to communicate with actuator
    bool _initialised;                  // true once driver has been initialised
    uint32_t _actuator_position_desired; // desired actuator position (set from within update method)
    uint32_t _last_send_actuator_ms;    // system time (in millis) last actuator position command was sent (used for health reporting)
    uint32_t _send_start_us;            // system time (in micros) when last message started being sent (used for timing to unset DE pin)
    uint32_t _send_delay_us;            // delay (in micros) to allow bytes to be sent after which pin can be unset.  0 if not delaying

    // health reporting
    HAL_Semaphore _last_healthy_sem;// semaphore protecting reading and updating of _last_send_actuator_ms and _last_received_ms


    // message parsing members
    uint32_t _last_received_ms;         // system time (in millis) last message was received

    // reply message handling
    uint8_t _reply_msgid;           // replies expected msgid (reply often does not specify the msgid so we must record it)
    uint32_t _reply_wait_start_ms;  // system time that we started waiting for a reply message


    static AP_IrisOrca *_singleton;     // singleton instance

};

namespace AP {
    AP_IrisOrca *irisorca();
};

#endif // HAL_IRISORCA_ENABLED
