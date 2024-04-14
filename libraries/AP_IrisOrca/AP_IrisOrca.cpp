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

#include "AP_IrisOrca.h"

#if HAL_IRISORCA_ENABLED

#include <AP_Common/AP_Common.h>
//#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
//#include <AP_Logger/AP_Logger.h>
//#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define IRISORCA_SERIAL_BAUD        19200   // communication is always at 19200
#define IRISORCA_SERIAL_PARITY      2       // communication is always even parity
#define IRISORCA_LOG_ORCA_INTERVAL_MS                      5000// log ORCA message at this interval in milliseconds
#define IRISORCA_SEND_ACTUATOR_POSITION_INTERVAL_MS        100 // actuator position sent at 10hz if connected to actuator
#define IRISORCA_SEND_ACTUATOR_STATUS_REQUEST_INTERVAL_MS  400 // actuator status requested every 0.4sec if connected to actuator
#define IRISORCA_SEND_ACTUATOR_PARAM_REQUEST_INTERVAL_MS   400 // actuator param requested every 0.4sec if connected to actuator
#define IRISORCA_REPLY_TIMEOUT_MS                          25      // stop waiting for replies after 25ms
#define IRISORCA_ERROR_REPORT_INTERVAL_MAX_MS              10000   // errors reported to user at no less than once every 10 seconds

#define HIGHWORD(x) ((uint16_t)((x) >> 16))
#define LOWWORD(x) ((uint16_t)(x))

extern const AP_HAL::HAL& hal;

// parameters
const AP_Param::GroupInfo AP_IrisOrca::var_info[] = {

    // @Param: DE_PIN
    // @DisplayName: Iris Orca DE pin
    // @Description: Pin number connected to RS485 to Serial converter's DE pin. -1 to use serial port's CTS pin if available
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("DE_PIN", 1, AP_IrisOrca, _pin_de, -1),

    // @Param: MAX_TRAVEL_MM
    // @DisplayName: Iris Orca shaft max travel distance
    // @Description: Iris Orca travel distance as measured from the zero position, which will be at one end of the actuator after zeroing.
    // @Units: mm
    // @Range: 0 300
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_TRAVEL_MM", 2, AP_IrisOrca, _max_travel_mm, 261),

    AP_GROUPEND
};

AP_IrisOrca::AP_IrisOrca()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_IrisOrca::init()
{
    // only init once
    // Note: a race condition exists here if init is called multiple times quickly before thread_main has a chance to set _initialise
    if (_initialised) {
        return;
    }

    // create background thread to process serial input and output
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_IrisOrca::thread_main, void), "irisorca", 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        return;
    }
}

// initialise serial port (run from background thread)
bool AP_IrisOrca::init_internals()
{
    // find serial driver and initialise
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IrisOrca, 0);
    if (_uart == nullptr) {
        return false;
    }
    _uart->begin(IRISORCA_SERIAL_BAUD);
    _uart->configure_parity(IRISORCA_SERIAL_PARITY);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_unbuffered_writes(true);

    // initialise RS485 DE pin (when high, allows send to actuator)
    if (_pin_de > -1) {
        hal.gpio->pinMode(_pin_de, HAL_GPIO_OUTPUT);
        hal.gpio->write(_pin_de, 0);
    } else {
        _uart->set_CTS_pin(false);
    }

    return true;
}

// consume incoming messages from actuator, reply with latest actuator speed
// runs in background thread
void AP_IrisOrca::thread_main()
{
    // initialisation
    if (!init_internals()) {
        return;
    }
    _initialised = true;

    while (true) {
        // 1ms loop delay
        hal.scheduler->delay_microseconds(1000);

        // check if transmit pin should be unset
        check_for_send_end();

        // check for timeout waiting for reply
        check_for_reply_timeout();

        // parse incoming characters
        uint32_t nbytes = MIN(_uart->available(), 1024U);
        while (nbytes-- > 0) {
            int16_t b = _uart->read();
            if (b >= 0 ) {
                // clear wait-for-reply because if we are waiting for a reply, this message must be it
                set_reply_received();
                }
            }
        }

        // send actuator position command
        //bool log_update = false;
        if (safe_to_send()) {
            uint32_t now_ms = AP_HAL::millis();

            if (now_ms - _last_send_actuator_ms > IRISORCA_SEND_ACTUATOR_POSITION_INTERVAL_MS) {
                send_actuator_position_cmd();
                //log_update = true;
            }
        }

        // log high level status and actuator position
        //log_ORCA(log_update);
}

// returns true if communicating with the actuator
bool AP_IrisOrca::healthy()
{
    if (!_initialised) {
        return false;
    }
    {
        // healthy if both receive and send have occurred in the last 3 seconds
        WITH_SEMAPHORE(_last_healthy_sem);
        const uint32_t now_ms = AP_HAL::millis();
        return ((now_ms - _last_received_ms < 3000) && (now_ms - _last_send_actuator_ms < 3000));
    }
}

// set DE Serial CTS pin to enable sending commands to actuator
void AP_IrisOrca::send_start()
{
    // set gpio pin or serial port's CTS pin
    if (_pin_de > -1) {
        hal.gpio->write(_pin_de, 1);
    } else {
        _uart->set_CTS_pin(true);
    }
}

// check for timeout after sending and unset pin if required
void AP_IrisOrca::check_for_send_end()
{
    if (_send_delay_us == 0) {
        // not sending
        return;
    }

    if (AP_HAL::micros() - _send_start_us < _send_delay_us) {
        // return if delay has not yet elapsed
        return;
    }
    _send_delay_us = 0;

    // unset gpio or serial port's CTS pin
    if (_pin_de > -1) {
        hal.gpio->write(_pin_de, 0);
    } else {
        _uart->set_CTS_pin(false);
    }
}

// calculate delay require to allow bytes to be sent
uint32_t AP_IrisOrca::calc_send_delay_us(uint8_t num_bytes)
{
    // baud rate of 19200 bits/sec
    // total number of bits = 10 x num_bytes
    // convert from seconds to micros by multiplying by 1,000,000
    // plus additional 300us safety margin
    const uint32_t delay_us = 1e6 * num_bytes * 10 / IRISORCA_SERIAL_BAUD + 300;
    return delay_us;
}


// record msgid of message to wait for and set timer for timeout handling
void AP_IrisOrca::set_expected_reply_msgid(uint8_t msg_id)
{
    _reply_msgid = msg_id;
    _reply_wait_start_ms = AP_HAL::millis();
}

// check for timeout waiting for reply message
void AP_IrisOrca::check_for_reply_timeout()
{
    // return immediately if not waiting for reply
    if (_reply_wait_start_ms == 0) {
        return;
    }
    if (AP_HAL::millis() - _reply_wait_start_ms > IRISORCA_REPLY_TIMEOUT_MS) {
        _reply_wait_start_ms = 0;
    }
}

// mark reply received. should be called whenever a message is received regardless of whether we are actually waiting for a reply
void AP_IrisOrca::set_reply_received()
{
    _reply_wait_start_ms = 0;
}


// send a 100/0x64 Motor Command Stream message to the actuator
// returns true on success
bool AP_IrisOrca::send_motor_command_stream(const uint8_t sub_code, const uint32_t data)
{
    // buffer for outgoing message
    uint8_t send_buff[IRISORCA_MESSAGE_LEN_MAX];
    uint8_t send_buff_num_bytes = 0;

    // build message
    send_buff[send_buff_num_bytes++] = static_cast<uint8_t>(MsgAddress::BUS_MASTER);
    send_buff[send_buff_num_bytes++] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[send_buff_num_bytes++] = 0x64; // Function Code: Motor Command Stream
    send_buff[send_buff_num_bytes++] = sub_code;
    // data is 32 bits - send as 4 bytes
    send_buff[send_buff_num_bytes++] = HIGHBYTE(HIGHWORD(data));
    send_buff[send_buff_num_bytes++] = LOWBYTE(HIGHWORD(data));
    send_buff[send_buff_num_bytes++] = HIGHBYTE(LOWWORD(data));
    send_buff[send_buff_num_bytes++] = LOWBYTE(LOWWORD(data));

    // Add Modbus CRC-16
    uint16_t crc = calc_crc_modbus(send_buff, send_buff_num_bytes);

    send_buff[send_buff_num_bytes++] = LOWBYTE(crc);
    send_buff[send_buff_num_bytes++] = HIGHBYTE(crc);

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, send_buff_num_bytes);

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(send_buff_num_bytes);

    return true;
}

// send a actuator speed position command as a value from 0 to max_travel_mm
void AP_IrisOrca::send_actuator_position_cmd()
{
    // calculate desired actuator position

    // if not armed, set to mid point
    if (!hal.util->get_soft_armed()) {
        _actuator_position_desired = (uint32_t) (_max_travel_mm * 1000 / 2);
    } else {
        // convert steering output to actuator output in range 0 to _max_travel_mm * 1000
        // ToDo: convert PWM output to actuator output so that SERVOx_MIN, MAX and TRIM take effect
        _actuator_position_desired = constrain_uint32(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_steering) * _max_travel_mm * 1000 / 2, 0, _max_travel_mm * 1000);
    }

    // send a message
    if (send_motor_command_stream((uint8_t)MotorCommandStreamSubCode::POSITION_CONTROL_STREAM, _actuator_position_desired)){
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_actuator_ms = AP_HAL::millis();
    }
}

// get the AP_IrisOrca singleton
AP_IrisOrca *AP_IrisOrca::get_singleton()
{
    return _singleton;
}

AP_IrisOrca *AP_IrisOrca::_singleton = nullptr;

namespace AP {
AP_IrisOrca *irisorca()
{
    return AP_IrisOrca::get_singleton();
}
};

#endif // HAL_IRISORCA_ENABLED
