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

// #if HAL_IRISORCA_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
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

namespace orca {

bool parse_write_register(uint8_t *rcvd_buff, uint8_t buff_len,
                          ActuatorState &state) {
  if (buff_len < WRITE_REG_MSG_RSP_LEN) {
    return false;
  }

  // Switch on the register address (bytes 2 and 3)
  switch ((rcvd_buff[WriteRegRsp::Idx::REG_ADDR_HI] << 8) |
          rcvd_buff[WriteRegRsp::Idx::REG_ADDR_LO]) {
    case static_cast<uint16_t>(Register::CTRL_REG_3):
      // Mode of operation was set
      state.mode =
          static_cast<OperatingMode>(rcvd_buff[WriteRegRsp::Idx::DATA_LO]);
      break;
    default:
      GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                    "IrisOrca: Unsupported write register.");
      return false;
  }

  return true;
}

bool parse_motor_command_stream(uint8_t *rcvd_buff, uint8_t buff_len,
                                ActuatorState &state) {
  if (buff_len < MOTOR_COMMAND_STREAM_MSG_RSP_LEN) {
    return false;
  }

  state.shaft_position =
      u32_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::POSITION_MSB_HI);
  state.force_realized =
      u32_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::FORCE_MSB_HI);
  state.power_consumed =
      u16_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::POWER_HI);
  state.temperature = rcvd_buff[MotorCommandStreamRsp::Idx::TEMP];
  state.voltage =
      u16_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::VOLTAGE_HI);
  state.errors = u16_from_be(rcvd_buff, MotorCommandStreamRsp::Idx::ERROR_HI);

  return true;
}

bool parse_motor_read_stream(uint8_t *rcvd_buff, uint8_t buff_len,
                             ActuatorState &state) {
  if (buff_len < MOTOR_READ_STREAM_MSG_RSP_LEN) {
    return false;
  }
  // Ignore the read register value and set the other state members
  state.mode =
      static_cast<OperatingMode>(rcvd_buff[MotorReadStreamRsp::Idx::MODE]);
  state.shaft_position =
      u32_from_be(rcvd_buff, MotorReadStreamRsp::Idx::POSITION_MSB_HI);
  state.force_realized =
      u32_from_be(rcvd_buff, MotorReadStreamRsp::Idx::FORCE_MSB_HI);
  state.power_consumed =
      u16_from_be(rcvd_buff, MotorReadStreamRsp::Idx::POWER_HI);
  state.temperature = rcvd_buff[MotorReadStreamRsp::Idx::TEMP];
  state.voltage = u16_from_be(rcvd_buff, MotorReadStreamRsp::Idx::VOLTAGE_HI);
  state.errors = u16_from_be(rcvd_buff, MotorReadStreamRsp::Idx::ERROR_HI);

  return true;
}

void add_crc_modbus(uint8_t *buff, uint8_t len) {
  uint16_t crc = calc_crc_modbus(buff, len);
  buff[len] = (uint8_t)(crc & 0xFF);
  buff[len + 1] = (uint8_t)((crc >> 8) & 0xFF);
}

}  // namespace orca

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

    // @Param: MAX_TRAVEL
    // @DisplayName: Shaft max physical travel distance
    // @Description: The max physical travel distance as measured from the zero position, which will be at one end of the actuator after zeroing.
    // @Units: mm
    // @Range: 0 300
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_TRAVEL", 2, AP_IrisOrca, _max_travel_mm, 261),

    // @Param: PAD_TRAVEL
    // @DisplayName: Pad travel distance
    // @Description: Amount to pad the physical travel distance by to ensure the actuator does not reach the physical end stops during normal motion.
    // @Units: mm
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PAD_TRAVEL", 3, AP_IrisOrca, _pad_travel_mm, 10),

    // @Param: REVERSE_DIR
    // @DisplayName: Reverse direction
    // @Description: Reverse the direction of the actuator
    // @Values: 0:Normal,1:Reverse
    // @User: Standard
    AP_GROUPINFO("REVERSE_DIR", 3, AP_IrisOrca, _reverse_direction, 0),

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

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Initialized");

    _control_state = orca::MotorControlState::AUTO_ZERO;
    bool waiting_for_auto_zero = false;

    while (true)
    {
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
                if (parse_byte((uint8_t)b)) {
                    // complete message received, parse it!
                    parse_message();
                    // clear wait-for-reply because if we are waiting for a reply, this message must be it
                    set_reply_received();
                }
            }
        }

        // send a single command depending on the control state
        // or send a sleep command if there is an active error
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _last_send_actuator_ms > IRISORCA_SEND_ACTUATOR_POSITION_INTERVAL_MS)
        {
            if (_actuator_state.errors != 0) {
                // send sleep command if in error state to attempt to clear the error
                send_actuator_sleep_cmd();
                if (now_ms - _last_error_report_ms > IRISORCA_ERROR_REPORT_INTERVAL_MAX_MS) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IrisOrca: Error %i", _actuator_state.errors);
                    _last_error_report_ms = now_ms;
                }
                continue;
            }
            
            // no errors - execute per control state
            switch (_control_state)
            {
                case orca::MotorControlState::AUTO_ZERO:
                    // Auto-zero mode is initiated by sending a write register command to the actuator with the 
                    // mode set to AUTO_ZERO. The actuator will then transition to AUTO_ZERO mode and will exit this mode
                    // to another mode (either SLEEP or POSITION) when the zero position is found.
                    if (!waiting_for_auto_zero) {
                        if (_actuator_state.mode == orca::OperatingMode::AUTO_ZERO) {
                            // Capture the entry into auto-zero mode
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto-zero started");
                            waiting_for_auto_zero = true;
                        }
                        else if (safe_to_send()) {
                            // Initiate auto-zero mode
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto-zero commanded");
                            send_auto_zero_mode_cmd();
                        }
                    }
                    else {
                        // waiting for auto-zero to complete
                        if (_actuator_state.mode != orca::OperatingMode::AUTO_ZERO) {
                            // was auto-zeroing and has exited to another mode, therefore auto-zeroing is complete
                            waiting_for_auto_zero = false;
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Auto-zero complete");
                            _control_state = orca::MotorControlState::POSITION_CONTROL;
                        }
                        else if (safe_to_send()) {
                            // read the mode of operation to check if auto-zero is complete
                            send_actuator_status_request();
                        }
                    }
                    break;
                    
                case orca::MotorControlState::POSITION_CONTROL:
                    // Send a position control command
                    if (safe_to_send()) {
                        send_actuator_position_cmd();
                    }
                    break;

                default:
                    break;
            }
        }
    }
}

// returns true if communicating with the actuator
bool AP_IrisOrca::healthy()
{
    if (!_initialised) {
        return false;
    }
    
    // healthy if both receive and send have occurred in the last 3 seconds
    WITH_SEMAPHORE(_last_healthy_sem);
    const uint32_t now_ms = AP_HAL::millis();
    return ((now_ms - _last_received_ms < 3000) && (now_ms - _last_send_actuator_ms < 3000));
    
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
    // total number of bits = 10 x num_bytes (no parity)
    // or 11 x num_bytes (with parity)
    // convert from seconds to micros by multiplying by 1,000,000
    // plus additional 300us safety margin
    uint8_t parity = IRISORCA_SERIAL_PARITY == 0 ? 0 : 1;
    uint8_t bits_per_data_byte = 10 + parity;
    const uint32_t delay_us = 1e6 * num_bytes * bits_per_data_byte / IRISORCA_SERIAL_BAUD + 300;
    return delay_us;
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

// send a 0x06 Write Register message to the actuator
// returns true on success
bool AP_IrisOrca::write_register(uint16_t reg_addr, uint16_t reg_value)
{
    using namespace orca;
    // buffer for outgoing message
    uint8_t send_buff[WRITE_REG_MSG_LEN];

    // set expected reply message length
    _reply_msg_len = WRITE_REG_MSG_RSP_LEN;

    // build message
    send_buff[WriteReg::Idx::DEVICE_ADDR] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[WriteReg::Idx::FUNCTION_CODE] = static_cast<uint8_t>(FunctionCode::WRITE_REGISTER);
    send_buff[WriteReg::Idx::REG_ADDR_HI] = HIGHBYTE(reg_addr);
    send_buff[WriteReg::Idx::REG_ADDR_LO] = LOWBYTE(reg_addr);
    send_buff[WriteReg::Idx::WRITE_DATA_HI] = HIGHBYTE(reg_value);
    send_buff[WriteReg::Idx::WRITE_DATA_LO] = LOWBYTE(reg_value);

    // Add Modbus CRC-16
    orca::add_crc_modbus(send_buff, WRITE_REG_MSG_LEN - CRC_LEN);

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, sizeof(send_buff));


    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(send_buff));

    _reply_wait_start_ms = AP_HAL::millis();

    return true;
}

// send a 100/0x64 Motor Command Stream message to the actuator
// returns true on success
bool AP_IrisOrca::write_motor_command_stream(const uint8_t sub_code, const uint32_t data)
{
    using namespace orca;
    // buffer for outgoing message
    uint8_t send_buff[MOTOR_COMMAND_STREAM_MSG_LEN];

    // set expected reply message length
    _reply_msg_len = MOTOR_COMMAND_STREAM_MSG_RSP_LEN;

    // build message
    send_buff[MotorCommandStream::Idx::DEVICE_ADDR] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[MotorCommandStream::Idx::FUNCTION_CODE] = static_cast<uint8_t>(FunctionCode::MOTOR_COMMAND_STREAM);
    send_buff[MotorCommandStream::Idx::SUB_CODE] = sub_code;
    // data is 32 bits - send as 4 bytes
    send_buff[MotorCommandStream::Idx::DATA_MSB_HI] = HIGHBYTE(HIGHWORD(data));
    send_buff[MotorCommandStream::Idx::DATA_MSB_LO] = LOWBYTE (HIGHWORD(data));
    send_buff[MotorCommandStream::Idx::DATA_LSB_HI] = HIGHBYTE(LOWWORD(data));
    send_buff[MotorCommandStream::Idx::DATA_LSB_LO] = LOWBYTE (LOWWORD(data));

    // Add Modbus CRC-16
    orca::add_crc_modbus(send_buff, MOTOR_COMMAND_STREAM_MSG_LEN - CRC_LEN);

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, sizeof(send_buff));

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(send_buff));

    _reply_wait_start_ms = AP_HAL::millis();

    return true;
}

// send a 0x68 Motor Read Stream message to the actuator
// returns true on success
bool AP_IrisOrca::write_motor_read_stream(const uint16_t reg_addr, const uint8_t reg_width)
{
    using namespace orca;
    // buffer for outgoing message
    uint8_t send_buff[MOTOR_READ_STREAM_MSG_LEN];

    // set expected reply message length
    _reply_msg_len = MOTOR_READ_STREAM_MSG_RSP_LEN;

    // build message
    send_buff[MotorReadStream::Idx::DEVICE_ADDR] = static_cast<uint8_t>(MsgAddress::DEVICE);
    send_buff[MotorReadStream::Idx::FUNCTION_CODE] = static_cast<uint8_t>(FunctionCode::MOTOR_READ_STREAM);
    send_buff[MotorReadStream::Idx::REG_ADDR_HI] = HIGHBYTE(reg_addr);
    send_buff[MotorReadStream::Idx::REG_ADDR_LO] = LOWBYTE(reg_addr);
    send_buff[MotorReadStream::Idx::REG_WIDTH] = reg_width;

    // Add Modbus CRC-16
    orca::add_crc_modbus(send_buff, MOTOR_READ_STREAM_MSG_LEN - CRC_LEN);

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, sizeof(send_buff));

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(sizeof(send_buff));

    _reply_wait_start_ms = AP_HAL::millis();

    return true;
}

// perform an auto-zero by sending an auto zero command
// and waiting for the actuator to complete the zeroing process
// (transition to a mode other than auto-zero)
// returns true on success
void AP_IrisOrca::send_auto_zero_mode_cmd()
{
    // send a message
    if (write_register((uint16_t)orca::Register::CTRL_REG_3,
                       static_cast<uint8_t>(orca::OperatingMode::AUTO_ZERO))) {
      // record time of send for health reporting
      WITH_SEMAPHORE(_last_healthy_sem);
      _last_send_actuator_ms = AP_HAL::millis();
    }
}

// send an actuator speed position command as a value from 0 to max_travel_mm
void AP_IrisOrca::send_actuator_position_cmd()
{
    // convert yaw output to actuator output in range _pad_travel_mm to _max_travel_mm - _pad_travel_mm
    _actuator_position_desired = constrain_uint32(
        (SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_steering) + 1) * _max_travel_mm * 0.5 * 1000, 
        _pad_travel_mm * 1000, _max_travel_mm * 1000 - (_pad_travel_mm * 1000));
    // reverse direction if required
    if (_reverse_direction) {
        _actuator_position_desired = _max_travel_mm * 1000 - _actuator_position_desired;
    }

    // send message
    if (write_motor_command_stream((uint8_t)orca::MotorCommandStreamSubCode::POSITION_CONTROL_STREAM, 
        _actuator_position_desired)){
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_actuator_ms = AP_HAL::millis();
    }
}

// send an actuator sleep command
void AP_IrisOrca::send_actuator_sleep_cmd()
{
    // send message
    if (write_motor_command_stream((uint8_t)orca::MotorCommandStreamSubCode::SLEEP_DATA_STREAM, 0)){
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_actuator_ms = AP_HAL::millis();

        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IrisOrca: Actuator sleep command sent");
    }
}

// send a request for actuator status
void AP_IrisOrca::send_actuator_status_request()
{
    // send message
    if (write_motor_read_stream((uint16_t)orca::Register::CTRL_REG_3, 1)){
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_actuator_ms = AP_HAL::millis();
    }
}

// process a single byte received on serial port
// return true if a complete message has been received (the message will be held in _received_buff)
bool AP_IrisOrca::parse_byte(uint8_t b)
{
    bool complete_msg_received = false;

    // add b to buffer

    _received_buff[_received_buff_len] = b;
    _received_buff_len++;

    // check for a complete message
    if (_received_buff_len >= _reply_msg_len)
    {
        // check CRC of the message
        uint16_t crc_expected = calc_crc_modbus(_received_buff, _received_buff_len - orca::CRC_LEN);
        uint16_t crc_received = (_received_buff[_received_buff_len - 2]) | (_received_buff[_received_buff_len - 1] << 8);
        if (crc_expected != crc_received) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IrisOrca: CRC error");
            _received_buff_len = 0;
            _parse_error_count++;
        } 
        else {
            // CRC is correct - reset buffer and set flag
            _received_buff_len = 0;
            _parse_success_count++;
            // record time of receive for health reporting
            WITH_SEMAPHORE(_last_healthy_sem);
            _last_received_ms = AP_HAL::millis();
            complete_msg_received = true;
        }
    } 
    return complete_msg_received;
}

// process message held in _received_buff
// return true if there are no errors and the message is as expected
bool AP_IrisOrca::parse_message()
{
    // check for expected reply
    switch (static_cast<orca::FunctionCode>(_received_buff[1])) 
    {
        case orca::FunctionCode::WRITE_REGISTER:
            return parse_write_register(_received_buff, _reply_msg_len, _actuator_state);
        case orca::FunctionCode::MOTOR_COMMAND_STREAM:
            return parse_motor_command_stream(_received_buff, _reply_msg_len, _actuator_state);
        case orca::FunctionCode::MOTOR_READ_STREAM:
            return parse_motor_read_stream(_received_buff, _reply_msg_len, _actuator_state);
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IrisOrca: Unexpected message");
            return false;
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

// #endif // HAL_IRISORCA_ENABLED
