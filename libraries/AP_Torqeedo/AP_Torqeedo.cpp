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

#include "AP_Torqeedo.h"

#if HAL_TORQEEDO_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#define TORQEEDO_SERIAL_BAUD        19200   // communication is always at 19200
#define TORQEEDO_PACKET_HEADER      0xAC    // communication packet header
#define TORQEEDO_PACKET_FOOTER      0xAD    // communication packer footer
#define TORQEEDO_LOG_INTERVAL_MS    5000    // log debug info at this interval in milliseconds
#define TORQEEDO_SEND_MOTOR_SPEED_INTERVAL_US   100000  // motor speed sent at 10hz if connected to motor

extern const AP_HAL::HAL& hal;

// parameters
const AP_Param::GroupInfo AP_Torqeedo::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Torqeedo connection type
    // @Description: Torqeedo connection type
    // @Values: 0:Disabled, 1:Tiller, 2:Motor
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_Torqeedo, _type, (int8_t)ConnectionType::TYPE_DISABLED, AP_PARAM_FLAG_ENABLE),

    // @Param: ONOFF_PIN
    // @DisplayName: Torqeedo ON/Off pin
    // @Description: Pin number connected to Torqeedo's on/off pin. -1 to use serial port's RTS pin if available
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ONOFF_PIN", 2, AP_Torqeedo, _pin_onoff, -1),

    // @Param: DE_PIN
    // @DisplayName: Torqeedo DE pin
    // @Description: Pin number connected to RS485 to Serial converter's DE pin. -1 to use serial port's CTS pin if available
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("DE_PIN", 3, AP_Torqeedo, _pin_de, -1),

    // @Param: OPTIONS
    // @DisplayName: Torqeedo Options
    // @Description: Torqeedo Options Bitmask
    // @Bitmask: 0:Log,1:Send debug to GCS
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 4, AP_Torqeedo, _options, (int8_t)options::LOG),

    // @Param: POWER
    // @DisplayName: Torqeedo Motor Power
    // @Description: Torqeedo motor power.  Only applied when using motor connection type (e.g. TRQD_TYPE=2)
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POWER", 5, AP_Torqeedo, _motor_power, 100),

    // @Param: SLEW_TIME
    // @DisplayName: Torqeedo Slew Time
    // @Description: Torqeedo slew rate specified as the minimum number of seconds required to increase the throttle from 0 to 100%.  A value of zero disables the limit
    // @Units: s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SLEW_TIME", 6, AP_Torqeedo, _slew_time, 2.0),

    // @Param: DIR_DELAY
    // @DisplayName: Torqeedo Direction Change Delay
    // @Description: Torqeedo direction change delay.  Output will remain at zero for this many seconds when transitioning between forward and backwards rotation
    // @Units: s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DIR_DELAY", 7, AP_Torqeedo, _dir_delay, 1.0),

    AP_GROUPEND
};

AP_Torqeedo::AP_Torqeedo()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise driver
void AP_Torqeedo::init()
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // only init once
    // Note: a race condition exists here if init is called multiple times quickly before thread_main has a chance to set _initialise
    if (_initialised) {
        return;
    }

    // create background thread to process serial input and output
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Torqeedo::thread_main, void), "torqeedo", 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        return;
    }
}

// initialise serial port and gpio pins (run from background thread)
bool AP_Torqeedo::init_internals()
{
    // find serial driver and initialise
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Torqeedo, 0);
    if (_uart == nullptr) {
        return false;
    }
    _uart->begin(TORQEEDO_SERIAL_BAUD);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_unbuffered_writes(true);

    // if using tiller connection set on/off pin for 0.5 sec to turn on battery
    if (_type == ConnectionType::TYPE_TILLER) {
        if (_pin_onoff > -1) {
            hal.gpio->pinMode(_pin_onoff, HAL_GPIO_OUTPUT);
            hal.gpio->write(_pin_onoff, 1);
            hal.scheduler->delay(500);
            hal.gpio->write(_pin_onoff, 0);
        } else {
            // use serial port's RTS pin to turn on battery
            _uart->set_RTS_pin(true);
            hal.scheduler->delay(500);
            _uart->set_RTS_pin(false);
        }
    }

    // initialise RS485 DE pin (when high, allows send to motor)
    if (_pin_de > -1) {
        hal.gpio->pinMode(_pin_de, HAL_GPIO_OUTPUT);
        hal.gpio->write(_pin_de, 0);
    } else {
        _uart->set_CTS_pin(false);
    }

    return true;
}

// returns true if the driver is enabled
bool AP_Torqeedo::enabled() const
{
    switch ((ConnectionType)_type) {
    case ConnectionType::TYPE_DISABLED:
        return false;
    case ConnectionType::TYPE_TILLER:
    case ConnectionType::TYPE_MOTOR:
        return true;
    }

    return false;
}

// consume incoming messages from motor, reply with latest motor speed
// runs in background thread
void AP_Torqeedo::thread_main()
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

        // parse incoming characters
        uint32_t nbytes = MIN(_uart->available(), 1024U);
        while (nbytes-- > 0) {
            int16_t b = _uart->read();
            if (b >= 0 ) {
                if (parse_byte((uint8_t)b)) {
                    // complete message received, check message id
                    const MsgId msg_id = (MsgId)_received_buff[0];
                    if ((msg_id == MsgId::REQUEST_MOTOR_SPEED) &&  (_type == ConnectionType::TYPE_TILLER)) {
                        // request received to send updated motor speed
                        _send_motor_speed = true;
                    }
                }
            }
        }

        // send motor speed
        bool log_update = false;
        if (safe_to_send()) {
            // if connected to motor send motor speed every 0.5sec
            if (_type == ConnectionType::TYPE_MOTOR &&
                (AP_HAL::micros() - _last_send_motor_us > TORQEEDO_SEND_MOTOR_SPEED_INTERVAL_US)) {
                _send_motor_speed = true;
            }

            // send motor speed
            if (_send_motor_speed) {
                send_motor_speed_cmd();
                _send_motor_speed = false;
                log_update = true;
            }
        }

        // logging and debug output
        log_and_debug(log_update);
    }
}

// returns true if communicating with the motor
bool AP_Torqeedo::healthy()
{
    if (!_initialised) {
        return false;
    }
    {
        // healthy if both receive and send have occurred in the last 3 seconds
        WITH_SEMAPHORE(_last_healthy_sem);
        const uint32_t now_ms = AP_HAL::millis();
        return ((now_ms - _last_received_ms < 3000) && (now_ms - _last_send_motor_ms < 3000));
    }
}

// run pre-arm check.  returns false on failure and fills in failure_msg
// any failure_msg returned will not include a prefix
bool AP_Torqeedo::pre_arm_checks(char *failure_msg, uint8_t failure_msg_len)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return true;
    }

    if (!_initialised) {
        strncpy(failure_msg, "not initialised", failure_msg_len);
        return false;
    }
    if (!healthy()) {
        strncpy(failure_msg, "not healthy", failure_msg_len);
        return false;
    }
    return true;
}

// process a single byte received on serial port
// return true if a complete message has been received (the message will be held in _received_buff)
bool AP_Torqeedo::parse_byte(uint8_t b)
{
    bool complete_msg_received = false;

    switch (_parse_state) {
    case ParseState::WAITING_FOR_HEADER:
        if (b == TORQEEDO_PACKET_HEADER) {
            _parse_state = ParseState::WAITING_FOR_FOOTER;
        }
        _received_buff_len = 0;
        break;
    case ParseState::WAITING_FOR_FOOTER:
        if (b == TORQEEDO_PACKET_FOOTER) {
            _parse_state = ParseState::WAITING_FOR_HEADER;

            // check message length
            if (_received_buff_len == 0) {
                _parse_error_count++;
                break;
            }

            // check crc
            const uint8_t crc_expected = crc8_maxim(_received_buff, _received_buff_len-1);
            if (_received_buff[_received_buff_len-1] != crc_expected) {
                _parse_error_count++;
                break;
            }
            _parse_success_count++;
            {
                // record time of successful receive for health reporting
                WITH_SEMAPHORE(_last_healthy_sem);
                _last_received_ms = AP_HAL::millis();
            }
            complete_msg_received = true;
        } else {
            // add to buffer
            _received_buff[_received_buff_len] = b;
            _received_buff_len++;
            if (_received_buff_len > TORQEEDO_MESSAGE_LEN_MAX) {
                // message too long
                _parse_state = ParseState::WAITING_FOR_HEADER;
                _parse_error_count++;
            }
        }
        break;
    }

    return complete_msg_received;
}

// set DE Serial CTS pin to enable sending commands to motor
void AP_Torqeedo::send_start()
{
    // set gpio pin or serial port's CTS pin
    if (_pin_de > -1) {
        hal.gpio->write(_pin_de, 1);
    } else {
        _uart->set_CTS_pin(true);
    }
}

// check for timeout after sending and unset pin if required
void AP_Torqeedo::check_for_send_end()
{
    if (_send_delay_us == 0) {
        // not sending
        return;
    }

    if (AP_HAL::micros() - _last_send_motor_us < _send_delay_us) {
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
uint32_t AP_Torqeedo::calc_send_delay_us(uint8_t num_bytes)
{
    // baud rate of 19200 bits/sec
    // total number of bits = 10 x num_bytes
    // convert from seconds to micros by multiplying by 1,000,000
    // plus additional 300us safety margin
    const uint32_t delay_us = 1e6 * num_bytes * 10 / TORQEEDO_SERIAL_BAUD + 300;
    return delay_us;
}

// format of motor speed command packet
//
// Data Byte    Field Definition    Example Value   Comments
// ---------------------------------------------------------------------------------
// byte 0       Header              0xAC
// byte 1       SourceId?           0x00            0 = tiller?
// byte 2       Destination ID?     0x00            0 = all?
// byte 3       Command Id?         0x05            0=Stop? 4=Don'tTurn? 5=Turn?
// byte 4       Command Id?         0x00            0x20 if byte3=4, 0x0 is byte3=5
// byte 5       Motor Speed MSB     ----            Motor Speed MSB (-1000 to +1000)
// byte 6       Motor Speed LSB     ----            Motor Speed LSB (-1000 to +1000)
// byte 7       CRC-Maxim           ----            CRC-Maxim value
// byte 8       Footer              0xAD
//
// example message when rotating forwards:  "AC 00 00 05 00 00 ED 95 AD"    (+237)
// example message when rotating backwards: "AC 00 00 05 00 FF AE 2C 0C AD" (-82)


// send a motor speed command as a value from -1000 to +1000
// value is taken directly from SRV_Channel
void AP_Torqeedo::send_motor_speed_cmd()
{
    // calculate desired motor speed
    if (!hal.util->get_soft_armed()) {
        _motor_speed_desired = 0;
    } else {
        // convert throttle output to motor output in range -1000 to +1000
        // ToDo: convert PWM output to motor output so that SERVOx_MIN, MAX and TRIM take effect
        _motor_speed_desired = constrain_int16(SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t::k_throttle) * 1000.0, -1000, 1000);
    }

    // updated limited motor speed
    int16_t mot_speed_limited = calc_motor_speed_limited(_motor_speed_desired);

    // set send pin
    send_start();

    // by default use tiller connection command
    uint8_t mot_speed_cmd_buff[] = {TORQEEDO_PACKET_HEADER, (uint8_t)MsgId::SET_MOTOR_SPEED, 0x0, 0x5, 0x0, HIGHBYTE(mot_speed_limited), LOWBYTE(mot_speed_limited), 0x0, TORQEEDO_PACKET_FOOTER};
    uint8_t buff_size = ARRAY_SIZE(mot_speed_cmd_buff);

    // update message if using motor connection
    if (_type == ConnectionType::TYPE_MOTOR) {
        const uint8_t motor_power = (uint8_t)constrain_int16(_motor_power, 0, 100);
        mot_speed_cmd_buff[1] = 0x30;
        mot_speed_cmd_buff[2] = 0x82;
        mot_speed_cmd_buff[3] = mot_speed_limited == 0 ? 0 : 0x1;    // enable motor
        mot_speed_cmd_buff[4] = mot_speed_limited == 0 ? 0 : motor_power;    // motor power from 0 to 100
    }

    // calculate crc and add to buffer
    const uint8_t crc = crc8_maxim(&mot_speed_cmd_buff[1], buff_size-3);
    mot_speed_cmd_buff[buff_size-2] = crc;

    // write message
    _uart->write(mot_speed_cmd_buff, buff_size);

    _last_send_motor_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(buff_size);

    {
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_motor_ms = AP_HAL::millis();
    }

}

// calculate the limited motor speed that is sent to the motors
// desired_motor_speed argument and returned value are in the range -1000 to 1000
int16_t AP_Torqeedo::calc_motor_speed_limited(int16_t desired_motor_speed)
{
    uint32_t now_ms = AP_HAL::millis();

    // update dir_limit flag for forward-reverse transition delay
    const bool dir_delay_active = (_dir_delay > 0);
    if (!dir_delay_active) {
        // allow movement in either direction
        _dir_limit = 0;
    } else {
        // by default limit motor direction to previous iteration's direction
        if (is_positive(_motor_speed_limited)) {
            _dir_limit = 1;
        } else if (is_negative(_motor_speed_limited)) {
            _dir_limit = -1;
        } else {
            // motor speed is zero
            if ((_motor_speed_zero_ms != 0) && ((now_ms - _motor_speed_zero_ms) > (_dir_delay * 1000))) {
                // delay has passed so allow movement in either direction
                _dir_limit = 0;
                _motor_speed_zero_ms = 0;
            }
        }
    }

    // calculate upper and lower limits for forward-reverse transition delay
    int16_t lower_limit = -1000;
    int16_t upper_limit = 1000;
    if (_dir_limit < 0) {
        upper_limit = 0;
    }
    if (_dir_limit > 0) {
        lower_limit = 0;
    }

    // calculate dt since last update
    float dt = (now_ms - _motor_speed_limited_ms) / 1000.0f;
    if (dt > 1.0) {
        // after a long delay limit motor output to zero to avoid sudden starts
        lower_limit = 0;
        upper_limit = 0;
    }
    _motor_speed_limited_ms = now_ms;

    // apply slew limit
    if (_slew_time > 0) {
       const float chg_max = 1000.0 * dt / _slew_time;
       _motor_speed_limited = constrain_float(desired_motor_speed, _motor_speed_limited - chg_max, _motor_speed_limited + chg_max);
    } else {
        // no slew limit
        _motor_speed_limited = desired_motor_speed;
    }

    // apply upper and lower limits
    _motor_speed_limited = constrain_float(_motor_speed_limited, lower_limit, upper_limit);

    // record time motor speed becomes zero
    if (is_zero(_motor_speed_limited)) {
        if (_motor_speed_zero_ms == 0) {
            _motor_speed_zero_ms = now_ms;
        }
    } else {
        // clear timer
        _motor_speed_zero_ms = 0;
    }

    return (int16_t)_motor_speed_limited;
}

// output logging and debug messages (if required)
// force_logging should be true if caller wants to ensure the latest status is logged
void AP_Torqeedo::log_and_debug(bool force_logging)
{
    // exit immediately if options are all unset
    if (_options == 0) {
        return;
    }

    // return if not enough time has passed since last output
    const uint32_t now_ms = AP_HAL::millis();
    if (!force_logging && (now_ms - _last_debug_ms < TORQEEDO_LOG_INTERVAL_MS)) {
        return;
    }
    _last_debug_ms = now_ms;

    const bool health = healthy();
    int16_t actual_motor_speed = get_motor_speed_limited();

    if ((_options & options::LOG) != 0) {
        // @LoggerMessage: TRQD
        // @Description: Torqeedo Status
        // @Field: TimeUS: Time since system startup
        // @Field: Health: Health
        // @Field: DesMotSpeed: Desired Motor Speed (-1000 to 1000)
        // @Field: MotSpeed: Motor Speed (-1000 to 1000)
        // @Field: SuccCnt: Success Count
        // @Field: ErrCnt: Error Count
        AP::logger().Write("TRQD", "TimeUS,Health,DesMotSpeed,MotSpeed,SuccCnt,ErrCnt", "QBhhII",
                           AP_HAL::micros64(),
                           health,
                           _motor_speed_desired,
                           actual_motor_speed,
                           _parse_success_count,
                           _parse_error_count);
    }

    if ((_options & options::DEBUG_TO_GCS) != 0) {
        gcs().send_text(MAV_SEVERITY_INFO,"Trqd h:%u dspd:%d spd:%d succ:%ld err:%ld",
                (unsigned)health,
                (int)_motor_speed_desired,
                (int)actual_motor_speed,
                (unsigned long)_parse_success_count,
                (unsigned long)_parse_error_count);
    }
}

// get the AP_Torqeedo singleton
AP_Torqeedo *AP_Torqeedo::get_singleton()
{
    return _singleton;
}

AP_Torqeedo *AP_Torqeedo::_singleton = nullptr;

namespace AP {
AP_Torqeedo *torqeedo()
{
    return AP_Torqeedo::get_singleton();
}
};

#endif // HAL_TORQEEDO_ENABLED
