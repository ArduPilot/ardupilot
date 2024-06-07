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

#include "AP_Torqeedo_TQBus.h"

#if HAL_TORQEEDO_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#define TORQEEDO_SERIAL_BAUD        19200   // communication is always at 19200
#define TORQEEDO_PACKET_HEADER      0xAC    // communication packet header
#define TORQEEDO_PACKET_FOOTER      0xAD    // communication packet footer
#define TORQEEDO_PACKET_ESCAPE      0xAE    // escape character for handling occurrences of header, footer and this escape bytes in original message
#define TORQEEDO_PACKET_ESCAPE_MASK 0x80    // byte after ESCAPE character should be XOR'd with this value
#define TORQEEDO_LOG_TRQD_INTERVAL_MS                   5000// log TRQD message at this interval in milliseconds
#define TORQEEDO_SEND_MOTOR_SPEED_INTERVAL_MS           100 // motor speed sent at 10hz if connected to motor
#define TORQEEDO_SEND_MOTOR_STATUS_REQUEST_INTERVAL_MS  400 // motor status requested every 0.4sec if connected to motor
#define TORQEEDO_SEND_MOTOR_PARAM_REQUEST_INTERVAL_MS   400 // motor param requested every 0.4sec if connected to motor
#define TORQEEDO_BATT_TIMEOUT_MS    5000    // battery info timeouts after 5 seconds
#define TORQEEDO_REPLY_TIMEOUT_MS   25      // stop waiting for replies after 25ms
#define TORQEEDO_ERROR_REPORT_INTERVAL_MAX_MS   10000   // errors reported to user at no less than once every 10 seconds

extern const AP_HAL::HAL& hal;

// initialise driver
void AP_Torqeedo_TQBus::init()
{
    // only init once
    // Note: a race condition exists here if init is called multiple times quickly before thread_main has a chance to set _initialise
    if (_initialised) {
        return;
    }

    // create background thread to process serial input and output
    char thread_name[15];
    hal.util->snprintf(thread_name, sizeof(thread_name), "torqeedo%u", (unsigned)_instance);
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Torqeedo_TQBus::thread_main, void), thread_name, 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        return;
    }
}

// returns true if communicating with the motor
bool AP_Torqeedo_TQBus::healthy()
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

// initialise serial port and gpio pins (run from background thread)
bool AP_Torqeedo_TQBus::init_internals()
{
    // find serial driver and initialise
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Torqeedo, _instance);
    if (_uart == nullptr) {
        return false;
    }
    _uart->begin(TORQEEDO_SERIAL_BAUD);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_unbuffered_writes(true);

    // if using tiller connection set on/off pin for 0.5 sec to turn on battery
    if (get_type() == AP_Torqeedo::ConnectionType::TYPE_TILLER) {
        if (_params.pin_onoff > -1) {
            hal.gpio->pinMode(_params.pin_onoff, HAL_GPIO_OUTPUT);
            hal.gpio->write(_params.pin_onoff, 1);
            hal.scheduler->delay(500);
            hal.gpio->write(_params.pin_onoff, 0);
        } else {
            // use serial port's RTS pin to turn on battery
            _uart->set_RTS_pin(true);
            hal.scheduler->delay(500);
            _uart->set_RTS_pin(false);
        }
    }

    // initialise RS485 DE pin (when high, allows send to motor)
    if (_params.pin_de > -1) {
        hal.gpio->pinMode(_params.pin_de, HAL_GPIO_OUTPUT);
        hal.gpio->write(_params.pin_de, 0);
    } else {
        _uart->set_CTS_pin(false);
    }

    return true;
}

// consume incoming messages from motor, reply with latest motor speed
// runs in background thread
void AP_Torqeedo_TQBus::thread_main()
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
                if (parse_byte((uint8_t)b)) {
                    // complete message received, parse it!
                    parse_message();
                    // clear wait-for-reply because if we are waiting for a reply, this message must be it
                    set_reply_received();
                }
            }
        }

        // send motor speed
        bool log_update = false;
        if (safe_to_send()) {
            uint32_t now_ms = AP_HAL::millis();

            // if connected to motor
            if (get_type() == AP_Torqeedo::ConnectionType::TYPE_MOTOR) {
                if (now_ms - _last_send_motor_ms > TORQEEDO_SEND_MOTOR_SPEED_INTERVAL_MS) {
                    // send motor speed every 0.1sec
                    _send_motor_speed = true;
                } else if (now_ms - _last_send_motor_status_request_ms > TORQEEDO_SEND_MOTOR_STATUS_REQUEST_INTERVAL_MS) {
                    // send request for motor status
                    send_motor_msg_request(MotorMsgId::STATUS);
                    _last_send_motor_status_request_ms = now_ms;
                } else if (now_ms - _last_send_motor_param_request_ms > TORQEEDO_SEND_MOTOR_PARAM_REQUEST_INTERVAL_MS) {
                    // send request for motor params
                    send_motor_msg_request(MotorMsgId::PARAM);
                    _last_send_motor_param_request_ms = now_ms;
                }
            }

            // send motor speed
            if (_send_motor_speed) {
                send_motor_speed_cmd();
                _send_motor_speed = false;
                log_update = true;
            }
        }

        // log high level status and motor speed
        log_TRQD(log_update);
    }
}

// returns a human-readable string corresponding the passed-in
// master error code (see page 93 of https://media.torqeedo.com/downloads/manuals/torqeedo-Travel-manual-DE-EN.pdf)
// If no conversion is available then nullptr is returned
const char * AP_Torqeedo_TQBus::map_master_error_code_to_string(uint8_t code) const
{
    switch (code) {
    case 2:
        return "stator high temp";
    case 5:
        return "propeller blocked";
    case 6:
        return "motor low voltage";
    case 7:
        return "motor high current";
    case 8:
        return "pcb temp high";
    case 21:
        return "tiller cal bad";
    case 22:
        return "mag bad";
    case 23:
        return "range incorrect";
    case 30:
        return "motor comm error";
    case 32:
        return "tiller comm error";
    case 33:
        return "general comm error";
    case 41:
    case 42:
        return "charge voltage bad";
    case 43:
        return "battery flat";
    case 45:
        return "battery high current";
    case 46:
        return "battery temp error";
    case 48:
        return "charging temp error";
    }

    return nullptr;
}

// report changes in error codes to user
void AP_Torqeedo_TQBus::report_error_codes()
{
    // skip reporting if we have already reported status very recently
    const uint32_t now_ms = AP_HAL::millis();

    // skip reporting if no changes in flags and already reported within 10 seconds
    const bool flags_changed = (_display_system_state_flags_prev.value != _display_system_state.flags.value) ||
                               (_display_system_state_master_error_code_prev != _display_system_state.master_error_code) ||
                               (_motor_status_prev.status_flags_value != _motor_status.status_flags_value) ||
                               (_motor_status_prev.error_flags_value != _motor_status.error_flags_value);
    if (!flags_changed && ((now_ms - _last_error_report_ms) < TORQEEDO_ERROR_REPORT_INTERVAL_MAX_MS)) {
        return;
    }

    // report display system errors
    const char* msg_prefix = "Torqeedo:";
    (void)msg_prefix;  // sometimes unused when HAL_GCS_ENABLED is false
    if (_display_system_state.flags.set_throttle_stop) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s zero throttle required", msg_prefix);
    }
    if (_display_system_state.flags.temp_warning) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s high temp", msg_prefix);
    }
    if (_display_system_state.flags.batt_nearly_empty) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s batt nearly empty", msg_prefix);
    }
    if (_display_system_state.master_error_code > 0) {
        const char *error_string = map_master_error_code_to_string(_display_system_state.master_error_code);
        if (error_string != nullptr) {
            GCS_SEND_TEXT(
                MAV_SEVERITY_CRITICAL, "%s err:%u %s",
                msg_prefix,
                _display_system_state.master_error_code,
                error_string);
        } else {
            GCS_SEND_TEXT(
                MAV_SEVERITY_CRITICAL, "%s err:%u",
                msg_prefix,
                _display_system_state.master_error_code);
        }
    }

    // report motor status errors
    if (_motor_status.error_flags.overcurrent) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s overcurrent", msg_prefix);
    }
    if (_motor_status.error_flags.blocked) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s prop blocked", msg_prefix);
    }
    if (_motor_status.error_flags.overvoltage_static || _motor_status.error_flags.overvoltage_current) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s high voltage", msg_prefix);
    }
    if (_motor_status.error_flags.undervoltage_static || _motor_status.error_flags.undervoltage_current) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s low voltage", msg_prefix);
    }
    if (_motor_status.error_flags.overtemp_motor || _motor_status.error_flags.overtemp_pcb) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s high temp", msg_prefix);
    }
    if (_motor_status.error_flags.timeout_rs485) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s comm timeout", msg_prefix);
    }
    if (_motor_status.error_flags.temp_sensor_error) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s temp sensor err", msg_prefix);
    }
    if (_motor_status.error_flags.tilt) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s tilted", msg_prefix);
    }

    // display OK if all errors cleared
    const bool prev_errored = (_display_system_state_flags_prev.value != 0) ||
                              (_display_system_state_master_error_code_prev != 0) ||
                              (_motor_status_prev.error_flags_value != 0);

    const bool now_errored = (_display_system_state.flags.value != 0) ||
                             (_display_system_state.master_error_code != 0) ||
                             (_motor_status.error_flags_value != 0);

    if (!now_errored && prev_errored) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s OK", msg_prefix);
    }

    // record change in state and reporting time
    _display_system_state_flags_prev.value = _display_system_state.flags.value;
    _display_system_state_master_error_code_prev = _display_system_state.master_error_code;
    _motor_status_prev = _motor_status;
    _last_error_report_ms = now_ms;
}

// get latest battery status info.  returns true on success and populates arguments
bool AP_Torqeedo_TQBus::get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const
{

    // use battery info from display_system_state if available (tiller connection)
    if ((AP_HAL::millis() - _display_system_state.last_update_ms) <= TORQEEDO_BATT_TIMEOUT_MS) {
        voltage = _display_system_state.batt_voltage;
        current_amps = _display_system_state.batt_current;
        temp_C = MAX(_display_system_state.temp_sw, _display_system_state.temp_rp);
        pct_remaining = _display_system_state.batt_charge_pct;
        return true;
    }

    // use battery info from motor_param if available (motor connection)
    if ((AP_HAL::millis() - _motor_param.last_update_ms) <= TORQEEDO_BATT_TIMEOUT_MS) {
        voltage = _motor_param.voltage;
        current_amps = _motor_param.current;
        temp_C = MAX(_motor_param.pcb_temp, _motor_param.stator_temp);
        pct_remaining = 0; // motor does not know percent remaining
        return true;
    }

    return false;
}

// get battery capacity.  returns true on success and populates argument
bool AP_Torqeedo_TQBus::get_batt_capacity_Ah(uint16_t &amp_hours) const
{
    if (_display_system_setup.batt_capacity == 0) {
        return false;
    }
    amp_hours = _display_system_setup.batt_capacity;
    return true;
}

// process a single byte received on serial port
// return true if a complete message has been received (the message will be held in _received_buff)
bool AP_Torqeedo_TQBus::parse_byte(uint8_t b)
{
    bool complete_msg_received = false;

    switch (_parse_state) {
    case ParseState::WAITING_FOR_HEADER:
        if (b == TORQEEDO_PACKET_HEADER) {
            _parse_state = ParseState::WAITING_FOR_FOOTER;
        }
        _received_buff_len = 0;
        _parse_escape_received = false;
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
            // escape character handling
            if (_parse_escape_received) {
                b ^= TORQEEDO_PACKET_ESCAPE_MASK;
                _parse_escape_received = false;
            } else if (b == TORQEEDO_PACKET_ESCAPE) {
                // escape character received, record this and ignore this byte
                _parse_escape_received = true;
                break;
            }
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

// process message held in _received_buff
void AP_Torqeedo_TQBus::parse_message()
{
    // message address (i.e. target of message)
    const MsgAddress msg_addr = (MsgAddress)_received_buff[0];

    // handle messages sent to "remote" (aka tiller)
    if ((get_type() == AP_Torqeedo::ConnectionType::TYPE_TILLER) && (msg_addr == MsgAddress::REMOTE1)) {
        RemoteMsgId msg_id = (RemoteMsgId)_received_buff[1];
        if (msg_id == RemoteMsgId::REMOTE) {
            // request received to send updated motor speed
            _send_motor_speed = true;
        }
        return;
    }

    // handle messages sent to Display
    if ((get_type() == AP_Torqeedo::ConnectionType::TYPE_TILLER) && (msg_addr == MsgAddress::DISPLAY)) {
        DisplayMsgId msg_id = (DisplayMsgId)_received_buff[1];
        switch (msg_id) {
        case DisplayMsgId::SYSTEM_STATE :
            if (_received_buff_len == 30) {
                // fill in _display_system_state
                _display_system_state.flags.value = UINT16_VALUE(_received_buff[2], _received_buff[3]);
                _display_system_state.master_state = _received_buff[4]; // deprecated
                _display_system_state.master_error_code = _received_buff[5];
                _display_system_state.motor_voltage = UINT16_VALUE(_received_buff[6], _received_buff[7]) * 0.01;
                _display_system_state.motor_current = UINT16_VALUE(_received_buff[8], _received_buff[9]) * 0.1;
                _display_system_state.motor_power = UINT16_VALUE(_received_buff[10], _received_buff[11]);
                _display_system_state.motor_rpm = (int16_t)UINT16_VALUE(_received_buff[12], _received_buff[13]);
                _display_system_state.motor_pcb_temp = _received_buff[14];
                _display_system_state.motor_stator_temp = _received_buff[15];
                _display_system_state.batt_charge_pct = _received_buff[16];
                _display_system_state.batt_voltage = UINT16_VALUE(_received_buff[17], _received_buff[18]) * 0.01;
                _display_system_state.batt_current = UINT16_VALUE(_received_buff[19], _received_buff[20]) * 0.1;
                _display_system_state.gps_speed = UINT16_VALUE(_received_buff[21], _received_buff[22]);
                _display_system_state.range_miles = UINT16_VALUE(_received_buff[23], _received_buff[24]);
                _display_system_state.range_minutes = UINT16_VALUE(_received_buff[25], _received_buff[26]);
                _display_system_state.temp_sw = _received_buff[27];
                _display_system_state.temp_rp = _received_buff[28];
                _display_system_state.last_update_ms = AP_HAL::millis();

                // update esc telem sent to ground station
                const uint8_t esc_temp = MAX(_display_system_state.temp_sw, _display_system_state.temp_rp);
                const uint8_t motor_temp = MAX(_display_system_state.motor_pcb_temp, _display_system_state.motor_stator_temp);
                update_esc_telem(_display_system_state.motor_rpm,
                        _display_system_state.motor_voltage,
                        _display_system_state.motor_current,
                        esc_temp,
                        motor_temp);

#if HAL_LOGGING_ENABLED
                // log data
                if (option_enabled(AP_Torqeedo::options::LOG)) {
                    // @LoggerMessage: TRST
                    // @Description: Torqeedo System State
                    // @Field: TimeUS: Time since system startup
                    // @Field: I: instance
                    // @Field: F: Flags bitmask
                    // @Field: Err: Master error code
                    // @Field: MVolt: Motor voltage
                    // @Field: MCur: Motor current
                    // @Field: Pow: Motor power
                    // @Field: RPM: Motor RPM
                    // @Field: MTemp: Motor Temp (higher of pcb or stator)
                    // @Field: BPct: Battery charge percentage
                    // @Field: BVolt: Battery voltage
                    // @Field: BCur: Battery current
                    AP::logger().Write("TRST", "TimeUS,I,F,Err,MVolt,MCur,Pow,RPM,MTemp,BPct,BVolt,BCur",
                                      "s#--vAWqO%vA",   // units
                                      "F---00000000",   // multipliers
                                      "QBHBffHhBBff",   // formats
                                       AP_HAL::micros64(),
                                       _instance,
                                       _display_system_state.flags.value,
                                       _display_system_state.master_error_code,
                                       _display_system_state.motor_voltage,
                                       _display_system_state.motor_current,
                                       _display_system_state.motor_power,
                                       _display_system_state.motor_rpm,
                                       motor_temp,
                                       _display_system_state.batt_charge_pct,
                                      _display_system_state.batt_voltage,
                                      _display_system_state.batt_current);
                }
#endif

                // send to GCS
                if (option_enabled(AP_Torqeedo::options::DEBUG_TO_GCS)) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"TRST i:%u, F:%u Err:%u MV:%4.1f MC:%4.1f P:%u MT:%d B%%:%d BV:%4.1f BC:%4.1f",
                            (unsigned)_instance,
                            (unsigned)_display_system_state.flags.value,
                            (unsigned)_display_system_state.master_error_code,
                            (double)_display_system_state.motor_voltage,
                            (double)_display_system_state.motor_current,
                            (unsigned)_display_system_state.motor_power,
                            (int)motor_temp,
                            (unsigned)_display_system_state.batt_charge_pct,
                            (double)_display_system_state.batt_voltage,
                            (double)_display_system_state.batt_current);
                }

                // report any errors
                report_error_codes();
            } else {
                // unexpected length
                _parse_error_count++;
            }
            break;
        case DisplayMsgId::SYSTEM_SETUP:
            if (_received_buff_len == 13) {
                // fill in display system setup
                _display_system_setup.flags = _received_buff[2];
                _display_system_setup.motor_type = _received_buff[3];
                _display_system_setup.motor_sw_version = UINT16_VALUE(_received_buff[4], _received_buff[5]);
                _display_system_setup.batt_capacity = UINT16_VALUE(_received_buff[6], _received_buff[7]);
                _display_system_setup.batt_charge_pct = _received_buff[8];
                _display_system_setup.batt_type = _received_buff[9];
                _display_system_setup.master_sw_version =  UINT16_VALUE(_received_buff[10], _received_buff[11]);

#if HAL_LOGGING_ENABLED
                // log data
                if (option_enabled(AP_Torqeedo::options::LOG)) {
                    // @LoggerMessage: TRSE
                    // @Description: Torqeedo System Setup
                    // @Field: TimeUS: Time since system startup
                    // @Field: I: instance
                    // @Field: Flag: Flags
                    // @Field: MotType: Motor type
                    // @Field: MotVer: Motor software version
                    // @Field: BattCap: Battery capacity
                    // @Field: BattPct: Battery charge percentage
                    // @Field: BattType: Battery type
                    // @Field: SwVer: Master software version
                    AP::logger().Write("TRSE", "TimeUS,I,Flag,MotType,MotVer,BattCap,BattPct,BattType,SwVer",
                                       "s#---a%--", // units
                                       "F----00--", // multipliers
                                       "QBBBHHBBH", // formats
                                       AP_HAL::micros64(),
                                       _instance,
                                       _display_system_setup.flags,
                                       _display_system_setup.motor_type,
                                       _display_system_setup.motor_sw_version,
                                       _display_system_setup.batt_capacity,
                                       _display_system_setup.batt_charge_pct,
                                       _display_system_setup.batt_type,
                                       _display_system_setup.master_sw_version);
                }
#endif

                // send to GCS
                if (option_enabled(AP_Torqeedo::options::DEBUG_TO_GCS)) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"TRSE i:%u v:%u F:%u Mot:%u/%u Bat:%u/%u/%u%%",
                            (unsigned)_instance,
                            (unsigned)_display_system_setup.master_sw_version,
                            (unsigned)_display_system_setup.flags,
                            (unsigned)_display_system_setup.motor_type,
                            (unsigned)_display_system_setup.motor_sw_version,
                            (unsigned)_display_system_setup.batt_type,
                            (unsigned)_display_system_setup.batt_capacity,
                            (unsigned)_display_system_setup.batt_charge_pct);
                }
            } else {
                // unexpected length
                _parse_error_count++;
            }
            break;
        default:
            // ignore message
            break;
        }
        return;
    }

    // handle reply from motor
    if ((get_type() == AP_Torqeedo::ConnectionType::TYPE_MOTOR) && (msg_addr == MsgAddress::BUS_MASTER)) {
        // replies strangely do not return the msgid so we must have stored it
        MotorMsgId msg_id = (MotorMsgId)_reply_msgid;
        switch (msg_id) {

        case MotorMsgId::PARAM:
            if (_received_buff_len == 15) {
                _motor_param.rpm = (int16_t)UINT16_VALUE(_received_buff[2], _received_buff[3]);
                _motor_param.power = UINT16_VALUE(_received_buff[4], _received_buff[5]);
                _motor_param.voltage = UINT16_VALUE(_received_buff[6], _received_buff[7]) * 0.01;
                _motor_param.current = UINT16_VALUE(_received_buff[8], _received_buff[9]) * 0.1;
                _motor_param.pcb_temp = (int16_t)UINT16_VALUE(_received_buff[10], _received_buff[11]) * 0.1;
                _motor_param.stator_temp = (int16_t)UINT16_VALUE(_received_buff[12], _received_buff[13]) * 0.1;
                _motor_param.last_update_ms = AP_HAL::millis();

                // update esc telem sent to ground station
                update_esc_telem(_motor_param.rpm,
                        _motor_param.voltage,
                        _motor_param.current,
                        _motor_param.pcb_temp,      // esc temp
                        _motor_param.stator_temp);  // motor temp

#if HAL_LOGGING_ENABLED
                // log data
                if (option_enabled(AP_Torqeedo::options::LOG)) {
                    // @LoggerMessage: TRMP
                    // @Description: Torqeedo Motor Param
                    // @Field: TimeUS: Time since system startup
                    // @Field: I: instance
                    // @Field: RPM: Motor RPM
                    // @Field: Pow: Motor power
                    // @Field: Volt: Motor voltage
                    // @Field: Cur: Motor current
                    // @Field: ETemp: ESC Temp
                    // @Field: MTemp: Motor Temp
                    AP::logger().Write("TRMP", "TimeUS,I,RPM,Pow,Volt,Cur,ETemp,MTemp",
                                       "s#qWvAOO",  // units
                                       "F-000000",  // multipliers
                                       "QBhHffff",  // formats
                                       AP_HAL::micros64(),
                                       _instance,
                                       _motor_param.rpm,
                                       _motor_param.power,
                                       _motor_param.voltage,
                                       _motor_param.current,
                                       _motor_param.pcb_temp,
                                       _motor_param.stator_temp);
                }
#endif

                // send to GCS
                if (option_enabled(AP_Torqeedo::options::DEBUG_TO_GCS)) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TRMP i:%u rpm:%d p:%u V:%4.1f C:%4.1f PT:%4.1f MT:%4.1f",
                                                       (unsigned)_instance,
                                                       (int)_motor_param.rpm,
                                                       (unsigned)_motor_param.power,
                                                       (double)_motor_param.voltage,
                                                       (double)_motor_param.current,
                                                       (double)_motor_param.pcb_temp,
                                                       (double)_motor_param.stator_temp);
                }
            } else {
                // unexpected length
                _parse_error_count++;
            }
            break;

        case MotorMsgId::STATUS:
            if (_received_buff_len == 6) {
                _motor_status.status_flags_value = _received_buff[2];
                _motor_status.error_flags_value = UINT16_VALUE(_received_buff[3], _received_buff[4]);

#if HAL_LOGGING_ENABLED
                // log data
                if (option_enabled(AP_Torqeedo::options::LOG)) {
                    // @LoggerMessage: TRMS
                    // @Description: Torqeedo Motor Status
                    // @Field: TimeUS: Time since system startup
                    // @Field: I: instance
                    // @Field: State: Motor status flags
                    // @Field: Err: Motor error flags
                    AP::logger().Write("TRMS", "TimeUS,I,State,Err",
                                       "s#--",  // units
                                       "F---",  // multipliers
                                       "QBBH",  // formats
                                       AP_HAL::micros64(),
                                        _instance,
                                       _motor_status.status_flags_value,
                                       _motor_status.error_flags_value);
                }
#endif

                // send to GCS
                if (option_enabled(AP_Torqeedo::options::DEBUG_TO_GCS)) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"TRMS i:%u S:%d Err:%d",
                                   (unsigned)_instance,
                                   _motor_status.status_flags_value,
                                   _motor_status.error_flags_value);
                }

                // report any errors
                report_error_codes();
            } else {
                // unexpected length
                _parse_error_count++;
            }
            break;

        case MotorMsgId::INFO:
        case MotorMsgId::DRIVE:
        case MotorMsgId::CONFIG:
            // we do not process these replies
            break;

        default:
            // ignore unknown messages
            break;
        }
    }
}

// set DE Serial CTS pin to enable sending commands to motor
void AP_Torqeedo_TQBus::send_start()
{
    // set gpio pin or serial port's CTS pin
    if (_params.pin_de > -1) {
        hal.gpio->write(_params.pin_de, 1);
    } else {
        _uart->set_CTS_pin(true);
    }
}

// check for timeout after sending and unset pin if required
void AP_Torqeedo_TQBus::check_for_send_end()
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
    if (_params.pin_de > -1) {
        hal.gpio->write(_params.pin_de, 0);
    } else {
        _uart->set_CTS_pin(false);
    }
}

// calculate delay require to allow bytes to be sent
uint32_t AP_Torqeedo_TQBus::calc_send_delay_us(uint8_t num_bytes)
{
    // baud rate of 19200 bits/sec
    // total number of bits = 10 x num_bytes
    // convert from seconds to micros by multiplying by 1,000,000
    // plus additional 300us safety margin
    const uint32_t delay_us = 1e6 * num_bytes * 10 / TORQEEDO_SERIAL_BAUD + 300;
    return delay_us;
}

// record msgid of message to wait for and set timer for timeout handling
void AP_Torqeedo_TQBus::set_expected_reply_msgid(uint8_t msg_id)
{
    _reply_msgid = msg_id;
    _reply_wait_start_ms = AP_HAL::millis();
}

// check for timeout waiting for reply message
void AP_Torqeedo_TQBus::check_for_reply_timeout()
{
    // return immediately if not waiting for reply
    if (_reply_wait_start_ms == 0) {
        return;
    }
    if (AP_HAL::millis() - _reply_wait_start_ms > TORQEEDO_REPLY_TIMEOUT_MS) {
        _reply_wait_start_ms = 0;
        _parse_error_count++;
    }
}

// mark reply received. should be called whenever a message is received regardless of whether we are actually waiting for a reply
void AP_Torqeedo_TQBus::set_reply_received()
{
    _reply_wait_start_ms = 0;
}

// send a message to the motor with the specified message contents
// msg_contents should not include the header, footer or CRC
// returns true on success
bool AP_Torqeedo_TQBus::send_message(const uint8_t msg_contents[], uint8_t num_bytes)
{
    // buffer for outgoing message
    uint8_t send_buff[TORQEEDO_MESSAGE_LEN_MAX];
    uint8_t send_buff_num_bytes = 0;

    // calculate crc
    const uint8_t crc = crc8_maxim(msg_contents, num_bytes);

    // add header
    send_buff[send_buff_num_bytes++] = TORQEEDO_PACKET_HEADER;

    // add contents
    for (uint8_t i=0; i<num_bytes; i++) {
        if (!add_byte_to_message(msg_contents[i], send_buff, ARRAY_SIZE(send_buff), send_buff_num_bytes)) {
            _parse_error_count++;
            return false;
        }
    }

    // add crc
    if (!add_byte_to_message(crc, send_buff, ARRAY_SIZE(send_buff), send_buff_num_bytes)) {
        _parse_error_count++;
        return false;
    }

    // add footer
    if (send_buff_num_bytes >= ARRAY_SIZE(send_buff)) {
        _parse_error_count++;
        return false;
    }
    send_buff[send_buff_num_bytes++] = TORQEEDO_PACKET_FOOTER;

    // set send pin
    send_start();

    // write message
    _uart->write(send_buff, send_buff_num_bytes);

    // record start and expected delay to send message
    _send_start_us = AP_HAL::micros();
    _send_delay_us = calc_send_delay_us(send_buff_num_bytes);

    return true;
}

// add a byte to a message buffer including adding the escape character (0xAE) if necessary
// this should only be used when adding the contents to the buffer, not the header and footer
// num_bytes is updated to the next free byte
bool AP_Torqeedo_TQBus::add_byte_to_message(uint8_t byte_to_add, uint8_t msg_buff[], uint8_t msg_buff_size, uint8_t &num_bytes) const
{
    bool escape_required = (byte_to_add == TORQEEDO_PACKET_HEADER ||
                            byte_to_add == TORQEEDO_PACKET_FOOTER ||
                            byte_to_add == TORQEEDO_PACKET_ESCAPE);

    // check if we have enough space
    if (num_bytes + (escape_required ? 2 : 1) >= msg_buff_size) {
        return false;
    }

    // add byte
    if (escape_required) {
        msg_buff[num_bytes++] = TORQEEDO_PACKET_ESCAPE;
        msg_buff[num_bytes++] = byte_to_add ^ TORQEEDO_PACKET_ESCAPE_MASK;
    } else {
        msg_buff[num_bytes++] = byte_to_add;
    }
    return true;
}

// Example "Remote (0x01)" reply message to allow tiller to control motor speed
// Byte     Field Definition    Example Value   Comments
// ---------------------------------------------------------------------------------
// byte 0   Header              0xAC
// byte 1   TargetAddress       0x00            see MsgAddress enum
// byte 2   Message ID          0x00            only master populates this. replies have this set to zero
// byte 3   Flags               0x05            bit0=pin present, bit2=motor speed valid
// byte 4   Status              0x00            0x20 if byte3=4, 0x0 is byte3=5
// byte 5   Motor Speed MSB     ----            Motor Speed MSB (-1000 to +1000)
// byte 6   Motor Speed LSB     ----            Motor Speed LSB (-1000 to +1000)
// byte 7   CRC-Maxim           ----            CRC-Maxim value
// byte 8   Footer              0xAD
//
// example message when rotating tiller handle forwards:  "AC 00 00 05 00 00 ED 95 AD"    (+237)
// example message when rotating tiller handle backwards: "AC 00 00 05 00 FF AE 2C 0C AD" (-82)

// send a motor speed command as a value from -1000 to +1000
// value is taken directly from SRV_Channel
// for tiller connection this sends the "Remote (0x01)" message
// for motor connection this sends the "Motor Drive (0x82)" message
void AP_Torqeedo_TQBus::send_motor_speed_cmd()
{
    // calculate desired motor speed
    if (!hal.util->get_soft_armed()) {
        _motor_speed_desired = 0;
    } else {
        // convert throttle output to motor output in range -1000 to +1000
        // ToDo: convert PWM output to motor output so that SERVOx_MIN, MAX and TRIM take effect
        _motor_speed_desired = constrain_int16(SRV_Channels::get_output_norm((SRV_Channel::Aux_servo_function_t)_params.servo_fn.get()) * 1000.0, -1000, 1000);
    }

    // updated limited motor speed
    int16_t mot_speed_limited = calc_motor_speed_limited(_motor_speed_desired);

    // by default use tiller connection command
    uint8_t mot_speed_cmd_buff[] = {(uint8_t)MsgAddress::BUS_MASTER, 0x0, 0x5, 0x0, HIGHBYTE(mot_speed_limited), LOWBYTE(mot_speed_limited)};

    // update message if using motor connection
    if (get_type() == AP_Torqeedo::ConnectionType::TYPE_MOTOR) {
        const uint8_t motor_power = (uint8_t)constrain_int16(_params.motor_power, 0, 100);
        mot_speed_cmd_buff[0] = (uint8_t)MsgAddress::MOTOR;
        mot_speed_cmd_buff[1] = (uint8_t)MotorMsgId::DRIVE;
        mot_speed_cmd_buff[2] = (mot_speed_limited == 0 ? 0 : 0x01) | (_motor_clear_error ? 0x04 : 0);  // 1:enable motor, 2:fast off, 4:clear error
        mot_speed_cmd_buff[3] = mot_speed_limited == 0 ? 0 : motor_power;   // motor power from 0 to 100

        // set expected reply message id
        set_expected_reply_msgid((uint8_t)MotorMsgId::DRIVE);

        // reset motor clear error request
        _motor_clear_error = false;
    }

    // send a message
    if (send_message(mot_speed_cmd_buff, ARRAY_SIZE(mot_speed_cmd_buff))) {
        // record time of send for health reporting
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_send_motor_ms = AP_HAL::millis();
    }
}

// send request to motor to reply with a particular message
// msg_id can be INFO, STATUS or PARAM
void AP_Torqeedo_TQBus::send_motor_msg_request(MotorMsgId msg_id)
{
    // prepare message
    uint8_t mot_status_request_buff[] = {(uint8_t)MsgAddress::MOTOR, (uint8_t)msg_id};

    // send a message
    if (send_message(mot_status_request_buff, ARRAY_SIZE(mot_status_request_buff))) {
        // record waiting for reply
        set_expected_reply_msgid((uint8_t)msg_id);
    }
}

// calculate the limited motor speed that is sent to the motors
// desired_motor_speed argument and returned value are in the range -1000 to 1000
int16_t AP_Torqeedo_TQBus::calc_motor_speed_limited(int16_t desired_motor_speed)
{
    const uint32_t now_ms = AP_HAL::millis();

    // update dir_limit flag for forward-reverse transition delay
    const bool dir_delay_active = is_positive(_params.dir_delay);
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
            if ((_motor_speed_zero_ms != 0) && ((now_ms - _motor_speed_zero_ms) > (_params.dir_delay * 1000))) {
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
    float dt = (now_ms - _motor_speed_limited_ms) * 0.001f;
    if (dt > 1.0) {
        // after a long delay limit motor output to zero to avoid sudden starts
        lower_limit = 0;
        upper_limit = 0;
    }
    _motor_speed_limited_ms = now_ms;

    // apply slew limit
    if (_params.slew_time > 0) {
       const float chg_max = 1000.0 * dt / _params.slew_time;
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
void AP_Torqeedo_TQBus::log_TRQD(bool force_logging)
{
    // exit immediately if logging and debug options are not set
    if (!option_enabled(AP_Torqeedo::options::LOG) && option_enabled(AP_Torqeedo::options::DEBUG_TO_GCS)) {
        return;
    }

    // return if not enough time has passed since last output
    const uint32_t now_ms = AP_HAL::millis();
    if (!force_logging && (now_ms - _last_log_TRQD_ms < TORQEEDO_LOG_TRQD_INTERVAL_MS)) {
        return;
    }
    _last_log_TRQD_ms = now_ms;

#if HAL_LOGGING_ENABLED || HAL_GCS_ENABLED
    const bool health = healthy();
    int16_t actual_motor_speed = get_motor_speed_limited();

#if HAL_LOGGING_ENABLED
    if (option_enabled(AP_Torqeedo::options::LOG)) {
        // @LoggerMessage: TRQD
        // @Description: Torqeedo Status
        // @Field: TimeUS: Time since system startup
        // @Field: I: instance
        // @Field: Health: Health
        // @Field: DesMotSpeed: Desired Motor Speed (-1000 to 1000)
        // @Field: MotSpeed: Motor Speed (-1000 to 1000)
        // @Field: SuccCnt: Success Count
        // @Field: ErrCnt: Error Count
        AP::logger().Write("TRQD", "TimeUS,I,Health,DesMotSpeed,MotSpeed,SuccCnt,ErrCnt",
                           "s#-----",   // units
                           "F------",   // multipliers
                           "QBBhhII",   // formats
                           AP_HAL::micros64(),
                           _instance,
                           health,
                           _motor_speed_desired,
                           actual_motor_speed,
                           _parse_success_count,
                           _parse_error_count);
    }
#endif

    if (option_enabled(AP_Torqeedo::options::DEBUG_TO_GCS)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"TRQD i:%u h:%u dspd:%d spd:%d succ:%ld err:%ld",
                (unsigned)_instance,
                (unsigned)health,
                (int)_motor_speed_desired,
                (int)actual_motor_speed,
                (unsigned long)_parse_success_count,
                (unsigned long)_parse_error_count);
    }
#endif  // HAL_LOGGING_ENABLED || HAL_GCS_ENABLED
}

// send ESC telemetry
void AP_Torqeedo_TQBus::update_esc_telem(float rpm, float voltage, float current_amps, float esc_tempC, float motor_tempC)
{
#if HAL_WITH_ESC_TELEM
    // find servo output channel
    uint8_t telem_esc_index = 0;
    IGNORE_RETURN(SRV_Channels::find_channel((SRV_Channel::Aux_servo_function_t)_params.servo_fn.get(), telem_esc_index));

    // fill in telemetry data structure
    AP_ESC_Telem_Backend::TelemetryData telem_dat {};
    telem_dat.temperature_cdeg = esc_tempC * 100;   // temperature in centi-degrees
    telem_dat.voltage = voltage;                    // voltage in volts
    telem_dat.current = current_amps;               // current in amps
    telem_dat.motor_temp_cdeg = motor_tempC * 100;  // motor temperature in centi-degrees

    // send telem and rpm data
    update_telem_data(telem_esc_index, telem_dat, AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE |
                                                  AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE |
                                                  AP_ESC_Telem_Backend::TelemetryType::CURRENT |
                                                  AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);

    update_rpm(telem_esc_index, rpm);
#endif
}

#endif // HAL_TORQEEDO_ENABLED
