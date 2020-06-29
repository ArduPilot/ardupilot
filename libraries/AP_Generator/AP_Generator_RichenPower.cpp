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

#include "AP_Generator_RichenPower.h"

#if GENERATOR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

// TODO: failsafe if we don't get readings?

AP_Generator_RichenPower::AP_Generator_RichenPower()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many richenpower generators");
#endif
        return;
    }
    _singleton = this;
}

void AP_Generator_RichenPower::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Generator, 0);
    if (uart != nullptr) {
        const uint32_t baud = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Generator, 0);
        uart->begin(baud, 256, 256);
    }
}

void AP_Generator_RichenPower::move_header_in_buffer(uint8_t initial_offset)
{
    uint8_t header_offset;
    for (header_offset=initial_offset; header_offset<body_length; header_offset++) {
        if (u.parse_buffer[header_offset] == HEADER_MAGIC1) {
            break;
        }
    }
    if (header_offset != 0) {
        // header was found, but not at index 0; move it back to start of array
        memmove(u.parse_buffer, &u.parse_buffer[header_offset], body_length - header_offset);
        body_length -= header_offset;
    }
}

// read - read serial port, return true if a new reading has been found
bool AP_Generator_RichenPower::get_reading()
{
    if (uart == nullptr) {
        return false;
    }

    // Example of a packet from a H2 controller:
    //AA 55 00 0A 00 00 00 00 00 04 1E B0 00 10 00 00 23 7A 23
    //7A 11 1D 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00
    //00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
    //00 00 00 00 00 00 00 00 00 1E BE 55 AA
    //16 bit one data Hight + Low

    // fill our buffer some more:
    uint32_t nbytes = uart->read(&u.parse_buffer[body_length],
                                 ARRAY_SIZE(u.parse_buffer)-body_length);
    if (nbytes == 0) {
        return false;
    }
    body_length += nbytes;

    move_header_in_buffer(0);

    // header byte 1 is correct.
    if (body_length < ARRAY_SIZE(u.parse_buffer)) {
        // need a full buffer to have a valid message...
        return false;
    }

    if (u.packet.headermagic2 != HEADER_MAGIC2) {
        move_header_in_buffer(1);
        return false;
    }

    // check for the footer signature:
    if (u.packet.footermagic1 != FOOTER_MAGIC1) {
        move_header_in_buffer(1);
        return false;
    }
    if (u.packet.footermagic2 != FOOTER_MAGIC2) {
        move_header_in_buffer(1);
        return false;
    }

    // calculate checksum....
    uint16_t checksum = 0;
    for (uint8_t i=0; i<5; i++) {
        checksum += be16toh(checksum_buffer[i]);
    }

    if (checksum != be16toh(u.packet.checksum)) {
        move_header_in_buffer(1);
        return false;
    }

    // check the version:
    const uint16_t version = be16toh(u.packet.version);
    const uint8_t major = version / 100;
    const uint8_t minor = (version % 100) / 10;
    const uint8_t point = version % 10;
    if (!protocol_information_anounced) {
        gcs().send_text(MAV_SEVERITY_INFO, "RichenPower: protocol %u.%u.%u", major, minor, point);
        protocol_information_anounced = true;
    }

    last_reading.runtime = be16toh(u.packet.runtime_hours) * 3600 +
        u.packet.runtime_minutes * 60 +
        u.packet.runtime_seconds;
    last_reading.seconds_until_maintenance = be32toh(u.packet.seconds_until_maintenance);
    last_reading.errors = be16toh(u.packet.errors);
    last_reading.rpm = be16toh(u.packet.rpm);
    last_reading.output_voltage = be16toh(u.packet.output_voltage) / 100.0f;
    last_reading.output_current = be16toh(u.packet.output_current) / 100.0f;
    last_reading.mode = (Mode)u.packet.mode;

    last_reading_ms = AP_HAL::millis();

    body_length = 0;

    return true;
}

void AP_Generator_RichenPower::update_heat()
{
    // assume heat increase is directly proportional to RPM.
    const uint32_t now = AP_HAL::millis();
    uint16_t rpm = last_reading.rpm;
    if (now - last_reading_ms > 2000) {
        // if we're not getting updates, assume we're getting colder
        rpm = 0;
        // ... and resend the version information when we get something again
        protocol_information_anounced = false;
    }

    const uint32_t time_delta_ms = now - last_heat_update_ms;
    last_heat_update_ms = now;

    heat += rpm * time_delta_ms * (1/1000.0f);
    // cap the heat of the motor:
    heat = MIN(heat, 60 * RUN_RPM); // so cap heat at 60 seconds at run-speed
    // now lose some heat to the environment
    heat -= (heat * environment_loss_factor * (time_delta_ms * (1/1000.0f)));  // lose some % of heat per second
}

constexpr float AP_Generator_RichenPower::heat_required_for_run()
{
    // assume that heat is proportional to RPM.  Return a number
    // proportial to RPM.  Reduce it to account for the cooling some%/s
    // cooling
    return (30 * IDLE_RPM) * environment_loss_30s;
}
bool AP_Generator_RichenPower::generator_ok_to_run() const
{
    return heat > heat_required_for_run();
}

constexpr float AP_Generator_RichenPower::heat_required_for_supply()
{
    // account for cooling that happens in that 60 seconds
    return (30 * IDLE_RPM + 30 * RUN_RPM) * environment_loss_60s;
}
bool AP_Generator_RichenPower::generator_ok_to_supply() const
{
    // duplicated into prearms
    return heat > heat_required_for_supply();
}



/*
  update the state of the sensor
*/
void AP_Generator_RichenPower::update(void)
{
    if (uart == nullptr) {
        return;
    }

    update_servo_channel();

    update_runstate();

    (void)get_reading();

    update_heat();

    Log_Write();
}

void AP_Generator_RichenPower::update_runstate()
{
    if (_servo_channel == nullptr) {
        // doesn't really matter what we want to do; we're not going
        // to be able to affect it...
        return;
    }

    // don't run the generator while the safety is on:
    // if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
    //     _servo_channel->set_output_pwm(SERVO_PWM_STOP);
    //     return;
    // }

    static const uint16_t SERVO_PWM_STOP = 1200;
    static const uint16_t SERVO_PWM_IDLE = 1500;
    static const uint16_t SERVO_PWM_RUN = 1900;

    switch (runstate) {
    case RunState::STOP:
        _servo_channel->set_output_pwm(SERVO_PWM_STOP);
        break;
    case RunState::IDLE:
        _servo_channel->set_output_pwm(SERVO_PWM_IDLE);
        break;
    case RunState::RUN:
        // we must have
        if (!generator_ok_to_run()) {
            _servo_channel->set_output_pwm(SERVO_PWM_IDLE);
            break;
        }
        _servo_channel->set_output_pwm(SERVO_PWM_RUN);
        break;
    }
}

void AP_Generator_RichenPower::update_servo_channel(void)
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_servo_channel_check < 1000) {
        return;
    }
    _last_servo_channel_check = now;
    SRV_Channel *control = SRV_Channels::get_channel_for(SRV_Channel::k_richenpower_control);
    if (control == nullptr) {
        if (_servo_channel != nullptr) {
            gcs().send_text(MAV_SEVERITY_INFO, "RichenPower: no control channel");
            _servo_channel = nullptr;
        }
        return;
    }
    if (_servo_channel != nullptr &&
        _servo_channel->get_function() == SRV_Channel::k_richenpower_control) {
        // note that _servo_channel could actually be == control here
        return;
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "RP: using control channel");
    _servo_channel = control;
}

void AP_Generator_RichenPower::Log_Write()
{
    if (last_logged_reading_ms == last_reading_ms) {
        return;
    }
    last_logged_reading_ms = last_reading_ms;

    AP::logger().Write(
        "GEN",
        "TimeUS,runTime,maintTime,errors,rpm,ovolt,ocurr,mode",
        "s-------",
        "F-------",
        "QIIHHffB",
        AP_HAL::micros64(),
        last_reading.runtime,
        last_reading.seconds_until_maintenance,
        last_reading.errors,
        last_reading.rpm,
        last_reading.output_voltage,
        last_reading.output_current,
        last_reading.mode
        );
}

bool AP_Generator_RichenPower::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    if (uart == nullptr) {
        // not configured in serial manager
        return true;
    }
    const uint32_t now = AP_HAL::millis();

    if (now - last_reading_ms > 2000) { // we expect @1Hz
        snprintf(failmsg, failmsg_len, "no messages in %ums", unsigned(now - last_reading_ms));
        return false;
    }
    if (last_reading.seconds_until_maintenance == 0) {
        snprintf(failmsg, failmsg_len, "requires maintenance");
        return false;
    }
    if (last_reading.errors) {
        for (uint16_t i=0; i<16; i++) {
            if (last_reading.errors & (1U << (uint16_t)i)) {
                if (i < (uint16_t)Errors::LAST) {
                    snprintf(failmsg, failmsg_len, "error: %s", error_strings[i]);
                } else {
                    snprintf(failmsg, failmsg_len, "unknown error: 1U<<%u", i);
                }
            }
        }
        return false;
    }
    if (last_reading.mode != Mode::RUN && last_reading.mode != Mode::CHARGE && last_reading.mode != Mode::BALANCE) {
        snprintf(failmsg, failmsg_len, "not running");
        return false;
    }
    if (runstate != RunState::RUN) {
        snprintf(failmsg, failmsg_len, "requested state is not RUN");
        return false;
    }
    if (!generator_ok_to_supply()) {
        snprintf(failmsg, failmsg_len, "warming up (%.0f%%)", (heat *100 / heat_required_for_supply()));
        return false;
    }

    return true;
}

bool AP_Generator_RichenPower::voltage(float &v) const
{
    if (last_reading_ms == 0) {
        return false;
    }
    v = last_reading.output_voltage;
    return true;
}

bool AP_Generator_RichenPower::current(float &curr) const
{
    if (last_reading_ms == 0) {
        return false;
    }
    curr = last_reading.output_current;
    return true;
}

bool AP_Generator_RichenPower::healthy() const
{
    const uint32_t now = AP_HAL::millis();

    if (last_reading_ms == 0 || now - last_reading_ms > 2000) {
        return false;
    }
    if (last_reading.errors) {
        return false;
    }
    return true;
}

//send generator status
void AP_Generator_RichenPower::send_generator_status(const GCS_MAVLINK &channel)
{
    uint64_t status = 0;
    switch (last_reading.mode) {
    case Mode::IDLE:
        break;
    case Mode::RUN:
        status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
        break;
    case Mode::CHARGE:
        status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
        status |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
        break;
    case Mode::BALANCE:
        status |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
        status |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
        break;
    }

    if (last_reading.rpm == 0) {
        status |= MAV_GENERATOR_STATUS_FLAG_OFF;
    }

    if (last_reading.errors & (uint8_t)Errors::Overload) {
        status |= MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT;
    }
    if (last_reading.errors & (uint8_t)Errors::LowVoltageOutput) {
        status |= MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER;
    }

    if (last_reading.errors & (uint8_t)Errors::MaintenanceRequired) {
        status |= MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED;
    }
    if (last_reading.errors & (uint8_t)Errors::StartDisabled) {
        status |= MAV_GENERATOR_STATUS_FLAG_START_INHIBITED;
    }
    if (last_reading.errors & (uint8_t)Errors::LowBatteryVoltage) {
        status |= MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT;
    }

    mavlink_msg_generator_status_send(
        channel.get_chan(),
        status,
        last_reading.rpm, // generator_speed
        std::numeric_limits<double>::quiet_NaN(), // battery_current; current into/out of battery
        last_reading.output_current, // load_current; Current going to UAV
        std::numeric_limits<double>::quiet_NaN(), // power_generated; the power being generated
        last_reading.output_voltage, // bus_voltage; Voltage of the bus seen at the generator
        INT16_MAX, // rectifier_temperature
        std::numeric_limits<double>::quiet_NaN(), // bat_current_setpoint; The target battery current
        INT16_MAX // generator temperature
        );
}

/*
 * Get the AP_Generator singleton
 */
AP_Generator_RichenPower *AP_Generator_RichenPower::_singleton;
AP_Generator_RichenPower *AP_Generator_RichenPower::get_singleton()
{
    return _singleton;
}

namespace AP {

AP_Generator_RichenPower *generator()
{
    return AP_Generator_RichenPower::get_singleton();
}

};

#endif
