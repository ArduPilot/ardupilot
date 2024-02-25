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

#include "AP_Generator_config.h"

#if AP_GENERATOR_RICHENPOWER_ENABLED

#include "AP_Generator_RichenPower.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

// init method; configure communications with the generator
void AP_Generator_RichenPower::init()
{
    ASSERT_STORAGE_SIZE(RichenPacket, 70);

    const AP_SerialManager &serial_manager = AP::serialmanager();

    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Generator, 0);
    if (uart != nullptr) {
        const uint32_t baud = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Generator, 0);
        uart->begin(baud, 256, 256);
    }

    // Tell frontend what measurements are available for this generator
    _frontend._has_current = true;
    _frontend._has_consumed_energy = false;
    _frontend._has_fuel_remaining = false;
}

// find a RichenPower message in the buffer, starting at
// initial_offset.  If dound, that message (or partial message) will
// be moved to the start of the buffer.
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
    const uint8_t *checksum_buffer = &u.parse_buffer[2];
    for (uint8_t i=0; i<5; i++) {
        checksum += be16toh_ptr(&checksum_buffer[2*i]);
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
    last_reading.seconds_until_maintenance = be16toh(u.packet.seconds_until_maintenance_high) * 65536 + be16toh(u.packet.seconds_until_maintenance_low);
    last_reading.errors = be16toh(u.packet.errors);
    last_reading.rpm = be16toh(u.packet.rpm);
    last_reading.output_voltage = be16toh(u.packet.output_voltage) * 0.01f;
    last_reading.output_current = be16toh(u.packet.output_current) * 0.01f;
    last_reading.mode = (Mode)u.packet.mode;

    last_reading_ms = AP_HAL::millis();

    body_length = 0;

    // update the time we started idling at:
    if (last_reading.mode == Mode::IDLE) {
        if (idle_state_start_ms == 0) {
            idle_state_start_ms = last_reading_ms;
        }
    } else {
        idle_state_start_ms = 0;
    }

    return true;
}

// update the synthetic heat measurement we are keeping for the
// generator.  We keep a synthetic heat measurement as the telemetry
// from the unit does not include the motor temperature, so we
// approximate one by assuming it produces heat in proportion to its
// current RPM.
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
    heat -= (heat * heat_environment_loss_factor * (time_delta_ms * (1/1000.0f)));  // lose some % of heat per second
}

// returns true if the generator should be allowed to move into
// the "run" (high-RPM) state:
bool AP_Generator_RichenPower::generator_ok_to_run() const
{
    return heat > heat_required_for_run();
}

// returns an amount of synthetic heat required for the generator
// to move into the "run" state:
constexpr float AP_Generator_RichenPower::heat_required_for_run()
{
    // assume that heat is proportional to RPM.  Return a number
    // proportional to RPM.  Reduce it to account for the cooling some%/s
    // cooling
    return (45 * IDLE_RPM) * heat_environment_loss_30s;
}


void AP_Generator_RichenPower::check_maintenance_required()
{
    // don't bother the user while flying:
    if (hal.util->get_soft_armed()) {
        return;
    }

    if (!AP::generator()->option_set(AP_Generator::Option::INHIBIT_MAINTENANCE_WARNINGS)) {
        const uint32_t now = AP_HAL::millis();

        if (last_reading.errors & (1U<<uint16_t(Errors::MaintenanceRequired))) {
            if (now - last_maintenance_warning_ms > 60000) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Generator: requires maintenance");
                last_maintenance_warning_ms = now;
            }
        }
    }
}

/*
  update the state of the sensor
*/
void AP_Generator_RichenPower::update(void)
{
    if (uart == nullptr) {
        return;
    }

    if (last_reading_ms != 0) {
        update_runstate();
        check_maintenance_required();
    }

    (void)get_reading();

    update_heat();

    update_frontend_readings();

#if HAL_LOGGING_ENABLED
    Log_Write();
#endif
}

// update_runstate updates the servo output we use to control the
// generator.  Which state we request the generator move to depends on
// the RC inputcontrol and the temperature the generator is at.
void AP_Generator_RichenPower::update_runstate()
{
    // don't run the generator while the safety is on:
    // if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
    //     _servo_channel->set_output_pwm(SERVO_PWM_STOP);
    //     return;
    // }

    static const uint16_t SERVO_PWM_STOP = 1200;
    static const uint16_t SERVO_PWM_IDLE = 1500;
    static const uint16_t SERVO_PWM_RUN = 1900;

    // if the vehicle crashes then we assume the pilot wants to stop
    // the motor.  This is done as a once-off when the crash is
    // detected to allow the operator to rearm the vehicle, or we end
    // up in a catch-22 situation where we force the stop state on the
    // generator so they can't arm and can't start the generator
    // because the vehicle is crashed.
    if (AP::vehicle()->is_crashed()) {
        if (!vehicle_was_crashed) {
            gcs().send_text(MAV_SEVERITY_INFO, "Crash; stopping generator");
            pilot_desired_runstate = RunState::STOP;
            vehicle_was_crashed = true;
        }
    } else {
        vehicle_was_crashed = false;
    }

    if (commanded_runstate != pilot_desired_runstate &&
        !hal.util->get_soft_armed()) {
        // consider changing the commanded runstate to the pilot
        // desired runstate:
        commanded_runstate = RunState::IDLE;
        switch (pilot_desired_runstate) {
        case RunState::STOP:
            // The H2 will keep the motor running for ~20 seconds for cool-down
            // we must go via the idle state or the H2 disallows moving to stop!
            if (time_in_idle_state_ms() > 1000) {
                commanded_runstate = pilot_desired_runstate;
            }
            break;
        case RunState::IDLE:
            // can always switch to idle
            commanded_runstate = pilot_desired_runstate;
            break;
        case RunState::RUN:
            // must have idled for a while before moving to run:
            if (generator_ok_to_run()) {
                commanded_runstate = pilot_desired_runstate;
            }
            break;
        }
    }

    switch (commanded_runstate) {
    case RunState::STOP:
        SRV_Channels::set_output_pwm(SRV_Channel::k_generator_control, SERVO_PWM_STOP);
        break;
    case RunState::IDLE:
        SRV_Channels::set_output_pwm(SRV_Channel::k_generator_control, SERVO_PWM_IDLE);
        break;
    case RunState::RUN:
        SRV_Channels::set_output_pwm(SRV_Channel::k_generator_control, SERVO_PWM_RUN);
        break;
    }
}

#if HAL_LOGGING_ENABLED
// log generator status to the onboard log
void AP_Generator_RichenPower::Log_Write()
{
#define MASK_LOG_ANY                    0xFFFF
    if (!AP::logger().should_log(MASK_LOG_ANY)) {
        return;
    }
    if (last_logged_reading_ms == last_reading_ms) {
        return;
    }
    last_logged_reading_ms = last_reading_ms;

    AP::logger().WriteStreaming(
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
#endif

// generator prearm checks; notably, if we never see a generator we do
// not run the checks.  Generators are attached/detached at will, and
// reconfiguring is painful.
bool AP_Generator_RichenPower::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    if (uart == nullptr) {
        // not configured in serial manager
        return true;
    }
    if (last_reading_ms == 0) {
        // allow optional use of generator
        return true;
    }

    const uint32_t now = AP_HAL::millis();

    if (now - last_reading_ms > 2000) { // we expect @1Hz
        hal.util->snprintf(failmsg, failmsg_len, "no messages in %ums", unsigned(now - last_reading_ms));
        return false;
    }
    if (SRV_Channels::get_channel_for(SRV_Channel::k_generator_control) == nullptr) {
        hal.util->snprintf(failmsg, failmsg_len, "need a servo output channel");
        return false;
    }

    uint16_t errors = last_reading.errors;

    // requiring maintenance isn't something that should stop
    // people flying - they have work to do.  But we definitely
    // complain about it - a lot.
    errors &= ~(1U << uint16_t(Errors::MaintenanceRequired));

    if (errors) {

        for (uint16_t i=0; i<16; i++) {
            if (errors & (1U << i)) {
                if (i < (uint16_t)Errors::LAST) {
                    hal.util->snprintf(failmsg, failmsg_len, "error: %s", error_strings[i]);
                } else {
                    hal.util->snprintf(failmsg, failmsg_len, "unknown error: 1U<<%u", i);
                }
            }
        }
        return false;
    }
    if (pilot_desired_runstate != RunState::RUN) {
        hal.util->snprintf(failmsg, failmsg_len, "requested state is not RUN");
        return false;
    }
    if (commanded_runstate != RunState::RUN) {
        hal.util->snprintf(failmsg, failmsg_len, "Generator warming up (%.0f%%)", (heat *100 / heat_required_for_run()));
        return false;
    }
    if (last_reading.mode != Mode::RUN &&
        last_reading.mode != Mode::CHARGE &&
        last_reading.mode != Mode::BALANCE) {
        hal.util->snprintf(failmsg, failmsg_len, "not running");
        return false;
    }

    return true;
}

// Update front end with voltage, current, and rpm values
void AP_Generator_RichenPower::update_frontend_readings(void)
{
    _voltage = last_reading.output_voltage;
    _current = last_reading.output_current;
    _rpm = last_reading.rpm;

    update_frontend();
}


// healthy returns true if the generator is not present, or it is
// present, providing telemetry and not indicating an errors.
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

//send mavlink generator status
void AP_Generator_RichenPower::send_generator_status(const GCS_MAVLINK &channel)
{
    if (last_reading_ms == 0) {
        // nothing to report
        return;
    }

    uint64_t status = 0;
    if (last_reading.rpm == 0) {
        status |= MAV_GENERATOR_STATUS_FLAG_OFF;
    } else {
        switch (last_reading.mode) {
        case Mode::OFF:
            status |= MAV_GENERATOR_STATUS_FLAG_OFF;
            break;
        case Mode::IDLE:
            if (pilot_desired_runstate == RunState::RUN) {
                status |= MAV_GENERATOR_STATUS_FLAG_WARMING_UP;
            } else {
                status |= MAV_GENERATOR_STATUS_FLAG_IDLE;
            }
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
        INT16_MAX, // generator temperature
        last_reading.runtime,
        (int32_t)last_reading.seconds_until_maintenance
        );
}

// methods to control the generator state:
bool AP_Generator_RichenPower::stop() 
{
    set_pilot_desired_runstate(RunState::STOP); 
    return true;
}

bool AP_Generator_RichenPower::idle()
{
    set_pilot_desired_runstate(RunState::IDLE);
    return true;
}

bool AP_Generator_RichenPower::run()
{
    set_pilot_desired_runstate(RunState::RUN);
    return true;
}
#endif  // AP_GENERATOR_RICHENPOWER_ENABLED
