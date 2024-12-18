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

#include "AP_Servo_Telem.h"

#if AP_SERVO_TELEM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

AP_Servo_Telem *AP_Servo_Telem::_singleton;

AP_Servo_Telem::AP_Servo_Telem()
{
    if (_singleton) {
        AP_HAL::panic("Too many AP_Servo_Telem instances");
    }
    _singleton = this;
}

// return true if the data is stale
bool AP_Servo_Telem::TelemetryData::stale(uint32_t now_ms) const volatile
{
    return (last_update_ms == 0) || ((now_ms - last_update_ms) > 5000);
}

// return true if the requested types of data are available
bool AP_Servo_Telem::TelemetryData::present(const uint16_t type_mask) const volatile
{
    return (valid_types & type_mask) != 0;
}

// return true if the requested types of data are available and not stale
bool AP_Servo_Telem::TelemetryData::valid(const uint16_t type_mask) const volatile
{
    return present(type_mask) && !stale(AP_HAL::millis());
}

// record an update to the telemetry data together with timestamp
// callback to update the data in the frontend, should be called by the driver when new data is available
void AP_Servo_Telem::update_telem_data(const uint8_t servo_index, const TelemetryData& new_data)
{
    // telemetry data are not protected by a semaphore even though updated from different threads
    // each element is a primitive type and the timestamp is only updated at the end, thus a caller
    // can only get slightly more up-to-date information that perhaps they were expecting or might
    // read data that has just gone stale - both of these are safe and avoid the overhead of locking

    if (servo_index >= ARRAY_SIZE(_telem_data) || (new_data.valid_types == 0)) {
        return;
    }
    active_mask |= 1U << servo_index;

    volatile TelemetryData &telemdata = _telem_data[servo_index];

    if (new_data.present(TelemetryData::Types::COMMANDED_POSITION)) {
        telemdata.command_position = new_data.command_position;
    }
    if (new_data.present(TelemetryData::Types::MEASURED_POSITION)) {
        telemdata.measured_position = new_data.measured_position;
    }
    if (new_data.present(TelemetryData::Types::FORCE)) {
        telemdata.force = new_data.force;
    }
    if (new_data.present(TelemetryData::Types::SPEED)) {
        telemdata.speed = new_data.speed;
    }
    if (new_data.present(TelemetryData::Types::VOLTAGE)) {
        telemdata.voltage = new_data.voltage;
    }
    if (new_data.present(TelemetryData::Types::CURRENT)) {
        telemdata.current = new_data.current;
    }
    if (new_data.present(TelemetryData::Types::DUTY_CYCLE)) {
        telemdata.duty_cycle = new_data.duty_cycle;
    }
    if (new_data.present(TelemetryData::Types::MOTOR_TEMP)) {
        telemdata.motor_temperature_cdeg = new_data.motor_temperature_cdeg;
    }
    if (new_data.present(TelemetryData::Types::PCB_TEMP)) {
        telemdata.pcb_temperature_cdeg = new_data.pcb_temperature_cdeg;
    }
    if (new_data.present(TelemetryData::Types::PCB_TEMP)) {
        telemdata.status_flags = new_data.status_flags;
    }

    telemdata.valid_types |= new_data.valid_types;
    telemdata.last_update_ms = AP_HAL::millis();
}

// Return the commanded position servo position in degrees if available
bool AP_Servo_Telem::get_commanded_position(const uint8_t servo_index, float &command_position) const
{
    if (!data_available(servo_index, TelemetryData::Types::COMMANDED_POSITION)) {
        return false;
    }

    command_position = _telem_data[servo_index].command_position;
    return true;
}

// Return the measured Servo position in degrees if available
bool AP_Servo_Telem::get_measured_position(const uint8_t servo_index, float &measured_position) const
{
    if (!data_available(servo_index, TelemetryData::Types::MEASURED_POSITION)) {
        return false;
    }

    measured_position = _telem_data[servo_index].measured_position;
    return true;
}

// Return the force in newton meters if available
bool AP_Servo_Telem::get_force(const uint8_t servo_index, float &force) const
{
    if (!data_available(servo_index, TelemetryData::Types::FORCE)) {
        return false;
    }

    force = _telem_data[servo_index].force;
    return true;
}

// Return the speed in degrees per second if available
bool AP_Servo_Telem::get_speed(const uint8_t servo_index, float &speed) const
{
    if (!data_available(servo_index, TelemetryData::Types::SPEED)) {
        return false;
    }

    speed = _telem_data[servo_index].speed;
    return true;
}

// Return the voltage in volts per second if available
bool AP_Servo_Telem::get_voltage(const uint8_t servo_index, float &voltage) const
{
    if (!data_available(servo_index, TelemetryData::Types::VOLTAGE)) {
        return false;
    }

    voltage = _telem_data[servo_index].voltage;
    return true;
}

// Return the current in amps per second if available
bool AP_Servo_Telem::get_current(const uint8_t servo_index, float &current) const
{
    if (!data_available(servo_index, TelemetryData::Types::CURRENT)) {
        return false;
    }

    current = _telem_data[servo_index].current;
    return true;
}


// Return the duty cycle 0% to 100% if available
bool AP_Servo_Telem::get_duty_cycle(const uint8_t servo_index, uint8_t &duty_cycle) const
{
    if (!data_available(servo_index, TelemetryData::Types::DUTY_CYCLE)) {
        return false;
    }

    duty_cycle = _telem_data[servo_index].duty_cycle;
    return true;
}

// Return the motor temperature in degrees C if available
bool AP_Servo_Telem::get_motor_temperature(const uint8_t servo_index, float &motor_temperature) const
{
    if (!data_available(servo_index, TelemetryData::Types::MOTOR_TEMP)) {
        return false;
    }

    motor_temperature = _telem_data[servo_index].motor_temperature_cdeg * 0.01;
    return true;
}

// Return the pcb temperature in degrees C if available
bool AP_Servo_Telem::get_pcb_temperature(const uint8_t servo_index, float &pcb_temperature) const
{
    if (!data_available(servo_index, TelemetryData::Types::PCB_TEMP)) {
        return false;
    }

    pcb_temperature = _telem_data[servo_index].pcb_temperature_cdeg * 0.01;
    return true;
}

// Return type specific status flags if available
bool AP_Servo_Telem::get_status_flags(const uint8_t servo_index, uint8_t &status_flags) const
{
    if (!data_available(servo_index, TelemetryData::Types::STATUS)) {
        return false;
    }

    status_flags = _telem_data[servo_index].status_flags;
    return true;
}

// Helper to check index and if data is available
bool AP_Servo_Telem::data_available(const uint8_t servo_index, const TelemetryData::Types type) const
{
    if (servo_index >= SERVO_TELEM_MAX_SERVOS) {
        return false;
    }

    if (!_telem_data[servo_index].valid(type)) {
        return false;
    }

    return true;
}

void AP_Servo_Telem::update()
{
#if HAL_LOGGING_ENABLED
    write_log();
#endif
}

#if HAL_LOGGING_ENABLED
void AP_Servo_Telem::write_log()
{
    AP_Logger *logger = AP_Logger::get_singleton();

    // Check logging is available and enabled
    if ((logger == nullptr) || !logger->logging_enabled() || active_mask == 0) {
        return;
    }

    const uint64_t now_us = AP_HAL::micros64();

    for (uint8_t i = 0; i < ARRAY_SIZE(_telem_data); i++) {
        const volatile TelemetryData &telemdata = _telem_data[i];

        if (telemdata.last_update_ms == _last_telem_log_ms[i]) {
            // No new data since last log call, skip
            continue;
        }

        // Update last log timestamp
        _last_telem_log_ms[i] = telemdata.last_update_ms;

        // Log, use nan for float values which are not available
        const struct log_CSRV pkt {
            LOG_PACKET_HEADER_INIT(LOG_CSRV_MSG),
            time_us     : now_us,
            id          : i,
            position    : telemdata.present(TelemetryData::Types::MEASURED_POSITION)  ? telemdata.measured_position             : AP::logger().quiet_nanf(),
            force       : telemdata.present(TelemetryData::Types::FORCE)              ? telemdata.force                         : AP::logger().quiet_nanf(),
            speed       : telemdata.present(TelemetryData::Types::SPEED)              ? telemdata.speed                         : AP::logger().quiet_nanf(),
            power_pct   : telemdata.duty_cycle,
            pos_cmd     : telemdata.present(TelemetryData::Types::COMMANDED_POSITION) ? telemdata.command_position              : AP::logger().quiet_nanf(),
            voltage     : telemdata.present(TelemetryData::Types::VOLTAGE)            ? telemdata.voltage                       : AP::logger().quiet_nanf(),
            current     : telemdata.present(TelemetryData::Types::CURRENT)            ? telemdata.current                       : AP::logger().quiet_nanf(),
            mot_temp    : telemdata.present(TelemetryData::Types::MOTOR_TEMP)         ? telemdata.motor_temperature_cdeg * 0.01 : AP::logger().quiet_nanf(),
            pcb_temp    : telemdata.present(TelemetryData::Types::PCB_TEMP)           ? telemdata.pcb_temperature_cdeg * 0.01   : AP::logger().quiet_nanf(),
            error       : telemdata.status_flags,
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

}
#endif  // HAL_LOGGING_ENABLED

// Get the AP_Servo_Telem singleton
AP_Servo_Telem *AP_Servo_Telem::get_singleton()
{
    return AP_Servo_Telem::_singleton;
}

#endif // AP_SERVO_TELEM_ENABLED
