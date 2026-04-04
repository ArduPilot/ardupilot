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

// return true if the requested types of data are available
bool AP_Servo_Telem::TelemetryData::present(const uint16_t type_mask) const volatile
{
    return (present_types & type_mask) != 0;
}

// record an update to the telemetry data together with timestamp
// callback to update the data in the frontend, should be called by the driver when new data is available
void AP_Servo_Telem::update_telem_data(const uint8_t servo_index, const TelemetryData& new_data)
{
    // telemetry data are not protected by a semaphore even though updated from different threads
    // each element is a primitive type and the timestamp is only updated at the end, thus a caller
    // can only get slightly more up-to-date information that perhaps they were expecting or might
    // read data that has just gone stale - both of these are safe and avoid the overhead of locking

    if (servo_index >= ARRAY_SIZE(_telem_data) || (new_data.present_types == 0)) {
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
    if (new_data.present(TelemetryData::Types::STATUS)) {
        telemdata.status_flags = new_data.status_flags;
    }

    telemdata.present_types |= new_data.present_types;
    telemdata.last_update_ms = AP_HAL::millis();
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
            position    : telemdata.present(TelemetryData::Types::MEASURED_POSITION)  ? telemdata.measured_position             : AP_Logger::quiet_nanf(),
            force       : telemdata.present(TelemetryData::Types::FORCE)              ? telemdata.force                         : AP_Logger::quiet_nanf(),
            speed       : telemdata.present(TelemetryData::Types::SPEED)              ? telemdata.speed                         : AP_Logger::quiet_nanf(),
            power_pct   : telemdata.duty_cycle,
            pos_cmd     : telemdata.present(TelemetryData::Types::COMMANDED_POSITION) ? telemdata.command_position              : AP_Logger::quiet_nanf(),
            voltage     : telemdata.present(TelemetryData::Types::VOLTAGE)            ? telemdata.voltage                       : AP_Logger::quiet_nanf(),
            current     : telemdata.present(TelemetryData::Types::CURRENT)            ? telemdata.current                       : AP_Logger::quiet_nanf(),
            mot_temp    : telemdata.present(TelemetryData::Types::MOTOR_TEMP)         ? telemdata.motor_temperature_cdeg * 0.01 : AP_Logger::quiet_nanf(),
            pcb_temp    : telemdata.present(TelemetryData::Types::PCB_TEMP)           ? telemdata.pcb_temperature_cdeg * 0.01   : AP_Logger::quiet_nanf(),
            error       : telemdata.status_flags,
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }

}
#endif  // HAL_LOGGING_ENABLED

// Fill in telem structure if telem is available, return false if not
bool AP_Servo_Telem::get_telem(const uint8_t servo_index, TelemetryData& telem) const volatile
{
    // Check for valid index
    if (servo_index >= ARRAY_SIZE(_telem_data)) {
        return false;
    }

    // Check if data has ever been received for the servo index provided
    if ((active_mask & (1U << servo_index)) == 0) {
        return false;
    }

    telem = *const_cast<TelemetryData*>(&_telem_data[servo_index]);
    return true;
}

// Get the AP_Servo_Telem singleton
AP_Servo_Telem *AP_Servo_Telem::get_singleton()
{
    return AP_Servo_Telem::_singleton;
}

#endif // AP_SERVO_TELEM_ENABLED
