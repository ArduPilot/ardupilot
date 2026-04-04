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
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#pragma GCC optimize("Os")

#include "AP_Generator_config.h"

#if AP_GENERATOR_CORTEX_ENABLED

#include "AP_Generator_Cortex.h"
#include <AP_PiccoloCAN/AP_PiccoloCAN_Device.h>

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_Param/AP_Param.h>

extern const AP_HAL::HAL& hal;

AP_Generator_Cortex* AP_Generator_Cortex::_singleton;


void AP_Generator_Cortex::init()
{
    _singleton = this;

    // Inform frontend which measurements are available for this generator
    _frontend._has_current = true;
    _frontend._has_consumed_energy = false;
    _frontend._has_fuel_remaining = false;
}


// Decode received CAN frame    
bool AP_Generator_Cortex::handle_message(AP_HAL::CANFrame &frame, AP_PiccoloCAN& can_iface)
{
    // Store a pointer to the CAN interface
    if (_can_iface == nullptr) {
        _can_iface = &can_iface;
    }

    if (decodeCortex_TelemetryStatusPacketStructure(&frame, &telemetry.status)) {
    } else if (decodeCortex_TelemetryGeneratorPacketStructure(&frame, &telemetry.generator)) {
    } else if (decodeCortex_TelemetryBatteryPacketStructure(&frame, &telemetry.battery)) {
    } else if (decodeCortex_TelemetryOutputRailPacketStructure(&frame, &telemetry.rails)) {
    } else if (decodeCortex_TelemetryControllerPacketStructure(&frame, &telemetry.controller)) {
    } else {
        // No matching packet
        return false;
    }

    // Extract device type and node ID from the CAN frame
    _can_device_id = frame.id & 0xFF;
    _can_device_type = (frame.id >> 8) & 0xFF;

    last_reading_ms = AP_HAL::millis();

    return true;
}


void AP_Generator_Cortex::update()
{
    if (connected != is_connected()) {
        connected = is_connected();

        if (connected) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Cortex generator connected");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Cortex generator disconnected");
        }
    }

    // Update internal readings
    _voltage = telemetry.generator.voltage;
    _current = telemetry.generator.current;
    _rpm = (uint16_t) abs(telemetry.generator.rpm);

    update_frontend();
}


bool AP_Generator_Cortex::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    if (!is_connected()) {
        hal.util->snprintf(failmsg, failmsg_len, "Generator is not connected");
        return false;
    }

    if (is_inhibited()) {
        hal.util->snprintf(failmsg, failmsg_len, "Generator is inhibited");
        return false;
    }

    if (!is_ready()) {
        hal.util->snprintf(failmsg, failmsg_len, "Generator is not ready");
        return false;
    }

    return true;
}


void AP_Generator_Cortex::send_generator_status(const GCS_MAVLINK &channel)
{
    uint64_t status_flags = 0;

    if (!is_connected()) {
        return;
    }

    // Copy status flags from MAV_GENERATOR_STATUS_FLAG enum
    const Cortex_StatusBits_t &status = telemetry.status.status;
    const Cortex_WarningBits_t &warning = telemetry.status.warning;
    const Cortex_ErrorBits_t &error = telemetry.status.error;

    // Check inihibit state
    if (status.inhibited) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_START_INHIBITED;
    }
    
    switch (status.mode) {
        case CORTEX_MODE_BOOTUP:
            break;
        case CORTEX_MODE_STANDBY:
            status_flags |= MAV_GENERATOR_STATUS_FLAG_IDLE;
            if (status.readyToRun) {
                status_flags |= MAV_GENERATOR_STATUS_FLAG_READY;
            }
            break;
        case CORTEX_MODE_PREFLIGHT:
            break;
        case CORTEX_MODE_CRANKING:
            break;
        case CORTEX_MODE_RESERVED:
            break;
        case CORTEX_MODE_RUNNING:
                if (telemetry.generator.current <= 0.05f) {
                    status_flags |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
                }
            break;
    }

    // Status flags
    if (status.readyToRun) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_READY;
    }

    if (telemetry.battery.current <= 0.1f) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
    }

    if (status.powerLimit) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER;
    }

    // Warning flags
    if (warning.generatorTempLimit) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING;
    }

    if (warning.temperature) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING;
    }

    if (warning.enginePowerLimit) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_MAXPOWER;
    }
    
    // Error flags
    if (error.generator) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT;
    }

    if (error.avionicsRail ||
        error.payloadRail ||
        error.servoRail ||
        error.batteryChargerRail) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT;
    }

    mavlink_msg_generator_status_send(
        channel.get_chan(),
        status_flags,
        (uint16_t) abs(telemetry.generator.rpm),
        batteryCurrent(),
        loadCurrent(),
        generatorPower(),
        generatorVoltage(),
        rectifierTemperature(),
        std::numeric_limits<double>::quiet_NaN(),
        telemetry.generator.temperature,
        telemetry.controller.runTime,
        0  // time until maintenance (not supported)
    );
}


bool AP_Generator_Cortex::is_connected(void) const
{
    const uint32_t now = AP_HAL::millis();

    return (now - last_reading_ms) < 1000;
}


int16_t AP_Generator_Cortex::rectifierTemperature(void) const
{
    const Cortex_TelemetryController_t &controller = telemetry.controller;

    // Return the maximum of the two internal temperature values
    return MAX(controller.rectifierTemperature, controller.regulatorTemperature);
}


bool AP_Generator_Cortex::send_message(AP_HAL::CANFrame &frame)
{

    if (_can_iface == nullptr) {
        return false;
    }

    if (!is_connected()) {
        return false;
    }

    // Encode stored device type and node ID
    frame.id |= _can_device_id;
    frame.id |= (_can_device_type << 8) & 0xFF00;

    return _can_iface->write_frame(frame, 1000);
}


// Send the "standby" command to the Cortex generator
bool AP_Generator_Cortex::stop(void)
{
    // Note: If the engine is not also disabled, the generator may restart automatically
    if (!is_connected()) {
        return false;
    }

    AP_HAL::CANFrame txFrame {};

    encodeCortex_StandbyPacket(&txFrame);
    return send_message(txFrame);
}


// Send the "preflight" command to the Cortex generator
bool AP_Generator_Cortex::idle(void)
{
    if (!is_connected()) {
        return false;
    }

    AP_HAL::CANFrame txFrame {};

    encodeCortex_PreflightPacket(&txFrame);

    return send_message(txFrame);
}


// Send the "crank engine" command to the Cortex generator
bool AP_Generator_Cortex::run(void)
{
    if (!is_connected()) {
        return false;
    }

    AP_HAL::CANFrame txFrame {};

    encodeCortex_StartCrankingPacket(&txFrame);

    return send_message(txFrame);
}

#endif // AP_GENERATOR_CORTEX_ENABLED
