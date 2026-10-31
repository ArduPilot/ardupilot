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
        last_rail_reading_ms = AP_HAL::millis();
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

#if HAL_LOGGING_ENABLED
    Log_Write();
#endif
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
            status_flags |= MAV_GENERATOR_STATUS_FLAG_READY;
            break;
    }

    // Status flags
    if (status.readyToRun) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_READY;
    }

    if (telemetry.battery.current <= 0.1f) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
    }

    if (telemetry.generator.current <= 0.05f) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
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


#if HAL_LOGGING_ENABLED
uint8_t AP_Generator_Cortex::get_logging_warning_mask(void) const
{
    const Cortex_WarningBits_t &w = telemetry.status.warning;
    return (w.generator              ? (1U << 0) : 0)
         | (w.temperature            ? (1U << 1) : 0)
         | (w.generatorTempLimit     ? (1U << 2) : 0)
         | (w.generatorCurrentLimit  ? (1U << 3) : 0)
         | (w.enginePowerLimit       ? (1U << 4) : 0)
         | (w.powerLoss              ? (1U << 5) : 0)
         | (w.lowRpm                 ? (1U << 6) : 0)
         | (w.batteryChargeCurrentLimit ? (1U << 7) : 0);
}


uint16_t AP_Generator_Cortex::get_logging_error_mask(void) const
{
    const Cortex_ErrorBits_t &e = telemetry.status.error;
    return (e.generator         ? (1U << 0) : 0)
         | (e.cranking          ? (1U << 1) : 0)
         | (e.avionicsRail      ? (1U << 2) : 0)
         | (e.payloadRail       ? (1U << 3) : 0)
         | (e.servoRail         ? (1U << 4) : 0)
         | (e.batteryChargerRail ? (1U << 5) : 0)
         | (e.powerMap          ? (1U << 6) : 0)
         | (e.lowRpm            ? (1U << 7) : 0)
         | (e.powerLoss         ? (1U << 8) : 0);
}


void AP_Generator_Cortex::Log_Write(void)
{
    if (!AP::logger().should_log(0xFFFF)) {
        return;
    }

    if (_last_logged_reading_ms == last_reading_ms) {
        return;
    }
    _last_logged_reading_ms = last_reading_ms;

// @LoggerMessage: CRTX
// @Description: Cortex generator telemetry
// @Field: TimeUS: Time since system startup
// @Field: Mode: Cortex operational mode
// @Field: RPM: Generator speed
// @Field: GVolt: Generator voltage
// @Field: GCurr: Generator current
// @Field: GTemp: Generator temperature
// @Field: BVolt: Battery voltage
// @Field: BCurr: Battery current
// @Field: BTemp: Battery temperature
// @Field: RTemp: Rectifier/regulator temperature (maximum of the two)
// @Field: RunTm: Generator cumulative run time
// @Field: Warn: Warning bitmask (bit0=generator,1=temperature,2=generatorTempLimit,3=generatorCurrentLimit,4=enginePowerLimit,5=powerLoss,6=lowRpm,7=batteryChargeCurrentLimit)
// @Field: Err: Error bitmask (bit0=generator,1=cranking,2=avionicsRail,3=payloadRail,4=servoRail,5=batteryChargerRail,6=powerMap,7=lowRpm,8=powerLoss)
    AP::logger().WriteStreaming(
        "CRTX",
        "TimeUS,Mode,RPM,GVolt,GCurr,GTemp,BVolt,BCurr,BTemp,RTemp,RunTm,Warn,Err",
        "s--VAOVAOOs--",
        "F------------",
        "QBhfffffffIBH",
        AP_HAL::micros64(),
        (uint8_t)telemetry.status.status.mode,
        telemetry.generator.rpm,
        telemetry.generator.voltage,
        telemetry.generator.current,
        telemetry.generator.temperature,
        telemetry.battery.voltage,
        telemetry.battery.current,
        telemetry.battery.temperature,
        (float)rectifierTemperature(),
        telemetry.controller.runTime,
        get_logging_warning_mask(),
        get_logging_error_mask()
    );

    // Only write rail status when we have new data,
    // not all Cortex devices support output rails
    if (last_rail_reading_ms == _last_logged_rail_reading_ms) {
        return;
    }

    _last_logged_rail_reading_ms = last_rail_reading_ms;

// @LoggerMessage: CRTR
// @Description: Cortex generator output rail telemetry
// @Field: TimeUS: Time since system startup
// @Field: AvV: Avionics rail voltage
// @Field: AvI: Avionics rail current
// @Field: PaV: Payload rail voltage
// @Field: PaI: Payload rail current
// @Field: SvV: Servo rail voltage
// @Field: SvI: Servo rail current
    AP::logger().WriteStreaming(
        "CRTR",
        "TimeUS,AvV,AvI,PaV,PaI,SvV,SvI",
        "sVAVAVA",
        "F------",
        "Qffffff",
        AP_HAL::micros64(),
        telemetry.rails.avionicsVoltage,
        telemetry.rails.avionicsCurrent,
        telemetry.rails.payloadVoltage,
        telemetry.rails.payloadCurrent,
        telemetry.rails.servoVoltage,
        telemetry.rails.servoCurrent
    );
}
#endif  // HAL_LOGGING_ENABLED

#endif // AP_GENERATOR_CORTEX_ENABLED
