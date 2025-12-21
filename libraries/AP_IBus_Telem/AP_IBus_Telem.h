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


   i-BUS telemetry for FlySky/Turnigy receivers and other peripherals
   (eg iA6B, iA10) by Nicole Ashley <nicole@ashley.kiwi>.

   Originally based on work by Jan Verhulst:
     https://github.com/ArduPilot/ardupilot/pull/16545

   Libraries used for reference and inspiration:

   * iBUStelemetry
     https://github.com/Hrastovc/iBUStelemetry

   * IBusBM
     https://github.com/bmellink/IBusBM

   * BetaFlight
     https://github.com/betaflight/betaflight/blob/master/src/main/telemetry/ibus_shared.c
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_IBUS_TELEM_ENABLED
#define AP_IBUS_TELEM_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 2048
#endif

#if AP_IBUS_TELEM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/vector3.h>
#include <AP_AHRS/AP_AHRS_config.h>
#include <AP_BattMonitor/AP_BattMonitor_config.h>
#include <AP_GPS/AP_GPS_config.h>
#include <AP_RPM/AP_RPM_config.h>

class AP_IBus_Telem
{
public:
    AP_IBus_Telem() {}

    CLASS_NO_COPY(AP_IBus_Telem);

    void init();

    typedef struct {
        uint8_t sensor_type;   // Sensor type (IBUS_SENSOR_TYPE_* above)
        uint8_t sensor_length; // Data length for defined sensor (can be 2 or 4 bytes)
    } SensorDefinition;

    typedef struct {
        uint8_t ap_mode;
        uint8_t ibus_mode;
    } ModeMap;

private:

    typedef union {
        uint16_t uint16;
        uint32_t uint32;
        int16_t int16;
        int32_t int32;
        uint8_t byte[4];
    } SensorValue;

    struct PACKED CommandPacket {
        uint8_t message_length;
        uint8_t sensor_id : 4;
        uint8_t command : 4;
        uint16_t checksum;
    };

    void tick();
    void handle_incoming_message(const CommandPacket &incoming);
    void handle_discover_command(const CommandPacket &incoming);
    void handle_type_command(const CommandPacket &incoming, const SensorDefinition &sensor);
    void handle_value_command(const CommandPacket &incoming, const SensorDefinition &sensor);
    void handle_2_byte_value_command(const CommandPacket &incoming, const SensorValue &value);
    void handle_4_byte_value_command(const CommandPacket &incoming, const SensorValue &value);
    SensorValue get_sensor_value(uint8_t sensor_id);
#if AP_BATTERY_ENABLED
    uint16_t get_average_cell_voltage_cV();
    uint16_t get_current_cAh();
    uint8_t get_battery_or_fuel_level_pct();
#endif
#if AP_RPM_ENABLED
    uint16_t get_rpm();
#endif
#if AP_GPS_ENABLED
    uint8_t get_gps_status();
#endif
#if AP_AHRS_ENABLED
    uint16_t get_distance_from_home_m();
#endif
    uint16_t get_vehicle_mode();
    void populate_checksum(uint8_t *packet, const uint16_t size);

    static const uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80; // Command to discover a sensor (lowest 4 bits are sensor)
    static const uint8_t PROTOCOL_COMMAND_TYPE = 0x90;     // Command to request a sensor type (lowest 4 bits are sensor)
    static const uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;    // Command to request a sensor's value (lowest 4 bits are sensor)
    static const uint8_t PROTOCOL_TIMEGAP = 3;             // Packets are received very ~7ms so use ~half that for the gap
    static const uint8_t PROTOCOL_FOUR_LENGTH = 0x04;      // indicate that the message has 4 bytes
    static const uint8_t PROTOCOL_SIX_LENGTH = 0x06;       // indicate that the message has 6 bytes
    static const uint8_t PROTOCOL_EIGHT_LENGTH = 0x08;     // indicate that the message has 8 bytes
    static const uint8_t PROTOCOL_INCOMING_MESSAGE_LENGTH = PROTOCOL_FOUR_LENGTH; // All incoming messages are the same length

    AP_HAL::UARTDriver *port;
    bool initialized;
    uint32_t last_received_message_time_ms; // When the last message was received
};

#endif
