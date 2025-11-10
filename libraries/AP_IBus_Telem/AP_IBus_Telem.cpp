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

#include <AP_IBus_Telem/AP_IBus_Telem.h>

#if AP_IBUS_TELEM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RPM/AP_RPM.h>

// 2-byte values
#define IBUS_SENSOR_TYPE_TEMPERATURE          0x01 // Temperature (in 0.1 degrees, where 0=-40'C)
#define IBUS_SENSOR_TYPE_RPM_FLYSKY           0x02 // FlySky-specific throttle value
#define IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE     0x03 // External voltage (in centivolts, so 1450 is 14.50V)
#define IBUS_SENSOR_TYPE_AVERAGE_CELL_VOLTAGE 0x04 // Avg cell voltage (in centivolts, so 1450 is 14.50V)
#define IBUS_SENSOR_TYPE_BATTERY_CURRENT      0x05 // Battery current (centi-amps)
#define IBUS_SENSOR_TYPE_FUEL                 0x06 // Remaining battery percentage
#define IBUS_SENSOR_TYPE_RPM                  0x07 // Throttle value (in 0.01, so 1200 is 12.00%)
#define IBUS_SENSOR_TYPE_COMPASS_HEADING      0x08 // Heading (0-360 degrees)
#define IBUS_SENSOR_TYPE_CLIMB_RATE           0x09 // Climb rate (cm/s)
#define IBUS_SENSOR_TYPE_COG                  0x0a // Course over ground (centidegrees, so 27015 is 270.15 degrees)
#define IBUS_SENSOR_TYPE_GPS_STATUS           0x0b // GPS status (2 values: fix type, and number of satellites)
#define IBUS_SENSOR_TYPE_ACC_X                0x0c // Acc X (cm/s)
#define IBUS_SENSOR_TYPE_ACC_Y                0x0d // Acc Y (cm/s)
#define IBUS_SENSOR_TYPE_ACC_Z                0x0e // Acc Z (cm/s)
#define IBUS_SENSOR_TYPE_ROLL                 0x0f // Roll (centidegrees)
#define IBUS_SENSOR_TYPE_PITCH                0x10 // Pitch (centidegrees)
#define IBUS_SENSOR_TYPE_YAW                  0x11 // Yaw (centidegrees)
#define IBUS_SENSOR_TYPE_VERTICAL_SPEED       0x12 // Vertical speed (cm/s)
#define IBUS_SENSOR_TYPE_GROUND_SPEED         0x13 // Speed (cm/s)
#define IBUS_SENSOR_TYPE_GPS_DIST             0x14 // Distance from home (m)
#define IBUS_SENSOR_TYPE_ARMED                0x15 // Armed / unarmed (1 = armed, 0 = unarmed)
#define IBUS_SENSOR_TYPE_FLIGHT_MODE          0x16 // Flight mode
#define IBUS_SENSOR_TYPE_ODO1                 0x7c // Odometer1
#define IBUS_SENSOR_TYPE_ODO2                 0x7d // Odometer2
#define IBUS_SENSOR_TYPE_SPEED                0x7e // Speed km/h
#define IBUS_SENSOR_TYPE_ALT_FLYSKY           0xf9 // FlySky-specific altitude (metres)

// 4-byte values
#define IBUS_SENSOR_TYPE_TEMPERATURE_PRESSURE 0x41 // Combined temperature & pressure value
#define IBUS_SENSOR_TYPE_GPS_LAT              0x80 // WGS84 in degrees * 1E7
#define IBUS_SENSOR_TYPE_GPS_LNG              0x81 // WGS84 in degrees * 1E7
#define IBUS_SENSOR_TYPE_GPS_ALT              0x82 // GPS (cm)
#define IBUS_SENSOR_TYPE_ALT                  0x83 // Alt (cm)
#define IBUS_SENSOR_TYPE_ALT_MAX              0x84 // MaxAlt (cm)

// i-BUS vehicle modes
#define IBUS_VEHICLE_MODE_STAB    0
#define IBUS_VEHICLE_MODE_ACRO    1
#define IBUS_VEHICLE_MODE_AHOLD   2
#define IBUS_VEHICLE_MODE_AUTO    3
#define IBUS_VEHICLE_MODE_GUIDED  4
#define IBUS_VEHICLE_MODE_LOITER  5
#define IBUS_VEHICLE_MODE_RTL     6
#define IBUS_VEHICLE_MODE_CIRCLE  7
#define IBUS_VEHICLE_MODE_PHOLD   8
#define IBUS_VEHICLE_MODE_LAND    9
#define IBUS_VEHICLE_MODE_UNKNOWN 255 // Must be positive and 0 is already used; out of range blanks the value

// All the sensors we can accurately provide are listed here.
// i-BUS will generally only query up to 15 sensors, so subjectively
// higher-value sensors are sorted to the top to make the most of a
// small telemetry window. In the future these could be configurable.
static const AP_IBus_Telem::SensorDefinition sensors[] {
#if AP_ARMING_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_ARMED, .sensor_length = 2},
#endif
    {.sensor_type = IBUS_SENSOR_TYPE_FLIGHT_MODE, .sensor_length = 2},
#if AP_GPS_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_GPS_STATUS, .sensor_length = 2},
#endif
#if AP_BATTERY_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_FUEL, .sensor_length = 2},
    {.sensor_type = IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE, .sensor_length = 2},
#endif
#if AP_BARO_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_ALT, .sensor_length = 4},
#endif
#if AP_AHRS_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_GPS_DIST, .sensor_length = 2},
#endif
#if AP_BARO_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_CLIMB_RATE, .sensor_length = 2},
#endif
#if AP_GPS_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_GROUND_SPEED, .sensor_length = 2},
#endif
#if AP_AHRS_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_ROLL, .sensor_length = 2},
    {.sensor_type = IBUS_SENSOR_TYPE_PITCH, .sensor_length = 2},
    {.sensor_type = IBUS_SENSOR_TYPE_YAW, .sensor_length = 2},
#endif
#if AP_AIRSPEED_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_SPEED, .sensor_length = 2},
#endif
#if AP_BARO_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_TEMPERATURE_PRESSURE, .sensor_length = 4},
#endif
#if AP_RPM_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_RPM, .sensor_length = 2},
#endif
#if AP_BATTERY_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_BATTERY_CURRENT, .sensor_length = 2},
    {.sensor_type = IBUS_SENSOR_TYPE_AVERAGE_CELL_VOLTAGE, .sensor_length = 2},
#endif
#if AP_AHRS_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_COMPASS_HEADING, .sensor_length = 2},
#endif
#if AP_GPS_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_COG, .sensor_length = 2},
    {.sensor_type = IBUS_SENSOR_TYPE_GPS_LAT, .sensor_length = 4},
    {.sensor_type = IBUS_SENSOR_TYPE_GPS_LNG, .sensor_length = 4},
    {.sensor_type = IBUS_SENSOR_TYPE_GPS_ALT, .sensor_length = 4},
#endif
#if AP_AHRS_ENABLED
    {.sensor_type = IBUS_SENSOR_TYPE_ACC_X, .sensor_length = 2},
    {.sensor_type = IBUS_SENSOR_TYPE_ACC_Y, .sensor_length = 2},
    {.sensor_type = IBUS_SENSOR_TYPE_ACC_Z, .sensor_length = 2},
#endif
};

#if APM_BUILD_TYPE(APM_BUILD_Rover)

/* Rover modes:
    MANUAL       = 0
    ACRO         = 1
    STEERING     = 3
    HOLD         = 4
    LOITER       = 5
    FOLLOW       = 6
    SIMPLE       = 7
    DOCK         = 8
    CIRCLE       = 9
    AUTO         = 10
    RTL          = 11
    SMART_RTL    = 12
    GUIDED       = 15
    INITIALISING = 16
*/
static const AP_IBus_Telem::ModeMap mode_map[] {
    {.ap_mode = 1, .ibus_mode = IBUS_VEHICLE_MODE_ACRO},
    {.ap_mode = 4, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
    {.ap_mode = 5, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
    {.ap_mode = 9, .ibus_mode = IBUS_VEHICLE_MODE_CIRCLE},
    {.ap_mode = 10, .ibus_mode = IBUS_VEHICLE_MODE_AUTO},
    {.ap_mode = 11, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
    {.ap_mode = 12, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
    {.ap_mode = 15, .ibus_mode = IBUS_VEHICLE_MODE_GUIDED},
};

#elif APM_BUILD_COPTER_OR_HELI

/* Copter modes:
    STABILIZE    =  0
    ACRO         =  1
    ALT_HOLD     =  2
    AUTO         =  3
    GUIDED       =  4
    LOITER       =  5
    RTL          =  6
    CIRCLE       =  7
    LAND         =  9
    DRIFT        = 11
    SPORT        = 13
    FLIP         = 14
    AUTOTUNE     = 15
    POSHOLD      = 16
    BRAKE        = 17
    THROW        = 18
    AVOID_ADSB   = 19
    GUIDED_NOGPS = 20
    SMART_RTL    = 21
    FLOWHOLD     = 22
    FOLLOW       = 23
    ZIGZAG       = 24
    SYSTEMID     = 25
    AUTOROTATE   = 26
    AUTO_RTL     = 27
    TURTLE       = 28
*/
static const AP_IBus_Telem::ModeMap mode_map[] {
    {.ap_mode = 0, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
    {.ap_mode = 1, .ibus_mode = IBUS_VEHICLE_MODE_ACRO},
    {.ap_mode = 2, .ibus_mode = IBUS_VEHICLE_MODE_AHOLD},
    {.ap_mode = 3, .ibus_mode = IBUS_VEHICLE_MODE_AUTO},
    {.ap_mode = 4, .ibus_mode = IBUS_VEHICLE_MODE_GUIDED},
    {.ap_mode = 5, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
    {.ap_mode = 6, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
    {.ap_mode = 7, .ibus_mode = IBUS_VEHICLE_MODE_CIRCLE},
    {.ap_mode = 9, .ibus_mode = IBUS_VEHICLE_MODE_LAND},
    {.ap_mode = 16, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
    {.ap_mode = 21, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
    {.ap_mode = 22, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
    {.ap_mode = 27, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
};

#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)

/* Plane modes:
    MANUAL        = 0
    CIRCLE        = 1
    STABILIZE     = 2
    TRAINING      = 3
    ACRO          = 4
    FLY_BY_WIRE_A = 5
    FLY_BY_WIRE_B = 6
    CRUISE        = 7
    AUTOTUNE      = 8
    AUTO          = 10
    RTL           = 11
    LOITER        = 12
    TAKEOFF       = 13
    AVOID_ADSB    = 14
    GUIDED        = 15
    INITIALISING  = 16
    QSTABILIZE    = 17
    QHOVER        = 18
    QLOITER       = 19
    QLAND         = 20
    QRTL          = 21
    QAUTOTUNE     = 22
    QACRO         = 23
    THERMAL       = 24
    LOITER_ALT_QLAND = 25
*/
static const AP_IBus_Telem::ModeMap mode_map[] {
    {.ap_mode = 1, .ibus_mode = IBUS_VEHICLE_MODE_CIRCLE},
    {.ap_mode = 2, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
    {.ap_mode = 4, .ibus_mode = IBUS_VEHICLE_MODE_ACRO},
    {.ap_mode = 5, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
    {.ap_mode = 6, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
    {.ap_mode = 7, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
    {.ap_mode = 10, .ibus_mode = IBUS_VEHICLE_MODE_AUTO},
    {.ap_mode = 11, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
    {.ap_mode = 12, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
    {.ap_mode = 13, .ibus_mode = IBUS_VEHICLE_MODE_AUTO},
    {.ap_mode = 15, .ibus_mode = IBUS_VEHICLE_MODE_GUIDED},
    {.ap_mode = 17, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
    {.ap_mode = 18, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
    {.ap_mode = 19, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
    {.ap_mode = 20, .ibus_mode = IBUS_VEHICLE_MODE_LAND},
    {.ap_mode = 21, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
    {.ap_mode = 23, .ibus_mode = IBUS_VEHICLE_MODE_ACRO},
    {.ap_mode = 25, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
};

#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)

/* Submarine modes:
    STABILIZE    = 0
    ACRO         = 1
    ALT_HOLD     = 2
    AUTO         = 3
    GUIDED       = 4
    CIRCLE       = 7
    SURFACE      = 9
    POSHOLD      = 16
    MANUAL       = 19
    MOTOR_DETECT = 20
*/
static const AP_IBus_Telem::ModeMap mode_map[] {
    {.ap_mode = 0, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
    {.ap_mode = 1, .ibus_mode = IBUS_VEHICLE_MODE_ACRO},
    {.ap_mode = 2, .ibus_mode = IBUS_VEHICLE_MODE_AHOLD},
    {.ap_mode = 3, .ibus_mode = IBUS_VEHICLE_MODE_AUTO},
    {.ap_mode = 4, .ibus_mode = IBUS_VEHICLE_MODE_GUIDED},
    {.ap_mode = 5, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
    {.ap_mode = 6, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
    {.ap_mode = 7, .ibus_mode = IBUS_VEHICLE_MODE_CIRCLE},
    {.ap_mode = 8, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
    {.ap_mode = 9, .ibus_mode = IBUS_VEHICLE_MODE_LAND},
    {.ap_mode = 16, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
};

#elif APM_BUILD_TYPE(APM_BUILD_Blimp)

/* Blimp modes:
    LAND     = 0
    MANUAL   = 1
    VELOCITY = 2
    LOITER   = 3
    RTL      = 4
*/
static const AP_IBus_Telem::ModeMap mode_map[] {
    {.ap_mode = 0, .ibus_mode = IBUS_VEHICLE_MODE_LAND},
    {.ap_mode = 3, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
    {.ap_mode = 4, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
};

#else

static const AP_IBus_Telem::ModeMap mode_map[] {};

#endif

void AP_IBus_Telem::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    if ((port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IBUS_Telem, 0))) {
        port->set_options(port->OPTION_HDPLEX);
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_IBus_Telem::tick, void));
    }
}

void AP_IBus_Telem::tick(void)
{
    if (!initialized) {
        port->begin(115200);
        initialized = true;
    }

    const uint16_t available = MIN(port->available(), 1024U);
    if (available == 0) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if ((now - last_received_message_time_ms) < PROTOCOL_TIMEGAP) {
        // Any bytes that arrive too soon since the last message should be discarded
        port->discard_input();
        return;
    }

    union {
        CommandPacket incoming_message;
        uint8_t buffer[sizeof(CommandPacket)];
    } u;

    if (available < sizeof(u.incoming_message)) {
        // We've got here mid-message; come back in the next loop when it might be complete
        return;
    }

    last_received_message_time_ms = now;

    if (available > sizeof(u.incoming_message)) {
        // We've received too many bytes, so probably a malformed message
        port->discard_input();
        return;
    }

    if (port->read(u.buffer, sizeof(u.buffer)) != sizeof(u.buffer)) {
        // Despite all our checks this is still the wrong size; safest to wait until the next message
        port->discard_input();
        return;
    }

    if (u.incoming_message.message_length != PROTOCOL_INCOMING_MESSAGE_LENGTH) {
        // Every message starts with this fixed message length; if not then it's not valid
        return;
    }

    uint16_t calculated_checksum = 0xFFFF;
    for (uint8_t i = 0; i < sizeof(u.buffer) - 2; i++) {
        calculated_checksum -= u.buffer[i];
    }

    if (u.incoming_message.checksum != calculated_checksum) {
        return;
    }

    handle_incoming_message(u.incoming_message);
}

void AP_IBus_Telem::handle_incoming_message(const CommandPacket &incoming)
{
    // Sensor 0 is reserved for the transmitter, so if for some reason it's requested we should ignore it
    if (incoming.sensor_id == 0) {
        return;
    }

    // If we've reached the end of our sensor list, we shouldn't respond; this tells the receiver that there
    // are no more sensors to discover.
    if (incoming.sensor_id > ARRAY_SIZE(sensors)) {
        return;
    }

    const auto &sensor_definition = sensors[incoming.sensor_id - 1];

    switch (incoming.command << 4) {
    case PROTOCOL_COMMAND_DISCOVER:
        handle_discover_command(incoming);
        break;

    case PROTOCOL_COMMAND_TYPE:
        handle_type_command(incoming, sensor_definition);
        break;

    case PROTOCOL_COMMAND_VALUE:
        handle_value_command(incoming, sensor_definition);
        break;
    }
}

/* A discovery query has the following format:
   * 0x04: Message length
   * 0x81: 0x80 for discovery + 0x01 for sensor ID 1
   * 0x7A: Checksum low byte
   * 0xFF: Checksum high byte
   Checksums are 0xFFFF minus the sum of the previous bytes

   To acknowledge a discovery query, we echo the command back.
 */
void AP_IBus_Telem::handle_discover_command(const CommandPacket &incoming)
{
    struct protocol_command_discover_response_t {
        uint8_t command_length;
        uint8_t address;
        uint16_t checksum;
    } packet {
        PROTOCOL_FOUR_LENGTH,
        (uint8_t)(PROTOCOL_COMMAND_DISCOVER | incoming.sensor_id)
    };
    populate_checksum((uint8_t*)&packet, sizeof(packet));
    port->write((uint8_t*)&packet, sizeof(packet));
}

/* A type query has the following format:
   * 0x04: Message length
   * 0x91: 0x90 for type + 0x01 for sensor ID 1
   * 0x6A: Checksum low byte
   * 0xFF: Checksum high byte
   Checksums are 0xFFFF minus the sum of the previous bytes

   To respond to a type query, we send:
   * 0x06: Message length
   * 0x91: 0x90 for type + 0x01 for sensor ID 1
   * 0x03: Sensor type, eg 0x03 for external voltage
   * 0x02: Sensor length (2 or 4 bytes)
   * 0x63: Checksum low byte
   * 0xFF: Checksum high byte
   Checksums are 0xFFFF minus the sum of the previous bytes
 */
void AP_IBus_Telem::handle_type_command(const CommandPacket &incoming, const SensorDefinition &sensor)
{
    struct protocol_command_type_response_t {
        uint8_t command_length;
        uint8_t address;
        uint8_t type;
        uint8_t length;
        uint16_t checksum;
    } packet {
        PROTOCOL_SIX_LENGTH,
        (uint8_t)(PROTOCOL_COMMAND_TYPE | incoming.sensor_id),
        sensor.sensor_type,
        sensor.sensor_length
    };
    populate_checksum((uint8_t*)&packet, sizeof(packet));
    port->write((uint8_t*)&packet, sizeof(packet));
}

/* A value query has the following format:
   * 0x04: Message length
   * 0xA1: 0xA0 for value + 0x01 for sensor ID 1
   * 0x5A: Checksum low byte
   * 0xFF: Checksum high byte
   Checksums are 0xFFFF minus the sum of the previous bytes

   To respond to a value query, we send:
   * 0x06: Message length (or 0x08 for a 4-byte sensor)
   * 0xA1: 0xA1 for value + 0x01 for sensor ID 1
   * 0xD4: Value byte (value is 12,500 in this example)
   * 0x30: Value byte
   * 0x54: Checksum low byte
   * 0xFE: Checksum high byte
   Checksums are 0xFFFF minus the sum of the previous bytes
 */
void AP_IBus_Telem::handle_value_command(const CommandPacket &incoming, const SensorDefinition &sensor)
{
    const SensorValue value = get_sensor_value(sensor.sensor_type);
    if (sensor.sensor_length == 2) {
        handle_2_byte_value_command(incoming, value);
    } else {
        handle_4_byte_value_command(incoming, value);
    }
}

void AP_IBus_Telem::handle_2_byte_value_command(const CommandPacket &incoming, const SensorValue &value)
{
    struct protocol_command_2byte_value_response_t {
        uint8_t command_length;
        uint8_t address;
        uint8_t value[2];
        uint16_t checksum;
    } packet {
        PROTOCOL_SIX_LENGTH,
        (uint8_t)(PROTOCOL_COMMAND_VALUE | incoming.sensor_id),
        {value.byte[0], value.byte[1]}
    };
    populate_checksum((uint8_t*)&packet, sizeof(packet));
    port->write((uint8_t*)&packet, sizeof(packet));
}

void AP_IBus_Telem::handle_4_byte_value_command(const CommandPacket &incoming, const SensorValue &value)
{
    struct protocol_command_4byte_value_response_t {
        uint8_t command_length;
        uint8_t address;
        uint8_t value[4];
        uint16_t checksum;
    } packet {
        PROTOCOL_EIGHT_LENGTH,
        (uint8_t)(PROTOCOL_COMMAND_VALUE | incoming.sensor_id),
        {value.byte[0], value.byte[1], value.byte[2], value.byte[3]}
    };
    populate_checksum((uint8_t*)&packet, sizeof(packet));
    port->write((uint8_t*)&packet, sizeof(packet));
}

AP_IBus_Telem::SensorValue AP_IBus_Telem::get_sensor_value(const uint8_t sensor_type)
{
    SensorValue value;
    switch (sensor_type) {
#if AP_BATTERY_ENABLED
    case IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE:
        value.uint16 = AP::battery().voltage() * 100;
        break;

    case IBUS_SENSOR_TYPE_AVERAGE_CELL_VOLTAGE:
        value.uint16 = get_average_cell_voltage_cV();
        break;

    case IBUS_SENSOR_TYPE_BATTERY_CURRENT:
        value.uint16 = get_current_cAh();
        break;

    case IBUS_SENSOR_TYPE_FUEL:
        value.uint16 = get_battery_or_fuel_level_pct();
        break;
#endif // AP_BATTERY_ENABLED

#if AP_RPM_ENABLED
    case IBUS_SENSOR_TYPE_RPM:
    case IBUS_SENSOR_TYPE_RPM_FLYSKY:
        value.uint16 = get_rpm();
        break;
#endif

#if AP_AHRS_ENABLED
    case IBUS_SENSOR_TYPE_COMPASS_HEADING:
        value.uint16 = uint16_t(AP::ahrs().get_yaw_deg());
        break;
#endif

#if AP_BARO_ENABLED
    case IBUS_SENSOR_TYPE_CLIMB_RATE:
    case IBUS_SENSOR_TYPE_VERTICAL_SPEED:
        value.int16 = AP::baro().get_climb_rate() * 100;
        break;
#endif

#if AP_GPS_ENABLED
    case IBUS_SENSOR_TYPE_COG:
        value.uint16 = AP::gps().ground_course() * 100;
        break;

    case IBUS_SENSOR_TYPE_GPS_STATUS:
        value.byte[0] = get_gps_status();
        value.byte[1] = AP::gps().num_sats();
        break;
#endif // AP_GPS_ENABLED

#if AP_AHRS_ENABLED
    case IBUS_SENSOR_TYPE_ACC_X:
        value.int16 = (AP::ahrs().get_accel() - AP::ahrs().get_accel_bias()).x * 1000;
        break;

    case IBUS_SENSOR_TYPE_ACC_Y:
        value.int16 = (AP::ahrs().get_accel() - AP::ahrs().get_accel_bias()).y * 1000;
        break;

    case IBUS_SENSOR_TYPE_ACC_Z:
        value.int16 = (AP::ahrs().get_accel() - AP::ahrs().get_accel_bias()).z * 1000;
        break;

    case IBUS_SENSOR_TYPE_ROLL:
        value.int16 = AP::ahrs().roll_sensor;
        break;

    case IBUS_SENSOR_TYPE_PITCH:
        value.int16 = AP::ahrs().pitch_sensor;
        break;

    case IBUS_SENSOR_TYPE_YAW:
        value.int16 = AP::ahrs().yaw_sensor;
        break;
#endif // AP_AHRS_ENABLED

#if AP_GPS_ENABLED
    case IBUS_SENSOR_TYPE_GROUND_SPEED:
        value.uint16 = AP::gps().ground_speed()*100;
        break;
#endif

#if AP_AHRS_ENABLED
    case IBUS_SENSOR_TYPE_GPS_DIST:
        value.uint16 = get_distance_from_home_m();
        break;
#endif

#if AP_ARMING_ENABLED
    case IBUS_SENSOR_TYPE_ARMED:
        value.uint16 = AP::arming().is_armed();
        break;
#endif

    case IBUS_SENSOR_TYPE_FLIGHT_MODE:
        value.uint16 = get_vehicle_mode();
        break;

#if AP_AIRSPEED_ENABLED
    case IBUS_SENSOR_TYPE_SPEED:
        value.uint16 = AP::airspeed()->get_airspeed();
        break;
#endif

#if AP_BARO_ENABLED
    case IBUS_SENSOR_TYPE_TEMPERATURE_PRESSURE: {
        const uint32_t pressure = AP::baro().get_pressure();
        const uint32_t temperature = (AP::baro().get_temperature() + 40) * 10;
        value.uint32 = pressure | (temperature << 19);
        break;
    }
#endif

#if AP_GPS_ENABLED
    case IBUS_SENSOR_TYPE_GPS_LAT:
        value.int32 = AP::gps().location().lat;
        break;

    case IBUS_SENSOR_TYPE_GPS_LNG:
        value.int32 = AP::gps().location().lng;
        break;

    case IBUS_SENSOR_TYPE_GPS_ALT:
        value.int32 = AP::gps().location().alt;
        break;
#endif // AP_GPS_ENABLED

#if AP_BARO_ENABLED
    case IBUS_SENSOR_TYPE_ALT:
        value.int32 = AP::baro().get_altitude() * 100;
        break;
#endif

    default:
        value.int32 = 0;
    }

    return value;
}

#if AP_BATTERY_ENABLED
uint16_t AP_IBus_Telem::get_average_cell_voltage_cV()
{
    if (!AP::battery().has_cell_voltages()) {
        return 0;
    }

    const auto &cell_voltages = AP::battery().get_cell_voltages();
    const uint8_t number_of_cells = ARRAY_SIZE(cell_voltages.cells);
    if (number_of_cells == 0) {
        return 0;
    }

    uint32_t voltage_sum = 0;
    for (auto i = 0; i < number_of_cells; i++) {
        voltage_sum += cell_voltages.cells[i] * 0.001;
    }
    return voltage_sum / number_of_cells * 100;

}

uint16_t AP_IBus_Telem::get_current_cAh()
{
    float current = 0;
    IGNORE_RETURN(AP::battery().current_amps(current));
    return current * 100;
}

uint8_t AP_IBus_Telem::get_battery_or_fuel_level_pct()
{
    uint8_t percentage = 0;
    IGNORE_RETURN(AP::battery().capacity_remaining_pct(percentage));
    return percentage;
}
#endif // AP_BATTERY_ENABLED

#if AP_RPM_ENABLED
uint16_t AP_IBus_Telem::get_rpm()
{
    const AP_RPM *rpm = AP::rpm();
    float rpm_value;
    if (rpm && rpm->get_rpm(0, rpm_value)) {
        return rpm_value;
    }

    return 0;
}
#endif

#if AP_GPS_ENABLED
uint8_t AP_IBus_Telem::get_gps_status()
{
    if (!AP::gps().is_healthy()) {
        return 0;
    }

    const AP_GPS::GPS_Status gps_status = AP::gps().status();
    if (gps_status >= AP_GPS::GPS_OK_FIX_3D) {
        return 3;
    } else if (gps_status >= AP_GPS::GPS_OK_FIX_2D) {
        return 2;
    } else if (gps_status == AP_GPS::NO_FIX) {
        return 1;
    } else {
        return 0;
    }
}
#endif // AP_GPS_ENABLED

#if AP_AHRS_ENABLED
uint16_t AP_IBus_Telem::get_distance_from_home_m()
{
    Vector2f home;
    if (AP::ahrs().get_relative_position_NE_home(home)) {
        return home.length();
    }

    return 0;
}
#endif

uint16_t AP_IBus_Telem::get_vehicle_mode()
{
    const uint8_t vehicle_mode = AP::vehicle()->get_mode();
    for (uint8_t i = 0; i < ARRAY_SIZE(mode_map); i++) {
        if (mode_map[i].ap_mode == vehicle_mode) {
            return mode_map[i].ibus_mode;
        }
    }
    return IBUS_VEHICLE_MODE_UNKNOWN;
}

// Populate the last two bytes of packet with the checksum of the preceding bytes
void AP_IBus_Telem::populate_checksum(uint8_t *packet, const uint16_t size)
{
    uint16_t checksum = 0xFFFF;

    for (int i=0; i<size-2; i++) {
        checksum -= packet[i];
    }

    packet[size-2] = checksum & 0x0ff;
    packet[size-1] = checksum >> 8;
}

#endif
