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
/*
 *
 * Libraries or parts of the code used in this project:
 *
 * iBUStelemetry
 * https://github.com/Hrastovc/iBUStelemetry
 *
 * IBusBM
 * https://github.com/bmellink/IBusBM
 *
 * Read more about the protocol on:
 * https://github.com/betaflight/betaflight/wiki/Single-wire-FlySky-(IBus)-telemetry
 *
 *
 * Big thanks to the authors!
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_IBUS_TELEM_ENABLED
#define HAL_IBUS_TELEM_ENABLED 1 // default enabled
#endif

#if HAL_IBUS_TELEM_ENABLED

#define IBUS_MEAS_TYPE_TEM            0x01 // Temperature (in 0.1 degrees, where 0=-40'C)
#define IBUS_MEAS_TYPE_EXTV           0x03 // External voltage (in 0.01, so 1450 is 14.50V)
#define IBUS_MEAS_TYPE_CELL           0x04 // Avg cell voltage (in 0.01, so 1450 is 14.50V)
#define IBUS_MEAS_TYPE_BAT_CURR       0x05 // Battery current
#define IBUS_MEAS_TYPE_FUEL           0x06 // Remaining battery percentage
#define IBUS_MEAS_TYPE_RPM            0x07 // Throttle value / battery capacity
#define IBUS_MEAS_TYPE_CMP_HEAD       0x08 // Heading
#define IBUS_MEAS_TYPE_CLIMB_RATE     0x09 // Climb rate
#define IBUS_MEAS_TYPE_COG            0x0a // Course over ground
#define IBUS_MEAS_TYPE_GPS_STATUS     0x0b // GPS status (2 values)
#define IBUS_MEAS_TYPE_ACC_X          0x0c // Acc X
#define IBUS_MEAS_TYPE_ACC_Y          0x0d // Acc Y
#define IBUS_MEAS_TYPE_ACC_Z          0x0e // Acc Z
#define IBUS_MEAS_TYPE_ROLL           0x0f // Roll
#define IBUS_MEAS_TYPE_PITCH          0x10 // Pitch
#define IBUS_MEAS_TYPE_YAW            0x11 // Yaw
#define IBUS_MEAS_TYPE_VERTICAL_SPEED 0x12 // Vertical speed
#define IBUS_MEAS_TYPE_GROUND_SPEED   0x13 // Speed m/s
#define IBUS_MEAS_TYPE_GPS_DIST       0x14 // Distance from home
#define IBUS_MEAS_TYPE_ARMED          0x15 // Armed / unarmed (1 = armed, 0 = unarmed)
#define IBUS_MEAS_TYPE_FLIGHT_MODE    0x16 // Flight mode (see Mode.h for flightmodes in copter)

#define IBUS_MEAS_TYPE_PRES           0x41 // Pressure
#define IBUS_MEAS_TYPE_SPE            0x7e // Speed km/h

class AP_IBus_Telem
{
public:
    // constructor
    AP_IBus_Telem() {}

    /* Do not allow copies */
    AP_IBus_Telem(const AP_IBus_Telem &other) = delete;
    AP_IBus_Telem &operator=(const AP_IBus_Telem &) = delete;

    void init();
    void loop();

private:
    AP_HAL::UARTDriver *_port;
    bool _initialized;

    void setSensorMeasurement(uint8_t adr, int32_t value); // sets a sensor measurement for a certain address
    void setSensorMeasurements(); // set new values for all sensors configured
    void populate_checksum(uint8_t *packet, uint16_t size); //populates the last two bytes with checksum
    void handleTelemetryMessage(); // handles the telemetry message

    enum class State {
        GET_LENGTH,
        GET_DATA,
        GET_CHKSUML,
        GET_CHKSUMH,
        DISCARD
    };

    static const uint8_t PROTOCOL_MAX_LENGTH = 0x08;           // maximum length for sensor frames - 8 bytes
    static const uint8_t PROTOCOL_OVERHEAD = 3;            // packet is <len><cmd><data....><chkl><chkh>, overhead=cmd+chk bytes
    static const uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80; // Command discover sensor (lowest 4 bits are sensor)
    static const uint8_t PROTOCOL_COMMAND_TYPE = 0x90;     // Command discover sensor (lowest 4 bits are sensor)
    static const uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;    // Command send sensor data (lowest 4 bits are sensor)
    static const uint8_t PROTOCOL_TIMEGAP = 3; // Packets are received very ~7ms so use ~half that for the gap
    static const uint16_t UPDATE_MEASUREMENTS_TIMING = 500; // every 500ms
    static const uint8_t PROTOCOL_FOUR_LENGTH = 0x04;      // indicate that the message has 4 bytes
    static const uint8_t PROTOCOL_SIX_LENGTH = 0x06;      // indicate that the message has 6 bytes

    State _state;
    uint32_t _last; // milis() of prior message
    uint32_t _last_send_measurements_time; // in millis time when the last time measurements were updated
    uint8_t _buffer[PROTOCOL_MAX_LENGTH];
    uint8_t _offs;     // pointer in buffer
    uint8_t _len;     // incoming message length
    uint16_t _chksum; // checksum calculation
    uint8_t _lchksum;           // checksum lower byte received

    typedef struct {
        uint8_t sensor_type;   // sensor type (0,1,2,3, etc)
        uint8_t sensor_length; // data length for defined sensor (can be 2 or 4)
        int16_t sensor_value;  // sensor data for defined sensors (16 or 32 bits)
    } _sensorinfo;

    _sensorinfo _sensors[4] = {
        {.sensor_type = IBUS_MEAS_TYPE_EXTV, .sensor_length = 2},
        {.sensor_type = IBUS_MEAS_TYPE_ARMED, .sensor_length = 2},
        {.sensor_type = IBUS_MEAS_TYPE_FLIGHT_MODE, .sensor_length = 2},
        {.sensor_type = IBUS_MEAS_TYPE_GPS_STATUS, .sensor_length = 2},
    };
};

#endif
