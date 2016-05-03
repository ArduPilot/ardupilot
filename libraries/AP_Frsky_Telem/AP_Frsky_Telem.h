// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_SerialManager/AP_SerialManager.h>

/* FrSky sensor hub data IDs
for FrSky D protocol (D-receivers)
*/
#define DATA_ID_GPS_ALT_BP     0x01
#define DATA_ID_TEMP1          0x02
#define DATA_ID_RPM            0x03
#define DATA_ID_FUEL           0x04
#define DATA_ID_TEMP2          0x05
#define DATA_ID_VOLTS          0x06
#define DATA_ID_GPS_ALT_AP     0x09
#define DATA_ID_BARO_ALT_BP    0x10
#define DATA_ID_GPS_SPEED_BP   0x11
#define DATA_ID_GPS_LONG_BP    0x12
#define DATA_ID_GPS_LAT_BP     0x13
#define DATA_ID_GPS_COURS_BP   0x14
#define DATA_ID_GPS_DAY_MONTH  0x15
#define DATA_ID_GPS_YEAR       0x16
#define DATA_ID_GPS_HOUR_MIN   0x17
#define DATA_ID_GPS_SEC        0x18
#define DATA_ID_GPS_SPEED_AP   0x19
#define DATA_ID_GPS_LONG_AP    0x1A
#define DATA_ID_GPS_LAT_AP     0x1B
#define DATA_ID_GPS_COURS_AP   0x1C
#define DATA_ID_BARO_ALT_AP    0x21
#define DATA_ID_GPS_LONG_EW    0x22
#define DATA_ID_GPS_LAT_NS     0x23
#define DATA_ID_ACCEL_X        0x24
#define DATA_ID_ACCEL_Y        0x25
#define DATA_ID_ACCEL_Z        0x26
#define DATA_ID_CURRENT        0x28
#define DATA_ID_VARIO          0x30
#define DATA_ID_VFAS           0x39
#define DATA_ID_VOLTS_BP       0x3A
#define DATA_ID_VOLTS_AP       0x3B

/* Standard FrSky Sensor IDs
for FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
*/
#define SENSOR_ID_VARIO            0x00 // 0
#define SENSOR_ID_FLVSS            0xA1 // 1
#define SENSOR_ID_FAS              0x22 // 2
#define SENSOR_ID_GPS              0x83 // 3
#define SENSOR_ID_RPM              0xE4 // 4
#define SENSOR_ID_SP2UH            0x45 // 5
#define SENSOR_ID_SP2UR            0xC6 // 6

class AP_Frsky_Telem
{
public:
    //constructor
    AP_Frsky_Telem(AP_AHRS &ahrs, const AP_BattMonitor &battery);

    // init - perform require initialisation including detecting which protocol to use
    void init(const AP_SerialManager &serial_manager, uint8_t *control_mode);

private:
    // send_FrSky_SPort - main transmission function when protocol is send_FrSky_SPort
    void send_FrSky_SPort(void);
    // send_FrSky_D - main transmission function when protocol is FrSky_D
    void send_FrSky_D(void);
    
    // tick - main call to send updates to transmitter (called by scheduler at 1kHz)
    void tick(void);

    // methods related to the nuts-and-bolts of sending data
    void frsky_calc_crc(uint8_t byte);
    void frsky_send_crc(void);
    void frsky_send_byte(uint8_t value);
    void frsky_send_data(uint16_t id, uint32_t data);

    // methods to convert flight controller data to frsky telemetry format
    void calc_nav_alt(void);
    float format_gps(float dec);
    void calc_gps_position(void);

    AP_AHRS &_ahrs;                             // reference to attitude estimate
    const AP_BattMonitor &_battery;                   // reference to battery monitor object
    AP_HAL::UARTDriver *_port;                  // UART used to send data to receiver
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter

    uint16_t _crc;

    uint8_t *_control_mode;
    
    struct
    {
    char lat_ns, lon_ew;
    uint16_t latdddmm;
    uint16_t latmmmm;
    uint16_t londddmm;
    uint16_t lonmmmm;
    uint16_t alt_gps_meters;
    uint16_t alt_gps_cm;
    uint16_t alt_nav_meters;
    uint16_t alt_nav_cm;
    int16_t speed_in_meter;
    uint16_t speed_in_centimeter;
    } _gps;
};
