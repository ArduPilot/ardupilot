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

#ifndef __AP_FRSKY_TELEM_H__
#define __AP_FRSKY_TELEM_H__

#include <AP_HAL.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Baro.h>
#include <AP_BattMonitor.h>

/* FrSky sensor hub data IDs */
#define FRSKY_ID_GPS_ALT_BP     0x01
#define FRSKY_ID_TEMP1          0x02
#define FRSKY_ID_RPM            0x03
#define FRSKY_ID_FUEL           0x04
#define FRSKY_ID_TEMP2          0x05
#define FRSKY_ID_VOLTS          0x06
#define FRSKY_ID_GPS_ALT_AP     0x09
#define FRSKY_ID_BARO_ALT_BP    0x10
#define FRSKY_ID_GPS_SPEED_BP   0x11
#define FRSKY_ID_GPS_LONG_BP    0x12
#define FRSKY_ID_GPS_LAT_BP     0x13
#define FRSKY_ID_GPS_COURS_BP   0x14
#define FRSKY_ID_GPS_DAY_MONTH  0x15
#define FRSKY_ID_GPS_YEAR       0x16
#define FRSKY_ID_GPS_HOUR_MIN   0x17
#define FRSKY_ID_GPS_SEC        0x18
#define FRSKY_ID_GPS_SPEED_AP   0x19
#define FRSKY_ID_GPS_LONG_AP    0x1A
#define FRSKY_ID_GPS_LAT_AP     0x1B
#define FRSKY_ID_GPS_COURS_AP   0x1C
#define FRSKY_ID_BARO_ALT_AP    0x21
#define FRSKY_ID_GPS_LONG_EW    0x22
#define FRSKY_ID_GPS_LAT_NS     0x23
#define FRSKY_ID_ACCEL_X        0x24
#define FRSKY_ID_ACCEL_Y        0x25
#define FRSKY_ID_ACCEL_Z        0x26
#define FRSKY_ID_CURRENT        0x28
#define FRSKY_ID_VARIO          0x30
#define FRSKY_ID_VFAS           0x39
#define FRSKY_ID_VOLTS_BP       0x3A
#define FRSKY_ID_VOLTS_AP       0x3B

#define HUB_PORT true
#define S_PORT false

class AP_Frsky_Telem
{
 public:
    //constructor
    AP_Frsky_Telem(AP_AHRS &ahrs, AP_BattMonitor &battery) :
    _initialised(false),
    _ahrs(ahrs),
    _battery(battery)
        {}

    // these enums must match up with TELEM2_PROTOCOL in vehicle code
    enum FrSkyProtocol {
        FrSkyDPORT = 2,
        FrSkySPORT = 3
    };

    void init(AP_HAL::UARTDriver *port, uint8_t frsky_type);
    void send_frames(uint8_t control_mode, enum FrSkyProtocol protocol);


 private:
    void frsky_send_data(uint8_t id, int16_t data);
    void frsky_send_frame1(uint8_t mode);
    void frsky_send_frame2();
    void check_sport_input(void);
	
    float frsky_format_gps(float dec);
    void frsky_send_byte(uint8_t value);
    void frsky_send_hub_startstop();

    AP_HAL::UARTDriver *_port;
    bool _initialised;
    AP_AHRS &_ahrs;
    AP_BattMonitor &_battery;
    uint32_t _last_frame1_ms;
    uint32_t _last_frame2_ms;
};
#endif
