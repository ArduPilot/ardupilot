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
#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>

#include "AP_Frsky_Backend.h"

#define FRSKY_TELEM_PAYLOAD_STATUS_CAPACITY          5 // size of the message buffer queue (max number of messages waiting to be sent)

/* 
for FrSky D protocol (D-receivers)
*/
// FrSky sensor hub data IDs
#define DATA_ID_GPS_ALT_BP          0x01
#define DATA_ID_TEMP1               0x02
#define DATA_ID_FUEL                0x04
#define DATA_ID_TEMP2               0x05
#define DATA_ID_GPS_ALT_AP          0x09
#define DATA_ID_BARO_ALT_BP         0x10
#define DATA_ID_GPS_SPEED_BP        0x11
#define DATA_ID_GPS_LONG_BP         0x12
#define DATA_ID_GPS_LAT_BP          0x13
#define DATA_ID_GPS_COURS_BP        0x14
#define DATA_ID_GPS_SPEED_AP        0x19
#define DATA_ID_GPS_LONG_AP         0x1A
#define DATA_ID_GPS_LAT_AP          0x1B
#define DATA_ID_BARO_ALT_AP         0x21
#define DATA_ID_GPS_LONG_EW         0x22
#define DATA_ID_GPS_LAT_NS          0x23
#define DATA_ID_CURRENT             0x28
#define DATA_ID_VARIO               0x30
#define DATA_ID_VFAS                0x39

#define START_STOP_D                0x5E
#define BYTESTUFF_D                 0x5D

/* 
for FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
*/
// FrSky Sensor IDs
#define SENSOR_ID_VARIO             0x00 // Sensor ID  0
#define SENSOR_ID_FAS               0x22 // Sensor ID  2
#define SENSOR_ID_GPS               0x83 // Sensor ID  3
#define SENSOR_ID_SP2UR             0xC6 // Sensor ID  6
#define SENSOR_ID_27                0x1B // Sensor ID 27

// FrSky data IDs
#define GPS_LONG_LATI_FIRST_ID      0x0800
#define DIY_FIRST_ID                0x5000

#define FRAME_HEAD                  0x7E
#define FRAME_DLE                   0x7D
#define FRAME_XOR                   0x20

#define SPORT_DATA_FRAME            0x10
/* 
for FrSky SPort Passthrough
*/
// data bits preparation
// for parameter data
#define PARAM_ID_OFFSET             24
#define PARAM_VALUE_LIMIT           0xFFFFFF
// for gps status data
#define GPS_SATS_LIMIT              0xF
#define GPS_STATUS_LIMIT            0x3
#define GPS_STATUS_OFFSET           4
#define GPS_HDOP_OFFSET             6
#define GPS_ADVSTATUS_OFFSET        14
#define GPS_ALTMSL_OFFSET           22
// for battery data
#define BATT_VOLTAGE_LIMIT          0x1FF
#define BATT_CURRENT_OFFSET         9
#define BATT_TOTALMAH_LIMIT         0x7FFF
#define BATT_TOTALMAH_OFFSET        17
// for autopilot status data
#define AP_CONTROL_MODE_LIMIT       0x1F
#define AP_SIMPLE_OFFSET            5
#define AP_SSIMPLE_OFFSET           6
#define AP_FLYING_OFFSET            7
#define AP_ARMED_OFFSET             8
#define AP_BATT_FS_OFFSET           9
#define AP_EKF_FS_OFFSET            10
#define AP_IMU_TEMP_MIN             19.0f
#define AP_IMU_TEMP_MAX             82.0f
#define AP_IMU_TEMP_OFFSET          26
// for home position related data
#define HOME_ALT_OFFSET             12
#define HOME_BEARING_LIMIT          0x7F
#define HOME_BEARING_OFFSET         25
// for velocity and yaw data
#define VELANDYAW_XYVEL_OFFSET      9
#define VELANDYAW_YAW_LIMIT         0x7FF
#define VELANDYAW_YAW_OFFSET        17
// for attitude (roll, pitch) and range data
#define ATTIANDRNG_ROLL_LIMIT       0x7FF
#define ATTIANDRNG_PITCH_LIMIT      0x3FF
#define ATTIANDRNG_PITCH_OFFSET     11
#define ATTIANDRNG_RNGFND_OFFSET    21

class AP_Frsky_Telem {
public:
    AP_Frsky_Telem();

    ~AP_Frsky_Telem();

    /* Do not allow copies */
    AP_Frsky_Telem(const AP_Frsky_Telem &other) = delete;
    AP_Frsky_Telem &operator=(const AP_Frsky_Telem&) = delete;

    // init - perform required initialisation
    bool init(bool use_external_data=false);

    static AP_Frsky_Telem *get_singleton(void) {
        return singleton;
    }

    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data);

    void queue_message(MAV_SEVERITY severity, const char *text) {
        if (_backend == nullptr) {
            return;
        }
        return _backend->queue_text_message(severity, text);
    }

private:

    AP_Frsky_Backend *_backend;

    // get next telemetry data for external consumers of SPort data (internal function)
    bool _get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data);

    static AP_Frsky_Telem *singleton;

};

namespace AP {
    AP_Frsky_Telem *frsky_telem();
};
