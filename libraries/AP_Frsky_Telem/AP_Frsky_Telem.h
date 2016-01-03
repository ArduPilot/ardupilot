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

#ifndef __AP_FRSKY_TELEM_H__
#define __AP_FRSKY_TELEM_H__

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>

#define MSG_BUFFER_LENGTH       8 // size of the message buffer queue (number of messages waiting to be sent)

/* FrSky sensor hub data IDs for D-receiver */
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

/* The X-receiver sends the following bytes on the Smart.Port to poll sensors:
    0x7E 0xA1 0x7E 0x22 0x7E 0x83 0x7E 0xE4 0x7E 0x45 0x7E 0xC6 0x7E 0x67
    0x7E 0x48 0x7E 0xE9 0x7E 0x6A 0x7E 0xCB 0x7E 0xAC 0x7E 0x0D 0x7E 0x8E
    0x7E 0x2F 0x7E 0xD0 0x7E 0x71 0x7E 0xF2 0x7E 0x53 0x7E 0x34 0x7E 0x95
    0x7E 0x16 0x7E 0xB7 0x7E 0x98 0x7E 0x39 0x7E 0xBA 0x7E 0x1B 0x7E 0x00
    The fist byte (0x7E) is the header of the poll request,
    the second byte is the sensor ID for the sensor being polled.
*/
// Standard FrSky SENSOR IDs
enum SPortSensors {
    SENSOR_VARIO     = 0x00, // seen as sensor ID 1
    SENSOR_FLVSS     = 0xA1, // seen as sensor ID 2 (reserved for FLVSS)
    SENSOR_FAS       = 0x22, // sensor ID 3
    SENSOR_GPS       = 0x83, 
    SENSOR_RPM       = 0xE4,
    SENSOR_SP2UH     = 0x45,
    SENSOR_SP2UR     = 0xC6,
    // some other polled IDs
    SENSOR_8         = 0x67,
    SENSOR_9         = 0x48,
    SENSOR_10        = 0xE9,
    SENSOR_11        = 0x6A,
    SENSOR_12        = 0xCB,
    SENSOR_13        = 0xAC,
    SENSOR_14        = 0x0D,
    SENSOR_15        = 0x8E,
    SENSOR_16        = 0x2F,
    SENSOR_17        = 0xD0,
    SENSOR_18        = 0x71,
    SENSOR_19        = 0xF2,
    SENSOR_20        = 0x53,
    SENSOR_21        = 0x34,
    SENSOR_22        = 0x95,
    SENSOR_23        = 0x16,
    SENSOR_24        = 0xB7,
    SENSOR_25        = 0x98,
    SENSOR_26        = 0x39,
    SENSOR_27        = 0xBA,
    SENSOR_28        = 0x1B,
    SENSOR_UNKNOWN   = 0xFE,
    SENSOR_ENUM_END
};

class AP_Frsky_Telem
{
public:
    //constructor
    AP_Frsky_Telem(const AP_InertialNav &inav, const AP_AHRS &ahrs, const AP_BattMonitor &battery, const RangeFinder &rng, const AC_Fence &fence, const AP_RPM &rpm_sensor, const AP_Motors &motors);

    // init - perform required initialisation including detecting which protocol to use and saving parameters
    void init(const AP_SerialManager &serial_manager, const uint8_t mav_type, const char *firmware_str, const char *frame_config_str, float fs_batt_voltage, float fs_batt_mah);

    void set_home_distance(float distance) { _home_distance = distance; }
    void set_home_bearing(float bearing) { _home_bearing = bearing; }
    void set_control_mode(int8_t mode) { _control_mode = (uint8_t)mode; }
    void set_simple_mode(uint8_t simple_mode) { _simple_mode = simple_mode; }
    void set_land_complete(uint8_t land_complete) { _land_complete = (bool)land_complete; }
    void set_control_sensors(uint32_t control_sensors_present, uint32_t control_sensors_enabled, uint32_t control_sensors_health) { _control_sensors_present = control_sensors_present; _control_sensors_enabled = control_sensors_enabled; _control_sensors_health = control_sensors_health; }

private:
    // tick - main call to send updates to transmitter (called by scheduler at 1kHz)
    void tick(void);

    const AP_InertialNav &_inav;
    const AP_AHRS &_ahrs;
    const AP_BattMonitor &_battery;
    const RangeFinder &_rng;
    const AC_Fence &_fence;
    const AP_RPM &_rpm_sensor;
    const AP_Motors &_motors;

	AP_HAL::UARTDriver *_port;                     // UART used to send data to receiver
    AP_SerialManager::SerialProtocol _protocol;    // protocol used - detected using SerialManager's SERIALX_PROTOCOL parameter
    uint16_t _crc;

    struct
    {
	uint8_t mav_type; // frame type (see MAV_TYPE in Mavlink definition file common.h)
    float fs_batt_voltage; // failsafe battery voltage in volts
    float fs_batt_mah; // failsafe reserve capacity in mAh
	int32_t batt_capacity_mah; // battery capacity in mAh as configured by user
    } _params;

    struct
    {
    mavlink_statustext_t message_data[MSG_BUFFER_LENGTH];
    uint8_t msg_queued_index;
    uint8_t msg_sent_index;
    } _mav;

    uint32_t _control_sensors_present;
    uint32_t _control_sensors_enabled;
    uint32_t _control_sensors_health;

    float _home_distance;
    float _home_bearing;
    uint8_t _control_mode;
    uint8_t _simple_mode;
    bool _land_complete;
    struct
    {
    char lat_ns, lon_ew;
    uint16_t latdddmm;
    uint16_t latmmmm;
    uint16_t londddmm;
    uint16_t lonmmmm;    
    uint16_t alt_gps_meters;
    uint16_t alt_gps_cm;
    int16_t speed_in_meter;
    uint16_t speed_in_centimeter;
    } _gps;

    // method for packetizing "status_text" style messages (severity and text) found in queue for transmission through FrSky link
    uint32_t get_next_msg_chunk(bool *chunk_sent);

    // methods for collecting in message queue what is essentially status_text, sys_status, and ekf_status_report mavlink messages
    void queue_message(uint8_t severity, const char *str);
	void mavlink_statustext_check(void);
    void control_sensors_check(void);
    void ekf_status_check(void);

	uint32_t calc_params(bool *params_sent);
	uint32_t calc_gps_latlng(bool *send_latitude);
    uint32_t calc_gps_status(void);
    uint32_t calc_batt(void);
    uint32_t calc_ap_status(void);
    uint32_t calc_home(void);
    uint32_t calc_velandrng(void);
    uint32_t calc_attitude(void);

    uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power);
    void send_XPort(void);
    void send_SPort(void);
    void send_DPort(void);
    float format_gps(float dec);
    void calc_gps_position(void);

    // methods related to the nuts-and-bolts of sending data
    void frsky_calc_crc(uint8_t byte);
    void frsky_send_crc(void);
    void frsky_send_byte(uint8_t byte);
    void frsky_send_data(uint16_t id, uint32_t value);
};
#endif
