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
   Graupner Hott Telemetry library
   Hott telemetry runs at 19200 8N1 on a non-inverted half-duplex UART

   With thanks to Graupner and betaflight
*/

#include "AP_Hott_Telem.h"

#if HAL_HOTT_TELEM_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Stats/AP_Stats.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>

#define PROT_BINARY   0x80
#define PROT_ID_GAM   0x8D
#define PROT_ID_EAM   0x8E
#define PROT_ID_GPS   0x8A
#define PROT_ID_VARIO 0x89

#define BYTE_DELAY_FIRST_US 4000
#define BYTE_DELAY_US 1200

extern const AP_HAL::HAL& hal;

AP_Hott_Telem *AP_Hott_Telem::singleton;

AP_Hott_Telem::AP_Hott_Telem(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (singleton != nullptr) {
        AP_HAL::panic("AP_Hott_Telem must be singleton");
    }
#endif
    singleton = this;
}

/*
 * initialise uart
 */
void AP_Hott_Telem::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Hott, 0);
    if (uart) {
        // register thread
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Hott_Telem::loop, void),
                                          "Hott",
                                          1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
            DEV_PRINTF("Failed to create Hott thread\n");
        }
    }
}

/*
  send EAM (Electric Air Model)
 */
void AP_Hott_Telem::send_EAM(void)
{
    // EAM message
    struct PACKED {
        uint8_t start_byte = 0x7C;   //#01 start uint8_t
        uint8_t eam_sensor_id = 0x8E;//#02 EAM sensort id. constat value 0x8e
        uint8_t warning_beeps;
        uint8_t sensor_id = 0xE0;
        uint16_t alarms;             //#05 alarm bitmask. Value is displayed inverted
        uint8_t cell_low[7];         //#07 cell voltage lower value. 0.02V steps, 124=2.48V
        uint8_t cell_high[7];        //#14 cell voltage high value. 0.02V steps, 124=2.48V
        uint16_t batt1_voltage;      //#21 battery 1 voltage in 100mv steps
        uint16_t batt2_voltage;      //#23 battery 2 voltage in 100mv steps
        uint8_t temp1;               //#25 Temperature sensor 1. 20=0C, 46=26C - offset of 20.
        uint8_t temp2;               //#26 temperature sensor 2
        uint16_t altitude;           //#27 Attitude unit: meters. Value of 500 = 0m
        uint16_t current;            //#29 Current in 0.1A steps
        uint16_t main_voltage;       //#31 Main power voltage (drive) in 0.1V steps
        uint16_t batt_used;          //#33 used battery capacity in 10mAh steps
        uint16_t climbrate;          //#35 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
        uint8_t climbrate3s;         //#37 climbrate in m/3sec. Value of 120 = 0m/3sec
        uint16_t rpm;                //#38 RPM. Steps: 10 rev/min
        uint8_t electric_min;        //#40 Electric minutes. Time starts when motor current is > 3 A
        uint8_t electric_sec;        //#41
        uint16_t speed;               //#42 speed in km/h. Steps 1km/h
        uint8_t stop_byte = 0x7D;     //#44 stop
    } msg {};

    const AP_BattMonitor &battery = AP::battery();
    if (battery.num_instances() > 0) {
        msg.batt1_voltage = uint16_t(battery.voltage(0) * 10);
    }
    if (battery.num_instances() > 1) {
        msg.batt2_voltage = uint16_t(battery.voltage(1) * 10);
    }
    float current;
    if (battery.current_amps(current)) {
        msg.current = uint16_t(current * 10);
    }
    msg.main_voltage = uint16_t(battery.voltage() * 10);
    float used_mah;
    if (battery.consumed_mah(used_mah)) {
        msg.batt_used = used_mah * 0.1;
    }

    const AP_Baro &baro = AP::baro();
    msg.temp1 = uint8_t(baro.get_temperature(0) + 20.5);
#if BARO_MAX_INSTANCES > 1
    if (baro.healthy(1)) {
        msg.temp2 = uint8_t(baro.get_temperature(1) + 20.5);
    }
#endif

    AP_AHRS &ahrs = AP::ahrs();
    float alt = 0;
    Vector3f vel;
    {
        WITH_SEMAPHORE(ahrs.get_semaphore());
        ahrs.get_relative_position_D_home(alt);
        alt = -alt;
        IGNORE_RETURN(ahrs.get_velocity_NED(vel));
    }
    msg.altitude = uint16_t(500.5 + alt);

    msg.climbrate = uint16_t(30000.5 + vel.z * -100);
    msg.climbrate3s = 120 + vel.z * -3;

#if AP_RPM_ENABLED
    const AP_RPM *rpm = AP::rpm();
    float rpm_value;
    if (rpm && rpm->get_rpm(0, rpm_value)) {
        msg.rpm = rpm_value * 0.1;
    }
#endif

    AP_Stats *stats = AP::stats();
    if (stats) {
        uint32_t t = stats->get_flight_time_s();
        msg.electric_min = t / 60U;
        msg.electric_sec = t % 60U;
    }

#if AP_AIRSPEED_ENABLED
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed && airspeed->healthy()) {
        msg.speed = uint16_t(airspeed->get_airspeed() * 3.6 + 0.5);
    } else {
        WITH_SEMAPHORE(ahrs.get_semaphore());
        msg.speed = uint16_t(ahrs.groundspeed() * 3.6 + 0.5);
    }
#else
    WITH_SEMAPHORE(ahrs.get_semaphore());
    msg.speed = uint16_t(ahrs.groundspeed() * 3.6 + 0.5);
#endif

    send_packet((const uint8_t *)&msg, sizeof(msg));
}

/*
  convert from a GPS lat/lon in decimal degrees to degrees plus decimal minutes
 */
void AP_Hott_Telem::GPS_to_DDM(float decimal, uint8_t &sign, uint16_t &dm, uint16_t &sec) const
{
    sign = decimal>=0?0:1;
    decimal = fabsf(decimal);
    uint8_t deg = uint16_t(decimal);
    uint8_t min = uint16_t((decimal - deg) * 60);
    dm = deg*100 + min;
    sec = (decimal - (deg + min/60.0)) * 60 * 10000 + 0.5;
}

/*
  send GPS packet
 */
void AP_Hott_Telem::send_GPS(void)
{
    // GPS message
    struct PACKED {
        uint8_t start_byte = 0x7c;    //#01 constant value 0x7c
        uint8_t gps_sensor_id = 0x8a; //#02 constant value 0x8a
        uint8_t warning_beeps;        //#03
        uint8_t sensor_id = 0xA0;     //#04 constant (?) value 0xa0
        uint16_t alarm;               //#05
        uint8_t flight_direction;     //#07 flight direction in 2 degreees/step (1 = 2degrees);
        uint16_t gps_speed_kmh;       //#08 km/h
        uint8_t pos_NS;               //#10 north = 0, south = 1
        uint16_t pos_NS_dm;           //#11 degree minutes
        uint16_t pos_NS_sec;          //#13 position seconds
        uint8_t pos_EW;               //#15 east = 0, west = 1
        uint16_t pos_EW_dm;           //#16 degree minutes
        uint16_t pos_EW_sec;          //#18 position seconds
        uint16_t home_distance;       //#20 meters
        uint16_t altitude;            //#22 meters. Value of 500 = 0m
        uint16_t climbrate;           //#24 m/s 0.01m/s resolution. Value of 30000 = 0.00 m/s
        uint8_t climbrate3s;          //#26 climbrate in m/3s resolution, value of 120 = 0 m/3s
        uint8_t gps_satelites;        //#27 sat count
        uint8_t gps_fix_char;         //#28 GPS fix character. display, 'D' = DGPS, '2' = 2D, '3' = 3D, '-' = no fix
        uint8_t home_direction;       //#29 direction from starting point to Model position (2 degree steps)
        int16_t vel_north;            //#30 velocity north mm/s
        uint8_t speed_acc;            //#32 speed accuracy cm/s
        uint8_t gps_time_h;           //#33 UTC time hours
        uint8_t gps_time_m;           //#34 UTC time minutes
        uint8_t gps_time_s;           //#35 UTC time seconds
        uint8_t gps_time_hs;          //#36 UTC time 0.01s units
        int16_t vel_east;             //#37 velocity north mm/s
        uint8_t horiz_acc;            //#39 horizontal accuracy
        uint8_t free_char1;           //#40 displayed to right of home
        uint8_t free_char2;           //#41
        uint8_t free_char3;           //#42 GPS fix character. display, 'D' = DGPS, '2' = 2D, '3' = 3D, '-' = no fix
        uint8_t version = 1;          //#43 0: GPS Graupner #33600, 1: ArduPilot
        uint8_t stop_byte = 0x7d;     //#44
    } msg {};

    AP_GPS &gps = AP::gps();
    Location loc;

    {
        WITH_SEMAPHORE(gps.get_semaphore());
        loc = gps.location();
        msg.flight_direction = uint16_t(gps.ground_course() * 0.5 + 0.5);
        msg.gps_speed_kmh = uint16_t(gps.ground_speed() * 3.6 + 0.5);
        float sacc, hacc;
        if (gps.speed_accuracy(sacc)) {
            msg.speed_acc = sacc * 100 + 0.5;
        }
        if (gps.horizontal_accuracy(hacc)) {
            msg.horiz_acc = hacc * 100 + 0.5;
        }
        msg.gps_satelites = gps.num_sats();
    }

    float lat = loc.lat * 1.0e-7;
    float lon = loc.lng * 1.0e-7;

    uint16_t dm, sec;
    GPS_to_DDM(lat, msg.pos_NS, dm, sec);
    msg.pos_NS_dm = dm;
    msg.pos_NS_sec = sec;

    GPS_to_DDM(lon, msg.pos_EW, dm, sec);
    msg.pos_EW_dm = dm;
    msg.pos_EW_sec = sec;

    AP_AHRS &ahrs = AP::ahrs();
    Vector2f home_vec;
    float alt = 0;
    Vector3f vel;
    {
        WITH_SEMAPHORE(ahrs.get_semaphore());
        if (ahrs.get_relative_position_NE_home(home_vec)) {
            msg.home_distance = home_vec.length();
        }
        ahrs.get_relative_position_D_home(alt);
        alt = -alt;
        IGNORE_RETURN(ahrs.get_velocity_NED(vel));
    }

    msg.climbrate = uint16_t(30000.5 + vel.z * -100);
    msg.climbrate3s = 120 + vel.z * -3;
    msg.vel_north = vel.x * 1000 + 0.5;
    msg.vel_east = vel.y * 1000 + 0.5;
    msg.altitude = uint16_t(500.5 + alt);

    msg.gps_fix_char = gps.status_onechar();
    msg.free_char3 = msg.gps_fix_char;

    msg.home_direction = degrees(atan2f(home_vec.y, home_vec.x)) * 0.5 + 0.5;

    AP_RTC &rtc = AP::rtc();
    {
        WITH_SEMAPHORE(rtc.get_semaphore());
        uint16_t ms;
        rtc.get_system_clock_utc(msg.gps_time_h, msg.gps_time_m, msg.gps_time_s, ms);
    }

    send_packet((const uint8_t *)&msg, sizeof(msg));
}

/*
  send Vario
 */
void AP_Hott_Telem::send_Vario(void)
{
    // Vario message
    struct PACKED {
        uint8_t start_byte = 0x7C;   //#01 start uint8_t
        uint8_t vario_id = 0x89;     //#02 ID
        uint8_t warning_beeps;       //#03 warnings
        uint8_t sensor_id = 0x90;    //#04 sensor ID
        uint8_t inv_status;          //#05 status
        uint16_t altitude;           //#06 Attitude meters. Value of 500 = 0m
        uint16_t altitude_max;       //#08 Attitude max meters. Value of 500 = 0m
        uint16_t altitude_min;       //#10 Attitude min meters. Value of 500 = 0m
        uint16_t climbrate;          //#12 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
        uint16_t climbrate3s;        //#14 climb rate in meters per 3s Value of 30000 = 0.00 m/s
        uint16_t climbrate10s;       //#16 climb rate in meters per 10s. Value of 30000 = 0.00 m/s
        char     text[3][7];         //#18 #Text display
        char     ascii3[3];          //#39 3 extra characters
        uint8_t  yaw;                //#42 yaw in 2 degree units, 0 = north
        uint8_t  version = 1;        //#43 protocol version
        uint8_t  stop_byte = 0x7D;   //#44 stop
    } msg {};

    AP_AHRS &ahrs = AP::ahrs();
    Vector3f vel;
    float alt = 0;
    {
        WITH_SEMAPHORE(ahrs.get_semaphore());
        ahrs.get_relative_position_D_home(alt);
        alt = -alt;
        IGNORE_RETURN(ahrs.get_velocity_NED(vel));
        msg.yaw = wrap_360_cd(ahrs.yaw_sensor) * 0.005;
    }

    min_alt = MIN(alt, min_alt);
    max_alt = MAX(alt, max_alt);

    msg.altitude = uint16_t(500.5 + alt);
    msg.altitude_max = uint16_t(500.5 + max_alt);
    msg.altitude_min = uint16_t(500.5 + min_alt);

    msg.climbrate = 30000.5 + vel.z * -100;
    msg.climbrate3s = 30000.5 + vel.z * -100*3;
    msg.climbrate10s = 30000.5 + vel.z * -100*10;
    
    AP_Notify *notify = AP_Notify::get_singleton();
    char fltmode[5] {};
    if (notify) {
        strncpy(fltmode, notify->get_flight_mode_str(), sizeof(fltmode));
        strncpy(msg.text[0], fltmode, sizeof(msg.text[0]));
    }
    if (hal.util->get_soft_armed()) {
        strncpy(msg.text[1], "ARMED", sizeof(msg.text[1]));
        if (strncmp(fltmode, "AUTO", sizeof(fltmode)) == 0) {
            const AP_Mission *mission = AP::mission();
            if (mission) {
                char wp[10] {};
                snprintf(wp, sizeof(wp), "WP %3u", mission->get_current_nav_index());
                memcpy(msg.text[2], wp, sizeof(msg.text[2]));
            }
        }
    } else {
        strncpy(msg.text[1], "DISARM", sizeof(msg.text[1]));
        const char *ck = AP_Notify::flags.pre_arm_check ? "CK:PASS" : "CK:FAIL";
        memcpy(msg.text[2], ck, MIN(strlen(ck), sizeof(msg.text[2])));
    }

    send_packet((const uint8_t *)&msg, sizeof(msg));
}

/*
  send a packet out
 */
void AP_Hott_Telem::send_packet(const uint8_t *b, uint8_t len)
{
    // initial delay
    hal.scheduler->delay_microseconds(BYTE_DELAY_FIRST_US);
    uint8_t crc = 0;
    while (len) {
        uint8_t ob = *b;
        if (uart->write(ob) == 1) {
            len--;
            crc += ob;
            b++;
            hal.scheduler->delay_microseconds(BYTE_DELAY_US);
        } else {
            hal.scheduler->delay_microseconds(100);
        }
    }
    uart->write(crc);

    // discard any bytes received during the send
    hal.scheduler->delay_microseconds(BYTE_DELAY_US*2);
    while (uart->available() != 0) {
        uart->read();
        hal.scheduler->delay_microseconds(100);
    }
}

/*
  thread to process requests
 */
void AP_Hott_Telem::loop(void)
{
    uart->begin(19200, 10, 10);
    uart->set_unbuffered_writes(true);
    uart->set_blocking_writes(true);

    while (true) {
        hal.scheduler->delay_microseconds(1500);
        uint32_t n = uart->available();
        if (n < 2) {
            // wait for 2 bytes
            continue;
        }
        if (n > 2) {
            uart->discard_input();
            continue;
        }

        const uint8_t prot_type = uart->read();
        const uint8_t sensor_id = uart->read();
        if (prot_type != PROT_BINARY) {
            // only do binary protocol for now
            continue;
        }

        switch (sensor_id) {
        case PROT_ID_EAM:
            send_EAM();
            break;
        case PROT_ID_GPS:
            send_GPS();
            break;
        case PROT_ID_VARIO:
            send_Vario();
            break;
        }
    }
}

namespace AP {
    AP_Hott_Telem *hott_telem() {
        return AP_Hott_Telem::get_singleton();
    }
};

#endif // HAL_HOTT_TELEM_ENABLED
