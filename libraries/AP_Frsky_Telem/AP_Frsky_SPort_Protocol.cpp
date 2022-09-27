#include "AP_Frsky_SPort_Protocol.h"

//#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
//#include <AP_GPS/AP_GPS.h>
//#include <AP_Notify/AP_Notify.h>
//#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Terrain/AP_Terrain.h>
//#include <AC_Fence/AC_Fence.h>
#include <AP_Vehicle/AP_Vehicle.h>
//#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>
#if APM_BUILD_TYPE(APM_BUILD_Rover)
#include <AP_WindVane/AP_WindVane.h>
#endif


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
#define AP_FS_OFFSET                12
#define AP_FENCE_PRESENT_OFFSET     13
#define AP_FENCE_BREACH_OFFSET      14
#define AP_THROTTLE_OFFSET          19
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
#define VELANDYAW_ARSPD_OFFSET      28
// for attitude (roll, pitch) and range data
#define ATTIANDRNG_ROLL_LIMIT       0x7FF
#define ATTIANDRNG_PITCH_LIMIT      0x3FF
#define ATTIANDRNG_PITCH_OFFSET     11
#define ATTIANDRNG_RNGFND_OFFSET    21
// for terrain data
#define TERRAIN_UNHEALTHY_OFFSET    13
// for wind data
#define WIND_ANGLE_LIMIT            0x7F
#define WIND_SPEED_OFFSET           7
#define WIND_APPARENT_ANGLE_OFFSET  15
#define WIND_APPARENT_SPEED_OFFSET  23
// for waypoint data
#define WP_NUMBER_LIMIT             2047
#define WP_DISTANCE_LIMIT           1023000
#define WP_DISTANCE_OFFSET          11
#define WP_BEARING_OFFSET           23


extern const AP_HAL::HAL& hal;


AP_Frsky_SPort_Protocol *AP_Frsky_SPort_Protocol::singleton;


bool AP_Frsky_SPort_Protocol::is_available_batt(uint8_t instance)
{
    return (AP::battery().num_instances() > instance);
}

bool AP_Frsky_SPort_Protocol::is_available_ap_status(void)
{
    return gcs().vehicle_initialised();
}

bool AP_Frsky_SPort_Protocol::is_available_rpm(uint8_t instance)
{
    const AP_RPM* rpm = AP::rpm();
    return (rpm != nullptr) && (rpm->num_sensors() > instance);
}

bool AP_Frsky_SPort_Protocol::is_available_terrain(void)
{
#if AP_TERRAIN_AVAILABLE
    const AP_Terrain* terrain = AP::terrain();
    return (terrain != nullptr) && terrain->enabled();
#endif
    return false;
}

bool AP_Frsky_SPort_Protocol::is_available_wind(void)
{
#if !APM_BUILD_TYPE(APM_BUILD_Rover)
    float a;
    WITH_SEMAPHORE(AP::ahrs().get_semaphore());
    return AP::ahrs().airspeed_estimate_true(a); // if we have an airspeed estimate then we have a valid wind estimate
#else
    const AP_WindVane* windvane = AP::windvane();
    return (windvane != nullptr) && windvane->enabled();
#endif
}

bool AP_Frsky_SPort_Protocol::is_available_waypoint(void)
{
    const AP_Mission* mission = AP::mission();
    return (mission != nullptr) && (mission->get_current_nav_index() > 0);
}


uint32_t AP_Frsky_SPort_Protocol::calc_gps_latlng(bool &send_latitude)
{
    const Location &loc = AP::gps().location(0); // use the first gps instance (same as in send_mavlink_gps_raw)

    // alternate between latitude and longitude
    if (send_latitude == true) {
        send_latitude = false;
        if (loc.lat < 0) {
            return ((labs(loc.lat)/100)*6) | 0x40000000;
        } else {
            return ((labs(loc.lat)/100)*6);
        }
    } else {
        send_latitude = true;
        if (loc.lng < 0) {
            return ((labs(loc.lng)/100)*6) | 0xC0000000;
        } else {
            return ((labs(loc.lng)/100)*6) | 0x80000000;
        }
    }
}


uint32_t AP_Frsky_SPort_Protocol::calc_gps_status(void)
{
    const AP_GPS &gps = AP::gps();

    // number of GPS satellites visible (limit to 15 (0xF) since the value is stored on 4 bits)
    uint32_t gps_status = (gps.num_sats() < GPS_SATS_LIMIT) ? gps.num_sats() : GPS_SATS_LIMIT;
    // GPS receiver status (limit to 0-3 (0x3) since the value is stored on 2 bits: NO_GPS = 0, NO_FIX = 1, GPS_OK_FIX_2D = 2, GPS_OK_FIX_3D or GPS_OK_FIX_3D_DGPS or GPS_OK_FIX_3D_RTK_FLOAT or GPS_OK_FIX_3D_RTK_FIXED = 3)
    gps_status |= ((gps.status() < GPS_STATUS_LIMIT) ? gps.status() : GPS_STATUS_LIMIT) << GPS_STATUS_OFFSET;
    // GPS horizontal dilution of precision in dm
    gps_status |= prep_number(roundf(gps.get_hdop() * 0.1f), 2, 1) << GPS_HDOP_OFFSET;
    // GPS receiver advanced status (0: no advanced fix, 1: GPS_OK_FIX_3D_DGPS, 2: GPS_OK_FIX_3D_RTK_FLOAT, 3: GPS_OK_FIX_3D_RTK_FIXED)
    gps_status |= ((gps.status() > GPS_STATUS_LIMIT) ? gps.status()-GPS_STATUS_LIMIT : 0) << GPS_ADVSTATUS_OFFSET;
    // Altitude MSL in dm
    const Location &loc = gps.location();
    gps_status |= prep_number(roundf(loc.alt * 0.1f), 2, 2) << GPS_ALTMSL_OFFSET;
    return gps_status;
}


uint32_t AP_Frsky_SPort_Protocol::calc_attiandrng(void)
{
    const RangeFinder *_rng = RangeFinder::get_singleton();

    AP_AHRS &_ahrs = AP::ahrs();
    // roll from [-18000;18000] centidegrees to unsigned .2 degree increments [0;1800] (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
    uint32_t attiandrng = ((uint16_t)roundf((_ahrs.roll_sensor + 18000) * 0.05f) & ATTIANDRNG_ROLL_LIMIT);
    // pitch from [-9000;9000] centidegrees to unsigned .2 degree increments [0;900] (just in case, limit to 1023 (0x3FF) since the value is stored on 10 bits)
    attiandrng |= ((uint16_t)roundf((_ahrs.pitch_sensor + 9000) * 0.05f) & ATTIANDRNG_PITCH_LIMIT) << ATTIANDRNG_PITCH_OFFSET;
    // rangefinder measurement in cm
    attiandrng |= prep_number(_rng ? _rng->distance_cm_orient(ROTATION_PITCH_270) : 0, 3, 1) << ATTIANDRNG_RNGFND_OFFSET;
    return attiandrng;
}


uint32_t AP_Frsky_SPort_Protocol::calc_velandyaw(bool airspeed_enabled, bool send_airspeed)
{
    float vspd = get_vspeed_ms();
    // vertical velocity in dm/s
    uint32_t velandyaw = prep_number(roundf(vspd * 10), 2, 1);
    float airspeed_m;       // m/s
    float hspeed_m;         // m/s
    bool airspeed_estimate_true;
    AP_AHRS &_ahrs = AP::ahrs();
    {
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        hspeed_m = _ahrs.groundspeed(); // default is to use groundspeed
        airspeed_estimate_true = AP::ahrs().airspeed_estimate_true(airspeed_m);
    }
    // airspeed estimate + airspeed option disabled (default) => send airspeed (we give priority to airspeed over groundspeed)
    // airspeed estimate + airspeed option enabled => alternate airspeed/groundspeed, i.e send airspeed only when _passthrough.send_airspeed==true
    if (airspeed_estimate_true && (!airspeed_enabled || send_airspeed)) {
        hspeed_m = airspeed_m;
    }
    // horizontal velocity in dm/s
    velandyaw |= prep_number(roundf(hspeed_m * 10), 2, 1) << VELANDYAW_XYVEL_OFFSET;
    // yaw from [0;36000] centidegrees to .2 degree increments [0;1800] (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
    velandyaw |= ((uint16_t)roundf(_ahrs.yaw_sensor * 0.05f) & VELANDYAW_YAW_LIMIT) << VELANDYAW_YAW_OFFSET;
    // flag the airspeed bit if required
    if (airspeed_estimate_true && airspeed_enabled && send_airspeed) {
        velandyaw |= (1U << VELANDYAW_ARSPD_OFFSET);
    }
    return velandyaw;
}


uint32_t AP_Frsky_SPort_Protocol::calc_batt(uint8_t instance)
{
    const AP_BattMonitor &_battery = AP::battery();

    float current, consumed_mah;
    if (!_battery.current_amps(current, instance)) {
        current = 0;
    }
    if (!_battery.consumed_mah(consumed_mah, instance)) {
        consumed_mah = 0;
    }

    // battery voltage in decivolts, can have up to a 12S battery (4.25Vx12S = 51.0V)
    uint32_t batt = (((uint16_t)roundf(_battery.voltage(instance) * 10.0f)) & BATT_VOLTAGE_LIMIT);
    // battery current draw in deciamps
    batt |= prep_number(roundf(current * 10.0f), 2, 1) << BATT_CURRENT_OFFSET;
    // battery current drawn since power on in mAh (limit to 32767 (0x7FFF) since value is stored on 15 bits)
    batt |= ((consumed_mah < BATT_TOTALMAH_LIMIT) ? ((uint16_t)roundf(consumed_mah) & BATT_TOTALMAH_LIMIT) : BATT_TOTALMAH_LIMIT) << BATT_TOTALMAH_OFFSET;
    return batt;
}


uint32_t AP_Frsky_SPort_Protocol::calc_ap_status(void)
{
    // IMU temperature: offset -19, 0 means temp =< 19°, 63 means temp => 82°
    uint8_t imu_temp = 0;
#if HAL_INS_ENABLED
    imu_temp = (uint8_t) roundf(constrain_float(AP::ins().get_temperature(0), AP_IMU_TEMP_MIN, AP_IMU_TEMP_MAX) - AP_IMU_TEMP_MIN);
#endif

    // control/flight mode number (limit to 31 (0x1F) since the value is stored on 5 bits)
    uint32_t ap_status = (uint8_t)((gcs().custom_mode() + 1) & AP_CONTROL_MODE_LIMIT);
    // simple/super simple modes flags
    ap_status |= (uint8_t)(gcs().simple_input_active()) << AP_SIMPLE_OFFSET;
    ap_status |= (uint8_t)(gcs().supersimple_input_active()) << AP_SSIMPLE_OFFSET;
    // is_flying flag
    ap_status |= (uint8_t)(AP_Notify::flags.flying) << AP_FLYING_OFFSET;
    // armed flag
    ap_status |= (uint8_t)(AP_Notify::flags.armed) << AP_ARMED_OFFSET;
    // battery failsafe flag
    ap_status |= (uint8_t)(AP_Notify::flags.failsafe_battery) << AP_BATT_FS_OFFSET;
    // bad ekf flag
    ap_status |= (uint8_t)(AP_Notify::flags.ekf_bad) << AP_EKF_FS_OFFSET;
    // generic failsafe
    ap_status |= (uint8_t)(AP_Notify::flags.failsafe_battery||AP_Notify::flags.failsafe_ekf||AP_Notify::flags.failsafe_gcs||AP_Notify::flags.failsafe_radio) << AP_FS_OFFSET;
#if AP_FENCE_ENABLED
    // fence status
    AC_Fence *fence = AP::fence();
    if (fence != nullptr) {
        ap_status |= (uint8_t)(fence->enabled() && fence->present()) << AP_FENCE_PRESENT_OFFSET;
        ap_status |= (uint8_t)(fence->get_breaches() > 0) << AP_FENCE_BREACH_OFFSET;
    }
#endif
    // signed throttle [-100,100] scaled down to [-63,63] on 7 bits, MSB for sign + 6 bits for 0-63
    ap_status |= prep_number(gcs().get_hud_throttle() * 0.63f, 2, 0) << AP_THROTTLE_OFFSET;
    // IMU temperature
    ap_status |= imu_temp << AP_IMU_TEMP_OFFSET;
    return ap_status;
}


uint32_t AP_Frsky_SPort_Protocol::calc_home(void)
{
    uint32_t home = 0;
    Location loc;
    Location home_loc;
    bool got_position = false;
    float _relative_home_altitude = 0;

    {
        AP_AHRS &_ahrs = AP::ahrs();
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        got_position = _ahrs.get_location(loc);
        home_loc = _ahrs.get_home();
    }

    if (got_position) {
        // check home_loc is valid
        if (home_loc.lat != 0 || home_loc.lng != 0) {
            // distance between vehicle and home_loc in meters
            home = prep_number(roundf(home_loc.get_distance(loc)), 3, 2);
            // angle from front of vehicle to the direction of home_loc in 3 degree increments (just in case, limit to 127 (0x7F) since the value is stored on 7 bits)
            home |= (((uint8_t)roundf(loc.get_bearing_to(home_loc) * 0.00333f)) & HOME_BEARING_LIMIT) << HOME_BEARING_OFFSET;
        }
        // altitude between vehicle and home_loc
        _relative_home_altitude = loc.alt;
        if (!loc.relative_alt) {
            // loc.alt has home altitude added, remove it
            _relative_home_altitude -= home_loc.alt;
        }
    }
    // altitude above home in decimeters
    home |= prep_number(roundf(_relative_home_altitude * 0.1f), 3, 2) << HOME_ALT_OFFSET;
    return home;
}


uint32_t AP_Frsky_SPort_Protocol::calc_rpm(void)
{
    const AP_RPM *ap_rpm = AP::rpm();
    if (ap_rpm == nullptr) {
        return 0;
    }

    uint32_t value = 0;
    // we send: rpm_value*0.1 as 16 bits signed
    float rpm;
    // bits 0-15 for rpm instance 0
    if (ap_rpm->get_rpm(0, rpm)) {
        value |= (int16_t)roundf(rpm * 0.1f);
    }
    // bits 16-31 for rpm instance 1
    if (ap_rpm->get_rpm(1, rpm)) {
        value |= (int16_t)roundf(rpm * 0.1f) << 16;
    }
    return value;
}


int32_t AP_Frsky_SPort_Protocol::calc_sensor_rpm(uint8_t instance)
{
    const AP_RPM* rpm = AP::rpm();
    if (rpm == nullptr) {
        return 0;
    }

    float rpm_value;
    if (!rpm->get_rpm(instance, rpm_value)) {
        return 0;
    }
    return (int32_t)roundf(rpm_value);
}



uint32_t AP_Frsky_SPort_Protocol::calc_terrain(void)
{
    uint32_t value = 0;
#if AP_TERRAIN_AVAILABLE
    AP_Terrain *terrain = AP::terrain();
    if (terrain == nullptr || !terrain->enabled()) {
        return value;
    }

    float height_above_terrain;
    if (terrain->height_above_terrain(height_above_terrain, true)) {
        // vehicle height above terrain
        value |= prep_number(roundf(height_above_terrain * 10.0f), 3, 2);
    }
    // terrain unhealthy flag
    value |= (uint8_t)(terrain->status() == AP_Terrain::TerrainStatus::TerrainStatusUnhealthy) << TERRAIN_UNHEALTHY_OFFSET;
#endif
    return value;
}


uint32_t AP_Frsky_SPort_Protocol::calc_wind(void)
{
#if !APM_BUILD_TYPE(APM_BUILD_Rover)
    Vector3f v;
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        v = ahrs.wind_estimate();
    }
    // wind angle in 3 degree increments 0,360 (unsigned)
    uint32_t value = prep_number(roundf(wrap_360(degrees(atan2f(-v.y, -v.x))) * (1.0f/3.0f)), 2, 0);
    // wind speed in dm/s
    value |= prep_number(roundf(v.length() * 10.0f), 2, 1) << WIND_SPEED_OFFSET;
#else
    const AP_WindVane* windvane = AP_WindVane::get_singleton();
    uint32_t value = 0;
    if (windvane != nullptr && windvane->enabled()) {
        // true wind angle in 3 degree increments 0,360 (unsigned)
        value = prep_number(roundf(wrap_360(degrees(windvane->get_true_wind_direction_rad())) * (1.0f/3.0f)), 2, 0);
        // true wind speed in dm/s
        value |= prep_number(roundf(windvane->get_true_wind_speed() * 10.0f), 2, 1) << WIND_SPEED_OFFSET;
        // apparent wind angle in 3 degree increments -180,180 (signed)
        value |= prep_number(roundf(degrees(windvane->get_apparent_wind_direction_rad()) * (1.0f/3.0f)), 2, 0);
        // apparent wind speed in dm/s
        value |= prep_number(roundf(windvane->get_apparent_wind_speed() * 10.0f), 2, 1) << WIND_APPARENT_SPEED_OFFSET;
    }
#endif
    return value;
}


uint32_t AP_Frsky_SPort_Protocol::calc_waypoint(void)
{
    const AP_Mission *mission = AP::mission();
    const AP_Vehicle *vehicle = AP::vehicle();
    if (mission == nullptr || vehicle == nullptr) {
        return 0;
    }

    float wp_distance;
    if (!vehicle->get_wp_distance_m(wp_distance)) {
        return 0;
    }
    float angle;
    if (!vehicle->get_wp_bearing_deg(angle)) {
        return 0;
    }
    // waypoint current nav index
    uint32_t value = MIN(mission->get_current_nav_index(), WP_NUMBER_LIMIT);
    // distance to next waypoint
    value |= prep_number(wp_distance, 3, 2) << WP_DISTANCE_OFFSET;
    // bearing encoded in 3 degrees increments
    value |= ((uint8_t)roundf(wrap_360(angle) * 0.333f)) << WP_BEARING_OFFSET;
    return value;
}


uint32_t AP_Frsky_SPort_Protocol::calc_param(uint8_t* param_id)
{
    uint8_t _param_id = *param_id; // cache it because it gets changed inside the switch
    uint32_t param_value = 0;

    switch (_param_id) {
    case NONE:
    case FRAME_TYPE:
        param_value = gcs().frame_type(); // see MAV_TYPE in Mavlink definition file common.h
        *param_id = BATT_CAPACITY_1;
        break;
    case BATT_CAPACITY_1:
        param_value = (uint32_t)roundf(AP::battery().pack_capacity_mah(0)); // battery pack capacity in mAh
        *param_id = (AP::battery().num_instances() > 1) ? BATT_CAPACITY_2 : TELEMETRY_FEATURES;
        break;
    case BATT_CAPACITY_2:
        param_value = (uint32_t)roundf(AP::battery().pack_capacity_mah(1)); // battery pack capacity in mAh
        *param_id = TELEMETRY_FEATURES;
        break;
    case TELEMETRY_FEATURES:
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        BIT_SET(param_value, PassthroughFeatures::BIDIR);
#endif
#if AP_SCRIPTING_ENABLED
        BIT_SET(param_value, PassthroughFeatures::SCRIPTING);
#endif
        *param_id = FRAME_TYPE;
        break;
    }
    //Reserve first 8 bits for param ID, use other 24 bits to store parameter value
    return (_param_id << PARAM_ID_OFFSET) | (param_value & PARAM_VALUE_LIMIT);
}


void AP_Frsky_SPort_Protocol::pack_packet(uint8_t* buf, uint8_t count, uint16_t id, uint32_t data)
{
    memcpy(&(buf[count*6]), &id, 2);
    memcpy(&(buf[count*6 + 2]), &data, 4);
}


float AP_Frsky_SPort_Protocol::get_vspeed_ms(void)
{
    {
        // release semaphore as soon as possible
        AP_AHRS &_ahrs = AP::ahrs();
        Vector3f v;
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        if (_ahrs.get_velocity_NED(v)) {
            return -v.z;
        }
    }

    auto &_baro = AP::baro();
    WITH_SEMAPHORE(_baro.get_semaphore());
    return _baro.get_climb_rate();
}


float AP_Frsky_SPort_Protocol::get_current_height_cm(void) // in centimeters above home
{
    Location loc;
    float current_height = 0.0f;

    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());
    if (_ahrs.get_location(loc)) {
        current_height = loc.alt * 0.01f;
        if (!loc.relative_alt) {
            // loc.alt has home altitude added, remove it
            current_height -= _ahrs.get_home().alt * 0.01f;
        }
    }
    return current_height;
}


uint16_t AP_Frsky_SPort_Protocol::prep_number(int32_t number, uint8_t digits, uint8_t power)
{
    uint16_t res = 0;
    uint32_t abs_number = abs(number);

    if ((digits == 2) && (power == 0)) { // number encoded on 7 bits, client side needs to know if expected range is 0,127 or -63,63
        uint8_t max_value = (number < 0) ? (1 << 6) - 1 : (1 << 7) - 1;
        res = constrain_int16(abs_number, 0, max_value);
        if (number < 0) {   // if number is negative, add sign bit in front
            res |= (1 << 6);
        }
    } else if ((digits == 2) && (power == 1)) { // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
        if (abs_number < 100) {
            res = (abs_number << 1);
        } else if (abs_number < 1270) {
            res = ((uint8_t)roundf(abs_number * 0.1f) << 1) | 0x0001;
        } else { // transmit max possible value (0x7F x 10^1 = 1270)
            res = 0x00FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= (1 << 8);
        }
    } else if ((digits == 2) && (power == 2)) { // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
        if (abs_number < 100) {
            res = (abs_number << 2);
        } else if (abs_number < 1000) {
            res = ((uint8_t)roundf(abs_number * 0.1f) << 2) | 0x0001;
        } else if (abs_number < 10000) {
            res = ((uint8_t)roundf(abs_number * 0.01f) << 2) | 0x0002;
        } else if (abs_number < 127000) {
            res = ((uint8_t)roundf(abs_number * 0.001f) << 2) | 0x0003;
        } else { // transmit max possible value (0x7F x 10^3 = 127000)
            res = 0x01FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= (1 << 9);
        }
    } else if ((digits == 3) && (power == 1)) { // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
        if (abs_number < 1000) {
            res = (abs_number << 1);
        } else if (abs_number < 10240) {
            res = ((uint16_t)roundf(abs_number * 0.1f) << 1) | 0x0001;
        } else { // transmit max possible value (0x3FF x 10^1 = 10230)
            res = 0x07FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= (0x1 << 11);
        }
    } else if ((digits == 3) && (power == 2)) { // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
        if (abs_number < 1000) {
            res = (abs_number << 2);
        } else if (abs_number < 10000) {
            res = ((uint16_t)roundf(abs_number * 0.1f) << 2) | 0x0001;
        } else if (abs_number < 100000) {
            res = ((uint16_t)roundf(abs_number * 0.01f) << 2) | 0x0002;
        } else if (abs_number < 1024000) {
            res = ((uint16_t)roundf(abs_number * 0.001f) << 2) | 0x0003;
        } else { // transmit max possible value (0x3FF x 10^3 = 1023000)
            res = 0x0FFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= (1 << 12);
        }
    }
    return res;
}


namespace AP
{
AP_Frsky_SPort_Protocol *frsky_sport_protocol()
{
    return AP_Frsky_SPort_Protocol::get_singleton();
}
};
