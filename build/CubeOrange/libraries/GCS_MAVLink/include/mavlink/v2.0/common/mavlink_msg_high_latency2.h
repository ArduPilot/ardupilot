#pragma once
// MESSAGE HIGH_LATENCY2 PACKING

#define MAVLINK_MSG_ID_HIGH_LATENCY2 235


typedef struct __mavlink_high_latency2_t {
 uint32_t timestamp; /*< [ms] Timestamp (milliseconds since boot or Unix epoch)*/
 int32_t latitude; /*< [degE7] Latitude*/
 int32_t longitude; /*< [degE7] Longitude*/
 uint16_t custom_mode; /*<  A bitfield for use for autopilot-specific flags (2 byte version).*/
 int16_t altitude; /*< [m] Altitude above mean sea level*/
 int16_t target_altitude; /*< [m] Altitude setpoint*/
 uint16_t target_distance; /*< [dam] Distance to target waypoint or position*/
 uint16_t wp_num; /*<  Current waypoint number*/
 uint16_t failure_flags; /*<  Bitmap of failure flags.*/
 uint8_t type; /*<  Type of the MAV (quadrotor, helicopter, etc.)*/
 uint8_t autopilot; /*<  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.*/
 uint8_t heading; /*< [deg/2] Heading*/
 uint8_t target_heading; /*< [deg/2] Heading setpoint*/
 uint8_t throttle; /*< [%] Throttle*/
 uint8_t airspeed; /*< [m/s*5] Airspeed*/
 uint8_t airspeed_sp; /*< [m/s*5] Airspeed setpoint*/
 uint8_t groundspeed; /*< [m/s*5] Groundspeed*/
 uint8_t windspeed; /*< [m/s*5] Windspeed*/
 uint8_t wind_heading; /*< [deg/2] Wind heading*/
 uint8_t eph; /*< [dm] Maximum error horizontal position since last message*/
 uint8_t epv; /*< [dm] Maximum error vertical position since last message*/
 int8_t temperature_air; /*< [degC] Air temperature from airspeed sensor*/
 int8_t climb_rate; /*< [dm/s] Maximum climb rate magnitude since last message*/
 int8_t battery; /*< [%] Battery level (-1 if field not provided).*/
 int8_t custom0; /*<  Field for custom payload.*/
 int8_t custom1; /*<  Field for custom payload.*/
 int8_t custom2; /*<  Field for custom payload.*/
} mavlink_high_latency2_t;

#define MAVLINK_MSG_ID_HIGH_LATENCY2_LEN 42
#define MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN 42
#define MAVLINK_MSG_ID_235_LEN 42
#define MAVLINK_MSG_ID_235_MIN_LEN 42

#define MAVLINK_MSG_ID_HIGH_LATENCY2_CRC 179
#define MAVLINK_MSG_ID_235_CRC 179



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIGH_LATENCY2 { \
    235, \
    "HIGH_LATENCY2", \
    27, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_high_latency2_t, timestamp) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_high_latency2_t, type) }, \
         { "autopilot", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_high_latency2_t, autopilot) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_high_latency2_t, custom_mode) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_high_latency2_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_high_latency2_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_high_latency2_t, altitude) }, \
         { "target_altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_high_latency2_t, target_altitude) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_high_latency2_t, heading) }, \
         { "target_heading", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_high_latency2_t, target_heading) }, \
         { "target_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_high_latency2_t, target_distance) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_high_latency2_t, throttle) }, \
         { "airspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_high_latency2_t, airspeed) }, \
         { "airspeed_sp", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_high_latency2_t, airspeed_sp) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_high_latency2_t, groundspeed) }, \
         { "windspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_high_latency2_t, windspeed) }, \
         { "wind_heading", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_high_latency2_t, wind_heading) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_high_latency2_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_high_latency2_t, epv) }, \
         { "temperature_air", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_high_latency2_t, temperature_air) }, \
         { "climb_rate", NULL, MAVLINK_TYPE_INT8_T, 0, 37, offsetof(mavlink_high_latency2_t, climb_rate) }, \
         { "battery", NULL, MAVLINK_TYPE_INT8_T, 0, 38, offsetof(mavlink_high_latency2_t, battery) }, \
         { "wp_num", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_high_latency2_t, wp_num) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_high_latency2_t, failure_flags) }, \
         { "custom0", NULL, MAVLINK_TYPE_INT8_T, 0, 39, offsetof(mavlink_high_latency2_t, custom0) }, \
         { "custom1", NULL, MAVLINK_TYPE_INT8_T, 0, 40, offsetof(mavlink_high_latency2_t, custom1) }, \
         { "custom2", NULL, MAVLINK_TYPE_INT8_T, 0, 41, offsetof(mavlink_high_latency2_t, custom2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIGH_LATENCY2 { \
    "HIGH_LATENCY2", \
    27, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_high_latency2_t, timestamp) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_high_latency2_t, type) }, \
         { "autopilot", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_high_latency2_t, autopilot) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_high_latency2_t, custom_mode) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_high_latency2_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_high_latency2_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_high_latency2_t, altitude) }, \
         { "target_altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_high_latency2_t, target_altitude) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_high_latency2_t, heading) }, \
         { "target_heading", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_high_latency2_t, target_heading) }, \
         { "target_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_high_latency2_t, target_distance) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_high_latency2_t, throttle) }, \
         { "airspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_high_latency2_t, airspeed) }, \
         { "airspeed_sp", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_high_latency2_t, airspeed_sp) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_high_latency2_t, groundspeed) }, \
         { "windspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_high_latency2_t, windspeed) }, \
         { "wind_heading", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_high_latency2_t, wind_heading) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_high_latency2_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_high_latency2_t, epv) }, \
         { "temperature_air", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_high_latency2_t, temperature_air) }, \
         { "climb_rate", NULL, MAVLINK_TYPE_INT8_T, 0, 37, offsetof(mavlink_high_latency2_t, climb_rate) }, \
         { "battery", NULL, MAVLINK_TYPE_INT8_T, 0, 38, offsetof(mavlink_high_latency2_t, battery) }, \
         { "wp_num", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_high_latency2_t, wp_num) }, \
         { "failure_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_high_latency2_t, failure_flags) }, \
         { "custom0", NULL, MAVLINK_TYPE_INT8_T, 0, 39, offsetof(mavlink_high_latency2_t, custom0) }, \
         { "custom1", NULL, MAVLINK_TYPE_INT8_T, 0, 40, offsetof(mavlink_high_latency2_t, custom1) }, \
         { "custom2", NULL, MAVLINK_TYPE_INT8_T, 0, 41, offsetof(mavlink_high_latency2_t, custom2) }, \
         } \
}
#endif

/**
 * @brief Pack a high_latency2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp (milliseconds since boot or Unix epoch)
 * @param type  Type of the MAV (quadrotor, helicopter, etc.)
 * @param autopilot  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
 * @param custom_mode  A bitfield for use for autopilot-specific flags (2 byte version).
 * @param latitude [degE7] Latitude
 * @param longitude [degE7] Longitude
 * @param altitude [m] Altitude above mean sea level
 * @param target_altitude [m] Altitude setpoint
 * @param heading [deg/2] Heading
 * @param target_heading [deg/2] Heading setpoint
 * @param target_distance [dam] Distance to target waypoint or position
 * @param throttle [%] Throttle
 * @param airspeed [m/s*5] Airspeed
 * @param airspeed_sp [m/s*5] Airspeed setpoint
 * @param groundspeed [m/s*5] Groundspeed
 * @param windspeed [m/s*5] Windspeed
 * @param wind_heading [deg/2] Wind heading
 * @param eph [dm] Maximum error horizontal position since last message
 * @param epv [dm] Maximum error vertical position since last message
 * @param temperature_air [degC] Air temperature from airspeed sensor
 * @param climb_rate [dm/s] Maximum climb rate magnitude since last message
 * @param battery [%] Battery level (-1 if field not provided).
 * @param wp_num  Current waypoint number
 * @param failure_flags  Bitmap of failure flags.
 * @param custom0  Field for custom payload.
 * @param custom1  Field for custom payload.
 * @param custom2  Field for custom payload.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_high_latency2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t timestamp, uint8_t type, uint8_t autopilot, uint16_t custom_mode, int32_t latitude, int32_t longitude, int16_t altitude, int16_t target_altitude, uint8_t heading, uint8_t target_heading, uint16_t target_distance, uint8_t throttle, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, uint8_t windspeed, uint8_t wind_heading, uint8_t eph, uint8_t epv, int8_t temperature_air, int8_t climb_rate, int8_t battery, uint16_t wp_num, uint16_t failure_flags, int8_t custom0, int8_t custom1, int8_t custom2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIGH_LATENCY2_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_uint16_t(buf, 12, custom_mode);
    _mav_put_int16_t(buf, 14, altitude);
    _mav_put_int16_t(buf, 16, target_altitude);
    _mav_put_uint16_t(buf, 18, target_distance);
    _mav_put_uint16_t(buf, 20, wp_num);
    _mav_put_uint16_t(buf, 22, failure_flags);
    _mav_put_uint8_t(buf, 24, type);
    _mav_put_uint8_t(buf, 25, autopilot);
    _mav_put_uint8_t(buf, 26, heading);
    _mav_put_uint8_t(buf, 27, target_heading);
    _mav_put_uint8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_uint8_t(buf, 32, windspeed);
    _mav_put_uint8_t(buf, 33, wind_heading);
    _mav_put_uint8_t(buf, 34, eph);
    _mav_put_uint8_t(buf, 35, epv);
    _mav_put_int8_t(buf, 36, temperature_air);
    _mav_put_int8_t(buf, 37, climb_rate);
    _mav_put_int8_t(buf, 38, battery);
    _mav_put_int8_t(buf, 39, custom0);
    _mav_put_int8_t(buf, 40, custom1);
    _mav_put_int8_t(buf, 41, custom2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN);
#else
    mavlink_high_latency2_t packet;
    packet.timestamp = timestamp;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.custom_mode = custom_mode;
    packet.altitude = altitude;
    packet.target_altitude = target_altitude;
    packet.target_distance = target_distance;
    packet.wp_num = wp_num;
    packet.failure_flags = failure_flags;
    packet.type = type;
    packet.autopilot = autopilot;
    packet.heading = heading;
    packet.target_heading = target_heading;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.groundspeed = groundspeed;
    packet.windspeed = windspeed;
    packet.wind_heading = wind_heading;
    packet.eph = eph;
    packet.epv = epv;
    packet.temperature_air = temperature_air;
    packet.climb_rate = climb_rate;
    packet.battery = battery;
    packet.custom0 = custom0;
    packet.custom1 = custom1;
    packet.custom2 = custom2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIGH_LATENCY2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_CRC);
}

/**
 * @brief Pack a high_latency2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] Timestamp (milliseconds since boot or Unix epoch)
 * @param type  Type of the MAV (quadrotor, helicopter, etc.)
 * @param autopilot  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
 * @param custom_mode  A bitfield for use for autopilot-specific flags (2 byte version).
 * @param latitude [degE7] Latitude
 * @param longitude [degE7] Longitude
 * @param altitude [m] Altitude above mean sea level
 * @param target_altitude [m] Altitude setpoint
 * @param heading [deg/2] Heading
 * @param target_heading [deg/2] Heading setpoint
 * @param target_distance [dam] Distance to target waypoint or position
 * @param throttle [%] Throttle
 * @param airspeed [m/s*5] Airspeed
 * @param airspeed_sp [m/s*5] Airspeed setpoint
 * @param groundspeed [m/s*5] Groundspeed
 * @param windspeed [m/s*5] Windspeed
 * @param wind_heading [deg/2] Wind heading
 * @param eph [dm] Maximum error horizontal position since last message
 * @param epv [dm] Maximum error vertical position since last message
 * @param temperature_air [degC] Air temperature from airspeed sensor
 * @param climb_rate [dm/s] Maximum climb rate magnitude since last message
 * @param battery [%] Battery level (-1 if field not provided).
 * @param wp_num  Current waypoint number
 * @param failure_flags  Bitmap of failure flags.
 * @param custom0  Field for custom payload.
 * @param custom1  Field for custom payload.
 * @param custom2  Field for custom payload.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_high_latency2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t timestamp,uint8_t type,uint8_t autopilot,uint16_t custom_mode,int32_t latitude,int32_t longitude,int16_t altitude,int16_t target_altitude,uint8_t heading,uint8_t target_heading,uint16_t target_distance,uint8_t throttle,uint8_t airspeed,uint8_t airspeed_sp,uint8_t groundspeed,uint8_t windspeed,uint8_t wind_heading,uint8_t eph,uint8_t epv,int8_t temperature_air,int8_t climb_rate,int8_t battery,uint16_t wp_num,uint16_t failure_flags,int8_t custom0,int8_t custom1,int8_t custom2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIGH_LATENCY2_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_uint16_t(buf, 12, custom_mode);
    _mav_put_int16_t(buf, 14, altitude);
    _mav_put_int16_t(buf, 16, target_altitude);
    _mav_put_uint16_t(buf, 18, target_distance);
    _mav_put_uint16_t(buf, 20, wp_num);
    _mav_put_uint16_t(buf, 22, failure_flags);
    _mav_put_uint8_t(buf, 24, type);
    _mav_put_uint8_t(buf, 25, autopilot);
    _mav_put_uint8_t(buf, 26, heading);
    _mav_put_uint8_t(buf, 27, target_heading);
    _mav_put_uint8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_uint8_t(buf, 32, windspeed);
    _mav_put_uint8_t(buf, 33, wind_heading);
    _mav_put_uint8_t(buf, 34, eph);
    _mav_put_uint8_t(buf, 35, epv);
    _mav_put_int8_t(buf, 36, temperature_air);
    _mav_put_int8_t(buf, 37, climb_rate);
    _mav_put_int8_t(buf, 38, battery);
    _mav_put_int8_t(buf, 39, custom0);
    _mav_put_int8_t(buf, 40, custom1);
    _mav_put_int8_t(buf, 41, custom2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN);
#else
    mavlink_high_latency2_t packet;
    packet.timestamp = timestamp;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.custom_mode = custom_mode;
    packet.altitude = altitude;
    packet.target_altitude = target_altitude;
    packet.target_distance = target_distance;
    packet.wp_num = wp_num;
    packet.failure_flags = failure_flags;
    packet.type = type;
    packet.autopilot = autopilot;
    packet.heading = heading;
    packet.target_heading = target_heading;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.groundspeed = groundspeed;
    packet.windspeed = windspeed;
    packet.wind_heading = wind_heading;
    packet.eph = eph;
    packet.epv = epv;
    packet.temperature_air = temperature_air;
    packet.climb_rate = climb_rate;
    packet.battery = battery;
    packet.custom0 = custom0;
    packet.custom1 = custom1;
    packet.custom2 = custom2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIGH_LATENCY2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_CRC);
}

/**
 * @brief Encode a high_latency2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param high_latency2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_high_latency2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_high_latency2_t* high_latency2)
{
    return mavlink_msg_high_latency2_pack(system_id, component_id, msg, high_latency2->timestamp, high_latency2->type, high_latency2->autopilot, high_latency2->custom_mode, high_latency2->latitude, high_latency2->longitude, high_latency2->altitude, high_latency2->target_altitude, high_latency2->heading, high_latency2->target_heading, high_latency2->target_distance, high_latency2->throttle, high_latency2->airspeed, high_latency2->airspeed_sp, high_latency2->groundspeed, high_latency2->windspeed, high_latency2->wind_heading, high_latency2->eph, high_latency2->epv, high_latency2->temperature_air, high_latency2->climb_rate, high_latency2->battery, high_latency2->wp_num, high_latency2->failure_flags, high_latency2->custom0, high_latency2->custom1, high_latency2->custom2);
}

/**
 * @brief Encode a high_latency2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param high_latency2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_high_latency2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_high_latency2_t* high_latency2)
{
    return mavlink_msg_high_latency2_pack_chan(system_id, component_id, chan, msg, high_latency2->timestamp, high_latency2->type, high_latency2->autopilot, high_latency2->custom_mode, high_latency2->latitude, high_latency2->longitude, high_latency2->altitude, high_latency2->target_altitude, high_latency2->heading, high_latency2->target_heading, high_latency2->target_distance, high_latency2->throttle, high_latency2->airspeed, high_latency2->airspeed_sp, high_latency2->groundspeed, high_latency2->windspeed, high_latency2->wind_heading, high_latency2->eph, high_latency2->epv, high_latency2->temperature_air, high_latency2->climb_rate, high_latency2->battery, high_latency2->wp_num, high_latency2->failure_flags, high_latency2->custom0, high_latency2->custom1, high_latency2->custom2);
}

/**
 * @brief Send a high_latency2 message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] Timestamp (milliseconds since boot or Unix epoch)
 * @param type  Type of the MAV (quadrotor, helicopter, etc.)
 * @param autopilot  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
 * @param custom_mode  A bitfield for use for autopilot-specific flags (2 byte version).
 * @param latitude [degE7] Latitude
 * @param longitude [degE7] Longitude
 * @param altitude [m] Altitude above mean sea level
 * @param target_altitude [m] Altitude setpoint
 * @param heading [deg/2] Heading
 * @param target_heading [deg/2] Heading setpoint
 * @param target_distance [dam] Distance to target waypoint or position
 * @param throttle [%] Throttle
 * @param airspeed [m/s*5] Airspeed
 * @param airspeed_sp [m/s*5] Airspeed setpoint
 * @param groundspeed [m/s*5] Groundspeed
 * @param windspeed [m/s*5] Windspeed
 * @param wind_heading [deg/2] Wind heading
 * @param eph [dm] Maximum error horizontal position since last message
 * @param epv [dm] Maximum error vertical position since last message
 * @param temperature_air [degC] Air temperature from airspeed sensor
 * @param climb_rate [dm/s] Maximum climb rate magnitude since last message
 * @param battery [%] Battery level (-1 if field not provided).
 * @param wp_num  Current waypoint number
 * @param failure_flags  Bitmap of failure flags.
 * @param custom0  Field for custom payload.
 * @param custom1  Field for custom payload.
 * @param custom2  Field for custom payload.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_high_latency2_send(mavlink_channel_t chan, uint32_t timestamp, uint8_t type, uint8_t autopilot, uint16_t custom_mode, int32_t latitude, int32_t longitude, int16_t altitude, int16_t target_altitude, uint8_t heading, uint8_t target_heading, uint16_t target_distance, uint8_t throttle, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, uint8_t windspeed, uint8_t wind_heading, uint8_t eph, uint8_t epv, int8_t temperature_air, int8_t climb_rate, int8_t battery, uint16_t wp_num, uint16_t failure_flags, int8_t custom0, int8_t custom1, int8_t custom2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIGH_LATENCY2_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_uint16_t(buf, 12, custom_mode);
    _mav_put_int16_t(buf, 14, altitude);
    _mav_put_int16_t(buf, 16, target_altitude);
    _mav_put_uint16_t(buf, 18, target_distance);
    _mav_put_uint16_t(buf, 20, wp_num);
    _mav_put_uint16_t(buf, 22, failure_flags);
    _mav_put_uint8_t(buf, 24, type);
    _mav_put_uint8_t(buf, 25, autopilot);
    _mav_put_uint8_t(buf, 26, heading);
    _mav_put_uint8_t(buf, 27, target_heading);
    _mav_put_uint8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_uint8_t(buf, 32, windspeed);
    _mav_put_uint8_t(buf, 33, wind_heading);
    _mav_put_uint8_t(buf, 34, eph);
    _mav_put_uint8_t(buf, 35, epv);
    _mav_put_int8_t(buf, 36, temperature_air);
    _mav_put_int8_t(buf, 37, climb_rate);
    _mav_put_int8_t(buf, 38, battery);
    _mav_put_int8_t(buf, 39, custom0);
    _mav_put_int8_t(buf, 40, custom1);
    _mav_put_int8_t(buf, 41, custom2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGH_LATENCY2, buf, MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_CRC);
#else
    mavlink_high_latency2_t packet;
    packet.timestamp = timestamp;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.custom_mode = custom_mode;
    packet.altitude = altitude;
    packet.target_altitude = target_altitude;
    packet.target_distance = target_distance;
    packet.wp_num = wp_num;
    packet.failure_flags = failure_flags;
    packet.type = type;
    packet.autopilot = autopilot;
    packet.heading = heading;
    packet.target_heading = target_heading;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.groundspeed = groundspeed;
    packet.windspeed = windspeed;
    packet.wind_heading = wind_heading;
    packet.eph = eph;
    packet.epv = epv;
    packet.temperature_air = temperature_air;
    packet.climb_rate = climb_rate;
    packet.battery = battery;
    packet.custom0 = custom0;
    packet.custom1 = custom1;
    packet.custom2 = custom2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGH_LATENCY2, (const char *)&packet, MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_CRC);
#endif
}

/**
 * @brief Send a high_latency2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_high_latency2_send_struct(mavlink_channel_t chan, const mavlink_high_latency2_t* high_latency2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_high_latency2_send(chan, high_latency2->timestamp, high_latency2->type, high_latency2->autopilot, high_latency2->custom_mode, high_latency2->latitude, high_latency2->longitude, high_latency2->altitude, high_latency2->target_altitude, high_latency2->heading, high_latency2->target_heading, high_latency2->target_distance, high_latency2->throttle, high_latency2->airspeed, high_latency2->airspeed_sp, high_latency2->groundspeed, high_latency2->windspeed, high_latency2->wind_heading, high_latency2->eph, high_latency2->epv, high_latency2->temperature_air, high_latency2->climb_rate, high_latency2->battery, high_latency2->wp_num, high_latency2->failure_flags, high_latency2->custom0, high_latency2->custom1, high_latency2->custom2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGH_LATENCY2, (const char *)high_latency2, MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIGH_LATENCY2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_high_latency2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t timestamp, uint8_t type, uint8_t autopilot, uint16_t custom_mode, int32_t latitude, int32_t longitude, int16_t altitude, int16_t target_altitude, uint8_t heading, uint8_t target_heading, uint16_t target_distance, uint8_t throttle, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, uint8_t windspeed, uint8_t wind_heading, uint8_t eph, uint8_t epv, int8_t temperature_air, int8_t climb_rate, int8_t battery, uint16_t wp_num, uint16_t failure_flags, int8_t custom0, int8_t custom1, int8_t custom2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_uint16_t(buf, 12, custom_mode);
    _mav_put_int16_t(buf, 14, altitude);
    _mav_put_int16_t(buf, 16, target_altitude);
    _mav_put_uint16_t(buf, 18, target_distance);
    _mav_put_uint16_t(buf, 20, wp_num);
    _mav_put_uint16_t(buf, 22, failure_flags);
    _mav_put_uint8_t(buf, 24, type);
    _mav_put_uint8_t(buf, 25, autopilot);
    _mav_put_uint8_t(buf, 26, heading);
    _mav_put_uint8_t(buf, 27, target_heading);
    _mav_put_uint8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_uint8_t(buf, 32, windspeed);
    _mav_put_uint8_t(buf, 33, wind_heading);
    _mav_put_uint8_t(buf, 34, eph);
    _mav_put_uint8_t(buf, 35, epv);
    _mav_put_int8_t(buf, 36, temperature_air);
    _mav_put_int8_t(buf, 37, climb_rate);
    _mav_put_int8_t(buf, 38, battery);
    _mav_put_int8_t(buf, 39, custom0);
    _mav_put_int8_t(buf, 40, custom1);
    _mav_put_int8_t(buf, 41, custom2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGH_LATENCY2, buf, MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_CRC);
#else
    mavlink_high_latency2_t *packet = (mavlink_high_latency2_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->custom_mode = custom_mode;
    packet->altitude = altitude;
    packet->target_altitude = target_altitude;
    packet->target_distance = target_distance;
    packet->wp_num = wp_num;
    packet->failure_flags = failure_flags;
    packet->type = type;
    packet->autopilot = autopilot;
    packet->heading = heading;
    packet->target_heading = target_heading;
    packet->throttle = throttle;
    packet->airspeed = airspeed;
    packet->airspeed_sp = airspeed_sp;
    packet->groundspeed = groundspeed;
    packet->windspeed = windspeed;
    packet->wind_heading = wind_heading;
    packet->eph = eph;
    packet->epv = epv;
    packet->temperature_air = temperature_air;
    packet->climb_rate = climb_rate;
    packet->battery = battery;
    packet->custom0 = custom0;
    packet->custom1 = custom1;
    packet->custom2 = custom2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGH_LATENCY2, (const char *)packet, MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_CRC);
#endif
}
#endif

#endif

// MESSAGE HIGH_LATENCY2 UNPACKING


/**
 * @brief Get field timestamp from high_latency2 message
 *
 * @return [ms] Timestamp (milliseconds since boot or Unix epoch)
 */
static inline uint32_t mavlink_msg_high_latency2_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field type from high_latency2 message
 *
 * @return  Type of the MAV (quadrotor, helicopter, etc.)
 */
static inline uint8_t mavlink_msg_high_latency2_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field autopilot from high_latency2 message
 *
 * @return  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
 */
static inline uint8_t mavlink_msg_high_latency2_get_autopilot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field custom_mode from high_latency2 message
 *
 * @return  A bitfield for use for autopilot-specific flags (2 byte version).
 */
static inline uint16_t mavlink_msg_high_latency2_get_custom_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field latitude from high_latency2 message
 *
 * @return [degE7] Latitude
 */
static inline int32_t mavlink_msg_high_latency2_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field longitude from high_latency2 message
 *
 * @return [degE7] Longitude
 */
static inline int32_t mavlink_msg_high_latency2_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field altitude from high_latency2 message
 *
 * @return [m] Altitude above mean sea level
 */
static inline int16_t mavlink_msg_high_latency2_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field target_altitude from high_latency2 message
 *
 * @return [m] Altitude setpoint
 */
static inline int16_t mavlink_msg_high_latency2_get_target_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field heading from high_latency2 message
 *
 * @return [deg/2] Heading
 */
static inline uint8_t mavlink_msg_high_latency2_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field target_heading from high_latency2 message
 *
 * @return [deg/2] Heading setpoint
 */
static inline uint8_t mavlink_msg_high_latency2_get_target_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field target_distance from high_latency2 message
 *
 * @return [dam] Distance to target waypoint or position
 */
static inline uint16_t mavlink_msg_high_latency2_get_target_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field throttle from high_latency2 message
 *
 * @return [%] Throttle
 */
static inline uint8_t mavlink_msg_high_latency2_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field airspeed from high_latency2 message
 *
 * @return [m/s*5] Airspeed
 */
static inline uint8_t mavlink_msg_high_latency2_get_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field airspeed_sp from high_latency2 message
 *
 * @return [m/s*5] Airspeed setpoint
 */
static inline uint8_t mavlink_msg_high_latency2_get_airspeed_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field groundspeed from high_latency2 message
 *
 * @return [m/s*5] Groundspeed
 */
static inline uint8_t mavlink_msg_high_latency2_get_groundspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field windspeed from high_latency2 message
 *
 * @return [m/s*5] Windspeed
 */
static inline uint8_t mavlink_msg_high_latency2_get_windspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field wind_heading from high_latency2 message
 *
 * @return [deg/2] Wind heading
 */
static inline uint8_t mavlink_msg_high_latency2_get_wind_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field eph from high_latency2 message
 *
 * @return [dm] Maximum error horizontal position since last message
 */
static inline uint8_t mavlink_msg_high_latency2_get_eph(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field epv from high_latency2 message
 *
 * @return [dm] Maximum error vertical position since last message
 */
static inline uint8_t mavlink_msg_high_latency2_get_epv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field temperature_air from high_latency2 message
 *
 * @return [degC] Air temperature from airspeed sensor
 */
static inline int8_t mavlink_msg_high_latency2_get_temperature_air(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  36);
}

/**
 * @brief Get field climb_rate from high_latency2 message
 *
 * @return [dm/s] Maximum climb rate magnitude since last message
 */
static inline int8_t mavlink_msg_high_latency2_get_climb_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  37);
}

/**
 * @brief Get field battery from high_latency2 message
 *
 * @return [%] Battery level (-1 if field not provided).
 */
static inline int8_t mavlink_msg_high_latency2_get_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  38);
}

/**
 * @brief Get field wp_num from high_latency2 message
 *
 * @return  Current waypoint number
 */
static inline uint16_t mavlink_msg_high_latency2_get_wp_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field failure_flags from high_latency2 message
 *
 * @return  Bitmap of failure flags.
 */
static inline uint16_t mavlink_msg_high_latency2_get_failure_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field custom0 from high_latency2 message
 *
 * @return  Field for custom payload.
 */
static inline int8_t mavlink_msg_high_latency2_get_custom0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  39);
}

/**
 * @brief Get field custom1 from high_latency2 message
 *
 * @return  Field for custom payload.
 */
static inline int8_t mavlink_msg_high_latency2_get_custom1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  40);
}

/**
 * @brief Get field custom2 from high_latency2 message
 *
 * @return  Field for custom payload.
 */
static inline int8_t mavlink_msg_high_latency2_get_custom2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  41);
}

/**
 * @brief Decode a high_latency2 message into a struct
 *
 * @param msg The message to decode
 * @param high_latency2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_high_latency2_decode(const mavlink_message_t* msg, mavlink_high_latency2_t* high_latency2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    high_latency2->timestamp = mavlink_msg_high_latency2_get_timestamp(msg);
    high_latency2->latitude = mavlink_msg_high_latency2_get_latitude(msg);
    high_latency2->longitude = mavlink_msg_high_latency2_get_longitude(msg);
    high_latency2->custom_mode = mavlink_msg_high_latency2_get_custom_mode(msg);
    high_latency2->altitude = mavlink_msg_high_latency2_get_altitude(msg);
    high_latency2->target_altitude = mavlink_msg_high_latency2_get_target_altitude(msg);
    high_latency2->target_distance = mavlink_msg_high_latency2_get_target_distance(msg);
    high_latency2->wp_num = mavlink_msg_high_latency2_get_wp_num(msg);
    high_latency2->failure_flags = mavlink_msg_high_latency2_get_failure_flags(msg);
    high_latency2->type = mavlink_msg_high_latency2_get_type(msg);
    high_latency2->autopilot = mavlink_msg_high_latency2_get_autopilot(msg);
    high_latency2->heading = mavlink_msg_high_latency2_get_heading(msg);
    high_latency2->target_heading = mavlink_msg_high_latency2_get_target_heading(msg);
    high_latency2->throttle = mavlink_msg_high_latency2_get_throttle(msg);
    high_latency2->airspeed = mavlink_msg_high_latency2_get_airspeed(msg);
    high_latency2->airspeed_sp = mavlink_msg_high_latency2_get_airspeed_sp(msg);
    high_latency2->groundspeed = mavlink_msg_high_latency2_get_groundspeed(msg);
    high_latency2->windspeed = mavlink_msg_high_latency2_get_windspeed(msg);
    high_latency2->wind_heading = mavlink_msg_high_latency2_get_wind_heading(msg);
    high_latency2->eph = mavlink_msg_high_latency2_get_eph(msg);
    high_latency2->epv = mavlink_msg_high_latency2_get_epv(msg);
    high_latency2->temperature_air = mavlink_msg_high_latency2_get_temperature_air(msg);
    high_latency2->climb_rate = mavlink_msg_high_latency2_get_climb_rate(msg);
    high_latency2->battery = mavlink_msg_high_latency2_get_battery(msg);
    high_latency2->custom0 = mavlink_msg_high_latency2_get_custom0(msg);
    high_latency2->custom1 = mavlink_msg_high_latency2_get_custom1(msg);
    high_latency2->custom2 = mavlink_msg_high_latency2_get_custom2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIGH_LATENCY2_LEN? msg->len : MAVLINK_MSG_ID_HIGH_LATENCY2_LEN;
        memset(high_latency2, 0, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN);
    memcpy(high_latency2, _MAV_PAYLOAD(msg), len);
#endif
}
