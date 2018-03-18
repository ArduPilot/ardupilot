#pragma once
// MESSAGE HIGH_LATENCY PACKING

#define MAVLINK_MSG_ID_HIGH_LATENCY 234

MAVPACKED(
typedef struct __mavlink_high_latency_t {
 uint32_t custom_mode; /*< A bitfield for use for autopilot-specific flags.*/
 int32_t latitude; /*< Latitude, expressed as degrees * 1E7*/
 int32_t longitude; /*< Longitude, expressed as degrees * 1E7*/
 int16_t roll; /*< roll (centidegrees)*/
 int16_t pitch; /*< pitch (centidegrees)*/
 uint16_t heading; /*< heading (centidegrees)*/
 int16_t heading_sp; /*< heading setpoint (centidegrees)*/
 int16_t altitude_amsl; /*< Altitude above mean sea level (meters)*/
 int16_t altitude_sp; /*< Altitude setpoint relative to the home position (meters)*/
 uint16_t wp_distance; /*< distance to target (meters)*/
 uint8_t base_mode; /*< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h*/
 uint8_t landed_state; /*< The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.*/
 int8_t throttle; /*< throttle (percentage)*/
 uint8_t airspeed; /*< airspeed (m/s)*/
 uint8_t airspeed_sp; /*< airspeed setpoint (m/s)*/
 uint8_t groundspeed; /*< groundspeed (m/s)*/
 int8_t climb_rate; /*< climb rate (m/s)*/
 uint8_t gps_nsat; /*< Number of satellites visible. If unknown, set to 255*/
 uint8_t gps_fix_type; /*< See the GPS_FIX_TYPE enum.*/
 uint8_t battery_remaining; /*< Remaining battery (percentage)*/
 int8_t temperature; /*< Autopilot temperature (degrees C)*/
 int8_t temperature_air; /*< Air temperature (degrees C) from airspeed sensor*/
 uint8_t failsafe; /*< failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)*/
 uint8_t wp_num; /*< current waypoint number*/
}) mavlink_high_latency_t;

#define MAVLINK_MSG_ID_HIGH_LATENCY_LEN 40
#define MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN 40
#define MAVLINK_MSG_ID_234_LEN 40
#define MAVLINK_MSG_ID_234_MIN_LEN 40

#define MAVLINK_MSG_ID_HIGH_LATENCY_CRC 150
#define MAVLINK_MSG_ID_234_CRC 150



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIGH_LATENCY { \
    234, \
    "HIGH_LATENCY", \
    24, \
    {  { "base_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_high_latency_t, base_mode) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_high_latency_t, custom_mode) }, \
         { "landed_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_high_latency_t, landed_state) }, \
         { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_high_latency_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_high_latency_t, pitch) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_high_latency_t, heading) }, \
         { "throttle", NULL, MAVLINK_TYPE_INT8_T, 0, 28, offsetof(mavlink_high_latency_t, throttle) }, \
         { "heading_sp", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_high_latency_t, heading_sp) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_high_latency_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_high_latency_t, longitude) }, \
         { "altitude_amsl", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_high_latency_t, altitude_amsl) }, \
         { "altitude_sp", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_high_latency_t, altitude_sp) }, \
         { "airspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_high_latency_t, airspeed) }, \
         { "airspeed_sp", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_high_latency_t, airspeed_sp) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_high_latency_t, groundspeed) }, \
         { "climb_rate", NULL, MAVLINK_TYPE_INT8_T, 0, 32, offsetof(mavlink_high_latency_t, climb_rate) }, \
         { "gps_nsat", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_high_latency_t, gps_nsat) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_high_latency_t, gps_fix_type) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_high_latency_t, battery_remaining) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_high_latency_t, temperature) }, \
         { "temperature_air", NULL, MAVLINK_TYPE_INT8_T, 0, 37, offsetof(mavlink_high_latency_t, temperature_air) }, \
         { "failsafe", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_high_latency_t, failsafe) }, \
         { "wp_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_high_latency_t, wp_num) }, \
         { "wp_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_high_latency_t, wp_distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIGH_LATENCY { \
    "HIGH_LATENCY", \
    24, \
    {  { "base_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_high_latency_t, base_mode) }, \
         { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_high_latency_t, custom_mode) }, \
         { "landed_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_high_latency_t, landed_state) }, \
         { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_high_latency_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_high_latency_t, pitch) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_high_latency_t, heading) }, \
         { "throttle", NULL, MAVLINK_TYPE_INT8_T, 0, 28, offsetof(mavlink_high_latency_t, throttle) }, \
         { "heading_sp", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_high_latency_t, heading_sp) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_high_latency_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_high_latency_t, longitude) }, \
         { "altitude_amsl", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_high_latency_t, altitude_amsl) }, \
         { "altitude_sp", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_high_latency_t, altitude_sp) }, \
         { "airspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_high_latency_t, airspeed) }, \
         { "airspeed_sp", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_high_latency_t, airspeed_sp) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_high_latency_t, groundspeed) }, \
         { "climb_rate", NULL, MAVLINK_TYPE_INT8_T, 0, 32, offsetof(mavlink_high_latency_t, climb_rate) }, \
         { "gps_nsat", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_high_latency_t, gps_nsat) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_high_latency_t, gps_fix_type) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_high_latency_t, battery_remaining) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_high_latency_t, temperature) }, \
         { "temperature_air", NULL, MAVLINK_TYPE_INT8_T, 0, 37, offsetof(mavlink_high_latency_t, temperature_air) }, \
         { "failsafe", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_high_latency_t, failsafe) }, \
         { "wp_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_high_latency_t, wp_num) }, \
         { "wp_distance", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_high_latency_t, wp_distance) }, \
         } \
}
#endif

/**
 * @brief Pack a high_latency message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param base_mode System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode A bitfield for use for autopilot-specific flags.
 * @param landed_state The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @param roll roll (centidegrees)
 * @param pitch pitch (centidegrees)
 * @param heading heading (centidegrees)
 * @param throttle throttle (percentage)
 * @param heading_sp heading setpoint (centidegrees)
 * @param latitude Latitude, expressed as degrees * 1E7
 * @param longitude Longitude, expressed as degrees * 1E7
 * @param altitude_amsl Altitude above mean sea level (meters)
 * @param altitude_sp Altitude setpoint relative to the home position (meters)
 * @param airspeed airspeed (m/s)
 * @param airspeed_sp airspeed setpoint (m/s)
 * @param groundspeed groundspeed (m/s)
 * @param climb_rate climb rate (m/s)
 * @param gps_nsat Number of satellites visible. If unknown, set to 255
 * @param gps_fix_type See the GPS_FIX_TYPE enum.
 * @param battery_remaining Remaining battery (percentage)
 * @param temperature Autopilot temperature (degrees C)
 * @param temperature_air Air temperature (degrees C) from airspeed sensor
 * @param failsafe failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
 * @param wp_num current waypoint number
 * @param wp_distance distance to target (meters)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_high_latency_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIGH_LATENCY_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, roll);
    _mav_put_int16_t(buf, 14, pitch);
    _mav_put_uint16_t(buf, 16, heading);
    _mav_put_int16_t(buf, 18, heading_sp);
    _mav_put_int16_t(buf, 20, altitude_amsl);
    _mav_put_int16_t(buf, 22, altitude_sp);
    _mav_put_uint16_t(buf, 24, wp_distance);
    _mav_put_uint8_t(buf, 26, base_mode);
    _mav_put_uint8_t(buf, 27, landed_state);
    _mav_put_int8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_int8_t(buf, 32, climb_rate);
    _mav_put_uint8_t(buf, 33, gps_nsat);
    _mav_put_uint8_t(buf, 34, gps_fix_type);
    _mav_put_uint8_t(buf, 35, battery_remaining);
    _mav_put_int8_t(buf, 36, temperature);
    _mav_put_int8_t(buf, 37, temperature_air);
    _mav_put_uint8_t(buf, 38, failsafe);
    _mav_put_uint8_t(buf, 39, wp_num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIGH_LATENCY_LEN);
#else
    mavlink_high_latency_t packet;
    packet.custom_mode = custom_mode;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.heading = heading;
    packet.heading_sp = heading_sp;
    packet.altitude_amsl = altitude_amsl;
    packet.altitude_sp = altitude_sp;
    packet.wp_distance = wp_distance;
    packet.base_mode = base_mode;
    packet.landed_state = landed_state;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.groundspeed = groundspeed;
    packet.climb_rate = climb_rate;
    packet.gps_nsat = gps_nsat;
    packet.gps_fix_type = gps_fix_type;
    packet.battery_remaining = battery_remaining;
    packet.temperature = temperature;
    packet.temperature_air = temperature_air;
    packet.failsafe = failsafe;
    packet.wp_num = wp_num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIGH_LATENCY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIGH_LATENCY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_CRC);
}

/**
 * @brief Pack a high_latency message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param base_mode System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode A bitfield for use for autopilot-specific flags.
 * @param landed_state The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @param roll roll (centidegrees)
 * @param pitch pitch (centidegrees)
 * @param heading heading (centidegrees)
 * @param throttle throttle (percentage)
 * @param heading_sp heading setpoint (centidegrees)
 * @param latitude Latitude, expressed as degrees * 1E7
 * @param longitude Longitude, expressed as degrees * 1E7
 * @param altitude_amsl Altitude above mean sea level (meters)
 * @param altitude_sp Altitude setpoint relative to the home position (meters)
 * @param airspeed airspeed (m/s)
 * @param airspeed_sp airspeed setpoint (m/s)
 * @param groundspeed groundspeed (m/s)
 * @param climb_rate climb rate (m/s)
 * @param gps_nsat Number of satellites visible. If unknown, set to 255
 * @param gps_fix_type See the GPS_FIX_TYPE enum.
 * @param battery_remaining Remaining battery (percentage)
 * @param temperature Autopilot temperature (degrees C)
 * @param temperature_air Air temperature (degrees C) from airspeed sensor
 * @param failsafe failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
 * @param wp_num current waypoint number
 * @param wp_distance distance to target (meters)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_high_latency_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t base_mode,uint32_t custom_mode,uint8_t landed_state,int16_t roll,int16_t pitch,uint16_t heading,int8_t throttle,int16_t heading_sp,int32_t latitude,int32_t longitude,int16_t altitude_amsl,int16_t altitude_sp,uint8_t airspeed,uint8_t airspeed_sp,uint8_t groundspeed,int8_t climb_rate,uint8_t gps_nsat,uint8_t gps_fix_type,uint8_t battery_remaining,int8_t temperature,int8_t temperature_air,uint8_t failsafe,uint8_t wp_num,uint16_t wp_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIGH_LATENCY_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, roll);
    _mav_put_int16_t(buf, 14, pitch);
    _mav_put_uint16_t(buf, 16, heading);
    _mav_put_int16_t(buf, 18, heading_sp);
    _mav_put_int16_t(buf, 20, altitude_amsl);
    _mav_put_int16_t(buf, 22, altitude_sp);
    _mav_put_uint16_t(buf, 24, wp_distance);
    _mav_put_uint8_t(buf, 26, base_mode);
    _mav_put_uint8_t(buf, 27, landed_state);
    _mav_put_int8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_int8_t(buf, 32, climb_rate);
    _mav_put_uint8_t(buf, 33, gps_nsat);
    _mav_put_uint8_t(buf, 34, gps_fix_type);
    _mav_put_uint8_t(buf, 35, battery_remaining);
    _mav_put_int8_t(buf, 36, temperature);
    _mav_put_int8_t(buf, 37, temperature_air);
    _mav_put_uint8_t(buf, 38, failsafe);
    _mav_put_uint8_t(buf, 39, wp_num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIGH_LATENCY_LEN);
#else
    mavlink_high_latency_t packet;
    packet.custom_mode = custom_mode;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.heading = heading;
    packet.heading_sp = heading_sp;
    packet.altitude_amsl = altitude_amsl;
    packet.altitude_sp = altitude_sp;
    packet.wp_distance = wp_distance;
    packet.base_mode = base_mode;
    packet.landed_state = landed_state;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.groundspeed = groundspeed;
    packet.climb_rate = climb_rate;
    packet.gps_nsat = gps_nsat;
    packet.gps_fix_type = gps_fix_type;
    packet.battery_remaining = battery_remaining;
    packet.temperature = temperature;
    packet.temperature_air = temperature_air;
    packet.failsafe = failsafe;
    packet.wp_num = wp_num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIGH_LATENCY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIGH_LATENCY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_CRC);
}

/**
 * @brief Encode a high_latency struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param high_latency C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_high_latency_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_high_latency_t* high_latency)
{
    return mavlink_msg_high_latency_pack(system_id, component_id, msg, high_latency->base_mode, high_latency->custom_mode, high_latency->landed_state, high_latency->roll, high_latency->pitch, high_latency->heading, high_latency->throttle, high_latency->heading_sp, high_latency->latitude, high_latency->longitude, high_latency->altitude_amsl, high_latency->altitude_sp, high_latency->airspeed, high_latency->airspeed_sp, high_latency->groundspeed, high_latency->climb_rate, high_latency->gps_nsat, high_latency->gps_fix_type, high_latency->battery_remaining, high_latency->temperature, high_latency->temperature_air, high_latency->failsafe, high_latency->wp_num, high_latency->wp_distance);
}

/**
 * @brief Encode a high_latency struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param high_latency C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_high_latency_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_high_latency_t* high_latency)
{
    return mavlink_msg_high_latency_pack_chan(system_id, component_id, chan, msg, high_latency->base_mode, high_latency->custom_mode, high_latency->landed_state, high_latency->roll, high_latency->pitch, high_latency->heading, high_latency->throttle, high_latency->heading_sp, high_latency->latitude, high_latency->longitude, high_latency->altitude_amsl, high_latency->altitude_sp, high_latency->airspeed, high_latency->airspeed_sp, high_latency->groundspeed, high_latency->climb_rate, high_latency->gps_nsat, high_latency->gps_fix_type, high_latency->battery_remaining, high_latency->temperature, high_latency->temperature_air, high_latency->failsafe, high_latency->wp_num, high_latency->wp_distance);
}

/**
 * @brief Send a high_latency message
 * @param chan MAVLink channel to send the message
 *
 * @param base_mode System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode A bitfield for use for autopilot-specific flags.
 * @param landed_state The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 * @param roll roll (centidegrees)
 * @param pitch pitch (centidegrees)
 * @param heading heading (centidegrees)
 * @param throttle throttle (percentage)
 * @param heading_sp heading setpoint (centidegrees)
 * @param latitude Latitude, expressed as degrees * 1E7
 * @param longitude Longitude, expressed as degrees * 1E7
 * @param altitude_amsl Altitude above mean sea level (meters)
 * @param altitude_sp Altitude setpoint relative to the home position (meters)
 * @param airspeed airspeed (m/s)
 * @param airspeed_sp airspeed setpoint (m/s)
 * @param groundspeed groundspeed (m/s)
 * @param climb_rate climb rate (m/s)
 * @param gps_nsat Number of satellites visible. If unknown, set to 255
 * @param gps_fix_type See the GPS_FIX_TYPE enum.
 * @param battery_remaining Remaining battery (percentage)
 * @param temperature Autopilot temperature (degrees C)
 * @param temperature_air Air temperature (degrees C) from airspeed sensor
 * @param failsafe failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
 * @param wp_num current waypoint number
 * @param wp_distance distance to target (meters)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_high_latency_send(mavlink_channel_t chan, uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIGH_LATENCY_LEN];
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, roll);
    _mav_put_int16_t(buf, 14, pitch);
    _mav_put_uint16_t(buf, 16, heading);
    _mav_put_int16_t(buf, 18, heading_sp);
    _mav_put_int16_t(buf, 20, altitude_amsl);
    _mav_put_int16_t(buf, 22, altitude_sp);
    _mav_put_uint16_t(buf, 24, wp_distance);
    _mav_put_uint8_t(buf, 26, base_mode);
    _mav_put_uint8_t(buf, 27, landed_state);
    _mav_put_int8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_int8_t(buf, 32, climb_rate);
    _mav_put_uint8_t(buf, 33, gps_nsat);
    _mav_put_uint8_t(buf, 34, gps_fix_type);
    _mav_put_uint8_t(buf, 35, battery_remaining);
    _mav_put_int8_t(buf, 36, temperature);
    _mav_put_int8_t(buf, 37, temperature_air);
    _mav_put_uint8_t(buf, 38, failsafe);
    _mav_put_uint8_t(buf, 39, wp_num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGH_LATENCY, buf, MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_CRC);
#else
    mavlink_high_latency_t packet;
    packet.custom_mode = custom_mode;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.heading = heading;
    packet.heading_sp = heading_sp;
    packet.altitude_amsl = altitude_amsl;
    packet.altitude_sp = altitude_sp;
    packet.wp_distance = wp_distance;
    packet.base_mode = base_mode;
    packet.landed_state = landed_state;
    packet.throttle = throttle;
    packet.airspeed = airspeed;
    packet.airspeed_sp = airspeed_sp;
    packet.groundspeed = groundspeed;
    packet.climb_rate = climb_rate;
    packet.gps_nsat = gps_nsat;
    packet.gps_fix_type = gps_fix_type;
    packet.battery_remaining = battery_remaining;
    packet.temperature = temperature;
    packet.temperature_air = temperature_air;
    packet.failsafe = failsafe;
    packet.wp_num = wp_num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGH_LATENCY, (const char *)&packet, MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_CRC);
#endif
}

/**
 * @brief Send a high_latency message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_high_latency_send_struct(mavlink_channel_t chan, const mavlink_high_latency_t* high_latency)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_high_latency_send(chan, high_latency->base_mode, high_latency->custom_mode, high_latency->landed_state, high_latency->roll, high_latency->pitch, high_latency->heading, high_latency->throttle, high_latency->heading_sp, high_latency->latitude, high_latency->longitude, high_latency->altitude_amsl, high_latency->altitude_sp, high_latency->airspeed, high_latency->airspeed_sp, high_latency->groundspeed, high_latency->climb_rate, high_latency->gps_nsat, high_latency->gps_fix_type, high_latency->battery_remaining, high_latency->temperature, high_latency->temperature_air, high_latency->failsafe, high_latency->wp_num, high_latency->wp_distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGH_LATENCY, (const char *)high_latency, MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIGH_LATENCY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_high_latency_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t base_mode, uint32_t custom_mode, uint8_t landed_state, int16_t roll, int16_t pitch, uint16_t heading, int8_t throttle, int16_t heading_sp, int32_t latitude, int32_t longitude, int16_t altitude_amsl, int16_t altitude_sp, uint8_t airspeed, uint8_t airspeed_sp, uint8_t groundspeed, int8_t climb_rate, uint8_t gps_nsat, uint8_t gps_fix_type, uint8_t battery_remaining, int8_t temperature, int8_t temperature_air, uint8_t failsafe, uint8_t wp_num, uint16_t wp_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, custom_mode);
    _mav_put_int32_t(buf, 4, latitude);
    _mav_put_int32_t(buf, 8, longitude);
    _mav_put_int16_t(buf, 12, roll);
    _mav_put_int16_t(buf, 14, pitch);
    _mav_put_uint16_t(buf, 16, heading);
    _mav_put_int16_t(buf, 18, heading_sp);
    _mav_put_int16_t(buf, 20, altitude_amsl);
    _mav_put_int16_t(buf, 22, altitude_sp);
    _mav_put_uint16_t(buf, 24, wp_distance);
    _mav_put_uint8_t(buf, 26, base_mode);
    _mav_put_uint8_t(buf, 27, landed_state);
    _mav_put_int8_t(buf, 28, throttle);
    _mav_put_uint8_t(buf, 29, airspeed);
    _mav_put_uint8_t(buf, 30, airspeed_sp);
    _mav_put_uint8_t(buf, 31, groundspeed);
    _mav_put_int8_t(buf, 32, climb_rate);
    _mav_put_uint8_t(buf, 33, gps_nsat);
    _mav_put_uint8_t(buf, 34, gps_fix_type);
    _mav_put_uint8_t(buf, 35, battery_remaining);
    _mav_put_int8_t(buf, 36, temperature);
    _mav_put_int8_t(buf, 37, temperature_air);
    _mav_put_uint8_t(buf, 38, failsafe);
    _mav_put_uint8_t(buf, 39, wp_num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGH_LATENCY, buf, MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_CRC);
#else
    mavlink_high_latency_t *packet = (mavlink_high_latency_t *)msgbuf;
    packet->custom_mode = custom_mode;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->heading = heading;
    packet->heading_sp = heading_sp;
    packet->altitude_amsl = altitude_amsl;
    packet->altitude_sp = altitude_sp;
    packet->wp_distance = wp_distance;
    packet->base_mode = base_mode;
    packet->landed_state = landed_state;
    packet->throttle = throttle;
    packet->airspeed = airspeed;
    packet->airspeed_sp = airspeed_sp;
    packet->groundspeed = groundspeed;
    packet->climb_rate = climb_rate;
    packet->gps_nsat = gps_nsat;
    packet->gps_fix_type = gps_fix_type;
    packet->battery_remaining = battery_remaining;
    packet->temperature = temperature;
    packet->temperature_air = temperature_air;
    packet->failsafe = failsafe;
    packet->wp_num = wp_num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGH_LATENCY, (const char *)packet, MAVLINK_MSG_ID_HIGH_LATENCY_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_LEN, MAVLINK_MSG_ID_HIGH_LATENCY_CRC);
#endif
}
#endif

#endif

// MESSAGE HIGH_LATENCY UNPACKING


/**
 * @brief Get field base_mode from high_latency message
 *
 * @return System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
 */
static inline uint8_t mavlink_msg_high_latency_get_base_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field custom_mode from high_latency message
 *
 * @return A bitfield for use for autopilot-specific flags.
 */
static inline uint32_t mavlink_msg_high_latency_get_custom_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field landed_state from high_latency message
 *
 * @return The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
 */
static inline uint8_t mavlink_msg_high_latency_get_landed_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field roll from high_latency message
 *
 * @return roll (centidegrees)
 */
static inline int16_t mavlink_msg_high_latency_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field pitch from high_latency message
 *
 * @return pitch (centidegrees)
 */
static inline int16_t mavlink_msg_high_latency_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field heading from high_latency message
 *
 * @return heading (centidegrees)
 */
static inline uint16_t mavlink_msg_high_latency_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field throttle from high_latency message
 *
 * @return throttle (percentage)
 */
static inline int8_t mavlink_msg_high_latency_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  28);
}

/**
 * @brief Get field heading_sp from high_latency message
 *
 * @return heading setpoint (centidegrees)
 */
static inline int16_t mavlink_msg_high_latency_get_heading_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field latitude from high_latency message
 *
 * @return Latitude, expressed as degrees * 1E7
 */
static inline int32_t mavlink_msg_high_latency_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field longitude from high_latency message
 *
 * @return Longitude, expressed as degrees * 1E7
 */
static inline int32_t mavlink_msg_high_latency_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field altitude_amsl from high_latency message
 *
 * @return Altitude above mean sea level (meters)
 */
static inline int16_t mavlink_msg_high_latency_get_altitude_amsl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field altitude_sp from high_latency message
 *
 * @return Altitude setpoint relative to the home position (meters)
 */
static inline int16_t mavlink_msg_high_latency_get_altitude_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field airspeed from high_latency message
 *
 * @return airspeed (m/s)
 */
static inline uint8_t mavlink_msg_high_latency_get_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field airspeed_sp from high_latency message
 *
 * @return airspeed setpoint (m/s)
 */
static inline uint8_t mavlink_msg_high_latency_get_airspeed_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field groundspeed from high_latency message
 *
 * @return groundspeed (m/s)
 */
static inline uint8_t mavlink_msg_high_latency_get_groundspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field climb_rate from high_latency message
 *
 * @return climb rate (m/s)
 */
static inline int8_t mavlink_msg_high_latency_get_climb_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  32);
}

/**
 * @brief Get field gps_nsat from high_latency message
 *
 * @return Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_high_latency_get_gps_nsat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field gps_fix_type from high_latency message
 *
 * @return See the GPS_FIX_TYPE enum.
 */
static inline uint8_t mavlink_msg_high_latency_get_gps_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field battery_remaining from high_latency message
 *
 * @return Remaining battery (percentage)
 */
static inline uint8_t mavlink_msg_high_latency_get_battery_remaining(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field temperature from high_latency message
 *
 * @return Autopilot temperature (degrees C)
 */
static inline int8_t mavlink_msg_high_latency_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  36);
}

/**
 * @brief Get field temperature_air from high_latency message
 *
 * @return Air temperature (degrees C) from airspeed sensor
 */
static inline int8_t mavlink_msg_high_latency_get_temperature_air(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  37);
}

/**
 * @brief Get field failsafe from high_latency message
 *
 * @return failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
 */
static inline uint8_t mavlink_msg_high_latency_get_failsafe(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field wp_num from high_latency message
 *
 * @return current waypoint number
 */
static inline uint8_t mavlink_msg_high_latency_get_wp_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field wp_distance from high_latency message
 *
 * @return distance to target (meters)
 */
static inline uint16_t mavlink_msg_high_latency_get_wp_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Decode a high_latency message into a struct
 *
 * @param msg The message to decode
 * @param high_latency C-struct to decode the message contents into
 */
static inline void mavlink_msg_high_latency_decode(const mavlink_message_t* msg, mavlink_high_latency_t* high_latency)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    high_latency->custom_mode = mavlink_msg_high_latency_get_custom_mode(msg);
    high_latency->latitude = mavlink_msg_high_latency_get_latitude(msg);
    high_latency->longitude = mavlink_msg_high_latency_get_longitude(msg);
    high_latency->roll = mavlink_msg_high_latency_get_roll(msg);
    high_latency->pitch = mavlink_msg_high_latency_get_pitch(msg);
    high_latency->heading = mavlink_msg_high_latency_get_heading(msg);
    high_latency->heading_sp = mavlink_msg_high_latency_get_heading_sp(msg);
    high_latency->altitude_amsl = mavlink_msg_high_latency_get_altitude_amsl(msg);
    high_latency->altitude_sp = mavlink_msg_high_latency_get_altitude_sp(msg);
    high_latency->wp_distance = mavlink_msg_high_latency_get_wp_distance(msg);
    high_latency->base_mode = mavlink_msg_high_latency_get_base_mode(msg);
    high_latency->landed_state = mavlink_msg_high_latency_get_landed_state(msg);
    high_latency->throttle = mavlink_msg_high_latency_get_throttle(msg);
    high_latency->airspeed = mavlink_msg_high_latency_get_airspeed(msg);
    high_latency->airspeed_sp = mavlink_msg_high_latency_get_airspeed_sp(msg);
    high_latency->groundspeed = mavlink_msg_high_latency_get_groundspeed(msg);
    high_latency->climb_rate = mavlink_msg_high_latency_get_climb_rate(msg);
    high_latency->gps_nsat = mavlink_msg_high_latency_get_gps_nsat(msg);
    high_latency->gps_fix_type = mavlink_msg_high_latency_get_gps_fix_type(msg);
    high_latency->battery_remaining = mavlink_msg_high_latency_get_battery_remaining(msg);
    high_latency->temperature = mavlink_msg_high_latency_get_temperature(msg);
    high_latency->temperature_air = mavlink_msg_high_latency_get_temperature_air(msg);
    high_latency->failsafe = mavlink_msg_high_latency_get_failsafe(msg);
    high_latency->wp_num = mavlink_msg_high_latency_get_wp_num(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIGH_LATENCY_LEN? msg->len : MAVLINK_MSG_ID_HIGH_LATENCY_LEN;
        memset(high_latency, 0, MAVLINK_MSG_ID_HIGH_LATENCY_LEN);
    memcpy(high_latency, _MAV_PAYLOAD(msg), len);
#endif
}
