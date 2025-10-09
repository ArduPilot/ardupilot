#pragma once
// MESSAGE COMMAND_INT_STAMPED PACKING

#define MAVLINK_MSG_ID_COMMAND_INT_STAMPED 223


typedef struct __mavlink_command_int_stamped_t {
 uint64_t vehicle_timestamp; /*<  Microseconds elapsed since vehicle boot*/
 uint32_t utc_time; /*<  UTC time, seconds elapsed since 01.01.1970*/
 float param1; /*<  PARAM1, see MAV_CMD enum*/
 float param2; /*<  PARAM2, see MAV_CMD enum*/
 float param3; /*<  PARAM3, see MAV_CMD enum*/
 float param4; /*<  PARAM4, see MAV_CMD enum*/
 int32_t x; /*<  PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7*/
 int32_t y; /*<  PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7*/
 float z; /*<  PARAM7 / z position: global: altitude in meters (MSL, WGS84, AGL or relative to home - depending on frame).*/
 uint16_t command; /*<  The scheduled action for the mission item, as defined by MAV_CMD enum*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t frame; /*<  The coordinate system of the COMMAND, as defined by MAV_FRAME enum*/
 uint8_t current; /*<  false:0, true:1*/
 uint8_t autocontinue; /*<  autocontinue to next wp*/
} mavlink_command_int_stamped_t;

#define MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN 47
#define MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN 47
#define MAVLINK_MSG_ID_223_LEN 47
#define MAVLINK_MSG_ID_223_MIN_LEN 47

#define MAVLINK_MSG_ID_COMMAND_INT_STAMPED_CRC 119
#define MAVLINK_MSG_ID_223_CRC 119



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMMAND_INT_STAMPED { \
    223, \
    "COMMAND_INT_STAMPED", \
    15, \
    {  { "utc_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_command_int_stamped_t, utc_time) }, \
         { "vehicle_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_command_int_stamped_t, vehicle_timestamp) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_command_int_stamped_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_command_int_stamped_t, target_component) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_command_int_stamped_t, frame) }, \
         { "command", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_command_int_stamped_t, command) }, \
         { "current", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_command_int_stamped_t, current) }, \
         { "autocontinue", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_command_int_stamped_t, autocontinue) }, \
         { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_command_int_stamped_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_command_int_stamped_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_command_int_stamped_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_command_int_stamped_t, param4) }, \
         { "x", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_command_int_stamped_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_command_int_stamped_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_command_int_stamped_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COMMAND_INT_STAMPED { \
    "COMMAND_INT_STAMPED", \
    15, \
    {  { "utc_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_command_int_stamped_t, utc_time) }, \
         { "vehicle_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_command_int_stamped_t, vehicle_timestamp) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_command_int_stamped_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_command_int_stamped_t, target_component) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_command_int_stamped_t, frame) }, \
         { "command", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_command_int_stamped_t, command) }, \
         { "current", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_command_int_stamped_t, current) }, \
         { "autocontinue", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_command_int_stamped_t, autocontinue) }, \
         { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_command_int_stamped_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_command_int_stamped_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_command_int_stamped_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_command_int_stamped_t, param4) }, \
         { "x", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_command_int_stamped_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_command_int_stamped_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_command_int_stamped_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a command_int_stamped message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param utc_time  UTC time, seconds elapsed since 01.01.1970
 * @param vehicle_timestamp  Microseconds elapsed since vehicle boot
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  The coordinate system of the COMMAND, as defined by MAV_FRAME enum
 * @param command  The scheduled action for the mission item, as defined by MAV_CMD enum
 * @param current  false:0, true:1
 * @param autocontinue  autocontinue to next wp
 * @param param1  PARAM1, see MAV_CMD enum
 * @param param2  PARAM2, see MAV_CMD enum
 * @param param3  PARAM3, see MAV_CMD enum
 * @param param4  PARAM4, see MAV_CMD enum
 * @param x  PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
 * @param y  PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
 * @param z  PARAM7 / z position: global: altitude in meters (MSL, WGS84, AGL or relative to home - depending on frame).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_int_stamped_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t utc_time, uint64_t vehicle_timestamp, uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN];
    _mav_put_uint64_t(buf, 0, vehicle_timestamp);
    _mav_put_uint32_t(buf, 8, utc_time);
    _mav_put_float(buf, 12, param1);
    _mav_put_float(buf, 16, param2);
    _mav_put_float(buf, 20, param3);
    _mav_put_float(buf, 24, param4);
    _mav_put_int32_t(buf, 28, x);
    _mav_put_int32_t(buf, 32, y);
    _mav_put_float(buf, 36, z);
    _mav_put_uint16_t(buf, 40, command);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, target_component);
    _mav_put_uint8_t(buf, 44, frame);
    _mav_put_uint8_t(buf, 45, current);
    _mav_put_uint8_t(buf, 46, autocontinue);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN);
#else
    mavlink_command_int_stamped_t packet;
    packet.vehicle_timestamp = vehicle_timestamp;
    packet.utc_time = utc_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_INT_STAMPED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_CRC);
}

/**
 * @brief Pack a command_int_stamped message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param utc_time  UTC time, seconds elapsed since 01.01.1970
 * @param vehicle_timestamp  Microseconds elapsed since vehicle boot
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  The coordinate system of the COMMAND, as defined by MAV_FRAME enum
 * @param command  The scheduled action for the mission item, as defined by MAV_CMD enum
 * @param current  false:0, true:1
 * @param autocontinue  autocontinue to next wp
 * @param param1  PARAM1, see MAV_CMD enum
 * @param param2  PARAM2, see MAV_CMD enum
 * @param param3  PARAM3, see MAV_CMD enum
 * @param param4  PARAM4, see MAV_CMD enum
 * @param x  PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
 * @param y  PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
 * @param z  PARAM7 / z position: global: altitude in meters (MSL, WGS84, AGL or relative to home - depending on frame).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_int_stamped_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t utc_time, uint64_t vehicle_timestamp, uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN];
    _mav_put_uint64_t(buf, 0, vehicle_timestamp);
    _mav_put_uint32_t(buf, 8, utc_time);
    _mav_put_float(buf, 12, param1);
    _mav_put_float(buf, 16, param2);
    _mav_put_float(buf, 20, param3);
    _mav_put_float(buf, 24, param4);
    _mav_put_int32_t(buf, 28, x);
    _mav_put_int32_t(buf, 32, y);
    _mav_put_float(buf, 36, z);
    _mav_put_uint16_t(buf, 40, command);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, target_component);
    _mav_put_uint8_t(buf, 44, frame);
    _mav_put_uint8_t(buf, 45, current);
    _mav_put_uint8_t(buf, 46, autocontinue);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN);
#else
    mavlink_command_int_stamped_t packet;
    packet.vehicle_timestamp = vehicle_timestamp;
    packet.utc_time = utc_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_INT_STAMPED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN);
#endif
}

/**
 * @brief Pack a command_int_stamped message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param utc_time  UTC time, seconds elapsed since 01.01.1970
 * @param vehicle_timestamp  Microseconds elapsed since vehicle boot
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  The coordinate system of the COMMAND, as defined by MAV_FRAME enum
 * @param command  The scheduled action for the mission item, as defined by MAV_CMD enum
 * @param current  false:0, true:1
 * @param autocontinue  autocontinue to next wp
 * @param param1  PARAM1, see MAV_CMD enum
 * @param param2  PARAM2, see MAV_CMD enum
 * @param param3  PARAM3, see MAV_CMD enum
 * @param param4  PARAM4, see MAV_CMD enum
 * @param x  PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
 * @param y  PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
 * @param z  PARAM7 / z position: global: altitude in meters (MSL, WGS84, AGL or relative to home - depending on frame).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_int_stamped_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t utc_time,uint64_t vehicle_timestamp,uint8_t target_system,uint8_t target_component,uint8_t frame,uint16_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,int32_t x,int32_t y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN];
    _mav_put_uint64_t(buf, 0, vehicle_timestamp);
    _mav_put_uint32_t(buf, 8, utc_time);
    _mav_put_float(buf, 12, param1);
    _mav_put_float(buf, 16, param2);
    _mav_put_float(buf, 20, param3);
    _mav_put_float(buf, 24, param4);
    _mav_put_int32_t(buf, 28, x);
    _mav_put_int32_t(buf, 32, y);
    _mav_put_float(buf, 36, z);
    _mav_put_uint16_t(buf, 40, command);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, target_component);
    _mav_put_uint8_t(buf, 44, frame);
    _mav_put_uint8_t(buf, 45, current);
    _mav_put_uint8_t(buf, 46, autocontinue);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN);
#else
    mavlink_command_int_stamped_t packet;
    packet.vehicle_timestamp = vehicle_timestamp;
    packet.utc_time = utc_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_INT_STAMPED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_CRC);
}

/**
 * @brief Encode a command_int_stamped struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_int_stamped C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_int_stamped_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_int_stamped_t* command_int_stamped)
{
    return mavlink_msg_command_int_stamped_pack(system_id, component_id, msg, command_int_stamped->utc_time, command_int_stamped->vehicle_timestamp, command_int_stamped->target_system, command_int_stamped->target_component, command_int_stamped->frame, command_int_stamped->command, command_int_stamped->current, command_int_stamped->autocontinue, command_int_stamped->param1, command_int_stamped->param2, command_int_stamped->param3, command_int_stamped->param4, command_int_stamped->x, command_int_stamped->y, command_int_stamped->z);
}

/**
 * @brief Encode a command_int_stamped struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_int_stamped C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_int_stamped_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_command_int_stamped_t* command_int_stamped)
{
    return mavlink_msg_command_int_stamped_pack_chan(system_id, component_id, chan, msg, command_int_stamped->utc_time, command_int_stamped->vehicle_timestamp, command_int_stamped->target_system, command_int_stamped->target_component, command_int_stamped->frame, command_int_stamped->command, command_int_stamped->current, command_int_stamped->autocontinue, command_int_stamped->param1, command_int_stamped->param2, command_int_stamped->param3, command_int_stamped->param4, command_int_stamped->x, command_int_stamped->y, command_int_stamped->z);
}

/**
 * @brief Encode a command_int_stamped struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param command_int_stamped C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_int_stamped_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_command_int_stamped_t* command_int_stamped)
{
    return mavlink_msg_command_int_stamped_pack_status(system_id, component_id, _status, msg,  command_int_stamped->utc_time, command_int_stamped->vehicle_timestamp, command_int_stamped->target_system, command_int_stamped->target_component, command_int_stamped->frame, command_int_stamped->command, command_int_stamped->current, command_int_stamped->autocontinue, command_int_stamped->param1, command_int_stamped->param2, command_int_stamped->param3, command_int_stamped->param4, command_int_stamped->x, command_int_stamped->y, command_int_stamped->z);
}

/**
 * @brief Send a command_int_stamped message
 * @param chan MAVLink channel to send the message
 *
 * @param utc_time  UTC time, seconds elapsed since 01.01.1970
 * @param vehicle_timestamp  Microseconds elapsed since vehicle boot
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  The coordinate system of the COMMAND, as defined by MAV_FRAME enum
 * @param command  The scheduled action for the mission item, as defined by MAV_CMD enum
 * @param current  false:0, true:1
 * @param autocontinue  autocontinue to next wp
 * @param param1  PARAM1, see MAV_CMD enum
 * @param param2  PARAM2, see MAV_CMD enum
 * @param param3  PARAM3, see MAV_CMD enum
 * @param param4  PARAM4, see MAV_CMD enum
 * @param x  PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
 * @param y  PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
 * @param z  PARAM7 / z position: global: altitude in meters (MSL, WGS84, AGL or relative to home - depending on frame).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_int_stamped_send(mavlink_channel_t chan, uint32_t utc_time, uint64_t vehicle_timestamp, uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN];
    _mav_put_uint64_t(buf, 0, vehicle_timestamp);
    _mav_put_uint32_t(buf, 8, utc_time);
    _mav_put_float(buf, 12, param1);
    _mav_put_float(buf, 16, param2);
    _mav_put_float(buf, 20, param3);
    _mav_put_float(buf, 24, param4);
    _mav_put_int32_t(buf, 28, x);
    _mav_put_int32_t(buf, 32, y);
    _mav_put_float(buf, 36, z);
    _mav_put_uint16_t(buf, 40, command);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, target_component);
    _mav_put_uint8_t(buf, 44, frame);
    _mav_put_uint8_t(buf, 45, current);
    _mav_put_uint8_t(buf, 46, autocontinue);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_INT_STAMPED, buf, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_CRC);
#else
    mavlink_command_int_stamped_t packet;
    packet.vehicle_timestamp = vehicle_timestamp;
    packet.utc_time = utc_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;
    packet.current = current;
    packet.autocontinue = autocontinue;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_INT_STAMPED, (const char *)&packet, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_CRC);
#endif
}

/**
 * @brief Send a command_int_stamped message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_command_int_stamped_send_struct(mavlink_channel_t chan, const mavlink_command_int_stamped_t* command_int_stamped)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_command_int_stamped_send(chan, command_int_stamped->utc_time, command_int_stamped->vehicle_timestamp, command_int_stamped->target_system, command_int_stamped->target_component, command_int_stamped->frame, command_int_stamped->command, command_int_stamped->current, command_int_stamped->autocontinue, command_int_stamped->param1, command_int_stamped->param2, command_int_stamped->param3, command_int_stamped->param4, command_int_stamped->x, command_int_stamped->y, command_int_stamped->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_INT_STAMPED, (const char *)command_int_stamped, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_command_int_stamped_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t utc_time, uint64_t vehicle_timestamp, uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, vehicle_timestamp);
    _mav_put_uint32_t(buf, 8, utc_time);
    _mav_put_float(buf, 12, param1);
    _mav_put_float(buf, 16, param2);
    _mav_put_float(buf, 20, param3);
    _mav_put_float(buf, 24, param4);
    _mav_put_int32_t(buf, 28, x);
    _mav_put_int32_t(buf, 32, y);
    _mav_put_float(buf, 36, z);
    _mav_put_uint16_t(buf, 40, command);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, target_component);
    _mav_put_uint8_t(buf, 44, frame);
    _mav_put_uint8_t(buf, 45, current);
    _mav_put_uint8_t(buf, 46, autocontinue);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_INT_STAMPED, buf, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_CRC);
#else
    mavlink_command_int_stamped_t *packet = (mavlink_command_int_stamped_t *)msgbuf;
    packet->vehicle_timestamp = vehicle_timestamp;
    packet->utc_time = utc_time;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->command = command;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->frame = frame;
    packet->current = current;
    packet->autocontinue = autocontinue;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_INT_STAMPED, (const char *)packet, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND_INT_STAMPED UNPACKING


/**
 * @brief Get field utc_time from command_int_stamped message
 *
 * @return  UTC time, seconds elapsed since 01.01.1970
 */
static inline uint32_t mavlink_msg_command_int_stamped_get_utc_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field vehicle_timestamp from command_int_stamped message
 *
 * @return  Microseconds elapsed since vehicle boot
 */
static inline uint64_t mavlink_msg_command_int_stamped_get_vehicle_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_system from command_int_stamped message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_command_int_stamped_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field target_component from command_int_stamped message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_command_int_stamped_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field frame from command_int_stamped message
 *
 * @return  The coordinate system of the COMMAND, as defined by MAV_FRAME enum
 */
static inline uint8_t mavlink_msg_command_int_stamped_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field command from command_int_stamped message
 *
 * @return  The scheduled action for the mission item, as defined by MAV_CMD enum
 */
static inline uint16_t mavlink_msg_command_int_stamped_get_command(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field current from command_int_stamped message
 *
 * @return  false:0, true:1
 */
static inline uint8_t mavlink_msg_command_int_stamped_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field autocontinue from command_int_stamped message
 *
 * @return  autocontinue to next wp
 */
static inline uint8_t mavlink_msg_command_int_stamped_get_autocontinue(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field param1 from command_int_stamped message
 *
 * @return  PARAM1, see MAV_CMD enum
 */
static inline float mavlink_msg_command_int_stamped_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field param2 from command_int_stamped message
 *
 * @return  PARAM2, see MAV_CMD enum
 */
static inline float mavlink_msg_command_int_stamped_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field param3 from command_int_stamped message
 *
 * @return  PARAM3, see MAV_CMD enum
 */
static inline float mavlink_msg_command_int_stamped_get_param3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field param4 from command_int_stamped message
 *
 * @return  PARAM4, see MAV_CMD enum
 */
static inline float mavlink_msg_command_int_stamped_get_param4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field x from command_int_stamped message
 *
 * @return  PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
 */
static inline int32_t mavlink_msg_command_int_stamped_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field y from command_int_stamped message
 *
 * @return  PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
 */
static inline int32_t mavlink_msg_command_int_stamped_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field z from command_int_stamped message
 *
 * @return  PARAM7 / z position: global: altitude in meters (MSL, WGS84, AGL or relative to home - depending on frame).
 */
static inline float mavlink_msg_command_int_stamped_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a command_int_stamped message into a struct
 *
 * @param msg The message to decode
 * @param command_int_stamped C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_int_stamped_decode(const mavlink_message_t* msg, mavlink_command_int_stamped_t* command_int_stamped)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    command_int_stamped->vehicle_timestamp = mavlink_msg_command_int_stamped_get_vehicle_timestamp(msg);
    command_int_stamped->utc_time = mavlink_msg_command_int_stamped_get_utc_time(msg);
    command_int_stamped->param1 = mavlink_msg_command_int_stamped_get_param1(msg);
    command_int_stamped->param2 = mavlink_msg_command_int_stamped_get_param2(msg);
    command_int_stamped->param3 = mavlink_msg_command_int_stamped_get_param3(msg);
    command_int_stamped->param4 = mavlink_msg_command_int_stamped_get_param4(msg);
    command_int_stamped->x = mavlink_msg_command_int_stamped_get_x(msg);
    command_int_stamped->y = mavlink_msg_command_int_stamped_get_y(msg);
    command_int_stamped->z = mavlink_msg_command_int_stamped_get_z(msg);
    command_int_stamped->command = mavlink_msg_command_int_stamped_get_command(msg);
    command_int_stamped->target_system = mavlink_msg_command_int_stamped_get_target_system(msg);
    command_int_stamped->target_component = mavlink_msg_command_int_stamped_get_target_component(msg);
    command_int_stamped->frame = mavlink_msg_command_int_stamped_get_frame(msg);
    command_int_stamped->current = mavlink_msg_command_int_stamped_get_current(msg);
    command_int_stamped->autocontinue = mavlink_msg_command_int_stamped_get_autocontinue(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN? msg->len : MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN;
        memset(command_int_stamped, 0, MAVLINK_MSG_ID_COMMAND_INT_STAMPED_LEN);
    memcpy(command_int_stamped, _MAV_PAYLOAD(msg), len);
#endif
}
