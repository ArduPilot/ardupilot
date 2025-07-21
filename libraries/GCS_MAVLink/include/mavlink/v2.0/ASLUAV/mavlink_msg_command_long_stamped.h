#pragma once
// MESSAGE COMMAND_LONG_STAMPED PACKING

#define MAVLINK_MSG_ID_COMMAND_LONG_STAMPED 224


typedef struct __mavlink_command_long_stamped_t {
 uint64_t vehicle_timestamp; /*<  Microseconds elapsed since vehicle boot*/
 uint32_t utc_time; /*<  UTC time, seconds elapsed since 01.01.1970*/
 float param1; /*<  Parameter 1, as defined by MAV_CMD enum.*/
 float param2; /*<  Parameter 2, as defined by MAV_CMD enum.*/
 float param3; /*<  Parameter 3, as defined by MAV_CMD enum.*/
 float param4; /*<  Parameter 4, as defined by MAV_CMD enum.*/
 float param5; /*<  Parameter 5, as defined by MAV_CMD enum.*/
 float param6; /*<  Parameter 6, as defined by MAV_CMD enum.*/
 float param7; /*<  Parameter 7, as defined by MAV_CMD enum.*/
 uint16_t command; /*<  Command ID, as defined by MAV_CMD enum.*/
 uint8_t target_system; /*<  System which should execute the command*/
 uint8_t target_component; /*<  Component which should execute the command, 0 for all components*/
 uint8_t confirmation; /*<  0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
} mavlink_command_long_stamped_t;

#define MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN 45
#define MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN 45
#define MAVLINK_MSG_ID_224_LEN 45
#define MAVLINK_MSG_ID_224_MIN_LEN 45

#define MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_CRC 102
#define MAVLINK_MSG_ID_224_CRC 102



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_COMMAND_LONG_STAMPED { \
    224, \
    "COMMAND_LONG_STAMPED", \
    13, \
    {  { "utc_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_command_long_stamped_t, utc_time) }, \
         { "vehicle_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_command_long_stamped_t, vehicle_timestamp) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_command_long_stamped_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_command_long_stamped_t, target_component) }, \
         { "command", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_command_long_stamped_t, command) }, \
         { "confirmation", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_command_long_stamped_t, confirmation) }, \
         { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_command_long_stamped_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_command_long_stamped_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_command_long_stamped_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_command_long_stamped_t, param4) }, \
         { "param5", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_command_long_stamped_t, param5) }, \
         { "param6", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_command_long_stamped_t, param6) }, \
         { "param7", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_command_long_stamped_t, param7) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_COMMAND_LONG_STAMPED { \
    "COMMAND_LONG_STAMPED", \
    13, \
    {  { "utc_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_command_long_stamped_t, utc_time) }, \
         { "vehicle_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_command_long_stamped_t, vehicle_timestamp) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_command_long_stamped_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_command_long_stamped_t, target_component) }, \
         { "command", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_command_long_stamped_t, command) }, \
         { "confirmation", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_command_long_stamped_t, confirmation) }, \
         { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_command_long_stamped_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_command_long_stamped_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_command_long_stamped_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_command_long_stamped_t, param4) }, \
         { "param5", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_command_long_stamped_t, param5) }, \
         { "param6", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_command_long_stamped_t, param6) }, \
         { "param7", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_command_long_stamped_t, param7) }, \
         } \
}
#endif

/**
 * @brief Pack a command_long_stamped message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param utc_time  UTC time, seconds elapsed since 01.01.1970
 * @param vehicle_timestamp  Microseconds elapsed since vehicle boot
 * @param target_system  System which should execute the command
 * @param target_component  Component which should execute the command, 0 for all components
 * @param command  Command ID, as defined by MAV_CMD enum.
 * @param confirmation  0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1  Parameter 1, as defined by MAV_CMD enum.
 * @param param2  Parameter 2, as defined by MAV_CMD enum.
 * @param param3  Parameter 3, as defined by MAV_CMD enum.
 * @param param4  Parameter 4, as defined by MAV_CMD enum.
 * @param param5  Parameter 5, as defined by MAV_CMD enum.
 * @param param6  Parameter 6, as defined by MAV_CMD enum.
 * @param param7  Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_long_stamped_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t utc_time, uint64_t vehicle_timestamp, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN];
    _mav_put_uint64_t(buf, 0, vehicle_timestamp);
    _mav_put_uint32_t(buf, 8, utc_time);
    _mav_put_float(buf, 12, param1);
    _mav_put_float(buf, 16, param2);
    _mav_put_float(buf, 20, param3);
    _mav_put_float(buf, 24, param4);
    _mav_put_float(buf, 28, param5);
    _mav_put_float(buf, 32, param6);
    _mav_put_float(buf, 36, param7);
    _mav_put_uint16_t(buf, 40, command);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, target_component);
    _mav_put_uint8_t(buf, 44, confirmation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN);
#else
    mavlink_command_long_stamped_t packet;
    packet.vehicle_timestamp = vehicle_timestamp;
    packet.utc_time = utc_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.confirmation = confirmation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG_STAMPED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_CRC);
}

/**
 * @brief Pack a command_long_stamped message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param utc_time  UTC time, seconds elapsed since 01.01.1970
 * @param vehicle_timestamp  Microseconds elapsed since vehicle boot
 * @param target_system  System which should execute the command
 * @param target_component  Component which should execute the command, 0 for all components
 * @param command  Command ID, as defined by MAV_CMD enum.
 * @param confirmation  0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1  Parameter 1, as defined by MAV_CMD enum.
 * @param param2  Parameter 2, as defined by MAV_CMD enum.
 * @param param3  Parameter 3, as defined by MAV_CMD enum.
 * @param param4  Parameter 4, as defined by MAV_CMD enum.
 * @param param5  Parameter 5, as defined by MAV_CMD enum.
 * @param param6  Parameter 6, as defined by MAV_CMD enum.
 * @param param7  Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_long_stamped_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t utc_time, uint64_t vehicle_timestamp, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN];
    _mav_put_uint64_t(buf, 0, vehicle_timestamp);
    _mav_put_uint32_t(buf, 8, utc_time);
    _mav_put_float(buf, 12, param1);
    _mav_put_float(buf, 16, param2);
    _mav_put_float(buf, 20, param3);
    _mav_put_float(buf, 24, param4);
    _mav_put_float(buf, 28, param5);
    _mav_put_float(buf, 32, param6);
    _mav_put_float(buf, 36, param7);
    _mav_put_uint16_t(buf, 40, command);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, target_component);
    _mav_put_uint8_t(buf, 44, confirmation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN);
#else
    mavlink_command_long_stamped_t packet;
    packet.vehicle_timestamp = vehicle_timestamp;
    packet.utc_time = utc_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.confirmation = confirmation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG_STAMPED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN);
#endif
}

/**
 * @brief Pack a command_long_stamped message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param utc_time  UTC time, seconds elapsed since 01.01.1970
 * @param vehicle_timestamp  Microseconds elapsed since vehicle boot
 * @param target_system  System which should execute the command
 * @param target_component  Component which should execute the command, 0 for all components
 * @param command  Command ID, as defined by MAV_CMD enum.
 * @param confirmation  0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1  Parameter 1, as defined by MAV_CMD enum.
 * @param param2  Parameter 2, as defined by MAV_CMD enum.
 * @param param3  Parameter 3, as defined by MAV_CMD enum.
 * @param param4  Parameter 4, as defined by MAV_CMD enum.
 * @param param5  Parameter 5, as defined by MAV_CMD enum.
 * @param param6  Parameter 6, as defined by MAV_CMD enum.
 * @param param7  Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_long_stamped_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t utc_time,uint64_t vehicle_timestamp,uint8_t target_system,uint8_t target_component,uint16_t command,uint8_t confirmation,float param1,float param2,float param3,float param4,float param5,float param6,float param7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN];
    _mav_put_uint64_t(buf, 0, vehicle_timestamp);
    _mav_put_uint32_t(buf, 8, utc_time);
    _mav_put_float(buf, 12, param1);
    _mav_put_float(buf, 16, param2);
    _mav_put_float(buf, 20, param3);
    _mav_put_float(buf, 24, param4);
    _mav_put_float(buf, 28, param5);
    _mav_put_float(buf, 32, param6);
    _mav_put_float(buf, 36, param7);
    _mav_put_uint16_t(buf, 40, command);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, target_component);
    _mav_put_uint8_t(buf, 44, confirmation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN);
#else
    mavlink_command_long_stamped_t packet;
    packet.vehicle_timestamp = vehicle_timestamp;
    packet.utc_time = utc_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.confirmation = confirmation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG_STAMPED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_CRC);
}

/**
 * @brief Encode a command_long_stamped struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_long_stamped C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_long_stamped_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_long_stamped_t* command_long_stamped)
{
    return mavlink_msg_command_long_stamped_pack(system_id, component_id, msg, command_long_stamped->utc_time, command_long_stamped->vehicle_timestamp, command_long_stamped->target_system, command_long_stamped->target_component, command_long_stamped->command, command_long_stamped->confirmation, command_long_stamped->param1, command_long_stamped->param2, command_long_stamped->param3, command_long_stamped->param4, command_long_stamped->param5, command_long_stamped->param6, command_long_stamped->param7);
}

/**
 * @brief Encode a command_long_stamped struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command_long_stamped C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_long_stamped_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_command_long_stamped_t* command_long_stamped)
{
    return mavlink_msg_command_long_stamped_pack_chan(system_id, component_id, chan, msg, command_long_stamped->utc_time, command_long_stamped->vehicle_timestamp, command_long_stamped->target_system, command_long_stamped->target_component, command_long_stamped->command, command_long_stamped->confirmation, command_long_stamped->param1, command_long_stamped->param2, command_long_stamped->param3, command_long_stamped->param4, command_long_stamped->param5, command_long_stamped->param6, command_long_stamped->param7);
}

/**
 * @brief Encode a command_long_stamped struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param command_long_stamped C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_long_stamped_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_command_long_stamped_t* command_long_stamped)
{
    return mavlink_msg_command_long_stamped_pack_status(system_id, component_id, _status, msg,  command_long_stamped->utc_time, command_long_stamped->vehicle_timestamp, command_long_stamped->target_system, command_long_stamped->target_component, command_long_stamped->command, command_long_stamped->confirmation, command_long_stamped->param1, command_long_stamped->param2, command_long_stamped->param3, command_long_stamped->param4, command_long_stamped->param5, command_long_stamped->param6, command_long_stamped->param7);
}

/**
 * @brief Send a command_long_stamped message
 * @param chan MAVLink channel to send the message
 *
 * @param utc_time  UTC time, seconds elapsed since 01.01.1970
 * @param vehicle_timestamp  Microseconds elapsed since vehicle boot
 * @param target_system  System which should execute the command
 * @param target_component  Component which should execute the command, 0 for all components
 * @param command  Command ID, as defined by MAV_CMD enum.
 * @param confirmation  0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1  Parameter 1, as defined by MAV_CMD enum.
 * @param param2  Parameter 2, as defined by MAV_CMD enum.
 * @param param3  Parameter 3, as defined by MAV_CMD enum.
 * @param param4  Parameter 4, as defined by MAV_CMD enum.
 * @param param5  Parameter 5, as defined by MAV_CMD enum.
 * @param param6  Parameter 6, as defined by MAV_CMD enum.
 * @param param7  Parameter 7, as defined by MAV_CMD enum.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_long_stamped_send(mavlink_channel_t chan, uint32_t utc_time, uint64_t vehicle_timestamp, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN];
    _mav_put_uint64_t(buf, 0, vehicle_timestamp);
    _mav_put_uint32_t(buf, 8, utc_time);
    _mav_put_float(buf, 12, param1);
    _mav_put_float(buf, 16, param2);
    _mav_put_float(buf, 20, param3);
    _mav_put_float(buf, 24, param4);
    _mav_put_float(buf, 28, param5);
    _mav_put_float(buf, 32, param6);
    _mav_put_float(buf, 36, param7);
    _mav_put_uint16_t(buf, 40, command);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, target_component);
    _mav_put_uint8_t(buf, 44, confirmation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED, buf, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_CRC);
#else
    mavlink_command_long_stamped_t packet;
    packet.vehicle_timestamp = vehicle_timestamp;
    packet.utc_time = utc_time;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;
    packet.param5 = param5;
    packet.param6 = param6;
    packet.param7 = param7;
    packet.command = command;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.confirmation = confirmation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED, (const char *)&packet, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_CRC);
#endif
}

/**
 * @brief Send a command_long_stamped message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_command_long_stamped_send_struct(mavlink_channel_t chan, const mavlink_command_long_stamped_t* command_long_stamped)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_command_long_stamped_send(chan, command_long_stamped->utc_time, command_long_stamped->vehicle_timestamp, command_long_stamped->target_system, command_long_stamped->target_component, command_long_stamped->command, command_long_stamped->confirmation, command_long_stamped->param1, command_long_stamped->param2, command_long_stamped->param3, command_long_stamped->param4, command_long_stamped->param5, command_long_stamped->param6, command_long_stamped->param7);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED, (const char *)command_long_stamped, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_CRC);
#endif
}

#if MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_command_long_stamped_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t utc_time, uint64_t vehicle_timestamp, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, vehicle_timestamp);
    _mav_put_uint32_t(buf, 8, utc_time);
    _mav_put_float(buf, 12, param1);
    _mav_put_float(buf, 16, param2);
    _mav_put_float(buf, 20, param3);
    _mav_put_float(buf, 24, param4);
    _mav_put_float(buf, 28, param5);
    _mav_put_float(buf, 32, param6);
    _mav_put_float(buf, 36, param7);
    _mav_put_uint16_t(buf, 40, command);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, target_component);
    _mav_put_uint8_t(buf, 44, confirmation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED, buf, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_CRC);
#else
    mavlink_command_long_stamped_t *packet = (mavlink_command_long_stamped_t *)msgbuf;
    packet->vehicle_timestamp = vehicle_timestamp;
    packet->utc_time = utc_time;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;
    packet->param5 = param5;
    packet->param6 = param6;
    packet->param7 = param7;
    packet->command = command;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->confirmation = confirmation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED, (const char *)packet, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_CRC);
#endif
}
#endif

#endif

// MESSAGE COMMAND_LONG_STAMPED UNPACKING


/**
 * @brief Get field utc_time from command_long_stamped message
 *
 * @return  UTC time, seconds elapsed since 01.01.1970
 */
static inline uint32_t mavlink_msg_command_long_stamped_get_utc_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field vehicle_timestamp from command_long_stamped message
 *
 * @return  Microseconds elapsed since vehicle boot
 */
static inline uint64_t mavlink_msg_command_long_stamped_get_vehicle_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_system from command_long_stamped message
 *
 * @return  System which should execute the command
 */
static inline uint8_t mavlink_msg_command_long_stamped_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field target_component from command_long_stamped message
 *
 * @return  Component which should execute the command, 0 for all components
 */
static inline uint8_t mavlink_msg_command_long_stamped_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field command from command_long_stamped message
 *
 * @return  Command ID, as defined by MAV_CMD enum.
 */
static inline uint16_t mavlink_msg_command_long_stamped_get_command(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field confirmation from command_long_stamped message
 *
 * @return  0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 */
static inline uint8_t mavlink_msg_command_long_stamped_get_confirmation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field param1 from command_long_stamped message
 *
 * @return  Parameter 1, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_stamped_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field param2 from command_long_stamped message
 *
 * @return  Parameter 2, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_stamped_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field param3 from command_long_stamped message
 *
 * @return  Parameter 3, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_stamped_get_param3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field param4 from command_long_stamped message
 *
 * @return  Parameter 4, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_stamped_get_param4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field param5 from command_long_stamped message
 *
 * @return  Parameter 5, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_stamped_get_param5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field param6 from command_long_stamped message
 *
 * @return  Parameter 6, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_stamped_get_param6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field param7 from command_long_stamped message
 *
 * @return  Parameter 7, as defined by MAV_CMD enum.
 */
static inline float mavlink_msg_command_long_stamped_get_param7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a command_long_stamped message into a struct
 *
 * @param msg The message to decode
 * @param command_long_stamped C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_long_stamped_decode(const mavlink_message_t* msg, mavlink_command_long_stamped_t* command_long_stamped)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    command_long_stamped->vehicle_timestamp = mavlink_msg_command_long_stamped_get_vehicle_timestamp(msg);
    command_long_stamped->utc_time = mavlink_msg_command_long_stamped_get_utc_time(msg);
    command_long_stamped->param1 = mavlink_msg_command_long_stamped_get_param1(msg);
    command_long_stamped->param2 = mavlink_msg_command_long_stamped_get_param2(msg);
    command_long_stamped->param3 = mavlink_msg_command_long_stamped_get_param3(msg);
    command_long_stamped->param4 = mavlink_msg_command_long_stamped_get_param4(msg);
    command_long_stamped->param5 = mavlink_msg_command_long_stamped_get_param5(msg);
    command_long_stamped->param6 = mavlink_msg_command_long_stamped_get_param6(msg);
    command_long_stamped->param7 = mavlink_msg_command_long_stamped_get_param7(msg);
    command_long_stamped->command = mavlink_msg_command_long_stamped_get_command(msg);
    command_long_stamped->target_system = mavlink_msg_command_long_stamped_get_target_system(msg);
    command_long_stamped->target_component = mavlink_msg_command_long_stamped_get_target_component(msg);
    command_long_stamped->confirmation = mavlink_msg_command_long_stamped_get_confirmation(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN? msg->len : MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN;
        memset(command_long_stamped, 0, MAVLINK_MSG_ID_COMMAND_LONG_STAMPED_LEN);
    memcpy(command_long_stamped, _MAV_PAYLOAD(msg), len);
#endif
}
