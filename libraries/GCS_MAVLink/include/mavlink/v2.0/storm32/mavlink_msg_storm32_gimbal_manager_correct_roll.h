#pragma once
// MESSAGE STORM32_GIMBAL_MANAGER_CORRECT_ROLL PACKING

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL 60014


typedef struct __mavlink_storm32_gimbal_manager_correct_roll_t {
 float roll; /*< [rad] Roll angle (positive to roll to the right).*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t gimbal_id; /*<  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).*/
 uint8_t client; /*<  Client which is contacting the gimbal manager (must be set).*/
} mavlink_storm32_gimbal_manager_correct_roll_t;

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN 8
#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN 8
#define MAVLINK_MSG_ID_60014_LEN 8
#define MAVLINK_MSG_ID_60014_MIN_LEN 8

#define MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_CRC 134
#define MAVLINK_MSG_ID_60014_CRC 134



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_CORRECT_ROLL { \
    60014, \
    "STORM32_GIMBAL_MANAGER_CORRECT_ROLL", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_storm32_gimbal_manager_correct_roll_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_storm32_gimbal_manager_correct_roll_t, target_component) }, \
         { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_storm32_gimbal_manager_correct_roll_t, gimbal_id) }, \
         { "client", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_storm32_gimbal_manager_correct_roll_t, client) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_storm32_gimbal_manager_correct_roll_t, roll) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STORM32_GIMBAL_MANAGER_CORRECT_ROLL { \
    "STORM32_GIMBAL_MANAGER_CORRECT_ROLL", \
    5, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_storm32_gimbal_manager_correct_roll_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_storm32_gimbal_manager_correct_roll_t, target_component) }, \
         { "gimbal_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_storm32_gimbal_manager_correct_roll_t, gimbal_id) }, \
         { "client", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_storm32_gimbal_manager_correct_roll_t, client) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_storm32_gimbal_manager_correct_roll_t, roll) }, \
         } \
}
#endif

/**
 * @brief Pack a storm32_gimbal_manager_correct_roll message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param client  Client which is contacting the gimbal manager (must be set).
 * @param roll [rad] Roll angle (positive to roll to the right).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_correct_roll_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, gimbal_id);
    _mav_put_uint8_t(buf, 7, client);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN);
#else
    mavlink_storm32_gimbal_manager_correct_roll_t packet;
    packet.roll = roll;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_CRC);
}

/**
 * @brief Pack a storm32_gimbal_manager_correct_roll message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param client  Client which is contacting the gimbal manager (must be set).
 * @param roll [rad] Roll angle (positive to roll to the right).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_correct_roll_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, gimbal_id);
    _mav_put_uint8_t(buf, 7, client);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN);
#else
    mavlink_storm32_gimbal_manager_correct_roll_t packet;
    packet.roll = roll;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN);
#endif
}

/**
 * @brief Pack a storm32_gimbal_manager_correct_roll message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param client  Client which is contacting the gimbal manager (must be set).
 * @param roll [rad] Roll angle (positive to roll to the right).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_correct_roll_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t gimbal_id,uint8_t client,float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, gimbal_id);
    _mav_put_uint8_t(buf, 7, client);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN);
#else
    mavlink_storm32_gimbal_manager_correct_roll_t packet;
    packet.roll = roll;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_CRC);
}

/**
 * @brief Encode a storm32_gimbal_manager_correct_roll struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_correct_roll C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_correct_roll_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_correct_roll_t* storm32_gimbal_manager_correct_roll)
{
    return mavlink_msg_storm32_gimbal_manager_correct_roll_pack(system_id, component_id, msg, storm32_gimbal_manager_correct_roll->target_system, storm32_gimbal_manager_correct_roll->target_component, storm32_gimbal_manager_correct_roll->gimbal_id, storm32_gimbal_manager_correct_roll->client, storm32_gimbal_manager_correct_roll->roll);
}

/**
 * @brief Encode a storm32_gimbal_manager_correct_roll struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_correct_roll C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_correct_roll_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_correct_roll_t* storm32_gimbal_manager_correct_roll)
{
    return mavlink_msg_storm32_gimbal_manager_correct_roll_pack_chan(system_id, component_id, chan, msg, storm32_gimbal_manager_correct_roll->target_system, storm32_gimbal_manager_correct_roll->target_component, storm32_gimbal_manager_correct_roll->gimbal_id, storm32_gimbal_manager_correct_roll->client, storm32_gimbal_manager_correct_roll->roll);
}

/**
 * @brief Encode a storm32_gimbal_manager_correct_roll struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param storm32_gimbal_manager_correct_roll C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storm32_gimbal_manager_correct_roll_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_storm32_gimbal_manager_correct_roll_t* storm32_gimbal_manager_correct_roll)
{
    return mavlink_msg_storm32_gimbal_manager_correct_roll_pack_status(system_id, component_id, _status, msg,  storm32_gimbal_manager_correct_roll->target_system, storm32_gimbal_manager_correct_roll->target_component, storm32_gimbal_manager_correct_roll->gimbal_id, storm32_gimbal_manager_correct_roll->client, storm32_gimbal_manager_correct_roll->roll);
}

/**
 * @brief Send a storm32_gimbal_manager_correct_roll message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param gimbal_id  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 * @param client  Client which is contacting the gimbal manager (must be set).
 * @param roll [rad] Roll angle (positive to roll to the right).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_storm32_gimbal_manager_correct_roll_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, gimbal_id);
    _mav_put_uint8_t(buf, 7, client);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_CRC);
#else
    mavlink_storm32_gimbal_manager_correct_roll_t packet;
    packet.roll = roll;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.gimbal_id = gimbal_id;
    packet.client = client;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL, (const char *)&packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_CRC);
#endif
}

/**
 * @brief Send a storm32_gimbal_manager_correct_roll message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_storm32_gimbal_manager_correct_roll_send_struct(mavlink_channel_t chan, const mavlink_storm32_gimbal_manager_correct_roll_t* storm32_gimbal_manager_correct_roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_storm32_gimbal_manager_correct_roll_send(chan, storm32_gimbal_manager_correct_roll->target_system, storm32_gimbal_manager_correct_roll->target_component, storm32_gimbal_manager_correct_roll->gimbal_id, storm32_gimbal_manager_correct_roll->client, storm32_gimbal_manager_correct_roll->roll);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL, (const char *)storm32_gimbal_manager_correct_roll, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_CRC);
#endif
}

#if MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_storm32_gimbal_manager_correct_roll_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t gimbal_id, uint8_t client, float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, gimbal_id);
    _mav_put_uint8_t(buf, 7, client);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL, buf, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_CRC);
#else
    mavlink_storm32_gimbal_manager_correct_roll_t *packet = (mavlink_storm32_gimbal_manager_correct_roll_t *)msgbuf;
    packet->roll = roll;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->gimbal_id = gimbal_id;
    packet->client = client;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL, (const char *)packet, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_MIN_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_CRC);
#endif
}
#endif

#endif

// MESSAGE STORM32_GIMBAL_MANAGER_CORRECT_ROLL UNPACKING


/**
 * @brief Get field target_system from storm32_gimbal_manager_correct_roll message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_correct_roll_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from storm32_gimbal_manager_correct_roll message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_correct_roll_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field gimbal_id from storm32_gimbal_manager_correct_roll message
 *
 * @return  Gimbal ID of the gimbal manager to address (component ID or 1-6 for non-MAVLink gimbal, 0 for all gimbals, send command multiple times for more than one but not all gimbals).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_correct_roll_get_gimbal_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field client from storm32_gimbal_manager_correct_roll message
 *
 * @return  Client which is contacting the gimbal manager (must be set).
 */
static inline uint8_t mavlink_msg_storm32_gimbal_manager_correct_roll_get_client(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field roll from storm32_gimbal_manager_correct_roll message
 *
 * @return [rad] Roll angle (positive to roll to the right).
 */
static inline float mavlink_msg_storm32_gimbal_manager_correct_roll_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a storm32_gimbal_manager_correct_roll message into a struct
 *
 * @param msg The message to decode
 * @param storm32_gimbal_manager_correct_roll C-struct to decode the message contents into
 */
static inline void mavlink_msg_storm32_gimbal_manager_correct_roll_decode(const mavlink_message_t* msg, mavlink_storm32_gimbal_manager_correct_roll_t* storm32_gimbal_manager_correct_roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    storm32_gimbal_manager_correct_roll->roll = mavlink_msg_storm32_gimbal_manager_correct_roll_get_roll(msg);
    storm32_gimbal_manager_correct_roll->target_system = mavlink_msg_storm32_gimbal_manager_correct_roll_get_target_system(msg);
    storm32_gimbal_manager_correct_roll->target_component = mavlink_msg_storm32_gimbal_manager_correct_roll_get_target_component(msg);
    storm32_gimbal_manager_correct_roll->gimbal_id = mavlink_msg_storm32_gimbal_manager_correct_roll_get_gimbal_id(msg);
    storm32_gimbal_manager_correct_roll->client = mavlink_msg_storm32_gimbal_manager_correct_roll_get_client(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN? msg->len : MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN;
        memset(storm32_gimbal_manager_correct_roll, 0, MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_CORRECT_ROLL_LEN);
    memcpy(storm32_gimbal_manager_correct_roll, _MAV_PAYLOAD(msg), len);
#endif
}
