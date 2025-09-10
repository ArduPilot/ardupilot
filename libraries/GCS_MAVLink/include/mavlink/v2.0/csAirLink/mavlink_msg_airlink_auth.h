#pragma once
// MESSAGE AIRLINK_AUTH PACKING

#define MAVLINK_MSG_ID_AIRLINK_AUTH 52000


typedef struct __mavlink_airlink_auth_t {
 char login[50]; /*<  Login*/
 char password[50]; /*<  Password*/
} mavlink_airlink_auth_t;

#define MAVLINK_MSG_ID_AIRLINK_AUTH_LEN 100
#define MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN 100
#define MAVLINK_MSG_ID_52000_LEN 100
#define MAVLINK_MSG_ID_52000_MIN_LEN 100

#define MAVLINK_MSG_ID_AIRLINK_AUTH_CRC 13
#define MAVLINK_MSG_ID_52000_CRC 13

#define MAVLINK_MSG_AIRLINK_AUTH_FIELD_LOGIN_LEN 50
#define MAVLINK_MSG_AIRLINK_AUTH_FIELD_PASSWORD_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AIRLINK_AUTH { \
    52000, \
    "AIRLINK_AUTH", \
    2, \
    {  { "login", NULL, MAVLINK_TYPE_CHAR, 50, 0, offsetof(mavlink_airlink_auth_t, login) }, \
         { "password", NULL, MAVLINK_TYPE_CHAR, 50, 50, offsetof(mavlink_airlink_auth_t, password) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AIRLINK_AUTH { \
    "AIRLINK_AUTH", \
    2, \
    {  { "login", NULL, MAVLINK_TYPE_CHAR, 50, 0, offsetof(mavlink_airlink_auth_t, login) }, \
         { "password", NULL, MAVLINK_TYPE_CHAR, 50, 50, offsetof(mavlink_airlink_auth_t, password) }, \
         } \
}
#endif

/**
 * @brief Pack a airlink_auth message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param login  Login
 * @param password  Password
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_auth_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *login, const char *password)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_AUTH_LEN];

    _mav_put_char_array(buf, 0, login, 50);
    _mav_put_char_array(buf, 50, password, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN);
#else
    mavlink_airlink_auth_t packet;

    mav_array_memcpy(packet.login, login, sizeof(char)*50);
    mav_array_memcpy(packet.password, password, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_AUTH;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_CRC);
}

/**
 * @brief Pack a airlink_auth message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param login  Login
 * @param password  Password
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_auth_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *login, const char *password)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_AUTH_LEN];

    _mav_put_char_array(buf, 0, login, 50);
    _mav_put_char_array(buf, 50, password, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN);
#else
    mavlink_airlink_auth_t packet;

    mav_array_memcpy(packet.login, login, sizeof(char)*50);
    mav_array_memcpy(packet.password, password, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_AUTH;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN);
#endif
}

/**
 * @brief Pack a airlink_auth message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param login  Login
 * @param password  Password
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airlink_auth_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *login,const char *password)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_AUTH_LEN];

    _mav_put_char_array(buf, 0, login, 50);
    _mav_put_char_array(buf, 50, password, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN);
#else
    mavlink_airlink_auth_t packet;

    mav_array_memcpy(packet.login, login, sizeof(char)*50);
    mav_array_memcpy(packet.password, password, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRLINK_AUTH;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_CRC);
}

/**
 * @brief Encode a airlink_auth struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airlink_auth C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_auth_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airlink_auth_t* airlink_auth)
{
    return mavlink_msg_airlink_auth_pack(system_id, component_id, msg, airlink_auth->login, airlink_auth->password);
}

/**
 * @brief Encode a airlink_auth struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param airlink_auth C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_auth_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_airlink_auth_t* airlink_auth)
{
    return mavlink_msg_airlink_auth_pack_chan(system_id, component_id, chan, msg, airlink_auth->login, airlink_auth->password);
}

/**
 * @brief Encode a airlink_auth struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param airlink_auth C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airlink_auth_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_airlink_auth_t* airlink_auth)
{
    return mavlink_msg_airlink_auth_pack_status(system_id, component_id, _status, msg,  airlink_auth->login, airlink_auth->password);
}

/**
 * @brief Send a airlink_auth message
 * @param chan MAVLink channel to send the message
 *
 * @param login  Login
 * @param password  Password
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airlink_auth_send(mavlink_channel_t chan, const char *login, const char *password)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRLINK_AUTH_LEN];

    _mav_put_char_array(buf, 0, login, 50);
    _mav_put_char_array(buf, 50, password, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_AUTH, buf, MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_CRC);
#else
    mavlink_airlink_auth_t packet;

    mav_array_memcpy(packet.login, login, sizeof(char)*50);
    mav_array_memcpy(packet.password, password, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_AUTH, (const char *)&packet, MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_CRC);
#endif
}

/**
 * @brief Send a airlink_auth message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_airlink_auth_send_struct(mavlink_channel_t chan, const mavlink_airlink_auth_t* airlink_auth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_airlink_auth_send(chan, airlink_auth->login, airlink_auth->password);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_AUTH, (const char *)airlink_auth, MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_CRC);
#endif
}

#if MAVLINK_MSG_ID_AIRLINK_AUTH_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_airlink_auth_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *login, const char *password)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, login, 50);
    _mav_put_char_array(buf, 50, password, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_AUTH, buf, MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_CRC);
#else
    mavlink_airlink_auth_t *packet = (mavlink_airlink_auth_t *)msgbuf;

    mav_array_memcpy(packet->login, login, sizeof(char)*50);
    mav_array_memcpy(packet->password, password, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRLINK_AUTH, (const char *)packet, MAVLINK_MSG_ID_AIRLINK_AUTH_MIN_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN, MAVLINK_MSG_ID_AIRLINK_AUTH_CRC);
#endif
}
#endif

#endif

// MESSAGE AIRLINK_AUTH UNPACKING


/**
 * @brief Get field login from airlink_auth message
 *
 * @return  Login
 */
static inline uint16_t mavlink_msg_airlink_auth_get_login(const mavlink_message_t* msg, char *login)
{
    return _MAV_RETURN_char_array(msg, login, 50,  0);
}

/**
 * @brief Get field password from airlink_auth message
 *
 * @return  Password
 */
static inline uint16_t mavlink_msg_airlink_auth_get_password(const mavlink_message_t* msg, char *password)
{
    return _MAV_RETURN_char_array(msg, password, 50,  50);
}

/**
 * @brief Decode a airlink_auth message into a struct
 *
 * @param msg The message to decode
 * @param airlink_auth C-struct to decode the message contents into
 */
static inline void mavlink_msg_airlink_auth_decode(const mavlink_message_t* msg, mavlink_airlink_auth_t* airlink_auth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_airlink_auth_get_login(msg, airlink_auth->login);
    mavlink_msg_airlink_auth_get_password(msg, airlink_auth->password);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AIRLINK_AUTH_LEN? msg->len : MAVLINK_MSG_ID_AIRLINK_AUTH_LEN;
        memset(airlink_auth, 0, MAVLINK_MSG_ID_AIRLINK_AUTH_LEN);
    memcpy(airlink_auth, _MAV_PAYLOAD(msg), len);
#endif
}
