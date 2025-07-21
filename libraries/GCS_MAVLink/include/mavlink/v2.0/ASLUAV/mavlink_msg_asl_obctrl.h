#pragma once
// MESSAGE ASL_OBCTRL PACKING

#define MAVLINK_MSG_ID_ASL_OBCTRL 8008


typedef struct __mavlink_asl_obctrl_t {
 uint64_t timestamp; /*< [us]  Time since system start*/
 float uElev; /*<   Elevator command [~]*/
 float uThrot; /*<   Throttle command [~]*/
 float uThrot2; /*<   Throttle 2 command [~]*/
 float uAilL; /*<   Left aileron command [~]*/
 float uAilR; /*<   Right aileron command [~]*/
 float uRud; /*<   Rudder command [~]*/
 uint8_t obctrl_status; /*<   Off-board computer status*/
} mavlink_asl_obctrl_t;

#define MAVLINK_MSG_ID_ASL_OBCTRL_LEN 33
#define MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN 33
#define MAVLINK_MSG_ID_8008_LEN 33
#define MAVLINK_MSG_ID_8008_MIN_LEN 33

#define MAVLINK_MSG_ID_ASL_OBCTRL_CRC 234
#define MAVLINK_MSG_ID_8008_CRC 234



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ASL_OBCTRL { \
    8008, \
    "ASL_OBCTRL", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_asl_obctrl_t, timestamp) }, \
         { "uElev", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_asl_obctrl_t, uElev) }, \
         { "uThrot", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_asl_obctrl_t, uThrot) }, \
         { "uThrot2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_asl_obctrl_t, uThrot2) }, \
         { "uAilL", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_asl_obctrl_t, uAilL) }, \
         { "uAilR", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_asl_obctrl_t, uAilR) }, \
         { "uRud", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_asl_obctrl_t, uRud) }, \
         { "obctrl_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_asl_obctrl_t, obctrl_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ASL_OBCTRL { \
    "ASL_OBCTRL", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_asl_obctrl_t, timestamp) }, \
         { "uElev", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_asl_obctrl_t, uElev) }, \
         { "uThrot", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_asl_obctrl_t, uThrot) }, \
         { "uThrot2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_asl_obctrl_t, uThrot2) }, \
         { "uAilL", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_asl_obctrl_t, uAilL) }, \
         { "uAilR", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_asl_obctrl_t, uAilR) }, \
         { "uRud", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_asl_obctrl_t, uRud) }, \
         { "obctrl_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_asl_obctrl_t, obctrl_status) }, \
         } \
}
#endif

/**
 * @brief Pack a asl_obctrl message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us]  Time since system start
 * @param uElev   Elevator command [~]
 * @param uThrot   Throttle command [~]
 * @param uThrot2   Throttle 2 command [~]
 * @param uAilL   Left aileron command [~]
 * @param uAilR   Right aileron command [~]
 * @param uRud   Rudder command [~]
 * @param obctrl_status   Off-board computer status
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_asl_obctrl_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float uElev, float uThrot, float uThrot2, float uAilL, float uAilR, float uRud, uint8_t obctrl_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASL_OBCTRL_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, uElev);
    _mav_put_float(buf, 12, uThrot);
    _mav_put_float(buf, 16, uThrot2);
    _mav_put_float(buf, 20, uAilL);
    _mav_put_float(buf, 24, uAilR);
    _mav_put_float(buf, 28, uRud);
    _mav_put_uint8_t(buf, 32, obctrl_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASL_OBCTRL_LEN);
#else
    mavlink_asl_obctrl_t packet;
    packet.timestamp = timestamp;
    packet.uElev = uElev;
    packet.uThrot = uThrot;
    packet.uThrot2 = uThrot2;
    packet.uAilL = uAilL;
    packet.uAilR = uAilR;
    packet.uRud = uRud;
    packet.obctrl_status = obctrl_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASL_OBCTRL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASL_OBCTRL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_CRC);
}

/**
 * @brief Pack a asl_obctrl message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us]  Time since system start
 * @param uElev   Elevator command [~]
 * @param uThrot   Throttle command [~]
 * @param uThrot2   Throttle 2 command [~]
 * @param uAilL   Left aileron command [~]
 * @param uAilR   Right aileron command [~]
 * @param uRud   Rudder command [~]
 * @param obctrl_status   Off-board computer status
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_asl_obctrl_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, float uElev, float uThrot, float uThrot2, float uAilL, float uAilR, float uRud, uint8_t obctrl_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASL_OBCTRL_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, uElev);
    _mav_put_float(buf, 12, uThrot);
    _mav_put_float(buf, 16, uThrot2);
    _mav_put_float(buf, 20, uAilL);
    _mav_put_float(buf, 24, uAilR);
    _mav_put_float(buf, 28, uRud);
    _mav_put_uint8_t(buf, 32, obctrl_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASL_OBCTRL_LEN);
#else
    mavlink_asl_obctrl_t packet;
    packet.timestamp = timestamp;
    packet.uElev = uElev;
    packet.uThrot = uThrot;
    packet.uThrot2 = uThrot2;
    packet.uAilL = uAilL;
    packet.uAilR = uAilR;
    packet.uRud = uRud;
    packet.obctrl_status = obctrl_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASL_OBCTRL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASL_OBCTRL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_LEN);
#endif
}

/**
 * @brief Pack a asl_obctrl message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us]  Time since system start
 * @param uElev   Elevator command [~]
 * @param uThrot   Throttle command [~]
 * @param uThrot2   Throttle 2 command [~]
 * @param uAilL   Left aileron command [~]
 * @param uAilR   Right aileron command [~]
 * @param uRud   Rudder command [~]
 * @param obctrl_status   Off-board computer status
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_asl_obctrl_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float uElev,float uThrot,float uThrot2,float uAilL,float uAilR,float uRud,uint8_t obctrl_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASL_OBCTRL_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, uElev);
    _mav_put_float(buf, 12, uThrot);
    _mav_put_float(buf, 16, uThrot2);
    _mav_put_float(buf, 20, uAilL);
    _mav_put_float(buf, 24, uAilR);
    _mav_put_float(buf, 28, uRud);
    _mav_put_uint8_t(buf, 32, obctrl_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASL_OBCTRL_LEN);
#else
    mavlink_asl_obctrl_t packet;
    packet.timestamp = timestamp;
    packet.uElev = uElev;
    packet.uThrot = uThrot;
    packet.uThrot2 = uThrot2;
    packet.uAilL = uAilL;
    packet.uAilR = uAilR;
    packet.uRud = uRud;
    packet.obctrl_status = obctrl_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASL_OBCTRL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASL_OBCTRL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_CRC);
}

/**
 * @brief Encode a asl_obctrl struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param asl_obctrl C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_asl_obctrl_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_asl_obctrl_t* asl_obctrl)
{
    return mavlink_msg_asl_obctrl_pack(system_id, component_id, msg, asl_obctrl->timestamp, asl_obctrl->uElev, asl_obctrl->uThrot, asl_obctrl->uThrot2, asl_obctrl->uAilL, asl_obctrl->uAilR, asl_obctrl->uRud, asl_obctrl->obctrl_status);
}

/**
 * @brief Encode a asl_obctrl struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param asl_obctrl C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_asl_obctrl_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_asl_obctrl_t* asl_obctrl)
{
    return mavlink_msg_asl_obctrl_pack_chan(system_id, component_id, chan, msg, asl_obctrl->timestamp, asl_obctrl->uElev, asl_obctrl->uThrot, asl_obctrl->uThrot2, asl_obctrl->uAilL, asl_obctrl->uAilR, asl_obctrl->uRud, asl_obctrl->obctrl_status);
}

/**
 * @brief Encode a asl_obctrl struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param asl_obctrl C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_asl_obctrl_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_asl_obctrl_t* asl_obctrl)
{
    return mavlink_msg_asl_obctrl_pack_status(system_id, component_id, _status, msg,  asl_obctrl->timestamp, asl_obctrl->uElev, asl_obctrl->uThrot, asl_obctrl->uThrot2, asl_obctrl->uAilL, asl_obctrl->uAilR, asl_obctrl->uRud, asl_obctrl->obctrl_status);
}

/**
 * @brief Send a asl_obctrl message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us]  Time since system start
 * @param uElev   Elevator command [~]
 * @param uThrot   Throttle command [~]
 * @param uThrot2   Throttle 2 command [~]
 * @param uAilL   Left aileron command [~]
 * @param uAilR   Right aileron command [~]
 * @param uRud   Rudder command [~]
 * @param obctrl_status   Off-board computer status
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_asl_obctrl_send(mavlink_channel_t chan, uint64_t timestamp, float uElev, float uThrot, float uThrot2, float uAilL, float uAilR, float uRud, uint8_t obctrl_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASL_OBCTRL_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, uElev);
    _mav_put_float(buf, 12, uThrot);
    _mav_put_float(buf, 16, uThrot2);
    _mav_put_float(buf, 20, uAilL);
    _mav_put_float(buf, 24, uAilR);
    _mav_put_float(buf, 28, uRud);
    _mav_put_uint8_t(buf, 32, obctrl_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASL_OBCTRL, buf, MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_CRC);
#else
    mavlink_asl_obctrl_t packet;
    packet.timestamp = timestamp;
    packet.uElev = uElev;
    packet.uThrot = uThrot;
    packet.uThrot2 = uThrot2;
    packet.uAilL = uAilL;
    packet.uAilR = uAilR;
    packet.uRud = uRud;
    packet.obctrl_status = obctrl_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASL_OBCTRL, (const char *)&packet, MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_CRC);
#endif
}

/**
 * @brief Send a asl_obctrl message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_asl_obctrl_send_struct(mavlink_channel_t chan, const mavlink_asl_obctrl_t* asl_obctrl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_asl_obctrl_send(chan, asl_obctrl->timestamp, asl_obctrl->uElev, asl_obctrl->uThrot, asl_obctrl->uThrot2, asl_obctrl->uAilL, asl_obctrl->uAilR, asl_obctrl->uRud, asl_obctrl->obctrl_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASL_OBCTRL, (const char *)asl_obctrl, MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_CRC);
#endif
}

#if MAVLINK_MSG_ID_ASL_OBCTRL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_asl_obctrl_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float uElev, float uThrot, float uThrot2, float uAilL, float uAilR, float uRud, uint8_t obctrl_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, uElev);
    _mav_put_float(buf, 12, uThrot);
    _mav_put_float(buf, 16, uThrot2);
    _mav_put_float(buf, 20, uAilL);
    _mav_put_float(buf, 24, uAilR);
    _mav_put_float(buf, 28, uRud);
    _mav_put_uint8_t(buf, 32, obctrl_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASL_OBCTRL, buf, MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_CRC);
#else
    mavlink_asl_obctrl_t *packet = (mavlink_asl_obctrl_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->uElev = uElev;
    packet->uThrot = uThrot;
    packet->uThrot2 = uThrot2;
    packet->uAilL = uAilL;
    packet->uAilR = uAilR;
    packet->uRud = uRud;
    packet->obctrl_status = obctrl_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASL_OBCTRL, (const char *)packet, MAVLINK_MSG_ID_ASL_OBCTRL_MIN_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_LEN, MAVLINK_MSG_ID_ASL_OBCTRL_CRC);
#endif
}
#endif

#endif

// MESSAGE ASL_OBCTRL UNPACKING


/**
 * @brief Get field timestamp from asl_obctrl message
 *
 * @return [us]  Time since system start
 */
static inline uint64_t mavlink_msg_asl_obctrl_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field uElev from asl_obctrl message
 *
 * @return   Elevator command [~]
 */
static inline float mavlink_msg_asl_obctrl_get_uElev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field uThrot from asl_obctrl message
 *
 * @return   Throttle command [~]
 */
static inline float mavlink_msg_asl_obctrl_get_uThrot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field uThrot2 from asl_obctrl message
 *
 * @return   Throttle 2 command [~]
 */
static inline float mavlink_msg_asl_obctrl_get_uThrot2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field uAilL from asl_obctrl message
 *
 * @return   Left aileron command [~]
 */
static inline float mavlink_msg_asl_obctrl_get_uAilL(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field uAilR from asl_obctrl message
 *
 * @return   Right aileron command [~]
 */
static inline float mavlink_msg_asl_obctrl_get_uAilR(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field uRud from asl_obctrl message
 *
 * @return   Rudder command [~]
 */
static inline float mavlink_msg_asl_obctrl_get_uRud(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field obctrl_status from asl_obctrl message
 *
 * @return   Off-board computer status
 */
static inline uint8_t mavlink_msg_asl_obctrl_get_obctrl_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Decode a asl_obctrl message into a struct
 *
 * @param msg The message to decode
 * @param asl_obctrl C-struct to decode the message contents into
 */
static inline void mavlink_msg_asl_obctrl_decode(const mavlink_message_t* msg, mavlink_asl_obctrl_t* asl_obctrl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    asl_obctrl->timestamp = mavlink_msg_asl_obctrl_get_timestamp(msg);
    asl_obctrl->uElev = mavlink_msg_asl_obctrl_get_uElev(msg);
    asl_obctrl->uThrot = mavlink_msg_asl_obctrl_get_uThrot(msg);
    asl_obctrl->uThrot2 = mavlink_msg_asl_obctrl_get_uThrot2(msg);
    asl_obctrl->uAilL = mavlink_msg_asl_obctrl_get_uAilL(msg);
    asl_obctrl->uAilR = mavlink_msg_asl_obctrl_get_uAilR(msg);
    asl_obctrl->uRud = mavlink_msg_asl_obctrl_get_uRud(msg);
    asl_obctrl->obctrl_status = mavlink_msg_asl_obctrl_get_obctrl_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ASL_OBCTRL_LEN? msg->len : MAVLINK_MSG_ID_ASL_OBCTRL_LEN;
        memset(asl_obctrl, 0, MAVLINK_MSG_ID_ASL_OBCTRL_LEN);
    memcpy(asl_obctrl, _MAV_PAYLOAD(msg), len);
#endif
}
