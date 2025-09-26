#pragma once
// MESSAGE AOA_SSA PACKING

#define MAVLINK_MSG_ID_AOA_SSA 11020


typedef struct __mavlink_aoa_ssa_t {
 uint64_t time_usec; /*< [us] Timestamp (since boot or Unix epoch).*/
 float AOA; /*< [deg] Angle of Attack.*/
 float SSA; /*< [deg] Side Slip Angle.*/
} mavlink_aoa_ssa_t;

#define MAVLINK_MSG_ID_AOA_SSA_LEN 16
#define MAVLINK_MSG_ID_AOA_SSA_MIN_LEN 16
#define MAVLINK_MSG_ID_11020_LEN 16
#define MAVLINK_MSG_ID_11020_MIN_LEN 16

#define MAVLINK_MSG_ID_AOA_SSA_CRC 205
#define MAVLINK_MSG_ID_11020_CRC 205



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AOA_SSA { \
    11020, \
    "AOA_SSA", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_aoa_ssa_t, time_usec) }, \
         { "AOA", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_aoa_ssa_t, AOA) }, \
         { "SSA", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_aoa_ssa_t, SSA) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AOA_SSA { \
    "AOA_SSA", \
    3, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_aoa_ssa_t, time_usec) }, \
         { "AOA", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_aoa_ssa_t, AOA) }, \
         { "SSA", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_aoa_ssa_t, SSA) }, \
         } \
}
#endif

/**
 * @brief Pack a aoa_ssa message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (since boot or Unix epoch).
 * @param AOA [deg] Angle of Attack.
 * @param SSA [deg] Side Slip Angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aoa_ssa_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float AOA, float SSA)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AOA_SSA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, AOA);
    _mav_put_float(buf, 12, SSA);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AOA_SSA_LEN);
#else
    mavlink_aoa_ssa_t packet;
    packet.time_usec = time_usec;
    packet.AOA = AOA;
    packet.SSA = SSA;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AOA_SSA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AOA_SSA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AOA_SSA_MIN_LEN, MAVLINK_MSG_ID_AOA_SSA_LEN, MAVLINK_MSG_ID_AOA_SSA_CRC);
}

/**
 * @brief Pack a aoa_ssa message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (since boot or Unix epoch).
 * @param AOA [deg] Angle of Attack.
 * @param SSA [deg] Side Slip Angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aoa_ssa_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, float AOA, float SSA)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AOA_SSA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, AOA);
    _mav_put_float(buf, 12, SSA);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AOA_SSA_LEN);
#else
    mavlink_aoa_ssa_t packet;
    packet.time_usec = time_usec;
    packet.AOA = AOA;
    packet.SSA = SSA;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AOA_SSA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AOA_SSA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AOA_SSA_MIN_LEN, MAVLINK_MSG_ID_AOA_SSA_LEN, MAVLINK_MSG_ID_AOA_SSA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AOA_SSA_MIN_LEN, MAVLINK_MSG_ID_AOA_SSA_LEN);
#endif
}

/**
 * @brief Pack a aoa_ssa message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (since boot or Unix epoch).
 * @param AOA [deg] Angle of Attack.
 * @param SSA [deg] Side Slip Angle.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aoa_ssa_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float AOA,float SSA)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AOA_SSA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, AOA);
    _mav_put_float(buf, 12, SSA);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AOA_SSA_LEN);
#else
    mavlink_aoa_ssa_t packet;
    packet.time_usec = time_usec;
    packet.AOA = AOA;
    packet.SSA = SSA;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AOA_SSA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AOA_SSA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AOA_SSA_MIN_LEN, MAVLINK_MSG_ID_AOA_SSA_LEN, MAVLINK_MSG_ID_AOA_SSA_CRC);
}

/**
 * @brief Encode a aoa_ssa struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param aoa_ssa C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aoa_ssa_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_aoa_ssa_t* aoa_ssa)
{
    return mavlink_msg_aoa_ssa_pack(system_id, component_id, msg, aoa_ssa->time_usec, aoa_ssa->AOA, aoa_ssa->SSA);
}

/**
 * @brief Encode a aoa_ssa struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param aoa_ssa C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aoa_ssa_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_aoa_ssa_t* aoa_ssa)
{
    return mavlink_msg_aoa_ssa_pack_chan(system_id, component_id, chan, msg, aoa_ssa->time_usec, aoa_ssa->AOA, aoa_ssa->SSA);
}

/**
 * @brief Encode a aoa_ssa struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param aoa_ssa C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aoa_ssa_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_aoa_ssa_t* aoa_ssa)
{
    return mavlink_msg_aoa_ssa_pack_status(system_id, component_id, _status, msg,  aoa_ssa->time_usec, aoa_ssa->AOA, aoa_ssa->SSA);
}

/**
 * @brief Send a aoa_ssa message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (since boot or Unix epoch).
 * @param AOA [deg] Angle of Attack.
 * @param SSA [deg] Side Slip Angle.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_aoa_ssa_send(mavlink_channel_t chan, uint64_t time_usec, float AOA, float SSA)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AOA_SSA_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, AOA);
    _mav_put_float(buf, 12, SSA);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AOA_SSA, buf, MAVLINK_MSG_ID_AOA_SSA_MIN_LEN, MAVLINK_MSG_ID_AOA_SSA_LEN, MAVLINK_MSG_ID_AOA_SSA_CRC);
#else
    mavlink_aoa_ssa_t packet;
    packet.time_usec = time_usec;
    packet.AOA = AOA;
    packet.SSA = SSA;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AOA_SSA, (const char *)&packet, MAVLINK_MSG_ID_AOA_SSA_MIN_LEN, MAVLINK_MSG_ID_AOA_SSA_LEN, MAVLINK_MSG_ID_AOA_SSA_CRC);
#endif
}

/**
 * @brief Send a aoa_ssa message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_aoa_ssa_send_struct(mavlink_channel_t chan, const mavlink_aoa_ssa_t* aoa_ssa)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_aoa_ssa_send(chan, aoa_ssa->time_usec, aoa_ssa->AOA, aoa_ssa->SSA);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AOA_SSA, (const char *)aoa_ssa, MAVLINK_MSG_ID_AOA_SSA_MIN_LEN, MAVLINK_MSG_ID_AOA_SSA_LEN, MAVLINK_MSG_ID_AOA_SSA_CRC);
#endif
}

#if MAVLINK_MSG_ID_AOA_SSA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_aoa_ssa_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float AOA, float SSA)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, AOA);
    _mav_put_float(buf, 12, SSA);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AOA_SSA, buf, MAVLINK_MSG_ID_AOA_SSA_MIN_LEN, MAVLINK_MSG_ID_AOA_SSA_LEN, MAVLINK_MSG_ID_AOA_SSA_CRC);
#else
    mavlink_aoa_ssa_t *packet = (mavlink_aoa_ssa_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->AOA = AOA;
    packet->SSA = SSA;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AOA_SSA, (const char *)packet, MAVLINK_MSG_ID_AOA_SSA_MIN_LEN, MAVLINK_MSG_ID_AOA_SSA_LEN, MAVLINK_MSG_ID_AOA_SSA_CRC);
#endif
}
#endif

#endif

// MESSAGE AOA_SSA UNPACKING


/**
 * @brief Get field time_usec from aoa_ssa message
 *
 * @return [us] Timestamp (since boot or Unix epoch).
 */
static inline uint64_t mavlink_msg_aoa_ssa_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field AOA from aoa_ssa message
 *
 * @return [deg] Angle of Attack.
 */
static inline float mavlink_msg_aoa_ssa_get_AOA(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field SSA from aoa_ssa message
 *
 * @return [deg] Side Slip Angle.
 */
static inline float mavlink_msg_aoa_ssa_get_SSA(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a aoa_ssa message into a struct
 *
 * @param msg The message to decode
 * @param aoa_ssa C-struct to decode the message contents into
 */
static inline void mavlink_msg_aoa_ssa_decode(const mavlink_message_t* msg, mavlink_aoa_ssa_t* aoa_ssa)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    aoa_ssa->time_usec = mavlink_msg_aoa_ssa_get_time_usec(msg);
    aoa_ssa->AOA = mavlink_msg_aoa_ssa_get_AOA(msg);
    aoa_ssa->SSA = mavlink_msg_aoa_ssa_get_SSA(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AOA_SSA_LEN? msg->len : MAVLINK_MSG_ID_AOA_SSA_LEN;
        memset(aoa_ssa, 0, MAVLINK_MSG_ID_AOA_SSA_LEN);
    memcpy(aoa_ssa, _MAV_PAYLOAD(msg), len);
#endif
}
