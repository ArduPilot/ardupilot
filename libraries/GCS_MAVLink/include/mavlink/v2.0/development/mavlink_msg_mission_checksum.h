#pragma once
// MESSAGE MISSION_CHECKSUM PACKING

#define MAVLINK_MSG_ID_MISSION_CHECKSUM 53


typedef struct __mavlink_mission_checksum_t {
 uint32_t checksum; /*<  CRC32 checksum of current plan for specified type.*/
 uint8_t mission_type; /*<  Mission type.*/
} mavlink_mission_checksum_t;

#define MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN 5
#define MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN 5
#define MAVLINK_MSG_ID_53_LEN 5
#define MAVLINK_MSG_ID_53_MIN_LEN 5

#define MAVLINK_MSG_ID_MISSION_CHECKSUM_CRC 3
#define MAVLINK_MSG_ID_53_CRC 3



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_CHECKSUM { \
    53, \
    "MISSION_CHECKSUM", \
    2, \
    {  { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mission_checksum_t, mission_type) }, \
         { "checksum", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mission_checksum_t, checksum) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_CHECKSUM { \
    "MISSION_CHECKSUM", \
    2, \
    {  { "mission_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mission_checksum_t, mission_type) }, \
         { "checksum", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mission_checksum_t, checksum) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_checksum message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mission_type  Mission type.
 * @param checksum  CRC32 checksum of current plan for specified type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_checksum_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t mission_type, uint32_t checksum)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN];
    _mav_put_uint32_t(buf, 0, checksum);
    _mav_put_uint8_t(buf, 4, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN);
#else
    mavlink_mission_checksum_t packet;
    packet.checksum = checksum;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CHECKSUM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_CRC);
}

/**
 * @brief Pack a mission_checksum message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param mission_type  Mission type.
 * @param checksum  CRC32 checksum of current plan for specified type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_checksum_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t mission_type, uint32_t checksum)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN];
    _mav_put_uint32_t(buf, 0, checksum);
    _mav_put_uint8_t(buf, 4, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN);
#else
    mavlink_mission_checksum_t packet;
    packet.checksum = checksum;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CHECKSUM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN);
#endif
}

/**
 * @brief Pack a mission_checksum message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_type  Mission type.
 * @param checksum  CRC32 checksum of current plan for specified type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_checksum_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t mission_type,uint32_t checksum)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN];
    _mav_put_uint32_t(buf, 0, checksum);
    _mav_put_uint8_t(buf, 4, mission_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN);
#else
    mavlink_mission_checksum_t packet;
    packet.checksum = checksum;
    packet.mission_type = mission_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CHECKSUM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_CRC);
}

/**
 * @brief Encode a mission_checksum struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_checksum C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_checksum_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_checksum_t* mission_checksum)
{
    return mavlink_msg_mission_checksum_pack(system_id, component_id, msg, mission_checksum->mission_type, mission_checksum->checksum);
}

/**
 * @brief Encode a mission_checksum struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_checksum C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_checksum_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_checksum_t* mission_checksum)
{
    return mavlink_msg_mission_checksum_pack_chan(system_id, component_id, chan, msg, mission_checksum->mission_type, mission_checksum->checksum);
}

/**
 * @brief Encode a mission_checksum struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mission_checksum C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_checksum_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mission_checksum_t* mission_checksum)
{
    return mavlink_msg_mission_checksum_pack_status(system_id, component_id, _status, msg,  mission_checksum->mission_type, mission_checksum->checksum);
}

/**
 * @brief Send a mission_checksum message
 * @param chan MAVLink channel to send the message
 *
 * @param mission_type  Mission type.
 * @param checksum  CRC32 checksum of current plan for specified type.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_checksum_send(mavlink_channel_t chan, uint8_t mission_type, uint32_t checksum)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN];
    _mav_put_uint32_t(buf, 0, checksum);
    _mav_put_uint8_t(buf, 4, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CHECKSUM, buf, MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_CRC);
#else
    mavlink_mission_checksum_t packet;
    packet.checksum = checksum;
    packet.mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CHECKSUM, (const char *)&packet, MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_CRC);
#endif
}

/**
 * @brief Send a mission_checksum message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_checksum_send_struct(mavlink_channel_t chan, const mavlink_mission_checksum_t* mission_checksum)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_checksum_send(chan, mission_checksum->mission_type, mission_checksum->checksum);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CHECKSUM, (const char *)mission_checksum, MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_checksum_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mission_type, uint32_t checksum)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, checksum);
    _mav_put_uint8_t(buf, 4, mission_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CHECKSUM, buf, MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_CRC);
#else
    mavlink_mission_checksum_t *packet = (mavlink_mission_checksum_t *)msgbuf;
    packet->checksum = checksum;
    packet->mission_type = mission_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CHECKSUM, (const char *)packet, MAVLINK_MSG_ID_MISSION_CHECKSUM_MIN_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN, MAVLINK_MSG_ID_MISSION_CHECKSUM_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_CHECKSUM UNPACKING


/**
 * @brief Get field mission_type from mission_checksum message
 *
 * @return  Mission type.
 */
static inline uint8_t mavlink_msg_mission_checksum_get_mission_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field checksum from mission_checksum message
 *
 * @return  CRC32 checksum of current plan for specified type.
 */
static inline uint32_t mavlink_msg_mission_checksum_get_checksum(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a mission_checksum message into a struct
 *
 * @param msg The message to decode
 * @param mission_checksum C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_checksum_decode(const mavlink_message_t* msg, mavlink_mission_checksum_t* mission_checksum)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_checksum->checksum = mavlink_msg_mission_checksum_get_checksum(msg);
    mission_checksum->mission_type = mavlink_msg_mission_checksum_get_mission_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN? msg->len : MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN;
        memset(mission_checksum, 0, MAVLINK_MSG_ID_MISSION_CHECKSUM_LEN);
    memcpy(mission_checksum, _MAV_PAYLOAD(msg), len);
#endif
}
