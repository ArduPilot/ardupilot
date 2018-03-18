#pragma once
// MESSAGE FLIGHT_INFORMATION PACKING

#define MAVLINK_MSG_ID_FLIGHT_INFORMATION 264

MAVPACKED(
typedef struct __mavlink_flight_information_t {
 uint64_t arming_time_utc; /*< Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown*/
 uint64_t takeoff_time_utc; /*< Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown*/
 uint64_t flight_uuid; /*< Universally unique identifier (UUID) of flight, should correspond to name of logfiles*/
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
}) mavlink_flight_information_t;

#define MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN 28
#define MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN 28
#define MAVLINK_MSG_ID_264_LEN 28
#define MAVLINK_MSG_ID_264_MIN_LEN 28

#define MAVLINK_MSG_ID_FLIGHT_INFORMATION_CRC 49
#define MAVLINK_MSG_ID_264_CRC 49



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FLIGHT_INFORMATION { \
    264, \
    "FLIGHT_INFORMATION", \
    4, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_flight_information_t, time_boot_ms) }, \
         { "arming_time_utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_flight_information_t, arming_time_utc) }, \
         { "takeoff_time_utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_flight_information_t, takeoff_time_utc) }, \
         { "flight_uuid", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_flight_information_t, flight_uuid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FLIGHT_INFORMATION { \
    "FLIGHT_INFORMATION", \
    4, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_flight_information_t, time_boot_ms) }, \
         { "arming_time_utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_flight_information_t, arming_time_utc) }, \
         { "takeoff_time_utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_flight_information_t, takeoff_time_utc) }, \
         { "flight_uuid", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_flight_information_t, flight_uuid) }, \
         } \
}
#endif

/**
 * @brief Pack a flight_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param arming_time_utc Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
 * @param takeoff_time_utc Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
 * @param flight_uuid Universally unique identifier (UUID) of flight, should correspond to name of logfiles
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flight_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN];
    _mav_put_uint64_t(buf, 0, arming_time_utc);
    _mav_put_uint64_t(buf, 8, takeoff_time_utc);
    _mav_put_uint64_t(buf, 16, flight_uuid);
    _mav_put_uint32_t(buf, 24, time_boot_ms);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN);
#else
    mavlink_flight_information_t packet;
    packet.arming_time_utc = arming_time_utc;
    packet.takeoff_time_utc = takeoff_time_utc;
    packet.flight_uuid = flight_uuid;
    packet.time_boot_ms = time_boot_ms;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLIGHT_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_CRC);
}

/**
 * @brief Pack a flight_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param arming_time_utc Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
 * @param takeoff_time_utc Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
 * @param flight_uuid Universally unique identifier (UUID) of flight, should correspond to name of logfiles
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flight_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint64_t arming_time_utc,uint64_t takeoff_time_utc,uint64_t flight_uuid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN];
    _mav_put_uint64_t(buf, 0, arming_time_utc);
    _mav_put_uint64_t(buf, 8, takeoff_time_utc);
    _mav_put_uint64_t(buf, 16, flight_uuid);
    _mav_put_uint32_t(buf, 24, time_boot_ms);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN);
#else
    mavlink_flight_information_t packet;
    packet.arming_time_utc = arming_time_utc;
    packet.takeoff_time_utc = takeoff_time_utc;
    packet.flight_uuid = flight_uuid;
    packet.time_boot_ms = time_boot_ms;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FLIGHT_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_CRC);
}

/**
 * @brief Encode a flight_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flight_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flight_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flight_information_t* flight_information)
{
    return mavlink_msg_flight_information_pack(system_id, component_id, msg, flight_information->time_boot_ms, flight_information->arming_time_utc, flight_information->takeoff_time_utc, flight_information->flight_uuid);
}

/**
 * @brief Encode a flight_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flight_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flight_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_flight_information_t* flight_information)
{
    return mavlink_msg_flight_information_pack_chan(system_id, component_id, chan, msg, flight_information->time_boot_ms, flight_information->arming_time_utc, flight_information->takeoff_time_utc, flight_information->flight_uuid);
}

/**
 * @brief Send a flight_information message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param arming_time_utc Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
 * @param takeoff_time_utc Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
 * @param flight_uuid Universally unique identifier (UUID) of flight, should correspond to name of logfiles
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flight_information_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN];
    _mav_put_uint64_t(buf, 0, arming_time_utc);
    _mav_put_uint64_t(buf, 8, takeoff_time_utc);
    _mav_put_uint64_t(buf, 16, flight_uuid);
    _mav_put_uint32_t(buf, 24, time_boot_ms);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_INFORMATION, buf, MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_CRC);
#else
    mavlink_flight_information_t packet;
    packet.arming_time_utc = arming_time_utc;
    packet.takeoff_time_utc = takeoff_time_utc;
    packet.flight_uuid = flight_uuid;
    packet.time_boot_ms = time_boot_ms;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a flight_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_flight_information_send_struct(mavlink_channel_t chan, const mavlink_flight_information_t* flight_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_flight_information_send(chan, flight_information->time_boot_ms, flight_information->arming_time_utc, flight_information->takeoff_time_utc, flight_information->flight_uuid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_INFORMATION, (const char *)flight_information, MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_flight_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint64_t arming_time_utc, uint64_t takeoff_time_utc, uint64_t flight_uuid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, arming_time_utc);
    _mav_put_uint64_t(buf, 8, takeoff_time_utc);
    _mav_put_uint64_t(buf, 16, flight_uuid);
    _mav_put_uint32_t(buf, 24, time_boot_ms);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_INFORMATION, buf, MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_CRC);
#else
    mavlink_flight_information_t *packet = (mavlink_flight_information_t *)msgbuf;
    packet->arming_time_utc = arming_time_utc;
    packet->takeoff_time_utc = takeoff_time_utc;
    packet->flight_uuid = flight_uuid;
    packet->time_boot_ms = time_boot_ms;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLIGHT_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_FLIGHT_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN, MAVLINK_MSG_ID_FLIGHT_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE FLIGHT_INFORMATION UNPACKING


/**
 * @brief Get field time_boot_ms from flight_information message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_flight_information_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field arming_time_utc from flight_information message
 *
 * @return Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown
 */
static inline uint64_t mavlink_msg_flight_information_get_arming_time_utc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field takeoff_time_utc from flight_information message
 *
 * @return Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown
 */
static inline uint64_t mavlink_msg_flight_information_get_takeoff_time_utc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field flight_uuid from flight_information message
 *
 * @return Universally unique identifier (UUID) of flight, should correspond to name of logfiles
 */
static inline uint64_t mavlink_msg_flight_information_get_flight_uuid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Decode a flight_information message into a struct
 *
 * @param msg The message to decode
 * @param flight_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_flight_information_decode(const mavlink_message_t* msg, mavlink_flight_information_t* flight_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    flight_information->arming_time_utc = mavlink_msg_flight_information_get_arming_time_utc(msg);
    flight_information->takeoff_time_utc = mavlink_msg_flight_information_get_takeoff_time_utc(msg);
    flight_information->flight_uuid = mavlink_msg_flight_information_get_flight_uuid(msg);
    flight_information->time_boot_ms = mavlink_msg_flight_information_get_time_boot_ms(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN;
        memset(flight_information, 0, MAVLINK_MSG_ID_FLIGHT_INFORMATION_LEN);
    memcpy(flight_information, _MAV_PAYLOAD(msg), len);
#endif
}
