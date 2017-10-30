#pragma once
// MESSAGE RPM PACKING

#define MAVLINK_MSG_ID_RPM 226

MAVPACKED(
typedef struct __mavlink_rpm_t {
 float rpm1; /*< RPM Sensor1*/
 float rpm2; /*< RPM Sensor2*/
}) mavlink_rpm_t;

#define MAVLINK_MSG_ID_RPM_LEN 8
#define MAVLINK_MSG_ID_RPM_MIN_LEN 8
#define MAVLINK_MSG_ID_226_LEN 8
#define MAVLINK_MSG_ID_226_MIN_LEN 8

#define MAVLINK_MSG_ID_RPM_CRC 207
#define MAVLINK_MSG_ID_226_CRC 207



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RPM { \
    226, \
    "RPM", \
    2, \
    {  { "rpm1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rpm_t, rpm1) }, \
         { "rpm2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rpm_t, rpm2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RPM { \
    "RPM", \
    2, \
    {  { "rpm1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rpm_t, rpm1) }, \
         { "rpm2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rpm_t, rpm2) }, \
         } \
}
#endif

/**
 * @brief Pack a rpm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rpm1 RPM Sensor1
 * @param rpm2 RPM Sensor2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float rpm1, float rpm2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RPM_LEN];
    _mav_put_float(buf, 0, rpm1);
    _mav_put_float(buf, 4, rpm2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPM_LEN);
#else
    mavlink_rpm_t packet;
    packet.rpm1 = rpm1;
    packet.rpm2 = rpm2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RPM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
}

/**
 * @brief Pack a rpm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rpm1 RPM Sensor1
 * @param rpm2 RPM Sensor2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rpm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float rpm1,float rpm2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RPM_LEN];
    _mav_put_float(buf, 0, rpm1);
    _mav_put_float(buf, 4, rpm2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RPM_LEN);
#else
    mavlink_rpm_t packet;
    packet.rpm1 = rpm1;
    packet.rpm2 = rpm2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RPM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RPM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
}

/**
 * @brief Encode a rpm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rpm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rpm_t* rpm)
{
    return mavlink_msg_rpm_pack(system_id, component_id, msg, rpm->rpm1, rpm->rpm2);
}

/**
 * @brief Encode a rpm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rpm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rpm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rpm_t* rpm)
{
    return mavlink_msg_rpm_pack_chan(system_id, component_id, chan, msg, rpm->rpm1, rpm->rpm2);
}

/**
 * @brief Send a rpm message
 * @param chan MAVLink channel to send the message
 *
 * @param rpm1 RPM Sensor1
 * @param rpm2 RPM Sensor2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rpm_send(mavlink_channel_t chan, float rpm1, float rpm2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RPM_LEN];
    _mav_put_float(buf, 0, rpm1);
    _mav_put_float(buf, 4, rpm2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM, buf, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
#else
    mavlink_rpm_t packet;
    packet.rpm1 = rpm1;
    packet.rpm2 = rpm2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM, (const char *)&packet, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
#endif
}

/**
 * @brief Send a rpm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rpm_send_struct(mavlink_channel_t chan, const mavlink_rpm_t* rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rpm_send(chan, rpm->rpm1, rpm->rpm2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM, (const char *)rpm, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
#endif
}

#if MAVLINK_MSG_ID_RPM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rpm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float rpm1, float rpm2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, rpm1);
    _mav_put_float(buf, 4, rpm2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM, buf, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
#else
    mavlink_rpm_t *packet = (mavlink_rpm_t *)msgbuf;
    packet->rpm1 = rpm1;
    packet->rpm2 = rpm2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RPM, (const char *)packet, MAVLINK_MSG_ID_RPM_MIN_LEN, MAVLINK_MSG_ID_RPM_LEN, MAVLINK_MSG_ID_RPM_CRC);
#endif
}
#endif

#endif

// MESSAGE RPM UNPACKING


/**
 * @brief Get field rpm1 from rpm message
 *
 * @return RPM Sensor1
 */
static inline float mavlink_msg_rpm_get_rpm1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field rpm2 from rpm message
 *
 * @return RPM Sensor2
 */
static inline float mavlink_msg_rpm_get_rpm2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a rpm message into a struct
 *
 * @param msg The message to decode
 * @param rpm C-struct to decode the message contents into
 */
static inline void mavlink_msg_rpm_decode(const mavlink_message_t* msg, mavlink_rpm_t* rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rpm->rpm1 = mavlink_msg_rpm_get_rpm1(msg);
    rpm->rpm2 = mavlink_msg_rpm_get_rpm2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RPM_LEN? msg->len : MAVLINK_MSG_ID_RPM_LEN;
        memset(rpm, 0, MAVLINK_MSG_ID_RPM_LEN);
    memcpy(rpm, _MAV_PAYLOAD(msg), len);
#endif
}
