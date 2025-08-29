#pragma once
// MESSAGE SENSOR_AIRFLOW_ANGLES PACKING

#define MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES 8016


typedef struct __mavlink_sensor_airflow_angles_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float angleofattack; /*< [deg] Angle of attack*/
 float sideslip; /*< [deg] Sideslip angle*/
 uint8_t angleofattack_valid; /*<  Angle of attack measurement valid*/
 uint8_t sideslip_valid; /*<  Sideslip angle measurement valid*/
} mavlink_sensor_airflow_angles_t;

#define MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN 18
#define MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN 18
#define MAVLINK_MSG_ID_8016_LEN 18
#define MAVLINK_MSG_ID_8016_MIN_LEN 18

#define MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_CRC 149
#define MAVLINK_MSG_ID_8016_CRC 149



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENSOR_AIRFLOW_ANGLES { \
    8016, \
    "SENSOR_AIRFLOW_ANGLES", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sensor_airflow_angles_t, timestamp) }, \
         { "angleofattack", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_airflow_angles_t, angleofattack) }, \
         { "angleofattack_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_sensor_airflow_angles_t, angleofattack_valid) }, \
         { "sideslip", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_airflow_angles_t, sideslip) }, \
         { "sideslip_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_sensor_airflow_angles_t, sideslip_valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENSOR_AIRFLOW_ANGLES { \
    "SENSOR_AIRFLOW_ANGLES", \
    5, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sensor_airflow_angles_t, timestamp) }, \
         { "angleofattack", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_airflow_angles_t, angleofattack) }, \
         { "angleofattack_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_sensor_airflow_angles_t, angleofattack_valid) }, \
         { "sideslip", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_airflow_angles_t, sideslip) }, \
         { "sideslip_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_sensor_airflow_angles_t, sideslip_valid) }, \
         } \
}
#endif

/**
 * @brief Pack a sensor_airflow_angles message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param angleofattack [deg] Angle of attack
 * @param angleofattack_valid  Angle of attack measurement valid
 * @param sideslip [deg] Sideslip angle
 * @param sideslip_valid  Sideslip angle measurement valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_airflow_angles_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float angleofattack, uint8_t angleofattack_valid, float sideslip, uint8_t sideslip_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, angleofattack);
    _mav_put_float(buf, 12, sideslip);
    _mav_put_uint8_t(buf, 16, angleofattack_valid);
    _mav_put_uint8_t(buf, 17, sideslip_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN);
#else
    mavlink_sensor_airflow_angles_t packet;
    packet.timestamp = timestamp;
    packet.angleofattack = angleofattack;
    packet.sideslip = sideslip;
    packet.angleofattack_valid = angleofattack_valid;
    packet.sideslip_valid = sideslip_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_CRC);
}

/**
 * @brief Pack a sensor_airflow_angles message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param angleofattack [deg] Angle of attack
 * @param angleofattack_valid  Angle of attack measurement valid
 * @param sideslip [deg] Sideslip angle
 * @param sideslip_valid  Sideslip angle measurement valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_airflow_angles_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, float angleofattack, uint8_t angleofattack_valid, float sideslip, uint8_t sideslip_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, angleofattack);
    _mav_put_float(buf, 12, sideslip);
    _mav_put_uint8_t(buf, 16, angleofattack_valid);
    _mav_put_uint8_t(buf, 17, sideslip_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN);
#else
    mavlink_sensor_airflow_angles_t packet;
    packet.timestamp = timestamp;
    packet.angleofattack = angleofattack;
    packet.sideslip = sideslip;
    packet.angleofattack_valid = angleofattack_valid;
    packet.sideslip_valid = sideslip_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN);
#endif
}

/**
 * @brief Pack a sensor_airflow_angles message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param angleofattack [deg] Angle of attack
 * @param angleofattack_valid  Angle of attack measurement valid
 * @param sideslip [deg] Sideslip angle
 * @param sideslip_valid  Sideslip angle measurement valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_airflow_angles_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float angleofattack,uint8_t angleofattack_valid,float sideslip,uint8_t sideslip_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, angleofattack);
    _mav_put_float(buf, 12, sideslip);
    _mav_put_uint8_t(buf, 16, angleofattack_valid);
    _mav_put_uint8_t(buf, 17, sideslip_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN);
#else
    mavlink_sensor_airflow_angles_t packet;
    packet.timestamp = timestamp;
    packet.angleofattack = angleofattack;
    packet.sideslip = sideslip;
    packet.angleofattack_valid = angleofattack_valid;
    packet.sideslip_valid = sideslip_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_CRC);
}

/**
 * @brief Encode a sensor_airflow_angles struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_airflow_angles C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_airflow_angles_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_airflow_angles_t* sensor_airflow_angles)
{
    return mavlink_msg_sensor_airflow_angles_pack(system_id, component_id, msg, sensor_airflow_angles->timestamp, sensor_airflow_angles->angleofattack, sensor_airflow_angles->angleofattack_valid, sensor_airflow_angles->sideslip, sensor_airflow_angles->sideslip_valid);
}

/**
 * @brief Encode a sensor_airflow_angles struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor_airflow_angles C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_airflow_angles_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_airflow_angles_t* sensor_airflow_angles)
{
    return mavlink_msg_sensor_airflow_angles_pack_chan(system_id, component_id, chan, msg, sensor_airflow_angles->timestamp, sensor_airflow_angles->angleofattack, sensor_airflow_angles->angleofattack_valid, sensor_airflow_angles->sideslip, sensor_airflow_angles->sideslip_valid);
}

/**
 * @brief Encode a sensor_airflow_angles struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sensor_airflow_angles C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_airflow_angles_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sensor_airflow_angles_t* sensor_airflow_angles)
{
    return mavlink_msg_sensor_airflow_angles_pack_status(system_id, component_id, _status, msg,  sensor_airflow_angles->timestamp, sensor_airflow_angles->angleofattack, sensor_airflow_angles->angleofattack_valid, sensor_airflow_angles->sideslip, sensor_airflow_angles->sideslip_valid);
}

/**
 * @brief Send a sensor_airflow_angles message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param angleofattack [deg] Angle of attack
 * @param angleofattack_valid  Angle of attack measurement valid
 * @param sideslip [deg] Sideslip angle
 * @param sideslip_valid  Sideslip angle measurement valid
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_airflow_angles_send(mavlink_channel_t chan, uint64_t timestamp, float angleofattack, uint8_t angleofattack_valid, float sideslip, uint8_t sideslip_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, angleofattack);
    _mav_put_float(buf, 12, sideslip);
    _mav_put_uint8_t(buf, 16, angleofattack_valid);
    _mav_put_uint8_t(buf, 17, sideslip_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES, buf, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_CRC);
#else
    mavlink_sensor_airflow_angles_t packet;
    packet.timestamp = timestamp;
    packet.angleofattack = angleofattack;
    packet.sideslip = sideslip;
    packet.angleofattack_valid = angleofattack_valid;
    packet.sideslip_valid = sideslip_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_CRC);
#endif
}

/**
 * @brief Send a sensor_airflow_angles message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sensor_airflow_angles_send_struct(mavlink_channel_t chan, const mavlink_sensor_airflow_angles_t* sensor_airflow_angles)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensor_airflow_angles_send(chan, sensor_airflow_angles->timestamp, sensor_airflow_angles->angleofattack, sensor_airflow_angles->angleofattack_valid, sensor_airflow_angles->sideslip, sensor_airflow_angles->sideslip_valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES, (const char *)sensor_airflow_angles, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensor_airflow_angles_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float angleofattack, uint8_t angleofattack_valid, float sideslip, uint8_t sideslip_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, angleofattack);
    _mav_put_float(buf, 12, sideslip);
    _mav_put_uint8_t(buf, 16, angleofattack_valid);
    _mav_put_uint8_t(buf, 17, sideslip_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES, buf, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_CRC);
#else
    mavlink_sensor_airflow_angles_t *packet = (mavlink_sensor_airflow_angles_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->angleofattack = angleofattack;
    packet->sideslip = sideslip;
    packet->angleofattack_valid = angleofattack_valid;
    packet->sideslip_valid = sideslip_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES, (const char *)packet, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_MIN_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_CRC);
#endif
}
#endif

#endif

// MESSAGE SENSOR_AIRFLOW_ANGLES UNPACKING


/**
 * @brief Get field timestamp from sensor_airflow_angles message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_sensor_airflow_angles_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field angleofattack from sensor_airflow_angles message
 *
 * @return [deg] Angle of attack
 */
static inline float mavlink_msg_sensor_airflow_angles_get_angleofattack(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field angleofattack_valid from sensor_airflow_angles message
 *
 * @return  Angle of attack measurement valid
 */
static inline uint8_t mavlink_msg_sensor_airflow_angles_get_angleofattack_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field sideslip from sensor_airflow_angles message
 *
 * @return [deg] Sideslip angle
 */
static inline float mavlink_msg_sensor_airflow_angles_get_sideslip(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field sideslip_valid from sensor_airflow_angles message
 *
 * @return  Sideslip angle measurement valid
 */
static inline uint8_t mavlink_msg_sensor_airflow_angles_get_sideslip_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Decode a sensor_airflow_angles message into a struct
 *
 * @param msg The message to decode
 * @param sensor_airflow_angles C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_airflow_angles_decode(const mavlink_message_t* msg, mavlink_sensor_airflow_angles_t* sensor_airflow_angles)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sensor_airflow_angles->timestamp = mavlink_msg_sensor_airflow_angles_get_timestamp(msg);
    sensor_airflow_angles->angleofattack = mavlink_msg_sensor_airflow_angles_get_angleofattack(msg);
    sensor_airflow_angles->sideslip = mavlink_msg_sensor_airflow_angles_get_sideslip(msg);
    sensor_airflow_angles->angleofattack_valid = mavlink_msg_sensor_airflow_angles_get_angleofattack_valid(msg);
    sensor_airflow_angles->sideslip_valid = mavlink_msg_sensor_airflow_angles_get_sideslip_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN? msg->len : MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN;
        memset(sensor_airflow_angles, 0, MAVLINK_MSG_ID_SENSOR_AIRFLOW_ANGLES_LEN);
    memcpy(sensor_airflow_angles, _MAV_PAYLOAD(msg), len);
#endif
}
