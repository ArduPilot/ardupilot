#pragma once
// MESSAGE MOUNT_ORIENTATION PACKING

#define MAVLINK_MSG_ID_MOUNT_ORIENTATION 265


typedef struct __mavlink_mount_orientation_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float roll; /*< [deg] Roll in global frame (set to NaN for invalid).*/
 float pitch; /*< [deg] Pitch in global frame (set to NaN for invalid).*/
 float yaw; /*< [deg] Yaw relative to vehicle (set to NaN for invalid).*/
 float yaw_absolute; /*< [deg] Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).*/
} mavlink_mount_orientation_t;

#define MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN 20
#define MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN 16
#define MAVLINK_MSG_ID_265_LEN 20
#define MAVLINK_MSG_ID_265_MIN_LEN 16

#define MAVLINK_MSG_ID_MOUNT_ORIENTATION_CRC 26
#define MAVLINK_MSG_ID_265_CRC 26



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOUNT_ORIENTATION { \
    265, \
    "MOUNT_ORIENTATION", \
    5, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mount_orientation_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mount_orientation_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mount_orientation_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mount_orientation_t, yaw) }, \
         { "yaw_absolute", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mount_orientation_t, yaw_absolute) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOUNT_ORIENTATION { \
    "MOUNT_ORIENTATION", \
    5, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mount_orientation_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mount_orientation_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mount_orientation_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mount_orientation_t, yaw) }, \
         { "yaw_absolute", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mount_orientation_t, yaw_absolute) }, \
         } \
}
#endif

/**
 * @brief Pack a mount_orientation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [deg] Roll in global frame (set to NaN for invalid).
 * @param pitch [deg] Pitch in global frame (set to NaN for invalid).
 * @param yaw [deg] Yaw relative to vehicle (set to NaN for invalid).
 * @param yaw_absolute [deg] Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_orientation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float roll, float pitch, float yaw, float yaw_absolute)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, yaw_absolute);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN);
#else
    mavlink_mount_orientation_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.yaw_absolute = yaw_absolute;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_ORIENTATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_CRC);
}

/**
 * @brief Pack a mount_orientation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [deg] Roll in global frame (set to NaN for invalid).
 * @param pitch [deg] Pitch in global frame (set to NaN for invalid).
 * @param yaw [deg] Yaw relative to vehicle (set to NaN for invalid).
 * @param yaw_absolute [deg] Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_orientation_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float roll, float pitch, float yaw, float yaw_absolute)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, yaw_absolute);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN);
#else
    mavlink_mount_orientation_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.yaw_absolute = yaw_absolute;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_ORIENTATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN);
#endif
}

/**
 * @brief Pack a mount_orientation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [deg] Roll in global frame (set to NaN for invalid).
 * @param pitch [deg] Pitch in global frame (set to NaN for invalid).
 * @param yaw [deg] Yaw relative to vehicle (set to NaN for invalid).
 * @param yaw_absolute [deg] Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_orientation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float roll,float pitch,float yaw,float yaw_absolute)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, yaw_absolute);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN);
#else
    mavlink_mount_orientation_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.yaw_absolute = yaw_absolute;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOUNT_ORIENTATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_CRC);
}

/**
 * @brief Encode a mount_orientation struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mount_orientation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_orientation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mount_orientation_t* mount_orientation)
{
    return mavlink_msg_mount_orientation_pack(system_id, component_id, msg, mount_orientation->time_boot_ms, mount_orientation->roll, mount_orientation->pitch, mount_orientation->yaw, mount_orientation->yaw_absolute);
}

/**
 * @brief Encode a mount_orientation struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mount_orientation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_orientation_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mount_orientation_t* mount_orientation)
{
    return mavlink_msg_mount_orientation_pack_chan(system_id, component_id, chan, msg, mount_orientation->time_boot_ms, mount_orientation->roll, mount_orientation->pitch, mount_orientation->yaw, mount_orientation->yaw_absolute);
}

/**
 * @brief Encode a mount_orientation struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mount_orientation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_orientation_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mount_orientation_t* mount_orientation)
{
    return mavlink_msg_mount_orientation_pack_status(system_id, component_id, _status, msg,  mount_orientation->time_boot_ms, mount_orientation->roll, mount_orientation->pitch, mount_orientation->yaw, mount_orientation->yaw_absolute);
}

/**
 * @brief Send a mount_orientation message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param roll [deg] Roll in global frame (set to NaN for invalid).
 * @param pitch [deg] Pitch in global frame (set to NaN for invalid).
 * @param yaw [deg] Yaw relative to vehicle (set to NaN for invalid).
 * @param yaw_absolute [deg] Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mount_orientation_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float yaw_absolute)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, yaw_absolute);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_ORIENTATION, buf, MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_CRC);
#else
    mavlink_mount_orientation_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.yaw_absolute = yaw_absolute;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_ORIENTATION, (const char *)&packet, MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_CRC);
#endif
}

/**
 * @brief Send a mount_orientation message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mount_orientation_send_struct(mavlink_channel_t chan, const mavlink_mount_orientation_t* mount_orientation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mount_orientation_send(chan, mount_orientation->time_boot_ms, mount_orientation->roll, mount_orientation->pitch, mount_orientation->yaw, mount_orientation->yaw_absolute);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_ORIENTATION, (const char *)mount_orientation, MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mount_orientation_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float roll, float pitch, float yaw, float yaw_absolute)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, yaw_absolute);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_ORIENTATION, buf, MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_CRC);
#else
    mavlink_mount_orientation_t *packet = (mavlink_mount_orientation_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->yaw_absolute = yaw_absolute;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_ORIENTATION, (const char *)packet, MAVLINK_MSG_ID_MOUNT_ORIENTATION_MIN_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN, MAVLINK_MSG_ID_MOUNT_ORIENTATION_CRC);
#endif
}
#endif

#endif

// MESSAGE MOUNT_ORIENTATION UNPACKING


/**
 * @brief Get field time_boot_ms from mount_orientation message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_mount_orientation_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll from mount_orientation message
 *
 * @return [deg] Roll in global frame (set to NaN for invalid).
 */
static inline float mavlink_msg_mount_orientation_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from mount_orientation message
 *
 * @return [deg] Pitch in global frame (set to NaN for invalid).
 */
static inline float mavlink_msg_mount_orientation_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from mount_orientation message
 *
 * @return [deg] Yaw relative to vehicle (set to NaN for invalid).
 */
static inline float mavlink_msg_mount_orientation_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yaw_absolute from mount_orientation message
 *
 * @return [deg] Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).
 */
static inline float mavlink_msg_mount_orientation_get_yaw_absolute(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a mount_orientation message into a struct
 *
 * @param msg The message to decode
 * @param mount_orientation C-struct to decode the message contents into
 */
static inline void mavlink_msg_mount_orientation_decode(const mavlink_message_t* msg, mavlink_mount_orientation_t* mount_orientation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mount_orientation->time_boot_ms = mavlink_msg_mount_orientation_get_time_boot_ms(msg);
    mount_orientation->roll = mavlink_msg_mount_orientation_get_roll(msg);
    mount_orientation->pitch = mavlink_msg_mount_orientation_get_pitch(msg);
    mount_orientation->yaw = mavlink_msg_mount_orientation_get_yaw(msg);
    mount_orientation->yaw_absolute = mavlink_msg_mount_orientation_get_yaw_absolute(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN? msg->len : MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN;
        memset(mount_orientation, 0, MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN);
    memcpy(mount_orientation, _MAV_PAYLOAD(msg), len);
#endif
}
