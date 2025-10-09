#pragma once
// MESSAGE CAMERA_CAPTURE_STATUS PACKING

#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS 262

MAVPACKED(
typedef struct __mavlink_camera_capture_status_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float image_interval; /*< [s] Image capture interval*/
 uint32_t recording_time_ms; /*< [ms] Time since recording started*/
 float available_capacity; /*< [MiB] Available storage capacity.*/
 uint8_t image_status; /*<  Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)*/
 uint8_t video_status; /*<  Current status of video capturing (0: idle, 1: capture in progress)*/
 int32_t image_count; /*<  Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).*/
}) mavlink_camera_capture_status_t;

#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN 22
#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN 18
#define MAVLINK_MSG_ID_262_LEN 22
#define MAVLINK_MSG_ID_262_MIN_LEN 18

#define MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC 12
#define MAVLINK_MSG_ID_262_CRC 12



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_CAPTURE_STATUS { \
    262, \
    "CAMERA_CAPTURE_STATUS", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_capture_status_t, time_boot_ms) }, \
         { "image_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_camera_capture_status_t, image_status) }, \
         { "video_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_camera_capture_status_t, video_status) }, \
         { "image_interval", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_camera_capture_status_t, image_interval) }, \
         { "recording_time_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_camera_capture_status_t, recording_time_ms) }, \
         { "available_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_capture_status_t, available_capacity) }, \
         { "image_count", NULL, MAVLINK_TYPE_INT32_T, 0, 18, offsetof(mavlink_camera_capture_status_t, image_count) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_CAPTURE_STATUS { \
    "CAMERA_CAPTURE_STATUS", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_capture_status_t, time_boot_ms) }, \
         { "image_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_camera_capture_status_t, image_status) }, \
         { "video_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_camera_capture_status_t, video_status) }, \
         { "image_interval", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_camera_capture_status_t, image_interval) }, \
         { "recording_time_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_camera_capture_status_t, recording_time_ms) }, \
         { "available_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_capture_status_t, available_capacity) }, \
         { "image_count", NULL, MAVLINK_TYPE_INT32_T, 0, 18, offsetof(mavlink_camera_capture_status_t, image_count) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_capture_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param image_status  Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
 * @param video_status  Current status of video capturing (0: idle, 1: capture in progress)
 * @param image_interval [s] Image capture interval
 * @param recording_time_ms [ms] Time since recording started
 * @param available_capacity [MiB] Available storage capacity.
 * @param image_count  Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_capture_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, image_interval);
    _mav_put_uint32_t(buf, 8, recording_time_ms);
    _mav_put_float(buf, 12, available_capacity);
    _mav_put_uint8_t(buf, 16, image_status);
    _mav_put_uint8_t(buf, 17, video_status);
    _mav_put_int32_t(buf, 18, image_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#else
    mavlink_camera_capture_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.image_interval = image_interval;
    packet.recording_time_ms = recording_time_ms;
    packet.available_capacity = available_capacity;
    packet.image_status = image_status;
    packet.video_status = video_status;
    packet.image_count = image_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
}

/**
 * @brief Pack a camera_capture_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param image_status  Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
 * @param video_status  Current status of video capturing (0: idle, 1: capture in progress)
 * @param image_interval [s] Image capture interval
 * @param recording_time_ms [ms] Time since recording started
 * @param available_capacity [MiB] Available storage capacity.
 * @param image_count  Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_capture_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, image_interval);
    _mav_put_uint32_t(buf, 8, recording_time_ms);
    _mav_put_float(buf, 12, available_capacity);
    _mav_put_uint8_t(buf, 16, image_status);
    _mav_put_uint8_t(buf, 17, video_status);
    _mav_put_int32_t(buf, 18, image_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#else
    mavlink_camera_capture_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.image_interval = image_interval;
    packet.recording_time_ms = recording_time_ms;
    packet.available_capacity = available_capacity;
    packet.image_status = image_status;
    packet.video_status = video_status;
    packet.image_count = image_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a camera_capture_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param image_status  Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
 * @param video_status  Current status of video capturing (0: idle, 1: capture in progress)
 * @param image_interval [s] Image capture interval
 * @param recording_time_ms [ms] Time since recording started
 * @param available_capacity [MiB] Available storage capacity.
 * @param image_count  Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_capture_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t image_status,uint8_t video_status,float image_interval,uint32_t recording_time_ms,float available_capacity,int32_t image_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, image_interval);
    _mav_put_uint32_t(buf, 8, recording_time_ms);
    _mav_put_float(buf, 12, available_capacity);
    _mav_put_uint8_t(buf, 16, image_status);
    _mav_put_uint8_t(buf, 17, video_status);
    _mav_put_int32_t(buf, 18, image_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#else
    mavlink_camera_capture_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.image_interval = image_interval;
    packet.recording_time_ms = recording_time_ms;
    packet.available_capacity = available_capacity;
    packet.image_status = image_status;
    packet.video_status = video_status;
    packet.image_count = image_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
}

/**
 * @brief Encode a camera_capture_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_capture_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_capture_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_capture_status_t* camera_capture_status)
{
    return mavlink_msg_camera_capture_status_pack(system_id, component_id, msg, camera_capture_status->time_boot_ms, camera_capture_status->image_status, camera_capture_status->video_status, camera_capture_status->image_interval, camera_capture_status->recording_time_ms, camera_capture_status->available_capacity, camera_capture_status->image_count);
}

/**
 * @brief Encode a camera_capture_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_capture_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_capture_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_capture_status_t* camera_capture_status)
{
    return mavlink_msg_camera_capture_status_pack_chan(system_id, component_id, chan, msg, camera_capture_status->time_boot_ms, camera_capture_status->image_status, camera_capture_status->video_status, camera_capture_status->image_interval, camera_capture_status->recording_time_ms, camera_capture_status->available_capacity, camera_capture_status->image_count);
}

/**
 * @brief Encode a camera_capture_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param camera_capture_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_capture_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_camera_capture_status_t* camera_capture_status)
{
    return mavlink_msg_camera_capture_status_pack_status(system_id, component_id, _status, msg,  camera_capture_status->time_boot_ms, camera_capture_status->image_status, camera_capture_status->video_status, camera_capture_status->image_interval, camera_capture_status->recording_time_ms, camera_capture_status->available_capacity, camera_capture_status->image_count);
}

/**
 * @brief Send a camera_capture_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param image_status  Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
 * @param video_status  Current status of video capturing (0: idle, 1: capture in progress)
 * @param image_interval [s] Image capture interval
 * @param recording_time_ms [ms] Time since recording started
 * @param available_capacity [MiB] Available storage capacity.
 * @param image_count  Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_capture_status_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, image_interval);
    _mav_put_uint32_t(buf, 8, recording_time_ms);
    _mav_put_float(buf, 12, available_capacity);
    _mav_put_uint8_t(buf, 16, image_status);
    _mav_put_uint8_t(buf, 17, video_status);
    _mav_put_int32_t(buf, 18, image_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, buf, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#else
    mavlink_camera_capture_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.image_interval = image_interval;
    packet.recording_time_ms = recording_time_ms;
    packet.available_capacity = available_capacity;
    packet.image_status = image_status;
    packet.video_status = video_status;
    packet.image_count = image_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#endif
}

/**
 * @brief Send a camera_capture_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_capture_status_send_struct(mavlink_channel_t chan, const mavlink_camera_capture_status_t* camera_capture_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_capture_status_send(chan, camera_capture_status->time_boot_ms, camera_capture_status->image_status, camera_capture_status->video_status, camera_capture_status->image_interval, camera_capture_status->recording_time_ms, camera_capture_status->available_capacity, camera_capture_status->image_count);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, (const char *)camera_capture_status, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_capture_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t image_status, uint8_t video_status, float image_interval, uint32_t recording_time_ms, float available_capacity, int32_t image_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, image_interval);
    _mav_put_uint32_t(buf, 8, recording_time_ms);
    _mav_put_float(buf, 12, available_capacity);
    _mav_put_uint8_t(buf, 16, image_status);
    _mav_put_uint8_t(buf, 17, video_status);
    _mav_put_int32_t(buf, 18, image_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, buf, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#else
    mavlink_camera_capture_status_t *packet = (mavlink_camera_capture_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->image_interval = image_interval;
    packet->recording_time_ms = recording_time_ms;
    packet->available_capacity = available_capacity;
    packet->image_status = image_status;
    packet->video_status = video_status;
    packet->image_count = image_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, (const char *)packet, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_CAPTURE_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from camera_capture_status message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_camera_capture_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field image_status from camera_capture_status message
 *
 * @return  Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
 */
static inline uint8_t mavlink_msg_camera_capture_status_get_image_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field video_status from camera_capture_status message
 *
 * @return  Current status of video capturing (0: idle, 1: capture in progress)
 */
static inline uint8_t mavlink_msg_camera_capture_status_get_video_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field image_interval from camera_capture_status message
 *
 * @return [s] Image capture interval
 */
static inline float mavlink_msg_camera_capture_status_get_image_interval(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field recording_time_ms from camera_capture_status message
 *
 * @return [ms] Time since recording started
 */
static inline uint32_t mavlink_msg_camera_capture_status_get_recording_time_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field available_capacity from camera_capture_status message
 *
 * @return [MiB] Available storage capacity.
 */
static inline float mavlink_msg_camera_capture_status_get_available_capacity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field image_count from camera_capture_status message
 *
 * @return  Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).
 */
static inline int32_t mavlink_msg_camera_capture_status_get_image_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  18);
}

/**
 * @brief Decode a camera_capture_status message into a struct
 *
 * @param msg The message to decode
 * @param camera_capture_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_capture_status_decode(const mavlink_message_t* msg, mavlink_camera_capture_status_t* camera_capture_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_capture_status->time_boot_ms = mavlink_msg_camera_capture_status_get_time_boot_ms(msg);
    camera_capture_status->image_interval = mavlink_msg_camera_capture_status_get_image_interval(msg);
    camera_capture_status->recording_time_ms = mavlink_msg_camera_capture_status_get_recording_time_ms(msg);
    camera_capture_status->available_capacity = mavlink_msg_camera_capture_status_get_available_capacity(msg);
    camera_capture_status->image_status = mavlink_msg_camera_capture_status_get_image_status(msg);
    camera_capture_status->video_status = mavlink_msg_camera_capture_status_get_video_status(msg);
    camera_capture_status->image_count = mavlink_msg_camera_capture_status_get_image_count(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN;
        memset(camera_capture_status, 0, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN);
    memcpy(camera_capture_status, _MAV_PAYLOAD(msg), len);
#endif
}
