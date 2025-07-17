#pragma once
// MESSAGE CAMERA_THERMAL_RANGE PACKING

#define MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE 277


typedef struct __mavlink_camera_thermal_range_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float max; /*< [degC] Temperature max.*/
 float max_point_x; /*<  Temperature max point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.*/
 float max_point_y; /*<  Temperature max point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.*/
 float min; /*< [degC] Temperature min.*/
 float min_point_x; /*<  Temperature min point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.*/
 float min_point_y; /*<  Temperature min point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.*/
 uint8_t stream_id; /*<  Video Stream ID (1 for first, 2 for second, etc.)*/
 uint8_t camera_device_id; /*<  Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).*/
} mavlink_camera_thermal_range_t;

#define MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN 30
#define MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN 30
#define MAVLINK_MSG_ID_277_LEN 30
#define MAVLINK_MSG_ID_277_MIN_LEN 30

#define MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_CRC 62
#define MAVLINK_MSG_ID_277_CRC 62



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_THERMAL_RANGE { \
    277, \
    "CAMERA_THERMAL_RANGE", \
    9, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_thermal_range_t, time_boot_ms) }, \
         { "stream_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_camera_thermal_range_t, stream_id) }, \
         { "camera_device_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_camera_thermal_range_t, camera_device_id) }, \
         { "max", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_camera_thermal_range_t, max) }, \
         { "max_point_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_thermal_range_t, max_point_x) }, \
         { "max_point_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_thermal_range_t, max_point_y) }, \
         { "min", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_thermal_range_t, min) }, \
         { "min_point_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_thermal_range_t, min_point_x) }, \
         { "min_point_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_thermal_range_t, min_point_y) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_THERMAL_RANGE { \
    "CAMERA_THERMAL_RANGE", \
    9, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_thermal_range_t, time_boot_ms) }, \
         { "stream_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_camera_thermal_range_t, stream_id) }, \
         { "camera_device_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_camera_thermal_range_t, camera_device_id) }, \
         { "max", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_camera_thermal_range_t, max) }, \
         { "max_point_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_thermal_range_t, max_point_x) }, \
         { "max_point_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_thermal_range_t, max_point_y) }, \
         { "min", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_thermal_range_t, min) }, \
         { "min_point_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_thermal_range_t, min_point_x) }, \
         { "min_point_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_thermal_range_t, min_point_y) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_thermal_range message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param stream_id  Video Stream ID (1 for first, 2 for second, etc.)
 * @param camera_device_id  Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
 * @param max [degC] Temperature max.
 * @param max_point_x  Temperature max point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
 * @param max_point_y  Temperature max point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
 * @param min [degC] Temperature min.
 * @param min_point_x  Temperature min point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
 * @param min_point_y  Temperature min point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_thermal_range_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t stream_id, uint8_t camera_device_id, float max, float max_point_x, float max_point_y, float min, float min_point_x, float min_point_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, max);
    _mav_put_float(buf, 8, max_point_x);
    _mav_put_float(buf, 12, max_point_y);
    _mav_put_float(buf, 16, min);
    _mav_put_float(buf, 20, min_point_x);
    _mav_put_float(buf, 24, min_point_y);
    _mav_put_uint8_t(buf, 28, stream_id);
    _mav_put_uint8_t(buf, 29, camera_device_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN);
#else
    mavlink_camera_thermal_range_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.max = max;
    packet.max_point_x = max_point_x;
    packet.max_point_y = max_point_y;
    packet.min = min;
    packet.min_point_x = min_point_x;
    packet.min_point_y = min_point_y;
    packet.stream_id = stream_id;
    packet.camera_device_id = camera_device_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_CRC);
}

/**
 * @brief Pack a camera_thermal_range message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param stream_id  Video Stream ID (1 for first, 2 for second, etc.)
 * @param camera_device_id  Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
 * @param max [degC] Temperature max.
 * @param max_point_x  Temperature max point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
 * @param max_point_y  Temperature max point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
 * @param min [degC] Temperature min.
 * @param min_point_x  Temperature min point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
 * @param min_point_y  Temperature min point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_thermal_range_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t stream_id, uint8_t camera_device_id, float max, float max_point_x, float max_point_y, float min, float min_point_x, float min_point_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, max);
    _mav_put_float(buf, 8, max_point_x);
    _mav_put_float(buf, 12, max_point_y);
    _mav_put_float(buf, 16, min);
    _mav_put_float(buf, 20, min_point_x);
    _mav_put_float(buf, 24, min_point_y);
    _mav_put_uint8_t(buf, 28, stream_id);
    _mav_put_uint8_t(buf, 29, camera_device_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN);
#else
    mavlink_camera_thermal_range_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.max = max;
    packet.max_point_x = max_point_x;
    packet.max_point_y = max_point_y;
    packet.min = min;
    packet.min_point_x = min_point_x;
    packet.min_point_y = min_point_y;
    packet.stream_id = stream_id;
    packet.camera_device_id = camera_device_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN);
#endif
}

/**
 * @brief Pack a camera_thermal_range message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param stream_id  Video Stream ID (1 for first, 2 for second, etc.)
 * @param camera_device_id  Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
 * @param max [degC] Temperature max.
 * @param max_point_x  Temperature max point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
 * @param max_point_y  Temperature max point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
 * @param min [degC] Temperature min.
 * @param min_point_x  Temperature min point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
 * @param min_point_y  Temperature min point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_thermal_range_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t stream_id,uint8_t camera_device_id,float max,float max_point_x,float max_point_y,float min,float min_point_x,float min_point_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, max);
    _mav_put_float(buf, 8, max_point_x);
    _mav_put_float(buf, 12, max_point_y);
    _mav_put_float(buf, 16, min);
    _mav_put_float(buf, 20, min_point_x);
    _mav_put_float(buf, 24, min_point_y);
    _mav_put_uint8_t(buf, 28, stream_id);
    _mav_put_uint8_t(buf, 29, camera_device_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN);
#else
    mavlink_camera_thermal_range_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.max = max;
    packet.max_point_x = max_point_x;
    packet.max_point_y = max_point_y;
    packet.min = min;
    packet.min_point_x = min_point_x;
    packet.min_point_y = min_point_y;
    packet.stream_id = stream_id;
    packet.camera_device_id = camera_device_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_CRC);
}

/**
 * @brief Encode a camera_thermal_range struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_thermal_range C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_thermal_range_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_thermal_range_t* camera_thermal_range)
{
    return mavlink_msg_camera_thermal_range_pack(system_id, component_id, msg, camera_thermal_range->time_boot_ms, camera_thermal_range->stream_id, camera_thermal_range->camera_device_id, camera_thermal_range->max, camera_thermal_range->max_point_x, camera_thermal_range->max_point_y, camera_thermal_range->min, camera_thermal_range->min_point_x, camera_thermal_range->min_point_y);
}

/**
 * @brief Encode a camera_thermal_range struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_thermal_range C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_thermal_range_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_thermal_range_t* camera_thermal_range)
{
    return mavlink_msg_camera_thermal_range_pack_chan(system_id, component_id, chan, msg, camera_thermal_range->time_boot_ms, camera_thermal_range->stream_id, camera_thermal_range->camera_device_id, camera_thermal_range->max, camera_thermal_range->max_point_x, camera_thermal_range->max_point_y, camera_thermal_range->min, camera_thermal_range->min_point_x, camera_thermal_range->min_point_y);
}

/**
 * @brief Encode a camera_thermal_range struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param camera_thermal_range C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_thermal_range_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_camera_thermal_range_t* camera_thermal_range)
{
    return mavlink_msg_camera_thermal_range_pack_status(system_id, component_id, _status, msg,  camera_thermal_range->time_boot_ms, camera_thermal_range->stream_id, camera_thermal_range->camera_device_id, camera_thermal_range->max, camera_thermal_range->max_point_x, camera_thermal_range->max_point_y, camera_thermal_range->min, camera_thermal_range->min_point_x, camera_thermal_range->min_point_y);
}

/**
 * @brief Send a camera_thermal_range message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param stream_id  Video Stream ID (1 for first, 2 for second, etc.)
 * @param camera_device_id  Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
 * @param max [degC] Temperature max.
 * @param max_point_x  Temperature max point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
 * @param max_point_y  Temperature max point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
 * @param min [degC] Temperature min.
 * @param min_point_x  Temperature min point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
 * @param min_point_y  Temperature min point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_thermal_range_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t stream_id, uint8_t camera_device_id, float max, float max_point_x, float max_point_y, float min, float min_point_x, float min_point_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, max);
    _mav_put_float(buf, 8, max_point_x);
    _mav_put_float(buf, 12, max_point_y);
    _mav_put_float(buf, 16, min);
    _mav_put_float(buf, 20, min_point_x);
    _mav_put_float(buf, 24, min_point_y);
    _mav_put_uint8_t(buf, 28, stream_id);
    _mav_put_uint8_t(buf, 29, camera_device_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE, buf, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_CRC);
#else
    mavlink_camera_thermal_range_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.max = max;
    packet.max_point_x = max_point_x;
    packet.max_point_y = max_point_y;
    packet.min = min;
    packet.min_point_x = min_point_x;
    packet.min_point_y = min_point_y;
    packet.stream_id = stream_id;
    packet.camera_device_id = camera_device_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_CRC);
#endif
}

/**
 * @brief Send a camera_thermal_range message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_thermal_range_send_struct(mavlink_channel_t chan, const mavlink_camera_thermal_range_t* camera_thermal_range)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_thermal_range_send(chan, camera_thermal_range->time_boot_ms, camera_thermal_range->stream_id, camera_thermal_range->camera_device_id, camera_thermal_range->max, camera_thermal_range->max_point_x, camera_thermal_range->max_point_y, camera_thermal_range->min, camera_thermal_range->min_point_x, camera_thermal_range->min_point_y);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE, (const char *)camera_thermal_range, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_thermal_range_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t stream_id, uint8_t camera_device_id, float max, float max_point_x, float max_point_y, float min, float min_point_x, float min_point_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, max);
    _mav_put_float(buf, 8, max_point_x);
    _mav_put_float(buf, 12, max_point_y);
    _mav_put_float(buf, 16, min);
    _mav_put_float(buf, 20, min_point_x);
    _mav_put_float(buf, 24, min_point_y);
    _mav_put_uint8_t(buf, 28, stream_id);
    _mav_put_uint8_t(buf, 29, camera_device_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE, buf, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_CRC);
#else
    mavlink_camera_thermal_range_t *packet = (mavlink_camera_thermal_range_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->max = max;
    packet->max_point_x = max_point_x;
    packet->max_point_y = max_point_y;
    packet->min = min;
    packet->min_point_x = min_point_x;
    packet->min_point_y = min_point_y;
    packet->stream_id = stream_id;
    packet->camera_device_id = camera_device_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE, (const char *)packet, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_MIN_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_THERMAL_RANGE UNPACKING


/**
 * @brief Get field time_boot_ms from camera_thermal_range message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_camera_thermal_range_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field stream_id from camera_thermal_range message
 *
 * @return  Video Stream ID (1 for first, 2 for second, etc.)
 */
static inline uint8_t mavlink_msg_camera_thermal_range_get_stream_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field camera_device_id from camera_thermal_range message
 *
 * @return  Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
 */
static inline uint8_t mavlink_msg_camera_thermal_range_get_camera_device_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field max from camera_thermal_range message
 *
 * @return [degC] Temperature max.
 */
static inline float mavlink_msg_camera_thermal_range_get_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field max_point_x from camera_thermal_range message
 *
 * @return  Temperature max point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
 */
static inline float mavlink_msg_camera_thermal_range_get_max_point_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field max_point_y from camera_thermal_range message
 *
 * @return  Temperature max point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
 */
static inline float mavlink_msg_camera_thermal_range_get_max_point_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field min from camera_thermal_range message
 *
 * @return [degC] Temperature min.
 */
static inline float mavlink_msg_camera_thermal_range_get_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field min_point_x from camera_thermal_range message
 *
 * @return  Temperature min point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
 */
static inline float mavlink_msg_camera_thermal_range_get_min_point_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field min_point_y from camera_thermal_range message
 *
 * @return  Temperature min point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
 */
static inline float mavlink_msg_camera_thermal_range_get_min_point_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a camera_thermal_range message into a struct
 *
 * @param msg The message to decode
 * @param camera_thermal_range C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_thermal_range_decode(const mavlink_message_t* msg, mavlink_camera_thermal_range_t* camera_thermal_range)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_thermal_range->time_boot_ms = mavlink_msg_camera_thermal_range_get_time_boot_ms(msg);
    camera_thermal_range->max = mavlink_msg_camera_thermal_range_get_max(msg);
    camera_thermal_range->max_point_x = mavlink_msg_camera_thermal_range_get_max_point_x(msg);
    camera_thermal_range->max_point_y = mavlink_msg_camera_thermal_range_get_max_point_y(msg);
    camera_thermal_range->min = mavlink_msg_camera_thermal_range_get_min(msg);
    camera_thermal_range->min_point_x = mavlink_msg_camera_thermal_range_get_min_point_x(msg);
    camera_thermal_range->min_point_y = mavlink_msg_camera_thermal_range_get_min_point_y(msg);
    camera_thermal_range->stream_id = mavlink_msg_camera_thermal_range_get_stream_id(msg);
    camera_thermal_range->camera_device_id = mavlink_msg_camera_thermal_range_get_camera_device_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN;
        memset(camera_thermal_range, 0, MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE_LEN);
    memcpy(camera_thermal_range, _MAV_PAYLOAD(msg), len);
#endif
}
