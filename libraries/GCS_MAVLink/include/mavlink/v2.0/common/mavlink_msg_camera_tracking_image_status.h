#pragma once
// MESSAGE CAMERA_TRACKING_IMAGE_STATUS PACKING

#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS 275


typedef struct __mavlink_camera_tracking_image_status_t {
 float point_x; /*<  Current tracked point x value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is left, 1 is right), NAN if unknown*/
 float point_y; /*<  Current tracked point y value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown*/
 float radius; /*<  Current tracked radius if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is image left, 1 is image right), NAN if unknown*/
 float rec_top_x; /*<  Current tracked rectangle top x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown*/
 float rec_top_y; /*<  Current tracked rectangle top y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown*/
 float rec_bottom_x; /*<  Current tracked rectangle bottom x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown*/
 float rec_bottom_y; /*<  Current tracked rectangle bottom y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown*/
 uint8_t tracking_status; /*<  Current tracking status*/
 uint8_t tracking_mode; /*<  Current tracking mode*/
 uint8_t target_data; /*<  Defines location of target data*/
} mavlink_camera_tracking_image_status_t;

#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN 31
#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN 31
#define MAVLINK_MSG_ID_275_LEN 31
#define MAVLINK_MSG_ID_275_MIN_LEN 31

#define MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC 126
#define MAVLINK_MSG_ID_275_CRC 126



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_TRACKING_IMAGE_STATUS { \
    275, \
    "CAMERA_TRACKING_IMAGE_STATUS", \
    10, \
    {  { "tracking_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_camera_tracking_image_status_t, tracking_status) }, \
         { "tracking_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_camera_tracking_image_status_t, tracking_mode) }, \
         { "target_data", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_camera_tracking_image_status_t, target_data) }, \
         { "point_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_camera_tracking_image_status_t, point_x) }, \
         { "point_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_camera_tracking_image_status_t, point_y) }, \
         { "radius", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_tracking_image_status_t, radius) }, \
         { "rec_top_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_tracking_image_status_t, rec_top_x) }, \
         { "rec_top_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_tracking_image_status_t, rec_top_y) }, \
         { "rec_bottom_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_tracking_image_status_t, rec_bottom_x) }, \
         { "rec_bottom_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_tracking_image_status_t, rec_bottom_y) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_TRACKING_IMAGE_STATUS { \
    "CAMERA_TRACKING_IMAGE_STATUS", \
    10, \
    {  { "tracking_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_camera_tracking_image_status_t, tracking_status) }, \
         { "tracking_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_camera_tracking_image_status_t, tracking_mode) }, \
         { "target_data", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_camera_tracking_image_status_t, target_data) }, \
         { "point_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_camera_tracking_image_status_t, point_x) }, \
         { "point_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_camera_tracking_image_status_t, point_y) }, \
         { "radius", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_tracking_image_status_t, radius) }, \
         { "rec_top_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_tracking_image_status_t, rec_top_x) }, \
         { "rec_top_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_tracking_image_status_t, rec_top_y) }, \
         { "rec_bottom_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_tracking_image_status_t, rec_bottom_x) }, \
         { "rec_bottom_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_tracking_image_status_t, rec_bottom_y) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_tracking_image_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tracking_status  Current tracking status
 * @param tracking_mode  Current tracking mode
 * @param target_data  Defines location of target data
 * @param point_x  Current tracked point x value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param point_y  Current tracked point y value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @param radius  Current tracked radius if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is image left, 1 is image right), NAN if unknown
 * @param rec_top_x  Current tracked rectangle top x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param rec_top_y  Current tracked rectangle top y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @param rec_bottom_x  Current tracked rectangle bottom x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param rec_bottom_y  Current tracked rectangle bottom y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_tracking_image_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN];
    _mav_put_float(buf, 0, point_x);
    _mav_put_float(buf, 4, point_y);
    _mav_put_float(buf, 8, radius);
    _mav_put_float(buf, 12, rec_top_x);
    _mav_put_float(buf, 16, rec_top_y);
    _mav_put_float(buf, 20, rec_bottom_x);
    _mav_put_float(buf, 24, rec_bottom_y);
    _mav_put_uint8_t(buf, 28, tracking_status);
    _mav_put_uint8_t(buf, 29, tracking_mode);
    _mav_put_uint8_t(buf, 30, target_data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN);
#else
    mavlink_camera_tracking_image_status_t packet;
    packet.point_x = point_x;
    packet.point_y = point_y;
    packet.radius = radius;
    packet.rec_top_x = rec_top_x;
    packet.rec_top_y = rec_top_y;
    packet.rec_bottom_x = rec_bottom_x;
    packet.rec_bottom_y = rec_bottom_y;
    packet.tracking_status = tracking_status;
    packet.tracking_mode = tracking_mode;
    packet.target_data = target_data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC);
}

/**
 * @brief Pack a camera_tracking_image_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param tracking_status  Current tracking status
 * @param tracking_mode  Current tracking mode
 * @param target_data  Defines location of target data
 * @param point_x  Current tracked point x value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param point_y  Current tracked point y value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @param radius  Current tracked radius if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is image left, 1 is image right), NAN if unknown
 * @param rec_top_x  Current tracked rectangle top x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param rec_top_y  Current tracked rectangle top y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @param rec_bottom_x  Current tracked rectangle bottom x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param rec_bottom_y  Current tracked rectangle bottom y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_tracking_image_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN];
    _mav_put_float(buf, 0, point_x);
    _mav_put_float(buf, 4, point_y);
    _mav_put_float(buf, 8, radius);
    _mav_put_float(buf, 12, rec_top_x);
    _mav_put_float(buf, 16, rec_top_y);
    _mav_put_float(buf, 20, rec_bottom_x);
    _mav_put_float(buf, 24, rec_bottom_y);
    _mav_put_uint8_t(buf, 28, tracking_status);
    _mav_put_uint8_t(buf, 29, tracking_mode);
    _mav_put_uint8_t(buf, 30, target_data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN);
#else
    mavlink_camera_tracking_image_status_t packet;
    packet.point_x = point_x;
    packet.point_y = point_y;
    packet.radius = radius;
    packet.rec_top_x = rec_top_x;
    packet.rec_top_y = rec_top_y;
    packet.rec_bottom_x = rec_bottom_x;
    packet.rec_bottom_y = rec_bottom_y;
    packet.tracking_status = tracking_status;
    packet.tracking_mode = tracking_mode;
    packet.target_data = target_data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a camera_tracking_image_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tracking_status  Current tracking status
 * @param tracking_mode  Current tracking mode
 * @param target_data  Defines location of target data
 * @param point_x  Current tracked point x value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param point_y  Current tracked point y value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @param radius  Current tracked radius if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is image left, 1 is image right), NAN if unknown
 * @param rec_top_x  Current tracked rectangle top x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param rec_top_y  Current tracked rectangle top y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @param rec_bottom_x  Current tracked rectangle bottom x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param rec_bottom_y  Current tracked rectangle bottom y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_tracking_image_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t tracking_status,uint8_t tracking_mode,uint8_t target_data,float point_x,float point_y,float radius,float rec_top_x,float rec_top_y,float rec_bottom_x,float rec_bottom_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN];
    _mav_put_float(buf, 0, point_x);
    _mav_put_float(buf, 4, point_y);
    _mav_put_float(buf, 8, radius);
    _mav_put_float(buf, 12, rec_top_x);
    _mav_put_float(buf, 16, rec_top_y);
    _mav_put_float(buf, 20, rec_bottom_x);
    _mav_put_float(buf, 24, rec_bottom_y);
    _mav_put_uint8_t(buf, 28, tracking_status);
    _mav_put_uint8_t(buf, 29, tracking_mode);
    _mav_put_uint8_t(buf, 30, target_data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN);
#else
    mavlink_camera_tracking_image_status_t packet;
    packet.point_x = point_x;
    packet.point_y = point_y;
    packet.radius = radius;
    packet.rec_top_x = rec_top_x;
    packet.rec_top_y = rec_top_y;
    packet.rec_bottom_x = rec_bottom_x;
    packet.rec_bottom_y = rec_bottom_y;
    packet.tracking_status = tracking_status;
    packet.tracking_mode = tracking_mode;
    packet.target_data = target_data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC);
}

/**
 * @brief Encode a camera_tracking_image_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_tracking_image_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_tracking_image_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_tracking_image_status_t* camera_tracking_image_status)
{
    return mavlink_msg_camera_tracking_image_status_pack(system_id, component_id, msg, camera_tracking_image_status->tracking_status, camera_tracking_image_status->tracking_mode, camera_tracking_image_status->target_data, camera_tracking_image_status->point_x, camera_tracking_image_status->point_y, camera_tracking_image_status->radius, camera_tracking_image_status->rec_top_x, camera_tracking_image_status->rec_top_y, camera_tracking_image_status->rec_bottom_x, camera_tracking_image_status->rec_bottom_y);
}

/**
 * @brief Encode a camera_tracking_image_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_tracking_image_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_tracking_image_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_tracking_image_status_t* camera_tracking_image_status)
{
    return mavlink_msg_camera_tracking_image_status_pack_chan(system_id, component_id, chan, msg, camera_tracking_image_status->tracking_status, camera_tracking_image_status->tracking_mode, camera_tracking_image_status->target_data, camera_tracking_image_status->point_x, camera_tracking_image_status->point_y, camera_tracking_image_status->radius, camera_tracking_image_status->rec_top_x, camera_tracking_image_status->rec_top_y, camera_tracking_image_status->rec_bottom_x, camera_tracking_image_status->rec_bottom_y);
}

/**
 * @brief Encode a camera_tracking_image_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param camera_tracking_image_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_tracking_image_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_camera_tracking_image_status_t* camera_tracking_image_status)
{
    return mavlink_msg_camera_tracking_image_status_pack_status(system_id, component_id, _status, msg,  camera_tracking_image_status->tracking_status, camera_tracking_image_status->tracking_mode, camera_tracking_image_status->target_data, camera_tracking_image_status->point_x, camera_tracking_image_status->point_y, camera_tracking_image_status->radius, camera_tracking_image_status->rec_top_x, camera_tracking_image_status->rec_top_y, camera_tracking_image_status->rec_bottom_x, camera_tracking_image_status->rec_bottom_y);
}

/**
 * @brief Send a camera_tracking_image_status message
 * @param chan MAVLink channel to send the message
 *
 * @param tracking_status  Current tracking status
 * @param tracking_mode  Current tracking mode
 * @param target_data  Defines location of target data
 * @param point_x  Current tracked point x value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param point_y  Current tracked point y value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @param radius  Current tracked radius if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is image left, 1 is image right), NAN if unknown
 * @param rec_top_x  Current tracked rectangle top x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param rec_top_y  Current tracked rectangle top y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 * @param rec_bottom_x  Current tracked rectangle bottom x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 * @param rec_bottom_y  Current tracked rectangle bottom y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_tracking_image_status_send(mavlink_channel_t chan, uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN];
    _mav_put_float(buf, 0, point_x);
    _mav_put_float(buf, 4, point_y);
    _mav_put_float(buf, 8, radius);
    _mav_put_float(buf, 12, rec_top_x);
    _mav_put_float(buf, 16, rec_top_y);
    _mav_put_float(buf, 20, rec_bottom_x);
    _mav_put_float(buf, 24, rec_bottom_y);
    _mav_put_uint8_t(buf, 28, tracking_status);
    _mav_put_uint8_t(buf, 29, tracking_mode);
    _mav_put_uint8_t(buf, 30, target_data);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS, buf, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC);
#else
    mavlink_camera_tracking_image_status_t packet;
    packet.point_x = point_x;
    packet.point_y = point_y;
    packet.radius = radius;
    packet.rec_top_x = rec_top_x;
    packet.rec_top_y = rec_top_y;
    packet.rec_bottom_x = rec_bottom_x;
    packet.rec_bottom_y = rec_bottom_y;
    packet.tracking_status = tracking_status;
    packet.tracking_mode = tracking_mode;
    packet.target_data = target_data;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC);
#endif
}

/**
 * @brief Send a camera_tracking_image_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_tracking_image_status_send_struct(mavlink_channel_t chan, const mavlink_camera_tracking_image_status_t* camera_tracking_image_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_tracking_image_status_send(chan, camera_tracking_image_status->tracking_status, camera_tracking_image_status->tracking_mode, camera_tracking_image_status->target_data, camera_tracking_image_status->point_x, camera_tracking_image_status->point_y, camera_tracking_image_status->radius, camera_tracking_image_status->rec_top_x, camera_tracking_image_status->rec_top_y, camera_tracking_image_status->rec_bottom_x, camera_tracking_image_status->rec_bottom_y);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS, (const char *)camera_tracking_image_status, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_tracking_image_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t tracking_status, uint8_t tracking_mode, uint8_t target_data, float point_x, float point_y, float radius, float rec_top_x, float rec_top_y, float rec_bottom_x, float rec_bottom_y)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, point_x);
    _mav_put_float(buf, 4, point_y);
    _mav_put_float(buf, 8, radius);
    _mav_put_float(buf, 12, rec_top_x);
    _mav_put_float(buf, 16, rec_top_y);
    _mav_put_float(buf, 20, rec_bottom_x);
    _mav_put_float(buf, 24, rec_bottom_y);
    _mav_put_uint8_t(buf, 28, tracking_status);
    _mav_put_uint8_t(buf, 29, tracking_mode);
    _mav_put_uint8_t(buf, 30, target_data);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS, buf, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC);
#else
    mavlink_camera_tracking_image_status_t *packet = (mavlink_camera_tracking_image_status_t *)msgbuf;
    packet->point_x = point_x;
    packet->point_y = point_y;
    packet->radius = radius;
    packet->rec_top_x = rec_top_x;
    packet->rec_top_y = rec_top_y;
    packet->rec_bottom_x = rec_bottom_x;
    packet->rec_bottom_y = rec_bottom_y;
    packet->tracking_status = tracking_status;
    packet->tracking_mode = tracking_mode;
    packet->target_data = target_data;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS, (const char *)packet, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_TRACKING_IMAGE_STATUS UNPACKING


/**
 * @brief Get field tracking_status from camera_tracking_image_status message
 *
 * @return  Current tracking status
 */
static inline uint8_t mavlink_msg_camera_tracking_image_status_get_tracking_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field tracking_mode from camera_tracking_image_status message
 *
 * @return  Current tracking mode
 */
static inline uint8_t mavlink_msg_camera_tracking_image_status_get_tracking_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field target_data from camera_tracking_image_status message
 *
 * @return  Defines location of target data
 */
static inline uint8_t mavlink_msg_camera_tracking_image_status_get_target_data(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field point_x from camera_tracking_image_status message
 *
 * @return  Current tracked point x value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_image_status_get_point_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field point_y from camera_tracking_image_status message
 *
 * @return  Current tracked point y value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_image_status_get_point_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field radius from camera_tracking_image_status message
 *
 * @return  Current tracked radius if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is image left, 1 is image right), NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_image_status_get_radius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rec_top_x from camera_tracking_image_status message
 *
 * @return  Current tracked rectangle top x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_image_status_get_rec_top_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field rec_top_y from camera_tracking_image_status message
 *
 * @return  Current tracked rectangle top y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_image_status_get_rec_top_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rec_bottom_x from camera_tracking_image_status message
 *
 * @return  Current tracked rectangle bottom x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_image_status_get_rec_bottom_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field rec_bottom_y from camera_tracking_image_status message
 *
 * @return  Current tracked rectangle bottom y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_image_status_get_rec_bottom_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a camera_tracking_image_status message into a struct
 *
 * @param msg The message to decode
 * @param camera_tracking_image_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_tracking_image_status_decode(const mavlink_message_t* msg, mavlink_camera_tracking_image_status_t* camera_tracking_image_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_tracking_image_status->point_x = mavlink_msg_camera_tracking_image_status_get_point_x(msg);
    camera_tracking_image_status->point_y = mavlink_msg_camera_tracking_image_status_get_point_y(msg);
    camera_tracking_image_status->radius = mavlink_msg_camera_tracking_image_status_get_radius(msg);
    camera_tracking_image_status->rec_top_x = mavlink_msg_camera_tracking_image_status_get_rec_top_x(msg);
    camera_tracking_image_status->rec_top_y = mavlink_msg_camera_tracking_image_status_get_rec_top_y(msg);
    camera_tracking_image_status->rec_bottom_x = mavlink_msg_camera_tracking_image_status_get_rec_bottom_x(msg);
    camera_tracking_image_status->rec_bottom_y = mavlink_msg_camera_tracking_image_status_get_rec_bottom_y(msg);
    camera_tracking_image_status->tracking_status = mavlink_msg_camera_tracking_image_status_get_tracking_status(msg);
    camera_tracking_image_status->tracking_mode = mavlink_msg_camera_tracking_image_status_get_tracking_mode(msg);
    camera_tracking_image_status->target_data = mavlink_msg_camera_tracking_image_status_get_target_data(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN;
        memset(camera_tracking_image_status, 0, MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS_LEN);
    memcpy(camera_tracking_image_status, _MAV_PAYLOAD(msg), len);
#endif
}
