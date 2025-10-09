#pragma once
// MESSAGE CAMERA_FOV_STATUS PACKING

#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS 271


typedef struct __mavlink_camera_fov_status_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 int32_t lat_camera; /*< [degE7] Latitude of camera (INT32_MAX if unknown).*/
 int32_t lon_camera; /*< [degE7] Longitude of camera (INT32_MAX if unknown).*/
 int32_t alt_camera; /*< [mm] Altitude (MSL) of camera (INT32_MAX if unknown).*/
 int32_t lat_image; /*< [degE7] Latitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).*/
 int32_t lon_image; /*< [degE7] Longitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).*/
 int32_t alt_image; /*< [mm] Altitude (MSL) of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).*/
 float q[4]; /*<  Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)*/
 float hfov; /*< [deg] Horizontal field of view (NaN if unknown).*/
 float vfov; /*< [deg] Vertical field of view (NaN if unknown).*/
} mavlink_camera_fov_status_t;

#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN 52
#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN 52
#define MAVLINK_MSG_ID_271_LEN 52
#define MAVLINK_MSG_ID_271_MIN_LEN 52

#define MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC 22
#define MAVLINK_MSG_ID_271_CRC 22

#define MAVLINK_MSG_CAMERA_FOV_STATUS_FIELD_Q_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_FOV_STATUS { \
    271, \
    "CAMERA_FOV_STATUS", \
    10, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_fov_status_t, time_boot_ms) }, \
         { "lat_camera", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_camera_fov_status_t, lat_camera) }, \
         { "lon_camera", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_camera_fov_status_t, lon_camera) }, \
         { "alt_camera", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_camera_fov_status_t, alt_camera) }, \
         { "lat_image", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_camera_fov_status_t, lat_image) }, \
         { "lon_image", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_camera_fov_status_t, lon_image) }, \
         { "alt_image", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_camera_fov_status_t, alt_image) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 28, offsetof(mavlink_camera_fov_status_t, q) }, \
         { "hfov", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_camera_fov_status_t, hfov) }, \
         { "vfov", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_camera_fov_status_t, vfov) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_FOV_STATUS { \
    "CAMERA_FOV_STATUS", \
    10, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_fov_status_t, time_boot_ms) }, \
         { "lat_camera", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_camera_fov_status_t, lat_camera) }, \
         { "lon_camera", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_camera_fov_status_t, lon_camera) }, \
         { "alt_camera", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_camera_fov_status_t, alt_camera) }, \
         { "lat_image", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_camera_fov_status_t, lat_image) }, \
         { "lon_image", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_camera_fov_status_t, lon_image) }, \
         { "alt_image", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_camera_fov_status_t, alt_image) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 28, offsetof(mavlink_camera_fov_status_t, q) }, \
         { "hfov", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_camera_fov_status_t, hfov) }, \
         { "vfov", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_camera_fov_status_t, vfov) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_fov_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param lat_camera [degE7] Latitude of camera (INT32_MAX if unknown).
 * @param lon_camera [degE7] Longitude of camera (INT32_MAX if unknown).
 * @param alt_camera [mm] Altitude (MSL) of camera (INT32_MAX if unknown).
 * @param lat_image [degE7] Latitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param lon_image [degE7] Longitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param alt_image [mm] Altitude (MSL) of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param q  Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param hfov [deg] Horizontal field of view (NaN if unknown).
 * @param vfov [deg] Vertical field of view (NaN if unknown).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_fov_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float *q, float hfov, float vfov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat_camera);
    _mav_put_int32_t(buf, 8, lon_camera);
    _mav_put_int32_t(buf, 12, alt_camera);
    _mav_put_int32_t(buf, 16, lat_image);
    _mav_put_int32_t(buf, 20, lon_image);
    _mav_put_int32_t(buf, 24, alt_image);
    _mav_put_float(buf, 44, hfov);
    _mav_put_float(buf, 48, vfov);
    _mav_put_float_array(buf, 28, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN);
#else
    mavlink_camera_fov_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat_camera = lat_camera;
    packet.lon_camera = lon_camera;
    packet.alt_camera = alt_camera;
    packet.lat_image = lat_image;
    packet.lon_image = lon_image;
    packet.alt_image = alt_image;
    packet.hfov = hfov;
    packet.vfov = vfov;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_FOV_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC);
}

/**
 * @brief Pack a camera_fov_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param lat_camera [degE7] Latitude of camera (INT32_MAX if unknown).
 * @param lon_camera [degE7] Longitude of camera (INT32_MAX if unknown).
 * @param alt_camera [mm] Altitude (MSL) of camera (INT32_MAX if unknown).
 * @param lat_image [degE7] Latitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param lon_image [degE7] Longitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param alt_image [mm] Altitude (MSL) of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param q  Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param hfov [deg] Horizontal field of view (NaN if unknown).
 * @param vfov [deg] Vertical field of view (NaN if unknown).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_fov_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float *q, float hfov, float vfov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat_camera);
    _mav_put_int32_t(buf, 8, lon_camera);
    _mav_put_int32_t(buf, 12, alt_camera);
    _mav_put_int32_t(buf, 16, lat_image);
    _mav_put_int32_t(buf, 20, lon_image);
    _mav_put_int32_t(buf, 24, alt_image);
    _mav_put_float(buf, 44, hfov);
    _mav_put_float(buf, 48, vfov);
    _mav_put_float_array(buf, 28, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN);
#else
    mavlink_camera_fov_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat_camera = lat_camera;
    packet.lon_camera = lon_camera;
    packet.alt_camera = alt_camera;
    packet.lat_image = lat_image;
    packet.lon_image = lon_image;
    packet.alt_image = alt_image;
    packet.hfov = hfov;
    packet.vfov = vfov;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_FOV_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN);
#endif
}

/**
 * @brief Pack a camera_fov_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param lat_camera [degE7] Latitude of camera (INT32_MAX if unknown).
 * @param lon_camera [degE7] Longitude of camera (INT32_MAX if unknown).
 * @param alt_camera [mm] Altitude (MSL) of camera (INT32_MAX if unknown).
 * @param lat_image [degE7] Latitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param lon_image [degE7] Longitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param alt_image [mm] Altitude (MSL) of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param q  Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param hfov [deg] Horizontal field of view (NaN if unknown).
 * @param vfov [deg] Vertical field of view (NaN if unknown).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_fov_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,int32_t lat_camera,int32_t lon_camera,int32_t alt_camera,int32_t lat_image,int32_t lon_image,int32_t alt_image,const float *q,float hfov,float vfov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat_camera);
    _mav_put_int32_t(buf, 8, lon_camera);
    _mav_put_int32_t(buf, 12, alt_camera);
    _mav_put_int32_t(buf, 16, lat_image);
    _mav_put_int32_t(buf, 20, lon_image);
    _mav_put_int32_t(buf, 24, alt_image);
    _mav_put_float(buf, 44, hfov);
    _mav_put_float(buf, 48, vfov);
    _mav_put_float_array(buf, 28, q, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN);
#else
    mavlink_camera_fov_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat_camera = lat_camera;
    packet.lon_camera = lon_camera;
    packet.alt_camera = alt_camera;
    packet.lat_image = lat_image;
    packet.lon_image = lon_image;
    packet.alt_image = alt_image;
    packet.hfov = hfov;
    packet.vfov = vfov;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_FOV_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC);
}

/**
 * @brief Encode a camera_fov_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_fov_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_fov_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_fov_status_t* camera_fov_status)
{
    return mavlink_msg_camera_fov_status_pack(system_id, component_id, msg, camera_fov_status->time_boot_ms, camera_fov_status->lat_camera, camera_fov_status->lon_camera, camera_fov_status->alt_camera, camera_fov_status->lat_image, camera_fov_status->lon_image, camera_fov_status->alt_image, camera_fov_status->q, camera_fov_status->hfov, camera_fov_status->vfov);
}

/**
 * @brief Encode a camera_fov_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_fov_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_fov_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_fov_status_t* camera_fov_status)
{
    return mavlink_msg_camera_fov_status_pack_chan(system_id, component_id, chan, msg, camera_fov_status->time_boot_ms, camera_fov_status->lat_camera, camera_fov_status->lon_camera, camera_fov_status->alt_camera, camera_fov_status->lat_image, camera_fov_status->lon_image, camera_fov_status->alt_image, camera_fov_status->q, camera_fov_status->hfov, camera_fov_status->vfov);
}

/**
 * @brief Encode a camera_fov_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param camera_fov_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_fov_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_camera_fov_status_t* camera_fov_status)
{
    return mavlink_msg_camera_fov_status_pack_status(system_id, component_id, _status, msg,  camera_fov_status->time_boot_ms, camera_fov_status->lat_camera, camera_fov_status->lon_camera, camera_fov_status->alt_camera, camera_fov_status->lat_image, camera_fov_status->lon_image, camera_fov_status->alt_image, camera_fov_status->q, camera_fov_status->hfov, camera_fov_status->vfov);
}

/**
 * @brief Send a camera_fov_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param lat_camera [degE7] Latitude of camera (INT32_MAX if unknown).
 * @param lon_camera [degE7] Longitude of camera (INT32_MAX if unknown).
 * @param alt_camera [mm] Altitude (MSL) of camera (INT32_MAX if unknown).
 * @param lat_image [degE7] Latitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param lon_image [degE7] Longitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param alt_image [mm] Altitude (MSL) of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 * @param q  Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 * @param hfov [deg] Horizontal field of view (NaN if unknown).
 * @param vfov [deg] Vertical field of view (NaN if unknown).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_fov_status_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float *q, float hfov, float vfov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat_camera);
    _mav_put_int32_t(buf, 8, lon_camera);
    _mav_put_int32_t(buf, 12, alt_camera);
    _mav_put_int32_t(buf, 16, lat_image);
    _mav_put_int32_t(buf, 20, lon_image);
    _mav_put_int32_t(buf, 24, alt_image);
    _mav_put_float(buf, 44, hfov);
    _mav_put_float(buf, 48, vfov);
    _mav_put_float_array(buf, 28, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FOV_STATUS, buf, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC);
#else
    mavlink_camera_fov_status_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat_camera = lat_camera;
    packet.lon_camera = lon_camera;
    packet.alt_camera = alt_camera;
    packet.lat_image = lat_image;
    packet.lon_image = lon_image;
    packet.alt_image = alt_image;
    packet.hfov = hfov;
    packet.vfov = vfov;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FOV_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC);
#endif
}

/**
 * @brief Send a camera_fov_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_fov_status_send_struct(mavlink_channel_t chan, const mavlink_camera_fov_status_t* camera_fov_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_fov_status_send(chan, camera_fov_status->time_boot_ms, camera_fov_status->lat_camera, camera_fov_status->lon_camera, camera_fov_status->alt_camera, camera_fov_status->lat_image, camera_fov_status->lon_image, camera_fov_status->alt_image, camera_fov_status->q, camera_fov_status->hfov, camera_fov_status->vfov);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FOV_STATUS, (const char *)camera_fov_status, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_fov_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int32_t lat_camera, int32_t lon_camera, int32_t alt_camera, int32_t lat_image, int32_t lon_image, int32_t alt_image, const float *q, float hfov, float vfov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat_camera);
    _mav_put_int32_t(buf, 8, lon_camera);
    _mav_put_int32_t(buf, 12, alt_camera);
    _mav_put_int32_t(buf, 16, lat_image);
    _mav_put_int32_t(buf, 20, lon_image);
    _mav_put_int32_t(buf, 24, alt_image);
    _mav_put_float(buf, 44, hfov);
    _mav_put_float(buf, 48, vfov);
    _mav_put_float_array(buf, 28, q, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FOV_STATUS, buf, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC);
#else
    mavlink_camera_fov_status_t *packet = (mavlink_camera_fov_status_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->lat_camera = lat_camera;
    packet->lon_camera = lon_camera;
    packet->alt_camera = alt_camera;
    packet->lat_image = lat_image;
    packet->lon_image = lon_image;
    packet->alt_image = alt_image;
    packet->hfov = hfov;
    packet->vfov = vfov;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FOV_STATUS, (const char *)packet, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_FOV_STATUS UNPACKING


/**
 * @brief Get field time_boot_ms from camera_fov_status message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_camera_fov_status_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat_camera from camera_fov_status message
 *
 * @return [degE7] Latitude of camera (INT32_MAX if unknown).
 */
static inline int32_t mavlink_msg_camera_fov_status_get_lat_camera(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon_camera from camera_fov_status message
 *
 * @return [degE7] Longitude of camera (INT32_MAX if unknown).
 */
static inline int32_t mavlink_msg_camera_fov_status_get_lon_camera(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt_camera from camera_fov_status message
 *
 * @return [mm] Altitude (MSL) of camera (INT32_MAX if unknown).
 */
static inline int32_t mavlink_msg_camera_fov_status_get_alt_camera(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field lat_image from camera_fov_status message
 *
 * @return [degE7] Latitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 */
static inline int32_t mavlink_msg_camera_fov_status_get_lat_image(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field lon_image from camera_fov_status message
 *
 * @return [degE7] Longitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 */
static inline int32_t mavlink_msg_camera_fov_status_get_lon_image(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field alt_image from camera_fov_status message
 *
 * @return [mm] Altitude (MSL) of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
 */
static inline int32_t mavlink_msg_camera_fov_status_get_alt_image(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field q from camera_fov_status message
 *
 * @return  Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 */
static inline uint16_t mavlink_msg_camera_fov_status_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  28);
}

/**
 * @brief Get field hfov from camera_fov_status message
 *
 * @return [deg] Horizontal field of view (NaN if unknown).
 */
static inline float mavlink_msg_camera_fov_status_get_hfov(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field vfov from camera_fov_status message
 *
 * @return [deg] Vertical field of view (NaN if unknown).
 */
static inline float mavlink_msg_camera_fov_status_get_vfov(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Decode a camera_fov_status message into a struct
 *
 * @param msg The message to decode
 * @param camera_fov_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_fov_status_decode(const mavlink_message_t* msg, mavlink_camera_fov_status_t* camera_fov_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_fov_status->time_boot_ms = mavlink_msg_camera_fov_status_get_time_boot_ms(msg);
    camera_fov_status->lat_camera = mavlink_msg_camera_fov_status_get_lat_camera(msg);
    camera_fov_status->lon_camera = mavlink_msg_camera_fov_status_get_lon_camera(msg);
    camera_fov_status->alt_camera = mavlink_msg_camera_fov_status_get_alt_camera(msg);
    camera_fov_status->lat_image = mavlink_msg_camera_fov_status_get_lat_image(msg);
    camera_fov_status->lon_image = mavlink_msg_camera_fov_status_get_lon_image(msg);
    camera_fov_status->alt_image = mavlink_msg_camera_fov_status_get_alt_image(msg);
    mavlink_msg_camera_fov_status_get_q(msg, camera_fov_status->q);
    camera_fov_status->hfov = mavlink_msg_camera_fov_status_get_hfov(msg);
    camera_fov_status->vfov = mavlink_msg_camera_fov_status_get_vfov(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN;
        memset(camera_fov_status, 0, MAVLINK_MSG_ID_CAMERA_FOV_STATUS_LEN);
    memcpy(camera_fov_status, _MAV_PAYLOAD(msg), len);
#endif
}
