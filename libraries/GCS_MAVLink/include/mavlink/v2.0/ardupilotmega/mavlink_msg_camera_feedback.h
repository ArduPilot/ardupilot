#pragma once
// MESSAGE CAMERA_FEEDBACK PACKING

#define MAVLINK_MSG_ID_CAMERA_FEEDBACK 180

MAVPACKED(
typedef struct __mavlink_camera_feedback_t {
 uint64_t time_usec; /*< [us] Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).*/
 int32_t lat; /*< [degE7] Latitude.*/
 int32_t lng; /*< [degE7] Longitude.*/
 float alt_msl; /*< [m] Altitude (MSL).*/
 float alt_rel; /*< [m] Altitude (Relative to HOME location).*/
 float roll; /*< [deg] Camera Roll angle (earth frame, +-180).*/
 float pitch; /*< [deg] Camera Pitch angle (earth frame, +-180).*/
 float yaw; /*< [deg] Camera Yaw (earth frame, 0-360, true).*/
 float foc_len; /*< [mm] Focal Length.*/
 uint16_t img_idx; /*<  Image index.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t cam_idx; /*<  Camera ID.*/
 uint8_t flags; /*<  Feedback flags.*/
 uint16_t completed_captures; /*<  Completed image captures.*/
}) mavlink_camera_feedback_t;

#define MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN 47
#define MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN 45
#define MAVLINK_MSG_ID_180_LEN 47
#define MAVLINK_MSG_ID_180_MIN_LEN 45

#define MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC 52
#define MAVLINK_MSG_ID_180_CRC 52



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_FEEDBACK { \
    180, \
    "CAMERA_FEEDBACK", \
    14, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_camera_feedback_t, time_usec) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_camera_feedback_t, target_system) }, \
         { "cam_idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_camera_feedback_t, cam_idx) }, \
         { "img_idx", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_camera_feedback_t, img_idx) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_camera_feedback_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_camera_feedback_t, lng) }, \
         { "alt_msl", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_feedback_t, alt_msl) }, \
         { "alt_rel", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_feedback_t, alt_rel) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_feedback_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_camera_feedback_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_camera_feedback_t, yaw) }, \
         { "foc_len", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_camera_feedback_t, foc_len) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_camera_feedback_t, flags) }, \
         { "completed_captures", NULL, MAVLINK_TYPE_UINT16_T, 0, 45, offsetof(mavlink_camera_feedback_t, completed_captures) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_FEEDBACK { \
    "CAMERA_FEEDBACK", \
    14, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_camera_feedback_t, time_usec) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_camera_feedback_t, target_system) }, \
         { "cam_idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_camera_feedback_t, cam_idx) }, \
         { "img_idx", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_camera_feedback_t, img_idx) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_camera_feedback_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_camera_feedback_t, lng) }, \
         { "alt_msl", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_feedback_t, alt_msl) }, \
         { "alt_rel", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_feedback_t, alt_rel) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_feedback_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_camera_feedback_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_camera_feedback_t, yaw) }, \
         { "foc_len", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_camera_feedback_t, foc_len) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_camera_feedback_t, flags) }, \
         { "completed_captures", NULL, MAVLINK_TYPE_UINT16_T, 0, 45, offsetof(mavlink_camera_feedback_t, completed_captures) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_feedback message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).
 * @param target_system  System ID.
 * @param cam_idx  Camera ID.
 * @param img_idx  Image index.
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 * @param alt_msl [m] Altitude (MSL).
 * @param alt_rel [m] Altitude (Relative to HOME location).
 * @param roll [deg] Camera Roll angle (earth frame, +-180).
 * @param pitch [deg] Camera Pitch angle (earth frame, +-180).
 * @param yaw [deg] Camera Yaw (earth frame, 0-360, true).
 * @param foc_len [mm] Focal Length.
 * @param flags  Feedback flags.
 * @param completed_captures  Completed image captures.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_feedback_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lng);
    _mav_put_float(buf, 16, alt_msl);
    _mav_put_float(buf, 20, alt_rel);
    _mav_put_float(buf, 24, roll);
    _mav_put_float(buf, 28, pitch);
    _mav_put_float(buf, 32, yaw);
    _mav_put_float(buf, 36, foc_len);
    _mav_put_uint16_t(buf, 40, img_idx);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, cam_idx);
    _mav_put_uint8_t(buf, 44, flags);
    _mav_put_uint16_t(buf, 45, completed_captures);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#else
    mavlink_camera_feedback_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt_msl = alt_msl;
    packet.alt_rel = alt_rel;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.foc_len = foc_len;
    packet.img_idx = img_idx;
    packet.target_system = target_system;
    packet.cam_idx = cam_idx;
    packet.flags = flags;
    packet.completed_captures = completed_captures;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_FEEDBACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
}

/**
 * @brief Pack a camera_feedback message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).
 * @param target_system  System ID.
 * @param cam_idx  Camera ID.
 * @param img_idx  Image index.
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 * @param alt_msl [m] Altitude (MSL).
 * @param alt_rel [m] Altitude (Relative to HOME location).
 * @param roll [deg] Camera Roll angle (earth frame, +-180).
 * @param pitch [deg] Camera Pitch angle (earth frame, +-180).
 * @param yaw [deg] Camera Yaw (earth frame, 0-360, true).
 * @param foc_len [mm] Focal Length.
 * @param flags  Feedback flags.
 * @param completed_captures  Completed image captures.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_feedback_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lng);
    _mav_put_float(buf, 16, alt_msl);
    _mav_put_float(buf, 20, alt_rel);
    _mav_put_float(buf, 24, roll);
    _mav_put_float(buf, 28, pitch);
    _mav_put_float(buf, 32, yaw);
    _mav_put_float(buf, 36, foc_len);
    _mav_put_uint16_t(buf, 40, img_idx);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, cam_idx);
    _mav_put_uint8_t(buf, 44, flags);
    _mav_put_uint16_t(buf, 45, completed_captures);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#else
    mavlink_camera_feedback_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt_msl = alt_msl;
    packet.alt_rel = alt_rel;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.foc_len = foc_len;
    packet.img_idx = img_idx;
    packet.target_system = target_system;
    packet.cam_idx = cam_idx;
    packet.flags = flags;
    packet.completed_captures = completed_captures;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_FEEDBACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif
}

/**
 * @brief Pack a camera_feedback message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).
 * @param target_system  System ID.
 * @param cam_idx  Camera ID.
 * @param img_idx  Image index.
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 * @param alt_msl [m] Altitude (MSL).
 * @param alt_rel [m] Altitude (Relative to HOME location).
 * @param roll [deg] Camera Roll angle (earth frame, +-180).
 * @param pitch [deg] Camera Pitch angle (earth frame, +-180).
 * @param yaw [deg] Camera Yaw (earth frame, 0-360, true).
 * @param foc_len [mm] Focal Length.
 * @param flags  Feedback flags.
 * @param completed_captures  Completed image captures.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_feedback_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t target_system,uint8_t cam_idx,uint16_t img_idx,int32_t lat,int32_t lng,float alt_msl,float alt_rel,float roll,float pitch,float yaw,float foc_len,uint8_t flags,uint16_t completed_captures)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lng);
    _mav_put_float(buf, 16, alt_msl);
    _mav_put_float(buf, 20, alt_rel);
    _mav_put_float(buf, 24, roll);
    _mav_put_float(buf, 28, pitch);
    _mav_put_float(buf, 32, yaw);
    _mav_put_float(buf, 36, foc_len);
    _mav_put_uint16_t(buf, 40, img_idx);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, cam_idx);
    _mav_put_uint8_t(buf, 44, flags);
    _mav_put_uint16_t(buf, 45, completed_captures);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#else
    mavlink_camera_feedback_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt_msl = alt_msl;
    packet.alt_rel = alt_rel;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.foc_len = foc_len;
    packet.img_idx = img_idx;
    packet.target_system = target_system;
    packet.cam_idx = cam_idx;
    packet.flags = flags;
    packet.completed_captures = completed_captures;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_FEEDBACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
}

/**
 * @brief Encode a camera_feedback struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_feedback_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_feedback_t* camera_feedback)
{
    return mavlink_msg_camera_feedback_pack(system_id, component_id, msg, camera_feedback->time_usec, camera_feedback->target_system, camera_feedback->cam_idx, camera_feedback->img_idx, camera_feedback->lat, camera_feedback->lng, camera_feedback->alt_msl, camera_feedback->alt_rel, camera_feedback->roll, camera_feedback->pitch, camera_feedback->yaw, camera_feedback->foc_len, camera_feedback->flags, camera_feedback->completed_captures);
}

/**
 * @brief Encode a camera_feedback struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_feedback_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_feedback_t* camera_feedback)
{
    return mavlink_msg_camera_feedback_pack_chan(system_id, component_id, chan, msg, camera_feedback->time_usec, camera_feedback->target_system, camera_feedback->cam_idx, camera_feedback->img_idx, camera_feedback->lat, camera_feedback->lng, camera_feedback->alt_msl, camera_feedback->alt_rel, camera_feedback->roll, camera_feedback->pitch, camera_feedback->yaw, camera_feedback->foc_len, camera_feedback->flags, camera_feedback->completed_captures);
}

/**
 * @brief Encode a camera_feedback struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param camera_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_feedback_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_camera_feedback_t* camera_feedback)
{
    return mavlink_msg_camera_feedback_pack_status(system_id, component_id, _status, msg,  camera_feedback->time_usec, camera_feedback->target_system, camera_feedback->cam_idx, camera_feedback->img_idx, camera_feedback->lat, camera_feedback->lng, camera_feedback->alt_msl, camera_feedback->alt_rel, camera_feedback->roll, camera_feedback->pitch, camera_feedback->yaw, camera_feedback->foc_len, camera_feedback->flags, camera_feedback->completed_captures);
}

/**
 * @brief Send a camera_feedback message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).
 * @param target_system  System ID.
 * @param cam_idx  Camera ID.
 * @param img_idx  Image index.
 * @param lat [degE7] Latitude.
 * @param lng [degE7] Longitude.
 * @param alt_msl [m] Altitude (MSL).
 * @param alt_rel [m] Altitude (Relative to HOME location).
 * @param roll [deg] Camera Roll angle (earth frame, +-180).
 * @param pitch [deg] Camera Pitch angle (earth frame, +-180).
 * @param yaw [deg] Camera Yaw (earth frame, 0-360, true).
 * @param foc_len [mm] Focal Length.
 * @param flags  Feedback flags.
 * @param completed_captures  Completed image captures.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_feedback_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lng);
    _mav_put_float(buf, 16, alt_msl);
    _mav_put_float(buf, 20, alt_rel);
    _mav_put_float(buf, 24, roll);
    _mav_put_float(buf, 28, pitch);
    _mav_put_float(buf, 32, yaw);
    _mav_put_float(buf, 36, foc_len);
    _mav_put_uint16_t(buf, 40, img_idx);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, cam_idx);
    _mav_put_uint8_t(buf, 44, flags);
    _mav_put_uint16_t(buf, 45, completed_captures);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#else
    mavlink_camera_feedback_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lng = lng;
    packet.alt_msl = alt_msl;
    packet.alt_rel = alt_rel;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.foc_len = foc_len;
    packet.img_idx = img_idx;
    packet.target_system = target_system;
    packet.cam_idx = cam_idx;
    packet.flags = flags;
    packet.completed_captures = completed_captures;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#endif
}

/**
 * @brief Send a camera_feedback message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_feedback_send_struct(mavlink_channel_t chan, const mavlink_camera_feedback_t* camera_feedback)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_feedback_send(chan, camera_feedback->time_usec, camera_feedback->target_system, camera_feedback->cam_idx, camera_feedback->img_idx, camera_feedback->lat, camera_feedback->lng, camera_feedback->alt_msl, camera_feedback->alt_rel, camera_feedback->roll, camera_feedback->pitch, camera_feedback->yaw, camera_feedback->foc_len, camera_feedback->flags, camera_feedback->completed_captures);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, (const char *)camera_feedback, MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_feedback_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, int32_t lat, int32_t lng, float alt_msl, float alt_rel, float roll, float pitch, float yaw, float foc_len, uint8_t flags, uint16_t completed_captures)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lng);
    _mav_put_float(buf, 16, alt_msl);
    _mav_put_float(buf, 20, alt_rel);
    _mav_put_float(buf, 24, roll);
    _mav_put_float(buf, 28, pitch);
    _mav_put_float(buf, 32, yaw);
    _mav_put_float(buf, 36, foc_len);
    _mav_put_uint16_t(buf, 40, img_idx);
    _mav_put_uint8_t(buf, 42, target_system);
    _mav_put_uint8_t(buf, 43, cam_idx);
    _mav_put_uint8_t(buf, 44, flags);
    _mav_put_uint16_t(buf, 45, completed_captures);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, buf, MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#else
    mavlink_camera_feedback_t *packet = (mavlink_camera_feedback_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->lat = lat;
    packet->lng = lng;
    packet->alt_msl = alt_msl;
    packet->alt_rel = alt_rel;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->foc_len = foc_len;
    packet->img_idx = img_idx;
    packet->target_system = target_system;
    packet->cam_idx = cam_idx;
    packet->flags = flags;
    packet->completed_captures = completed_captures;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_FEEDBACK, (const char *)packet, MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN, MAVLINK_MSG_ID_CAMERA_FEEDBACK_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_FEEDBACK UNPACKING


/**
 * @brief Get field time_usec from camera_feedback message
 *
 * @return [us] Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).
 */
static inline uint64_t mavlink_msg_camera_feedback_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_system from camera_feedback message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_camera_feedback_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field cam_idx from camera_feedback message
 *
 * @return  Camera ID.
 */
static inline uint8_t mavlink_msg_camera_feedback_get_cam_idx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field img_idx from camera_feedback message
 *
 * @return  Image index.
 */
static inline uint16_t mavlink_msg_camera_feedback_get_img_idx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field lat from camera_feedback message
 *
 * @return [degE7] Latitude.
 */
static inline int32_t mavlink_msg_camera_feedback_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lng from camera_feedback message
 *
 * @return [degE7] Longitude.
 */
static inline int32_t mavlink_msg_camera_feedback_get_lng(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt_msl from camera_feedback message
 *
 * @return [m] Altitude (MSL).
 */
static inline float mavlink_msg_camera_feedback_get_alt_msl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field alt_rel from camera_feedback message
 *
 * @return [m] Altitude (Relative to HOME location).
 */
static inline float mavlink_msg_camera_feedback_get_alt_rel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field roll from camera_feedback message
 *
 * @return [deg] Camera Roll angle (earth frame, +-180).
 */
static inline float mavlink_msg_camera_feedback_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pitch from camera_feedback message
 *
 * @return [deg] Camera Pitch angle (earth frame, +-180).
 */
static inline float mavlink_msg_camera_feedback_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field yaw from camera_feedback message
 *
 * @return [deg] Camera Yaw (earth frame, 0-360, true).
 */
static inline float mavlink_msg_camera_feedback_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field foc_len from camera_feedback message
 *
 * @return [mm] Focal Length.
 */
static inline float mavlink_msg_camera_feedback_get_foc_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field flags from camera_feedback message
 *
 * @return  Feedback flags.
 */
static inline uint8_t mavlink_msg_camera_feedback_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field completed_captures from camera_feedback message
 *
 * @return  Completed image captures.
 */
static inline uint16_t mavlink_msg_camera_feedback_get_completed_captures(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  45);
}

/**
 * @brief Decode a camera_feedback message into a struct
 *
 * @param msg The message to decode
 * @param camera_feedback C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_feedback_decode(const mavlink_message_t* msg, mavlink_camera_feedback_t* camera_feedback)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_feedback->time_usec = mavlink_msg_camera_feedback_get_time_usec(msg);
    camera_feedback->lat = mavlink_msg_camera_feedback_get_lat(msg);
    camera_feedback->lng = mavlink_msg_camera_feedback_get_lng(msg);
    camera_feedback->alt_msl = mavlink_msg_camera_feedback_get_alt_msl(msg);
    camera_feedback->alt_rel = mavlink_msg_camera_feedback_get_alt_rel(msg);
    camera_feedback->roll = mavlink_msg_camera_feedback_get_roll(msg);
    camera_feedback->pitch = mavlink_msg_camera_feedback_get_pitch(msg);
    camera_feedback->yaw = mavlink_msg_camera_feedback_get_yaw(msg);
    camera_feedback->foc_len = mavlink_msg_camera_feedback_get_foc_len(msg);
    camera_feedback->img_idx = mavlink_msg_camera_feedback_get_img_idx(msg);
    camera_feedback->target_system = mavlink_msg_camera_feedback_get_target_system(msg);
    camera_feedback->cam_idx = mavlink_msg_camera_feedback_get_cam_idx(msg);
    camera_feedback->flags = mavlink_msg_camera_feedback_get_flags(msg);
    camera_feedback->completed_captures = mavlink_msg_camera_feedback_get_completed_captures(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN;
        memset(camera_feedback, 0, MAVLINK_MSG_ID_CAMERA_FEEDBACK_LEN);
    memcpy(camera_feedback, _MAV_PAYLOAD(msg), len);
#endif
}
