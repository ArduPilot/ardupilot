#pragma once
// MESSAGE HERELINK_VIDEO_STREAM_INFORMATION PACKING

#define MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION 50002


typedef struct __mavlink_herelink_video_stream_information_t {
 float framerate; /*< [Hz] Frame rate.*/
 uint32_t bitrate; /*< [bits/s] Bit rate.*/
 uint16_t resolution_h; /*< [pix] Horizontal resolution.*/
 uint16_t resolution_v; /*< [pix] Vertical resolution.*/
 uint16_t rotation; /*< [deg] Video image rotation clockwise.*/
 uint8_t camera_id; /*<  Video Stream ID (1 for first, 2 for second, etc.)*/
 uint8_t status; /*<  Number of streams available.*/
 char uri[230]; /*<  Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station should listen to).*/
} mavlink_herelink_video_stream_information_t;

#define MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN 246
#define MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN 246
#define MAVLINK_MSG_ID_50002_LEN 246
#define MAVLINK_MSG_ID_50002_MIN_LEN 246

#define MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_CRC 181
#define MAVLINK_MSG_ID_50002_CRC 181

#define MAVLINK_MSG_HERELINK_VIDEO_STREAM_INFORMATION_FIELD_URI_LEN 230

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HERELINK_VIDEO_STREAM_INFORMATION { \
    50002, \
    "HERELINK_VIDEO_STREAM_INFORMATION", \
    8, \
    {  { "camera_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_herelink_video_stream_information_t, camera_id) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_herelink_video_stream_information_t, status) }, \
         { "framerate", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_herelink_video_stream_information_t, framerate) }, \
         { "resolution_h", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_herelink_video_stream_information_t, resolution_h) }, \
         { "resolution_v", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_herelink_video_stream_information_t, resolution_v) }, \
         { "bitrate", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_herelink_video_stream_information_t, bitrate) }, \
         { "rotation", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_herelink_video_stream_information_t, rotation) }, \
         { "uri", NULL, MAVLINK_TYPE_CHAR, 230, 16, offsetof(mavlink_herelink_video_stream_information_t, uri) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HERELINK_VIDEO_STREAM_INFORMATION { \
    "HERELINK_VIDEO_STREAM_INFORMATION", \
    8, \
    {  { "camera_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_herelink_video_stream_information_t, camera_id) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_herelink_video_stream_information_t, status) }, \
         { "framerate", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_herelink_video_stream_information_t, framerate) }, \
         { "resolution_h", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_herelink_video_stream_information_t, resolution_h) }, \
         { "resolution_v", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_herelink_video_stream_information_t, resolution_v) }, \
         { "bitrate", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_herelink_video_stream_information_t, bitrate) }, \
         { "rotation", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_herelink_video_stream_information_t, rotation) }, \
         { "uri", NULL, MAVLINK_TYPE_CHAR, 230, 16, offsetof(mavlink_herelink_video_stream_information_t, uri) }, \
         } \
}
#endif

/**
 * @brief Pack a herelink_video_stream_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param camera_id  Video Stream ID (1 for first, 2 for second, etc.)
 * @param status  Number of streams available.
 * @param framerate [Hz] Frame rate.
 * @param resolution_h [pix] Horizontal resolution.
 * @param resolution_v [pix] Vertical resolution.
 * @param bitrate [bits/s] Bit rate.
 * @param rotation [deg] Video image rotation clockwise.
 * @param uri  Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station should listen to).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_herelink_video_stream_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t camera_id, uint8_t status, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, const char *uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN];
    _mav_put_float(buf, 0, framerate);
    _mav_put_uint32_t(buf, 4, bitrate);
    _mav_put_uint16_t(buf, 8, resolution_h);
    _mav_put_uint16_t(buf, 10, resolution_v);
    _mav_put_uint16_t(buf, 12, rotation);
    _mav_put_uint8_t(buf, 14, camera_id);
    _mav_put_uint8_t(buf, 15, status);
    _mav_put_char_array(buf, 16, uri, 230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN);
#else
    mavlink_herelink_video_stream_information_t packet;
    packet.framerate = framerate;
    packet.bitrate = bitrate;
    packet.resolution_h = resolution_h;
    packet.resolution_v = resolution_v;
    packet.rotation = rotation;
    packet.camera_id = camera_id;
    packet.status = status;
    mav_array_memcpy(packet.uri, uri, sizeof(char)*230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_CRC);
}

/**
 * @brief Pack a herelink_video_stream_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param camera_id  Video Stream ID (1 for first, 2 for second, etc.)
 * @param status  Number of streams available.
 * @param framerate [Hz] Frame rate.
 * @param resolution_h [pix] Horizontal resolution.
 * @param resolution_v [pix] Vertical resolution.
 * @param bitrate [bits/s] Bit rate.
 * @param rotation [deg] Video image rotation clockwise.
 * @param uri  Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station should listen to).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_herelink_video_stream_information_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t camera_id, uint8_t status, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, const char *uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN];
    _mav_put_float(buf, 0, framerate);
    _mav_put_uint32_t(buf, 4, bitrate);
    _mav_put_uint16_t(buf, 8, resolution_h);
    _mav_put_uint16_t(buf, 10, resolution_v);
    _mav_put_uint16_t(buf, 12, rotation);
    _mav_put_uint8_t(buf, 14, camera_id);
    _mav_put_uint8_t(buf, 15, status);
    _mav_put_char_array(buf, 16, uri, 230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN);
#else
    mavlink_herelink_video_stream_information_t packet;
    packet.framerate = framerate;
    packet.bitrate = bitrate;
    packet.resolution_h = resolution_h;
    packet.resolution_v = resolution_v;
    packet.rotation = rotation;
    packet.camera_id = camera_id;
    packet.status = status;
    mav_array_memcpy(packet.uri, uri, sizeof(char)*230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN);
#endif
}

/**
 * @brief Pack a herelink_video_stream_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_id  Video Stream ID (1 for first, 2 for second, etc.)
 * @param status  Number of streams available.
 * @param framerate [Hz] Frame rate.
 * @param resolution_h [pix] Horizontal resolution.
 * @param resolution_v [pix] Vertical resolution.
 * @param bitrate [bits/s] Bit rate.
 * @param rotation [deg] Video image rotation clockwise.
 * @param uri  Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station should listen to).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_herelink_video_stream_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t camera_id,uint8_t status,float framerate,uint16_t resolution_h,uint16_t resolution_v,uint32_t bitrate,uint16_t rotation,const char *uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN];
    _mav_put_float(buf, 0, framerate);
    _mav_put_uint32_t(buf, 4, bitrate);
    _mav_put_uint16_t(buf, 8, resolution_h);
    _mav_put_uint16_t(buf, 10, resolution_v);
    _mav_put_uint16_t(buf, 12, rotation);
    _mav_put_uint8_t(buf, 14, camera_id);
    _mav_put_uint8_t(buf, 15, status);
    _mav_put_char_array(buf, 16, uri, 230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN);
#else
    mavlink_herelink_video_stream_information_t packet;
    packet.framerate = framerate;
    packet.bitrate = bitrate;
    packet.resolution_h = resolution_h;
    packet.resolution_v = resolution_v;
    packet.rotation = rotation;
    packet.camera_id = camera_id;
    packet.status = status;
    mav_array_memcpy(packet.uri, uri, sizeof(char)*230);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_CRC);
}

/**
 * @brief Encode a herelink_video_stream_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param herelink_video_stream_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_herelink_video_stream_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_herelink_video_stream_information_t* herelink_video_stream_information)
{
    return mavlink_msg_herelink_video_stream_information_pack(system_id, component_id, msg, herelink_video_stream_information->camera_id, herelink_video_stream_information->status, herelink_video_stream_information->framerate, herelink_video_stream_information->resolution_h, herelink_video_stream_information->resolution_v, herelink_video_stream_information->bitrate, herelink_video_stream_information->rotation, herelink_video_stream_information->uri);
}

/**
 * @brief Encode a herelink_video_stream_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param herelink_video_stream_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_herelink_video_stream_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_herelink_video_stream_information_t* herelink_video_stream_information)
{
    return mavlink_msg_herelink_video_stream_information_pack_chan(system_id, component_id, chan, msg, herelink_video_stream_information->camera_id, herelink_video_stream_information->status, herelink_video_stream_information->framerate, herelink_video_stream_information->resolution_h, herelink_video_stream_information->resolution_v, herelink_video_stream_information->bitrate, herelink_video_stream_information->rotation, herelink_video_stream_information->uri);
}

/**
 * @brief Encode a herelink_video_stream_information struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param herelink_video_stream_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_herelink_video_stream_information_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_herelink_video_stream_information_t* herelink_video_stream_information)
{
    return mavlink_msg_herelink_video_stream_information_pack_status(system_id, component_id, _status, msg,  herelink_video_stream_information->camera_id, herelink_video_stream_information->status, herelink_video_stream_information->framerate, herelink_video_stream_information->resolution_h, herelink_video_stream_information->resolution_v, herelink_video_stream_information->bitrate, herelink_video_stream_information->rotation, herelink_video_stream_information->uri);
}

/**
 * @brief Send a herelink_video_stream_information message
 * @param chan MAVLink channel to send the message
 *
 * @param camera_id  Video Stream ID (1 for first, 2 for second, etc.)
 * @param status  Number of streams available.
 * @param framerate [Hz] Frame rate.
 * @param resolution_h [pix] Horizontal resolution.
 * @param resolution_v [pix] Vertical resolution.
 * @param bitrate [bits/s] Bit rate.
 * @param rotation [deg] Video image rotation clockwise.
 * @param uri  Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station should listen to).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_herelink_video_stream_information_send(mavlink_channel_t chan, uint8_t camera_id, uint8_t status, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, const char *uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN];
    _mav_put_float(buf, 0, framerate);
    _mav_put_uint32_t(buf, 4, bitrate);
    _mav_put_uint16_t(buf, 8, resolution_h);
    _mav_put_uint16_t(buf, 10, resolution_v);
    _mav_put_uint16_t(buf, 12, rotation);
    _mav_put_uint8_t(buf, 14, camera_id);
    _mav_put_uint8_t(buf, 15, status);
    _mav_put_char_array(buf, 16, uri, 230);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION, buf, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_CRC);
#else
    mavlink_herelink_video_stream_information_t packet;
    packet.framerate = framerate;
    packet.bitrate = bitrate;
    packet.resolution_h = resolution_h;
    packet.resolution_v = resolution_v;
    packet.rotation = rotation;
    packet.camera_id = camera_id;
    packet.status = status;
    mav_array_memcpy(packet.uri, uri, sizeof(char)*230);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a herelink_video_stream_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_herelink_video_stream_information_send_struct(mavlink_channel_t chan, const mavlink_herelink_video_stream_information_t* herelink_video_stream_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_herelink_video_stream_information_send(chan, herelink_video_stream_information->camera_id, herelink_video_stream_information->status, herelink_video_stream_information->framerate, herelink_video_stream_information->resolution_h, herelink_video_stream_information->resolution_v, herelink_video_stream_information->bitrate, herelink_video_stream_information->rotation, herelink_video_stream_information->uri);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION, (const char *)herelink_video_stream_information, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_herelink_video_stream_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t camera_id, uint8_t status, float framerate, uint16_t resolution_h, uint16_t resolution_v, uint32_t bitrate, uint16_t rotation, const char *uri)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, framerate);
    _mav_put_uint32_t(buf, 4, bitrate);
    _mav_put_uint16_t(buf, 8, resolution_h);
    _mav_put_uint16_t(buf, 10, resolution_v);
    _mav_put_uint16_t(buf, 12, rotation);
    _mav_put_uint8_t(buf, 14, camera_id);
    _mav_put_uint8_t(buf, 15, status);
    _mav_put_char_array(buf, 16, uri, 230);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION, buf, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_CRC);
#else
    mavlink_herelink_video_stream_information_t *packet = (mavlink_herelink_video_stream_information_t *)msgbuf;
    packet->framerate = framerate;
    packet->bitrate = bitrate;
    packet->resolution_h = resolution_h;
    packet->resolution_v = resolution_v;
    packet->rotation = rotation;
    packet->camera_id = camera_id;
    packet->status = status;
    mav_array_memcpy(packet->uri, uri, sizeof(char)*230);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE HERELINK_VIDEO_STREAM_INFORMATION UNPACKING


/**
 * @brief Get field camera_id from herelink_video_stream_information message
 *
 * @return  Video Stream ID (1 for first, 2 for second, etc.)
 */
static inline uint8_t mavlink_msg_herelink_video_stream_information_get_camera_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field status from herelink_video_stream_information message
 *
 * @return  Number of streams available.
 */
static inline uint8_t mavlink_msg_herelink_video_stream_information_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field framerate from herelink_video_stream_information message
 *
 * @return [Hz] Frame rate.
 */
static inline float mavlink_msg_herelink_video_stream_information_get_framerate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field resolution_h from herelink_video_stream_information message
 *
 * @return [pix] Horizontal resolution.
 */
static inline uint16_t mavlink_msg_herelink_video_stream_information_get_resolution_h(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field resolution_v from herelink_video_stream_information message
 *
 * @return [pix] Vertical resolution.
 */
static inline uint16_t mavlink_msg_herelink_video_stream_information_get_resolution_v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field bitrate from herelink_video_stream_information message
 *
 * @return [bits/s] Bit rate.
 */
static inline uint32_t mavlink_msg_herelink_video_stream_information_get_bitrate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field rotation from herelink_video_stream_information message
 *
 * @return [deg] Video image rotation clockwise.
 */
static inline uint16_t mavlink_msg_herelink_video_stream_information_get_rotation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field uri from herelink_video_stream_information message
 *
 * @return  Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station should listen to).
 */
static inline uint16_t mavlink_msg_herelink_video_stream_information_get_uri(const mavlink_message_t* msg, char *uri)
{
    return _MAV_RETURN_char_array(msg, uri, 230,  16);
}

/**
 * @brief Decode a herelink_video_stream_information message into a struct
 *
 * @param msg The message to decode
 * @param herelink_video_stream_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_herelink_video_stream_information_decode(const mavlink_message_t* msg, mavlink_herelink_video_stream_information_t* herelink_video_stream_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    herelink_video_stream_information->framerate = mavlink_msg_herelink_video_stream_information_get_framerate(msg);
    herelink_video_stream_information->bitrate = mavlink_msg_herelink_video_stream_information_get_bitrate(msg);
    herelink_video_stream_information->resolution_h = mavlink_msg_herelink_video_stream_information_get_resolution_h(msg);
    herelink_video_stream_information->resolution_v = mavlink_msg_herelink_video_stream_information_get_resolution_v(msg);
    herelink_video_stream_information->rotation = mavlink_msg_herelink_video_stream_information_get_rotation(msg);
    herelink_video_stream_information->camera_id = mavlink_msg_herelink_video_stream_information_get_camera_id(msg);
    herelink_video_stream_information->status = mavlink_msg_herelink_video_stream_information_get_status(msg);
    mavlink_msg_herelink_video_stream_information_get_uri(msg, herelink_video_stream_information->uri);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN;
        memset(herelink_video_stream_information, 0, MAVLINK_MSG_ID_HERELINK_VIDEO_STREAM_INFORMATION_LEN);
    memcpy(herelink_video_stream_information, _MAV_PAYLOAD(msg), len);
#endif
}
