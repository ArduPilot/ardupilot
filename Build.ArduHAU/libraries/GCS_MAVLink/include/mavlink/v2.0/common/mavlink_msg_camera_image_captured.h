#pragma once
// MESSAGE CAMERA_IMAGE_CAPTURED PACKING

#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED 263

MAVPACKED(
typedef struct __mavlink_camera_image_captured_t {
 uint64_t time_utc; /*< Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.*/
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 int32_t lat; /*< Latitude, expressed as degrees * 1E7 where image was taken*/
 int32_t lon; /*< Longitude, expressed as degrees * 1E7 where capture was taken*/
 int32_t alt; /*< Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken*/
 int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1E3 where image was taken*/
 float q[4]; /*< Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)*/
 int32_t image_index; /*< Zero based index of this image (image count since armed -1)*/
 uint8_t camera_id; /*< Camera ID (1 for first, 2 for second, etc.)*/
 int8_t capture_result; /*< Boolean indicating success (1) or failure (0) while capturing this image.*/
 char file_url[205]; /*< URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.*/
}) mavlink_camera_image_captured_t;

#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN 255
#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN 255
#define MAVLINK_MSG_ID_263_LEN 255
#define MAVLINK_MSG_ID_263_MIN_LEN 255

#define MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC 133
#define MAVLINK_MSG_ID_263_CRC 133

#define MAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_Q_LEN 4
#define MAVLINK_MSG_CAMERA_IMAGE_CAPTURED_FIELD_FILE_URL_LEN 205

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_IMAGE_CAPTURED { \
    263, \
    "CAMERA_IMAGE_CAPTURED", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_camera_image_captured_t, time_boot_ms) }, \
         { "time_utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_camera_image_captured_t, time_utc) }, \
         { "camera_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_camera_image_captured_t, camera_id) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_camera_image_captured_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_camera_image_captured_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_camera_image_captured_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_camera_image_captured_t, relative_alt) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 28, offsetof(mavlink_camera_image_captured_t, q) }, \
         { "image_index", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_camera_image_captured_t, image_index) }, \
         { "capture_result", NULL, MAVLINK_TYPE_INT8_T, 0, 49, offsetof(mavlink_camera_image_captured_t, capture_result) }, \
         { "file_url", NULL, MAVLINK_TYPE_CHAR, 205, 50, offsetof(mavlink_camera_image_captured_t, file_url) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_IMAGE_CAPTURED { \
    "CAMERA_IMAGE_CAPTURED", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_camera_image_captured_t, time_boot_ms) }, \
         { "time_utc", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_camera_image_captured_t, time_utc) }, \
         { "camera_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_camera_image_captured_t, camera_id) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_camera_image_captured_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_camera_image_captured_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_camera_image_captured_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_camera_image_captured_t, relative_alt) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 4, 28, offsetof(mavlink_camera_image_captured_t, q) }, \
         { "image_index", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_camera_image_captured_t, image_index) }, \
         { "capture_result", NULL, MAVLINK_TYPE_INT8_T, 0, 49, offsetof(mavlink_camera_image_captured_t, capture_result) }, \
         { "file_url", NULL, MAVLINK_TYPE_CHAR, 205, 50, offsetof(mavlink_camera_image_captured_t, file_url) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_image_captured message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param time_utc Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
 * @param camera_id Camera ID (1 for first, 2 for second, etc.)
 * @param lat Latitude, expressed as degrees * 1E7 where image was taken
 * @param lon Longitude, expressed as degrees * 1E7 where capture was taken
 * @param alt Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
 * @param relative_alt Altitude above ground in meters, expressed as * 1E3 where image was taken
 * @param q Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
 * @param image_index Zero based index of this image (image count since armed -1)
 * @param capture_result Boolean indicating success (1) or failure (0) while capturing this image.
 * @param file_url URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_image_captured_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float *q, int32_t image_index, int8_t capture_result, const char *file_url)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN];
    _mav_put_uint64_t(buf, 0, time_utc);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_int32_t(buf, 20, alt);
    _mav_put_int32_t(buf, 24, relative_alt);
    _mav_put_int32_t(buf, 44, image_index);
    _mav_put_uint8_t(buf, 48, camera_id);
    _mav_put_int8_t(buf, 49, capture_result);
    _mav_put_float_array(buf, 28, q, 4);
    _mav_put_char_array(buf, 50, file_url, 205);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN);
#else
    mavlink_camera_image_captured_t packet;
    packet.time_utc = time_utc;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.image_index = image_index;
    packet.camera_id = camera_id;
    packet.capture_result = capture_result;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.file_url, file_url, sizeof(char)*205);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
}

/**
 * @brief Pack a camera_image_captured message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param time_utc Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
 * @param camera_id Camera ID (1 for first, 2 for second, etc.)
 * @param lat Latitude, expressed as degrees * 1E7 where image was taken
 * @param lon Longitude, expressed as degrees * 1E7 where capture was taken
 * @param alt Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
 * @param relative_alt Altitude above ground in meters, expressed as * 1E3 where image was taken
 * @param q Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
 * @param image_index Zero based index of this image (image count since armed -1)
 * @param capture_result Boolean indicating success (1) or failure (0) while capturing this image.
 * @param file_url URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_image_captured_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint64_t time_utc,uint8_t camera_id,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,const float *q,int32_t image_index,int8_t capture_result,const char *file_url)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN];
    _mav_put_uint64_t(buf, 0, time_utc);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_int32_t(buf, 20, alt);
    _mav_put_int32_t(buf, 24, relative_alt);
    _mav_put_int32_t(buf, 44, image_index);
    _mav_put_uint8_t(buf, 48, camera_id);
    _mav_put_int8_t(buf, 49, capture_result);
    _mav_put_float_array(buf, 28, q, 4);
    _mav_put_char_array(buf, 50, file_url, 205);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN);
#else
    mavlink_camera_image_captured_t packet;
    packet.time_utc = time_utc;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.image_index = image_index;
    packet.camera_id = camera_id;
    packet.capture_result = capture_result;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.file_url, file_url, sizeof(char)*205);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
}

/**
 * @brief Encode a camera_image_captured struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_image_captured C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_image_captured_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_image_captured_t* camera_image_captured)
{
    return mavlink_msg_camera_image_captured_pack(system_id, component_id, msg, camera_image_captured->time_boot_ms, camera_image_captured->time_utc, camera_image_captured->camera_id, camera_image_captured->lat, camera_image_captured->lon, camera_image_captured->alt, camera_image_captured->relative_alt, camera_image_captured->q, camera_image_captured->image_index, camera_image_captured->capture_result, camera_image_captured->file_url);
}

/**
 * @brief Encode a camera_image_captured struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_image_captured C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_image_captured_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_image_captured_t* camera_image_captured)
{
    return mavlink_msg_camera_image_captured_pack_chan(system_id, component_id, chan, msg, camera_image_captured->time_boot_ms, camera_image_captured->time_utc, camera_image_captured->camera_id, camera_image_captured->lat, camera_image_captured->lon, camera_image_captured->alt, camera_image_captured->relative_alt, camera_image_captured->q, camera_image_captured->image_index, camera_image_captured->capture_result, camera_image_captured->file_url);
}

/**
 * @brief Send a camera_image_captured message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param time_utc Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
 * @param camera_id Camera ID (1 for first, 2 for second, etc.)
 * @param lat Latitude, expressed as degrees * 1E7 where image was taken
 * @param lon Longitude, expressed as degrees * 1E7 where capture was taken
 * @param alt Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
 * @param relative_alt Altitude above ground in meters, expressed as * 1E3 where image was taken
 * @param q Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
 * @param image_index Zero based index of this image (image count since armed -1)
 * @param capture_result Boolean indicating success (1) or failure (0) while capturing this image.
 * @param file_url URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_image_captured_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float *q, int32_t image_index, int8_t capture_result, const char *file_url)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN];
    _mav_put_uint64_t(buf, 0, time_utc);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_int32_t(buf, 20, alt);
    _mav_put_int32_t(buf, 24, relative_alt);
    _mav_put_int32_t(buf, 44, image_index);
    _mav_put_uint8_t(buf, 48, camera_id);
    _mav_put_int8_t(buf, 49, capture_result);
    _mav_put_float_array(buf, 28, q, 4);
    _mav_put_char_array(buf, 50, file_url, 205);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED, buf, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
#else
    mavlink_camera_image_captured_t packet;
    packet.time_utc = time_utc;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.image_index = image_index;
    packet.camera_id = camera_id;
    packet.capture_result = capture_result;
    mav_array_memcpy(packet.q, q, sizeof(float)*4);
    mav_array_memcpy(packet.file_url, file_url, sizeof(char)*205);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
#endif
}

/**
 * @brief Send a camera_image_captured message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_image_captured_send_struct(mavlink_channel_t chan, const mavlink_camera_image_captured_t* camera_image_captured)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_image_captured_send(chan, camera_image_captured->time_boot_ms, camera_image_captured->time_utc, camera_image_captured->camera_id, camera_image_captured->lat, camera_image_captured->lon, camera_image_captured->alt, camera_image_captured->relative_alt, camera_image_captured->q, camera_image_captured->image_index, camera_image_captured->capture_result, camera_image_captured->file_url);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED, (const char *)camera_image_captured, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_image_captured_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint64_t time_utc, uint8_t camera_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, const float *q, int32_t image_index, int8_t capture_result, const char *file_url)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_utc);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_int32_t(buf, 20, alt);
    _mav_put_int32_t(buf, 24, relative_alt);
    _mav_put_int32_t(buf, 44, image_index);
    _mav_put_uint8_t(buf, 48, camera_id);
    _mav_put_int8_t(buf, 49, capture_result);
    _mav_put_float_array(buf, 28, q, 4);
    _mav_put_char_array(buf, 50, file_url, 205);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED, buf, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
#else
    mavlink_camera_image_captured_t *packet = (mavlink_camera_image_captured_t *)msgbuf;
    packet->time_utc = time_utc;
    packet->time_boot_ms = time_boot_ms;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->relative_alt = relative_alt;
    packet->image_index = image_index;
    packet->camera_id = camera_id;
    packet->capture_result = capture_result;
    mav_array_memcpy(packet->q, q, sizeof(float)*4);
    mav_array_memcpy(packet->file_url, file_url, sizeof(char)*205);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED, (const char *)packet, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_MIN_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_IMAGE_CAPTURED UNPACKING


/**
 * @brief Get field time_boot_ms from camera_image_captured message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_camera_image_captured_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field time_utc from camera_image_captured message
 *
 * @return Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown.
 */
static inline uint64_t mavlink_msg_camera_image_captured_get_time_utc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field camera_id from camera_image_captured message
 *
 * @return Camera ID (1 for first, 2 for second, etc.)
 */
static inline uint8_t mavlink_msg_camera_image_captured_get_camera_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field lat from camera_image_captured message
 *
 * @return Latitude, expressed as degrees * 1E7 where image was taken
 */
static inline int32_t mavlink_msg_camera_image_captured_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field lon from camera_image_captured message
 *
 * @return Longitude, expressed as degrees * 1E7 where capture was taken
 */
static inline int32_t mavlink_msg_camera_image_captured_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field alt from camera_image_captured message
 *
 * @return Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken
 */
static inline int32_t mavlink_msg_camera_image_captured_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field relative_alt from camera_image_captured message
 *
 * @return Altitude above ground in meters, expressed as * 1E3 where image was taken
 */
static inline int32_t mavlink_msg_camera_image_captured_get_relative_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field q from camera_image_captured message
 *
 * @return Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)
 */
static inline uint16_t mavlink_msg_camera_image_captured_get_q(const mavlink_message_t* msg, float *q)
{
    return _MAV_RETURN_float_array(msg, q, 4,  28);
}

/**
 * @brief Get field image_index from camera_image_captured message
 *
 * @return Zero based index of this image (image count since armed -1)
 */
static inline int32_t mavlink_msg_camera_image_captured_get_image_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  44);
}

/**
 * @brief Get field capture_result from camera_image_captured message
 *
 * @return Boolean indicating success (1) or failure (0) while capturing this image.
 */
static inline int8_t mavlink_msg_camera_image_captured_get_capture_result(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  49);
}

/**
 * @brief Get field file_url from camera_image_captured message
 *
 * @return URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
 */
static inline uint16_t mavlink_msg_camera_image_captured_get_file_url(const mavlink_message_t* msg, char *file_url)
{
    return _MAV_RETURN_char_array(msg, file_url, 205,  50);
}

/**
 * @brief Decode a camera_image_captured message into a struct
 *
 * @param msg The message to decode
 * @param camera_image_captured C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_image_captured_decode(const mavlink_message_t* msg, mavlink_camera_image_captured_t* camera_image_captured)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_image_captured->time_utc = mavlink_msg_camera_image_captured_get_time_utc(msg);
    camera_image_captured->time_boot_ms = mavlink_msg_camera_image_captured_get_time_boot_ms(msg);
    camera_image_captured->lat = mavlink_msg_camera_image_captured_get_lat(msg);
    camera_image_captured->lon = mavlink_msg_camera_image_captured_get_lon(msg);
    camera_image_captured->alt = mavlink_msg_camera_image_captured_get_alt(msg);
    camera_image_captured->relative_alt = mavlink_msg_camera_image_captured_get_relative_alt(msg);
    mavlink_msg_camera_image_captured_get_q(msg, camera_image_captured->q);
    camera_image_captured->image_index = mavlink_msg_camera_image_captured_get_image_index(msg);
    camera_image_captured->camera_id = mavlink_msg_camera_image_captured_get_camera_id(msg);
    camera_image_captured->capture_result = mavlink_msg_camera_image_captured_get_capture_result(msg);
    mavlink_msg_camera_image_captured_get_file_url(msg, camera_image_captured->file_url);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN;
        memset(camera_image_captured, 0, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN);
    memcpy(camera_image_captured, _MAV_PAYLOAD(msg), len);
#endif
}
