#pragma once
// MESSAGE CAMERA_SETTINGS PACKING

#define MAVLINK_MSG_ID_CAMERA_SETTINGS 260

MAVPACKED(
typedef struct __mavlink_camera_settings_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float aperture; /*< Aperture is 1/value*/
 float shutter_speed; /*< Shutter speed in s*/
 float iso_sensitivity; /*< ISO sensitivity*/
 float white_balance; /*< Color temperature in degrees Kelvin*/
 uint8_t camera_id; /*< Camera ID if there are multiple*/
 uint8_t aperture_locked; /*< Aperture locked (0: auto, 1: locked)*/
 uint8_t shutter_speed_locked; /*< Shutter speed locked (0: auto, 1: locked)*/
 uint8_t iso_sensitivity_locked; /*< ISO sensitivity locked (0: auto, 1: locked)*/
 uint8_t white_balance_locked; /*< Color temperature locked (0: auto, 1: locked)*/
 uint8_t mode_id; /*< Reserved for a camera mode ID*/
 uint8_t color_mode_id; /*< Reserved for a color mode ID*/
 uint8_t image_format_id; /*< Reserved for image format ID*/
}) mavlink_camera_settings_t;

#define MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN 28
#define MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN 28
#define MAVLINK_MSG_ID_260_LEN 28
#define MAVLINK_MSG_ID_260_MIN_LEN 28

#define MAVLINK_MSG_ID_CAMERA_SETTINGS_CRC 8
#define MAVLINK_MSG_ID_260_CRC 8



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_SETTINGS { \
    260, \
    "CAMERA_SETTINGS", \
    13, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_settings_t, time_boot_ms) }, \
         { "camera_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_camera_settings_t, camera_id) }, \
         { "aperture", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_camera_settings_t, aperture) }, \
         { "aperture_locked", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_camera_settings_t, aperture_locked) }, \
         { "shutter_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_settings_t, shutter_speed) }, \
         { "shutter_speed_locked", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_camera_settings_t, shutter_speed_locked) }, \
         { "iso_sensitivity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_settings_t, iso_sensitivity) }, \
         { "iso_sensitivity_locked", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_camera_settings_t, iso_sensitivity_locked) }, \
         { "white_balance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_settings_t, white_balance) }, \
         { "white_balance_locked", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_camera_settings_t, white_balance_locked) }, \
         { "mode_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_camera_settings_t, mode_id) }, \
         { "color_mode_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_camera_settings_t, color_mode_id) }, \
         { "image_format_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_camera_settings_t, image_format_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_SETTINGS { \
    "CAMERA_SETTINGS", \
    13, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_settings_t, time_boot_ms) }, \
         { "camera_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_camera_settings_t, camera_id) }, \
         { "aperture", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_camera_settings_t, aperture) }, \
         { "aperture_locked", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_camera_settings_t, aperture_locked) }, \
         { "shutter_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_settings_t, shutter_speed) }, \
         { "shutter_speed_locked", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_camera_settings_t, shutter_speed_locked) }, \
         { "iso_sensitivity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_settings_t, iso_sensitivity) }, \
         { "iso_sensitivity_locked", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_camera_settings_t, iso_sensitivity_locked) }, \
         { "white_balance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_settings_t, white_balance) }, \
         { "white_balance_locked", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_camera_settings_t, white_balance_locked) }, \
         { "mode_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_camera_settings_t, mode_id) }, \
         { "color_mode_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_camera_settings_t, color_mode_id) }, \
         { "image_format_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_camera_settings_t, image_format_id) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_settings message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID if there are multiple
 * @param aperture Aperture is 1/value
 * @param aperture_locked Aperture locked (0: auto, 1: locked)
 * @param shutter_speed Shutter speed in s
 * @param shutter_speed_locked Shutter speed locked (0: auto, 1: locked)
 * @param iso_sensitivity ISO sensitivity
 * @param iso_sensitivity_locked ISO sensitivity locked (0: auto, 1: locked)
 * @param white_balance Color temperature in degrees Kelvin
 * @param white_balance_locked Color temperature locked (0: auto, 1: locked)
 * @param mode_id Reserved for a camera mode ID
 * @param color_mode_id Reserved for a color mode ID
 * @param image_format_id Reserved for image format ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_settings_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t camera_id, float aperture, uint8_t aperture_locked, float shutter_speed, uint8_t shutter_speed_locked, float iso_sensitivity, uint8_t iso_sensitivity_locked, float white_balance, uint8_t white_balance_locked, uint8_t mode_id, uint8_t color_mode_id, uint8_t image_format_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, aperture);
    _mav_put_float(buf, 8, shutter_speed);
    _mav_put_float(buf, 12, iso_sensitivity);
    _mav_put_float(buf, 16, white_balance);
    _mav_put_uint8_t(buf, 20, camera_id);
    _mav_put_uint8_t(buf, 21, aperture_locked);
    _mav_put_uint8_t(buf, 22, shutter_speed_locked);
    _mav_put_uint8_t(buf, 23, iso_sensitivity_locked);
    _mav_put_uint8_t(buf, 24, white_balance_locked);
    _mav_put_uint8_t(buf, 25, mode_id);
    _mav_put_uint8_t(buf, 26, color_mode_id);
    _mav_put_uint8_t(buf, 27, image_format_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN);
#else
    mavlink_camera_settings_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.aperture = aperture;
    packet.shutter_speed = shutter_speed;
    packet.iso_sensitivity = iso_sensitivity;
    packet.white_balance = white_balance;
    packet.camera_id = camera_id;
    packet.aperture_locked = aperture_locked;
    packet.shutter_speed_locked = shutter_speed_locked;
    packet.iso_sensitivity_locked = iso_sensitivity_locked;
    packet.white_balance_locked = white_balance_locked;
    packet.mode_id = mode_id;
    packet.color_mode_id = color_mode_id;
    packet.image_format_id = image_format_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_SETTINGS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_CRC);
}

/**
 * @brief Pack a camera_settings message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID if there are multiple
 * @param aperture Aperture is 1/value
 * @param aperture_locked Aperture locked (0: auto, 1: locked)
 * @param shutter_speed Shutter speed in s
 * @param shutter_speed_locked Shutter speed locked (0: auto, 1: locked)
 * @param iso_sensitivity ISO sensitivity
 * @param iso_sensitivity_locked ISO sensitivity locked (0: auto, 1: locked)
 * @param white_balance Color temperature in degrees Kelvin
 * @param white_balance_locked Color temperature locked (0: auto, 1: locked)
 * @param mode_id Reserved for a camera mode ID
 * @param color_mode_id Reserved for a color mode ID
 * @param image_format_id Reserved for image format ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_settings_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t camera_id,float aperture,uint8_t aperture_locked,float shutter_speed,uint8_t shutter_speed_locked,float iso_sensitivity,uint8_t iso_sensitivity_locked,float white_balance,uint8_t white_balance_locked,uint8_t mode_id,uint8_t color_mode_id,uint8_t image_format_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, aperture);
    _mav_put_float(buf, 8, shutter_speed);
    _mav_put_float(buf, 12, iso_sensitivity);
    _mav_put_float(buf, 16, white_balance);
    _mav_put_uint8_t(buf, 20, camera_id);
    _mav_put_uint8_t(buf, 21, aperture_locked);
    _mav_put_uint8_t(buf, 22, shutter_speed_locked);
    _mav_put_uint8_t(buf, 23, iso_sensitivity_locked);
    _mav_put_uint8_t(buf, 24, white_balance_locked);
    _mav_put_uint8_t(buf, 25, mode_id);
    _mav_put_uint8_t(buf, 26, color_mode_id);
    _mav_put_uint8_t(buf, 27, image_format_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN);
#else
    mavlink_camera_settings_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.aperture = aperture;
    packet.shutter_speed = shutter_speed;
    packet.iso_sensitivity = iso_sensitivity;
    packet.white_balance = white_balance;
    packet.camera_id = camera_id;
    packet.aperture_locked = aperture_locked;
    packet.shutter_speed_locked = shutter_speed_locked;
    packet.iso_sensitivity_locked = iso_sensitivity_locked;
    packet.white_balance_locked = white_balance_locked;
    packet.mode_id = mode_id;
    packet.color_mode_id = color_mode_id;
    packet.image_format_id = image_format_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_SETTINGS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_CRC);
}

/**
 * @brief Encode a camera_settings struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_settings C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_settings_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_settings_t* camera_settings)
{
    return mavlink_msg_camera_settings_pack(system_id, component_id, msg, camera_settings->time_boot_ms, camera_settings->camera_id, camera_settings->aperture, camera_settings->aperture_locked, camera_settings->shutter_speed, camera_settings->shutter_speed_locked, camera_settings->iso_sensitivity, camera_settings->iso_sensitivity_locked, camera_settings->white_balance, camera_settings->white_balance_locked, camera_settings->mode_id, camera_settings->color_mode_id, camera_settings->image_format_id);
}

/**
 * @brief Encode a camera_settings struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_settings C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_settings_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_settings_t* camera_settings)
{
    return mavlink_msg_camera_settings_pack_chan(system_id, component_id, chan, msg, camera_settings->time_boot_ms, camera_settings->camera_id, camera_settings->aperture, camera_settings->aperture_locked, camera_settings->shutter_speed, camera_settings->shutter_speed_locked, camera_settings->iso_sensitivity, camera_settings->iso_sensitivity_locked, camera_settings->white_balance, camera_settings->white_balance_locked, camera_settings->mode_id, camera_settings->color_mode_id, camera_settings->image_format_id);
}

/**
 * @brief Send a camera_settings message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param camera_id Camera ID if there are multiple
 * @param aperture Aperture is 1/value
 * @param aperture_locked Aperture locked (0: auto, 1: locked)
 * @param shutter_speed Shutter speed in s
 * @param shutter_speed_locked Shutter speed locked (0: auto, 1: locked)
 * @param iso_sensitivity ISO sensitivity
 * @param iso_sensitivity_locked ISO sensitivity locked (0: auto, 1: locked)
 * @param white_balance Color temperature in degrees Kelvin
 * @param white_balance_locked Color temperature locked (0: auto, 1: locked)
 * @param mode_id Reserved for a camera mode ID
 * @param color_mode_id Reserved for a color mode ID
 * @param image_format_id Reserved for image format ID
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_settings_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t camera_id, float aperture, uint8_t aperture_locked, float shutter_speed, uint8_t shutter_speed_locked, float iso_sensitivity, uint8_t iso_sensitivity_locked, float white_balance, uint8_t white_balance_locked, uint8_t mode_id, uint8_t color_mode_id, uint8_t image_format_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, aperture);
    _mav_put_float(buf, 8, shutter_speed);
    _mav_put_float(buf, 12, iso_sensitivity);
    _mav_put_float(buf, 16, white_balance);
    _mav_put_uint8_t(buf, 20, camera_id);
    _mav_put_uint8_t(buf, 21, aperture_locked);
    _mav_put_uint8_t(buf, 22, shutter_speed_locked);
    _mav_put_uint8_t(buf, 23, iso_sensitivity_locked);
    _mav_put_uint8_t(buf, 24, white_balance_locked);
    _mav_put_uint8_t(buf, 25, mode_id);
    _mav_put_uint8_t(buf, 26, color_mode_id);
    _mav_put_uint8_t(buf, 27, image_format_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_SETTINGS, buf, MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_CRC);
#else
    mavlink_camera_settings_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.aperture = aperture;
    packet.shutter_speed = shutter_speed;
    packet.iso_sensitivity = iso_sensitivity;
    packet.white_balance = white_balance;
    packet.camera_id = camera_id;
    packet.aperture_locked = aperture_locked;
    packet.shutter_speed_locked = shutter_speed_locked;
    packet.iso_sensitivity_locked = iso_sensitivity_locked;
    packet.white_balance_locked = white_balance_locked;
    packet.mode_id = mode_id;
    packet.color_mode_id = color_mode_id;
    packet.image_format_id = image_format_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_SETTINGS, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_CRC);
#endif
}

/**
 * @brief Send a camera_settings message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_settings_send_struct(mavlink_channel_t chan, const mavlink_camera_settings_t* camera_settings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_settings_send(chan, camera_settings->time_boot_ms, camera_settings->camera_id, camera_settings->aperture, camera_settings->aperture_locked, camera_settings->shutter_speed, camera_settings->shutter_speed_locked, camera_settings->iso_sensitivity, camera_settings->iso_sensitivity_locked, camera_settings->white_balance, camera_settings->white_balance_locked, camera_settings->mode_id, camera_settings->color_mode_id, camera_settings->image_format_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_SETTINGS, (const char *)camera_settings, MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_settings_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t camera_id, float aperture, uint8_t aperture_locked, float shutter_speed, uint8_t shutter_speed_locked, float iso_sensitivity, uint8_t iso_sensitivity_locked, float white_balance, uint8_t white_balance_locked, uint8_t mode_id, uint8_t color_mode_id, uint8_t image_format_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, aperture);
    _mav_put_float(buf, 8, shutter_speed);
    _mav_put_float(buf, 12, iso_sensitivity);
    _mav_put_float(buf, 16, white_balance);
    _mav_put_uint8_t(buf, 20, camera_id);
    _mav_put_uint8_t(buf, 21, aperture_locked);
    _mav_put_uint8_t(buf, 22, shutter_speed_locked);
    _mav_put_uint8_t(buf, 23, iso_sensitivity_locked);
    _mav_put_uint8_t(buf, 24, white_balance_locked);
    _mav_put_uint8_t(buf, 25, mode_id);
    _mav_put_uint8_t(buf, 26, color_mode_id);
    _mav_put_uint8_t(buf, 27, image_format_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_SETTINGS, buf, MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_CRC);
#else
    mavlink_camera_settings_t *packet = (mavlink_camera_settings_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->aperture = aperture;
    packet->shutter_speed = shutter_speed;
    packet->iso_sensitivity = iso_sensitivity;
    packet->white_balance = white_balance;
    packet->camera_id = camera_id;
    packet->aperture_locked = aperture_locked;
    packet->shutter_speed_locked = shutter_speed_locked;
    packet->iso_sensitivity_locked = iso_sensitivity_locked;
    packet->white_balance_locked = white_balance_locked;
    packet->mode_id = mode_id;
    packet->color_mode_id = color_mode_id;
    packet->image_format_id = image_format_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_SETTINGS, (const char *)packet, MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN, MAVLINK_MSG_ID_CAMERA_SETTINGS_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_SETTINGS UNPACKING


/**
 * @brief Get field time_boot_ms from camera_settings message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_camera_settings_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field camera_id from camera_settings message
 *
 * @return Camera ID if there are multiple
 */
static inline uint8_t mavlink_msg_camera_settings_get_camera_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field aperture from camera_settings message
 *
 * @return Aperture is 1/value
 */
static inline float mavlink_msg_camera_settings_get_aperture(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field aperture_locked from camera_settings message
 *
 * @return Aperture locked (0: auto, 1: locked)
 */
static inline uint8_t mavlink_msg_camera_settings_get_aperture_locked(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field shutter_speed from camera_settings message
 *
 * @return Shutter speed in s
 */
static inline float mavlink_msg_camera_settings_get_shutter_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field shutter_speed_locked from camera_settings message
 *
 * @return Shutter speed locked (0: auto, 1: locked)
 */
static inline uint8_t mavlink_msg_camera_settings_get_shutter_speed_locked(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field iso_sensitivity from camera_settings message
 *
 * @return ISO sensitivity
 */
static inline float mavlink_msg_camera_settings_get_iso_sensitivity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field iso_sensitivity_locked from camera_settings message
 *
 * @return ISO sensitivity locked (0: auto, 1: locked)
 */
static inline uint8_t mavlink_msg_camera_settings_get_iso_sensitivity_locked(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field white_balance from camera_settings message
 *
 * @return Color temperature in degrees Kelvin
 */
static inline float mavlink_msg_camera_settings_get_white_balance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field white_balance_locked from camera_settings message
 *
 * @return Color temperature locked (0: auto, 1: locked)
 */
static inline uint8_t mavlink_msg_camera_settings_get_white_balance_locked(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field mode_id from camera_settings message
 *
 * @return Reserved for a camera mode ID
 */
static inline uint8_t mavlink_msg_camera_settings_get_mode_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field color_mode_id from camera_settings message
 *
 * @return Reserved for a color mode ID
 */
static inline uint8_t mavlink_msg_camera_settings_get_color_mode_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field image_format_id from camera_settings message
 *
 * @return Reserved for image format ID
 */
static inline uint8_t mavlink_msg_camera_settings_get_image_format_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Decode a camera_settings message into a struct
 *
 * @param msg The message to decode
 * @param camera_settings C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_settings_decode(const mavlink_message_t* msg, mavlink_camera_settings_t* camera_settings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_settings->time_boot_ms = mavlink_msg_camera_settings_get_time_boot_ms(msg);
    camera_settings->aperture = mavlink_msg_camera_settings_get_aperture(msg);
    camera_settings->shutter_speed = mavlink_msg_camera_settings_get_shutter_speed(msg);
    camera_settings->iso_sensitivity = mavlink_msg_camera_settings_get_iso_sensitivity(msg);
    camera_settings->white_balance = mavlink_msg_camera_settings_get_white_balance(msg);
    camera_settings->camera_id = mavlink_msg_camera_settings_get_camera_id(msg);
    camera_settings->aperture_locked = mavlink_msg_camera_settings_get_aperture_locked(msg);
    camera_settings->shutter_speed_locked = mavlink_msg_camera_settings_get_shutter_speed_locked(msg);
    camera_settings->iso_sensitivity_locked = mavlink_msg_camera_settings_get_iso_sensitivity_locked(msg);
    camera_settings->white_balance_locked = mavlink_msg_camera_settings_get_white_balance_locked(msg);
    camera_settings->mode_id = mavlink_msg_camera_settings_get_mode_id(msg);
    camera_settings->color_mode_id = mavlink_msg_camera_settings_get_color_mode_id(msg);
    camera_settings->image_format_id = mavlink_msg_camera_settings_get_image_format_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN;
        memset(camera_settings, 0, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN);
    memcpy(camera_settings, _MAV_PAYLOAD(msg), len);
#endif
}
