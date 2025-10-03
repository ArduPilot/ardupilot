#pragma once
// MESSAGE DIGICAM_CONFIGURE PACKING

#define MAVLINK_MSG_ID_DIGICAM_CONFIGURE 154


typedef struct __mavlink_digicam_configure_t {
 float extra_value; /*<  Correspondent value to given extra_param.*/
 uint16_t shutter_speed; /*<  Divisor number //e.g. 1000 means 1/1000 (0 means ignore).*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t mode; /*<  Mode enumeration from 1 to N //P, TV, AV, M, etc. (0 means ignore).*/
 uint8_t aperture; /*<  F stop number x 10 //e.g. 28 means 2.8 (0 means ignore).*/
 uint8_t iso; /*<  ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore).*/
 uint8_t exposure_type; /*<  Exposure type enumeration from 1 to N (0 means ignore).*/
 uint8_t command_id; /*<  Command Identity (incremental loop: 0 to 255). //A command sent multiple times will be executed or pooled just once.*/
 uint8_t engine_cut_off; /*< [ds] Main engine cut-off time before camera trigger (0 means no cut-off).*/
 uint8_t extra_param; /*<  Extra parameters enumeration (0 means ignore).*/
} mavlink_digicam_configure_t;

#define MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN 15
#define MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN 15
#define MAVLINK_MSG_ID_154_LEN 15
#define MAVLINK_MSG_ID_154_MIN_LEN 15

#define MAVLINK_MSG_ID_DIGICAM_CONFIGURE_CRC 84
#define MAVLINK_MSG_ID_154_CRC 84



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DIGICAM_CONFIGURE { \
    154, \
    "DIGICAM_CONFIGURE", \
    11, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_digicam_configure_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_digicam_configure_t, target_component) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_digicam_configure_t, mode) }, \
         { "shutter_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_digicam_configure_t, shutter_speed) }, \
         { "aperture", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_digicam_configure_t, aperture) }, \
         { "iso", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_digicam_configure_t, iso) }, \
         { "exposure_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_digicam_configure_t, exposure_type) }, \
         { "command_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_digicam_configure_t, command_id) }, \
         { "engine_cut_off", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_digicam_configure_t, engine_cut_off) }, \
         { "extra_param", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_digicam_configure_t, extra_param) }, \
         { "extra_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_digicam_configure_t, extra_value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DIGICAM_CONFIGURE { \
    "DIGICAM_CONFIGURE", \
    11, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_digicam_configure_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_digicam_configure_t, target_component) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_digicam_configure_t, mode) }, \
         { "shutter_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_digicam_configure_t, shutter_speed) }, \
         { "aperture", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_digicam_configure_t, aperture) }, \
         { "iso", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_digicam_configure_t, iso) }, \
         { "exposure_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_digicam_configure_t, exposure_type) }, \
         { "command_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_digicam_configure_t, command_id) }, \
         { "engine_cut_off", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_digicam_configure_t, engine_cut_off) }, \
         { "extra_param", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_digicam_configure_t, extra_param) }, \
         { "extra_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_digicam_configure_t, extra_value) }, \
         } \
}
#endif

/**
 * @brief Pack a digicam_configure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param mode  Mode enumeration from 1 to N //P, TV, AV, M, etc. (0 means ignore).
 * @param shutter_speed  Divisor number //e.g. 1000 means 1/1000 (0 means ignore).
 * @param aperture  F stop number x 10 //e.g. 28 means 2.8 (0 means ignore).
 * @param iso  ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore).
 * @param exposure_type  Exposure type enumeration from 1 to N (0 means ignore).
 * @param command_id  Command Identity (incremental loop: 0 to 255). //A command sent multiple times will be executed or pooled just once.
 * @param engine_cut_off [ds] Main engine cut-off time before camera trigger (0 means no cut-off).
 * @param extra_param  Extra parameters enumeration (0 means ignore).
 * @param extra_value  Correspondent value to given extra_param.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_digicam_configure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN];
    _mav_put_float(buf, 0, extra_value);
    _mav_put_uint16_t(buf, 4, shutter_speed);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, target_component);
    _mav_put_uint8_t(buf, 8, mode);
    _mav_put_uint8_t(buf, 9, aperture);
    _mav_put_uint8_t(buf, 10, iso);
    _mav_put_uint8_t(buf, 11, exposure_type);
    _mav_put_uint8_t(buf, 12, command_id);
    _mav_put_uint8_t(buf, 13, engine_cut_off);
    _mav_put_uint8_t(buf, 14, extra_param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN);
#else
    mavlink_digicam_configure_t packet;
    packet.extra_value = extra_value;
    packet.shutter_speed = shutter_speed;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mode = mode;
    packet.aperture = aperture;
    packet.iso = iso;
    packet.exposure_type = exposure_type;
    packet.command_id = command_id;
    packet.engine_cut_off = engine_cut_off;
    packet.extra_param = extra_param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DIGICAM_CONFIGURE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_CRC);
}

/**
 * @brief Pack a digicam_configure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param mode  Mode enumeration from 1 to N //P, TV, AV, M, etc. (0 means ignore).
 * @param shutter_speed  Divisor number //e.g. 1000 means 1/1000 (0 means ignore).
 * @param aperture  F stop number x 10 //e.g. 28 means 2.8 (0 means ignore).
 * @param iso  ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore).
 * @param exposure_type  Exposure type enumeration from 1 to N (0 means ignore).
 * @param command_id  Command Identity (incremental loop: 0 to 255). //A command sent multiple times will be executed or pooled just once.
 * @param engine_cut_off [ds] Main engine cut-off time before camera trigger (0 means no cut-off).
 * @param extra_param  Extra parameters enumeration (0 means ignore).
 * @param extra_value  Correspondent value to given extra_param.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_digicam_configure_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN];
    _mav_put_float(buf, 0, extra_value);
    _mav_put_uint16_t(buf, 4, shutter_speed);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, target_component);
    _mav_put_uint8_t(buf, 8, mode);
    _mav_put_uint8_t(buf, 9, aperture);
    _mav_put_uint8_t(buf, 10, iso);
    _mav_put_uint8_t(buf, 11, exposure_type);
    _mav_put_uint8_t(buf, 12, command_id);
    _mav_put_uint8_t(buf, 13, engine_cut_off);
    _mav_put_uint8_t(buf, 14, extra_param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN);
#else
    mavlink_digicam_configure_t packet;
    packet.extra_value = extra_value;
    packet.shutter_speed = shutter_speed;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mode = mode;
    packet.aperture = aperture;
    packet.iso = iso;
    packet.exposure_type = exposure_type;
    packet.command_id = command_id;
    packet.engine_cut_off = engine_cut_off;
    packet.extra_param = extra_param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DIGICAM_CONFIGURE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN);
#endif
}

/**
 * @brief Pack a digicam_configure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param mode  Mode enumeration from 1 to N //P, TV, AV, M, etc. (0 means ignore).
 * @param shutter_speed  Divisor number //e.g. 1000 means 1/1000 (0 means ignore).
 * @param aperture  F stop number x 10 //e.g. 28 means 2.8 (0 means ignore).
 * @param iso  ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore).
 * @param exposure_type  Exposure type enumeration from 1 to N (0 means ignore).
 * @param command_id  Command Identity (incremental loop: 0 to 255). //A command sent multiple times will be executed or pooled just once.
 * @param engine_cut_off [ds] Main engine cut-off time before camera trigger (0 means no cut-off).
 * @param extra_param  Extra parameters enumeration (0 means ignore).
 * @param extra_value  Correspondent value to given extra_param.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_digicam_configure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t mode,uint16_t shutter_speed,uint8_t aperture,uint8_t iso,uint8_t exposure_type,uint8_t command_id,uint8_t engine_cut_off,uint8_t extra_param,float extra_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN];
    _mav_put_float(buf, 0, extra_value);
    _mav_put_uint16_t(buf, 4, shutter_speed);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, target_component);
    _mav_put_uint8_t(buf, 8, mode);
    _mav_put_uint8_t(buf, 9, aperture);
    _mav_put_uint8_t(buf, 10, iso);
    _mav_put_uint8_t(buf, 11, exposure_type);
    _mav_put_uint8_t(buf, 12, command_id);
    _mav_put_uint8_t(buf, 13, engine_cut_off);
    _mav_put_uint8_t(buf, 14, extra_param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN);
#else
    mavlink_digicam_configure_t packet;
    packet.extra_value = extra_value;
    packet.shutter_speed = shutter_speed;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mode = mode;
    packet.aperture = aperture;
    packet.iso = iso;
    packet.exposure_type = exposure_type;
    packet.command_id = command_id;
    packet.engine_cut_off = engine_cut_off;
    packet.extra_param = extra_param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DIGICAM_CONFIGURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_CRC);
}

/**
 * @brief Encode a digicam_configure struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param digicam_configure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_digicam_configure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_digicam_configure_t* digicam_configure)
{
    return mavlink_msg_digicam_configure_pack(system_id, component_id, msg, digicam_configure->target_system, digicam_configure->target_component, digicam_configure->mode, digicam_configure->shutter_speed, digicam_configure->aperture, digicam_configure->iso, digicam_configure->exposure_type, digicam_configure->command_id, digicam_configure->engine_cut_off, digicam_configure->extra_param, digicam_configure->extra_value);
}

/**
 * @brief Encode a digicam_configure struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param digicam_configure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_digicam_configure_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_digicam_configure_t* digicam_configure)
{
    return mavlink_msg_digicam_configure_pack_chan(system_id, component_id, chan, msg, digicam_configure->target_system, digicam_configure->target_component, digicam_configure->mode, digicam_configure->shutter_speed, digicam_configure->aperture, digicam_configure->iso, digicam_configure->exposure_type, digicam_configure->command_id, digicam_configure->engine_cut_off, digicam_configure->extra_param, digicam_configure->extra_value);
}

/**
 * @brief Encode a digicam_configure struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param digicam_configure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_digicam_configure_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_digicam_configure_t* digicam_configure)
{
    return mavlink_msg_digicam_configure_pack_status(system_id, component_id, _status, msg,  digicam_configure->target_system, digicam_configure->target_component, digicam_configure->mode, digicam_configure->shutter_speed, digicam_configure->aperture, digicam_configure->iso, digicam_configure->exposure_type, digicam_configure->command_id, digicam_configure->engine_cut_off, digicam_configure->extra_param, digicam_configure->extra_value);
}

/**
 * @brief Send a digicam_configure message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param mode  Mode enumeration from 1 to N //P, TV, AV, M, etc. (0 means ignore).
 * @param shutter_speed  Divisor number //e.g. 1000 means 1/1000 (0 means ignore).
 * @param aperture  F stop number x 10 //e.g. 28 means 2.8 (0 means ignore).
 * @param iso  ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore).
 * @param exposure_type  Exposure type enumeration from 1 to N (0 means ignore).
 * @param command_id  Command Identity (incremental loop: 0 to 255). //A command sent multiple times will be executed or pooled just once.
 * @param engine_cut_off [ds] Main engine cut-off time before camera trigger (0 means no cut-off).
 * @param extra_param  Extra parameters enumeration (0 means ignore).
 * @param extra_value  Correspondent value to given extra_param.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_digicam_configure_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN];
    _mav_put_float(buf, 0, extra_value);
    _mav_put_uint16_t(buf, 4, shutter_speed);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, target_component);
    _mav_put_uint8_t(buf, 8, mode);
    _mav_put_uint8_t(buf, 9, aperture);
    _mav_put_uint8_t(buf, 10, iso);
    _mav_put_uint8_t(buf, 11, exposure_type);
    _mav_put_uint8_t(buf, 12, command_id);
    _mav_put_uint8_t(buf, 13, engine_cut_off);
    _mav_put_uint8_t(buf, 14, extra_param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DIGICAM_CONFIGURE, buf, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_CRC);
#else
    mavlink_digicam_configure_t packet;
    packet.extra_value = extra_value;
    packet.shutter_speed = shutter_speed;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.mode = mode;
    packet.aperture = aperture;
    packet.iso = iso;
    packet.exposure_type = exposure_type;
    packet.command_id = command_id;
    packet.engine_cut_off = engine_cut_off;
    packet.extra_param = extra_param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DIGICAM_CONFIGURE, (const char *)&packet, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_CRC);
#endif
}

/**
 * @brief Send a digicam_configure message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_digicam_configure_send_struct(mavlink_channel_t chan, const mavlink_digicam_configure_t* digicam_configure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_digicam_configure_send(chan, digicam_configure->target_system, digicam_configure->target_component, digicam_configure->mode, digicam_configure->shutter_speed, digicam_configure->aperture, digicam_configure->iso, digicam_configure->exposure_type, digicam_configure->command_id, digicam_configure->engine_cut_off, digicam_configure->extra_param, digicam_configure->extra_value);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DIGICAM_CONFIGURE, (const char *)digicam_configure, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_CRC);
#endif
}

#if MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_digicam_configure_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t mode, uint16_t shutter_speed, uint8_t aperture, uint8_t iso, uint8_t exposure_type, uint8_t command_id, uint8_t engine_cut_off, uint8_t extra_param, float extra_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, extra_value);
    _mav_put_uint16_t(buf, 4, shutter_speed);
    _mav_put_uint8_t(buf, 6, target_system);
    _mav_put_uint8_t(buf, 7, target_component);
    _mav_put_uint8_t(buf, 8, mode);
    _mav_put_uint8_t(buf, 9, aperture);
    _mav_put_uint8_t(buf, 10, iso);
    _mav_put_uint8_t(buf, 11, exposure_type);
    _mav_put_uint8_t(buf, 12, command_id);
    _mav_put_uint8_t(buf, 13, engine_cut_off);
    _mav_put_uint8_t(buf, 14, extra_param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DIGICAM_CONFIGURE, buf, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_CRC);
#else
    mavlink_digicam_configure_t *packet = (mavlink_digicam_configure_t *)msgbuf;
    packet->extra_value = extra_value;
    packet->shutter_speed = shutter_speed;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->mode = mode;
    packet->aperture = aperture;
    packet->iso = iso;
    packet->exposure_type = exposure_type;
    packet->command_id = command_id;
    packet->engine_cut_off = engine_cut_off;
    packet->extra_param = extra_param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DIGICAM_CONFIGURE, (const char *)packet, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_CRC);
#endif
}
#endif

#endif

// MESSAGE DIGICAM_CONFIGURE UNPACKING


/**
 * @brief Get field target_system from digicam_configure message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_digicam_configure_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field target_component from digicam_configure message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_digicam_configure_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field mode from digicam_configure message
 *
 * @return  Mode enumeration from 1 to N //P, TV, AV, M, etc. (0 means ignore).
 */
static inline uint8_t mavlink_msg_digicam_configure_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field shutter_speed from digicam_configure message
 *
 * @return  Divisor number //e.g. 1000 means 1/1000 (0 means ignore).
 */
static inline uint16_t mavlink_msg_digicam_configure_get_shutter_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field aperture from digicam_configure message
 *
 * @return  F stop number x 10 //e.g. 28 means 2.8 (0 means ignore).
 */
static inline uint8_t mavlink_msg_digicam_configure_get_aperture(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field iso from digicam_configure message
 *
 * @return  ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore).
 */
static inline uint8_t mavlink_msg_digicam_configure_get_iso(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field exposure_type from digicam_configure message
 *
 * @return  Exposure type enumeration from 1 to N (0 means ignore).
 */
static inline uint8_t mavlink_msg_digicam_configure_get_exposure_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field command_id from digicam_configure message
 *
 * @return  Command Identity (incremental loop: 0 to 255). //A command sent multiple times will be executed or pooled just once.
 */
static inline uint8_t mavlink_msg_digicam_configure_get_command_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field engine_cut_off from digicam_configure message
 *
 * @return [ds] Main engine cut-off time before camera trigger (0 means no cut-off).
 */
static inline uint8_t mavlink_msg_digicam_configure_get_engine_cut_off(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field extra_param from digicam_configure message
 *
 * @return  Extra parameters enumeration (0 means ignore).
 */
static inline uint8_t mavlink_msg_digicam_configure_get_extra_param(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field extra_value from digicam_configure message
 *
 * @return  Correspondent value to given extra_param.
 */
static inline float mavlink_msg_digicam_configure_get_extra_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a digicam_configure message into a struct
 *
 * @param msg The message to decode
 * @param digicam_configure C-struct to decode the message contents into
 */
static inline void mavlink_msg_digicam_configure_decode(const mavlink_message_t* msg, mavlink_digicam_configure_t* digicam_configure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    digicam_configure->extra_value = mavlink_msg_digicam_configure_get_extra_value(msg);
    digicam_configure->shutter_speed = mavlink_msg_digicam_configure_get_shutter_speed(msg);
    digicam_configure->target_system = mavlink_msg_digicam_configure_get_target_system(msg);
    digicam_configure->target_component = mavlink_msg_digicam_configure_get_target_component(msg);
    digicam_configure->mode = mavlink_msg_digicam_configure_get_mode(msg);
    digicam_configure->aperture = mavlink_msg_digicam_configure_get_aperture(msg);
    digicam_configure->iso = mavlink_msg_digicam_configure_get_iso(msg);
    digicam_configure->exposure_type = mavlink_msg_digicam_configure_get_exposure_type(msg);
    digicam_configure->command_id = mavlink_msg_digicam_configure_get_command_id(msg);
    digicam_configure->engine_cut_off = mavlink_msg_digicam_configure_get_engine_cut_off(msg);
    digicam_configure->extra_param = mavlink_msg_digicam_configure_get_extra_param(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN? msg->len : MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN;
        memset(digicam_configure, 0, MAVLINK_MSG_ID_DIGICAM_CONFIGURE_LEN);
    memcpy(digicam_configure, _MAV_PAYLOAD(msg), len);
#endif
}
