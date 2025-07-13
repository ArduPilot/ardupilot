#pragma once
// MESSAGE SAFETY_SET_ALLOWED_AREA PACKING

#define MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA 54


typedef struct __mavlink_safety_set_allowed_area_t {
 float p1x; /*< [m] x position 1 / Latitude 1*/
 float p1y; /*< [m] y position 1 / Longitude 1*/
 float p1z; /*< [m] z position 1 / Altitude 1*/
 float p2x; /*< [m] x position 2 / Latitude 2*/
 float p2y; /*< [m] y position 2 / Longitude 2*/
 float p2z; /*< [m] z position 2 / Altitude 2*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t frame; /*<  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.*/
} mavlink_safety_set_allowed_area_t;

#define MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN 27
#define MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN 27
#define MAVLINK_MSG_ID_54_LEN 27
#define MAVLINK_MSG_ID_54_MIN_LEN 27

#define MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_CRC 15
#define MAVLINK_MSG_ID_54_CRC 15



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SAFETY_SET_ALLOWED_AREA { \
    54, \
    "SAFETY_SET_ALLOWED_AREA", \
    9, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_safety_set_allowed_area_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_safety_set_allowed_area_t, target_component) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_safety_set_allowed_area_t, frame) }, \
         { "p1x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_safety_set_allowed_area_t, p1x) }, \
         { "p1y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_safety_set_allowed_area_t, p1y) }, \
         { "p1z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_safety_set_allowed_area_t, p1z) }, \
         { "p2x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_safety_set_allowed_area_t, p2x) }, \
         { "p2y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_safety_set_allowed_area_t, p2y) }, \
         { "p2z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_safety_set_allowed_area_t, p2z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SAFETY_SET_ALLOWED_AREA { \
    "SAFETY_SET_ALLOWED_AREA", \
    9, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_safety_set_allowed_area_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_safety_set_allowed_area_t, target_component) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_safety_set_allowed_area_t, frame) }, \
         { "p1x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_safety_set_allowed_area_t, p1x) }, \
         { "p1y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_safety_set_allowed_area_t, p1y) }, \
         { "p1z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_safety_set_allowed_area_t, p1z) }, \
         { "p2x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_safety_set_allowed_area_t, p2x) }, \
         { "p2y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_safety_set_allowed_area_t, p2y) }, \
         { "p2z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_safety_set_allowed_area_t, p2z) }, \
         } \
}
#endif

/**
 * @brief Pack a safety_set_allowed_area message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x [m] x position 1 / Latitude 1
 * @param p1y [m] y position 1 / Longitude 1
 * @param p1z [m] z position 1 / Altitude 1
 * @param p2x [m] x position 2 / Latitude 2
 * @param p2y [m] y position 2 / Longitude 2
 * @param p2z [m] z position 2 / Altitude 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_safety_set_allowed_area_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN];
    _mav_put_float(buf, 0, p1x);
    _mav_put_float(buf, 4, p1y);
    _mav_put_float(buf, 8, p1z);
    _mav_put_float(buf, 12, p2x);
    _mav_put_float(buf, 16, p2y);
    _mav_put_float(buf, 20, p2z);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);
    _mav_put_uint8_t(buf, 26, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN);
#else
    mavlink_safety_set_allowed_area_t packet;
    packet.p1x = p1x;
    packet.p1y = p1y;
    packet.p1z = p1z;
    packet.p2x = p2x;
    packet.p2y = p2y;
    packet.p2z = p2z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_CRC);
}

/**
 * @brief Pack a safety_set_allowed_area message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x [m] x position 1 / Latitude 1
 * @param p1y [m] y position 1 / Longitude 1
 * @param p1z [m] z position 1 / Altitude 1
 * @param p2x [m] x position 2 / Latitude 2
 * @param p2y [m] y position 2 / Longitude 2
 * @param p2z [m] z position 2 / Altitude 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_safety_set_allowed_area_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN];
    _mav_put_float(buf, 0, p1x);
    _mav_put_float(buf, 4, p1y);
    _mav_put_float(buf, 8, p1z);
    _mav_put_float(buf, 12, p2x);
    _mav_put_float(buf, 16, p2y);
    _mav_put_float(buf, 20, p2z);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);
    _mav_put_uint8_t(buf, 26, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN);
#else
    mavlink_safety_set_allowed_area_t packet;
    packet.p1x = p1x;
    packet.p1y = p1y;
    packet.p1z = p1z;
    packet.p2x = p2x;
    packet.p2y = p2y;
    packet.p2z = p2z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN);
#endif
}

/**
 * @brief Pack a safety_set_allowed_area message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x [m] x position 1 / Latitude 1
 * @param p1y [m] y position 1 / Longitude 1
 * @param p1z [m] z position 1 / Altitude 1
 * @param p2x [m] x position 2 / Latitude 2
 * @param p2y [m] y position 2 / Longitude 2
 * @param p2z [m] z position 2 / Altitude 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_safety_set_allowed_area_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t frame,float p1x,float p1y,float p1z,float p2x,float p2y,float p2z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN];
    _mav_put_float(buf, 0, p1x);
    _mav_put_float(buf, 4, p1y);
    _mav_put_float(buf, 8, p1z);
    _mav_put_float(buf, 12, p2x);
    _mav_put_float(buf, 16, p2y);
    _mav_put_float(buf, 20, p2z);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);
    _mav_put_uint8_t(buf, 26, frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN);
#else
    mavlink_safety_set_allowed_area_t packet;
    packet.p1x = p1x;
    packet.p1y = p1y;
    packet.p1z = p1z;
    packet.p2x = p2x;
    packet.p2y = p2y;
    packet.p2z = p2z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_CRC);
}

/**
 * @brief Encode a safety_set_allowed_area struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param safety_set_allowed_area C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_safety_set_allowed_area_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_safety_set_allowed_area_t* safety_set_allowed_area)
{
    return mavlink_msg_safety_set_allowed_area_pack(system_id, component_id, msg, safety_set_allowed_area->target_system, safety_set_allowed_area->target_component, safety_set_allowed_area->frame, safety_set_allowed_area->p1x, safety_set_allowed_area->p1y, safety_set_allowed_area->p1z, safety_set_allowed_area->p2x, safety_set_allowed_area->p2y, safety_set_allowed_area->p2z);
}

/**
 * @brief Encode a safety_set_allowed_area struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param safety_set_allowed_area C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_safety_set_allowed_area_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_safety_set_allowed_area_t* safety_set_allowed_area)
{
    return mavlink_msg_safety_set_allowed_area_pack_chan(system_id, component_id, chan, msg, safety_set_allowed_area->target_system, safety_set_allowed_area->target_component, safety_set_allowed_area->frame, safety_set_allowed_area->p1x, safety_set_allowed_area->p1y, safety_set_allowed_area->p1z, safety_set_allowed_area->p2x, safety_set_allowed_area->p2y, safety_set_allowed_area->p2z);
}

/**
 * @brief Encode a safety_set_allowed_area struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param safety_set_allowed_area C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_safety_set_allowed_area_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_safety_set_allowed_area_t* safety_set_allowed_area)
{
    return mavlink_msg_safety_set_allowed_area_pack_status(system_id, component_id, _status, msg,  safety_set_allowed_area->target_system, safety_set_allowed_area->target_component, safety_set_allowed_area->frame, safety_set_allowed_area->p1x, safety_set_allowed_area->p1y, safety_set_allowed_area->p1z, safety_set_allowed_area->p2x, safety_set_allowed_area->p2y, safety_set_allowed_area->p2z);
}

/**
 * @brief Send a safety_set_allowed_area message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param frame  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x [m] x position 1 / Latitude 1
 * @param p1y [m] y position 1 / Longitude 1
 * @param p1z [m] z position 1 / Altitude 1
 * @param p2x [m] x position 2 / Latitude 2
 * @param p2y [m] y position 2 / Longitude 2
 * @param p2z [m] z position 2 / Altitude 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_safety_set_allowed_area_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN];
    _mav_put_float(buf, 0, p1x);
    _mav_put_float(buf, 4, p1y);
    _mav_put_float(buf, 8, p1z);
    _mav_put_float(buf, 12, p2x);
    _mav_put_float(buf, 16, p2y);
    _mav_put_float(buf, 20, p2z);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);
    _mav_put_uint8_t(buf, 26, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA, buf, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_CRC);
#else
    mavlink_safety_set_allowed_area_t packet;
    packet.p1x = p1x;
    packet.p1y = p1y;
    packet.p1z = p1z;
    packet.p2x = p2x;
    packet.p2y = p2y;
    packet.p2z = p2z;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA, (const char *)&packet, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_CRC);
#endif
}

/**
 * @brief Send a safety_set_allowed_area message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_safety_set_allowed_area_send_struct(mavlink_channel_t chan, const mavlink_safety_set_allowed_area_t* safety_set_allowed_area)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_safety_set_allowed_area_send(chan, safety_set_allowed_area->target_system, safety_set_allowed_area->target_component, safety_set_allowed_area->frame, safety_set_allowed_area->p1x, safety_set_allowed_area->p1y, safety_set_allowed_area->p1z, safety_set_allowed_area->p2x, safety_set_allowed_area->p2y, safety_set_allowed_area->p2z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA, (const char *)safety_set_allowed_area, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_CRC);
#endif
}

#if MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_safety_set_allowed_area_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, p1x);
    _mav_put_float(buf, 4, p1y);
    _mav_put_float(buf, 8, p1z);
    _mav_put_float(buf, 12, p2x);
    _mav_put_float(buf, 16, p2y);
    _mav_put_float(buf, 20, p2z);
    _mav_put_uint8_t(buf, 24, target_system);
    _mav_put_uint8_t(buf, 25, target_component);
    _mav_put_uint8_t(buf, 26, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA, buf, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_CRC);
#else
    mavlink_safety_set_allowed_area_t *packet = (mavlink_safety_set_allowed_area_t *)msgbuf;
    packet->p1x = p1x;
    packet->p1y = p1y;
    packet->p1z = p1z;
    packet->p2x = p2x;
    packet->p2y = p2y;
    packet->p2z = p2z;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA, (const char *)packet, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_MIN_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_CRC);
#endif
}
#endif

#endif

// MESSAGE SAFETY_SET_ALLOWED_AREA UNPACKING


/**
 * @brief Get field target_system from safety_set_allowed_area message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_safety_set_allowed_area_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field target_component from safety_set_allowed_area message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_safety_set_allowed_area_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field frame from safety_set_allowed_area message
 *
 * @return  Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 */
static inline uint8_t mavlink_msg_safety_set_allowed_area_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field p1x from safety_set_allowed_area message
 *
 * @return [m] x position 1 / Latitude 1
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p1x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field p1y from safety_set_allowed_area message
 *
 * @return [m] y position 1 / Longitude 1
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p1y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field p1z from safety_set_allowed_area message
 *
 * @return [m] z position 1 / Altitude 1
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p1z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field p2x from safety_set_allowed_area message
 *
 * @return [m] x position 2 / Latitude 2
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p2x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field p2y from safety_set_allowed_area message
 *
 * @return [m] y position 2 / Longitude 2
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p2y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field p2z from safety_set_allowed_area message
 *
 * @return [m] z position 2 / Altitude 2
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p2z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a safety_set_allowed_area message into a struct
 *
 * @param msg The message to decode
 * @param safety_set_allowed_area C-struct to decode the message contents into
 */
static inline void mavlink_msg_safety_set_allowed_area_decode(const mavlink_message_t* msg, mavlink_safety_set_allowed_area_t* safety_set_allowed_area)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    safety_set_allowed_area->p1x = mavlink_msg_safety_set_allowed_area_get_p1x(msg);
    safety_set_allowed_area->p1y = mavlink_msg_safety_set_allowed_area_get_p1y(msg);
    safety_set_allowed_area->p1z = mavlink_msg_safety_set_allowed_area_get_p1z(msg);
    safety_set_allowed_area->p2x = mavlink_msg_safety_set_allowed_area_get_p2x(msg);
    safety_set_allowed_area->p2y = mavlink_msg_safety_set_allowed_area_get_p2y(msg);
    safety_set_allowed_area->p2z = mavlink_msg_safety_set_allowed_area_get_p2z(msg);
    safety_set_allowed_area->target_system = mavlink_msg_safety_set_allowed_area_get_target_system(msg);
    safety_set_allowed_area->target_component = mavlink_msg_safety_set_allowed_area_get_target_component(msg);
    safety_set_allowed_area->frame = mavlink_msg_safety_set_allowed_area_get_frame(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN? msg->len : MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN;
        memset(safety_set_allowed_area, 0, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA_LEN);
    memcpy(safety_set_allowed_area, _MAV_PAYLOAD(msg), len);
#endif
}
