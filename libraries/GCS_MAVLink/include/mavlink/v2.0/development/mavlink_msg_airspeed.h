#pragma once
// MESSAGE AIRSPEED PACKING

#define MAVLINK_MSG_ID_AIRSPEED 295


typedef struct __mavlink_airspeed_t {
 float airspeed; /*< [m/s] Calibrated airspeed (CAS).*/
 float raw_press; /*< [hPa] Raw differential pressure. NaN for value unknown/not supplied.*/
 int16_t temperature; /*< [cdegC] Temperature. INT16_MAX for value unknown/not supplied.*/
 uint8_t id; /*<  Sensor ID.*/
 uint8_t flags; /*<  Airspeed sensor flags.*/
} mavlink_airspeed_t;

#define MAVLINK_MSG_ID_AIRSPEED_LEN 12
#define MAVLINK_MSG_ID_AIRSPEED_MIN_LEN 12
#define MAVLINK_MSG_ID_295_LEN 12
#define MAVLINK_MSG_ID_295_MIN_LEN 12

#define MAVLINK_MSG_ID_AIRSPEED_CRC 234
#define MAVLINK_MSG_ID_295_CRC 234



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AIRSPEED { \
    295, \
    "AIRSPEED", \
    5, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_airspeed_t, id) }, \
         { "airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_airspeed_t, airspeed) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_airspeed_t, temperature) }, \
         { "raw_press", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_airspeed_t, raw_press) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_airspeed_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AIRSPEED { \
    "AIRSPEED", \
    5, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_airspeed_t, id) }, \
         { "airspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_airspeed_t, airspeed) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_airspeed_t, temperature) }, \
         { "raw_press", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_airspeed_t, raw_press) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_airspeed_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a airspeed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Sensor ID.
 * @param airspeed [m/s] Calibrated airspeed (CAS).
 * @param temperature [cdegC] Temperature. INT16_MAX for value unknown/not supplied.
 * @param raw_press [hPa] Raw differential pressure. NaN for value unknown/not supplied.
 * @param flags  Airspeed sensor flags.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeed_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, float airspeed, int16_t temperature, float raw_press, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_LEN];
    _mav_put_float(buf, 0, airspeed);
    _mav_put_float(buf, 4, raw_press);
    _mav_put_int16_t(buf, 8, temperature);
    _mav_put_uint8_t(buf, 10, id);
    _mav_put_uint8_t(buf, 11, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRSPEED_LEN);
#else
    mavlink_airspeed_t packet;
    packet.airspeed = airspeed;
    packet.raw_press = raw_press;
    packet.temperature = temperature;
    packet.id = id;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRSPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRSPEED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRSPEED_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_LEN, MAVLINK_MSG_ID_AIRSPEED_CRC);
}

/**
 * @brief Pack a airspeed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Sensor ID.
 * @param airspeed [m/s] Calibrated airspeed (CAS).
 * @param temperature [cdegC] Temperature. INT16_MAX for value unknown/not supplied.
 * @param raw_press [hPa] Raw differential pressure. NaN for value unknown/not supplied.
 * @param flags  Airspeed sensor flags.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeed_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t id, float airspeed, int16_t temperature, float raw_press, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_LEN];
    _mav_put_float(buf, 0, airspeed);
    _mav_put_float(buf, 4, raw_press);
    _mav_put_int16_t(buf, 8, temperature);
    _mav_put_uint8_t(buf, 10, id);
    _mav_put_uint8_t(buf, 11, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRSPEED_LEN);
#else
    mavlink_airspeed_t packet;
    packet.airspeed = airspeed;
    packet.raw_press = raw_press;
    packet.temperature = temperature;
    packet.id = id;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRSPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRSPEED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRSPEED_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_LEN, MAVLINK_MSG_ID_AIRSPEED_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRSPEED_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_LEN);
#endif
}

/**
 * @brief Pack a airspeed message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  Sensor ID.
 * @param airspeed [m/s] Calibrated airspeed (CAS).
 * @param temperature [cdegC] Temperature. INT16_MAX for value unknown/not supplied.
 * @param raw_press [hPa] Raw differential pressure. NaN for value unknown/not supplied.
 * @param flags  Airspeed sensor flags.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeed_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,float airspeed,int16_t temperature,float raw_press,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_LEN];
    _mav_put_float(buf, 0, airspeed);
    _mav_put_float(buf, 4, raw_press);
    _mav_put_int16_t(buf, 8, temperature);
    _mav_put_uint8_t(buf, 10, id);
    _mav_put_uint8_t(buf, 11, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRSPEED_LEN);
#else
    mavlink_airspeed_t packet;
    packet.airspeed = airspeed;
    packet.raw_press = raw_press;
    packet.temperature = temperature;
    packet.id = id;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRSPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRSPEED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRSPEED_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_LEN, MAVLINK_MSG_ID_AIRSPEED_CRC);
}

/**
 * @brief Encode a airspeed struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airspeed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airspeed_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airspeed_t* airspeed)
{
    return mavlink_msg_airspeed_pack(system_id, component_id, msg, airspeed->id, airspeed->airspeed, airspeed->temperature, airspeed->raw_press, airspeed->flags);
}

/**
 * @brief Encode a airspeed struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param airspeed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airspeed_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_airspeed_t* airspeed)
{
    return mavlink_msg_airspeed_pack_chan(system_id, component_id, chan, msg, airspeed->id, airspeed->airspeed, airspeed->temperature, airspeed->raw_press, airspeed->flags);
}

/**
 * @brief Encode a airspeed struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param airspeed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airspeed_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_airspeed_t* airspeed)
{
    return mavlink_msg_airspeed_pack_status(system_id, component_id, _status, msg,  airspeed->id, airspeed->airspeed, airspeed->temperature, airspeed->raw_press, airspeed->flags);
}

/**
 * @brief Send a airspeed message
 * @param chan MAVLink channel to send the message
 *
 * @param id  Sensor ID.
 * @param airspeed [m/s] Calibrated airspeed (CAS).
 * @param temperature [cdegC] Temperature. INT16_MAX for value unknown/not supplied.
 * @param raw_press [hPa] Raw differential pressure. NaN for value unknown/not supplied.
 * @param flags  Airspeed sensor flags.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airspeed_send(mavlink_channel_t chan, uint8_t id, float airspeed, int16_t temperature, float raw_press, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_LEN];
    _mav_put_float(buf, 0, airspeed);
    _mav_put_float(buf, 4, raw_press);
    _mav_put_int16_t(buf, 8, temperature);
    _mav_put_uint8_t(buf, 10, id);
    _mav_put_uint8_t(buf, 11, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED, buf, MAVLINK_MSG_ID_AIRSPEED_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_LEN, MAVLINK_MSG_ID_AIRSPEED_CRC);
#else
    mavlink_airspeed_t packet;
    packet.airspeed = airspeed;
    packet.raw_press = raw_press;
    packet.temperature = temperature;
    packet.id = id;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED, (const char *)&packet, MAVLINK_MSG_ID_AIRSPEED_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_LEN, MAVLINK_MSG_ID_AIRSPEED_CRC);
#endif
}

/**
 * @brief Send a airspeed message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_airspeed_send_struct(mavlink_channel_t chan, const mavlink_airspeed_t* airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_airspeed_send(chan, airspeed->id, airspeed->airspeed, airspeed->temperature, airspeed->raw_press, airspeed->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED, (const char *)airspeed, MAVLINK_MSG_ID_AIRSPEED_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_LEN, MAVLINK_MSG_ID_AIRSPEED_CRC);
#endif
}

#if MAVLINK_MSG_ID_AIRSPEED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_airspeed_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, float airspeed, int16_t temperature, float raw_press, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, airspeed);
    _mav_put_float(buf, 4, raw_press);
    _mav_put_int16_t(buf, 8, temperature);
    _mav_put_uint8_t(buf, 10, id);
    _mav_put_uint8_t(buf, 11, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED, buf, MAVLINK_MSG_ID_AIRSPEED_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_LEN, MAVLINK_MSG_ID_AIRSPEED_CRC);
#else
    mavlink_airspeed_t *packet = (mavlink_airspeed_t *)msgbuf;
    packet->airspeed = airspeed;
    packet->raw_press = raw_press;
    packet->temperature = temperature;
    packet->id = id;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED, (const char *)packet, MAVLINK_MSG_ID_AIRSPEED_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_LEN, MAVLINK_MSG_ID_AIRSPEED_CRC);
#endif
}
#endif

#endif

// MESSAGE AIRSPEED UNPACKING


/**
 * @brief Get field id from airspeed message
 *
 * @return  Sensor ID.
 */
static inline uint8_t mavlink_msg_airspeed_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field airspeed from airspeed message
 *
 * @return [m/s] Calibrated airspeed (CAS).
 */
static inline float mavlink_msg_airspeed_get_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field temperature from airspeed message
 *
 * @return [cdegC] Temperature. INT16_MAX for value unknown/not supplied.
 */
static inline int16_t mavlink_msg_airspeed_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field raw_press from airspeed message
 *
 * @return [hPa] Raw differential pressure. NaN for value unknown/not supplied.
 */
static inline float mavlink_msg_airspeed_get_raw_press(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field flags from airspeed message
 *
 * @return  Airspeed sensor flags.
 */
static inline uint8_t mavlink_msg_airspeed_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Decode a airspeed message into a struct
 *
 * @param msg The message to decode
 * @param airspeed C-struct to decode the message contents into
 */
static inline void mavlink_msg_airspeed_decode(const mavlink_message_t* msg, mavlink_airspeed_t* airspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    airspeed->airspeed = mavlink_msg_airspeed_get_airspeed(msg);
    airspeed->raw_press = mavlink_msg_airspeed_get_raw_press(msg);
    airspeed->temperature = mavlink_msg_airspeed_get_temperature(msg);
    airspeed->id = mavlink_msg_airspeed_get_id(msg);
    airspeed->flags = mavlink_msg_airspeed_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AIRSPEED_LEN? msg->len : MAVLINK_MSG_ID_AIRSPEED_LEN;
        memset(airspeed, 0, MAVLINK_MSG_ID_AIRSPEED_LEN);
    memcpy(airspeed, _MAV_PAYLOAD(msg), len);
#endif
}
