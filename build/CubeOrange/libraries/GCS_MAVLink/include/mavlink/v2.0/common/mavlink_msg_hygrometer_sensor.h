#pragma once
// MESSAGE HYGROMETER_SENSOR PACKING

#define MAVLINK_MSG_ID_HYGROMETER_SENSOR 12920


typedef struct __mavlink_hygrometer_sensor_t {
 int16_t temperature; /*< [cdegC] Temperature*/
 uint16_t humidity; /*< [c%] Humidity*/
 uint8_t id; /*<  Hygrometer ID*/
} mavlink_hygrometer_sensor_t;

#define MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN 5
#define MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN 5
#define MAVLINK_MSG_ID_12920_LEN 5
#define MAVLINK_MSG_ID_12920_MIN_LEN 5

#define MAVLINK_MSG_ID_HYGROMETER_SENSOR_CRC 20
#define MAVLINK_MSG_ID_12920_CRC 20



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HYGROMETER_SENSOR { \
    12920, \
    "HYGROMETER_SENSOR", \
    3, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_hygrometer_sensor_t, id) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_hygrometer_sensor_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_hygrometer_sensor_t, humidity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HYGROMETER_SENSOR { \
    "HYGROMETER_SENSOR", \
    3, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_hygrometer_sensor_t, id) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_hygrometer_sensor_t, temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_hygrometer_sensor_t, humidity) }, \
         } \
}
#endif

/**
 * @brief Pack a hygrometer_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Hygrometer ID
 * @param temperature [cdegC] Temperature
 * @param humidity [c%] Humidity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hygrometer_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, int16_t temperature, uint16_t humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN];
    _mav_put_int16_t(buf, 0, temperature);
    _mav_put_uint16_t(buf, 2, humidity);
    _mav_put_uint8_t(buf, 4, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN);
#else
    mavlink_hygrometer_sensor_t packet;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HYGROMETER_SENSOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_CRC);
}

/**
 * @brief Pack a hygrometer_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  Hygrometer ID
 * @param temperature [cdegC] Temperature
 * @param humidity [c%] Humidity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hygrometer_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,int16_t temperature,uint16_t humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN];
    _mav_put_int16_t(buf, 0, temperature);
    _mav_put_uint16_t(buf, 2, humidity);
    _mav_put_uint8_t(buf, 4, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN);
#else
    mavlink_hygrometer_sensor_t packet;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HYGROMETER_SENSOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_CRC);
}

/**
 * @brief Encode a hygrometer_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hygrometer_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hygrometer_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hygrometer_sensor_t* hygrometer_sensor)
{
    return mavlink_msg_hygrometer_sensor_pack(system_id, component_id, msg, hygrometer_sensor->id, hygrometer_sensor->temperature, hygrometer_sensor->humidity);
}

/**
 * @brief Encode a hygrometer_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hygrometer_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hygrometer_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hygrometer_sensor_t* hygrometer_sensor)
{
    return mavlink_msg_hygrometer_sensor_pack_chan(system_id, component_id, chan, msg, hygrometer_sensor->id, hygrometer_sensor->temperature, hygrometer_sensor->humidity);
}

/**
 * @brief Send a hygrometer_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param id  Hygrometer ID
 * @param temperature [cdegC] Temperature
 * @param humidity [c%] Humidity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hygrometer_sensor_send(mavlink_channel_t chan, uint8_t id, int16_t temperature, uint16_t humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN];
    _mav_put_int16_t(buf, 0, temperature);
    _mav_put_uint16_t(buf, 2, humidity);
    _mav_put_uint8_t(buf, 4, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HYGROMETER_SENSOR, buf, MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_CRC);
#else
    mavlink_hygrometer_sensor_t packet;
    packet.temperature = temperature;
    packet.humidity = humidity;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HYGROMETER_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_CRC);
#endif
}

/**
 * @brief Send a hygrometer_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hygrometer_sensor_send_struct(mavlink_channel_t chan, const mavlink_hygrometer_sensor_t* hygrometer_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hygrometer_sensor_send(chan, hygrometer_sensor->id, hygrometer_sensor->temperature, hygrometer_sensor->humidity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HYGROMETER_SENSOR, (const char *)hygrometer_sensor, MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hygrometer_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, int16_t temperature, uint16_t humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, temperature);
    _mav_put_uint16_t(buf, 2, humidity);
    _mav_put_uint8_t(buf, 4, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HYGROMETER_SENSOR, buf, MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_CRC);
#else
    mavlink_hygrometer_sensor_t *packet = (mavlink_hygrometer_sensor_t *)msgbuf;
    packet->temperature = temperature;
    packet->humidity = humidity;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HYGROMETER_SENSOR, (const char *)packet, MAVLINK_MSG_ID_HYGROMETER_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN, MAVLINK_MSG_ID_HYGROMETER_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE HYGROMETER_SENSOR UNPACKING


/**
 * @brief Get field id from hygrometer_sensor message
 *
 * @return  Hygrometer ID
 */
static inline uint8_t mavlink_msg_hygrometer_sensor_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field temperature from hygrometer_sensor message
 *
 * @return [cdegC] Temperature
 */
static inline int16_t mavlink_msg_hygrometer_sensor_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field humidity from hygrometer_sensor message
 *
 * @return [c%] Humidity
 */
static inline uint16_t mavlink_msg_hygrometer_sensor_get_humidity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a hygrometer_sensor message into a struct
 *
 * @param msg The message to decode
 * @param hygrometer_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_hygrometer_sensor_decode(const mavlink_message_t* msg, mavlink_hygrometer_sensor_t* hygrometer_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hygrometer_sensor->temperature = mavlink_msg_hygrometer_sensor_get_temperature(msg);
    hygrometer_sensor->humidity = mavlink_msg_hygrometer_sensor_get_humidity(msg);
    hygrometer_sensor->id = mavlink_msg_hygrometer_sensor_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN? msg->len : MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN;
        memset(hygrometer_sensor, 0, MAVLINK_MSG_ID_HYGROMETER_SENSOR_LEN);
    memcpy(hygrometer_sensor, _MAV_PAYLOAD(msg), len);
#endif
}
