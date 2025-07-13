#pragma once
// MESSAGE SCALED_PRESSURE PACKING

#define MAVLINK_MSG_ID_SCALED_PRESSURE 29


typedef struct __mavlink_scaled_pressure_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float press_abs; /*< [hPa] Absolute pressure*/
 float press_diff; /*< [hPa] Differential pressure 1*/
 int16_t temperature; /*< [cdegC] Absolute pressure temperature*/
 int16_t temperature_press_diff; /*< [cdegC] Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.*/
} mavlink_scaled_pressure_t;

#define MAVLINK_MSG_ID_SCALED_PRESSURE_LEN 16
#define MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN 14
#define MAVLINK_MSG_ID_29_LEN 16
#define MAVLINK_MSG_ID_29_MIN_LEN 14

#define MAVLINK_MSG_ID_SCALED_PRESSURE_CRC 115
#define MAVLINK_MSG_ID_29_CRC 115



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SCALED_PRESSURE { \
    29, \
    "SCALED_PRESSURE", \
    5, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_scaled_pressure_t, time_boot_ms) }, \
         { "press_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_scaled_pressure_t, press_abs) }, \
         { "press_diff", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_scaled_pressure_t, press_diff) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_scaled_pressure_t, temperature) }, \
         { "temperature_press_diff", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_scaled_pressure_t, temperature_press_diff) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SCALED_PRESSURE { \
    "SCALED_PRESSURE", \
    5, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_scaled_pressure_t, time_boot_ms) }, \
         { "press_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_scaled_pressure_t, press_abs) }, \
         { "press_diff", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_scaled_pressure_t, press_diff) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_scaled_pressure_t, temperature) }, \
         { "temperature_press_diff", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_scaled_pressure_t, temperature_press_diff) }, \
         } \
}
#endif

/**
 * @brief Pack a scaled_pressure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param press_abs [hPa] Absolute pressure
 * @param press_diff [hPa] Differential pressure 1
 * @param temperature [cdegC] Absolute pressure temperature
 * @param temperature_press_diff [cdegC] Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_pressure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCALED_PRESSURE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, press_abs);
    _mav_put_float(buf, 8, press_diff);
    _mav_put_int16_t(buf, 12, temperature);
    _mav_put_int16_t(buf, 14, temperature_press_diff);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN);
#else
    mavlink_scaled_pressure_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.press_abs = press_abs;
    packet.press_diff = press_diff;
    packet.temperature = temperature;
    packet.temperature_press_diff = temperature_press_diff;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCALED_PRESSURE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_CRC);
}

/**
 * @brief Pack a scaled_pressure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param press_abs [hPa] Absolute pressure
 * @param press_diff [hPa] Differential pressure 1
 * @param temperature [cdegC] Absolute pressure temperature
 * @param temperature_press_diff [cdegC] Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_pressure_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCALED_PRESSURE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, press_abs);
    _mav_put_float(buf, 8, press_diff);
    _mav_put_int16_t(buf, 12, temperature);
    _mav_put_int16_t(buf, 14, temperature_press_diff);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN);
#else
    mavlink_scaled_pressure_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.press_abs = press_abs;
    packet.press_diff = press_diff;
    packet.temperature = temperature;
    packet.temperature_press_diff = temperature_press_diff;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCALED_PRESSURE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN);
#endif
}

/**
 * @brief Pack a scaled_pressure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param press_abs [hPa] Absolute pressure
 * @param press_diff [hPa] Differential pressure 1
 * @param temperature [cdegC] Absolute pressure temperature
 * @param temperature_press_diff [cdegC] Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_pressure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float press_abs,float press_diff,int16_t temperature,int16_t temperature_press_diff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCALED_PRESSURE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, press_abs);
    _mav_put_float(buf, 8, press_diff);
    _mav_put_int16_t(buf, 12, temperature);
    _mav_put_int16_t(buf, 14, temperature_press_diff);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN);
#else
    mavlink_scaled_pressure_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.press_abs = press_abs;
    packet.press_diff = press_diff;
    packet.temperature = temperature;
    packet.temperature_press_diff = temperature_press_diff;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCALED_PRESSURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_CRC);
}

/**
 * @brief Encode a scaled_pressure struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scaled_pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_pressure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scaled_pressure_t* scaled_pressure)
{
    return mavlink_msg_scaled_pressure_pack(system_id, component_id, msg, scaled_pressure->time_boot_ms, scaled_pressure->press_abs, scaled_pressure->press_diff, scaled_pressure->temperature, scaled_pressure->temperature_press_diff);
}

/**
 * @brief Encode a scaled_pressure struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param scaled_pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_pressure_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_scaled_pressure_t* scaled_pressure)
{
    return mavlink_msg_scaled_pressure_pack_chan(system_id, component_id, chan, msg, scaled_pressure->time_boot_ms, scaled_pressure->press_abs, scaled_pressure->press_diff, scaled_pressure->temperature, scaled_pressure->temperature_press_diff);
}

/**
 * @brief Encode a scaled_pressure struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param scaled_pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_pressure_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_scaled_pressure_t* scaled_pressure)
{
    return mavlink_msg_scaled_pressure_pack_status(system_id, component_id, _status, msg,  scaled_pressure->time_boot_ms, scaled_pressure->press_abs, scaled_pressure->press_diff, scaled_pressure->temperature, scaled_pressure->temperature_press_diff);
}

/**
 * @brief Send a scaled_pressure message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param press_abs [hPa] Absolute pressure
 * @param press_diff [hPa] Differential pressure 1
 * @param temperature [cdegC] Absolute pressure temperature
 * @param temperature_press_diff [cdegC] Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_scaled_pressure_send(mavlink_channel_t chan, uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCALED_PRESSURE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, press_abs);
    _mav_put_float(buf, 8, press_diff);
    _mav_put_int16_t(buf, 12, temperature);
    _mav_put_int16_t(buf, 14, temperature_press_diff);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE, buf, MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_CRC);
#else
    mavlink_scaled_pressure_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.press_abs = press_abs;
    packet.press_diff = press_diff;
    packet.temperature = temperature;
    packet.temperature_press_diff = temperature_press_diff;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE, (const char *)&packet, MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_CRC);
#endif
}

/**
 * @brief Send a scaled_pressure message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_scaled_pressure_send_struct(mavlink_channel_t chan, const mavlink_scaled_pressure_t* scaled_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_scaled_pressure_send(chan, scaled_pressure->time_boot_ms, scaled_pressure->press_abs, scaled_pressure->press_diff, scaled_pressure->temperature, scaled_pressure->temperature_press_diff);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE, (const char *)scaled_pressure, MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_CRC);
#endif
}

#if MAVLINK_MSG_ID_SCALED_PRESSURE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_scaled_pressure_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, press_abs);
    _mav_put_float(buf, 8, press_diff);
    _mav_put_int16_t(buf, 12, temperature);
    _mav_put_int16_t(buf, 14, temperature_press_diff);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE, buf, MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_CRC);
#else
    mavlink_scaled_pressure_t *packet = (mavlink_scaled_pressure_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->press_abs = press_abs;
    packet->press_diff = press_diff;
    packet->temperature = temperature;
    packet->temperature_press_diff = temperature_press_diff;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE, (const char *)packet, MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_CRC);
#endif
}
#endif

#endif

// MESSAGE SCALED_PRESSURE UNPACKING


/**
 * @brief Get field time_boot_ms from scaled_pressure message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_scaled_pressure_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field press_abs from scaled_pressure message
 *
 * @return [hPa] Absolute pressure
 */
static inline float mavlink_msg_scaled_pressure_get_press_abs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field press_diff from scaled_pressure message
 *
 * @return [hPa] Differential pressure 1
 */
static inline float mavlink_msg_scaled_pressure_get_press_diff(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field temperature from scaled_pressure message
 *
 * @return [cdegC] Absolute pressure temperature
 */
static inline int16_t mavlink_msg_scaled_pressure_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field temperature_press_diff from scaled_pressure message
 *
 * @return [cdegC] Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
 */
static inline int16_t mavlink_msg_scaled_pressure_get_temperature_press_diff(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Decode a scaled_pressure message into a struct
 *
 * @param msg The message to decode
 * @param scaled_pressure C-struct to decode the message contents into
 */
static inline void mavlink_msg_scaled_pressure_decode(const mavlink_message_t* msg, mavlink_scaled_pressure_t* scaled_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    scaled_pressure->time_boot_ms = mavlink_msg_scaled_pressure_get_time_boot_ms(msg);
    scaled_pressure->press_abs = mavlink_msg_scaled_pressure_get_press_abs(msg);
    scaled_pressure->press_diff = mavlink_msg_scaled_pressure_get_press_diff(msg);
    scaled_pressure->temperature = mavlink_msg_scaled_pressure_get_temperature(msg);
    scaled_pressure->temperature_press_diff = mavlink_msg_scaled_pressure_get_temperature_press_diff(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SCALED_PRESSURE_LEN? msg->len : MAVLINK_MSG_ID_SCALED_PRESSURE_LEN;
        memset(scaled_pressure, 0, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN);
    memcpy(scaled_pressure, _MAV_PAYLOAD(msg), len);
#endif
}
