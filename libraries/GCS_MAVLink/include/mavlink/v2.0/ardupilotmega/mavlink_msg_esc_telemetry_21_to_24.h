#pragma once
// MESSAGE ESC_TELEMETRY_21_TO_24 PACKING

#define MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24 11042


typedef struct __mavlink_esc_telemetry_21_to_24_t {
 uint16_t voltage[4]; /*< [cV] Voltage.*/
 uint16_t current[4]; /*< [cA] Current.*/
 uint16_t totalcurrent[4]; /*< [mAh] Total current.*/
 uint16_t rpm[4]; /*< [rpm] RPM (eRPM).*/
 uint16_t count[4]; /*<  count of telemetry packets received (wraps at 65535).*/
 uint8_t temperature[4]; /*< [degC] Temperature.*/
} mavlink_esc_telemetry_21_to_24_t;

#define MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN 44
#define MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN 44
#define MAVLINK_MSG_ID_11042_LEN 44
#define MAVLINK_MSG_ID_11042_MIN_LEN 44

#define MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_CRC 201
#define MAVLINK_MSG_ID_11042_CRC 201

#define MAVLINK_MSG_ESC_TELEMETRY_21_TO_24_FIELD_VOLTAGE_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_21_TO_24_FIELD_CURRENT_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_21_TO_24_FIELD_TOTALCURRENT_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_21_TO_24_FIELD_RPM_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_21_TO_24_FIELD_COUNT_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_21_TO_24_FIELD_TEMPERATURE_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_21_TO_24 { \
    11042, \
    "ESC_TELEMETRY_21_TO_24", \
    6, \
    {  { "temperature", NULL, MAVLINK_TYPE_UINT8_T, 4, 40, offsetof(mavlink_esc_telemetry_21_to_24_t, temperature) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 4, 0, offsetof(mavlink_esc_telemetry_21_to_24_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_UINT16_T, 4, 8, offsetof(mavlink_esc_telemetry_21_to_24_t, current) }, \
         { "totalcurrent", NULL, MAVLINK_TYPE_UINT16_T, 4, 16, offsetof(mavlink_esc_telemetry_21_to_24_t, totalcurrent) }, \
         { "rpm", NULL, MAVLINK_TYPE_UINT16_T, 4, 24, offsetof(mavlink_esc_telemetry_21_to_24_t, rpm) }, \
         { "count", NULL, MAVLINK_TYPE_UINT16_T, 4, 32, offsetof(mavlink_esc_telemetry_21_to_24_t, count) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_21_TO_24 { \
    "ESC_TELEMETRY_21_TO_24", \
    6, \
    {  { "temperature", NULL, MAVLINK_TYPE_UINT8_T, 4, 40, offsetof(mavlink_esc_telemetry_21_to_24_t, temperature) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 4, 0, offsetof(mavlink_esc_telemetry_21_to_24_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_UINT16_T, 4, 8, offsetof(mavlink_esc_telemetry_21_to_24_t, current) }, \
         { "totalcurrent", NULL, MAVLINK_TYPE_UINT16_T, 4, 16, offsetof(mavlink_esc_telemetry_21_to_24_t, totalcurrent) }, \
         { "rpm", NULL, MAVLINK_TYPE_UINT16_T, 4, 24, offsetof(mavlink_esc_telemetry_21_to_24_t, rpm) }, \
         { "count", NULL, MAVLINK_TYPE_UINT16_T, 4, 32, offsetof(mavlink_esc_telemetry_21_to_24_t, count) }, \
         } \
}
#endif

/**
 * @brief Pack a esc_telemetry_21_to_24 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature [degC] Temperature.
 * @param voltage [cV] Voltage.
 * @param current [cA] Current.
 * @param totalcurrent [mAh] Total current.
 * @param rpm [rpm] RPM (eRPM).
 * @param count  count of telemetry packets received (wraps at 65535).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint8_t *temperature, const uint16_t *voltage, const uint16_t *current, const uint16_t *totalcurrent, const uint16_t *rpm, const uint16_t *count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN];

    _mav_put_uint16_t_array(buf, 0, voltage, 4);
    _mav_put_uint16_t_array(buf, 8, current, 4);
    _mav_put_uint16_t_array(buf, 16, totalcurrent, 4);
    _mav_put_uint16_t_array(buf, 24, rpm, 4);
    _mav_put_uint16_t_array(buf, 32, count, 4);
    _mav_put_uint8_t_array(buf, 40, temperature, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN);
#else
    mavlink_esc_telemetry_21_to_24_t packet;

    mav_array_memcpy(packet.voltage, voltage, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.totalcurrent, totalcurrent, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.count, count, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.temperature, temperature, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_CRC);
}

/**
 * @brief Pack a esc_telemetry_21_to_24 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature [degC] Temperature.
 * @param voltage [cV] Voltage.
 * @param current [cA] Current.
 * @param totalcurrent [mAh] Total current.
 * @param rpm [rpm] RPM (eRPM).
 * @param count  count of telemetry packets received (wraps at 65535).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const uint8_t *temperature, const uint16_t *voltage, const uint16_t *current, const uint16_t *totalcurrent, const uint16_t *rpm, const uint16_t *count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN];

    _mav_put_uint16_t_array(buf, 0, voltage, 4);
    _mav_put_uint16_t_array(buf, 8, current, 4);
    _mav_put_uint16_t_array(buf, 16, totalcurrent, 4);
    _mav_put_uint16_t_array(buf, 24, rpm, 4);
    _mav_put_uint16_t_array(buf, 32, count, 4);
    _mav_put_uint8_t_array(buf, 40, temperature, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN);
#else
    mavlink_esc_telemetry_21_to_24_t packet;

    mav_array_memcpy(packet.voltage, voltage, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.totalcurrent, totalcurrent, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.count, count, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.temperature, temperature, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN);
#endif
}

/**
 * @brief Pack a esc_telemetry_21_to_24 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param temperature [degC] Temperature.
 * @param voltage [cV] Voltage.
 * @param current [cA] Current.
 * @param totalcurrent [mAh] Total current.
 * @param rpm [rpm] RPM (eRPM).
 * @param count  count of telemetry packets received (wraps at 65535).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint8_t *temperature,const uint16_t *voltage,const uint16_t *current,const uint16_t *totalcurrent,const uint16_t *rpm,const uint16_t *count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN];

    _mav_put_uint16_t_array(buf, 0, voltage, 4);
    _mav_put_uint16_t_array(buf, 8, current, 4);
    _mav_put_uint16_t_array(buf, 16, totalcurrent, 4);
    _mav_put_uint16_t_array(buf, 24, rpm, 4);
    _mav_put_uint16_t_array(buf, 32, count, 4);
    _mav_put_uint8_t_array(buf, 40, temperature, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN);
#else
    mavlink_esc_telemetry_21_to_24_t packet;

    mav_array_memcpy(packet.voltage, voltage, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.totalcurrent, totalcurrent, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.count, count, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.temperature, temperature, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_CRC);
}

/**
 * @brief Encode a esc_telemetry_21_to_24 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param esc_telemetry_21_to_24 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_esc_telemetry_21_to_24_t* esc_telemetry_21_to_24)
{
    return mavlink_msg_esc_telemetry_21_to_24_pack(system_id, component_id, msg, esc_telemetry_21_to_24->temperature, esc_telemetry_21_to_24->voltage, esc_telemetry_21_to_24->current, esc_telemetry_21_to_24->totalcurrent, esc_telemetry_21_to_24->rpm, esc_telemetry_21_to_24->count);
}

/**
 * @brief Encode a esc_telemetry_21_to_24 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param esc_telemetry_21_to_24 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_esc_telemetry_21_to_24_t* esc_telemetry_21_to_24)
{
    return mavlink_msg_esc_telemetry_21_to_24_pack_chan(system_id, component_id, chan, msg, esc_telemetry_21_to_24->temperature, esc_telemetry_21_to_24->voltage, esc_telemetry_21_to_24->current, esc_telemetry_21_to_24->totalcurrent, esc_telemetry_21_to_24->rpm, esc_telemetry_21_to_24->count);
}

/**
 * @brief Encode a esc_telemetry_21_to_24 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param esc_telemetry_21_to_24 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_esc_telemetry_21_to_24_t* esc_telemetry_21_to_24)
{
    return mavlink_msg_esc_telemetry_21_to_24_pack_status(system_id, component_id, _status, msg,  esc_telemetry_21_to_24->temperature, esc_telemetry_21_to_24->voltage, esc_telemetry_21_to_24->current, esc_telemetry_21_to_24->totalcurrent, esc_telemetry_21_to_24->rpm, esc_telemetry_21_to_24->count);
}

/**
 * @brief Send a esc_telemetry_21_to_24 message
 * @param chan MAVLink channel to send the message
 *
 * @param temperature [degC] Temperature.
 * @param voltage [cV] Voltage.
 * @param current [cA] Current.
 * @param totalcurrent [mAh] Total current.
 * @param rpm [rpm] RPM (eRPM).
 * @param count  count of telemetry packets received (wraps at 65535).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_esc_telemetry_21_to_24_send(mavlink_channel_t chan, const uint8_t *temperature, const uint16_t *voltage, const uint16_t *current, const uint16_t *totalcurrent, const uint16_t *rpm, const uint16_t *count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN];

    _mav_put_uint16_t_array(buf, 0, voltage, 4);
    _mav_put_uint16_t_array(buf, 8, current, 4);
    _mav_put_uint16_t_array(buf, 16, totalcurrent, 4);
    _mav_put_uint16_t_array(buf, 24, rpm, 4);
    _mav_put_uint16_t_array(buf, 32, count, 4);
    _mav_put_uint8_t_array(buf, 40, temperature, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24, buf, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_CRC);
#else
    mavlink_esc_telemetry_21_to_24_t packet;

    mav_array_memcpy(packet.voltage, voltage, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.totalcurrent, totalcurrent, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.count, count, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.temperature, temperature, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24, (const char *)&packet, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_CRC);
#endif
}

/**
 * @brief Send a esc_telemetry_21_to_24 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_esc_telemetry_21_to_24_send_struct(mavlink_channel_t chan, const mavlink_esc_telemetry_21_to_24_t* esc_telemetry_21_to_24)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_esc_telemetry_21_to_24_send(chan, esc_telemetry_21_to_24->temperature, esc_telemetry_21_to_24->voltage, esc_telemetry_21_to_24->current, esc_telemetry_21_to_24->totalcurrent, esc_telemetry_21_to_24->rpm, esc_telemetry_21_to_24->count);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24, (const char *)esc_telemetry_21_to_24, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_CRC);
#endif
}

#if MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_esc_telemetry_21_to_24_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *temperature, const uint16_t *voltage, const uint16_t *current, const uint16_t *totalcurrent, const uint16_t *rpm, const uint16_t *count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint16_t_array(buf, 0, voltage, 4);
    _mav_put_uint16_t_array(buf, 8, current, 4);
    _mav_put_uint16_t_array(buf, 16, totalcurrent, 4);
    _mav_put_uint16_t_array(buf, 24, rpm, 4);
    _mav_put_uint16_t_array(buf, 32, count, 4);
    _mav_put_uint8_t_array(buf, 40, temperature, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24, buf, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_CRC);
#else
    mavlink_esc_telemetry_21_to_24_t *packet = (mavlink_esc_telemetry_21_to_24_t *)msgbuf;

    mav_array_memcpy(packet->voltage, voltage, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->current, current, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->totalcurrent, totalcurrent, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->rpm, rpm, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->count, count, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->temperature, temperature, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24, (const char *)packet, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_CRC);
#endif
}
#endif

#endif

// MESSAGE ESC_TELEMETRY_21_TO_24 UNPACKING


/**
 * @brief Get field temperature from esc_telemetry_21_to_24 message
 *
 * @return [degC] Temperature.
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_get_temperature(const mavlink_message_t* msg, uint8_t *temperature)
{
    return _MAV_RETURN_uint8_t_array(msg, temperature, 4,  40);
}

/**
 * @brief Get field voltage from esc_telemetry_21_to_24 message
 *
 * @return [cV] Voltage.
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_get_voltage(const mavlink_message_t* msg, uint16_t *voltage)
{
    return _MAV_RETURN_uint16_t_array(msg, voltage, 4,  0);
}

/**
 * @brief Get field current from esc_telemetry_21_to_24 message
 *
 * @return [cA] Current.
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_get_current(const mavlink_message_t* msg, uint16_t *current)
{
    return _MAV_RETURN_uint16_t_array(msg, current, 4,  8);
}

/**
 * @brief Get field totalcurrent from esc_telemetry_21_to_24 message
 *
 * @return [mAh] Total current.
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_get_totalcurrent(const mavlink_message_t* msg, uint16_t *totalcurrent)
{
    return _MAV_RETURN_uint16_t_array(msg, totalcurrent, 4,  16);
}

/**
 * @brief Get field rpm from esc_telemetry_21_to_24 message
 *
 * @return [rpm] RPM (eRPM).
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_get_rpm(const mavlink_message_t* msg, uint16_t *rpm)
{
    return _MAV_RETURN_uint16_t_array(msg, rpm, 4,  24);
}

/**
 * @brief Get field count from esc_telemetry_21_to_24 message
 *
 * @return  count of telemetry packets received (wraps at 65535).
 */
static inline uint16_t mavlink_msg_esc_telemetry_21_to_24_get_count(const mavlink_message_t* msg, uint16_t *count)
{
    return _MAV_RETURN_uint16_t_array(msg, count, 4,  32);
}

/**
 * @brief Decode a esc_telemetry_21_to_24 message into a struct
 *
 * @param msg The message to decode
 * @param esc_telemetry_21_to_24 C-struct to decode the message contents into
 */
static inline void mavlink_msg_esc_telemetry_21_to_24_decode(const mavlink_message_t* msg, mavlink_esc_telemetry_21_to_24_t* esc_telemetry_21_to_24)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_esc_telemetry_21_to_24_get_voltage(msg, esc_telemetry_21_to_24->voltage);
    mavlink_msg_esc_telemetry_21_to_24_get_current(msg, esc_telemetry_21_to_24->current);
    mavlink_msg_esc_telemetry_21_to_24_get_totalcurrent(msg, esc_telemetry_21_to_24->totalcurrent);
    mavlink_msg_esc_telemetry_21_to_24_get_rpm(msg, esc_telemetry_21_to_24->rpm);
    mavlink_msg_esc_telemetry_21_to_24_get_count(msg, esc_telemetry_21_to_24->count);
    mavlink_msg_esc_telemetry_21_to_24_get_temperature(msg, esc_telemetry_21_to_24->temperature);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN? msg->len : MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN;
        memset(esc_telemetry_21_to_24, 0, MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_LEN);
    memcpy(esc_telemetry_21_to_24, _MAV_PAYLOAD(msg), len);
#endif
}
