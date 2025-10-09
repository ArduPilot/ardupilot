#pragma once
// MESSAGE ESC_TELEMETRY_1_TO_4 PACKING

#define MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4 11030


typedef struct __mavlink_esc_telemetry_1_to_4_t {
 uint16_t voltage[4]; /*< [cV] Voltage.*/
 uint16_t current[4]; /*< [cA] Current.*/
 uint16_t totalcurrent[4]; /*< [mAh] Total current.*/
 uint16_t rpm[4]; /*< [rpm] RPM (eRPM).*/
 uint16_t count[4]; /*<  count of telemetry packets received (wraps at 65535).*/
 uint8_t temperature[4]; /*< [degC] Temperature.*/
} mavlink_esc_telemetry_1_to_4_t;

#define MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN 44
#define MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN 44
#define MAVLINK_MSG_ID_11030_LEN 44
#define MAVLINK_MSG_ID_11030_MIN_LEN 44

#define MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_CRC 144
#define MAVLINK_MSG_ID_11030_CRC 144

#define MAVLINK_MSG_ESC_TELEMETRY_1_TO_4_FIELD_VOLTAGE_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_1_TO_4_FIELD_CURRENT_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_1_TO_4_FIELD_TOTALCURRENT_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_1_TO_4_FIELD_RPM_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_1_TO_4_FIELD_COUNT_LEN 4
#define MAVLINK_MSG_ESC_TELEMETRY_1_TO_4_FIELD_TEMPERATURE_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_1_TO_4 { \
    11030, \
    "ESC_TELEMETRY_1_TO_4", \
    6, \
    {  { "temperature", NULL, MAVLINK_TYPE_UINT8_T, 4, 40, offsetof(mavlink_esc_telemetry_1_to_4_t, temperature) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 4, 0, offsetof(mavlink_esc_telemetry_1_to_4_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_UINT16_T, 4, 8, offsetof(mavlink_esc_telemetry_1_to_4_t, current) }, \
         { "totalcurrent", NULL, MAVLINK_TYPE_UINT16_T, 4, 16, offsetof(mavlink_esc_telemetry_1_to_4_t, totalcurrent) }, \
         { "rpm", NULL, MAVLINK_TYPE_UINT16_T, 4, 24, offsetof(mavlink_esc_telemetry_1_to_4_t, rpm) }, \
         { "count", NULL, MAVLINK_TYPE_UINT16_T, 4, 32, offsetof(mavlink_esc_telemetry_1_to_4_t, count) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_1_TO_4 { \
    "ESC_TELEMETRY_1_TO_4", \
    6, \
    {  { "temperature", NULL, MAVLINK_TYPE_UINT8_T, 4, 40, offsetof(mavlink_esc_telemetry_1_to_4_t, temperature) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 4, 0, offsetof(mavlink_esc_telemetry_1_to_4_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_UINT16_T, 4, 8, offsetof(mavlink_esc_telemetry_1_to_4_t, current) }, \
         { "totalcurrent", NULL, MAVLINK_TYPE_UINT16_T, 4, 16, offsetof(mavlink_esc_telemetry_1_to_4_t, totalcurrent) }, \
         { "rpm", NULL, MAVLINK_TYPE_UINT16_T, 4, 24, offsetof(mavlink_esc_telemetry_1_to_4_t, rpm) }, \
         { "count", NULL, MAVLINK_TYPE_UINT16_T, 4, 32, offsetof(mavlink_esc_telemetry_1_to_4_t, count) }, \
         } \
}
#endif

/**
 * @brief Pack a esc_telemetry_1_to_4 message
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
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint8_t *temperature, const uint16_t *voltage, const uint16_t *current, const uint16_t *totalcurrent, const uint16_t *rpm, const uint16_t *count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN];

    _mav_put_uint16_t_array(buf, 0, voltage, 4);
    _mav_put_uint16_t_array(buf, 8, current, 4);
    _mav_put_uint16_t_array(buf, 16, totalcurrent, 4);
    _mav_put_uint16_t_array(buf, 24, rpm, 4);
    _mav_put_uint16_t_array(buf, 32, count, 4);
    _mav_put_uint8_t_array(buf, 40, temperature, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN);
#else
    mavlink_esc_telemetry_1_to_4_t packet;

    mav_array_memcpy(packet.voltage, voltage, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.totalcurrent, totalcurrent, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.count, count, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.temperature, temperature, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_CRC);
}

/**
 * @brief Pack a esc_telemetry_1_to_4 message
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
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const uint8_t *temperature, const uint16_t *voltage, const uint16_t *current, const uint16_t *totalcurrent, const uint16_t *rpm, const uint16_t *count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN];

    _mav_put_uint16_t_array(buf, 0, voltage, 4);
    _mav_put_uint16_t_array(buf, 8, current, 4);
    _mav_put_uint16_t_array(buf, 16, totalcurrent, 4);
    _mav_put_uint16_t_array(buf, 24, rpm, 4);
    _mav_put_uint16_t_array(buf, 32, count, 4);
    _mav_put_uint8_t_array(buf, 40, temperature, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN);
#else
    mavlink_esc_telemetry_1_to_4_t packet;

    mav_array_memcpy(packet.voltage, voltage, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.totalcurrent, totalcurrent, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.count, count, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.temperature, temperature, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN);
#endif
}

/**
 * @brief Pack a esc_telemetry_1_to_4 message on a channel
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
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint8_t *temperature,const uint16_t *voltage,const uint16_t *current,const uint16_t *totalcurrent,const uint16_t *rpm,const uint16_t *count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN];

    _mav_put_uint16_t_array(buf, 0, voltage, 4);
    _mav_put_uint16_t_array(buf, 8, current, 4);
    _mav_put_uint16_t_array(buf, 16, totalcurrent, 4);
    _mav_put_uint16_t_array(buf, 24, rpm, 4);
    _mav_put_uint16_t_array(buf, 32, count, 4);
    _mav_put_uint8_t_array(buf, 40, temperature, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN);
#else
    mavlink_esc_telemetry_1_to_4_t packet;

    mav_array_memcpy(packet.voltage, voltage, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.totalcurrent, totalcurrent, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.count, count, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.temperature, temperature, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_CRC);
}

/**
 * @brief Encode a esc_telemetry_1_to_4 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param esc_telemetry_1_to_4 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_esc_telemetry_1_to_4_t* esc_telemetry_1_to_4)
{
    return mavlink_msg_esc_telemetry_1_to_4_pack(system_id, component_id, msg, esc_telemetry_1_to_4->temperature, esc_telemetry_1_to_4->voltage, esc_telemetry_1_to_4->current, esc_telemetry_1_to_4->totalcurrent, esc_telemetry_1_to_4->rpm, esc_telemetry_1_to_4->count);
}

/**
 * @brief Encode a esc_telemetry_1_to_4 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param esc_telemetry_1_to_4 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_esc_telemetry_1_to_4_t* esc_telemetry_1_to_4)
{
    return mavlink_msg_esc_telemetry_1_to_4_pack_chan(system_id, component_id, chan, msg, esc_telemetry_1_to_4->temperature, esc_telemetry_1_to_4->voltage, esc_telemetry_1_to_4->current, esc_telemetry_1_to_4->totalcurrent, esc_telemetry_1_to_4->rpm, esc_telemetry_1_to_4->count);
}

/**
 * @brief Encode a esc_telemetry_1_to_4 struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param esc_telemetry_1_to_4 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_esc_telemetry_1_to_4_t* esc_telemetry_1_to_4)
{
    return mavlink_msg_esc_telemetry_1_to_4_pack_status(system_id, component_id, _status, msg,  esc_telemetry_1_to_4->temperature, esc_telemetry_1_to_4->voltage, esc_telemetry_1_to_4->current, esc_telemetry_1_to_4->totalcurrent, esc_telemetry_1_to_4->rpm, esc_telemetry_1_to_4->count);
}

/**
 * @brief Send a esc_telemetry_1_to_4 message
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

static inline void mavlink_msg_esc_telemetry_1_to_4_send(mavlink_channel_t chan, const uint8_t *temperature, const uint16_t *voltage, const uint16_t *current, const uint16_t *totalcurrent, const uint16_t *rpm, const uint16_t *count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN];

    _mav_put_uint16_t_array(buf, 0, voltage, 4);
    _mav_put_uint16_t_array(buf, 8, current, 4);
    _mav_put_uint16_t_array(buf, 16, totalcurrent, 4);
    _mav_put_uint16_t_array(buf, 24, rpm, 4);
    _mav_put_uint16_t_array(buf, 32, count, 4);
    _mav_put_uint8_t_array(buf, 40, temperature, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, buf, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_CRC);
#else
    mavlink_esc_telemetry_1_to_4_t packet;

    mav_array_memcpy(packet.voltage, voltage, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.current, current, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.totalcurrent, totalcurrent, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.rpm, rpm, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.count, count, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.temperature, temperature, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, (const char *)&packet, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_CRC);
#endif
}

/**
 * @brief Send a esc_telemetry_1_to_4 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_esc_telemetry_1_to_4_send_struct(mavlink_channel_t chan, const mavlink_esc_telemetry_1_to_4_t* esc_telemetry_1_to_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_esc_telemetry_1_to_4_send(chan, esc_telemetry_1_to_4->temperature, esc_telemetry_1_to_4->voltage, esc_telemetry_1_to_4->current, esc_telemetry_1_to_4->totalcurrent, esc_telemetry_1_to_4->rpm, esc_telemetry_1_to_4->count);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, (const char *)esc_telemetry_1_to_4, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_CRC);
#endif
}

#if MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_esc_telemetry_1_to_4_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *temperature, const uint16_t *voltage, const uint16_t *current, const uint16_t *totalcurrent, const uint16_t *rpm, const uint16_t *count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint16_t_array(buf, 0, voltage, 4);
    _mav_put_uint16_t_array(buf, 8, current, 4);
    _mav_put_uint16_t_array(buf, 16, totalcurrent, 4);
    _mav_put_uint16_t_array(buf, 24, rpm, 4);
    _mav_put_uint16_t_array(buf, 32, count, 4);
    _mav_put_uint8_t_array(buf, 40, temperature, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, buf, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_CRC);
#else
    mavlink_esc_telemetry_1_to_4_t *packet = (mavlink_esc_telemetry_1_to_4_t *)msgbuf;

    mav_array_memcpy(packet->voltage, voltage, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->current, current, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->totalcurrent, totalcurrent, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->rpm, rpm, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->count, count, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->temperature, temperature, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4, (const char *)packet, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_CRC);
#endif
}
#endif

#endif

// MESSAGE ESC_TELEMETRY_1_TO_4 UNPACKING


/**
 * @brief Get field temperature from esc_telemetry_1_to_4 message
 *
 * @return [degC] Temperature.
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_get_temperature(const mavlink_message_t* msg, uint8_t *temperature)
{
    return _MAV_RETURN_uint8_t_array(msg, temperature, 4,  40);
}

/**
 * @brief Get field voltage from esc_telemetry_1_to_4 message
 *
 * @return [cV] Voltage.
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_get_voltage(const mavlink_message_t* msg, uint16_t *voltage)
{
    return _MAV_RETURN_uint16_t_array(msg, voltage, 4,  0);
}

/**
 * @brief Get field current from esc_telemetry_1_to_4 message
 *
 * @return [cA] Current.
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_get_current(const mavlink_message_t* msg, uint16_t *current)
{
    return _MAV_RETURN_uint16_t_array(msg, current, 4,  8);
}

/**
 * @brief Get field totalcurrent from esc_telemetry_1_to_4 message
 *
 * @return [mAh] Total current.
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_get_totalcurrent(const mavlink_message_t* msg, uint16_t *totalcurrent)
{
    return _MAV_RETURN_uint16_t_array(msg, totalcurrent, 4,  16);
}

/**
 * @brief Get field rpm from esc_telemetry_1_to_4 message
 *
 * @return [rpm] RPM (eRPM).
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_get_rpm(const mavlink_message_t* msg, uint16_t *rpm)
{
    return _MAV_RETURN_uint16_t_array(msg, rpm, 4,  24);
}

/**
 * @brief Get field count from esc_telemetry_1_to_4 message
 *
 * @return  count of telemetry packets received (wraps at 65535).
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_to_4_get_count(const mavlink_message_t* msg, uint16_t *count)
{
    return _MAV_RETURN_uint16_t_array(msg, count, 4,  32);
}

/**
 * @brief Decode a esc_telemetry_1_to_4 message into a struct
 *
 * @param msg The message to decode
 * @param esc_telemetry_1_to_4 C-struct to decode the message contents into
 */
static inline void mavlink_msg_esc_telemetry_1_to_4_decode(const mavlink_message_t* msg, mavlink_esc_telemetry_1_to_4_t* esc_telemetry_1_to_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_esc_telemetry_1_to_4_get_voltage(msg, esc_telemetry_1_to_4->voltage);
    mavlink_msg_esc_telemetry_1_to_4_get_current(msg, esc_telemetry_1_to_4->current);
    mavlink_msg_esc_telemetry_1_to_4_get_totalcurrent(msg, esc_telemetry_1_to_4->totalcurrent);
    mavlink_msg_esc_telemetry_1_to_4_get_rpm(msg, esc_telemetry_1_to_4->rpm);
    mavlink_msg_esc_telemetry_1_to_4_get_count(msg, esc_telemetry_1_to_4->count);
    mavlink_msg_esc_telemetry_1_to_4_get_temperature(msg, esc_telemetry_1_to_4->temperature);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN? msg->len : MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN;
        memset(esc_telemetry_1_to_4, 0, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN);
    memcpy(esc_telemetry_1_to_4, _MAV_PAYLOAD(msg), len);
#endif
}
