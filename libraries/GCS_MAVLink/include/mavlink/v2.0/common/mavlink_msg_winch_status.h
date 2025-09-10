#pragma once
// MESSAGE WINCH_STATUS PACKING

#define MAVLINK_MSG_ID_WINCH_STATUS 9005


typedef struct __mavlink_winch_status_t {
 uint64_t time_usec; /*< [us] Timestamp (synced to UNIX time or since system boot).*/
 float line_length; /*< [m] Length of line released. NaN if unknown*/
 float speed; /*< [m/s] Speed line is being released or retracted. Positive values if being released, negative values if being retracted, NaN if unknown*/
 float tension; /*< [kg] Tension on the line. NaN if unknown*/
 float voltage; /*< [V] Voltage of the battery supplying the winch. NaN if unknown*/
 float current; /*< [A] Current draw from the winch. NaN if unknown*/
 uint32_t status; /*<  Status flags*/
 int16_t temperature; /*< [degC] Temperature of the motor. INT16_MAX if unknown*/
} mavlink_winch_status_t;

#define MAVLINK_MSG_ID_WINCH_STATUS_LEN 34
#define MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN 34
#define MAVLINK_MSG_ID_9005_LEN 34
#define MAVLINK_MSG_ID_9005_MIN_LEN 34

#define MAVLINK_MSG_ID_WINCH_STATUS_CRC 117
#define MAVLINK_MSG_ID_9005_CRC 117



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WINCH_STATUS { \
    9005, \
    "WINCH_STATUS", \
    8, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_winch_status_t, time_usec) }, \
         { "line_length", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_winch_status_t, line_length) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_winch_status_t, speed) }, \
         { "tension", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_winch_status_t, tension) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_winch_status_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_winch_status_t, current) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_winch_status_t, temperature) }, \
         { "status", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_winch_status_t, status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WINCH_STATUS { \
    "WINCH_STATUS", \
    8, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_winch_status_t, time_usec) }, \
         { "line_length", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_winch_status_t, line_length) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_winch_status_t, speed) }, \
         { "tension", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_winch_status_t, tension) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_winch_status_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_winch_status_t, current) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_winch_status_t, temperature) }, \
         { "status", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_winch_status_t, status) }, \
         } \
}
#endif

/**
 * @brief Pack a winch_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param line_length [m] Length of line released. NaN if unknown
 * @param speed [m/s] Speed line is being released or retracted. Positive values if being released, negative values if being retracted, NaN if unknown
 * @param tension [kg] Tension on the line. NaN if unknown
 * @param voltage [V] Voltage of the battery supplying the winch. NaN if unknown
 * @param current [A] Current draw from the winch. NaN if unknown
 * @param temperature [degC] Temperature of the motor. INT16_MAX if unknown
 * @param status  Status flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_winch_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINCH_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, line_length);
    _mav_put_float(buf, 12, speed);
    _mav_put_float(buf, 16, tension);
    _mav_put_float(buf, 20, voltage);
    _mav_put_float(buf, 24, current);
    _mav_put_uint32_t(buf, 28, status);
    _mav_put_int16_t(buf, 32, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WINCH_STATUS_LEN);
#else
    mavlink_winch_status_t packet;
    packet.time_usec = time_usec;
    packet.line_length = line_length;
    packet.speed = speed;
    packet.tension = tension;
    packet.voltage = voltage;
    packet.current = current;
    packet.status = status;
    packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WINCH_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WINCH_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_WINCH_STATUS_LEN, MAVLINK_MSG_ID_WINCH_STATUS_CRC);
}

/**
 * @brief Pack a winch_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param line_length [m] Length of line released. NaN if unknown
 * @param speed [m/s] Speed line is being released or retracted. Positive values if being released, negative values if being retracted, NaN if unknown
 * @param tension [kg] Tension on the line. NaN if unknown
 * @param voltage [V] Voltage of the battery supplying the winch. NaN if unknown
 * @param current [A] Current draw from the winch. NaN if unknown
 * @param temperature [degC] Temperature of the motor. INT16_MAX if unknown
 * @param status  Status flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_winch_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINCH_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, line_length);
    _mav_put_float(buf, 12, speed);
    _mav_put_float(buf, 16, tension);
    _mav_put_float(buf, 20, voltage);
    _mav_put_float(buf, 24, current);
    _mav_put_uint32_t(buf, 28, status);
    _mav_put_int16_t(buf, 32, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WINCH_STATUS_LEN);
#else
    mavlink_winch_status_t packet;
    packet.time_usec = time_usec;
    packet.line_length = line_length;
    packet.speed = speed;
    packet.tension = tension;
    packet.voltage = voltage;
    packet.current = current;
    packet.status = status;
    packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WINCH_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WINCH_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_WINCH_STATUS_LEN, MAVLINK_MSG_ID_WINCH_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_WINCH_STATUS_LEN);
#endif
}

/**
 * @brief Pack a winch_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param line_length [m] Length of line released. NaN if unknown
 * @param speed [m/s] Speed line is being released or retracted. Positive values if being released, negative values if being retracted, NaN if unknown
 * @param tension [kg] Tension on the line. NaN if unknown
 * @param voltage [V] Voltage of the battery supplying the winch. NaN if unknown
 * @param current [A] Current draw from the winch. NaN if unknown
 * @param temperature [degC] Temperature of the motor. INT16_MAX if unknown
 * @param status  Status flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_winch_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float line_length,float speed,float tension,float voltage,float current,int16_t temperature,uint32_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINCH_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, line_length);
    _mav_put_float(buf, 12, speed);
    _mav_put_float(buf, 16, tension);
    _mav_put_float(buf, 20, voltage);
    _mav_put_float(buf, 24, current);
    _mav_put_uint32_t(buf, 28, status);
    _mav_put_int16_t(buf, 32, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WINCH_STATUS_LEN);
#else
    mavlink_winch_status_t packet;
    packet.time_usec = time_usec;
    packet.line_length = line_length;
    packet.speed = speed;
    packet.tension = tension;
    packet.voltage = voltage;
    packet.current = current;
    packet.status = status;
    packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WINCH_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WINCH_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_WINCH_STATUS_LEN, MAVLINK_MSG_ID_WINCH_STATUS_CRC);
}

/**
 * @brief Encode a winch_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param winch_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_winch_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_winch_status_t* winch_status)
{
    return mavlink_msg_winch_status_pack(system_id, component_id, msg, winch_status->time_usec, winch_status->line_length, winch_status->speed, winch_status->tension, winch_status->voltage, winch_status->current, winch_status->temperature, winch_status->status);
}

/**
 * @brief Encode a winch_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param winch_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_winch_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_winch_status_t* winch_status)
{
    return mavlink_msg_winch_status_pack_chan(system_id, component_id, chan, msg, winch_status->time_usec, winch_status->line_length, winch_status->speed, winch_status->tension, winch_status->voltage, winch_status->current, winch_status->temperature, winch_status->status);
}

/**
 * @brief Encode a winch_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param winch_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_winch_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_winch_status_t* winch_status)
{
    return mavlink_msg_winch_status_pack_status(system_id, component_id, _status, msg,  winch_status->time_usec, winch_status->line_length, winch_status->speed, winch_status->tension, winch_status->voltage, winch_status->current, winch_status->temperature, winch_status->status);
}

/**
 * @brief Send a winch_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param line_length [m] Length of line released. NaN if unknown
 * @param speed [m/s] Speed line is being released or retracted. Positive values if being released, negative values if being retracted, NaN if unknown
 * @param tension [kg] Tension on the line. NaN if unknown
 * @param voltage [V] Voltage of the battery supplying the winch. NaN if unknown
 * @param current [A] Current draw from the winch. NaN if unknown
 * @param temperature [degC] Temperature of the motor. INT16_MAX if unknown
 * @param status  Status flags
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_winch_status_send(mavlink_channel_t chan, uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WINCH_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, line_length);
    _mav_put_float(buf, 12, speed);
    _mav_put_float(buf, 16, tension);
    _mav_put_float(buf, 20, voltage);
    _mav_put_float(buf, 24, current);
    _mav_put_uint32_t(buf, 28, status);
    _mav_put_int16_t(buf, 32, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_STATUS, buf, MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_WINCH_STATUS_LEN, MAVLINK_MSG_ID_WINCH_STATUS_CRC);
#else
    mavlink_winch_status_t packet;
    packet.time_usec = time_usec;
    packet.line_length = line_length;
    packet.speed = speed;
    packet.tension = tension;
    packet.voltage = voltage;
    packet.current = current;
    packet.status = status;
    packet.temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_STATUS, (const char *)&packet, MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_WINCH_STATUS_LEN, MAVLINK_MSG_ID_WINCH_STATUS_CRC);
#endif
}

/**
 * @brief Send a winch_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_winch_status_send_struct(mavlink_channel_t chan, const mavlink_winch_status_t* winch_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_winch_status_send(chan, winch_status->time_usec, winch_status->line_length, winch_status->speed, winch_status->tension, winch_status->voltage, winch_status->current, winch_status->temperature, winch_status->status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_STATUS, (const char *)winch_status, MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_WINCH_STATUS_LEN, MAVLINK_MSG_ID_WINCH_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_WINCH_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_winch_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float line_length, float speed, float tension, float voltage, float current, int16_t temperature, uint32_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, line_length);
    _mav_put_float(buf, 12, speed);
    _mav_put_float(buf, 16, tension);
    _mav_put_float(buf, 20, voltage);
    _mav_put_float(buf, 24, current);
    _mav_put_uint32_t(buf, 28, status);
    _mav_put_int16_t(buf, 32, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_STATUS, buf, MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_WINCH_STATUS_LEN, MAVLINK_MSG_ID_WINCH_STATUS_CRC);
#else
    mavlink_winch_status_t *packet = (mavlink_winch_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->line_length = line_length;
    packet->speed = speed;
    packet->tension = tension;
    packet->voltage = voltage;
    packet->current = current;
    packet->status = status;
    packet->temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WINCH_STATUS, (const char *)packet, MAVLINK_MSG_ID_WINCH_STATUS_MIN_LEN, MAVLINK_MSG_ID_WINCH_STATUS_LEN, MAVLINK_MSG_ID_WINCH_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE WINCH_STATUS UNPACKING


/**
 * @brief Get field time_usec from winch_status message
 *
 * @return [us] Timestamp (synced to UNIX time or since system boot).
 */
static inline uint64_t mavlink_msg_winch_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field line_length from winch_status message
 *
 * @return [m] Length of line released. NaN if unknown
 */
static inline float mavlink_msg_winch_status_get_line_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field speed from winch_status message
 *
 * @return [m/s] Speed line is being released or retracted. Positive values if being released, negative values if being retracted, NaN if unknown
 */
static inline float mavlink_msg_winch_status_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field tension from winch_status message
 *
 * @return [kg] Tension on the line. NaN if unknown
 */
static inline float mavlink_msg_winch_status_get_tension(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field voltage from winch_status message
 *
 * @return [V] Voltage of the battery supplying the winch. NaN if unknown
 */
static inline float mavlink_msg_winch_status_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field current from winch_status message
 *
 * @return [A] Current draw from the winch. NaN if unknown
 */
static inline float mavlink_msg_winch_status_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field temperature from winch_status message
 *
 * @return [degC] Temperature of the motor. INT16_MAX if unknown
 */
static inline int16_t mavlink_msg_winch_status_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field status from winch_status message
 *
 * @return  Status flags
 */
static inline uint32_t mavlink_msg_winch_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Decode a winch_status message into a struct
 *
 * @param msg The message to decode
 * @param winch_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_winch_status_decode(const mavlink_message_t* msg, mavlink_winch_status_t* winch_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    winch_status->time_usec = mavlink_msg_winch_status_get_time_usec(msg);
    winch_status->line_length = mavlink_msg_winch_status_get_line_length(msg);
    winch_status->speed = mavlink_msg_winch_status_get_speed(msg);
    winch_status->tension = mavlink_msg_winch_status_get_tension(msg);
    winch_status->voltage = mavlink_msg_winch_status_get_voltage(msg);
    winch_status->current = mavlink_msg_winch_status_get_current(msg);
    winch_status->status = mavlink_msg_winch_status_get_status(msg);
    winch_status->temperature = mavlink_msg_winch_status_get_temperature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WINCH_STATUS_LEN? msg->len : MAVLINK_MSG_ID_WINCH_STATUS_LEN;
        memset(winch_status, 0, MAVLINK_MSG_ID_WINCH_STATUS_LEN);
    memcpy(winch_status, _MAV_PAYLOAD(msg), len);
#endif
}
