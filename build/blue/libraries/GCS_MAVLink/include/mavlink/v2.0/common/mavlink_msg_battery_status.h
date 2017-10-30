#pragma once
// MESSAGE BATTERY_STATUS PACKING

#define MAVLINK_MSG_ID_BATTERY_STATUS 147

MAVPACKED(
typedef struct __mavlink_battery_status_t {
 int32_t current_consumed; /*< Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate*/
 int32_t energy_consumed; /*< Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate*/
 int16_t temperature; /*< Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.*/
 uint16_t voltages[10]; /*< Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.*/
 int16_t current_battery; /*< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current*/
 uint8_t id; /*< Battery ID*/
 uint8_t battery_function; /*< Function of the battery*/
 uint8_t type; /*< Type (chemistry) of the battery*/
 int8_t battery_remaining; /*< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery*/
}) mavlink_battery_status_t;

#define MAVLINK_MSG_ID_BATTERY_STATUS_LEN 36
#define MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN 36
#define MAVLINK_MSG_ID_147_LEN 36
#define MAVLINK_MSG_ID_147_MIN_LEN 36

#define MAVLINK_MSG_ID_BATTERY_STATUS_CRC 154
#define MAVLINK_MSG_ID_147_CRC 154

#define MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BATTERY_STATUS { \
    147, \
    "BATTERY_STATUS", \
    9, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_battery_status_t, id) }, \
         { "battery_function", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_battery_status_t, battery_function) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_battery_status_t, type) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_battery_status_t, temperature) }, \
         { "voltages", NULL, MAVLINK_TYPE_UINT16_T, 10, 10, offsetof(mavlink_battery_status_t, voltages) }, \
         { "current_battery", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_battery_status_t, current_battery) }, \
         { "current_consumed", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_battery_status_t, current_consumed) }, \
         { "energy_consumed", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_battery_status_t, energy_consumed) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_INT8_T, 0, 35, offsetof(mavlink_battery_status_t, battery_remaining) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BATTERY_STATUS { \
    "BATTERY_STATUS", \
    9, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_battery_status_t, id) }, \
         { "battery_function", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_battery_status_t, battery_function) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_battery_status_t, type) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_battery_status_t, temperature) }, \
         { "voltages", NULL, MAVLINK_TYPE_UINT16_T, 10, 10, offsetof(mavlink_battery_status_t, voltages) }, \
         { "current_battery", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_battery_status_t, current_battery) }, \
         { "current_consumed", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_battery_status_t, current_consumed) }, \
         { "energy_consumed", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_battery_status_t, energy_consumed) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_INT8_T, 0, 35, offsetof(mavlink_battery_status_t, battery_remaining) }, \
         } \
}
#endif

/**
 * @brief Pack a battery_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id Battery ID
 * @param battery_function Function of the battery
 * @param type Type (chemistry) of the battery
 * @param temperature Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
 * @param voltages Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param current_consumed Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
 * @param energy_consumed Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_STATUS_LEN];
    _mav_put_int32_t(buf, 0, current_consumed);
    _mav_put_int32_t(buf, 4, energy_consumed);
    _mav_put_int16_t(buf, 8, temperature);
    _mav_put_int16_t(buf, 30, current_battery);
    _mav_put_uint8_t(buf, 32, id);
    _mav_put_uint8_t(buf, 33, battery_function);
    _mav_put_uint8_t(buf, 34, type);
    _mav_put_int8_t(buf, 35, battery_remaining);
    _mav_put_uint16_t_array(buf, 10, voltages, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_STATUS_LEN);
#else
    mavlink_battery_status_t packet;
    packet.current_consumed = current_consumed;
    packet.energy_consumed = energy_consumed;
    packet.temperature = temperature;
    packet.current_battery = current_battery;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.battery_remaining = battery_remaining;
    mav_array_memcpy(packet.voltages, voltages, sizeof(uint16_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_CRC);
}

/**
 * @brief Pack a battery_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id Battery ID
 * @param battery_function Function of the battery
 * @param type Type (chemistry) of the battery
 * @param temperature Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
 * @param voltages Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param current_consumed Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
 * @param energy_consumed Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint8_t battery_function,uint8_t type,int16_t temperature,const uint16_t *voltages,int16_t current_battery,int32_t current_consumed,int32_t energy_consumed,int8_t battery_remaining)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_STATUS_LEN];
    _mav_put_int32_t(buf, 0, current_consumed);
    _mav_put_int32_t(buf, 4, energy_consumed);
    _mav_put_int16_t(buf, 8, temperature);
    _mav_put_int16_t(buf, 30, current_battery);
    _mav_put_uint8_t(buf, 32, id);
    _mav_put_uint8_t(buf, 33, battery_function);
    _mav_put_uint8_t(buf, 34, type);
    _mav_put_int8_t(buf, 35, battery_remaining);
    _mav_put_uint16_t_array(buf, 10, voltages, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_STATUS_LEN);
#else
    mavlink_battery_status_t packet;
    packet.current_consumed = current_consumed;
    packet.energy_consumed = energy_consumed;
    packet.temperature = temperature;
    packet.current_battery = current_battery;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.battery_remaining = battery_remaining;
    mav_array_memcpy(packet.voltages, voltages, sizeof(uint16_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_CRC);
}

/**
 * @brief Encode a battery_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param battery_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery_status_t* battery_status)
{
    return mavlink_msg_battery_status_pack(system_id, component_id, msg, battery_status->id, battery_status->battery_function, battery_status->type, battery_status->temperature, battery_status->voltages, battery_status->current_battery, battery_status->current_consumed, battery_status->energy_consumed, battery_status->battery_remaining);
}

/**
 * @brief Encode a battery_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_battery_status_t* battery_status)
{
    return mavlink_msg_battery_status_pack_chan(system_id, component_id, chan, msg, battery_status->id, battery_status->battery_function, battery_status->type, battery_status->temperature, battery_status->voltages, battery_status->current_battery, battery_status->current_consumed, battery_status->energy_consumed, battery_status->battery_remaining);
}

/**
 * @brief Send a battery_status message
 * @param chan MAVLink channel to send the message
 *
 * @param id Battery ID
 * @param battery_function Function of the battery
 * @param type Type (chemistry) of the battery
 * @param temperature Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
 * @param voltages Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param current_consumed Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
 * @param energy_consumed Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_battery_status_send(mavlink_channel_t chan, uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_STATUS_LEN];
    _mav_put_int32_t(buf, 0, current_consumed);
    _mav_put_int32_t(buf, 4, energy_consumed);
    _mav_put_int16_t(buf, 8, temperature);
    _mav_put_int16_t(buf, 30, current_battery);
    _mav_put_uint8_t(buf, 32, id);
    _mav_put_uint8_t(buf, 33, battery_function);
    _mav_put_uint8_t(buf, 34, type);
    _mav_put_int8_t(buf, 35, battery_remaining);
    _mav_put_uint16_t_array(buf, 10, voltages, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_STATUS, buf, MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_CRC);
#else
    mavlink_battery_status_t packet;
    packet.current_consumed = current_consumed;
    packet.energy_consumed = energy_consumed;
    packet.temperature = temperature;
    packet.current_battery = current_battery;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.battery_remaining = battery_remaining;
    mav_array_memcpy(packet.voltages, voltages, sizeof(uint16_t)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_STATUS, (const char *)&packet, MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_CRC);
#endif
}

/**
 * @brief Send a battery_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_battery_status_send_struct(mavlink_channel_t chan, const mavlink_battery_status_t* battery_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_battery_status_send(chan, battery_status->id, battery_status->battery_function, battery_status->type, battery_status->temperature, battery_status->voltages, battery_status->current_battery, battery_status->current_consumed, battery_status->energy_consumed, battery_status->battery_remaining);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_STATUS, (const char *)battery_status, MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_BATTERY_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_battery_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, current_consumed);
    _mav_put_int32_t(buf, 4, energy_consumed);
    _mav_put_int16_t(buf, 8, temperature);
    _mav_put_int16_t(buf, 30, current_battery);
    _mav_put_uint8_t(buf, 32, id);
    _mav_put_uint8_t(buf, 33, battery_function);
    _mav_put_uint8_t(buf, 34, type);
    _mav_put_int8_t(buf, 35, battery_remaining);
    _mav_put_uint16_t_array(buf, 10, voltages, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_STATUS, buf, MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_CRC);
#else
    mavlink_battery_status_t *packet = (mavlink_battery_status_t *)msgbuf;
    packet->current_consumed = current_consumed;
    packet->energy_consumed = energy_consumed;
    packet->temperature = temperature;
    packet->current_battery = current_battery;
    packet->id = id;
    packet->battery_function = battery_function;
    packet->type = type;
    packet->battery_remaining = battery_remaining;
    mav_array_memcpy(packet->voltages, voltages, sizeof(uint16_t)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_STATUS, (const char *)packet, MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE BATTERY_STATUS UNPACKING


/**
 * @brief Get field id from battery_status message
 *
 * @return Battery ID
 */
static inline uint8_t mavlink_msg_battery_status_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field battery_function from battery_status message
 *
 * @return Function of the battery
 */
static inline uint8_t mavlink_msg_battery_status_get_battery_function(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field type from battery_status message
 *
 * @return Type (chemistry) of the battery
 */
static inline uint8_t mavlink_msg_battery_status_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field temperature from battery_status message
 *
 * @return Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
 */
static inline int16_t mavlink_msg_battery_status_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field voltages from battery_status message
 *
 * @return Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
 */
static inline uint16_t mavlink_msg_battery_status_get_voltages(const mavlink_message_t* msg, uint16_t *voltages)
{
    return _MAV_RETURN_uint16_t_array(msg, voltages, 10,  10);
}

/**
 * @brief Get field current_battery from battery_status message
 *
 * @return Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 */
static inline int16_t mavlink_msg_battery_status_get_current_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field current_consumed from battery_status message
 *
 * @return Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
 */
static inline int32_t mavlink_msg_battery_status_get_current_consumed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field energy_consumed from battery_status message
 *
 * @return Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
 */
static inline int32_t mavlink_msg_battery_status_get_energy_consumed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field battery_remaining from battery_status message
 *
 * @return Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
 */
static inline int8_t mavlink_msg_battery_status_get_battery_remaining(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  35);
}

/**
 * @brief Decode a battery_status message into a struct
 *
 * @param msg The message to decode
 * @param battery_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_battery_status_decode(const mavlink_message_t* msg, mavlink_battery_status_t* battery_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    battery_status->current_consumed = mavlink_msg_battery_status_get_current_consumed(msg);
    battery_status->energy_consumed = mavlink_msg_battery_status_get_energy_consumed(msg);
    battery_status->temperature = mavlink_msg_battery_status_get_temperature(msg);
    mavlink_msg_battery_status_get_voltages(msg, battery_status->voltages);
    battery_status->current_battery = mavlink_msg_battery_status_get_current_battery(msg);
    battery_status->id = mavlink_msg_battery_status_get_id(msg);
    battery_status->battery_function = mavlink_msg_battery_status_get_battery_function(msg);
    battery_status->type = mavlink_msg_battery_status_get_type(msg);
    battery_status->battery_remaining = mavlink_msg_battery_status_get_battery_remaining(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BATTERY_STATUS_LEN? msg->len : MAVLINK_MSG_ID_BATTERY_STATUS_LEN;
        memset(battery_status, 0, MAVLINK_MSG_ID_BATTERY_STATUS_LEN);
    memcpy(battery_status, _MAV_PAYLOAD(msg), len);
#endif
}
