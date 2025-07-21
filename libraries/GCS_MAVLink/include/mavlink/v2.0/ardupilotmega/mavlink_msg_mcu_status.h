#pragma once
// MESSAGE MCU_STATUS PACKING

#define MAVLINK_MSG_ID_MCU_STATUS 11039


typedef struct __mavlink_mcu_status_t {
 int16_t MCU_temperature; /*< [cdegC] MCU Internal temperature*/
 uint16_t MCU_voltage; /*< [mV] MCU voltage*/
 uint16_t MCU_voltage_min; /*< [mV] MCU voltage minimum*/
 uint16_t MCU_voltage_max; /*< [mV] MCU voltage maximum*/
 uint8_t id; /*<  MCU instance*/
} mavlink_mcu_status_t;

#define MAVLINK_MSG_ID_MCU_STATUS_LEN 9
#define MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN 9
#define MAVLINK_MSG_ID_11039_LEN 9
#define MAVLINK_MSG_ID_11039_MIN_LEN 9

#define MAVLINK_MSG_ID_MCU_STATUS_CRC 142
#define MAVLINK_MSG_ID_11039_CRC 142



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MCU_STATUS { \
    11039, \
    "MCU_STATUS", \
    5, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_mcu_status_t, id) }, \
         { "MCU_temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_mcu_status_t, MCU_temperature) }, \
         { "MCU_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_mcu_status_t, MCU_voltage) }, \
         { "MCU_voltage_min", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_mcu_status_t, MCU_voltage_min) }, \
         { "MCU_voltage_max", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_mcu_status_t, MCU_voltage_max) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MCU_STATUS { \
    "MCU_STATUS", \
    5, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_mcu_status_t, id) }, \
         { "MCU_temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_mcu_status_t, MCU_temperature) }, \
         { "MCU_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_mcu_status_t, MCU_voltage) }, \
         { "MCU_voltage_min", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_mcu_status_t, MCU_voltage_min) }, \
         { "MCU_voltage_max", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_mcu_status_t, MCU_voltage_max) }, \
         } \
}
#endif

/**
 * @brief Pack a mcu_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  MCU instance
 * @param MCU_temperature [cdegC] MCU Internal temperature
 * @param MCU_voltage [mV] MCU voltage
 * @param MCU_voltage_min [mV] MCU voltage minimum
 * @param MCU_voltage_max [mV] MCU voltage maximum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, int16_t MCU_temperature, uint16_t MCU_voltage, uint16_t MCU_voltage_min, uint16_t MCU_voltage_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_STATUS_LEN];
    _mav_put_int16_t(buf, 0, MCU_temperature);
    _mav_put_uint16_t(buf, 2, MCU_voltage);
    _mav_put_uint16_t(buf, 4, MCU_voltage_min);
    _mav_put_uint16_t(buf, 6, MCU_voltage_max);
    _mav_put_uint8_t(buf, 8, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_STATUS_LEN);
#else
    mavlink_mcu_status_t packet;
    packet.MCU_temperature = MCU_temperature;
    packet.MCU_voltage = MCU_voltage;
    packet.MCU_voltage_min = MCU_voltage_min;
    packet.MCU_voltage_max = MCU_voltage_max;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_STATUS_LEN, MAVLINK_MSG_ID_MCU_STATUS_CRC);
}

/**
 * @brief Pack a mcu_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  MCU instance
 * @param MCU_temperature [cdegC] MCU Internal temperature
 * @param MCU_voltage [mV] MCU voltage
 * @param MCU_voltage_min [mV] MCU voltage minimum
 * @param MCU_voltage_max [mV] MCU voltage maximum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t id, int16_t MCU_temperature, uint16_t MCU_voltage, uint16_t MCU_voltage_min, uint16_t MCU_voltage_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_STATUS_LEN];
    _mav_put_int16_t(buf, 0, MCU_temperature);
    _mav_put_uint16_t(buf, 2, MCU_voltage);
    _mav_put_uint16_t(buf, 4, MCU_voltage_min);
    _mav_put_uint16_t(buf, 6, MCU_voltage_max);
    _mav_put_uint8_t(buf, 8, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_STATUS_LEN);
#else
    mavlink_mcu_status_t packet;
    packet.MCU_temperature = MCU_temperature;
    packet.MCU_voltage = MCU_voltage;
    packet.MCU_voltage_min = MCU_voltage_min;
    packet.MCU_voltage_max = MCU_voltage_max;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_STATUS_LEN, MAVLINK_MSG_ID_MCU_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_STATUS_LEN);
#endif
}

/**
 * @brief Pack a mcu_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  MCU instance
 * @param MCU_temperature [cdegC] MCU Internal temperature
 * @param MCU_voltage [mV] MCU voltage
 * @param MCU_voltage_min [mV] MCU voltage minimum
 * @param MCU_voltage_max [mV] MCU voltage maximum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mcu_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,int16_t MCU_temperature,uint16_t MCU_voltage,uint16_t MCU_voltage_min,uint16_t MCU_voltage_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_STATUS_LEN];
    _mav_put_int16_t(buf, 0, MCU_temperature);
    _mav_put_uint16_t(buf, 2, MCU_voltage);
    _mav_put_uint16_t(buf, 4, MCU_voltage_min);
    _mav_put_uint16_t(buf, 6, MCU_voltage_max);
    _mav_put_uint8_t(buf, 8, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MCU_STATUS_LEN);
#else
    mavlink_mcu_status_t packet;
    packet.MCU_temperature = MCU_temperature;
    packet.MCU_voltage = MCU_voltage;
    packet.MCU_voltage_min = MCU_voltage_min;
    packet.MCU_voltage_max = MCU_voltage_max;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MCU_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MCU_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_STATUS_LEN, MAVLINK_MSG_ID_MCU_STATUS_CRC);
}

/**
 * @brief Encode a mcu_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mcu_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mcu_status_t* mcu_status)
{
    return mavlink_msg_mcu_status_pack(system_id, component_id, msg, mcu_status->id, mcu_status->MCU_temperature, mcu_status->MCU_voltage, mcu_status->MCU_voltage_min, mcu_status->MCU_voltage_max);
}

/**
 * @brief Encode a mcu_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mcu_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mcu_status_t* mcu_status)
{
    return mavlink_msg_mcu_status_pack_chan(system_id, component_id, chan, msg, mcu_status->id, mcu_status->MCU_temperature, mcu_status->MCU_voltage, mcu_status->MCU_voltage_min, mcu_status->MCU_voltage_max);
}

/**
 * @brief Encode a mcu_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mcu_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mcu_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mcu_status_t* mcu_status)
{
    return mavlink_msg_mcu_status_pack_status(system_id, component_id, _status, msg,  mcu_status->id, mcu_status->MCU_temperature, mcu_status->MCU_voltage, mcu_status->MCU_voltage_min, mcu_status->MCU_voltage_max);
}

/**
 * @brief Send a mcu_status message
 * @param chan MAVLink channel to send the message
 *
 * @param id  MCU instance
 * @param MCU_temperature [cdegC] MCU Internal temperature
 * @param MCU_voltage [mV] MCU voltage
 * @param MCU_voltage_min [mV] MCU voltage minimum
 * @param MCU_voltage_max [mV] MCU voltage maximum
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mcu_status_send(mavlink_channel_t chan, uint8_t id, int16_t MCU_temperature, uint16_t MCU_voltage, uint16_t MCU_voltage_min, uint16_t MCU_voltage_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MCU_STATUS_LEN];
    _mav_put_int16_t(buf, 0, MCU_temperature);
    _mav_put_uint16_t(buf, 2, MCU_voltage);
    _mav_put_uint16_t(buf, 4, MCU_voltage_min);
    _mav_put_uint16_t(buf, 6, MCU_voltage_max);
    _mav_put_uint8_t(buf, 8, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_STATUS, buf, MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_STATUS_LEN, MAVLINK_MSG_ID_MCU_STATUS_CRC);
#else
    mavlink_mcu_status_t packet;
    packet.MCU_temperature = MCU_temperature;
    packet.MCU_voltage = MCU_voltage;
    packet.MCU_voltage_min = MCU_voltage_min;
    packet.MCU_voltage_max = MCU_voltage_max;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_STATUS_LEN, MAVLINK_MSG_ID_MCU_STATUS_CRC);
#endif
}

/**
 * @brief Send a mcu_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mcu_status_send_struct(mavlink_channel_t chan, const mavlink_mcu_status_t* mcu_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mcu_status_send(chan, mcu_status->id, mcu_status->MCU_temperature, mcu_status->MCU_voltage, mcu_status->MCU_voltage_min, mcu_status->MCU_voltage_max);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_STATUS, (const char *)mcu_status, MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_STATUS_LEN, MAVLINK_MSG_ID_MCU_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MCU_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mcu_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, int16_t MCU_temperature, uint16_t MCU_voltage, uint16_t MCU_voltage_min, uint16_t MCU_voltage_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, MCU_temperature);
    _mav_put_uint16_t(buf, 2, MCU_voltage);
    _mav_put_uint16_t(buf, 4, MCU_voltage_min);
    _mav_put_uint16_t(buf, 6, MCU_voltage_max);
    _mav_put_uint8_t(buf, 8, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_STATUS, buf, MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_STATUS_LEN, MAVLINK_MSG_ID_MCU_STATUS_CRC);
#else
    mavlink_mcu_status_t *packet = (mavlink_mcu_status_t *)msgbuf;
    packet->MCU_temperature = MCU_temperature;
    packet->MCU_voltage = MCU_voltage;
    packet->MCU_voltage_min = MCU_voltage_min;
    packet->MCU_voltage_max = MCU_voltage_max;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MCU_STATUS, (const char *)packet, MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN, MAVLINK_MSG_ID_MCU_STATUS_LEN, MAVLINK_MSG_ID_MCU_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MCU_STATUS UNPACKING


/**
 * @brief Get field id from mcu_status message
 *
 * @return  MCU instance
 */
static inline uint8_t mavlink_msg_mcu_status_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field MCU_temperature from mcu_status message
 *
 * @return [cdegC] MCU Internal temperature
 */
static inline int16_t mavlink_msg_mcu_status_get_MCU_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field MCU_voltage from mcu_status message
 *
 * @return [mV] MCU voltage
 */
static inline uint16_t mavlink_msg_mcu_status_get_MCU_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field MCU_voltage_min from mcu_status message
 *
 * @return [mV] MCU voltage minimum
 */
static inline uint16_t mavlink_msg_mcu_status_get_MCU_voltage_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field MCU_voltage_max from mcu_status message
 *
 * @return [mV] MCU voltage maximum
 */
static inline uint16_t mavlink_msg_mcu_status_get_MCU_voltage_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a mcu_status message into a struct
 *
 * @param msg The message to decode
 * @param mcu_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mcu_status_decode(const mavlink_message_t* msg, mavlink_mcu_status_t* mcu_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mcu_status->MCU_temperature = mavlink_msg_mcu_status_get_MCU_temperature(msg);
    mcu_status->MCU_voltage = mavlink_msg_mcu_status_get_MCU_voltage(msg);
    mcu_status->MCU_voltage_min = mavlink_msg_mcu_status_get_MCU_voltage_min(msg);
    mcu_status->MCU_voltage_max = mavlink_msg_mcu_status_get_MCU_voltage_max(msg);
    mcu_status->id = mavlink_msg_mcu_status_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MCU_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MCU_STATUS_LEN;
        memset(mcu_status, 0, MAVLINK_MSG_ID_MCU_STATUS_LEN);
    memcpy(mcu_status, _MAV_PAYLOAD(msg), len);
#endif
}
