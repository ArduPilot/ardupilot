#pragma once
// MESSAGE ASLUAV_STATUS PACKING

#define MAVLINK_MSG_ID_ASLUAV_STATUS 8006


typedef struct __mavlink_asluav_status_t {
 float Motor_rpm; /*<   Motor RPM */
 uint8_t LED_status; /*<   Status of the position-indicator LEDs*/
 uint8_t SATCOM_status; /*<   Status of the IRIDIUM satellite communication system*/
 uint8_t Servo_status[8]; /*<   Status vector for up to 8 servos*/
} mavlink_asluav_status_t;

#define MAVLINK_MSG_ID_ASLUAV_STATUS_LEN 14
#define MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN 14
#define MAVLINK_MSG_ID_8006_LEN 14
#define MAVLINK_MSG_ID_8006_MIN_LEN 14

#define MAVLINK_MSG_ID_ASLUAV_STATUS_CRC 97
#define MAVLINK_MSG_ID_8006_CRC 97

#define MAVLINK_MSG_ASLUAV_STATUS_FIELD_SERVO_STATUS_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ASLUAV_STATUS { \
    8006, \
    "ASLUAV_STATUS", \
    4, \
    {  { "LED_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_asluav_status_t, LED_status) }, \
         { "SATCOM_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_asluav_status_t, SATCOM_status) }, \
         { "Servo_status", NULL, MAVLINK_TYPE_UINT8_T, 8, 6, offsetof(mavlink_asluav_status_t, Servo_status) }, \
         { "Motor_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_asluav_status_t, Motor_rpm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ASLUAV_STATUS { \
    "ASLUAV_STATUS", \
    4, \
    {  { "LED_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_asluav_status_t, LED_status) }, \
         { "SATCOM_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_asluav_status_t, SATCOM_status) }, \
         { "Servo_status", NULL, MAVLINK_TYPE_UINT8_T, 8, 6, offsetof(mavlink_asluav_status_t, Servo_status) }, \
         { "Motor_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_asluav_status_t, Motor_rpm) }, \
         } \
}
#endif

/**
 * @brief Pack a asluav_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param LED_status   Status of the position-indicator LEDs
 * @param SATCOM_status   Status of the IRIDIUM satellite communication system
 * @param Servo_status   Status vector for up to 8 servos
 * @param Motor_rpm   Motor RPM 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_asluav_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t LED_status, uint8_t SATCOM_status, const uint8_t *Servo_status, float Motor_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLUAV_STATUS_LEN];
    _mav_put_float(buf, 0, Motor_rpm);
    _mav_put_uint8_t(buf, 4, LED_status);
    _mav_put_uint8_t(buf, 5, SATCOM_status);
    _mav_put_uint8_t_array(buf, 6, Servo_status, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN);
#else
    mavlink_asluav_status_t packet;
    packet.Motor_rpm = Motor_rpm;
    packet.LED_status = LED_status;
    packet.SATCOM_status = SATCOM_status;
    mav_array_memcpy(packet.Servo_status, Servo_status, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASLUAV_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_CRC);
}

/**
 * @brief Pack a asluav_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param LED_status   Status of the position-indicator LEDs
 * @param SATCOM_status   Status of the IRIDIUM satellite communication system
 * @param Servo_status   Status vector for up to 8 servos
 * @param Motor_rpm   Motor RPM 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_asluav_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t LED_status, uint8_t SATCOM_status, const uint8_t *Servo_status, float Motor_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLUAV_STATUS_LEN];
    _mav_put_float(buf, 0, Motor_rpm);
    _mav_put_uint8_t(buf, 4, LED_status);
    _mav_put_uint8_t(buf, 5, SATCOM_status);
    _mav_put_uint8_t_array(buf, 6, Servo_status, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN);
#else
    mavlink_asluav_status_t packet;
    packet.Motor_rpm = Motor_rpm;
    packet.LED_status = LED_status;
    packet.SATCOM_status = SATCOM_status;
    mav_array_memcpy(packet.Servo_status, Servo_status, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASLUAV_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN);
#endif
}

/**
 * @brief Pack a asluav_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param LED_status   Status of the position-indicator LEDs
 * @param SATCOM_status   Status of the IRIDIUM satellite communication system
 * @param Servo_status   Status vector for up to 8 servos
 * @param Motor_rpm   Motor RPM 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_asluav_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t LED_status,uint8_t SATCOM_status,const uint8_t *Servo_status,float Motor_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLUAV_STATUS_LEN];
    _mav_put_float(buf, 0, Motor_rpm);
    _mav_put_uint8_t(buf, 4, LED_status);
    _mav_put_uint8_t(buf, 5, SATCOM_status);
    _mav_put_uint8_t_array(buf, 6, Servo_status, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN);
#else
    mavlink_asluav_status_t packet;
    packet.Motor_rpm = Motor_rpm;
    packet.LED_status = LED_status;
    packet.SATCOM_status = SATCOM_status;
    mav_array_memcpy(packet.Servo_status, Servo_status, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASLUAV_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_CRC);
}

/**
 * @brief Encode a asluav_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param asluav_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_asluav_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_asluav_status_t* asluav_status)
{
    return mavlink_msg_asluav_status_pack(system_id, component_id, msg, asluav_status->LED_status, asluav_status->SATCOM_status, asluav_status->Servo_status, asluav_status->Motor_rpm);
}

/**
 * @brief Encode a asluav_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param asluav_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_asluav_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_asluav_status_t* asluav_status)
{
    return mavlink_msg_asluav_status_pack_chan(system_id, component_id, chan, msg, asluav_status->LED_status, asluav_status->SATCOM_status, asluav_status->Servo_status, asluav_status->Motor_rpm);
}

/**
 * @brief Encode a asluav_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param asluav_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_asluav_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_asluav_status_t* asluav_status)
{
    return mavlink_msg_asluav_status_pack_status(system_id, component_id, _status, msg,  asluav_status->LED_status, asluav_status->SATCOM_status, asluav_status->Servo_status, asluav_status->Motor_rpm);
}

/**
 * @brief Send a asluav_status message
 * @param chan MAVLink channel to send the message
 *
 * @param LED_status   Status of the position-indicator LEDs
 * @param SATCOM_status   Status of the IRIDIUM satellite communication system
 * @param Servo_status   Status vector for up to 8 servos
 * @param Motor_rpm   Motor RPM 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_asluav_status_send(mavlink_channel_t chan, uint8_t LED_status, uint8_t SATCOM_status, const uint8_t *Servo_status, float Motor_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLUAV_STATUS_LEN];
    _mav_put_float(buf, 0, Motor_rpm);
    _mav_put_uint8_t(buf, 4, LED_status);
    _mav_put_uint8_t(buf, 5, SATCOM_status);
    _mav_put_uint8_t_array(buf, 6, Servo_status, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLUAV_STATUS, buf, MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_CRC);
#else
    mavlink_asluav_status_t packet;
    packet.Motor_rpm = Motor_rpm;
    packet.LED_status = LED_status;
    packet.SATCOM_status = SATCOM_status;
    mav_array_memcpy(packet.Servo_status, Servo_status, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLUAV_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_CRC);
#endif
}

/**
 * @brief Send a asluav_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_asluav_status_send_struct(mavlink_channel_t chan, const mavlink_asluav_status_t* asluav_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_asluav_status_send(chan, asluav_status->LED_status, asluav_status->SATCOM_status, asluav_status->Servo_status, asluav_status->Motor_rpm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLUAV_STATUS, (const char *)asluav_status, MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ASLUAV_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_asluav_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t LED_status, uint8_t SATCOM_status, const uint8_t *Servo_status, float Motor_rpm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, Motor_rpm);
    _mav_put_uint8_t(buf, 4, LED_status);
    _mav_put_uint8_t(buf, 5, SATCOM_status);
    _mav_put_uint8_t_array(buf, 6, Servo_status, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLUAV_STATUS, buf, MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_CRC);
#else
    mavlink_asluav_status_t *packet = (mavlink_asluav_status_t *)msgbuf;
    packet->Motor_rpm = Motor_rpm;
    packet->LED_status = LED_status;
    packet->SATCOM_status = SATCOM_status;
    mav_array_memcpy(packet->Servo_status, Servo_status, sizeof(uint8_t)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLUAV_STATUS, (const char *)packet, MAVLINK_MSG_ID_ASLUAV_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN, MAVLINK_MSG_ID_ASLUAV_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ASLUAV_STATUS UNPACKING


/**
 * @brief Get field LED_status from asluav_status message
 *
 * @return   Status of the position-indicator LEDs
 */
static inline uint8_t mavlink_msg_asluav_status_get_LED_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field SATCOM_status from asluav_status message
 *
 * @return   Status of the IRIDIUM satellite communication system
 */
static inline uint8_t mavlink_msg_asluav_status_get_SATCOM_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field Servo_status from asluav_status message
 *
 * @return   Status vector for up to 8 servos
 */
static inline uint16_t mavlink_msg_asluav_status_get_Servo_status(const mavlink_message_t* msg, uint8_t *Servo_status)
{
    return _MAV_RETURN_uint8_t_array(msg, Servo_status, 8,  6);
}

/**
 * @brief Get field Motor_rpm from asluav_status message
 *
 * @return   Motor RPM 
 */
static inline float mavlink_msg_asluav_status_get_Motor_rpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a asluav_status message into a struct
 *
 * @param msg The message to decode
 * @param asluav_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_asluav_status_decode(const mavlink_message_t* msg, mavlink_asluav_status_t* asluav_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    asluav_status->Motor_rpm = mavlink_msg_asluav_status_get_Motor_rpm(msg);
    asluav_status->LED_status = mavlink_msg_asluav_status_get_LED_status(msg);
    asluav_status->SATCOM_status = mavlink_msg_asluav_status_get_SATCOM_status(msg);
    mavlink_msg_asluav_status_get_Servo_status(msg, asluav_status->Servo_status);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ASLUAV_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ASLUAV_STATUS_LEN;
        memset(asluav_status, 0, MAVLINK_MSG_ID_ASLUAV_STATUS_LEN);
    memcpy(asluav_status, _MAV_PAYLOAD(msg), len);
#endif
}
