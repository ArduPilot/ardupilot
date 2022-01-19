#pragma once
// MESSAGE SCALED_IMU3 PACKING

#define MAVLINK_MSG_ID_SCALED_IMU3 129


typedef struct __mavlink_scaled_imu3_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 int16_t xacc; /*< [mG] X acceleration*/
 int16_t yacc; /*< [mG] Y acceleration*/
 int16_t zacc; /*< [mG] Z acceleration*/
 int16_t xgyro; /*< [mrad/s] Angular speed around X axis*/
 int16_t ygyro; /*< [mrad/s] Angular speed around Y axis*/
 int16_t zgyro; /*< [mrad/s] Angular speed around Z axis*/
 int16_t xmag; /*< [mgauss] X Magnetic field*/
 int16_t ymag; /*< [mgauss] Y Magnetic field*/
 int16_t zmag; /*< [mgauss] Z Magnetic field*/
 int16_t temperature; /*< [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).*/
} mavlink_scaled_imu3_t;

#define MAVLINK_MSG_ID_SCALED_IMU3_LEN 24
#define MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN 22
#define MAVLINK_MSG_ID_129_LEN 24
#define MAVLINK_MSG_ID_129_MIN_LEN 22

#define MAVLINK_MSG_ID_SCALED_IMU3_CRC 46
#define MAVLINK_MSG_ID_129_CRC 46



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SCALED_IMU3 { \
    129, \
    "SCALED_IMU3", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_scaled_imu3_t, time_boot_ms) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_scaled_imu3_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_scaled_imu3_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_scaled_imu3_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_scaled_imu3_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_scaled_imu3_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_scaled_imu3_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_scaled_imu3_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_scaled_imu3_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_scaled_imu3_t, zmag) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_scaled_imu3_t, temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SCALED_IMU3 { \
    "SCALED_IMU3", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_scaled_imu3_t, time_boot_ms) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_scaled_imu3_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_scaled_imu3_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_scaled_imu3_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_scaled_imu3_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_scaled_imu3_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_scaled_imu3_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_scaled_imu3_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_scaled_imu3_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_scaled_imu3_t, zmag) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_scaled_imu3_t, temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a scaled_imu3 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param xacc [mG] X acceleration
 * @param yacc [mG] Y acceleration
 * @param zacc [mG] Z acceleration
 * @param xgyro [mrad/s] Angular speed around X axis
 * @param ygyro [mrad/s] Angular speed around Y axis
 * @param zgyro [mrad/s] Angular speed around Z axis
 * @param xmag [mgauss] X Magnetic field
 * @param ymag [mgauss] Y Magnetic field
 * @param zmag [mgauss] Z Magnetic field
 * @param temperature [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_imu3_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCALED_IMU3_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int16_t(buf, 4, xacc);
    _mav_put_int16_t(buf, 6, yacc);
    _mav_put_int16_t(buf, 8, zacc);
    _mav_put_int16_t(buf, 10, xgyro);
    _mav_put_int16_t(buf, 12, ygyro);
    _mav_put_int16_t(buf, 14, zgyro);
    _mav_put_int16_t(buf, 16, xmag);
    _mav_put_int16_t(buf, 18, ymag);
    _mav_put_int16_t(buf, 20, zmag);
    _mav_put_int16_t(buf, 22, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCALED_IMU3_LEN);
#else
    mavlink_scaled_imu3_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;
    packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCALED_IMU3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCALED_IMU3;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN, MAVLINK_MSG_ID_SCALED_IMU3_LEN, MAVLINK_MSG_ID_SCALED_IMU3_CRC);
}

/**
 * @brief Pack a scaled_imu3 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param xacc [mG] X acceleration
 * @param yacc [mG] Y acceleration
 * @param zacc [mG] Z acceleration
 * @param xgyro [mrad/s] Angular speed around X axis
 * @param ygyro [mrad/s] Angular speed around Y axis
 * @param zgyro [mrad/s] Angular speed around Z axis
 * @param xmag [mgauss] X Magnetic field
 * @param ymag [mgauss] Y Magnetic field
 * @param zmag [mgauss] Z Magnetic field
 * @param temperature [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_imu3_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,int16_t xacc,int16_t yacc,int16_t zacc,int16_t xgyro,int16_t ygyro,int16_t zgyro,int16_t xmag,int16_t ymag,int16_t zmag,int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCALED_IMU3_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int16_t(buf, 4, xacc);
    _mav_put_int16_t(buf, 6, yacc);
    _mav_put_int16_t(buf, 8, zacc);
    _mav_put_int16_t(buf, 10, xgyro);
    _mav_put_int16_t(buf, 12, ygyro);
    _mav_put_int16_t(buf, 14, zgyro);
    _mav_put_int16_t(buf, 16, xmag);
    _mav_put_int16_t(buf, 18, ymag);
    _mav_put_int16_t(buf, 20, zmag);
    _mav_put_int16_t(buf, 22, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCALED_IMU3_LEN);
#else
    mavlink_scaled_imu3_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;
    packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCALED_IMU3_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCALED_IMU3;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN, MAVLINK_MSG_ID_SCALED_IMU3_LEN, MAVLINK_MSG_ID_SCALED_IMU3_CRC);
}

/**
 * @brief Encode a scaled_imu3 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scaled_imu3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_imu3_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scaled_imu3_t* scaled_imu3)
{
    return mavlink_msg_scaled_imu3_pack(system_id, component_id, msg, scaled_imu3->time_boot_ms, scaled_imu3->xacc, scaled_imu3->yacc, scaled_imu3->zacc, scaled_imu3->xgyro, scaled_imu3->ygyro, scaled_imu3->zgyro, scaled_imu3->xmag, scaled_imu3->ymag, scaled_imu3->zmag, scaled_imu3->temperature);
}

/**
 * @brief Encode a scaled_imu3 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param scaled_imu3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_imu3_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_scaled_imu3_t* scaled_imu3)
{
    return mavlink_msg_scaled_imu3_pack_chan(system_id, component_id, chan, msg, scaled_imu3->time_boot_ms, scaled_imu3->xacc, scaled_imu3->yacc, scaled_imu3->zacc, scaled_imu3->xgyro, scaled_imu3->ygyro, scaled_imu3->zgyro, scaled_imu3->xmag, scaled_imu3->ymag, scaled_imu3->zmag, scaled_imu3->temperature);
}

/**
 * @brief Send a scaled_imu3 message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param xacc [mG] X acceleration
 * @param yacc [mG] Y acceleration
 * @param zacc [mG] Z acceleration
 * @param xgyro [mrad/s] Angular speed around X axis
 * @param ygyro [mrad/s] Angular speed around Y axis
 * @param zgyro [mrad/s] Angular speed around Z axis
 * @param xmag [mgauss] X Magnetic field
 * @param ymag [mgauss] Y Magnetic field
 * @param zmag [mgauss] Z Magnetic field
 * @param temperature [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_scaled_imu3_send(mavlink_channel_t chan, uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SCALED_IMU3_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int16_t(buf, 4, xacc);
    _mav_put_int16_t(buf, 6, yacc);
    _mav_put_int16_t(buf, 8, zacc);
    _mav_put_int16_t(buf, 10, xgyro);
    _mav_put_int16_t(buf, 12, ygyro);
    _mav_put_int16_t(buf, 14, zgyro);
    _mav_put_int16_t(buf, 16, xmag);
    _mav_put_int16_t(buf, 18, ymag);
    _mav_put_int16_t(buf, 20, zmag);
    _mav_put_int16_t(buf, 22, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU3, buf, MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN, MAVLINK_MSG_ID_SCALED_IMU3_LEN, MAVLINK_MSG_ID_SCALED_IMU3_CRC);
#else
    mavlink_scaled_imu3_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;
    packet.temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU3, (const char *)&packet, MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN, MAVLINK_MSG_ID_SCALED_IMU3_LEN, MAVLINK_MSG_ID_SCALED_IMU3_CRC);
#endif
}

/**
 * @brief Send a scaled_imu3 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_scaled_imu3_send_struct(mavlink_channel_t chan, const mavlink_scaled_imu3_t* scaled_imu3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_scaled_imu3_send(chan, scaled_imu3->time_boot_ms, scaled_imu3->xacc, scaled_imu3->yacc, scaled_imu3->zacc, scaled_imu3->xgyro, scaled_imu3->ygyro, scaled_imu3->zgyro, scaled_imu3->xmag, scaled_imu3->ymag, scaled_imu3->zmag, scaled_imu3->temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU3, (const char *)scaled_imu3, MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN, MAVLINK_MSG_ID_SCALED_IMU3_LEN, MAVLINK_MSG_ID_SCALED_IMU3_CRC);
#endif
}

#if MAVLINK_MSG_ID_SCALED_IMU3_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_scaled_imu3_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int16_t(buf, 4, xacc);
    _mav_put_int16_t(buf, 6, yacc);
    _mav_put_int16_t(buf, 8, zacc);
    _mav_put_int16_t(buf, 10, xgyro);
    _mav_put_int16_t(buf, 12, ygyro);
    _mav_put_int16_t(buf, 14, zgyro);
    _mav_put_int16_t(buf, 16, xmag);
    _mav_put_int16_t(buf, 18, ymag);
    _mav_put_int16_t(buf, 20, zmag);
    _mav_put_int16_t(buf, 22, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU3, buf, MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN, MAVLINK_MSG_ID_SCALED_IMU3_LEN, MAVLINK_MSG_ID_SCALED_IMU3_CRC);
#else
    mavlink_scaled_imu3_t *packet = (mavlink_scaled_imu3_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;
    packet->xgyro = xgyro;
    packet->ygyro = ygyro;
    packet->zgyro = zgyro;
    packet->xmag = xmag;
    packet->ymag = ymag;
    packet->zmag = zmag;
    packet->temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU3, (const char *)packet, MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN, MAVLINK_MSG_ID_SCALED_IMU3_LEN, MAVLINK_MSG_ID_SCALED_IMU3_CRC);
#endif
}
#endif

#endif

// MESSAGE SCALED_IMU3 UNPACKING


/**
 * @brief Get field time_boot_ms from scaled_imu3 message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_scaled_imu3_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field xacc from scaled_imu3 message
 *
 * @return [mG] X acceleration
 */
static inline int16_t mavlink_msg_scaled_imu3_get_xacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field yacc from scaled_imu3 message
 *
 * @return [mG] Y acceleration
 */
static inline int16_t mavlink_msg_scaled_imu3_get_yacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field zacc from scaled_imu3 message
 *
 * @return [mG] Z acceleration
 */
static inline int16_t mavlink_msg_scaled_imu3_get_zacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field xgyro from scaled_imu3 message
 *
 * @return [mrad/s] Angular speed around X axis
 */
static inline int16_t mavlink_msg_scaled_imu3_get_xgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field ygyro from scaled_imu3 message
 *
 * @return [mrad/s] Angular speed around Y axis
 */
static inline int16_t mavlink_msg_scaled_imu3_get_ygyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field zgyro from scaled_imu3 message
 *
 * @return [mrad/s] Angular speed around Z axis
 */
static inline int16_t mavlink_msg_scaled_imu3_get_zgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field xmag from scaled_imu3 message
 *
 * @return [mgauss] X Magnetic field
 */
static inline int16_t mavlink_msg_scaled_imu3_get_xmag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field ymag from scaled_imu3 message
 *
 * @return [mgauss] Y Magnetic field
 */
static inline int16_t mavlink_msg_scaled_imu3_get_ymag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field zmag from scaled_imu3 message
 *
 * @return [mgauss] Z Magnetic field
 */
static inline int16_t mavlink_msg_scaled_imu3_get_zmag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field temperature from scaled_imu3 message
 *
 * @return [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
 */
static inline int16_t mavlink_msg_scaled_imu3_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Decode a scaled_imu3 message into a struct
 *
 * @param msg The message to decode
 * @param scaled_imu3 C-struct to decode the message contents into
 */
static inline void mavlink_msg_scaled_imu3_decode(const mavlink_message_t* msg, mavlink_scaled_imu3_t* scaled_imu3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    scaled_imu3->time_boot_ms = mavlink_msg_scaled_imu3_get_time_boot_ms(msg);
    scaled_imu3->xacc = mavlink_msg_scaled_imu3_get_xacc(msg);
    scaled_imu3->yacc = mavlink_msg_scaled_imu3_get_yacc(msg);
    scaled_imu3->zacc = mavlink_msg_scaled_imu3_get_zacc(msg);
    scaled_imu3->xgyro = mavlink_msg_scaled_imu3_get_xgyro(msg);
    scaled_imu3->ygyro = mavlink_msg_scaled_imu3_get_ygyro(msg);
    scaled_imu3->zgyro = mavlink_msg_scaled_imu3_get_zgyro(msg);
    scaled_imu3->xmag = mavlink_msg_scaled_imu3_get_xmag(msg);
    scaled_imu3->ymag = mavlink_msg_scaled_imu3_get_ymag(msg);
    scaled_imu3->zmag = mavlink_msg_scaled_imu3_get_zmag(msg);
    scaled_imu3->temperature = mavlink_msg_scaled_imu3_get_temperature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SCALED_IMU3_LEN? msg->len : MAVLINK_MSG_ID_SCALED_IMU3_LEN;
        memset(scaled_imu3, 0, MAVLINK_MSG_ID_SCALED_IMU3_LEN);
    memcpy(scaled_imu3, _MAV_PAYLOAD(msg), len);
#endif
}
