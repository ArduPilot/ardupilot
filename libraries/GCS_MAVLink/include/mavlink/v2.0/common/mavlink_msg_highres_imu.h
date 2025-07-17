#pragma once
// MESSAGE HIGHRES_IMU PACKING

#define MAVLINK_MSG_ID_HIGHRES_IMU 105


typedef struct __mavlink_highres_imu_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float xacc; /*< [m/s/s] X acceleration*/
 float yacc; /*< [m/s/s] Y acceleration*/
 float zacc; /*< [m/s/s] Z acceleration*/
 float xgyro; /*< [rad/s] Angular speed around X axis*/
 float ygyro; /*< [rad/s] Angular speed around Y axis*/
 float zgyro; /*< [rad/s] Angular speed around Z axis*/
 float xmag; /*< [gauss] X Magnetic field*/
 float ymag; /*< [gauss] Y Magnetic field*/
 float zmag; /*< [gauss] Z Magnetic field*/
 float abs_pressure; /*< [hPa] Absolute pressure*/
 float diff_pressure; /*< [hPa] Differential pressure*/
 float pressure_alt; /*<  Altitude calculated from pressure*/
 float temperature; /*< [degC] Temperature*/
 uint16_t fields_updated; /*<  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature*/
 uint8_t id; /*<  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)*/
} mavlink_highres_imu_t;

#define MAVLINK_MSG_ID_HIGHRES_IMU_LEN 63
#define MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN 62
#define MAVLINK_MSG_ID_105_LEN 63
#define MAVLINK_MSG_ID_105_MIN_LEN 62

#define MAVLINK_MSG_ID_HIGHRES_IMU_CRC 93
#define MAVLINK_MSG_ID_105_CRC 93



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIGHRES_IMU { \
    105, \
    "HIGHRES_IMU", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_highres_imu_t, time_usec) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_highres_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_highres_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_highres_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_highres_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_highres_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_highres_imu_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_highres_imu_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_highres_imu_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_highres_imu_t, zmag) }, \
         { "abs_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_highres_imu_t, abs_pressure) }, \
         { "diff_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_highres_imu_t, diff_pressure) }, \
         { "pressure_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_highres_imu_t, pressure_alt) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_highres_imu_t, temperature) }, \
         { "fields_updated", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_highres_imu_t, fields_updated) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_highres_imu_t, id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIGHRES_IMU { \
    "HIGHRES_IMU", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_highres_imu_t, time_usec) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_highres_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_highres_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_highres_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_highres_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_highres_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_highres_imu_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_highres_imu_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_highres_imu_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_highres_imu_t, zmag) }, \
         { "abs_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_highres_imu_t, abs_pressure) }, \
         { "diff_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_highres_imu_t, diff_pressure) }, \
         { "pressure_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_highres_imu_t, pressure_alt) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_highres_imu_t, temperature) }, \
         { "fields_updated", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_highres_imu_t, fields_updated) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_highres_imu_t, id) }, \
         } \
}
#endif

/**
 * @brief Pack a highres_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 * @param abs_pressure [hPa] Absolute pressure
 * @param diff_pressure [hPa] Differential pressure
 * @param pressure_alt  Altitude calculated from pressure
 * @param temperature [degC] Temperature
 * @param fields_updated  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
 * @param id  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_highres_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint16_t fields_updated, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIGHRES_IMU_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, xacc);
    _mav_put_float(buf, 12, yacc);
    _mav_put_float(buf, 16, zacc);
    _mav_put_float(buf, 20, xgyro);
    _mav_put_float(buf, 24, ygyro);
    _mav_put_float(buf, 28, zgyro);
    _mav_put_float(buf, 32, xmag);
    _mav_put_float(buf, 36, ymag);
    _mav_put_float(buf, 40, zmag);
    _mav_put_float(buf, 44, abs_pressure);
    _mav_put_float(buf, 48, diff_pressure);
    _mav_put_float(buf, 52, pressure_alt);
    _mav_put_float(buf, 56, temperature);
    _mav_put_uint16_t(buf, 60, fields_updated);
    _mav_put_uint8_t(buf, 62, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIGHRES_IMU_LEN);
#else
    mavlink_highres_imu_t packet;
    packet.time_usec = time_usec;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;
    packet.abs_pressure = abs_pressure;
    packet.diff_pressure = diff_pressure;
    packet.pressure_alt = pressure_alt;
    packet.temperature = temperature;
    packet.fields_updated = fields_updated;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIGHRES_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIGHRES_IMU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_CRC);
}

/**
 * @brief Pack a highres_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 * @param abs_pressure [hPa] Absolute pressure
 * @param diff_pressure [hPa] Differential pressure
 * @param pressure_alt  Altitude calculated from pressure
 * @param temperature [degC] Temperature
 * @param fields_updated  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
 * @param id  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_highres_imu_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint16_t fields_updated, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIGHRES_IMU_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, xacc);
    _mav_put_float(buf, 12, yacc);
    _mav_put_float(buf, 16, zacc);
    _mav_put_float(buf, 20, xgyro);
    _mav_put_float(buf, 24, ygyro);
    _mav_put_float(buf, 28, zgyro);
    _mav_put_float(buf, 32, xmag);
    _mav_put_float(buf, 36, ymag);
    _mav_put_float(buf, 40, zmag);
    _mav_put_float(buf, 44, abs_pressure);
    _mav_put_float(buf, 48, diff_pressure);
    _mav_put_float(buf, 52, pressure_alt);
    _mav_put_float(buf, 56, temperature);
    _mav_put_uint16_t(buf, 60, fields_updated);
    _mav_put_uint8_t(buf, 62, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIGHRES_IMU_LEN);
#else
    mavlink_highres_imu_t packet;
    packet.time_usec = time_usec;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;
    packet.abs_pressure = abs_pressure;
    packet.diff_pressure = diff_pressure;
    packet.pressure_alt = pressure_alt;
    packet.temperature = temperature;
    packet.fields_updated = fields_updated;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIGHRES_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIGHRES_IMU;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_LEN);
#endif
}

/**
 * @brief Pack a highres_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 * @param abs_pressure [hPa] Absolute pressure
 * @param diff_pressure [hPa] Differential pressure
 * @param pressure_alt  Altitude calculated from pressure
 * @param temperature [degC] Temperature
 * @param fields_updated  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
 * @param id  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_highres_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint16_t fields_updated,uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIGHRES_IMU_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, xacc);
    _mav_put_float(buf, 12, yacc);
    _mav_put_float(buf, 16, zacc);
    _mav_put_float(buf, 20, xgyro);
    _mav_put_float(buf, 24, ygyro);
    _mav_put_float(buf, 28, zgyro);
    _mav_put_float(buf, 32, xmag);
    _mav_put_float(buf, 36, ymag);
    _mav_put_float(buf, 40, zmag);
    _mav_put_float(buf, 44, abs_pressure);
    _mav_put_float(buf, 48, diff_pressure);
    _mav_put_float(buf, 52, pressure_alt);
    _mav_put_float(buf, 56, temperature);
    _mav_put_uint16_t(buf, 60, fields_updated);
    _mav_put_uint8_t(buf, 62, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIGHRES_IMU_LEN);
#else
    mavlink_highres_imu_t packet;
    packet.time_usec = time_usec;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;
    packet.abs_pressure = abs_pressure;
    packet.diff_pressure = diff_pressure;
    packet.pressure_alt = pressure_alt;
    packet.temperature = temperature;
    packet.fields_updated = fields_updated;
    packet.id = id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIGHRES_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIGHRES_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_CRC);
}

/**
 * @brief Encode a highres_imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param highres_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_highres_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_highres_imu_t* highres_imu)
{
    return mavlink_msg_highres_imu_pack(system_id, component_id, msg, highres_imu->time_usec, highres_imu->xacc, highres_imu->yacc, highres_imu->zacc, highres_imu->xgyro, highres_imu->ygyro, highres_imu->zgyro, highres_imu->xmag, highres_imu->ymag, highres_imu->zmag, highres_imu->abs_pressure, highres_imu->diff_pressure, highres_imu->pressure_alt, highres_imu->temperature, highres_imu->fields_updated, highres_imu->id);
}

/**
 * @brief Encode a highres_imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param highres_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_highres_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_highres_imu_t* highres_imu)
{
    return mavlink_msg_highres_imu_pack_chan(system_id, component_id, chan, msg, highres_imu->time_usec, highres_imu->xacc, highres_imu->yacc, highres_imu->zacc, highres_imu->xgyro, highres_imu->ygyro, highres_imu->zgyro, highres_imu->xmag, highres_imu->ymag, highres_imu->zmag, highres_imu->abs_pressure, highres_imu->diff_pressure, highres_imu->pressure_alt, highres_imu->temperature, highres_imu->fields_updated, highres_imu->id);
}

/**
 * @brief Encode a highres_imu struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param highres_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_highres_imu_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_highres_imu_t* highres_imu)
{
    return mavlink_msg_highres_imu_pack_status(system_id, component_id, _status, msg,  highres_imu->time_usec, highres_imu->xacc, highres_imu->yacc, highres_imu->zacc, highres_imu->xgyro, highres_imu->ygyro, highres_imu->zgyro, highres_imu->xmag, highres_imu->ymag, highres_imu->zmag, highres_imu->abs_pressure, highres_imu->diff_pressure, highres_imu->pressure_alt, highres_imu->temperature, highres_imu->fields_updated, highres_imu->id);
}

/**
 * @brief Send a highres_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis
 * @param ygyro [rad/s] Angular speed around Y axis
 * @param zgyro [rad/s] Angular speed around Z axis
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 * @param abs_pressure [hPa] Absolute pressure
 * @param diff_pressure [hPa] Differential pressure
 * @param pressure_alt  Altitude calculated from pressure
 * @param temperature [degC] Temperature
 * @param fields_updated  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
 * @param id  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_highres_imu_send(mavlink_channel_t chan, uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint16_t fields_updated, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIGHRES_IMU_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, xacc);
    _mav_put_float(buf, 12, yacc);
    _mav_put_float(buf, 16, zacc);
    _mav_put_float(buf, 20, xgyro);
    _mav_put_float(buf, 24, ygyro);
    _mav_put_float(buf, 28, zgyro);
    _mav_put_float(buf, 32, xmag);
    _mav_put_float(buf, 36, ymag);
    _mav_put_float(buf, 40, zmag);
    _mav_put_float(buf, 44, abs_pressure);
    _mav_put_float(buf, 48, diff_pressure);
    _mav_put_float(buf, 52, pressure_alt);
    _mav_put_float(buf, 56, temperature);
    _mav_put_uint16_t(buf, 60, fields_updated);
    _mav_put_uint8_t(buf, 62, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGHRES_IMU, buf, MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_CRC);
#else
    mavlink_highres_imu_t packet;
    packet.time_usec = time_usec;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;
    packet.abs_pressure = abs_pressure;
    packet.diff_pressure = diff_pressure;
    packet.pressure_alt = pressure_alt;
    packet.temperature = temperature;
    packet.fields_updated = fields_updated;
    packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGHRES_IMU, (const char *)&packet, MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_CRC);
#endif
}

/**
 * @brief Send a highres_imu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_highres_imu_send_struct(mavlink_channel_t chan, const mavlink_highres_imu_t* highres_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_highres_imu_send(chan, highres_imu->time_usec, highres_imu->xacc, highres_imu->yacc, highres_imu->zacc, highres_imu->xgyro, highres_imu->ygyro, highres_imu->zgyro, highres_imu->xmag, highres_imu->ymag, highres_imu->zmag, highres_imu->abs_pressure, highres_imu->diff_pressure, highres_imu->pressure_alt, highres_imu->temperature, highres_imu->fields_updated, highres_imu->id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGHRES_IMU, (const char *)highres_imu, MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIGHRES_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_highres_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint16_t fields_updated, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, xacc);
    _mav_put_float(buf, 12, yacc);
    _mav_put_float(buf, 16, zacc);
    _mav_put_float(buf, 20, xgyro);
    _mav_put_float(buf, 24, ygyro);
    _mav_put_float(buf, 28, zgyro);
    _mav_put_float(buf, 32, xmag);
    _mav_put_float(buf, 36, ymag);
    _mav_put_float(buf, 40, zmag);
    _mav_put_float(buf, 44, abs_pressure);
    _mav_put_float(buf, 48, diff_pressure);
    _mav_put_float(buf, 52, pressure_alt);
    _mav_put_float(buf, 56, temperature);
    _mav_put_uint16_t(buf, 60, fields_updated);
    _mav_put_uint8_t(buf, 62, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGHRES_IMU, buf, MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_CRC);
#else
    mavlink_highres_imu_t *packet = (mavlink_highres_imu_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;
    packet->xgyro = xgyro;
    packet->ygyro = ygyro;
    packet->zgyro = zgyro;
    packet->xmag = xmag;
    packet->ymag = ymag;
    packet->zmag = zmag;
    packet->abs_pressure = abs_pressure;
    packet->diff_pressure = diff_pressure;
    packet->pressure_alt = pressure_alt;
    packet->temperature = temperature;
    packet->fields_updated = fields_updated;
    packet->id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGHRES_IMU, (const char *)packet, MAVLINK_MSG_ID_HIGHRES_IMU_MIN_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_LEN, MAVLINK_MSG_ID_HIGHRES_IMU_CRC);
#endif
}
#endif

#endif

// MESSAGE HIGHRES_IMU UNPACKING


/**
 * @brief Get field time_usec from highres_imu message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_highres_imu_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field xacc from highres_imu message
 *
 * @return [m/s/s] X acceleration
 */
static inline float mavlink_msg_highres_imu_get_xacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yacc from highres_imu message
 *
 * @return [m/s/s] Y acceleration
 */
static inline float mavlink_msg_highres_imu_get_yacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field zacc from highres_imu message
 *
 * @return [m/s/s] Z acceleration
 */
static inline float mavlink_msg_highres_imu_get_zacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field xgyro from highres_imu message
 *
 * @return [rad/s] Angular speed around X axis
 */
static inline float mavlink_msg_highres_imu_get_xgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ygyro from highres_imu message
 *
 * @return [rad/s] Angular speed around Y axis
 */
static inline float mavlink_msg_highres_imu_get_ygyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field zgyro from highres_imu message
 *
 * @return [rad/s] Angular speed around Z axis
 */
static inline float mavlink_msg_highres_imu_get_zgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field xmag from highres_imu message
 *
 * @return [gauss] X Magnetic field
 */
static inline float mavlink_msg_highres_imu_get_xmag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field ymag from highres_imu message
 *
 * @return [gauss] Y Magnetic field
 */
static inline float mavlink_msg_highres_imu_get_ymag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field zmag from highres_imu message
 *
 * @return [gauss] Z Magnetic field
 */
static inline float mavlink_msg_highres_imu_get_zmag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field abs_pressure from highres_imu message
 *
 * @return [hPa] Absolute pressure
 */
static inline float mavlink_msg_highres_imu_get_abs_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field diff_pressure from highres_imu message
 *
 * @return [hPa] Differential pressure
 */
static inline float mavlink_msg_highres_imu_get_diff_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field pressure_alt from highres_imu message
 *
 * @return  Altitude calculated from pressure
 */
static inline float mavlink_msg_highres_imu_get_pressure_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field temperature from highres_imu message
 *
 * @return [degC] Temperature
 */
static inline float mavlink_msg_highres_imu_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field fields_updated from highres_imu message
 *
 * @return  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
 */
static inline uint16_t mavlink_msg_highres_imu_get_fields_updated(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  60);
}

/**
 * @brief Get field id from highres_imu message
 *
 * @return  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
 */
static inline uint8_t mavlink_msg_highres_imu_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  62);
}

/**
 * @brief Decode a highres_imu message into a struct
 *
 * @param msg The message to decode
 * @param highres_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_highres_imu_decode(const mavlink_message_t* msg, mavlink_highres_imu_t* highres_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    highres_imu->time_usec = mavlink_msg_highres_imu_get_time_usec(msg);
    highres_imu->xacc = mavlink_msg_highres_imu_get_xacc(msg);
    highres_imu->yacc = mavlink_msg_highres_imu_get_yacc(msg);
    highres_imu->zacc = mavlink_msg_highres_imu_get_zacc(msg);
    highres_imu->xgyro = mavlink_msg_highres_imu_get_xgyro(msg);
    highres_imu->ygyro = mavlink_msg_highres_imu_get_ygyro(msg);
    highres_imu->zgyro = mavlink_msg_highres_imu_get_zgyro(msg);
    highres_imu->xmag = mavlink_msg_highres_imu_get_xmag(msg);
    highres_imu->ymag = mavlink_msg_highres_imu_get_ymag(msg);
    highres_imu->zmag = mavlink_msg_highres_imu_get_zmag(msg);
    highres_imu->abs_pressure = mavlink_msg_highres_imu_get_abs_pressure(msg);
    highres_imu->diff_pressure = mavlink_msg_highres_imu_get_diff_pressure(msg);
    highres_imu->pressure_alt = mavlink_msg_highres_imu_get_pressure_alt(msg);
    highres_imu->temperature = mavlink_msg_highres_imu_get_temperature(msg);
    highres_imu->fields_updated = mavlink_msg_highres_imu_get_fields_updated(msg);
    highres_imu->id = mavlink_msg_highres_imu_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIGHRES_IMU_LEN? msg->len : MAVLINK_MSG_ID_HIGHRES_IMU_LEN;
        memset(highres_imu, 0, MAVLINK_MSG_ID_HIGHRES_IMU_LEN);
    memcpy(highres_imu, _MAV_PAYLOAD(msg), len);
#endif
}
