#pragma once
// MESSAGE HIL_SENSOR PACKING

#define MAVLINK_MSG_ID_HIL_SENSOR 107


typedef struct __mavlink_hil_sensor_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float xacc; /*< [m/s/s] X acceleration*/
 float yacc; /*< [m/s/s] Y acceleration*/
 float zacc; /*< [m/s/s] Z acceleration*/
 float xgyro; /*< [rad/s] Angular speed around X axis in body frame*/
 float ygyro; /*< [rad/s] Angular speed around Y axis in body frame*/
 float zgyro; /*< [rad/s] Angular speed around Z axis in body frame*/
 float xmag; /*< [gauss] X Magnetic field*/
 float ymag; /*< [gauss] Y Magnetic field*/
 float zmag; /*< [gauss] Z Magnetic field*/
 float abs_pressure; /*< [hPa] Absolute pressure*/
 float diff_pressure; /*< [hPa] Differential pressure (airspeed)*/
 float pressure_alt; /*<  Altitude calculated from pressure*/
 float temperature; /*< [degC] Temperature*/
 uint32_t fields_updated; /*<  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.*/
 uint8_t id; /*<  Sensor ID (zero indexed). Used for multiple sensor inputs*/
} mavlink_hil_sensor_t;

#define MAVLINK_MSG_ID_HIL_SENSOR_LEN 65
#define MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN 64
#define MAVLINK_MSG_ID_107_LEN 65
#define MAVLINK_MSG_ID_107_MIN_LEN 64

#define MAVLINK_MSG_ID_HIL_SENSOR_CRC 108
#define MAVLINK_MSG_ID_107_CRC 108



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIL_SENSOR { \
    107, \
    "HIL_SENSOR", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_sensor_t, time_usec) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_sensor_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_sensor_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_sensor_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_sensor_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_sensor_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_sensor_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hil_sensor_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hil_sensor_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_hil_sensor_t, zmag) }, \
         { "abs_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_hil_sensor_t, abs_pressure) }, \
         { "diff_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_hil_sensor_t, diff_pressure) }, \
         { "pressure_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_hil_sensor_t, pressure_alt) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_hil_sensor_t, temperature) }, \
         { "fields_updated", NULL, MAVLINK_TYPE_UINT32_T, 0, 60, offsetof(mavlink_hil_sensor_t, fields_updated) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_hil_sensor_t, id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIL_SENSOR { \
    "HIL_SENSOR", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_sensor_t, time_usec) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_sensor_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_sensor_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_sensor_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_sensor_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_sensor_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_sensor_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hil_sensor_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hil_sensor_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_hil_sensor_t, zmag) }, \
         { "abs_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_hil_sensor_t, abs_pressure) }, \
         { "diff_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_hil_sensor_t, diff_pressure) }, \
         { "pressure_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_hil_sensor_t, pressure_alt) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_hil_sensor_t, temperature) }, \
         { "fields_updated", NULL, MAVLINK_TYPE_UINT32_T, 0, 60, offsetof(mavlink_hil_sensor_t, fields_updated) }, \
         { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_hil_sensor_t, id) }, \
         } \
}
#endif

/**
 * @brief Pack a hil_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis in body frame
 * @param ygyro [rad/s] Angular speed around Y axis in body frame
 * @param zgyro [rad/s] Angular speed around Z axis in body frame
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 * @param abs_pressure [hPa] Absolute pressure
 * @param diff_pressure [hPa] Differential pressure (airspeed)
 * @param pressure_alt  Altitude calculated from pressure
 * @param temperature [degC] Temperature
 * @param fields_updated  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 * @param id  Sensor ID (zero indexed). Used for multiple sensor inputs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_SENSOR_LEN];
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
    _mav_put_uint32_t(buf, 60, fields_updated);
    _mav_put_uint8_t(buf, 64, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_SENSOR_LEN);
#else
    mavlink_hil_sensor_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_SENSOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HIL_SENSOR_LEN, MAVLINK_MSG_ID_HIL_SENSOR_CRC);
}

/**
 * @brief Pack a hil_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis in body frame
 * @param ygyro [rad/s] Angular speed around Y axis in body frame
 * @param zgyro [rad/s] Angular speed around Z axis in body frame
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 * @param abs_pressure [hPa] Absolute pressure
 * @param diff_pressure [hPa] Differential pressure (airspeed)
 * @param pressure_alt  Altitude calculated from pressure
 * @param temperature [degC] Temperature
 * @param fields_updated  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 * @param id  Sensor ID (zero indexed). Used for multiple sensor inputs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_sensor_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_SENSOR_LEN];
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
    _mav_put_uint32_t(buf, 60, fields_updated);
    _mav_put_uint8_t(buf, 64, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_SENSOR_LEN);
#else
    mavlink_hil_sensor_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_SENSOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HIL_SENSOR_LEN, MAVLINK_MSG_ID_HIL_SENSOR_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HIL_SENSOR_LEN);
#endif
}

/**
 * @brief Pack a hil_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis in body frame
 * @param ygyro [rad/s] Angular speed around Y axis in body frame
 * @param zgyro [rad/s] Angular speed around Z axis in body frame
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 * @param abs_pressure [hPa] Absolute pressure
 * @param diff_pressure [hPa] Differential pressure (airspeed)
 * @param pressure_alt  Altitude calculated from pressure
 * @param temperature [degC] Temperature
 * @param fields_updated  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 * @param id  Sensor ID (zero indexed). Used for multiple sensor inputs
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated,uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_SENSOR_LEN];
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
    _mav_put_uint32_t(buf, 60, fields_updated);
    _mav_put_uint8_t(buf, 64, id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_SENSOR_LEN);
#else
    mavlink_hil_sensor_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_SENSOR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_SENSOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HIL_SENSOR_LEN, MAVLINK_MSG_ID_HIL_SENSOR_CRC);
}

/**
 * @brief Encode a hil_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_sensor_t* hil_sensor)
{
    return mavlink_msg_hil_sensor_pack(system_id, component_id, msg, hil_sensor->time_usec, hil_sensor->xacc, hil_sensor->yacc, hil_sensor->zacc, hil_sensor->xgyro, hil_sensor->ygyro, hil_sensor->zgyro, hil_sensor->xmag, hil_sensor->ymag, hil_sensor->zmag, hil_sensor->abs_pressure, hil_sensor->diff_pressure, hil_sensor->pressure_alt, hil_sensor->temperature, hil_sensor->fields_updated, hil_sensor->id);
}

/**
 * @brief Encode a hil_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hil_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hil_sensor_t* hil_sensor)
{
    return mavlink_msg_hil_sensor_pack_chan(system_id, component_id, chan, msg, hil_sensor->time_usec, hil_sensor->xacc, hil_sensor->yacc, hil_sensor->zacc, hil_sensor->xgyro, hil_sensor->ygyro, hil_sensor->zgyro, hil_sensor->xmag, hil_sensor->ymag, hil_sensor->zmag, hil_sensor->abs_pressure, hil_sensor->diff_pressure, hil_sensor->pressure_alt, hil_sensor->temperature, hil_sensor->fields_updated, hil_sensor->id);
}

/**
 * @brief Encode a hil_sensor struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param hil_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_sensor_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_hil_sensor_t* hil_sensor)
{
    return mavlink_msg_hil_sensor_pack_status(system_id, component_id, _status, msg,  hil_sensor->time_usec, hil_sensor->xacc, hil_sensor->yacc, hil_sensor->zacc, hil_sensor->xgyro, hil_sensor->ygyro, hil_sensor->zgyro, hil_sensor->xmag, hil_sensor->ymag, hil_sensor->zmag, hil_sensor->abs_pressure, hil_sensor->diff_pressure, hil_sensor->pressure_alt, hil_sensor->temperature, hil_sensor->fields_updated, hil_sensor->id);
}

/**
 * @brief Send a hil_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis in body frame
 * @param ygyro [rad/s] Angular speed around Y axis in body frame
 * @param zgyro [rad/s] Angular speed around Z axis in body frame
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 * @param abs_pressure [hPa] Absolute pressure
 * @param diff_pressure [hPa] Differential pressure (airspeed)
 * @param pressure_alt  Altitude calculated from pressure
 * @param temperature [degC] Temperature
 * @param fields_updated  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 * @param id  Sensor ID (zero indexed). Used for multiple sensor inputs
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_sensor_send(mavlink_channel_t chan, uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, uint8_t id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_SENSOR_LEN];
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
    _mav_put_uint32_t(buf, 60, fields_updated);
    _mav_put_uint8_t(buf, 64, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_SENSOR, buf, MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HIL_SENSOR_LEN, MAVLINK_MSG_ID_HIL_SENSOR_CRC);
#else
    mavlink_hil_sensor_t packet;
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

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HIL_SENSOR_LEN, MAVLINK_MSG_ID_HIL_SENSOR_CRC);
#endif
}

/**
 * @brief Send a hil_sensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hil_sensor_send_struct(mavlink_channel_t chan, const mavlink_hil_sensor_t* hil_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hil_sensor_send(chan, hil_sensor->time_usec, hil_sensor->xacc, hil_sensor->yacc, hil_sensor->zacc, hil_sensor->xgyro, hil_sensor->ygyro, hil_sensor->zgyro, hil_sensor->xmag, hil_sensor->ymag, hil_sensor->zmag, hil_sensor->abs_pressure, hil_sensor->diff_pressure, hil_sensor->pressure_alt, hil_sensor->temperature, hil_sensor->fields_updated, hil_sensor->id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_SENSOR, (const char *)hil_sensor, MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HIL_SENSOR_LEN, MAVLINK_MSG_ID_HIL_SENSOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIL_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hil_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint32_t fields_updated, uint8_t id)
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
    _mav_put_uint32_t(buf, 60, fields_updated);
    _mav_put_uint8_t(buf, 64, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_SENSOR, buf, MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HIL_SENSOR_LEN, MAVLINK_MSG_ID_HIL_SENSOR_CRC);
#else
    mavlink_hil_sensor_t *packet = (mavlink_hil_sensor_t *)msgbuf;
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

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_SENSOR, (const char *)packet, MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HIL_SENSOR_LEN, MAVLINK_MSG_ID_HIL_SENSOR_CRC);
#endif
}
#endif

#endif

// MESSAGE HIL_SENSOR UNPACKING


/**
 * @brief Get field time_usec from hil_sensor message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_hil_sensor_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field xacc from hil_sensor message
 *
 * @return [m/s/s] X acceleration
 */
static inline float mavlink_msg_hil_sensor_get_xacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yacc from hil_sensor message
 *
 * @return [m/s/s] Y acceleration
 */
static inline float mavlink_msg_hil_sensor_get_yacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field zacc from hil_sensor message
 *
 * @return [m/s/s] Z acceleration
 */
static inline float mavlink_msg_hil_sensor_get_zacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field xgyro from hil_sensor message
 *
 * @return [rad/s] Angular speed around X axis in body frame
 */
static inline float mavlink_msg_hil_sensor_get_xgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ygyro from hil_sensor message
 *
 * @return [rad/s] Angular speed around Y axis in body frame
 */
static inline float mavlink_msg_hil_sensor_get_ygyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field zgyro from hil_sensor message
 *
 * @return [rad/s] Angular speed around Z axis in body frame
 */
static inline float mavlink_msg_hil_sensor_get_zgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field xmag from hil_sensor message
 *
 * @return [gauss] X Magnetic field
 */
static inline float mavlink_msg_hil_sensor_get_xmag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field ymag from hil_sensor message
 *
 * @return [gauss] Y Magnetic field
 */
static inline float mavlink_msg_hil_sensor_get_ymag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field zmag from hil_sensor message
 *
 * @return [gauss] Z Magnetic field
 */
static inline float mavlink_msg_hil_sensor_get_zmag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field abs_pressure from hil_sensor message
 *
 * @return [hPa] Absolute pressure
 */
static inline float mavlink_msg_hil_sensor_get_abs_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field diff_pressure from hil_sensor message
 *
 * @return [hPa] Differential pressure (airspeed)
 */
static inline float mavlink_msg_hil_sensor_get_diff_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field pressure_alt from hil_sensor message
 *
 * @return  Altitude calculated from pressure
 */
static inline float mavlink_msg_hil_sensor_get_pressure_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field temperature from hil_sensor message
 *
 * @return [degC] Temperature
 */
static inline float mavlink_msg_hil_sensor_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field fields_updated from hil_sensor message
 *
 * @return  Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
 */
static inline uint32_t mavlink_msg_hil_sensor_get_fields_updated(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  60);
}

/**
 * @brief Get field id from hil_sensor message
 *
 * @return  Sensor ID (zero indexed). Used for multiple sensor inputs
 */
static inline uint8_t mavlink_msg_hil_sensor_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  64);
}

/**
 * @brief Decode a hil_sensor message into a struct
 *
 * @param msg The message to decode
 * @param hil_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_sensor_decode(const mavlink_message_t* msg, mavlink_hil_sensor_t* hil_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hil_sensor->time_usec = mavlink_msg_hil_sensor_get_time_usec(msg);
    hil_sensor->xacc = mavlink_msg_hil_sensor_get_xacc(msg);
    hil_sensor->yacc = mavlink_msg_hil_sensor_get_yacc(msg);
    hil_sensor->zacc = mavlink_msg_hil_sensor_get_zacc(msg);
    hil_sensor->xgyro = mavlink_msg_hil_sensor_get_xgyro(msg);
    hil_sensor->ygyro = mavlink_msg_hil_sensor_get_ygyro(msg);
    hil_sensor->zgyro = mavlink_msg_hil_sensor_get_zgyro(msg);
    hil_sensor->xmag = mavlink_msg_hil_sensor_get_xmag(msg);
    hil_sensor->ymag = mavlink_msg_hil_sensor_get_ymag(msg);
    hil_sensor->zmag = mavlink_msg_hil_sensor_get_zmag(msg);
    hil_sensor->abs_pressure = mavlink_msg_hil_sensor_get_abs_pressure(msg);
    hil_sensor->diff_pressure = mavlink_msg_hil_sensor_get_diff_pressure(msg);
    hil_sensor->pressure_alt = mavlink_msg_hil_sensor_get_pressure_alt(msg);
    hil_sensor->temperature = mavlink_msg_hil_sensor_get_temperature(msg);
    hil_sensor->fields_updated = mavlink_msg_hil_sensor_get_fields_updated(msg);
    hil_sensor->id = mavlink_msg_hil_sensor_get_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIL_SENSOR_LEN? msg->len : MAVLINK_MSG_ID_HIL_SENSOR_LEN;
        memset(hil_sensor, 0, MAVLINK_MSG_ID_HIL_SENSOR_LEN);
    memcpy(hil_sensor, _MAV_PAYLOAD(msg), len);
#endif
}
