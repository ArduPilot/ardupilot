#pragma once
// MESSAGE GLOBAL_POSITION_INT_COV PACKING

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV 63


typedef struct __mavlink_global_position_int_cov_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 int32_t lat; /*< [degE7] Latitude*/
 int32_t lon; /*< [degE7] Longitude*/
 int32_t alt; /*< [mm] Altitude in meters above MSL*/
 int32_t relative_alt; /*< [mm] Altitude above ground*/
 float vx; /*< [m/s] Ground X Speed (Latitude)*/
 float vy; /*< [m/s] Ground Y Speed (Longitude)*/
 float vz; /*< [m/s] Ground Z Speed (Altitude)*/
 float covariance[36]; /*<  Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If unknown, assign NaN value to first element in the array.*/
 uint8_t estimator_type; /*<  Class id of the estimator this estimate originated from.*/
} mavlink_global_position_int_cov_t;

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN 181
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN 181
#define MAVLINK_MSG_ID_63_LEN 181
#define MAVLINK_MSG_ID_63_MIN_LEN 181

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC 119
#define MAVLINK_MSG_ID_63_CRC 119

#define MAVLINK_MSG_GLOBAL_POSITION_INT_COV_FIELD_COVARIANCE_LEN 36

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT_COV { \
    63, \
    "GLOBAL_POSITION_INT_COV", \
    10, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_global_position_int_cov_t, time_usec) }, \
         { "estimator_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 180, offsetof(mavlink_global_position_int_cov_t, estimator_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_position_int_cov_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_position_int_cov_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_global_position_int_cov_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_global_position_int_cov_t, relative_alt) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_global_position_int_cov_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_global_position_int_cov_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_global_position_int_cov_t, vz) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 36, 36, offsetof(mavlink_global_position_int_cov_t, covariance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT_COV { \
    "GLOBAL_POSITION_INT_COV", \
    10, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_global_position_int_cov_t, time_usec) }, \
         { "estimator_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 180, offsetof(mavlink_global_position_int_cov_t, estimator_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_position_int_cov_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_position_int_cov_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_global_position_int_cov_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_global_position_int_cov_t, relative_alt) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_global_position_int_cov_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_global_position_int_cov_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_global_position_int_cov_t, vz) }, \
         { "covariance", NULL, MAVLINK_TYPE_FLOAT, 36, 36, offsetof(mavlink_global_position_int_cov_t, covariance) }, \
         } \
}
#endif

/**
 * @brief Pack a global_position_int_cov message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param estimator_type  Class id of the estimator this estimate originated from.
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param alt [mm] Altitude in meters above MSL
 * @param relative_alt [mm] Altitude above ground
 * @param vx [m/s] Ground X Speed (Latitude)
 * @param vy [m/s] Ground Y Speed (Longitude)
 * @param vz [m/s] Ground Z Speed (Altitude)
 * @param covariance  Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int_cov_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_int32_t(buf, 20, relative_alt);
    _mav_put_float(buf, 24, vx);
    _mav_put_float(buf, 28, vy);
    _mav_put_float(buf, 32, vz);
    _mav_put_uint8_t(buf, 180, estimator_type);
    _mav_put_float_array(buf, 36, covariance, 36);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN);
#else
    mavlink_global_position_int_cov_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.estimator_type = estimator_type;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*36);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC);
}

/**
 * @brief Pack a global_position_int_cov message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param estimator_type  Class id of the estimator this estimate originated from.
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param alt [mm] Altitude in meters above MSL
 * @param relative_alt [mm] Altitude above ground
 * @param vx [m/s] Ground X Speed (Latitude)
 * @param vy [m/s] Ground Y Speed (Longitude)
 * @param vz [m/s] Ground Z Speed (Altitude)
 * @param covariance  Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int_cov_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_int32_t(buf, 20, relative_alt);
    _mav_put_float(buf, 24, vx);
    _mav_put_float(buf, 28, vy);
    _mav_put_float(buf, 32, vz);
    _mav_put_uint8_t(buf, 180, estimator_type);
    _mav_put_float_array(buf, 36, covariance, 36);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN);
#else
    mavlink_global_position_int_cov_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.estimator_type = estimator_type;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*36);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN);
#endif
}

/**
 * @brief Pack a global_position_int_cov message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param estimator_type  Class id of the estimator this estimate originated from.
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param alt [mm] Altitude in meters above MSL
 * @param relative_alt [mm] Altitude above ground
 * @param vx [m/s] Ground X Speed (Latitude)
 * @param vy [m/s] Ground Y Speed (Longitude)
 * @param vz [m/s] Ground Z Speed (Altitude)
 * @param covariance  Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int_cov_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t estimator_type,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,float vx,float vy,float vz,const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_int32_t(buf, 20, relative_alt);
    _mav_put_float(buf, 24, vx);
    _mav_put_float(buf, 28, vy);
    _mav_put_float(buf, 32, vz);
    _mav_put_uint8_t(buf, 180, estimator_type);
    _mav_put_float_array(buf, 36, covariance, 36);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN);
#else
    mavlink_global_position_int_cov_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.estimator_type = estimator_type;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*36);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC);
}

/**
 * @brief Encode a global_position_int_cov struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int_cov C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_int_cov_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_int_cov_t* global_position_int_cov)
{
    return mavlink_msg_global_position_int_cov_pack(system_id, component_id, msg, global_position_int_cov->time_usec, global_position_int_cov->estimator_type, global_position_int_cov->lat, global_position_int_cov->lon, global_position_int_cov->alt, global_position_int_cov->relative_alt, global_position_int_cov->vx, global_position_int_cov->vy, global_position_int_cov->vz, global_position_int_cov->covariance);
}

/**
 * @brief Encode a global_position_int_cov struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int_cov C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_int_cov_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_position_int_cov_t* global_position_int_cov)
{
    return mavlink_msg_global_position_int_cov_pack_chan(system_id, component_id, chan, msg, global_position_int_cov->time_usec, global_position_int_cov->estimator_type, global_position_int_cov->lat, global_position_int_cov->lon, global_position_int_cov->alt, global_position_int_cov->relative_alt, global_position_int_cov->vx, global_position_int_cov->vy, global_position_int_cov->vz, global_position_int_cov->covariance);
}

/**
 * @brief Encode a global_position_int_cov struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int_cov C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_int_cov_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_global_position_int_cov_t* global_position_int_cov)
{
    return mavlink_msg_global_position_int_cov_pack_status(system_id, component_id, _status, msg,  global_position_int_cov->time_usec, global_position_int_cov->estimator_type, global_position_int_cov->lat, global_position_int_cov->lon, global_position_int_cov->alt, global_position_int_cov->relative_alt, global_position_int_cov->vx, global_position_int_cov->vy, global_position_int_cov->vz, global_position_int_cov->covariance);
}

/**
 * @brief Send a global_position_int_cov message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param estimator_type  Class id of the estimator this estimate originated from.
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param alt [mm] Altitude in meters above MSL
 * @param relative_alt [mm] Altitude above ground
 * @param vx [m/s] Ground X Speed (Latitude)
 * @param vy [m/s] Ground Y Speed (Longitude)
 * @param vz [m/s] Ground Z Speed (Altitude)
 * @param covariance  Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_int_cov_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_int32_t(buf, 20, relative_alt);
    _mav_put_float(buf, 24, vx);
    _mav_put_float(buf, 28, vy);
    _mav_put_float(buf, 32, vz);
    _mav_put_uint8_t(buf, 180, estimator_type);
    _mav_put_float_array(buf, 36, covariance, 36);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC);
#else
    mavlink_global_position_int_cov_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.estimator_type = estimator_type;
    mav_array_memcpy(packet.covariance, covariance, sizeof(float)*36);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC);
#endif
}

/**
 * @brief Send a global_position_int_cov message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_global_position_int_cov_send_struct(mavlink_channel_t chan, const mavlink_global_position_int_cov_t* global_position_int_cov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_global_position_int_cov_send(chan, global_position_int_cov->time_usec, global_position_int_cov->estimator_type, global_position_int_cov->lat, global_position_int_cov->lon, global_position_int_cov->alt, global_position_int_cov->relative_alt, global_position_int_cov->vx, global_position_int_cov->vy, global_position_int_cov->vz, global_position_int_cov->covariance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV, (const char *)global_position_int_cov, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC);
#endif
}

#if MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_global_position_int_cov_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t estimator_type, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, float vx, float vy, float vz, const float *covariance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_int32_t(buf, 20, relative_alt);
    _mav_put_float(buf, 24, vx);
    _mav_put_float(buf, 28, vy);
    _mav_put_float(buf, 32, vz);
    _mav_put_uint8_t(buf, 180, estimator_type);
    _mav_put_float_array(buf, 36, covariance, 36);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC);
#else
    mavlink_global_position_int_cov_t *packet = (mavlink_global_position_int_cov_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->relative_alt = relative_alt;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->estimator_type = estimator_type;
    mav_array_memcpy(packet->covariance, covariance, sizeof(float)*36);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_CRC);
#endif
}
#endif

#endif

// MESSAGE GLOBAL_POSITION_INT_COV UNPACKING


/**
 * @brief Get field time_usec from global_position_int_cov message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_global_position_int_cov_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field estimator_type from global_position_int_cov message
 *
 * @return  Class id of the estimator this estimate originated from.
 */
static inline uint8_t mavlink_msg_global_position_int_cov_get_estimator_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  180);
}

/**
 * @brief Get field lat from global_position_int_cov message
 *
 * @return [degE7] Latitude
 */
static inline int32_t mavlink_msg_global_position_int_cov_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from global_position_int_cov message
 *
 * @return [degE7] Longitude
 */
static inline int32_t mavlink_msg_global_position_int_cov_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from global_position_int_cov message
 *
 * @return [mm] Altitude in meters above MSL
 */
static inline int32_t mavlink_msg_global_position_int_cov_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field relative_alt from global_position_int_cov message
 *
 * @return [mm] Altitude above ground
 */
static inline int32_t mavlink_msg_global_position_int_cov_get_relative_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field vx from global_position_int_cov message
 *
 * @return [m/s] Ground X Speed (Latitude)
 */
static inline float mavlink_msg_global_position_int_cov_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vy from global_position_int_cov message
 *
 * @return [m/s] Ground Y Speed (Longitude)
 */
static inline float mavlink_msg_global_position_int_cov_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vz from global_position_int_cov message
 *
 * @return [m/s] Ground Z Speed (Altitude)
 */
static inline float mavlink_msg_global_position_int_cov_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field covariance from global_position_int_cov message
 *
 * @return  Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
 */
static inline uint16_t mavlink_msg_global_position_int_cov_get_covariance(const mavlink_message_t* msg, float *covariance)
{
    return _MAV_RETURN_float_array(msg, covariance, 36,  36);
}

/**
 * @brief Decode a global_position_int_cov message into a struct
 *
 * @param msg The message to decode
 * @param global_position_int_cov C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_position_int_cov_decode(const mavlink_message_t* msg, mavlink_global_position_int_cov_t* global_position_int_cov)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    global_position_int_cov->time_usec = mavlink_msg_global_position_int_cov_get_time_usec(msg);
    global_position_int_cov->lat = mavlink_msg_global_position_int_cov_get_lat(msg);
    global_position_int_cov->lon = mavlink_msg_global_position_int_cov_get_lon(msg);
    global_position_int_cov->alt = mavlink_msg_global_position_int_cov_get_alt(msg);
    global_position_int_cov->relative_alt = mavlink_msg_global_position_int_cov_get_relative_alt(msg);
    global_position_int_cov->vx = mavlink_msg_global_position_int_cov_get_vx(msg);
    global_position_int_cov->vy = mavlink_msg_global_position_int_cov_get_vy(msg);
    global_position_int_cov->vz = mavlink_msg_global_position_int_cov_get_vz(msg);
    mavlink_msg_global_position_int_cov_get_covariance(msg, global_position_int_cov->covariance);
    global_position_int_cov->estimator_type = mavlink_msg_global_position_int_cov_get_estimator_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN? msg->len : MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN;
        memset(global_position_int_cov, 0, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV_LEN);
    memcpy(global_position_int_cov, _MAV_PAYLOAD(msg), len);
#endif
}
