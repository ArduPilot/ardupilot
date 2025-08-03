#pragma once
// MESSAGE HIL_OPTICAL_FLOW PACKING

#define MAVLINK_MSG_ID_HIL_OPTICAL_FLOW 114


typedef struct __mavlink_hil_optical_flow_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 uint32_t integration_time_us; /*< [us] Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.*/
 float integrated_x; /*< [rad] Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)*/
 float integrated_y; /*< [rad] Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)*/
 float integrated_xgyro; /*< [rad] RH rotation around X axis*/
 float integrated_ygyro; /*< [rad] RH rotation around Y axis*/
 float integrated_zgyro; /*< [rad] RH rotation around Z axis*/
 uint32_t time_delta_distance_us; /*< [us] Time since the distance was sampled.*/
 float distance; /*< [m] Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.*/
 int16_t temperature; /*< [cdegC] Temperature*/
 uint8_t sensor_id; /*<  Sensor ID*/
 uint8_t quality; /*<  Optical flow quality / confidence. 0: no valid flow, 255: maximum quality*/
} mavlink_hil_optical_flow_t;

#define MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN 44
#define MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN 44
#define MAVLINK_MSG_ID_114_LEN 44
#define MAVLINK_MSG_ID_114_MIN_LEN 44

#define MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_CRC 237
#define MAVLINK_MSG_ID_114_CRC 237



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIL_OPTICAL_FLOW { \
    114, \
    "HIL_OPTICAL_FLOW", \
    12, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_optical_flow_t, time_usec) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_hil_optical_flow_t, sensor_id) }, \
         { "integration_time_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_hil_optical_flow_t, integration_time_us) }, \
         { "integrated_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_optical_flow_t, integrated_x) }, \
         { "integrated_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_optical_flow_t, integrated_y) }, \
         { "integrated_xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_optical_flow_t, integrated_xgyro) }, \
         { "integrated_ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_optical_flow_t, integrated_ygyro) }, \
         { "integrated_zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_optical_flow_t, integrated_zgyro) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_hil_optical_flow_t, temperature) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_hil_optical_flow_t, quality) }, \
         { "time_delta_distance_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_hil_optical_flow_t, time_delta_distance_us) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hil_optical_flow_t, distance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIL_OPTICAL_FLOW { \
    "HIL_OPTICAL_FLOW", \
    12, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_optical_flow_t, time_usec) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_hil_optical_flow_t, sensor_id) }, \
         { "integration_time_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_hil_optical_flow_t, integration_time_us) }, \
         { "integrated_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_optical_flow_t, integrated_x) }, \
         { "integrated_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_optical_flow_t, integrated_y) }, \
         { "integrated_xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_optical_flow_t, integrated_xgyro) }, \
         { "integrated_ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_optical_flow_t, integrated_ygyro) }, \
         { "integrated_zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_optical_flow_t, integrated_zgyro) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_hil_optical_flow_t, temperature) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_hil_optical_flow_t, quality) }, \
         { "time_delta_distance_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_hil_optical_flow_t, time_delta_distance_us) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hil_optical_flow_t, distance) }, \
         } \
}
#endif

/**
 * @brief Pack a hil_optical_flow message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param sensor_id  Sensor ID
 * @param integration_time_us [us] Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
 * @param integrated_x [rad] Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
 * @param integrated_y [rad] Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
 * @param integrated_xgyro [rad] RH rotation around X axis
 * @param integrated_ygyro [rad] RH rotation around Y axis
 * @param integrated_zgyro [rad] RH rotation around Z axis
 * @param temperature [cdegC] Temperature
 * @param quality  Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
 * @param time_delta_distance_us [us] Time since the distance was sampled.
 * @param distance [m] Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_optical_flow_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t sensor_id, uint32_t integration_time_us, float integrated_x, float integrated_y, float integrated_xgyro, float integrated_ygyro, float integrated_zgyro, int16_t temperature, uint8_t quality, uint32_t time_delta_distance_us, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, integration_time_us);
    _mav_put_float(buf, 12, integrated_x);
    _mav_put_float(buf, 16, integrated_y);
    _mav_put_float(buf, 20, integrated_xgyro);
    _mav_put_float(buf, 24, integrated_ygyro);
    _mav_put_float(buf, 28, integrated_zgyro);
    _mav_put_uint32_t(buf, 32, time_delta_distance_us);
    _mav_put_float(buf, 36, distance);
    _mav_put_int16_t(buf, 40, temperature);
    _mav_put_uint8_t(buf, 42, sensor_id);
    _mav_put_uint8_t(buf, 43, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN);
#else
    mavlink_hil_optical_flow_t packet;
    packet.time_usec = time_usec;
    packet.integration_time_us = integration_time_us;
    packet.integrated_x = integrated_x;
    packet.integrated_y = integrated_y;
    packet.integrated_xgyro = integrated_xgyro;
    packet.integrated_ygyro = integrated_ygyro;
    packet.integrated_zgyro = integrated_zgyro;
    packet.time_delta_distance_us = time_delta_distance_us;
    packet.distance = distance;
    packet.temperature = temperature;
    packet.sensor_id = sensor_id;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_OPTICAL_FLOW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_CRC);
}

/**
 * @brief Pack a hil_optical_flow message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param sensor_id  Sensor ID
 * @param integration_time_us [us] Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
 * @param integrated_x [rad] Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
 * @param integrated_y [rad] Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
 * @param integrated_xgyro [rad] RH rotation around X axis
 * @param integrated_ygyro [rad] RH rotation around Y axis
 * @param integrated_zgyro [rad] RH rotation around Z axis
 * @param temperature [cdegC] Temperature
 * @param quality  Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
 * @param time_delta_distance_us [us] Time since the distance was sampled.
 * @param distance [m] Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_optical_flow_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t sensor_id, uint32_t integration_time_us, float integrated_x, float integrated_y, float integrated_xgyro, float integrated_ygyro, float integrated_zgyro, int16_t temperature, uint8_t quality, uint32_t time_delta_distance_us, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, integration_time_us);
    _mav_put_float(buf, 12, integrated_x);
    _mav_put_float(buf, 16, integrated_y);
    _mav_put_float(buf, 20, integrated_xgyro);
    _mav_put_float(buf, 24, integrated_ygyro);
    _mav_put_float(buf, 28, integrated_zgyro);
    _mav_put_uint32_t(buf, 32, time_delta_distance_us);
    _mav_put_float(buf, 36, distance);
    _mav_put_int16_t(buf, 40, temperature);
    _mav_put_uint8_t(buf, 42, sensor_id);
    _mav_put_uint8_t(buf, 43, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN);
#else
    mavlink_hil_optical_flow_t packet;
    packet.time_usec = time_usec;
    packet.integration_time_us = integration_time_us;
    packet.integrated_x = integrated_x;
    packet.integrated_y = integrated_y;
    packet.integrated_xgyro = integrated_xgyro;
    packet.integrated_ygyro = integrated_ygyro;
    packet.integrated_zgyro = integrated_zgyro;
    packet.time_delta_distance_us = time_delta_distance_us;
    packet.distance = distance;
    packet.temperature = temperature;
    packet.sensor_id = sensor_id;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_OPTICAL_FLOW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN);
#endif
}

/**
 * @brief Pack a hil_optical_flow message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param sensor_id  Sensor ID
 * @param integration_time_us [us] Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
 * @param integrated_x [rad] Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
 * @param integrated_y [rad] Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
 * @param integrated_xgyro [rad] RH rotation around X axis
 * @param integrated_ygyro [rad] RH rotation around Y axis
 * @param integrated_zgyro [rad] RH rotation around Z axis
 * @param temperature [cdegC] Temperature
 * @param quality  Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
 * @param time_delta_distance_us [us] Time since the distance was sampled.
 * @param distance [m] Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_optical_flow_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t sensor_id,uint32_t integration_time_us,float integrated_x,float integrated_y,float integrated_xgyro,float integrated_ygyro,float integrated_zgyro,int16_t temperature,uint8_t quality,uint32_t time_delta_distance_us,float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, integration_time_us);
    _mav_put_float(buf, 12, integrated_x);
    _mav_put_float(buf, 16, integrated_y);
    _mav_put_float(buf, 20, integrated_xgyro);
    _mav_put_float(buf, 24, integrated_ygyro);
    _mav_put_float(buf, 28, integrated_zgyro);
    _mav_put_uint32_t(buf, 32, time_delta_distance_us);
    _mav_put_float(buf, 36, distance);
    _mav_put_int16_t(buf, 40, temperature);
    _mav_put_uint8_t(buf, 42, sensor_id);
    _mav_put_uint8_t(buf, 43, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN);
#else
    mavlink_hil_optical_flow_t packet;
    packet.time_usec = time_usec;
    packet.integration_time_us = integration_time_us;
    packet.integrated_x = integrated_x;
    packet.integrated_y = integrated_y;
    packet.integrated_xgyro = integrated_xgyro;
    packet.integrated_ygyro = integrated_ygyro;
    packet.integrated_zgyro = integrated_zgyro;
    packet.time_delta_distance_us = time_delta_distance_us;
    packet.distance = distance;
    packet.temperature = temperature;
    packet.sensor_id = sensor_id;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_OPTICAL_FLOW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_CRC);
}

/**
 * @brief Encode a hil_optical_flow struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_optical_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_optical_flow_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_optical_flow_t* hil_optical_flow)
{
    return mavlink_msg_hil_optical_flow_pack(system_id, component_id, msg, hil_optical_flow->time_usec, hil_optical_flow->sensor_id, hil_optical_flow->integration_time_us, hil_optical_flow->integrated_x, hil_optical_flow->integrated_y, hil_optical_flow->integrated_xgyro, hil_optical_flow->integrated_ygyro, hil_optical_flow->integrated_zgyro, hil_optical_flow->temperature, hil_optical_flow->quality, hil_optical_flow->time_delta_distance_us, hil_optical_flow->distance);
}

/**
 * @brief Encode a hil_optical_flow struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hil_optical_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_optical_flow_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hil_optical_flow_t* hil_optical_flow)
{
    return mavlink_msg_hil_optical_flow_pack_chan(system_id, component_id, chan, msg, hil_optical_flow->time_usec, hil_optical_flow->sensor_id, hil_optical_flow->integration_time_us, hil_optical_flow->integrated_x, hil_optical_flow->integrated_y, hil_optical_flow->integrated_xgyro, hil_optical_flow->integrated_ygyro, hil_optical_flow->integrated_zgyro, hil_optical_flow->temperature, hil_optical_flow->quality, hil_optical_flow->time_delta_distance_us, hil_optical_flow->distance);
}

/**
 * @brief Encode a hil_optical_flow struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param hil_optical_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_optical_flow_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_hil_optical_flow_t* hil_optical_flow)
{
    return mavlink_msg_hil_optical_flow_pack_status(system_id, component_id, _status, msg,  hil_optical_flow->time_usec, hil_optical_flow->sensor_id, hil_optical_flow->integration_time_us, hil_optical_flow->integrated_x, hil_optical_flow->integrated_y, hil_optical_flow->integrated_xgyro, hil_optical_flow->integrated_ygyro, hil_optical_flow->integrated_zgyro, hil_optical_flow->temperature, hil_optical_flow->quality, hil_optical_flow->time_delta_distance_us, hil_optical_flow->distance);
}

/**
 * @brief Send a hil_optical_flow message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param sensor_id  Sensor ID
 * @param integration_time_us [us] Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
 * @param integrated_x [rad] Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
 * @param integrated_y [rad] Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
 * @param integrated_xgyro [rad] RH rotation around X axis
 * @param integrated_ygyro [rad] RH rotation around Y axis
 * @param integrated_zgyro [rad] RH rotation around Z axis
 * @param temperature [cdegC] Temperature
 * @param quality  Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
 * @param time_delta_distance_us [us] Time since the distance was sampled.
 * @param distance [m] Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_optical_flow_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t sensor_id, uint32_t integration_time_us, float integrated_x, float integrated_y, float integrated_xgyro, float integrated_ygyro, float integrated_zgyro, int16_t temperature, uint8_t quality, uint32_t time_delta_distance_us, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, integration_time_us);
    _mav_put_float(buf, 12, integrated_x);
    _mav_put_float(buf, 16, integrated_y);
    _mav_put_float(buf, 20, integrated_xgyro);
    _mav_put_float(buf, 24, integrated_ygyro);
    _mav_put_float(buf, 28, integrated_zgyro);
    _mav_put_uint32_t(buf, 32, time_delta_distance_us);
    _mav_put_float(buf, 36, distance);
    _mav_put_int16_t(buf, 40, temperature);
    _mav_put_uint8_t(buf, 42, sensor_id);
    _mav_put_uint8_t(buf, 43, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW, buf, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_CRC);
#else
    mavlink_hil_optical_flow_t packet;
    packet.time_usec = time_usec;
    packet.integration_time_us = integration_time_us;
    packet.integrated_x = integrated_x;
    packet.integrated_y = integrated_y;
    packet.integrated_xgyro = integrated_xgyro;
    packet.integrated_ygyro = integrated_ygyro;
    packet.integrated_zgyro = integrated_zgyro;
    packet.time_delta_distance_us = time_delta_distance_us;
    packet.distance = distance;
    packet.temperature = temperature;
    packet.sensor_id = sensor_id;
    packet.quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW, (const char *)&packet, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_CRC);
#endif
}

/**
 * @brief Send a hil_optical_flow message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hil_optical_flow_send_struct(mavlink_channel_t chan, const mavlink_hil_optical_flow_t* hil_optical_flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hil_optical_flow_send(chan, hil_optical_flow->time_usec, hil_optical_flow->sensor_id, hil_optical_flow->integration_time_us, hil_optical_flow->integrated_x, hil_optical_flow->integrated_y, hil_optical_flow->integrated_xgyro, hil_optical_flow->integrated_ygyro, hil_optical_flow->integrated_zgyro, hil_optical_flow->temperature, hil_optical_flow->quality, hil_optical_flow->time_delta_distance_us, hil_optical_flow->distance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW, (const char *)hil_optical_flow, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hil_optical_flow_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t sensor_id, uint32_t integration_time_us, float integrated_x, float integrated_y, float integrated_xgyro, float integrated_ygyro, float integrated_zgyro, int16_t temperature, uint8_t quality, uint32_t time_delta_distance_us, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, integration_time_us);
    _mav_put_float(buf, 12, integrated_x);
    _mav_put_float(buf, 16, integrated_y);
    _mav_put_float(buf, 20, integrated_xgyro);
    _mav_put_float(buf, 24, integrated_ygyro);
    _mav_put_float(buf, 28, integrated_zgyro);
    _mav_put_uint32_t(buf, 32, time_delta_distance_us);
    _mav_put_float(buf, 36, distance);
    _mav_put_int16_t(buf, 40, temperature);
    _mav_put_uint8_t(buf, 42, sensor_id);
    _mav_put_uint8_t(buf, 43, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW, buf, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_CRC);
#else
    mavlink_hil_optical_flow_t *packet = (mavlink_hil_optical_flow_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->integration_time_us = integration_time_us;
    packet->integrated_x = integrated_x;
    packet->integrated_y = integrated_y;
    packet->integrated_xgyro = integrated_xgyro;
    packet->integrated_ygyro = integrated_ygyro;
    packet->integrated_zgyro = integrated_zgyro;
    packet->time_delta_distance_us = time_delta_distance_us;
    packet->distance = distance;
    packet->temperature = temperature;
    packet->sensor_id = sensor_id;
    packet->quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW, (const char *)packet, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_CRC);
#endif
}
#endif

#endif

// MESSAGE HIL_OPTICAL_FLOW UNPACKING


/**
 * @brief Get field time_usec from hil_optical_flow message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_hil_optical_flow_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_id from hil_optical_flow message
 *
 * @return  Sensor ID
 */
static inline uint8_t mavlink_msg_hil_optical_flow_get_sensor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field integration_time_us from hil_optical_flow message
 *
 * @return [us] Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
 */
static inline uint32_t mavlink_msg_hil_optical_flow_get_integration_time_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field integrated_x from hil_optical_flow message
 *
 * @return [rad] Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
 */
static inline float mavlink_msg_hil_optical_flow_get_integrated_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field integrated_y from hil_optical_flow message
 *
 * @return [rad] Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
 */
static inline float mavlink_msg_hil_optical_flow_get_integrated_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field integrated_xgyro from hil_optical_flow message
 *
 * @return [rad] RH rotation around X axis
 */
static inline float mavlink_msg_hil_optical_flow_get_integrated_xgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field integrated_ygyro from hil_optical_flow message
 *
 * @return [rad] RH rotation around Y axis
 */
static inline float mavlink_msg_hil_optical_flow_get_integrated_ygyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field integrated_zgyro from hil_optical_flow message
 *
 * @return [rad] RH rotation around Z axis
 */
static inline float mavlink_msg_hil_optical_flow_get_integrated_zgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field temperature from hil_optical_flow message
 *
 * @return [cdegC] Temperature
 */
static inline int16_t mavlink_msg_hil_optical_flow_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Get field quality from hil_optical_flow message
 *
 * @return  Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
 */
static inline uint8_t mavlink_msg_hil_optical_flow_get_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field time_delta_distance_us from hil_optical_flow message
 *
 * @return [us] Time since the distance was sampled.
 */
static inline uint32_t mavlink_msg_hil_optical_flow_get_time_delta_distance_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Get field distance from hil_optical_flow message
 *
 * @return [m] Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
 */
static inline float mavlink_msg_hil_optical_flow_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a hil_optical_flow message into a struct
 *
 * @param msg The message to decode
 * @param hil_optical_flow C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_optical_flow_decode(const mavlink_message_t* msg, mavlink_hil_optical_flow_t* hil_optical_flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hil_optical_flow->time_usec = mavlink_msg_hil_optical_flow_get_time_usec(msg);
    hil_optical_flow->integration_time_us = mavlink_msg_hil_optical_flow_get_integration_time_us(msg);
    hil_optical_flow->integrated_x = mavlink_msg_hil_optical_flow_get_integrated_x(msg);
    hil_optical_flow->integrated_y = mavlink_msg_hil_optical_flow_get_integrated_y(msg);
    hil_optical_flow->integrated_xgyro = mavlink_msg_hil_optical_flow_get_integrated_xgyro(msg);
    hil_optical_flow->integrated_ygyro = mavlink_msg_hil_optical_flow_get_integrated_ygyro(msg);
    hil_optical_flow->integrated_zgyro = mavlink_msg_hil_optical_flow_get_integrated_zgyro(msg);
    hil_optical_flow->time_delta_distance_us = mavlink_msg_hil_optical_flow_get_time_delta_distance_us(msg);
    hil_optical_flow->distance = mavlink_msg_hil_optical_flow_get_distance(msg);
    hil_optical_flow->temperature = mavlink_msg_hil_optical_flow_get_temperature(msg);
    hil_optical_flow->sensor_id = mavlink_msg_hil_optical_flow_get_sensor_id(msg);
    hil_optical_flow->quality = mavlink_msg_hil_optical_flow_get_quality(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN? msg->len : MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN;
        memset(hil_optical_flow, 0, MAVLINK_MSG_ID_HIL_OPTICAL_FLOW_LEN);
    memcpy(hil_optical_flow, _MAV_PAYLOAD(msg), len);
#endif
}
