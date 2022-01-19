#pragma once
// MESSAGE ADAP_TUNING PACKING

#define MAVLINK_MSG_ID_ADAP_TUNING 11010


typedef struct __mavlink_adap_tuning_t {
 float desired; /*< [deg/s] Desired rate.*/
 float achieved; /*< [deg/s] Achieved rate.*/
 float error; /*<  Error between model and vehicle.*/
 float theta; /*<  Theta estimated state predictor.*/
 float omega; /*<  Omega estimated state predictor.*/
 float sigma; /*<  Sigma estimated state predictor.*/
 float theta_dot; /*<  Theta derivative.*/
 float omega_dot; /*<  Omega derivative.*/
 float sigma_dot; /*<  Sigma derivative.*/
 float f; /*<  Projection operator value.*/
 float f_dot; /*<  Projection operator derivative.*/
 float u; /*<  u adaptive controlled output command.*/
 uint8_t axis; /*<  Axis.*/
} mavlink_adap_tuning_t;

#define MAVLINK_MSG_ID_ADAP_TUNING_LEN 49
#define MAVLINK_MSG_ID_ADAP_TUNING_MIN_LEN 49
#define MAVLINK_MSG_ID_11010_LEN 49
#define MAVLINK_MSG_ID_11010_MIN_LEN 49

#define MAVLINK_MSG_ID_ADAP_TUNING_CRC 46
#define MAVLINK_MSG_ID_11010_CRC 46



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADAP_TUNING { \
    11010, \
    "ADAP_TUNING", \
    13, \
    {  { "axis", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_adap_tuning_t, axis) }, \
         { "desired", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_adap_tuning_t, desired) }, \
         { "achieved", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_adap_tuning_t, achieved) }, \
         { "error", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adap_tuning_t, error) }, \
         { "theta", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adap_tuning_t, theta) }, \
         { "omega", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adap_tuning_t, omega) }, \
         { "sigma", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adap_tuning_t, sigma) }, \
         { "theta_dot", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adap_tuning_t, theta_dot) }, \
         { "omega_dot", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_adap_tuning_t, omega_dot) }, \
         { "sigma_dot", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_adap_tuning_t, sigma_dot) }, \
         { "f", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_adap_tuning_t, f) }, \
         { "f_dot", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_adap_tuning_t, f_dot) }, \
         { "u", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_adap_tuning_t, u) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADAP_TUNING { \
    "ADAP_TUNING", \
    13, \
    {  { "axis", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_adap_tuning_t, axis) }, \
         { "desired", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_adap_tuning_t, desired) }, \
         { "achieved", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_adap_tuning_t, achieved) }, \
         { "error", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adap_tuning_t, error) }, \
         { "theta", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adap_tuning_t, theta) }, \
         { "omega", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adap_tuning_t, omega) }, \
         { "sigma", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adap_tuning_t, sigma) }, \
         { "theta_dot", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adap_tuning_t, theta_dot) }, \
         { "omega_dot", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_adap_tuning_t, omega_dot) }, \
         { "sigma_dot", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_adap_tuning_t, sigma_dot) }, \
         { "f", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_adap_tuning_t, f) }, \
         { "f_dot", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_adap_tuning_t, f_dot) }, \
         { "u", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_adap_tuning_t, u) }, \
         } \
}
#endif

/**
 * @brief Pack a adap_tuning message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param axis  Axis.
 * @param desired [deg/s] Desired rate.
 * @param achieved [deg/s] Achieved rate.
 * @param error  Error between model and vehicle.
 * @param theta  Theta estimated state predictor.
 * @param omega  Omega estimated state predictor.
 * @param sigma  Sigma estimated state predictor.
 * @param theta_dot  Theta derivative.
 * @param omega_dot  Omega derivative.
 * @param sigma_dot  Sigma derivative.
 * @param f  Projection operator value.
 * @param f_dot  Projection operator derivative.
 * @param u  u adaptive controlled output command.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adap_tuning_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t axis, float desired, float achieved, float error, float theta, float omega, float sigma, float theta_dot, float omega_dot, float sigma_dot, float f, float f_dot, float u)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADAP_TUNING_LEN];
    _mav_put_float(buf, 0, desired);
    _mav_put_float(buf, 4, achieved);
    _mav_put_float(buf, 8, error);
    _mav_put_float(buf, 12, theta);
    _mav_put_float(buf, 16, omega);
    _mav_put_float(buf, 20, sigma);
    _mav_put_float(buf, 24, theta_dot);
    _mav_put_float(buf, 28, omega_dot);
    _mav_put_float(buf, 32, sigma_dot);
    _mav_put_float(buf, 36, f);
    _mav_put_float(buf, 40, f_dot);
    _mav_put_float(buf, 44, u);
    _mav_put_uint8_t(buf, 48, axis);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADAP_TUNING_LEN);
#else
    mavlink_adap_tuning_t packet;
    packet.desired = desired;
    packet.achieved = achieved;
    packet.error = error;
    packet.theta = theta;
    packet.omega = omega;
    packet.sigma = sigma;
    packet.theta_dot = theta_dot;
    packet.omega_dot = omega_dot;
    packet.sigma_dot = sigma_dot;
    packet.f = f;
    packet.f_dot = f_dot;
    packet.u = u;
    packet.axis = axis;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADAP_TUNING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADAP_TUNING;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADAP_TUNING_MIN_LEN, MAVLINK_MSG_ID_ADAP_TUNING_LEN, MAVLINK_MSG_ID_ADAP_TUNING_CRC);
}

/**
 * @brief Pack a adap_tuning message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param axis  Axis.
 * @param desired [deg/s] Desired rate.
 * @param achieved [deg/s] Achieved rate.
 * @param error  Error between model and vehicle.
 * @param theta  Theta estimated state predictor.
 * @param omega  Omega estimated state predictor.
 * @param sigma  Sigma estimated state predictor.
 * @param theta_dot  Theta derivative.
 * @param omega_dot  Omega derivative.
 * @param sigma_dot  Sigma derivative.
 * @param f  Projection operator value.
 * @param f_dot  Projection operator derivative.
 * @param u  u adaptive controlled output command.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adap_tuning_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t axis,float desired,float achieved,float error,float theta,float omega,float sigma,float theta_dot,float omega_dot,float sigma_dot,float f,float f_dot,float u)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADAP_TUNING_LEN];
    _mav_put_float(buf, 0, desired);
    _mav_put_float(buf, 4, achieved);
    _mav_put_float(buf, 8, error);
    _mav_put_float(buf, 12, theta);
    _mav_put_float(buf, 16, omega);
    _mav_put_float(buf, 20, sigma);
    _mav_put_float(buf, 24, theta_dot);
    _mav_put_float(buf, 28, omega_dot);
    _mav_put_float(buf, 32, sigma_dot);
    _mav_put_float(buf, 36, f);
    _mav_put_float(buf, 40, f_dot);
    _mav_put_float(buf, 44, u);
    _mav_put_uint8_t(buf, 48, axis);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADAP_TUNING_LEN);
#else
    mavlink_adap_tuning_t packet;
    packet.desired = desired;
    packet.achieved = achieved;
    packet.error = error;
    packet.theta = theta;
    packet.omega = omega;
    packet.sigma = sigma;
    packet.theta_dot = theta_dot;
    packet.omega_dot = omega_dot;
    packet.sigma_dot = sigma_dot;
    packet.f = f;
    packet.f_dot = f_dot;
    packet.u = u;
    packet.axis = axis;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADAP_TUNING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADAP_TUNING;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADAP_TUNING_MIN_LEN, MAVLINK_MSG_ID_ADAP_TUNING_LEN, MAVLINK_MSG_ID_ADAP_TUNING_CRC);
}

/**
 * @brief Encode a adap_tuning struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adap_tuning C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adap_tuning_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adap_tuning_t* adap_tuning)
{
    return mavlink_msg_adap_tuning_pack(system_id, component_id, msg, adap_tuning->axis, adap_tuning->desired, adap_tuning->achieved, adap_tuning->error, adap_tuning->theta, adap_tuning->omega, adap_tuning->sigma, adap_tuning->theta_dot, adap_tuning->omega_dot, adap_tuning->sigma_dot, adap_tuning->f, adap_tuning->f_dot, adap_tuning->u);
}

/**
 * @brief Encode a adap_tuning struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adap_tuning C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adap_tuning_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adap_tuning_t* adap_tuning)
{
    return mavlink_msg_adap_tuning_pack_chan(system_id, component_id, chan, msg, adap_tuning->axis, adap_tuning->desired, adap_tuning->achieved, adap_tuning->error, adap_tuning->theta, adap_tuning->omega, adap_tuning->sigma, adap_tuning->theta_dot, adap_tuning->omega_dot, adap_tuning->sigma_dot, adap_tuning->f, adap_tuning->f_dot, adap_tuning->u);
}

/**
 * @brief Send a adap_tuning message
 * @param chan MAVLink channel to send the message
 *
 * @param axis  Axis.
 * @param desired [deg/s] Desired rate.
 * @param achieved [deg/s] Achieved rate.
 * @param error  Error between model and vehicle.
 * @param theta  Theta estimated state predictor.
 * @param omega  Omega estimated state predictor.
 * @param sigma  Sigma estimated state predictor.
 * @param theta_dot  Theta derivative.
 * @param omega_dot  Omega derivative.
 * @param sigma_dot  Sigma derivative.
 * @param f  Projection operator value.
 * @param f_dot  Projection operator derivative.
 * @param u  u adaptive controlled output command.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adap_tuning_send(mavlink_channel_t chan, uint8_t axis, float desired, float achieved, float error, float theta, float omega, float sigma, float theta_dot, float omega_dot, float sigma_dot, float f, float f_dot, float u)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADAP_TUNING_LEN];
    _mav_put_float(buf, 0, desired);
    _mav_put_float(buf, 4, achieved);
    _mav_put_float(buf, 8, error);
    _mav_put_float(buf, 12, theta);
    _mav_put_float(buf, 16, omega);
    _mav_put_float(buf, 20, sigma);
    _mav_put_float(buf, 24, theta_dot);
    _mav_put_float(buf, 28, omega_dot);
    _mav_put_float(buf, 32, sigma_dot);
    _mav_put_float(buf, 36, f);
    _mav_put_float(buf, 40, f_dot);
    _mav_put_float(buf, 44, u);
    _mav_put_uint8_t(buf, 48, axis);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADAP_TUNING, buf, MAVLINK_MSG_ID_ADAP_TUNING_MIN_LEN, MAVLINK_MSG_ID_ADAP_TUNING_LEN, MAVLINK_MSG_ID_ADAP_TUNING_CRC);
#else
    mavlink_adap_tuning_t packet;
    packet.desired = desired;
    packet.achieved = achieved;
    packet.error = error;
    packet.theta = theta;
    packet.omega = omega;
    packet.sigma = sigma;
    packet.theta_dot = theta_dot;
    packet.omega_dot = omega_dot;
    packet.sigma_dot = sigma_dot;
    packet.f = f;
    packet.f_dot = f_dot;
    packet.u = u;
    packet.axis = axis;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADAP_TUNING, (const char *)&packet, MAVLINK_MSG_ID_ADAP_TUNING_MIN_LEN, MAVLINK_MSG_ID_ADAP_TUNING_LEN, MAVLINK_MSG_ID_ADAP_TUNING_CRC);
#endif
}

/**
 * @brief Send a adap_tuning message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adap_tuning_send_struct(mavlink_channel_t chan, const mavlink_adap_tuning_t* adap_tuning)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adap_tuning_send(chan, adap_tuning->axis, adap_tuning->desired, adap_tuning->achieved, adap_tuning->error, adap_tuning->theta, adap_tuning->omega, adap_tuning->sigma, adap_tuning->theta_dot, adap_tuning->omega_dot, adap_tuning->sigma_dot, adap_tuning->f, adap_tuning->f_dot, adap_tuning->u);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADAP_TUNING, (const char *)adap_tuning, MAVLINK_MSG_ID_ADAP_TUNING_MIN_LEN, MAVLINK_MSG_ID_ADAP_TUNING_LEN, MAVLINK_MSG_ID_ADAP_TUNING_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADAP_TUNING_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adap_tuning_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t axis, float desired, float achieved, float error, float theta, float omega, float sigma, float theta_dot, float omega_dot, float sigma_dot, float f, float f_dot, float u)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, desired);
    _mav_put_float(buf, 4, achieved);
    _mav_put_float(buf, 8, error);
    _mav_put_float(buf, 12, theta);
    _mav_put_float(buf, 16, omega);
    _mav_put_float(buf, 20, sigma);
    _mav_put_float(buf, 24, theta_dot);
    _mav_put_float(buf, 28, omega_dot);
    _mav_put_float(buf, 32, sigma_dot);
    _mav_put_float(buf, 36, f);
    _mav_put_float(buf, 40, f_dot);
    _mav_put_float(buf, 44, u);
    _mav_put_uint8_t(buf, 48, axis);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADAP_TUNING, buf, MAVLINK_MSG_ID_ADAP_TUNING_MIN_LEN, MAVLINK_MSG_ID_ADAP_TUNING_LEN, MAVLINK_MSG_ID_ADAP_TUNING_CRC);
#else
    mavlink_adap_tuning_t *packet = (mavlink_adap_tuning_t *)msgbuf;
    packet->desired = desired;
    packet->achieved = achieved;
    packet->error = error;
    packet->theta = theta;
    packet->omega = omega;
    packet->sigma = sigma;
    packet->theta_dot = theta_dot;
    packet->omega_dot = omega_dot;
    packet->sigma_dot = sigma_dot;
    packet->f = f;
    packet->f_dot = f_dot;
    packet->u = u;
    packet->axis = axis;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADAP_TUNING, (const char *)packet, MAVLINK_MSG_ID_ADAP_TUNING_MIN_LEN, MAVLINK_MSG_ID_ADAP_TUNING_LEN, MAVLINK_MSG_ID_ADAP_TUNING_CRC);
#endif
}
#endif

#endif

// MESSAGE ADAP_TUNING UNPACKING


/**
 * @brief Get field axis from adap_tuning message
 *
 * @return  Axis.
 */
static inline uint8_t mavlink_msg_adap_tuning_get_axis(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field desired from adap_tuning message
 *
 * @return [deg/s] Desired rate.
 */
static inline float mavlink_msg_adap_tuning_get_desired(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field achieved from adap_tuning message
 *
 * @return [deg/s] Achieved rate.
 */
static inline float mavlink_msg_adap_tuning_get_achieved(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field error from adap_tuning message
 *
 * @return  Error between model and vehicle.
 */
static inline float mavlink_msg_adap_tuning_get_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field theta from adap_tuning message
 *
 * @return  Theta estimated state predictor.
 */
static inline float mavlink_msg_adap_tuning_get_theta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field omega from adap_tuning message
 *
 * @return  Omega estimated state predictor.
 */
static inline float mavlink_msg_adap_tuning_get_omega(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field sigma from adap_tuning message
 *
 * @return  Sigma estimated state predictor.
 */
static inline float mavlink_msg_adap_tuning_get_sigma(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field theta_dot from adap_tuning message
 *
 * @return  Theta derivative.
 */
static inline float mavlink_msg_adap_tuning_get_theta_dot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field omega_dot from adap_tuning message
 *
 * @return  Omega derivative.
 */
static inline float mavlink_msg_adap_tuning_get_omega_dot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field sigma_dot from adap_tuning message
 *
 * @return  Sigma derivative.
 */
static inline float mavlink_msg_adap_tuning_get_sigma_dot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field f from adap_tuning message
 *
 * @return  Projection operator value.
 */
static inline float mavlink_msg_adap_tuning_get_f(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field f_dot from adap_tuning message
 *
 * @return  Projection operator derivative.
 */
static inline float mavlink_msg_adap_tuning_get_f_dot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field u from adap_tuning message
 *
 * @return  u adaptive controlled output command.
 */
static inline float mavlink_msg_adap_tuning_get_u(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a adap_tuning message into a struct
 *
 * @param msg The message to decode
 * @param adap_tuning C-struct to decode the message contents into
 */
static inline void mavlink_msg_adap_tuning_decode(const mavlink_message_t* msg, mavlink_adap_tuning_t* adap_tuning)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    adap_tuning->desired = mavlink_msg_adap_tuning_get_desired(msg);
    adap_tuning->achieved = mavlink_msg_adap_tuning_get_achieved(msg);
    adap_tuning->error = mavlink_msg_adap_tuning_get_error(msg);
    adap_tuning->theta = mavlink_msg_adap_tuning_get_theta(msg);
    adap_tuning->omega = mavlink_msg_adap_tuning_get_omega(msg);
    adap_tuning->sigma = mavlink_msg_adap_tuning_get_sigma(msg);
    adap_tuning->theta_dot = mavlink_msg_adap_tuning_get_theta_dot(msg);
    adap_tuning->omega_dot = mavlink_msg_adap_tuning_get_omega_dot(msg);
    adap_tuning->sigma_dot = mavlink_msg_adap_tuning_get_sigma_dot(msg);
    adap_tuning->f = mavlink_msg_adap_tuning_get_f(msg);
    adap_tuning->f_dot = mavlink_msg_adap_tuning_get_f_dot(msg);
    adap_tuning->u = mavlink_msg_adap_tuning_get_u(msg);
    adap_tuning->axis = mavlink_msg_adap_tuning_get_axis(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADAP_TUNING_LEN? msg->len : MAVLINK_MSG_ID_ADAP_TUNING_LEN;
        memset(adap_tuning, 0, MAVLINK_MSG_ID_ADAP_TUNING_LEN);
    memcpy(adap_tuning, _MAV_PAYLOAD(msg), len);
#endif
}
