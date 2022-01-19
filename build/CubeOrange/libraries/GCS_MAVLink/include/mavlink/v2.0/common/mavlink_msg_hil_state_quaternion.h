#pragma once
// MESSAGE HIL_STATE_QUATERNION PACKING

#define MAVLINK_MSG_ID_HIL_STATE_QUATERNION 115


typedef struct __mavlink_hil_state_quaternion_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float attitude_quaternion[4]; /*<  Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)*/
 float rollspeed; /*< [rad/s] Body frame roll / phi angular speed*/
 float pitchspeed; /*< [rad/s] Body frame pitch / theta angular speed*/
 float yawspeed; /*< [rad/s] Body frame yaw / psi angular speed*/
 int32_t lat; /*< [degE7] Latitude*/
 int32_t lon; /*< [degE7] Longitude*/
 int32_t alt; /*< [mm] Altitude*/
 int16_t vx; /*< [cm/s] Ground X Speed (Latitude)*/
 int16_t vy; /*< [cm/s] Ground Y Speed (Longitude)*/
 int16_t vz; /*< [cm/s] Ground Z Speed (Altitude)*/
 uint16_t ind_airspeed; /*< [cm/s] Indicated airspeed*/
 uint16_t true_airspeed; /*< [cm/s] True airspeed*/
 int16_t xacc; /*< [mG] X acceleration*/
 int16_t yacc; /*< [mG] Y acceleration*/
 int16_t zacc; /*< [mG] Z acceleration*/
} mavlink_hil_state_quaternion_t;

#define MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN 64
#define MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN 64
#define MAVLINK_MSG_ID_115_LEN 64
#define MAVLINK_MSG_ID_115_MIN_LEN 64

#define MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC 4
#define MAVLINK_MSG_ID_115_CRC 4

#define MAVLINK_MSG_HIL_STATE_QUATERNION_FIELD_ATTITUDE_QUATERNION_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIL_STATE_QUATERNION { \
    115, \
    "HIL_STATE_QUATERNION", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_state_quaternion_t, time_usec) }, \
         { "attitude_quaternion", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_hil_state_quaternion_t, attitude_quaternion) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_state_quaternion_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_state_quaternion_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hil_state_quaternion_t, yawspeed) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_hil_state_quaternion_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_hil_state_quaternion_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_hil_state_quaternion_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_hil_state_quaternion_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 50, offsetof(mavlink_hil_state_quaternion_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 52, offsetof(mavlink_hil_state_quaternion_t, vz) }, \
         { "ind_airspeed", NULL, MAVLINK_TYPE_UINT16_T, 0, 54, offsetof(mavlink_hil_state_quaternion_t, ind_airspeed) }, \
         { "true_airspeed", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_hil_state_quaternion_t, true_airspeed) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 58, offsetof(mavlink_hil_state_quaternion_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 60, offsetof(mavlink_hil_state_quaternion_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 62, offsetof(mavlink_hil_state_quaternion_t, zacc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIL_STATE_QUATERNION { \
    "HIL_STATE_QUATERNION", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_state_quaternion_t, time_usec) }, \
         { "attitude_quaternion", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_hil_state_quaternion_t, attitude_quaternion) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_state_quaternion_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_state_quaternion_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hil_state_quaternion_t, yawspeed) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_hil_state_quaternion_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_hil_state_quaternion_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_hil_state_quaternion_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_hil_state_quaternion_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 50, offsetof(mavlink_hil_state_quaternion_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 52, offsetof(mavlink_hil_state_quaternion_t, vz) }, \
         { "ind_airspeed", NULL, MAVLINK_TYPE_UINT16_T, 0, 54, offsetof(mavlink_hil_state_quaternion_t, ind_airspeed) }, \
         { "true_airspeed", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_hil_state_quaternion_t, true_airspeed) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 58, offsetof(mavlink_hil_state_quaternion_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 60, offsetof(mavlink_hil_state_quaternion_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 62, offsetof(mavlink_hil_state_quaternion_t, zacc) }, \
         } \
}
#endif

/**
 * @brief Pack a hil_state_quaternion message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param attitude_quaternion  Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 * @param rollspeed [rad/s] Body frame roll / phi angular speed
 * @param pitchspeed [rad/s] Body frame pitch / theta angular speed
 * @param yawspeed [rad/s] Body frame yaw / psi angular speed
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param alt [mm] Altitude
 * @param vx [cm/s] Ground X Speed (Latitude)
 * @param vy [cm/s] Ground Y Speed (Longitude)
 * @param vz [cm/s] Ground Z Speed (Altitude)
 * @param ind_airspeed [cm/s] Indicated airspeed
 * @param true_airspeed [cm/s] True airspeed
 * @param xacc [mG] X acceleration
 * @param yacc [mG] Y acceleration
 * @param zacc [mG] Z acceleration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_state_quaternion_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);
    _mav_put_int32_t(buf, 36, lat);
    _mav_put_int32_t(buf, 40, lon);
    _mav_put_int32_t(buf, 44, alt);
    _mav_put_int16_t(buf, 48, vx);
    _mav_put_int16_t(buf, 50, vy);
    _mav_put_int16_t(buf, 52, vz);
    _mav_put_uint16_t(buf, 54, ind_airspeed);
    _mav_put_uint16_t(buf, 56, true_airspeed);
    _mav_put_int16_t(buf, 58, xacc);
    _mav_put_int16_t(buf, 60, yacc);
    _mav_put_int16_t(buf, 62, zacc);
    _mav_put_float_array(buf, 8, attitude_quaternion, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN);
#else
    mavlink_hil_state_quaternion_t packet;
    packet.time_usec = time_usec;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.ind_airspeed = ind_airspeed;
    packet.true_airspeed = true_airspeed;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    mav_array_memcpy(packet.attitude_quaternion, attitude_quaternion, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_STATE_QUATERNION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC);
}

/**
 * @brief Pack a hil_state_quaternion message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param attitude_quaternion  Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 * @param rollspeed [rad/s] Body frame roll / phi angular speed
 * @param pitchspeed [rad/s] Body frame pitch / theta angular speed
 * @param yawspeed [rad/s] Body frame yaw / psi angular speed
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param alt [mm] Altitude
 * @param vx [cm/s] Ground X Speed (Latitude)
 * @param vy [cm/s] Ground Y Speed (Longitude)
 * @param vz [cm/s] Ground Z Speed (Altitude)
 * @param ind_airspeed [cm/s] Indicated airspeed
 * @param true_airspeed [cm/s] True airspeed
 * @param xacc [mG] X acceleration
 * @param yacc [mG] Y acceleration
 * @param zacc [mG] Z acceleration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_state_quaternion_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,const float *attitude_quaternion,float rollspeed,float pitchspeed,float yawspeed,int32_t lat,int32_t lon,int32_t alt,int16_t vx,int16_t vy,int16_t vz,uint16_t ind_airspeed,uint16_t true_airspeed,int16_t xacc,int16_t yacc,int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);
    _mav_put_int32_t(buf, 36, lat);
    _mav_put_int32_t(buf, 40, lon);
    _mav_put_int32_t(buf, 44, alt);
    _mav_put_int16_t(buf, 48, vx);
    _mav_put_int16_t(buf, 50, vy);
    _mav_put_int16_t(buf, 52, vz);
    _mav_put_uint16_t(buf, 54, ind_airspeed);
    _mav_put_uint16_t(buf, 56, true_airspeed);
    _mav_put_int16_t(buf, 58, xacc);
    _mav_put_int16_t(buf, 60, yacc);
    _mav_put_int16_t(buf, 62, zacc);
    _mav_put_float_array(buf, 8, attitude_quaternion, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN);
#else
    mavlink_hil_state_quaternion_t packet;
    packet.time_usec = time_usec;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.ind_airspeed = ind_airspeed;
    packet.true_airspeed = true_airspeed;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    mav_array_memcpy(packet.attitude_quaternion, attitude_quaternion, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_STATE_QUATERNION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC);
}

/**
 * @brief Encode a hil_state_quaternion struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_state_quaternion C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_state_quaternion_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_state_quaternion_t* hil_state_quaternion)
{
    return mavlink_msg_hil_state_quaternion_pack(system_id, component_id, msg, hil_state_quaternion->time_usec, hil_state_quaternion->attitude_quaternion, hil_state_quaternion->rollspeed, hil_state_quaternion->pitchspeed, hil_state_quaternion->yawspeed, hil_state_quaternion->lat, hil_state_quaternion->lon, hil_state_quaternion->alt, hil_state_quaternion->vx, hil_state_quaternion->vy, hil_state_quaternion->vz, hil_state_quaternion->ind_airspeed, hil_state_quaternion->true_airspeed, hil_state_quaternion->xacc, hil_state_quaternion->yacc, hil_state_quaternion->zacc);
}

/**
 * @brief Encode a hil_state_quaternion struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hil_state_quaternion C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_state_quaternion_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hil_state_quaternion_t* hil_state_quaternion)
{
    return mavlink_msg_hil_state_quaternion_pack_chan(system_id, component_id, chan, msg, hil_state_quaternion->time_usec, hil_state_quaternion->attitude_quaternion, hil_state_quaternion->rollspeed, hil_state_quaternion->pitchspeed, hil_state_quaternion->yawspeed, hil_state_quaternion->lat, hil_state_quaternion->lon, hil_state_quaternion->alt, hil_state_quaternion->vx, hil_state_quaternion->vy, hil_state_quaternion->vz, hil_state_quaternion->ind_airspeed, hil_state_quaternion->true_airspeed, hil_state_quaternion->xacc, hil_state_quaternion->yacc, hil_state_quaternion->zacc);
}

/**
 * @brief Send a hil_state_quaternion message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param attitude_quaternion  Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 * @param rollspeed [rad/s] Body frame roll / phi angular speed
 * @param pitchspeed [rad/s] Body frame pitch / theta angular speed
 * @param yawspeed [rad/s] Body frame yaw / psi angular speed
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param alt [mm] Altitude
 * @param vx [cm/s] Ground X Speed (Latitude)
 * @param vy [cm/s] Ground Y Speed (Longitude)
 * @param vz [cm/s] Ground Z Speed (Altitude)
 * @param ind_airspeed [cm/s] Indicated airspeed
 * @param true_airspeed [cm/s] True airspeed
 * @param xacc [mG] X acceleration
 * @param yacc [mG] Y acceleration
 * @param zacc [mG] Z acceleration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_state_quaternion_send(mavlink_channel_t chan, uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);
    _mav_put_int32_t(buf, 36, lat);
    _mav_put_int32_t(buf, 40, lon);
    _mav_put_int32_t(buf, 44, alt);
    _mav_put_int16_t(buf, 48, vx);
    _mav_put_int16_t(buf, 50, vy);
    _mav_put_int16_t(buf, 52, vz);
    _mav_put_uint16_t(buf, 54, ind_airspeed);
    _mav_put_uint16_t(buf, 56, true_airspeed);
    _mav_put_int16_t(buf, 58, xacc);
    _mav_put_int16_t(buf, 60, yacc);
    _mav_put_int16_t(buf, 62, zacc);
    _mav_put_float_array(buf, 8, attitude_quaternion, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE_QUATERNION, buf, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC);
#else
    mavlink_hil_state_quaternion_t packet;
    packet.time_usec = time_usec;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.ind_airspeed = ind_airspeed;
    packet.true_airspeed = true_airspeed;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    mav_array_memcpy(packet.attitude_quaternion, attitude_quaternion, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE_QUATERNION, (const char *)&packet, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC);
#endif
}

/**
 * @brief Send a hil_state_quaternion message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hil_state_quaternion_send_struct(mavlink_channel_t chan, const mavlink_hil_state_quaternion_t* hil_state_quaternion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hil_state_quaternion_send(chan, hil_state_quaternion->time_usec, hil_state_quaternion->attitude_quaternion, hil_state_quaternion->rollspeed, hil_state_quaternion->pitchspeed, hil_state_quaternion->yawspeed, hil_state_quaternion->lat, hil_state_quaternion->lon, hil_state_quaternion->alt, hil_state_quaternion->vx, hil_state_quaternion->vy, hil_state_quaternion->vz, hil_state_quaternion->ind_airspeed, hil_state_quaternion->true_airspeed, hil_state_quaternion->xacc, hil_state_quaternion->yacc, hil_state_quaternion->zacc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE_QUATERNION, (const char *)hil_state_quaternion, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hil_state_quaternion_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, const float *attitude_quaternion, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 24, rollspeed);
    _mav_put_float(buf, 28, pitchspeed);
    _mav_put_float(buf, 32, yawspeed);
    _mav_put_int32_t(buf, 36, lat);
    _mav_put_int32_t(buf, 40, lon);
    _mav_put_int32_t(buf, 44, alt);
    _mav_put_int16_t(buf, 48, vx);
    _mav_put_int16_t(buf, 50, vy);
    _mav_put_int16_t(buf, 52, vz);
    _mav_put_uint16_t(buf, 54, ind_airspeed);
    _mav_put_uint16_t(buf, 56, true_airspeed);
    _mav_put_int16_t(buf, 58, xacc);
    _mav_put_int16_t(buf, 60, yacc);
    _mav_put_int16_t(buf, 62, zacc);
    _mav_put_float_array(buf, 8, attitude_quaternion, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE_QUATERNION, buf, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC);
#else
    mavlink_hil_state_quaternion_t *packet = (mavlink_hil_state_quaternion_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->ind_airspeed = ind_airspeed;
    packet->true_airspeed = true_airspeed;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;
    mav_array_memcpy(packet->attitude_quaternion, attitude_quaternion, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE_QUATERNION, (const char *)packet, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_CRC);
#endif
}
#endif

#endif

// MESSAGE HIL_STATE_QUATERNION UNPACKING


/**
 * @brief Get field time_usec from hil_state_quaternion message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_hil_state_quaternion_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field attitude_quaternion from hil_state_quaternion message
 *
 * @return  Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
 */
static inline uint16_t mavlink_msg_hil_state_quaternion_get_attitude_quaternion(const mavlink_message_t* msg, float *attitude_quaternion)
{
    return _MAV_RETURN_float_array(msg, attitude_quaternion, 4,  8);
}

/**
 * @brief Get field rollspeed from hil_state_quaternion message
 *
 * @return [rad/s] Body frame roll / phi angular speed
 */
static inline float mavlink_msg_hil_state_quaternion_get_rollspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pitchspeed from hil_state_quaternion message
 *
 * @return [rad/s] Body frame pitch / theta angular speed
 */
static inline float mavlink_msg_hil_state_quaternion_get_pitchspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field yawspeed from hil_state_quaternion message
 *
 * @return [rad/s] Body frame yaw / psi angular speed
 */
static inline float mavlink_msg_hil_state_quaternion_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field lat from hil_state_quaternion message
 *
 * @return [degE7] Latitude
 */
static inline int32_t mavlink_msg_hil_state_quaternion_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field lon from hil_state_quaternion message
 *
 * @return [degE7] Longitude
 */
static inline int32_t mavlink_msg_hil_state_quaternion_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  40);
}

/**
 * @brief Get field alt from hil_state_quaternion message
 *
 * @return [mm] Altitude
 */
static inline int32_t mavlink_msg_hil_state_quaternion_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  44);
}

/**
 * @brief Get field vx from hil_state_quaternion message
 *
 * @return [cm/s] Ground X Speed (Latitude)
 */
static inline int16_t mavlink_msg_hil_state_quaternion_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  48);
}

/**
 * @brief Get field vy from hil_state_quaternion message
 *
 * @return [cm/s] Ground Y Speed (Longitude)
 */
static inline int16_t mavlink_msg_hil_state_quaternion_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  50);
}

/**
 * @brief Get field vz from hil_state_quaternion message
 *
 * @return [cm/s] Ground Z Speed (Altitude)
 */
static inline int16_t mavlink_msg_hil_state_quaternion_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  52);
}

/**
 * @brief Get field ind_airspeed from hil_state_quaternion message
 *
 * @return [cm/s] Indicated airspeed
 */
static inline uint16_t mavlink_msg_hil_state_quaternion_get_ind_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  54);
}

/**
 * @brief Get field true_airspeed from hil_state_quaternion message
 *
 * @return [cm/s] True airspeed
 */
static inline uint16_t mavlink_msg_hil_state_quaternion_get_true_airspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  56);
}

/**
 * @brief Get field xacc from hil_state_quaternion message
 *
 * @return [mG] X acceleration
 */
static inline int16_t mavlink_msg_hil_state_quaternion_get_xacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  58);
}

/**
 * @brief Get field yacc from hil_state_quaternion message
 *
 * @return [mG] Y acceleration
 */
static inline int16_t mavlink_msg_hil_state_quaternion_get_yacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  60);
}

/**
 * @brief Get field zacc from hil_state_quaternion message
 *
 * @return [mG] Z acceleration
 */
static inline int16_t mavlink_msg_hil_state_quaternion_get_zacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  62);
}

/**
 * @brief Decode a hil_state_quaternion message into a struct
 *
 * @param msg The message to decode
 * @param hil_state_quaternion C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_state_quaternion_decode(const mavlink_message_t* msg, mavlink_hil_state_quaternion_t* hil_state_quaternion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hil_state_quaternion->time_usec = mavlink_msg_hil_state_quaternion_get_time_usec(msg);
    mavlink_msg_hil_state_quaternion_get_attitude_quaternion(msg, hil_state_quaternion->attitude_quaternion);
    hil_state_quaternion->rollspeed = mavlink_msg_hil_state_quaternion_get_rollspeed(msg);
    hil_state_quaternion->pitchspeed = mavlink_msg_hil_state_quaternion_get_pitchspeed(msg);
    hil_state_quaternion->yawspeed = mavlink_msg_hil_state_quaternion_get_yawspeed(msg);
    hil_state_quaternion->lat = mavlink_msg_hil_state_quaternion_get_lat(msg);
    hil_state_quaternion->lon = mavlink_msg_hil_state_quaternion_get_lon(msg);
    hil_state_quaternion->alt = mavlink_msg_hil_state_quaternion_get_alt(msg);
    hil_state_quaternion->vx = mavlink_msg_hil_state_quaternion_get_vx(msg);
    hil_state_quaternion->vy = mavlink_msg_hil_state_quaternion_get_vy(msg);
    hil_state_quaternion->vz = mavlink_msg_hil_state_quaternion_get_vz(msg);
    hil_state_quaternion->ind_airspeed = mavlink_msg_hil_state_quaternion_get_ind_airspeed(msg);
    hil_state_quaternion->true_airspeed = mavlink_msg_hil_state_quaternion_get_true_airspeed(msg);
    hil_state_quaternion->xacc = mavlink_msg_hil_state_quaternion_get_xacc(msg);
    hil_state_quaternion->yacc = mavlink_msg_hil_state_quaternion_get_yacc(msg);
    hil_state_quaternion->zacc = mavlink_msg_hil_state_quaternion_get_zacc(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN? msg->len : MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN;
        memset(hil_state_quaternion, 0, MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN);
    memcpy(hil_state_quaternion, _MAV_PAYLOAD(msg), len);
#endif
}
