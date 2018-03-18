#pragma once
// MESSAGE HIL_STATE PACKING

#define MAVLINK_MSG_ID_HIL_STATE 90

MAVPACKED(
typedef struct __mavlink_hil_state_t {
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 float roll; /*< Roll angle (rad)*/
 float pitch; /*< Pitch angle (rad)*/
 float yaw; /*< Yaw angle (rad)*/
 float rollspeed; /*< Body frame roll / phi angular speed (rad/s)*/
 float pitchspeed; /*< Body frame pitch / theta angular speed (rad/s)*/
 float yawspeed; /*< Body frame yaw / psi angular speed (rad/s)*/
 int32_t lat; /*< Latitude, expressed as degrees * 1E7*/
 int32_t lon; /*< Longitude, expressed as degrees * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters)*/
 int16_t vx; /*< Ground X Speed (Latitude), expressed as m/s * 100*/
 int16_t vy; /*< Ground Y Speed (Longitude), expressed as m/s * 100*/
 int16_t vz; /*< Ground Z Speed (Altitude), expressed as m/s * 100*/
 int16_t xacc; /*< X acceleration (mg)*/
 int16_t yacc; /*< Y acceleration (mg)*/
 int16_t zacc; /*< Z acceleration (mg)*/
}) mavlink_hil_state_t;

#define MAVLINK_MSG_ID_HIL_STATE_LEN 56
#define MAVLINK_MSG_ID_HIL_STATE_MIN_LEN 56
#define MAVLINK_MSG_ID_90_LEN 56
#define MAVLINK_MSG_ID_90_MIN_LEN 56

#define MAVLINK_MSG_ID_HIL_STATE_CRC 183
#define MAVLINK_MSG_ID_90_CRC 183



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIL_STATE { \
    90, \
    "HIL_STATE", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_state_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_state_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_state_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_state_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_state_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_state_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_state_t, yawspeed) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_hil_state_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_hil_state_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_hil_state_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_hil_state_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 46, offsetof(mavlink_hil_state_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_hil_state_t, vz) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 50, offsetof(mavlink_hil_state_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 52, offsetof(mavlink_hil_state_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 54, offsetof(mavlink_hil_state_t, zacc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIL_STATE { \
    "HIL_STATE", \
    16, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_state_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_state_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_state_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_state_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_state_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_state_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_state_t, yawspeed) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_hil_state_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_hil_state_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_hil_state_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_hil_state_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 46, offsetof(mavlink_hil_state_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_hil_state_t, vz) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 50, offsetof(mavlink_hil_state_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 52, offsetof(mavlink_hil_state_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 54, offsetof(mavlink_hil_state_t, zacc) }, \
         } \
}
#endif

/**
 * @brief Pack a hil_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Body frame roll / phi angular speed (rad/s)
 * @param pitchspeed Body frame pitch / theta angular speed (rad/s)
 * @param yawspeed Body frame yaw / psi angular speed (rad/s)
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_STATE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, rollspeed);
    _mav_put_float(buf, 24, pitchspeed);
    _mav_put_float(buf, 28, yawspeed);
    _mav_put_int32_t(buf, 32, lat);
    _mav_put_int32_t(buf, 36, lon);
    _mav_put_int32_t(buf, 40, alt);
    _mav_put_int16_t(buf, 44, vx);
    _mav_put_int16_t(buf, 46, vy);
    _mav_put_int16_t(buf, 48, vz);
    _mav_put_int16_t(buf, 50, xacc);
    _mav_put_int16_t(buf, 52, yacc);
    _mav_put_int16_t(buf, 54, zacc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_STATE_LEN);
#else
    mavlink_hil_state_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_STATE_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_LEN, MAVLINK_MSG_ID_HIL_STATE_CRC);
}

/**
 * @brief Pack a hil_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Body frame roll / phi angular speed (rad/s)
 * @param pitchspeed Body frame pitch / theta angular speed (rad/s)
 * @param yawspeed Body frame yaw / psi angular speed (rad/s)
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed,int32_t lat,int32_t lon,int32_t alt,int16_t vx,int16_t vy,int16_t vz,int16_t xacc,int16_t yacc,int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_STATE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, rollspeed);
    _mav_put_float(buf, 24, pitchspeed);
    _mav_put_float(buf, 28, yawspeed);
    _mav_put_int32_t(buf, 32, lat);
    _mav_put_int32_t(buf, 36, lon);
    _mav_put_int32_t(buf, 40, alt);
    _mav_put_int16_t(buf, 44, vx);
    _mav_put_int16_t(buf, 46, vy);
    _mav_put_int16_t(buf, 48, vz);
    _mav_put_int16_t(buf, 50, xacc);
    _mav_put_int16_t(buf, 52, yacc);
    _mav_put_int16_t(buf, 54, zacc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_STATE_LEN);
#else
    mavlink_hil_state_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_STATE_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_LEN, MAVLINK_MSG_ID_HIL_STATE_CRC);
}

/**
 * @brief Encode a hil_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_state_t* hil_state)
{
    return mavlink_msg_hil_state_pack(system_id, component_id, msg, hil_state->time_usec, hil_state->roll, hil_state->pitch, hil_state->yaw, hil_state->rollspeed, hil_state->pitchspeed, hil_state->yawspeed, hil_state->lat, hil_state->lon, hil_state->alt, hil_state->vx, hil_state->vy, hil_state->vz, hil_state->xacc, hil_state->yacc, hil_state->zacc);
}

/**
 * @brief Encode a hil_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hil_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hil_state_t* hil_state)
{
    return mavlink_msg_hil_state_pack_chan(system_id, component_id, chan, msg, hil_state->time_usec, hil_state->roll, hil_state->pitch, hil_state->yaw, hil_state->rollspeed, hil_state->pitchspeed, hil_state->yawspeed, hil_state->lat, hil_state->lon, hil_state->alt, hil_state->vx, hil_state->vy, hil_state->vz, hil_state->xacc, hil_state->yacc, hil_state->zacc);
}

/**
 * @brief Send a hil_state message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Body frame roll / phi angular speed (rad/s)
 * @param pitchspeed Body frame pitch / theta angular speed (rad/s)
 * @param yawspeed Body frame yaw / psi angular speed (rad/s)
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_state_send(mavlink_channel_t chan, uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_STATE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, rollspeed);
    _mav_put_float(buf, 24, pitchspeed);
    _mav_put_float(buf, 28, yawspeed);
    _mav_put_int32_t(buf, 32, lat);
    _mav_put_int32_t(buf, 36, lon);
    _mav_put_int32_t(buf, 40, alt);
    _mav_put_int16_t(buf, 44, vx);
    _mav_put_int16_t(buf, 46, vy);
    _mav_put_int16_t(buf, 48, vz);
    _mav_put_int16_t(buf, 50, xacc);
    _mav_put_int16_t(buf, 52, yacc);
    _mav_put_int16_t(buf, 54, zacc);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE, buf, MAVLINK_MSG_ID_HIL_STATE_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_LEN, MAVLINK_MSG_ID_HIL_STATE_CRC);
#else
    mavlink_hil_state_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE, (const char *)&packet, MAVLINK_MSG_ID_HIL_STATE_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_LEN, MAVLINK_MSG_ID_HIL_STATE_CRC);
#endif
}

/**
 * @brief Send a hil_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hil_state_send_struct(mavlink_channel_t chan, const mavlink_hil_state_t* hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hil_state_send(chan, hil_state->time_usec, hil_state->roll, hil_state->pitch, hil_state->yaw, hil_state->rollspeed, hil_state->pitchspeed, hil_state->yawspeed, hil_state->lat, hil_state->lon, hil_state->alt, hil_state->vx, hil_state->vy, hil_state->vz, hil_state->xacc, hil_state->yacc, hil_state->zacc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE, (const char *)hil_state, MAVLINK_MSG_ID_HIL_STATE_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_LEN, MAVLINK_MSG_ID_HIL_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIL_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hil_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_float(buf, 20, rollspeed);
    _mav_put_float(buf, 24, pitchspeed);
    _mav_put_float(buf, 28, yawspeed);
    _mav_put_int32_t(buf, 32, lat);
    _mav_put_int32_t(buf, 36, lon);
    _mav_put_int32_t(buf, 40, alt);
    _mav_put_int16_t(buf, 44, vx);
    _mav_put_int16_t(buf, 46, vy);
    _mav_put_int16_t(buf, 48, vz);
    _mav_put_int16_t(buf, 50, xacc);
    _mav_put_int16_t(buf, 52, yacc);
    _mav_put_int16_t(buf, 54, zacc);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE, buf, MAVLINK_MSG_ID_HIL_STATE_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_LEN, MAVLINK_MSG_ID_HIL_STATE_CRC);
#else
    mavlink_hil_state_t *packet = (mavlink_hil_state_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE, (const char *)packet, MAVLINK_MSG_ID_HIL_STATE_MIN_LEN, MAVLINK_MSG_ID_HIL_STATE_LEN, MAVLINK_MSG_ID_HIL_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE HIL_STATE UNPACKING


/**
 * @brief Get field time_usec from hil_state message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_hil_state_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field roll from hil_state message
 *
 * @return Roll angle (rad)
 */
static inline float mavlink_msg_hil_state_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch from hil_state message
 *
 * @return Pitch angle (rad)
 */
static inline float mavlink_msg_hil_state_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yaw from hil_state message
 *
 * @return Yaw angle (rad)
 */
static inline float mavlink_msg_hil_state_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rollspeed from hil_state message
 *
 * @return Body frame roll / phi angular speed (rad/s)
 */
static inline float mavlink_msg_hil_state_get_rollspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitchspeed from hil_state message
 *
 * @return Body frame pitch / theta angular speed (rad/s)
 */
static inline float mavlink_msg_hil_state_get_pitchspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field yawspeed from hil_state message
 *
 * @return Body frame yaw / psi angular speed (rad/s)
 */
static inline float mavlink_msg_hil_state_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field lat from hil_state message
 *
 * @return Latitude, expressed as degrees * 1E7
 */
static inline int32_t mavlink_msg_hil_state_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field lon from hil_state message
 *
 * @return Longitude, expressed as degrees * 1E7
 */
static inline int32_t mavlink_msg_hil_state_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field alt from hil_state message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_hil_state_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  40);
}

/**
 * @brief Get field vx from hil_state message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_hil_state_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  44);
}

/**
 * @brief Get field vy from hil_state message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_hil_state_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  46);
}

/**
 * @brief Get field vz from hil_state message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_hil_state_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  48);
}

/**
 * @brief Get field xacc from hil_state message
 *
 * @return X acceleration (mg)
 */
static inline int16_t mavlink_msg_hil_state_get_xacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  50);
}

/**
 * @brief Get field yacc from hil_state message
 *
 * @return Y acceleration (mg)
 */
static inline int16_t mavlink_msg_hil_state_get_yacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  52);
}

/**
 * @brief Get field zacc from hil_state message
 *
 * @return Z acceleration (mg)
 */
static inline int16_t mavlink_msg_hil_state_get_zacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  54);
}

/**
 * @brief Decode a hil_state message into a struct
 *
 * @param msg The message to decode
 * @param hil_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_state_decode(const mavlink_message_t* msg, mavlink_hil_state_t* hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hil_state->time_usec = mavlink_msg_hil_state_get_time_usec(msg);
    hil_state->roll = mavlink_msg_hil_state_get_roll(msg);
    hil_state->pitch = mavlink_msg_hil_state_get_pitch(msg);
    hil_state->yaw = mavlink_msg_hil_state_get_yaw(msg);
    hil_state->rollspeed = mavlink_msg_hil_state_get_rollspeed(msg);
    hil_state->pitchspeed = mavlink_msg_hil_state_get_pitchspeed(msg);
    hil_state->yawspeed = mavlink_msg_hil_state_get_yawspeed(msg);
    hil_state->lat = mavlink_msg_hil_state_get_lat(msg);
    hil_state->lon = mavlink_msg_hil_state_get_lon(msg);
    hil_state->alt = mavlink_msg_hil_state_get_alt(msg);
    hil_state->vx = mavlink_msg_hil_state_get_vx(msg);
    hil_state->vy = mavlink_msg_hil_state_get_vy(msg);
    hil_state->vz = mavlink_msg_hil_state_get_vz(msg);
    hil_state->xacc = mavlink_msg_hil_state_get_xacc(msg);
    hil_state->yacc = mavlink_msg_hil_state_get_yacc(msg);
    hil_state->zacc = mavlink_msg_hil_state_get_zacc(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIL_STATE_LEN? msg->len : MAVLINK_MSG_ID_HIL_STATE_LEN;
        memset(hil_state, 0, MAVLINK_MSG_ID_HIL_STATE_LEN);
    memcpy(hil_state, _MAV_PAYLOAD(msg), len);
#endif
}
