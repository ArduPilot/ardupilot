#pragma once
// MESSAGE AIRSPEED_AUTOCAL PACKING

#define MAVLINK_MSG_ID_AIRSPEED_AUTOCAL 174


typedef struct __mavlink_airspeed_autocal_t {
 float vx; /*< [m/s] GPS velocity north.*/
 float vy; /*< [m/s] GPS velocity east.*/
 float vz; /*< [m/s] GPS velocity down.*/
 float diff_pressure; /*< [Pa] Differential pressure.*/
 float EAS2TAS; /*<  Estimated to true airspeed ratio.*/
 float ratio; /*<  Airspeed ratio.*/
 float state_x; /*<  EKF state x.*/
 float state_y; /*<  EKF state y.*/
 float state_z; /*<  EKF state z.*/
 float Pax; /*<  EKF Pax.*/
 float Pby; /*<  EKF Pby.*/
 float Pcz; /*<  EKF Pcz.*/
} mavlink_airspeed_autocal_t;

#define MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN 48
#define MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN 48
#define MAVLINK_MSG_ID_174_LEN 48
#define MAVLINK_MSG_ID_174_MIN_LEN 48

#define MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_CRC 167
#define MAVLINK_MSG_ID_174_CRC 167



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AIRSPEED_AUTOCAL { \
    174, \
    "AIRSPEED_AUTOCAL", \
    12, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_airspeed_autocal_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_airspeed_autocal_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_airspeed_autocal_t, vz) }, \
         { "diff_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_airspeed_autocal_t, diff_pressure) }, \
         { "EAS2TAS", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_airspeed_autocal_t, EAS2TAS) }, \
         { "ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_airspeed_autocal_t, ratio) }, \
         { "state_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_airspeed_autocal_t, state_x) }, \
         { "state_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_airspeed_autocal_t, state_y) }, \
         { "state_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_airspeed_autocal_t, state_z) }, \
         { "Pax", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_airspeed_autocal_t, Pax) }, \
         { "Pby", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_airspeed_autocal_t, Pby) }, \
         { "Pcz", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_airspeed_autocal_t, Pcz) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AIRSPEED_AUTOCAL { \
    "AIRSPEED_AUTOCAL", \
    12, \
    {  { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_airspeed_autocal_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_airspeed_autocal_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_airspeed_autocal_t, vz) }, \
         { "diff_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_airspeed_autocal_t, diff_pressure) }, \
         { "EAS2TAS", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_airspeed_autocal_t, EAS2TAS) }, \
         { "ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_airspeed_autocal_t, ratio) }, \
         { "state_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_airspeed_autocal_t, state_x) }, \
         { "state_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_airspeed_autocal_t, state_y) }, \
         { "state_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_airspeed_autocal_t, state_z) }, \
         { "Pax", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_airspeed_autocal_t, Pax) }, \
         { "Pby", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_airspeed_autocal_t, Pby) }, \
         { "Pcz", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_airspeed_autocal_t, Pcz) }, \
         } \
}
#endif

/**
 * @brief Pack a airspeed_autocal message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] GPS velocity north.
 * @param vy [m/s] GPS velocity east.
 * @param vz [m/s] GPS velocity down.
 * @param diff_pressure [Pa] Differential pressure.
 * @param EAS2TAS  Estimated to true airspeed ratio.
 * @param ratio  Airspeed ratio.
 * @param state_x  EKF state x.
 * @param state_y  EKF state y.
 * @param state_z  EKF state z.
 * @param Pax  EKF Pax.
 * @param Pby  EKF Pby.
 * @param Pcz  EKF Pcz.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeed_autocal_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float vx, float vy, float vz, float diff_pressure, float EAS2TAS, float ratio, float state_x, float state_y, float state_z, float Pax, float Pby, float Pcz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, diff_pressure);
    _mav_put_float(buf, 16, EAS2TAS);
    _mav_put_float(buf, 20, ratio);
    _mav_put_float(buf, 24, state_x);
    _mav_put_float(buf, 28, state_y);
    _mav_put_float(buf, 32, state_z);
    _mav_put_float(buf, 36, Pax);
    _mav_put_float(buf, 40, Pby);
    _mav_put_float(buf, 44, Pcz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN);
#else
    mavlink_airspeed_autocal_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.diff_pressure = diff_pressure;
    packet.EAS2TAS = EAS2TAS;
    packet.ratio = ratio;
    packet.state_x = state_x;
    packet.state_y = state_y;
    packet.state_z = state_z;
    packet.Pax = Pax;
    packet.Pby = Pby;
    packet.Pcz = Pcz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRSPEED_AUTOCAL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_CRC);
}

/**
 * @brief Pack a airspeed_autocal message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx [m/s] GPS velocity north.
 * @param vy [m/s] GPS velocity east.
 * @param vz [m/s] GPS velocity down.
 * @param diff_pressure [Pa] Differential pressure.
 * @param EAS2TAS  Estimated to true airspeed ratio.
 * @param ratio  Airspeed ratio.
 * @param state_x  EKF state x.
 * @param state_y  EKF state y.
 * @param state_z  EKF state z.
 * @param Pax  EKF Pax.
 * @param Pby  EKF Pby.
 * @param Pcz  EKF Pcz.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeed_autocal_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float vx, float vy, float vz, float diff_pressure, float EAS2TAS, float ratio, float state_x, float state_y, float state_z, float Pax, float Pby, float Pcz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, diff_pressure);
    _mav_put_float(buf, 16, EAS2TAS);
    _mav_put_float(buf, 20, ratio);
    _mav_put_float(buf, 24, state_x);
    _mav_put_float(buf, 28, state_y);
    _mav_put_float(buf, 32, state_z);
    _mav_put_float(buf, 36, Pax);
    _mav_put_float(buf, 40, Pby);
    _mav_put_float(buf, 44, Pcz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN);
#else
    mavlink_airspeed_autocal_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.diff_pressure = diff_pressure;
    packet.EAS2TAS = EAS2TAS;
    packet.ratio = ratio;
    packet.state_x = state_x;
    packet.state_y = state_y;
    packet.state_z = state_z;
    packet.Pax = Pax;
    packet.Pby = Pby;
    packet.Pcz = Pcz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRSPEED_AUTOCAL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN);
#endif
}

/**
 * @brief Pack a airspeed_autocal message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vx [m/s] GPS velocity north.
 * @param vy [m/s] GPS velocity east.
 * @param vz [m/s] GPS velocity down.
 * @param diff_pressure [Pa] Differential pressure.
 * @param EAS2TAS  Estimated to true airspeed ratio.
 * @param ratio  Airspeed ratio.
 * @param state_x  EKF state x.
 * @param state_y  EKF state y.
 * @param state_z  EKF state z.
 * @param Pax  EKF Pax.
 * @param Pby  EKF Pby.
 * @param Pcz  EKF Pcz.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeed_autocal_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float vx,float vy,float vz,float diff_pressure,float EAS2TAS,float ratio,float state_x,float state_y,float state_z,float Pax,float Pby,float Pcz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, diff_pressure);
    _mav_put_float(buf, 16, EAS2TAS);
    _mav_put_float(buf, 20, ratio);
    _mav_put_float(buf, 24, state_x);
    _mav_put_float(buf, 28, state_y);
    _mav_put_float(buf, 32, state_z);
    _mav_put_float(buf, 36, Pax);
    _mav_put_float(buf, 40, Pby);
    _mav_put_float(buf, 44, Pcz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN);
#else
    mavlink_airspeed_autocal_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.diff_pressure = diff_pressure;
    packet.EAS2TAS = EAS2TAS;
    packet.ratio = ratio;
    packet.state_x = state_x;
    packet.state_y = state_y;
    packet.state_z = state_z;
    packet.Pax = Pax;
    packet.Pby = Pby;
    packet.Pcz = Pcz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRSPEED_AUTOCAL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_CRC);
}

/**
 * @brief Encode a airspeed_autocal struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airspeed_autocal C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airspeed_autocal_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airspeed_autocal_t* airspeed_autocal)
{
    return mavlink_msg_airspeed_autocal_pack(system_id, component_id, msg, airspeed_autocal->vx, airspeed_autocal->vy, airspeed_autocal->vz, airspeed_autocal->diff_pressure, airspeed_autocal->EAS2TAS, airspeed_autocal->ratio, airspeed_autocal->state_x, airspeed_autocal->state_y, airspeed_autocal->state_z, airspeed_autocal->Pax, airspeed_autocal->Pby, airspeed_autocal->Pcz);
}

/**
 * @brief Encode a airspeed_autocal struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param airspeed_autocal C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airspeed_autocal_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_airspeed_autocal_t* airspeed_autocal)
{
    return mavlink_msg_airspeed_autocal_pack_chan(system_id, component_id, chan, msg, airspeed_autocal->vx, airspeed_autocal->vy, airspeed_autocal->vz, airspeed_autocal->diff_pressure, airspeed_autocal->EAS2TAS, airspeed_autocal->ratio, airspeed_autocal->state_x, airspeed_autocal->state_y, airspeed_autocal->state_z, airspeed_autocal->Pax, airspeed_autocal->Pby, airspeed_autocal->Pcz);
}

/**
 * @brief Encode a airspeed_autocal struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param airspeed_autocal C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airspeed_autocal_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_airspeed_autocal_t* airspeed_autocal)
{
    return mavlink_msg_airspeed_autocal_pack_status(system_id, component_id, _status, msg,  airspeed_autocal->vx, airspeed_autocal->vy, airspeed_autocal->vz, airspeed_autocal->diff_pressure, airspeed_autocal->EAS2TAS, airspeed_autocal->ratio, airspeed_autocal->state_x, airspeed_autocal->state_y, airspeed_autocal->state_z, airspeed_autocal->Pax, airspeed_autocal->Pby, airspeed_autocal->Pcz);
}

/**
 * @brief Send a airspeed_autocal message
 * @param chan MAVLink channel to send the message
 *
 * @param vx [m/s] GPS velocity north.
 * @param vy [m/s] GPS velocity east.
 * @param vz [m/s] GPS velocity down.
 * @param diff_pressure [Pa] Differential pressure.
 * @param EAS2TAS  Estimated to true airspeed ratio.
 * @param ratio  Airspeed ratio.
 * @param state_x  EKF state x.
 * @param state_y  EKF state y.
 * @param state_z  EKF state z.
 * @param Pax  EKF Pax.
 * @param Pby  EKF Pby.
 * @param Pcz  EKF Pcz.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airspeed_autocal_send(mavlink_channel_t chan, float vx, float vy, float vz, float diff_pressure, float EAS2TAS, float ratio, float state_x, float state_y, float state_z, float Pax, float Pby, float Pcz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN];
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, diff_pressure);
    _mav_put_float(buf, 16, EAS2TAS);
    _mav_put_float(buf, 20, ratio);
    _mav_put_float(buf, 24, state_x);
    _mav_put_float(buf, 28, state_y);
    _mav_put_float(buf, 32, state_z);
    _mav_put_float(buf, 36, Pax);
    _mav_put_float(buf, 40, Pby);
    _mav_put_float(buf, 44, Pcz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL, buf, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_CRC);
#else
    mavlink_airspeed_autocal_t packet;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.diff_pressure = diff_pressure;
    packet.EAS2TAS = EAS2TAS;
    packet.ratio = ratio;
    packet.state_x = state_x;
    packet.state_y = state_y;
    packet.state_z = state_z;
    packet.Pax = Pax;
    packet.Pby = Pby;
    packet.Pcz = Pcz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL, (const char *)&packet, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_CRC);
#endif
}

/**
 * @brief Send a airspeed_autocal message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_airspeed_autocal_send_struct(mavlink_channel_t chan, const mavlink_airspeed_autocal_t* airspeed_autocal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_airspeed_autocal_send(chan, airspeed_autocal->vx, airspeed_autocal->vy, airspeed_autocal->vz, airspeed_autocal->diff_pressure, airspeed_autocal->EAS2TAS, airspeed_autocal->ratio, airspeed_autocal->state_x, airspeed_autocal->state_y, airspeed_autocal->state_z, airspeed_autocal->Pax, airspeed_autocal->Pby, airspeed_autocal->Pcz);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL, (const char *)airspeed_autocal, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_CRC);
#endif
}

#if MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_airspeed_autocal_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vx, float vy, float vz, float diff_pressure, float EAS2TAS, float ratio, float state_x, float state_y, float state_z, float Pax, float Pby, float Pcz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vx);
    _mav_put_float(buf, 4, vy);
    _mav_put_float(buf, 8, vz);
    _mav_put_float(buf, 12, diff_pressure);
    _mav_put_float(buf, 16, EAS2TAS);
    _mav_put_float(buf, 20, ratio);
    _mav_put_float(buf, 24, state_x);
    _mav_put_float(buf, 28, state_y);
    _mav_put_float(buf, 32, state_z);
    _mav_put_float(buf, 36, Pax);
    _mav_put_float(buf, 40, Pby);
    _mav_put_float(buf, 44, Pcz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL, buf, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_CRC);
#else
    mavlink_airspeed_autocal_t *packet = (mavlink_airspeed_autocal_t *)msgbuf;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->diff_pressure = diff_pressure;
    packet->EAS2TAS = EAS2TAS;
    packet->ratio = ratio;
    packet->state_x = state_x;
    packet->state_y = state_y;
    packet->state_z = state_z;
    packet->Pax = Pax;
    packet->Pby = Pby;
    packet->Pcz = Pcz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL, (const char *)packet, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_CRC);
#endif
}
#endif

#endif

// MESSAGE AIRSPEED_AUTOCAL UNPACKING


/**
 * @brief Get field vx from airspeed_autocal message
 *
 * @return [m/s] GPS velocity north.
 */
static inline float mavlink_msg_airspeed_autocal_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vy from airspeed_autocal message
 *
 * @return [m/s] GPS velocity east.
 */
static inline float mavlink_msg_airspeed_autocal_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vz from airspeed_autocal message
 *
 * @return [m/s] GPS velocity down.
 */
static inline float mavlink_msg_airspeed_autocal_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field diff_pressure from airspeed_autocal message
 *
 * @return [Pa] Differential pressure.
 */
static inline float mavlink_msg_airspeed_autocal_get_diff_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field EAS2TAS from airspeed_autocal message
 *
 * @return  Estimated to true airspeed ratio.
 */
static inline float mavlink_msg_airspeed_autocal_get_EAS2TAS(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ratio from airspeed_autocal message
 *
 * @return  Airspeed ratio.
 */
static inline float mavlink_msg_airspeed_autocal_get_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field state_x from airspeed_autocal message
 *
 * @return  EKF state x.
 */
static inline float mavlink_msg_airspeed_autocal_get_state_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field state_y from airspeed_autocal message
 *
 * @return  EKF state y.
 */
static inline float mavlink_msg_airspeed_autocal_get_state_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field state_z from airspeed_autocal message
 *
 * @return  EKF state z.
 */
static inline float mavlink_msg_airspeed_autocal_get_state_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field Pax from airspeed_autocal message
 *
 * @return  EKF Pax.
 */
static inline float mavlink_msg_airspeed_autocal_get_Pax(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field Pby from airspeed_autocal message
 *
 * @return  EKF Pby.
 */
static inline float mavlink_msg_airspeed_autocal_get_Pby(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field Pcz from airspeed_autocal message
 *
 * @return  EKF Pcz.
 */
static inline float mavlink_msg_airspeed_autocal_get_Pcz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a airspeed_autocal message into a struct
 *
 * @param msg The message to decode
 * @param airspeed_autocal C-struct to decode the message contents into
 */
static inline void mavlink_msg_airspeed_autocal_decode(const mavlink_message_t* msg, mavlink_airspeed_autocal_t* airspeed_autocal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    airspeed_autocal->vx = mavlink_msg_airspeed_autocal_get_vx(msg);
    airspeed_autocal->vy = mavlink_msg_airspeed_autocal_get_vy(msg);
    airspeed_autocal->vz = mavlink_msg_airspeed_autocal_get_vz(msg);
    airspeed_autocal->diff_pressure = mavlink_msg_airspeed_autocal_get_diff_pressure(msg);
    airspeed_autocal->EAS2TAS = mavlink_msg_airspeed_autocal_get_EAS2TAS(msg);
    airspeed_autocal->ratio = mavlink_msg_airspeed_autocal_get_ratio(msg);
    airspeed_autocal->state_x = mavlink_msg_airspeed_autocal_get_state_x(msg);
    airspeed_autocal->state_y = mavlink_msg_airspeed_autocal_get_state_y(msg);
    airspeed_autocal->state_z = mavlink_msg_airspeed_autocal_get_state_z(msg);
    airspeed_autocal->Pax = mavlink_msg_airspeed_autocal_get_Pax(msg);
    airspeed_autocal->Pby = mavlink_msg_airspeed_autocal_get_Pby(msg);
    airspeed_autocal->Pcz = mavlink_msg_airspeed_autocal_get_Pcz(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN? msg->len : MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN;
        memset(airspeed_autocal, 0, MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN);
    memcpy(airspeed_autocal, _MAV_PAYLOAD(msg), len);
#endif
}
