#pragma once
// MESSAGE CAMERA_TRACKING_GEO_STATUS PACKING

#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS 276


typedef struct __mavlink_camera_tracking_geo_status_t {
 int32_t lat; /*< [degE7] Latitude of tracked object*/
 int32_t lon; /*< [degE7] Longitude of tracked object*/
 float alt; /*< [m] Altitude of tracked object(AMSL, WGS84)*/
 float h_acc; /*< [m] Horizontal accuracy. NAN if unknown*/
 float v_acc; /*< [m] Vertical accuracy. NAN if unknown*/
 float vel_n; /*< [m/s] North velocity of tracked object. NAN if unknown*/
 float vel_e; /*< [m/s] East velocity of tracked object. NAN if unknown*/
 float vel_d; /*< [m/s] Down velocity of tracked object. NAN if unknown*/
 float vel_acc; /*< [m/s] Velocity accuracy. NAN if unknown*/
 float dist; /*< [m] Distance between camera and tracked object. NAN if unknown*/
 float hdg; /*< [rad] Heading in radians, in NED. NAN if unknown*/
 float hdg_acc; /*< [rad] Accuracy of heading, in NED. NAN if unknown*/
 uint8_t tracking_status; /*<  Current tracking status*/
} mavlink_camera_tracking_geo_status_t;

#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN 49
#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN 49
#define MAVLINK_MSG_ID_276_LEN 49
#define MAVLINK_MSG_ID_276_MIN_LEN 49

#define MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC 18
#define MAVLINK_MSG_ID_276_CRC 18



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAMERA_TRACKING_GEO_STATUS { \
    276, \
    "CAMERA_TRACKING_GEO_STATUS", \
    13, \
    {  { "tracking_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_camera_tracking_geo_status_t, tracking_status) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_camera_tracking_geo_status_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_camera_tracking_geo_status_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_tracking_geo_status_t, alt) }, \
         { "h_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_tracking_geo_status_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_tracking_geo_status_t, v_acc) }, \
         { "vel_n", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_tracking_geo_status_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_tracking_geo_status_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_camera_tracking_geo_status_t, vel_d) }, \
         { "vel_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_camera_tracking_geo_status_t, vel_acc) }, \
         { "dist", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_camera_tracking_geo_status_t, dist) }, \
         { "hdg", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_camera_tracking_geo_status_t, hdg) }, \
         { "hdg_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_camera_tracking_geo_status_t, hdg_acc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAMERA_TRACKING_GEO_STATUS { \
    "CAMERA_TRACKING_GEO_STATUS", \
    13, \
    {  { "tracking_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_camera_tracking_geo_status_t, tracking_status) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_camera_tracking_geo_status_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_camera_tracking_geo_status_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_tracking_geo_status_t, alt) }, \
         { "h_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_tracking_geo_status_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_tracking_geo_status_t, v_acc) }, \
         { "vel_n", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_tracking_geo_status_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_camera_tracking_geo_status_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_camera_tracking_geo_status_t, vel_d) }, \
         { "vel_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_camera_tracking_geo_status_t, vel_acc) }, \
         { "dist", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_camera_tracking_geo_status_t, dist) }, \
         { "hdg", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_camera_tracking_geo_status_t, hdg) }, \
         { "hdg_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_camera_tracking_geo_status_t, hdg_acc) }, \
         } \
}
#endif

/**
 * @brief Pack a camera_tracking_geo_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tracking_status  Current tracking status
 * @param lat [degE7] Latitude of tracked object
 * @param lon [degE7] Longitude of tracked object
 * @param alt [m] Altitude of tracked object(AMSL, WGS84)
 * @param h_acc [m] Horizontal accuracy. NAN if unknown
 * @param v_acc [m] Vertical accuracy. NAN if unknown
 * @param vel_n [m/s] North velocity of tracked object. NAN if unknown
 * @param vel_e [m/s] East velocity of tracked object. NAN if unknown
 * @param vel_d [m/s] Down velocity of tracked object. NAN if unknown
 * @param vel_acc [m/s] Velocity accuracy. NAN if unknown
 * @param dist [m] Distance between camera and tracked object. NAN if unknown
 * @param hdg [rad] Heading in radians, in NED. NAN if unknown
 * @param hdg_acc [rad] Accuracy of heading, in NED. NAN if unknown
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_tracking_geo_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_float(buf, 8, alt);
    _mav_put_float(buf, 12, h_acc);
    _mav_put_float(buf, 16, v_acc);
    _mav_put_float(buf, 20, vel_n);
    _mav_put_float(buf, 24, vel_e);
    _mav_put_float(buf, 28, vel_d);
    _mav_put_float(buf, 32, vel_acc);
    _mav_put_float(buf, 36, dist);
    _mav_put_float(buf, 40, hdg);
    _mav_put_float(buf, 44, hdg_acc);
    _mav_put_uint8_t(buf, 48, tracking_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN);
#else
    mavlink_camera_tracking_geo_status_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.vel_acc = vel_acc;
    packet.dist = dist;
    packet.hdg = hdg;
    packet.hdg_acc = hdg_acc;
    packet.tracking_status = tracking_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC);
}

/**
 * @brief Pack a camera_tracking_geo_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param tracking_status  Current tracking status
 * @param lat [degE7] Latitude of tracked object
 * @param lon [degE7] Longitude of tracked object
 * @param alt [m] Altitude of tracked object(AMSL, WGS84)
 * @param h_acc [m] Horizontal accuracy. NAN if unknown
 * @param v_acc [m] Vertical accuracy. NAN if unknown
 * @param vel_n [m/s] North velocity of tracked object. NAN if unknown
 * @param vel_e [m/s] East velocity of tracked object. NAN if unknown
 * @param vel_d [m/s] Down velocity of tracked object. NAN if unknown
 * @param vel_acc [m/s] Velocity accuracy. NAN if unknown
 * @param dist [m] Distance between camera and tracked object. NAN if unknown
 * @param hdg [rad] Heading in radians, in NED. NAN if unknown
 * @param hdg_acc [rad] Accuracy of heading, in NED. NAN if unknown
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_tracking_geo_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_float(buf, 8, alt);
    _mav_put_float(buf, 12, h_acc);
    _mav_put_float(buf, 16, v_acc);
    _mav_put_float(buf, 20, vel_n);
    _mav_put_float(buf, 24, vel_e);
    _mav_put_float(buf, 28, vel_d);
    _mav_put_float(buf, 32, vel_acc);
    _mav_put_float(buf, 36, dist);
    _mav_put_float(buf, 40, hdg);
    _mav_put_float(buf, 44, hdg_acc);
    _mav_put_uint8_t(buf, 48, tracking_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN);
#else
    mavlink_camera_tracking_geo_status_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.vel_acc = vel_acc;
    packet.dist = dist;
    packet.hdg = hdg;
    packet.hdg_acc = hdg_acc;
    packet.tracking_status = tracking_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN);
#endif
}

/**
 * @brief Pack a camera_tracking_geo_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tracking_status  Current tracking status
 * @param lat [degE7] Latitude of tracked object
 * @param lon [degE7] Longitude of tracked object
 * @param alt [m] Altitude of tracked object(AMSL, WGS84)
 * @param h_acc [m] Horizontal accuracy. NAN if unknown
 * @param v_acc [m] Vertical accuracy. NAN if unknown
 * @param vel_n [m/s] North velocity of tracked object. NAN if unknown
 * @param vel_e [m/s] East velocity of tracked object. NAN if unknown
 * @param vel_d [m/s] Down velocity of tracked object. NAN if unknown
 * @param vel_acc [m/s] Velocity accuracy. NAN if unknown
 * @param dist [m] Distance between camera and tracked object. NAN if unknown
 * @param hdg [rad] Heading in radians, in NED. NAN if unknown
 * @param hdg_acc [rad] Accuracy of heading, in NED. NAN if unknown
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_tracking_geo_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t tracking_status,int32_t lat,int32_t lon,float alt,float h_acc,float v_acc,float vel_n,float vel_e,float vel_d,float vel_acc,float dist,float hdg,float hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_float(buf, 8, alt);
    _mav_put_float(buf, 12, h_acc);
    _mav_put_float(buf, 16, v_acc);
    _mav_put_float(buf, 20, vel_n);
    _mav_put_float(buf, 24, vel_e);
    _mav_put_float(buf, 28, vel_d);
    _mav_put_float(buf, 32, vel_acc);
    _mav_put_float(buf, 36, dist);
    _mav_put_float(buf, 40, hdg);
    _mav_put_float(buf, 44, hdg_acc);
    _mav_put_uint8_t(buf, 48, tracking_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN);
#else
    mavlink_camera_tracking_geo_status_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.vel_acc = vel_acc;
    packet.dist = dist;
    packet.hdg = hdg;
    packet.hdg_acc = hdg_acc;
    packet.tracking_status = tracking_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC);
}

/**
 * @brief Encode a camera_tracking_geo_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_tracking_geo_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_tracking_geo_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_tracking_geo_status_t* camera_tracking_geo_status)
{
    return mavlink_msg_camera_tracking_geo_status_pack(system_id, component_id, msg, camera_tracking_geo_status->tracking_status, camera_tracking_geo_status->lat, camera_tracking_geo_status->lon, camera_tracking_geo_status->alt, camera_tracking_geo_status->h_acc, camera_tracking_geo_status->v_acc, camera_tracking_geo_status->vel_n, camera_tracking_geo_status->vel_e, camera_tracking_geo_status->vel_d, camera_tracking_geo_status->vel_acc, camera_tracking_geo_status->dist, camera_tracking_geo_status->hdg, camera_tracking_geo_status->hdg_acc);
}

/**
 * @brief Encode a camera_tracking_geo_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_tracking_geo_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_tracking_geo_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_tracking_geo_status_t* camera_tracking_geo_status)
{
    return mavlink_msg_camera_tracking_geo_status_pack_chan(system_id, component_id, chan, msg, camera_tracking_geo_status->tracking_status, camera_tracking_geo_status->lat, camera_tracking_geo_status->lon, camera_tracking_geo_status->alt, camera_tracking_geo_status->h_acc, camera_tracking_geo_status->v_acc, camera_tracking_geo_status->vel_n, camera_tracking_geo_status->vel_e, camera_tracking_geo_status->vel_d, camera_tracking_geo_status->vel_acc, camera_tracking_geo_status->dist, camera_tracking_geo_status->hdg, camera_tracking_geo_status->hdg_acc);
}

/**
 * @brief Encode a camera_tracking_geo_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param camera_tracking_geo_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_tracking_geo_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_camera_tracking_geo_status_t* camera_tracking_geo_status)
{
    return mavlink_msg_camera_tracking_geo_status_pack_status(system_id, component_id, _status, msg,  camera_tracking_geo_status->tracking_status, camera_tracking_geo_status->lat, camera_tracking_geo_status->lon, camera_tracking_geo_status->alt, camera_tracking_geo_status->h_acc, camera_tracking_geo_status->v_acc, camera_tracking_geo_status->vel_n, camera_tracking_geo_status->vel_e, camera_tracking_geo_status->vel_d, camera_tracking_geo_status->vel_acc, camera_tracking_geo_status->dist, camera_tracking_geo_status->hdg, camera_tracking_geo_status->hdg_acc);
}

/**
 * @brief Send a camera_tracking_geo_status message
 * @param chan MAVLink channel to send the message
 *
 * @param tracking_status  Current tracking status
 * @param lat [degE7] Latitude of tracked object
 * @param lon [degE7] Longitude of tracked object
 * @param alt [m] Altitude of tracked object(AMSL, WGS84)
 * @param h_acc [m] Horizontal accuracy. NAN if unknown
 * @param v_acc [m] Vertical accuracy. NAN if unknown
 * @param vel_n [m/s] North velocity of tracked object. NAN if unknown
 * @param vel_e [m/s] East velocity of tracked object. NAN if unknown
 * @param vel_d [m/s] Down velocity of tracked object. NAN if unknown
 * @param vel_acc [m/s] Velocity accuracy. NAN if unknown
 * @param dist [m] Distance between camera and tracked object. NAN if unknown
 * @param hdg [rad] Heading in radians, in NED. NAN if unknown
 * @param hdg_acc [rad] Accuracy of heading, in NED. NAN if unknown
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_tracking_geo_status_send(mavlink_channel_t chan, uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_float(buf, 8, alt);
    _mav_put_float(buf, 12, h_acc);
    _mav_put_float(buf, 16, v_acc);
    _mav_put_float(buf, 20, vel_n);
    _mav_put_float(buf, 24, vel_e);
    _mav_put_float(buf, 28, vel_d);
    _mav_put_float(buf, 32, vel_acc);
    _mav_put_float(buf, 36, dist);
    _mav_put_float(buf, 40, hdg);
    _mav_put_float(buf, 44, hdg_acc);
    _mav_put_uint8_t(buf, 48, tracking_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS, buf, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC);
#else
    mavlink_camera_tracking_geo_status_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_n = vel_n;
    packet.vel_e = vel_e;
    packet.vel_d = vel_d;
    packet.vel_acc = vel_acc;
    packet.dist = dist;
    packet.hdg = hdg;
    packet.hdg_acc = hdg_acc;
    packet.tracking_status = tracking_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC);
#endif
}

/**
 * @brief Send a camera_tracking_geo_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_camera_tracking_geo_status_send_struct(mavlink_channel_t chan, const mavlink_camera_tracking_geo_status_t* camera_tracking_geo_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_camera_tracking_geo_status_send(chan, camera_tracking_geo_status->tracking_status, camera_tracking_geo_status->lat, camera_tracking_geo_status->lon, camera_tracking_geo_status->alt, camera_tracking_geo_status->h_acc, camera_tracking_geo_status->v_acc, camera_tracking_geo_status->vel_n, camera_tracking_geo_status->vel_e, camera_tracking_geo_status->vel_d, camera_tracking_geo_status->vel_acc, camera_tracking_geo_status->dist, camera_tracking_geo_status->hdg, camera_tracking_geo_status->hdg_acc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS, (const char *)camera_tracking_geo_status, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_tracking_geo_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t tracking_status, int32_t lat, int32_t lon, float alt, float h_acc, float v_acc, float vel_n, float vel_e, float vel_d, float vel_acc, float dist, float hdg, float hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_float(buf, 8, alt);
    _mav_put_float(buf, 12, h_acc);
    _mav_put_float(buf, 16, v_acc);
    _mav_put_float(buf, 20, vel_n);
    _mav_put_float(buf, 24, vel_e);
    _mav_put_float(buf, 28, vel_d);
    _mav_put_float(buf, 32, vel_acc);
    _mav_put_float(buf, 36, dist);
    _mav_put_float(buf, 40, hdg);
    _mav_put_float(buf, 44, hdg_acc);
    _mav_put_uint8_t(buf, 48, tracking_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS, buf, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC);
#else
    mavlink_camera_tracking_geo_status_t *packet = (mavlink_camera_tracking_geo_status_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->h_acc = h_acc;
    packet->v_acc = v_acc;
    packet->vel_n = vel_n;
    packet->vel_e = vel_e;
    packet->vel_d = vel_d;
    packet->vel_acc = vel_acc;
    packet->dist = dist;
    packet->hdg = hdg;
    packet->hdg_acc = hdg_acc;
    packet->tracking_status = tracking_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS, (const char *)packet, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CAMERA_TRACKING_GEO_STATUS UNPACKING


/**
 * @brief Get field tracking_status from camera_tracking_geo_status message
 *
 * @return  Current tracking status
 */
static inline uint8_t mavlink_msg_camera_tracking_geo_status_get_tracking_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field lat from camera_tracking_geo_status message
 *
 * @return [degE7] Latitude of tracked object
 */
static inline int32_t mavlink_msg_camera_tracking_geo_status_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from camera_tracking_geo_status message
 *
 * @return [degE7] Longitude of tracked object
 */
static inline int32_t mavlink_msg_camera_tracking_geo_status_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from camera_tracking_geo_status message
 *
 * @return [m] Altitude of tracked object(AMSL, WGS84)
 */
static inline float mavlink_msg_camera_tracking_geo_status_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field h_acc from camera_tracking_geo_status message
 *
 * @return [m] Horizontal accuracy. NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_geo_status_get_h_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field v_acc from camera_tracking_geo_status message
 *
 * @return [m] Vertical accuracy. NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_geo_status_get_v_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_n from camera_tracking_geo_status message
 *
 * @return [m/s] North velocity of tracked object. NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_geo_status_get_vel_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vel_e from camera_tracking_geo_status message
 *
 * @return [m/s] East velocity of tracked object. NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_geo_status_get_vel_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vel_d from camera_tracking_geo_status message
 *
 * @return [m/s] Down velocity of tracked object. NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_geo_status_get_vel_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vel_acc from camera_tracking_geo_status message
 *
 * @return [m/s] Velocity accuracy. NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_geo_status_get_vel_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field dist from camera_tracking_geo_status message
 *
 * @return [m] Distance between camera and tracked object. NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_geo_status_get_dist(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field hdg from camera_tracking_geo_status message
 *
 * @return [rad] Heading in radians, in NED. NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_geo_status_get_hdg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field hdg_acc from camera_tracking_geo_status message
 *
 * @return [rad] Accuracy of heading, in NED. NAN if unknown
 */
static inline float mavlink_msg_camera_tracking_geo_status_get_hdg_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a camera_tracking_geo_status message into a struct
 *
 * @param msg The message to decode
 * @param camera_tracking_geo_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_tracking_geo_status_decode(const mavlink_message_t* msg, mavlink_camera_tracking_geo_status_t* camera_tracking_geo_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    camera_tracking_geo_status->lat = mavlink_msg_camera_tracking_geo_status_get_lat(msg);
    camera_tracking_geo_status->lon = mavlink_msg_camera_tracking_geo_status_get_lon(msg);
    camera_tracking_geo_status->alt = mavlink_msg_camera_tracking_geo_status_get_alt(msg);
    camera_tracking_geo_status->h_acc = mavlink_msg_camera_tracking_geo_status_get_h_acc(msg);
    camera_tracking_geo_status->v_acc = mavlink_msg_camera_tracking_geo_status_get_v_acc(msg);
    camera_tracking_geo_status->vel_n = mavlink_msg_camera_tracking_geo_status_get_vel_n(msg);
    camera_tracking_geo_status->vel_e = mavlink_msg_camera_tracking_geo_status_get_vel_e(msg);
    camera_tracking_geo_status->vel_d = mavlink_msg_camera_tracking_geo_status_get_vel_d(msg);
    camera_tracking_geo_status->vel_acc = mavlink_msg_camera_tracking_geo_status_get_vel_acc(msg);
    camera_tracking_geo_status->dist = mavlink_msg_camera_tracking_geo_status_get_dist(msg);
    camera_tracking_geo_status->hdg = mavlink_msg_camera_tracking_geo_status_get_hdg(msg);
    camera_tracking_geo_status->hdg_acc = mavlink_msg_camera_tracking_geo_status_get_hdg_acc(msg);
    camera_tracking_geo_status->tracking_status = mavlink_msg_camera_tracking_geo_status_get_tracking_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN;
        memset(camera_tracking_geo_status, 0, MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS_LEN);
    memcpy(camera_tracking_geo_status, _MAV_PAYLOAD(msg), len);
#endif
}
