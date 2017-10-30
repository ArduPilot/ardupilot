#pragma once
// MESSAGE HIL_GPS PACKING

#define MAVLINK_MSG_ID_HIL_GPS 113

MAVPACKED(
typedef struct __mavlink_hil_gps_t {
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 int32_t lat; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t lon; /*< Longitude (WGS84), in degrees * 1E7*/
 int32_t alt; /*< Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)*/
 uint16_t eph; /*< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535*/
 uint16_t epv; /*< GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535*/
 uint16_t vel; /*< GPS ground speed in cm/s. If unknown, set to: 65535*/
 int16_t vn; /*< GPS velocity in cm/s in NORTH direction in earth-fixed NED frame*/
 int16_t ve; /*< GPS velocity in cm/s in EAST direction in earth-fixed NED frame*/
 int16_t vd; /*< GPS velocity in cm/s in DOWN direction in earth-fixed NED frame*/
 uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535*/
 uint8_t fix_type; /*< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
 uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/
}) mavlink_hil_gps_t;

#define MAVLINK_MSG_ID_HIL_GPS_LEN 36
#define MAVLINK_MSG_ID_HIL_GPS_MIN_LEN 36
#define MAVLINK_MSG_ID_113_LEN 36
#define MAVLINK_MSG_ID_113_MIN_LEN 36

#define MAVLINK_MSG_ID_HIL_GPS_CRC 124
#define MAVLINK_MSG_ID_113_CRC 124



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HIL_GPS { \
    113, \
    "HIL_GPS", \
    13, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_gps_t, time_usec) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_hil_gps_t, fix_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_hil_gps_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_hil_gps_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_hil_gps_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_hil_gps_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_hil_gps_t, epv) }, \
         { "vel", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_hil_gps_t, vel) }, \
         { "vn", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_hil_gps_t, vn) }, \
         { "ve", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_hil_gps_t, ve) }, \
         { "vd", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_hil_gps_t, vd) }, \
         { "cog", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_hil_gps_t, cog) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_hil_gps_t, satellites_visible) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HIL_GPS { \
    "HIL_GPS", \
    13, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_gps_t, time_usec) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_hil_gps_t, fix_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_hil_gps_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_hil_gps_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_hil_gps_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_hil_gps_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_hil_gps_t, epv) }, \
         { "vel", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_hil_gps_t, vel) }, \
         { "vn", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_hil_gps_t, vn) }, \
         { "ve", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_hil_gps_t, ve) }, \
         { "vd", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_hil_gps_t, vd) }, \
         { "cog", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_hil_gps_t, cog) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_hil_gps_t, satellites_visible) }, \
         } \
}
#endif

/**
 * @brief Pack a hil_gps message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed in cm/s. If unknown, set to: 65535
 * @param vn GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_gps_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_GPS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_uint16_t(buf, 20, eph);
    _mav_put_uint16_t(buf, 22, epv);
    _mav_put_uint16_t(buf, 24, vel);
    _mav_put_int16_t(buf, 26, vn);
    _mav_put_int16_t(buf, 28, ve);
    _mav_put_int16_t(buf, 30, vd);
    _mav_put_uint16_t(buf, 32, cog);
    _mav_put_uint8_t(buf, 34, fix_type);
    _mav_put_uint8_t(buf, 35, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_GPS_LEN);
#else
    mavlink_hil_gps_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_GPS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_GPS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_GPS_MIN_LEN, MAVLINK_MSG_ID_HIL_GPS_LEN, MAVLINK_MSG_ID_HIL_GPS_CRC);
}

/**
 * @brief Pack a hil_gps message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed in cm/s. If unknown, set to: 65535
 * @param vn GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_gps_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,int16_t vn,int16_t ve,int16_t vd,uint16_t cog,uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_GPS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_uint16_t(buf, 20, eph);
    _mav_put_uint16_t(buf, 22, epv);
    _mav_put_uint16_t(buf, 24, vel);
    _mav_put_int16_t(buf, 26, vn);
    _mav_put_int16_t(buf, 28, ve);
    _mav_put_int16_t(buf, 30, vd);
    _mav_put_uint16_t(buf, 32, cog);
    _mav_put_uint8_t(buf, 34, fix_type);
    _mav_put_uint8_t(buf, 35, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_GPS_LEN);
#else
    mavlink_hil_gps_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_GPS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_GPS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_GPS_MIN_LEN, MAVLINK_MSG_ID_HIL_GPS_LEN, MAVLINK_MSG_ID_HIL_GPS_CRC);
}

/**
 * @brief Encode a hil_gps struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_gps C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_gps_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_gps_t* hil_gps)
{
    return mavlink_msg_hil_gps_pack(system_id, component_id, msg, hil_gps->time_usec, hil_gps->fix_type, hil_gps->lat, hil_gps->lon, hil_gps->alt, hil_gps->eph, hil_gps->epv, hil_gps->vel, hil_gps->vn, hil_gps->ve, hil_gps->vd, hil_gps->cog, hil_gps->satellites_visible);
}

/**
 * @brief Encode a hil_gps struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hil_gps C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_gps_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hil_gps_t* hil_gps)
{
    return mavlink_msg_hil_gps_pack_chan(system_id, component_id, chan, msg, hil_gps->time_usec, hil_gps->fix_type, hil_gps->lat, hil_gps->lon, hil_gps->alt, hil_gps->eph, hil_gps->epv, hil_gps->vel, hil_gps->vn, hil_gps->ve, hil_gps->vd, hil_gps->cog, hil_gps->satellites_visible);
}

/**
 * @brief Send a hil_gps message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed in cm/s. If unknown, set to: 65535
 * @param vn GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_gps_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HIL_GPS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_uint16_t(buf, 20, eph);
    _mav_put_uint16_t(buf, 22, epv);
    _mav_put_uint16_t(buf, 24, vel);
    _mav_put_int16_t(buf, 26, vn);
    _mav_put_int16_t(buf, 28, ve);
    _mav_put_int16_t(buf, 30, vd);
    _mav_put_uint16_t(buf, 32, cog);
    _mav_put_uint8_t(buf, 34, fix_type);
    _mav_put_uint8_t(buf, 35, satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_GPS, buf, MAVLINK_MSG_ID_HIL_GPS_MIN_LEN, MAVLINK_MSG_ID_HIL_GPS_LEN, MAVLINK_MSG_ID_HIL_GPS_CRC);
#else
    mavlink_hil_gps_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_GPS, (const char *)&packet, MAVLINK_MSG_ID_HIL_GPS_MIN_LEN, MAVLINK_MSG_ID_HIL_GPS_LEN, MAVLINK_MSG_ID_HIL_GPS_CRC);
#endif
}

/**
 * @brief Send a hil_gps message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hil_gps_send_struct(mavlink_channel_t chan, const mavlink_hil_gps_t* hil_gps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hil_gps_send(chan, hil_gps->time_usec, hil_gps->fix_type, hil_gps->lat, hil_gps->lon, hil_gps->alt, hil_gps->eph, hil_gps->epv, hil_gps->vel, hil_gps->vn, hil_gps->ve, hil_gps->vd, hil_gps->cog, hil_gps->satellites_visible);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_GPS, (const char *)hil_gps, MAVLINK_MSG_ID_HIL_GPS_MIN_LEN, MAVLINK_MSG_ID_HIL_GPS_LEN, MAVLINK_MSG_ID_HIL_GPS_CRC);
#endif
}

#if MAVLINK_MSG_ID_HIL_GPS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hil_gps_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_uint16_t(buf, 20, eph);
    _mav_put_uint16_t(buf, 22, epv);
    _mav_put_uint16_t(buf, 24, vel);
    _mav_put_int16_t(buf, 26, vn);
    _mav_put_int16_t(buf, 28, ve);
    _mav_put_int16_t(buf, 30, vd);
    _mav_put_uint16_t(buf, 32, cog);
    _mav_put_uint8_t(buf, 34, fix_type);
    _mav_put_uint8_t(buf, 35, satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_GPS, buf, MAVLINK_MSG_ID_HIL_GPS_MIN_LEN, MAVLINK_MSG_ID_HIL_GPS_LEN, MAVLINK_MSG_ID_HIL_GPS_CRC);
#else
    mavlink_hil_gps_t *packet = (mavlink_hil_gps_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->eph = eph;
    packet->epv = epv;
    packet->vel = vel;
    packet->vn = vn;
    packet->ve = ve;
    packet->vd = vd;
    packet->cog = cog;
    packet->fix_type = fix_type;
    packet->satellites_visible = satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_GPS, (const char *)packet, MAVLINK_MSG_ID_HIL_GPS_MIN_LEN, MAVLINK_MSG_ID_HIL_GPS_LEN, MAVLINK_MSG_ID_HIL_GPS_CRC);
#endif
}
#endif

#endif

// MESSAGE HIL_GPS UNPACKING


/**
 * @brief Get field time_usec from hil_gps message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_hil_gps_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field fix_type from hil_gps message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 */
static inline uint8_t mavlink_msg_hil_gps_get_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field lat from hil_gps message
 *
 * @return Latitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_hil_gps_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from hil_gps message
 *
 * @return Longitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_hil_gps_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from hil_gps message
 *
 * @return Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
 */
static inline int32_t mavlink_msg_hil_gps_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field eph from hil_gps message
 *
 * @return GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_hil_gps_get_eph(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field epv from hil_gps message
 *
 * @return GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_hil_gps_get_epv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field vel from hil_gps message
 *
 * @return GPS ground speed in cm/s. If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_hil_gps_get_vel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field vn from hil_gps message
 *
 * @return GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
 */
static inline int16_t mavlink_msg_hil_gps_get_vn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field ve from hil_gps message
 *
 * @return GPS velocity in cm/s in EAST direction in earth-fixed NED frame
 */
static inline int16_t mavlink_msg_hil_gps_get_ve(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field vd from hil_gps message
 *
 * @return GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
 */
static inline int16_t mavlink_msg_hil_gps_get_vd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field cog from hil_gps message
 *
 * @return Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_hil_gps_get_cog(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field satellites_visible from hil_gps message
 *
 * @return Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_hil_gps_get_satellites_visible(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Decode a hil_gps message into a struct
 *
 * @param msg The message to decode
 * @param hil_gps C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_gps_decode(const mavlink_message_t* msg, mavlink_hil_gps_t* hil_gps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hil_gps->time_usec = mavlink_msg_hil_gps_get_time_usec(msg);
    hil_gps->lat = mavlink_msg_hil_gps_get_lat(msg);
    hil_gps->lon = mavlink_msg_hil_gps_get_lon(msg);
    hil_gps->alt = mavlink_msg_hil_gps_get_alt(msg);
    hil_gps->eph = mavlink_msg_hil_gps_get_eph(msg);
    hil_gps->epv = mavlink_msg_hil_gps_get_epv(msg);
    hil_gps->vel = mavlink_msg_hil_gps_get_vel(msg);
    hil_gps->vn = mavlink_msg_hil_gps_get_vn(msg);
    hil_gps->ve = mavlink_msg_hil_gps_get_ve(msg);
    hil_gps->vd = mavlink_msg_hil_gps_get_vd(msg);
    hil_gps->cog = mavlink_msg_hil_gps_get_cog(msg);
    hil_gps->fix_type = mavlink_msg_hil_gps_get_fix_type(msg);
    hil_gps->satellites_visible = mavlink_msg_hil_gps_get_satellites_visible(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HIL_GPS_LEN? msg->len : MAVLINK_MSG_ID_HIL_GPS_LEN;
        memset(hil_gps, 0, MAVLINK_MSG_ID_HIL_GPS_LEN);
    memcpy(hil_gps, _MAV_PAYLOAD(msg), len);
#endif
}
