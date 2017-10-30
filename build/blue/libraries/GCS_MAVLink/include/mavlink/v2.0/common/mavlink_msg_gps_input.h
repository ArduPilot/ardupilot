#pragma once
// MESSAGE GPS_INPUT PACKING

#define MAVLINK_MSG_ID_GPS_INPUT 232

MAVPACKED(
typedef struct __mavlink_gps_input_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 uint32_t time_week_ms; /*< GPS time (milliseconds from start of GPS week)*/
 int32_t lat; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t lon; /*< Longitude (WGS84), in degrees * 1E7*/
 float alt; /*< Altitude (AMSL, not WGS84), in m (positive for up)*/
 float hdop; /*< GPS HDOP horizontal dilution of position in m*/
 float vdop; /*< GPS VDOP vertical dilution of position in m*/
 float vn; /*< GPS velocity in m/s in NORTH direction in earth-fixed NED frame*/
 float ve; /*< GPS velocity in m/s in EAST direction in earth-fixed NED frame*/
 float vd; /*< GPS velocity in m/s in DOWN direction in earth-fixed NED frame*/
 float speed_accuracy; /*< GPS speed accuracy in m/s*/
 float horiz_accuracy; /*< GPS horizontal accuracy in m*/
 float vert_accuracy; /*< GPS vertical accuracy in m*/
 uint16_t ignore_flags; /*< Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.*/
 uint16_t time_week; /*< GPS week number*/
 uint8_t gps_id; /*< ID of the GPS for multiple GPS inputs*/
 uint8_t fix_type; /*< 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK*/
 uint8_t satellites_visible; /*< Number of satellites visible.*/
}) mavlink_gps_input_t;

#define MAVLINK_MSG_ID_GPS_INPUT_LEN 63
#define MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN 63
#define MAVLINK_MSG_ID_232_LEN 63
#define MAVLINK_MSG_ID_232_MIN_LEN 63

#define MAVLINK_MSG_ID_GPS_INPUT_CRC 151
#define MAVLINK_MSG_ID_232_CRC 151



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_INPUT { \
    232, \
    "GPS_INPUT", \
    18, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_input_t, time_usec) }, \
         { "gps_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_gps_input_t, gps_id) }, \
         { "ignore_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_gps_input_t, ignore_flags) }, \
         { "time_week_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gps_input_t, time_week_ms) }, \
         { "time_week", NULL, MAVLINK_TYPE_UINT16_T, 0, 58, offsetof(mavlink_gps_input_t, time_week) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_gps_input_t, fix_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gps_input_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gps_input_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gps_input_t, alt) }, \
         { "hdop", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gps_input_t, hdop) }, \
         { "vdop", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gps_input_t, vdop) }, \
         { "vn", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gps_input_t, vn) }, \
         { "ve", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gps_input_t, ve) }, \
         { "vd", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gps_input_t, vd) }, \
         { "speed_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_gps_input_t, speed_accuracy) }, \
         { "horiz_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_gps_input_t, horiz_accuracy) }, \
         { "vert_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_gps_input_t, vert_accuracy) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_gps_input_t, satellites_visible) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_INPUT { \
    "GPS_INPUT", \
    18, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_input_t, time_usec) }, \
         { "gps_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 60, offsetof(mavlink_gps_input_t, gps_id) }, \
         { "ignore_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_gps_input_t, ignore_flags) }, \
         { "time_week_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gps_input_t, time_week_ms) }, \
         { "time_week", NULL, MAVLINK_TYPE_UINT16_T, 0, 58, offsetof(mavlink_gps_input_t, time_week) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_gps_input_t, fix_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gps_input_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gps_input_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gps_input_t, alt) }, \
         { "hdop", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gps_input_t, hdop) }, \
         { "vdop", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gps_input_t, vdop) }, \
         { "vn", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gps_input_t, vn) }, \
         { "ve", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gps_input_t, ve) }, \
         { "vd", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gps_input_t, vd) }, \
         { "speed_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_gps_input_t, speed_accuracy) }, \
         { "horiz_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_gps_input_t, horiz_accuracy) }, \
         { "vert_accuracy", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_gps_input_t, vert_accuracy) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_gps_input_t, satellites_visible) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param gps_id ID of the GPS for multiple GPS inputs
 * @param ignore_flags Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.
 * @param time_week_ms GPS time (milliseconds from start of GPS week)
 * @param time_week GPS week number
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in m (positive for up)
 * @param hdop GPS HDOP horizontal dilution of position in m
 * @param vdop GPS VDOP vertical dilution of position in m
 * @param vn GPS velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in m/s in DOWN direction in earth-fixed NED frame
 * @param speed_accuracy GPS speed accuracy in m/s
 * @param horiz_accuracy GPS horizontal accuracy in m
 * @param vert_accuracy GPS vertical accuracy in m
 * @param satellites_visible Number of satellites visible.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_input_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t gps_id, uint16_t ignore_flags, uint32_t time_week_ms, uint16_t time_week, uint8_t fix_type, int32_t lat, int32_t lon, float alt, float hdop, float vdop, float vn, float ve, float vd, float speed_accuracy, float horiz_accuracy, float vert_accuracy, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INPUT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, time_week_ms);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt);
    _mav_put_float(buf, 24, hdop);
    _mav_put_float(buf, 28, vdop);
    _mav_put_float(buf, 32, vn);
    _mav_put_float(buf, 36, ve);
    _mav_put_float(buf, 40, vd);
    _mav_put_float(buf, 44, speed_accuracy);
    _mav_put_float(buf, 48, horiz_accuracy);
    _mav_put_float(buf, 52, vert_accuracy);
    _mav_put_uint16_t(buf, 56, ignore_flags);
    _mav_put_uint16_t(buf, 58, time_week);
    _mav_put_uint8_t(buf, 60, gps_id);
    _mav_put_uint8_t(buf, 61, fix_type);
    _mav_put_uint8_t(buf, 62, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_INPUT_LEN);
#else
    mavlink_gps_input_t packet;
    packet.time_usec = time_usec;
    packet.time_week_ms = time_week_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.speed_accuracy = speed_accuracy;
    packet.horiz_accuracy = horiz_accuracy;
    packet.vert_accuracy = vert_accuracy;
    packet.ignore_flags = ignore_flags;
    packet.time_week = time_week;
    packet.gps_id = gps_id;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_INPUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN, MAVLINK_MSG_ID_GPS_INPUT_LEN, MAVLINK_MSG_ID_GPS_INPUT_CRC);
}

/**
 * @brief Pack a gps_input message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param gps_id ID of the GPS for multiple GPS inputs
 * @param ignore_flags Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.
 * @param time_week_ms GPS time (milliseconds from start of GPS week)
 * @param time_week GPS week number
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in m (positive for up)
 * @param hdop GPS HDOP horizontal dilution of position in m
 * @param vdop GPS VDOP vertical dilution of position in m
 * @param vn GPS velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in m/s in DOWN direction in earth-fixed NED frame
 * @param speed_accuracy GPS speed accuracy in m/s
 * @param horiz_accuracy GPS horizontal accuracy in m
 * @param vert_accuracy GPS vertical accuracy in m
 * @param satellites_visible Number of satellites visible.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_input_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t gps_id,uint16_t ignore_flags,uint32_t time_week_ms,uint16_t time_week,uint8_t fix_type,int32_t lat,int32_t lon,float alt,float hdop,float vdop,float vn,float ve,float vd,float speed_accuracy,float horiz_accuracy,float vert_accuracy,uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INPUT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, time_week_ms);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt);
    _mav_put_float(buf, 24, hdop);
    _mav_put_float(buf, 28, vdop);
    _mav_put_float(buf, 32, vn);
    _mav_put_float(buf, 36, ve);
    _mav_put_float(buf, 40, vd);
    _mav_put_float(buf, 44, speed_accuracy);
    _mav_put_float(buf, 48, horiz_accuracy);
    _mav_put_float(buf, 52, vert_accuracy);
    _mav_put_uint16_t(buf, 56, ignore_flags);
    _mav_put_uint16_t(buf, 58, time_week);
    _mav_put_uint8_t(buf, 60, gps_id);
    _mav_put_uint8_t(buf, 61, fix_type);
    _mav_put_uint8_t(buf, 62, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_INPUT_LEN);
#else
    mavlink_gps_input_t packet;
    packet.time_usec = time_usec;
    packet.time_week_ms = time_week_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.speed_accuracy = speed_accuracy;
    packet.horiz_accuracy = horiz_accuracy;
    packet.vert_accuracy = vert_accuracy;
    packet.ignore_flags = ignore_flags;
    packet.time_week = time_week;
    packet.gps_id = gps_id;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_INPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN, MAVLINK_MSG_ID_GPS_INPUT_LEN, MAVLINK_MSG_ID_GPS_INPUT_CRC);
}

/**
 * @brief Encode a gps_input struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_input_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_input_t* gps_input)
{
    return mavlink_msg_gps_input_pack(system_id, component_id, msg, gps_input->time_usec, gps_input->gps_id, gps_input->ignore_flags, gps_input->time_week_ms, gps_input->time_week, gps_input->fix_type, gps_input->lat, gps_input->lon, gps_input->alt, gps_input->hdop, gps_input->vdop, gps_input->vn, gps_input->ve, gps_input->vd, gps_input->speed_accuracy, gps_input->horiz_accuracy, gps_input->vert_accuracy, gps_input->satellites_visible);
}

/**
 * @brief Encode a gps_input struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_input_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_input_t* gps_input)
{
    return mavlink_msg_gps_input_pack_chan(system_id, component_id, chan, msg, gps_input->time_usec, gps_input->gps_id, gps_input->ignore_flags, gps_input->time_week_ms, gps_input->time_week, gps_input->fix_type, gps_input->lat, gps_input->lon, gps_input->alt, gps_input->hdop, gps_input->vdop, gps_input->vn, gps_input->ve, gps_input->vd, gps_input->speed_accuracy, gps_input->horiz_accuracy, gps_input->vert_accuracy, gps_input->satellites_visible);
}

/**
 * @brief Send a gps_input message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param gps_id ID of the GPS for multiple GPS inputs
 * @param ignore_flags Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.
 * @param time_week_ms GPS time (milliseconds from start of GPS week)
 * @param time_week GPS week number
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, not WGS84), in m (positive for up)
 * @param hdop GPS HDOP horizontal dilution of position in m
 * @param vdop GPS VDOP vertical dilution of position in m
 * @param vn GPS velocity in m/s in NORTH direction in earth-fixed NED frame
 * @param ve GPS velocity in m/s in EAST direction in earth-fixed NED frame
 * @param vd GPS velocity in m/s in DOWN direction in earth-fixed NED frame
 * @param speed_accuracy GPS speed accuracy in m/s
 * @param horiz_accuracy GPS horizontal accuracy in m
 * @param vert_accuracy GPS vertical accuracy in m
 * @param satellites_visible Number of satellites visible.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_input_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t gps_id, uint16_t ignore_flags, uint32_t time_week_ms, uint16_t time_week, uint8_t fix_type, int32_t lat, int32_t lon, float alt, float hdop, float vdop, float vn, float ve, float vd, float speed_accuracy, float horiz_accuracy, float vert_accuracy, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_INPUT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, time_week_ms);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt);
    _mav_put_float(buf, 24, hdop);
    _mav_put_float(buf, 28, vdop);
    _mav_put_float(buf, 32, vn);
    _mav_put_float(buf, 36, ve);
    _mav_put_float(buf, 40, vd);
    _mav_put_float(buf, 44, speed_accuracy);
    _mav_put_float(buf, 48, horiz_accuracy);
    _mav_put_float(buf, 52, vert_accuracy);
    _mav_put_uint16_t(buf, 56, ignore_flags);
    _mav_put_uint16_t(buf, 58, time_week);
    _mav_put_uint8_t(buf, 60, gps_id);
    _mav_put_uint8_t(buf, 61, fix_type);
    _mav_put_uint8_t(buf, 62, satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INPUT, buf, MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN, MAVLINK_MSG_ID_GPS_INPUT_LEN, MAVLINK_MSG_ID_GPS_INPUT_CRC);
#else
    mavlink_gps_input_t packet;
    packet.time_usec = time_usec;
    packet.time_week_ms = time_week_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.vn = vn;
    packet.ve = ve;
    packet.vd = vd;
    packet.speed_accuracy = speed_accuracy;
    packet.horiz_accuracy = horiz_accuracy;
    packet.vert_accuracy = vert_accuracy;
    packet.ignore_flags = ignore_flags;
    packet.time_week = time_week;
    packet.gps_id = gps_id;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INPUT, (const char *)&packet, MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN, MAVLINK_MSG_ID_GPS_INPUT_LEN, MAVLINK_MSG_ID_GPS_INPUT_CRC);
#endif
}

/**
 * @brief Send a gps_input message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_input_send_struct(mavlink_channel_t chan, const mavlink_gps_input_t* gps_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_input_send(chan, gps_input->time_usec, gps_input->gps_id, gps_input->ignore_flags, gps_input->time_week_ms, gps_input->time_week, gps_input->fix_type, gps_input->lat, gps_input->lon, gps_input->alt, gps_input->hdop, gps_input->vdop, gps_input->vn, gps_input->ve, gps_input->vd, gps_input->speed_accuracy, gps_input->horiz_accuracy, gps_input->vert_accuracy, gps_input->satellites_visible);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INPUT, (const char *)gps_input, MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN, MAVLINK_MSG_ID_GPS_INPUT_LEN, MAVLINK_MSG_ID_GPS_INPUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_INPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_input_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t gps_id, uint16_t ignore_flags, uint32_t time_week_ms, uint16_t time_week, uint8_t fix_type, int32_t lat, int32_t lon, float alt, float hdop, float vdop, float vn, float ve, float vd, float speed_accuracy, float horiz_accuracy, float vert_accuracy, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, time_week_ms);
    _mav_put_int32_t(buf, 12, lat);
    _mav_put_int32_t(buf, 16, lon);
    _mav_put_float(buf, 20, alt);
    _mav_put_float(buf, 24, hdop);
    _mav_put_float(buf, 28, vdop);
    _mav_put_float(buf, 32, vn);
    _mav_put_float(buf, 36, ve);
    _mav_put_float(buf, 40, vd);
    _mav_put_float(buf, 44, speed_accuracy);
    _mav_put_float(buf, 48, horiz_accuracy);
    _mav_put_float(buf, 52, vert_accuracy);
    _mav_put_uint16_t(buf, 56, ignore_flags);
    _mav_put_uint16_t(buf, 58, time_week);
    _mav_put_uint8_t(buf, 60, gps_id);
    _mav_put_uint8_t(buf, 61, fix_type);
    _mav_put_uint8_t(buf, 62, satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INPUT, buf, MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN, MAVLINK_MSG_ID_GPS_INPUT_LEN, MAVLINK_MSG_ID_GPS_INPUT_CRC);
#else
    mavlink_gps_input_t *packet = (mavlink_gps_input_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->time_week_ms = time_week_ms;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->hdop = hdop;
    packet->vdop = vdop;
    packet->vn = vn;
    packet->ve = ve;
    packet->vd = vd;
    packet->speed_accuracy = speed_accuracy;
    packet->horiz_accuracy = horiz_accuracy;
    packet->vert_accuracy = vert_accuracy;
    packet->ignore_flags = ignore_flags;
    packet->time_week = time_week;
    packet->gps_id = gps_id;
    packet->fix_type = fix_type;
    packet->satellites_visible = satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_INPUT, (const char *)packet, MAVLINK_MSG_ID_GPS_INPUT_MIN_LEN, MAVLINK_MSG_ID_GPS_INPUT_LEN, MAVLINK_MSG_ID_GPS_INPUT_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_INPUT UNPACKING


/**
 * @brief Get field time_usec from gps_input message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_gps_input_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field gps_id from gps_input message
 *
 * @return ID of the GPS for multiple GPS inputs
 */
static inline uint8_t mavlink_msg_gps_input_get_gps_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  60);
}

/**
 * @brief Get field ignore_flags from gps_input message
 *
 * @return Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.
 */
static inline uint16_t mavlink_msg_gps_input_get_ignore_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  56);
}

/**
 * @brief Get field time_week_ms from gps_input message
 *
 * @return GPS time (milliseconds from start of GPS week)
 */
static inline uint32_t mavlink_msg_gps_input_get_time_week_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field time_week from gps_input message
 *
 * @return GPS week number
 */
static inline uint16_t mavlink_msg_gps_input_get_time_week(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  58);
}

/**
 * @brief Get field fix_type from gps_input message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
 */
static inline uint8_t mavlink_msg_gps_input_get_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  61);
}

/**
 * @brief Get field lat from gps_input message
 *
 * @return Latitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_gps_input_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field lon from gps_input message
 *
 * @return Longitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_gps_input_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field alt from gps_input message
 *
 * @return Altitude (AMSL, not WGS84), in m (positive for up)
 */
static inline float mavlink_msg_gps_input_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field hdop from gps_input message
 *
 * @return GPS HDOP horizontal dilution of position in m
 */
static inline float mavlink_msg_gps_input_get_hdop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vdop from gps_input message
 *
 * @return GPS VDOP vertical dilution of position in m
 */
static inline float mavlink_msg_gps_input_get_vdop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vn from gps_input message
 *
 * @return GPS velocity in m/s in NORTH direction in earth-fixed NED frame
 */
static inline float mavlink_msg_gps_input_get_vn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field ve from gps_input message
 *
 * @return GPS velocity in m/s in EAST direction in earth-fixed NED frame
 */
static inline float mavlink_msg_gps_input_get_ve(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field vd from gps_input message
 *
 * @return GPS velocity in m/s in DOWN direction in earth-fixed NED frame
 */
static inline float mavlink_msg_gps_input_get_vd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field speed_accuracy from gps_input message
 *
 * @return GPS speed accuracy in m/s
 */
static inline float mavlink_msg_gps_input_get_speed_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field horiz_accuracy from gps_input message
 *
 * @return GPS horizontal accuracy in m
 */
static inline float mavlink_msg_gps_input_get_horiz_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field vert_accuracy from gps_input message
 *
 * @return GPS vertical accuracy in m
 */
static inline float mavlink_msg_gps_input_get_vert_accuracy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field satellites_visible from gps_input message
 *
 * @return Number of satellites visible.
 */
static inline uint8_t mavlink_msg_gps_input_get_satellites_visible(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  62);
}

/**
 * @brief Decode a gps_input message into a struct
 *
 * @param msg The message to decode
 * @param gps_input C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_input_decode(const mavlink_message_t* msg, mavlink_gps_input_t* gps_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps_input->time_usec = mavlink_msg_gps_input_get_time_usec(msg);
    gps_input->time_week_ms = mavlink_msg_gps_input_get_time_week_ms(msg);
    gps_input->lat = mavlink_msg_gps_input_get_lat(msg);
    gps_input->lon = mavlink_msg_gps_input_get_lon(msg);
    gps_input->alt = mavlink_msg_gps_input_get_alt(msg);
    gps_input->hdop = mavlink_msg_gps_input_get_hdop(msg);
    gps_input->vdop = mavlink_msg_gps_input_get_vdop(msg);
    gps_input->vn = mavlink_msg_gps_input_get_vn(msg);
    gps_input->ve = mavlink_msg_gps_input_get_ve(msg);
    gps_input->vd = mavlink_msg_gps_input_get_vd(msg);
    gps_input->speed_accuracy = mavlink_msg_gps_input_get_speed_accuracy(msg);
    gps_input->horiz_accuracy = mavlink_msg_gps_input_get_horiz_accuracy(msg);
    gps_input->vert_accuracy = mavlink_msg_gps_input_get_vert_accuracy(msg);
    gps_input->ignore_flags = mavlink_msg_gps_input_get_ignore_flags(msg);
    gps_input->time_week = mavlink_msg_gps_input_get_time_week(msg);
    gps_input->gps_id = mavlink_msg_gps_input_get_gps_id(msg);
    gps_input->fix_type = mavlink_msg_gps_input_get_fix_type(msg);
    gps_input->satellites_visible = mavlink_msg_gps_input_get_satellites_visible(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_INPUT_LEN? msg->len : MAVLINK_MSG_ID_GPS_INPUT_LEN;
        memset(gps_input, 0, MAVLINK_MSG_ID_GPS_INPUT_LEN);
    memcpy(gps_input, _MAV_PAYLOAD(msg), len);
#endif
}
