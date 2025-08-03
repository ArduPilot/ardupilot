#pragma once
// MESSAGE GPS2_RAW PACKING

#define MAVLINK_MSG_ID_GPS2_RAW 124

MAVPACKED(
typedef struct __mavlink_gps2_raw_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 int32_t lat; /*< [degE7] Latitude (WGS84)*/
 int32_t lon; /*< [degE7] Longitude (WGS84)*/
 int32_t alt; /*< [mm] Altitude (MSL). Positive for up.*/
 uint32_t dgps_age; /*< [ms] Age of DGPS info*/
 uint16_t eph; /*<  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
 uint16_t epv; /*<  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
 uint16_t vel; /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
 uint16_t cog; /*< [cdeg] Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint8_t fix_type; /*<  GPS fix type.*/
 uint8_t satellites_visible; /*<  Number of satellites visible. If unknown, set to 255*/
 uint8_t dgps_numch; /*<  Number of DGPS satellites*/
 uint16_t yaw; /*< [cdeg] Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.*/
 int32_t alt_ellipsoid; /*< [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.*/
 uint32_t h_acc; /*< [mm] Position uncertainty.*/
 uint32_t v_acc; /*< [mm] Altitude uncertainty.*/
 uint32_t vel_acc; /*< [mm/s] Speed uncertainty.*/
 uint32_t hdg_acc; /*< [degE5] Heading / track uncertainty*/
}) mavlink_gps2_raw_t;

#define MAVLINK_MSG_ID_GPS2_RAW_LEN 57
#define MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN 35
#define MAVLINK_MSG_ID_124_LEN 57
#define MAVLINK_MSG_ID_124_MIN_LEN 35

#define MAVLINK_MSG_ID_GPS2_RAW_CRC 87
#define MAVLINK_MSG_ID_124_CRC 87



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS2_RAW { \
    124, \
    "GPS2_RAW", \
    18, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps2_raw_t, time_usec) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_gps2_raw_t, fix_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gps2_raw_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gps2_raw_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gps2_raw_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_gps2_raw_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_gps2_raw_t, epv) }, \
         { "vel", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_gps2_raw_t, vel) }, \
         { "cog", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_gps2_raw_t, cog) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_gps2_raw_t, satellites_visible) }, \
         { "dgps_numch", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_gps2_raw_t, dgps_numch) }, \
         { "dgps_age", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_gps2_raw_t, dgps_age) }, \
         { "yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 35, offsetof(mavlink_gps2_raw_t, yaw) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_INT32_T, 0, 37, offsetof(mavlink_gps2_raw_t, alt_ellipsoid) }, \
         { "h_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 41, offsetof(mavlink_gps2_raw_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 45, offsetof(mavlink_gps2_raw_t, v_acc) }, \
         { "vel_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 49, offsetof(mavlink_gps2_raw_t, vel_acc) }, \
         { "hdg_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 53, offsetof(mavlink_gps2_raw_t, hdg_acc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS2_RAW { \
    "GPS2_RAW", \
    18, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps2_raw_t, time_usec) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_gps2_raw_t, fix_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gps2_raw_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gps2_raw_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gps2_raw_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_gps2_raw_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_gps2_raw_t, epv) }, \
         { "vel", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_gps2_raw_t, vel) }, \
         { "cog", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_gps2_raw_t, cog) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_gps2_raw_t, satellites_visible) }, \
         { "dgps_numch", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_gps2_raw_t, dgps_numch) }, \
         { "dgps_age", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_gps2_raw_t, dgps_age) }, \
         { "yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 35, offsetof(mavlink_gps2_raw_t, yaw) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_INT32_T, 0, 37, offsetof(mavlink_gps2_raw_t, alt_ellipsoid) }, \
         { "h_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 41, offsetof(mavlink_gps2_raw_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 45, offsetof(mavlink_gps2_raw_t, v_acc) }, \
         { "vel_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 49, offsetof(mavlink_gps2_raw_t, vel_acc) }, \
         { "hdg_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 53, offsetof(mavlink_gps2_raw_t, hdg_acc) }, \
         } \
}
#endif

/**
 * @brief Pack a gps2_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param fix_type  GPS fix type.
 * @param lat [degE7] Latitude (WGS84)
 * @param lon [degE7] Longitude (WGS84)
 * @param alt [mm] Altitude (MSL). Positive for up.
 * @param eph  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param epv  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param vel [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 * @param cog [cdeg] Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible  Number of satellites visible. If unknown, set to 255
 * @param dgps_numch  Number of DGPS satellites
 * @param dgps_age [ms] Age of DGPS info
 * @param yaw [cdeg] Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.
 * @param alt_ellipsoid [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
 * @param h_acc [mm] Position uncertainty.
 * @param v_acc [mm] Altitude uncertainty.
 * @param vel_acc [mm/s] Speed uncertainty.
 * @param hdg_acc [degE5] Heading / track uncertainty
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps2_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS2_RAW_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_uint32_t(buf, 20, dgps_age);
    _mav_put_uint16_t(buf, 24, eph);
    _mav_put_uint16_t(buf, 26, epv);
    _mav_put_uint16_t(buf, 28, vel);
    _mav_put_uint16_t(buf, 30, cog);
    _mav_put_uint8_t(buf, 32, fix_type);
    _mav_put_uint8_t(buf, 33, satellites_visible);
    _mav_put_uint8_t(buf, 34, dgps_numch);
    _mav_put_uint16_t(buf, 35, yaw);
    _mav_put_int32_t(buf, 37, alt_ellipsoid);
    _mav_put_uint32_t(buf, 41, h_acc);
    _mav_put_uint32_t(buf, 45, v_acc);
    _mav_put_uint32_t(buf, 49, vel_acc);
    _mav_put_uint32_t(buf, 53, hdg_acc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS2_RAW_LEN);
#else
    mavlink_gps2_raw_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.dgps_age = dgps_age;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.dgps_numch = dgps_numch;
    packet.yaw = yaw;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_acc = vel_acc;
    packet.hdg_acc = hdg_acc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS2_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS2_RAW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN, MAVLINK_MSG_ID_GPS2_RAW_LEN, MAVLINK_MSG_ID_GPS2_RAW_CRC);
}

/**
 * @brief Pack a gps2_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param fix_type  GPS fix type.
 * @param lat [degE7] Latitude (WGS84)
 * @param lon [degE7] Longitude (WGS84)
 * @param alt [mm] Altitude (MSL). Positive for up.
 * @param eph  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param epv  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param vel [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 * @param cog [cdeg] Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible  Number of satellites visible. If unknown, set to 255
 * @param dgps_numch  Number of DGPS satellites
 * @param dgps_age [ms] Age of DGPS info
 * @param yaw [cdeg] Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.
 * @param alt_ellipsoid [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
 * @param h_acc [mm] Position uncertainty.
 * @param v_acc [mm] Altitude uncertainty.
 * @param vel_acc [mm/s] Speed uncertainty.
 * @param hdg_acc [degE5] Heading / track uncertainty
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps2_raw_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS2_RAW_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_uint32_t(buf, 20, dgps_age);
    _mav_put_uint16_t(buf, 24, eph);
    _mav_put_uint16_t(buf, 26, epv);
    _mav_put_uint16_t(buf, 28, vel);
    _mav_put_uint16_t(buf, 30, cog);
    _mav_put_uint8_t(buf, 32, fix_type);
    _mav_put_uint8_t(buf, 33, satellites_visible);
    _mav_put_uint8_t(buf, 34, dgps_numch);
    _mav_put_uint16_t(buf, 35, yaw);
    _mav_put_int32_t(buf, 37, alt_ellipsoid);
    _mav_put_uint32_t(buf, 41, h_acc);
    _mav_put_uint32_t(buf, 45, v_acc);
    _mav_put_uint32_t(buf, 49, vel_acc);
    _mav_put_uint32_t(buf, 53, hdg_acc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS2_RAW_LEN);
#else
    mavlink_gps2_raw_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.dgps_age = dgps_age;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.dgps_numch = dgps_numch;
    packet.yaw = yaw;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_acc = vel_acc;
    packet.hdg_acc = hdg_acc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS2_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS2_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN, MAVLINK_MSG_ID_GPS2_RAW_LEN, MAVLINK_MSG_ID_GPS2_RAW_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN, MAVLINK_MSG_ID_GPS2_RAW_LEN);
#endif
}

/**
 * @brief Pack a gps2_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param fix_type  GPS fix type.
 * @param lat [degE7] Latitude (WGS84)
 * @param lon [degE7] Longitude (WGS84)
 * @param alt [mm] Altitude (MSL). Positive for up.
 * @param eph  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param epv  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param vel [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 * @param cog [cdeg] Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible  Number of satellites visible. If unknown, set to 255
 * @param dgps_numch  Number of DGPS satellites
 * @param dgps_age [ms] Age of DGPS info
 * @param yaw [cdeg] Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.
 * @param alt_ellipsoid [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
 * @param h_acc [mm] Position uncertainty.
 * @param v_acc [mm] Altitude uncertainty.
 * @param vel_acc [mm/s] Speed uncertainty.
 * @param hdg_acc [degE5] Heading / track uncertainty
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps2_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,uint16_t cog,uint8_t satellites_visible,uint8_t dgps_numch,uint32_t dgps_age,uint16_t yaw,int32_t alt_ellipsoid,uint32_t h_acc,uint32_t v_acc,uint32_t vel_acc,uint32_t hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS2_RAW_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_uint32_t(buf, 20, dgps_age);
    _mav_put_uint16_t(buf, 24, eph);
    _mav_put_uint16_t(buf, 26, epv);
    _mav_put_uint16_t(buf, 28, vel);
    _mav_put_uint16_t(buf, 30, cog);
    _mav_put_uint8_t(buf, 32, fix_type);
    _mav_put_uint8_t(buf, 33, satellites_visible);
    _mav_put_uint8_t(buf, 34, dgps_numch);
    _mav_put_uint16_t(buf, 35, yaw);
    _mav_put_int32_t(buf, 37, alt_ellipsoid);
    _mav_put_uint32_t(buf, 41, h_acc);
    _mav_put_uint32_t(buf, 45, v_acc);
    _mav_put_uint32_t(buf, 49, vel_acc);
    _mav_put_uint32_t(buf, 53, hdg_acc);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS2_RAW_LEN);
#else
    mavlink_gps2_raw_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.dgps_age = dgps_age;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.dgps_numch = dgps_numch;
    packet.yaw = yaw;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_acc = vel_acc;
    packet.hdg_acc = hdg_acc;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS2_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS2_RAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN, MAVLINK_MSG_ID_GPS2_RAW_LEN, MAVLINK_MSG_ID_GPS2_RAW_CRC);
}

/**
 * @brief Encode a gps2_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps2_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps2_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps2_raw_t* gps2_raw)
{
    return mavlink_msg_gps2_raw_pack(system_id, component_id, msg, gps2_raw->time_usec, gps2_raw->fix_type, gps2_raw->lat, gps2_raw->lon, gps2_raw->alt, gps2_raw->eph, gps2_raw->epv, gps2_raw->vel, gps2_raw->cog, gps2_raw->satellites_visible, gps2_raw->dgps_numch, gps2_raw->dgps_age, gps2_raw->yaw, gps2_raw->alt_ellipsoid, gps2_raw->h_acc, gps2_raw->v_acc, gps2_raw->vel_acc, gps2_raw->hdg_acc);
}

/**
 * @brief Encode a gps2_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps2_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps2_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps2_raw_t* gps2_raw)
{
    return mavlink_msg_gps2_raw_pack_chan(system_id, component_id, chan, msg, gps2_raw->time_usec, gps2_raw->fix_type, gps2_raw->lat, gps2_raw->lon, gps2_raw->alt, gps2_raw->eph, gps2_raw->epv, gps2_raw->vel, gps2_raw->cog, gps2_raw->satellites_visible, gps2_raw->dgps_numch, gps2_raw->dgps_age, gps2_raw->yaw, gps2_raw->alt_ellipsoid, gps2_raw->h_acc, gps2_raw->v_acc, gps2_raw->vel_acc, gps2_raw->hdg_acc);
}

/**
 * @brief Encode a gps2_raw struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gps2_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps2_raw_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gps2_raw_t* gps2_raw)
{
    return mavlink_msg_gps2_raw_pack_status(system_id, component_id, _status, msg,  gps2_raw->time_usec, gps2_raw->fix_type, gps2_raw->lat, gps2_raw->lon, gps2_raw->alt, gps2_raw->eph, gps2_raw->epv, gps2_raw->vel, gps2_raw->cog, gps2_raw->satellites_visible, gps2_raw->dgps_numch, gps2_raw->dgps_age, gps2_raw->yaw, gps2_raw->alt_ellipsoid, gps2_raw->h_acc, gps2_raw->v_acc, gps2_raw->vel_acc, gps2_raw->hdg_acc);
}

/**
 * @brief Send a gps2_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param fix_type  GPS fix type.
 * @param lat [degE7] Latitude (WGS84)
 * @param lon [degE7] Longitude (WGS84)
 * @param alt [mm] Altitude (MSL). Positive for up.
 * @param eph  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param epv  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
 * @param vel [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 * @param cog [cdeg] Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible  Number of satellites visible. If unknown, set to 255
 * @param dgps_numch  Number of DGPS satellites
 * @param dgps_age [ms] Age of DGPS info
 * @param yaw [cdeg] Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.
 * @param alt_ellipsoid [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
 * @param h_acc [mm] Position uncertainty.
 * @param v_acc [mm] Altitude uncertainty.
 * @param vel_acc [mm/s] Speed uncertainty.
 * @param hdg_acc [degE5] Heading / track uncertainty
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps2_raw_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS2_RAW_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_uint32_t(buf, 20, dgps_age);
    _mav_put_uint16_t(buf, 24, eph);
    _mav_put_uint16_t(buf, 26, epv);
    _mav_put_uint16_t(buf, 28, vel);
    _mav_put_uint16_t(buf, 30, cog);
    _mav_put_uint8_t(buf, 32, fix_type);
    _mav_put_uint8_t(buf, 33, satellites_visible);
    _mav_put_uint8_t(buf, 34, dgps_numch);
    _mav_put_uint16_t(buf, 35, yaw);
    _mav_put_int32_t(buf, 37, alt_ellipsoid);
    _mav_put_uint32_t(buf, 41, h_acc);
    _mav_put_uint32_t(buf, 45, v_acc);
    _mav_put_uint32_t(buf, 49, vel_acc);
    _mav_put_uint32_t(buf, 53, hdg_acc);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS2_RAW, buf, MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN, MAVLINK_MSG_ID_GPS2_RAW_LEN, MAVLINK_MSG_ID_GPS2_RAW_CRC);
#else
    mavlink_gps2_raw_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.dgps_age = dgps_age;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    packet.dgps_numch = dgps_numch;
    packet.yaw = yaw;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_acc = vel_acc;
    packet.hdg_acc = hdg_acc;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS2_RAW, (const char *)&packet, MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN, MAVLINK_MSG_ID_GPS2_RAW_LEN, MAVLINK_MSG_ID_GPS2_RAW_CRC);
#endif
}

/**
 * @brief Send a gps2_raw message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps2_raw_send_struct(mavlink_channel_t chan, const mavlink_gps2_raw_t* gps2_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps2_raw_send(chan, gps2_raw->time_usec, gps2_raw->fix_type, gps2_raw->lat, gps2_raw->lon, gps2_raw->alt, gps2_raw->eph, gps2_raw->epv, gps2_raw->vel, gps2_raw->cog, gps2_raw->satellites_visible, gps2_raw->dgps_numch, gps2_raw->dgps_age, gps2_raw->yaw, gps2_raw->alt_ellipsoid, gps2_raw->h_acc, gps2_raw->v_acc, gps2_raw->vel_acc, gps2_raw->hdg_acc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS2_RAW, (const char *)gps2_raw, MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN, MAVLINK_MSG_ID_GPS2_RAW_LEN, MAVLINK_MSG_ID_GPS2_RAW_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS2_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps2_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, uint8_t dgps_numch, uint32_t dgps_age, uint16_t yaw, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_uint32_t(buf, 20, dgps_age);
    _mav_put_uint16_t(buf, 24, eph);
    _mav_put_uint16_t(buf, 26, epv);
    _mav_put_uint16_t(buf, 28, vel);
    _mav_put_uint16_t(buf, 30, cog);
    _mav_put_uint8_t(buf, 32, fix_type);
    _mav_put_uint8_t(buf, 33, satellites_visible);
    _mav_put_uint8_t(buf, 34, dgps_numch);
    _mav_put_uint16_t(buf, 35, yaw);
    _mav_put_int32_t(buf, 37, alt_ellipsoid);
    _mav_put_uint32_t(buf, 41, h_acc);
    _mav_put_uint32_t(buf, 45, v_acc);
    _mav_put_uint32_t(buf, 49, vel_acc);
    _mav_put_uint32_t(buf, 53, hdg_acc);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS2_RAW, buf, MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN, MAVLINK_MSG_ID_GPS2_RAW_LEN, MAVLINK_MSG_ID_GPS2_RAW_CRC);
#else
    mavlink_gps2_raw_t *packet = (mavlink_gps2_raw_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->dgps_age = dgps_age;
    packet->eph = eph;
    packet->epv = epv;
    packet->vel = vel;
    packet->cog = cog;
    packet->fix_type = fix_type;
    packet->satellites_visible = satellites_visible;
    packet->dgps_numch = dgps_numch;
    packet->yaw = yaw;
    packet->alt_ellipsoid = alt_ellipsoid;
    packet->h_acc = h_acc;
    packet->v_acc = v_acc;
    packet->vel_acc = vel_acc;
    packet->hdg_acc = hdg_acc;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS2_RAW, (const char *)packet, MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN, MAVLINK_MSG_ID_GPS2_RAW_LEN, MAVLINK_MSG_ID_GPS2_RAW_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS2_RAW UNPACKING


/**
 * @brief Get field time_usec from gps2_raw message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_gps2_raw_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field fix_type from gps2_raw message
 *
 * @return  GPS fix type.
 */
static inline uint8_t mavlink_msg_gps2_raw_get_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field lat from gps2_raw message
 *
 * @return [degE7] Latitude (WGS84)
 */
static inline int32_t mavlink_msg_gps2_raw_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from gps2_raw message
 *
 * @return [degE7] Longitude (WGS84)
 */
static inline int32_t mavlink_msg_gps2_raw_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from gps2_raw message
 *
 * @return [mm] Altitude (MSL). Positive for up.
 */
static inline int32_t mavlink_msg_gps2_raw_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field eph from gps2_raw message
 *
 * @return  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_gps2_raw_get_eph(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field epv from gps2_raw message
 *
 * @return  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_gps2_raw_get_epv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field vel from gps2_raw message
 *
 * @return [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_gps2_raw_get_vel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field cog from gps2_raw message
 *
 * @return [cdeg] Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_gps2_raw_get_cog(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field satellites_visible from gps2_raw message
 *
 * @return  Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_gps2_raw_get_satellites_visible(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field dgps_numch from gps2_raw message
 *
 * @return  Number of DGPS satellites
 */
static inline uint8_t mavlink_msg_gps2_raw_get_dgps_numch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field dgps_age from gps2_raw message
 *
 * @return [ms] Age of DGPS info
 */
static inline uint32_t mavlink_msg_gps2_raw_get_dgps_age(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field yaw from gps2_raw message
 *
 * @return [cdeg] Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.
 */
static inline uint16_t mavlink_msg_gps2_raw_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  35);
}

/**
 * @brief Get field alt_ellipsoid from gps2_raw message
 *
 * @return [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
 */
static inline int32_t mavlink_msg_gps2_raw_get_alt_ellipsoid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  37);
}

/**
 * @brief Get field h_acc from gps2_raw message
 *
 * @return [mm] Position uncertainty.
 */
static inline uint32_t mavlink_msg_gps2_raw_get_h_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  41);
}

/**
 * @brief Get field v_acc from gps2_raw message
 *
 * @return [mm] Altitude uncertainty.
 */
static inline uint32_t mavlink_msg_gps2_raw_get_v_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  45);
}

/**
 * @brief Get field vel_acc from gps2_raw message
 *
 * @return [mm/s] Speed uncertainty.
 */
static inline uint32_t mavlink_msg_gps2_raw_get_vel_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  49);
}

/**
 * @brief Get field hdg_acc from gps2_raw message
 *
 * @return [degE5] Heading / track uncertainty
 */
static inline uint32_t mavlink_msg_gps2_raw_get_hdg_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  53);
}

/**
 * @brief Decode a gps2_raw message into a struct
 *
 * @param msg The message to decode
 * @param gps2_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps2_raw_decode(const mavlink_message_t* msg, mavlink_gps2_raw_t* gps2_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps2_raw->time_usec = mavlink_msg_gps2_raw_get_time_usec(msg);
    gps2_raw->lat = mavlink_msg_gps2_raw_get_lat(msg);
    gps2_raw->lon = mavlink_msg_gps2_raw_get_lon(msg);
    gps2_raw->alt = mavlink_msg_gps2_raw_get_alt(msg);
    gps2_raw->dgps_age = mavlink_msg_gps2_raw_get_dgps_age(msg);
    gps2_raw->eph = mavlink_msg_gps2_raw_get_eph(msg);
    gps2_raw->epv = mavlink_msg_gps2_raw_get_epv(msg);
    gps2_raw->vel = mavlink_msg_gps2_raw_get_vel(msg);
    gps2_raw->cog = mavlink_msg_gps2_raw_get_cog(msg);
    gps2_raw->fix_type = mavlink_msg_gps2_raw_get_fix_type(msg);
    gps2_raw->satellites_visible = mavlink_msg_gps2_raw_get_satellites_visible(msg);
    gps2_raw->dgps_numch = mavlink_msg_gps2_raw_get_dgps_numch(msg);
    gps2_raw->yaw = mavlink_msg_gps2_raw_get_yaw(msg);
    gps2_raw->alt_ellipsoid = mavlink_msg_gps2_raw_get_alt_ellipsoid(msg);
    gps2_raw->h_acc = mavlink_msg_gps2_raw_get_h_acc(msg);
    gps2_raw->v_acc = mavlink_msg_gps2_raw_get_v_acc(msg);
    gps2_raw->vel_acc = mavlink_msg_gps2_raw_get_vel_acc(msg);
    gps2_raw->hdg_acc = mavlink_msg_gps2_raw_get_hdg_acc(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS2_RAW_LEN? msg->len : MAVLINK_MSG_ID_GPS2_RAW_LEN;
        memset(gps2_raw, 0, MAVLINK_MSG_ID_GPS2_RAW_LEN);
    memcpy(gps2_raw, _MAV_PAYLOAD(msg), len);
#endif
}
