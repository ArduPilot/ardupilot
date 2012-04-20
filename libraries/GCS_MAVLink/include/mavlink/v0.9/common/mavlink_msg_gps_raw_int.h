// MESSAGE GPS_RAW_INT PACKING

#define MAVLINK_MSG_ID_GPS_RAW_INT 25

typedef struct __mavlink_gps_raw_int_t
{
 uint64_t usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 uint8_t fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 int32_t lat; ///< Latitude in 1E7 degrees
 int32_t lon; ///< Longitude in 1E7 degrees
 int32_t alt; ///< Altitude in 1E3 meters (millimeters)
 float eph; ///< GPS HDOP
 float epv; ///< GPS VDOP
 float v; ///< GPS ground speed (m/s)
 float hdg; ///< Compass heading in degrees, 0..360 degrees
} mavlink_gps_raw_int_t;

#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 37
#define MAVLINK_MSG_ID_25_LEN 37



#define MAVLINK_MESSAGE_INFO_GPS_RAW_INT { \
	"GPS_RAW_INT", \
	9, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_raw_int_t, usec) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gps_raw_int_t, fix_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 9, offsetof(mavlink_gps_raw_int_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 13, offsetof(mavlink_gps_raw_int_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 17, offsetof(mavlink_gps_raw_int_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_FLOAT, 0, 21, offsetof(mavlink_gps_raw_int_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_FLOAT, 0, 25, offsetof(mavlink_gps_raw_int_t, epv) }, \
         { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 29, offsetof(mavlink_gps_raw_int_t, v) }, \
         { "hdg", NULL, MAVLINK_TYPE_FLOAT, 0, 33, offsetof(mavlink_gps_raw_int_t, hdg) }, \
         } \
}


/**
 * @brief Pack a gps_raw_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters)
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed (m/s)
 * @param hdg Compass heading in degrees, 0..360 degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_raw_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, float eph, float epv, float v, float hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[37];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_uint8_t(buf, 8, fix_type);
	_mav_put_int32_t(buf, 9, lat);
	_mav_put_int32_t(buf, 13, lon);
	_mav_put_int32_t(buf, 17, alt);
	_mav_put_float(buf, 21, eph);
	_mav_put_float(buf, 25, epv);
	_mav_put_float(buf, 29, v);
	_mav_put_float(buf, 33, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 37);
#else
	mavlink_gps_raw_int_t packet;
	packet.usec = usec;
	packet.fix_type = fix_type;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.v = v;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 37);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_RAW_INT;
	return mavlink_finalize_message(msg, system_id, component_id, 37);
}

/**
 * @brief Pack a gps_raw_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters)
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed (m/s)
 * @param hdg Compass heading in degrees, 0..360 degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_raw_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,float eph,float epv,float v,float hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[37];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_uint8_t(buf, 8, fix_type);
	_mav_put_int32_t(buf, 9, lat);
	_mav_put_int32_t(buf, 13, lon);
	_mav_put_int32_t(buf, 17, alt);
	_mav_put_float(buf, 21, eph);
	_mav_put_float(buf, 25, epv);
	_mav_put_float(buf, 29, v);
	_mav_put_float(buf, 33, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 37);
#else
	mavlink_gps_raw_int_t packet;
	packet.usec = usec;
	packet.fix_type = fix_type;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.v = v;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 37);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_RAW_INT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 37);
}

/**
 * @brief Encode a gps_raw_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_raw_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_raw_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_raw_int_t* gps_raw_int)
{
	return mavlink_msg_gps_raw_int_pack(system_id, component_id, msg, gps_raw_int->usec, gps_raw_int->fix_type, gps_raw_int->lat, gps_raw_int->lon, gps_raw_int->alt, gps_raw_int->eph, gps_raw_int->epv, gps_raw_int->v, gps_raw_int->hdg);
}

/**
 * @brief Send a gps_raw_int message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters)
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed (m/s)
 * @param hdg Compass heading in degrees, 0..360 degrees
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_raw_int_send(mavlink_channel_t chan, uint64_t usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, float eph, float epv, float v, float hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[37];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_uint8_t(buf, 8, fix_type);
	_mav_put_int32_t(buf, 9, lat);
	_mav_put_int32_t(buf, 13, lon);
	_mav_put_int32_t(buf, 17, alt);
	_mav_put_float(buf, 21, eph);
	_mav_put_float(buf, 25, epv);
	_mav_put_float(buf, 29, v);
	_mav_put_float(buf, 33, hdg);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW_INT, buf, 37);
#else
	mavlink_gps_raw_int_t packet;
	packet.usec = usec;
	packet.fix_type = fix_type;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.v = v;
	packet.hdg = hdg;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW_INT, (const char *)&packet, 37);
#endif
}

#endif

// MESSAGE GPS_RAW_INT UNPACKING


/**
 * @brief Get field usec from gps_raw_int message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_gps_raw_int_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field fix_type from gps_raw_int message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 */
static inline uint8_t mavlink_msg_gps_raw_int_get_fix_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field lat from gps_raw_int message
 *
 * @return Latitude in 1E7 degrees
 */
static inline int32_t mavlink_msg_gps_raw_int_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  9);
}

/**
 * @brief Get field lon from gps_raw_int message
 *
 * @return Longitude in 1E7 degrees
 */
static inline int32_t mavlink_msg_gps_raw_int_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  13);
}

/**
 * @brief Get field alt from gps_raw_int message
 *
 * @return Altitude in 1E3 meters (millimeters)
 */
static inline int32_t mavlink_msg_gps_raw_int_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  17);
}

/**
 * @brief Get field eph from gps_raw_int message
 *
 * @return GPS HDOP
 */
static inline float mavlink_msg_gps_raw_int_get_eph(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  21);
}

/**
 * @brief Get field epv from gps_raw_int message
 *
 * @return GPS VDOP
 */
static inline float mavlink_msg_gps_raw_int_get_epv(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  25);
}

/**
 * @brief Get field v from gps_raw_int message
 *
 * @return GPS ground speed (m/s)
 */
static inline float mavlink_msg_gps_raw_int_get_v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  29);
}

/**
 * @brief Get field hdg from gps_raw_int message
 *
 * @return Compass heading in degrees, 0..360 degrees
 */
static inline float mavlink_msg_gps_raw_int_get_hdg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  33);
}

/**
 * @brief Decode a gps_raw_int message into a struct
 *
 * @param msg The message to decode
 * @param gps_raw_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_raw_int_decode(const mavlink_message_t* msg, mavlink_gps_raw_int_t* gps_raw_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	gps_raw_int->usec = mavlink_msg_gps_raw_int_get_usec(msg);
	gps_raw_int->fix_type = mavlink_msg_gps_raw_int_get_fix_type(msg);
	gps_raw_int->lat = mavlink_msg_gps_raw_int_get_lat(msg);
	gps_raw_int->lon = mavlink_msg_gps_raw_int_get_lon(msg);
	gps_raw_int->alt = mavlink_msg_gps_raw_int_get_alt(msg);
	gps_raw_int->eph = mavlink_msg_gps_raw_int_get_eph(msg);
	gps_raw_int->epv = mavlink_msg_gps_raw_int_get_epv(msg);
	gps_raw_int->v = mavlink_msg_gps_raw_int_get_v(msg);
	gps_raw_int->hdg = mavlink_msg_gps_raw_int_get_hdg(msg);
#else
	memcpy(gps_raw_int, _MAV_PAYLOAD(msg), 37);
#endif
}
