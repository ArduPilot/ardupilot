// MESSAGE GPS_RAW_INT PACKING

#define MAVLINK_MSG_ID_GPS_RAW_INT 24

typedef struct __mavlink_gps_raw_int_t
{
 uint64_t time_usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 int32_t lat; ///< Latitude in 1E7 degrees
 int32_t lon; ///< Longitude in 1E7 degrees
 int32_t alt; ///< Altitude in 1E3 meters (millimeters) above MSL
 uint16_t eph; ///< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 uint16_t epv; ///< GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 uint16_t vel; ///< GPS ground speed (m/s * 100). If unknown, set to: 65535
 uint16_t cog; ///< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 uint8_t fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 uint8_t satellites_visible; ///< Number of satellites visible. If unknown, set to 255
} mavlink_gps_raw_int_t;

#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30
#define MAVLINK_MSG_ID_24_LEN 30



#define MAVLINK_MESSAGE_INFO_GPS_RAW_INT { \
	"GPS_RAW_INT", \
	10, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_raw_int_t, time_usec) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gps_raw_int_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gps_raw_int_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gps_raw_int_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_gps_raw_int_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_gps_raw_int_t, epv) }, \
         { "vel", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_gps_raw_int_t, vel) }, \
         { "cog", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_gps_raw_int_t, cog) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_gps_raw_int_t, fix_type) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_gps_raw_int_t, satellites_visible) }, \
         } \
}


/**
 * @brief Pack a gps_raw_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters) above MSL
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_raw_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[30];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, eph);
	_mav_put_uint16_t(buf, 22, epv);
	_mav_put_uint16_t(buf, 24, vel);
	_mav_put_uint16_t(buf, 26, cog);
	_mav_put_uint8_t(buf, 28, fix_type);
	_mav_put_uint8_t(buf, 29, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);
#else
	mavlink_gps_raw_int_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.vel = vel;
	packet.cog = cog;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 30);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_RAW_INT;
	return mavlink_finalize_message(msg, system_id, component_id, 30, 24);
}

/**
 * @brief Pack a gps_raw_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters) above MSL
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_raw_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,uint16_t cog,uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[30];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, eph);
	_mav_put_uint16_t(buf, 22, epv);
	_mav_put_uint16_t(buf, 24, vel);
	_mav_put_uint16_t(buf, 26, cog);
	_mav_put_uint8_t(buf, 28, fix_type);
	_mav_put_uint8_t(buf, 29, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 30);
#else
	mavlink_gps_raw_int_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.vel = vel;
	packet.cog = cog;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 30);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_RAW_INT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 30, 24);
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
	return mavlink_msg_gps_raw_int_pack(system_id, component_id, msg, gps_raw_int->time_usec, gps_raw_int->fix_type, gps_raw_int->lat, gps_raw_int->lon, gps_raw_int->alt, gps_raw_int->eph, gps_raw_int->epv, gps_raw_int->vel, gps_raw_int->cog, gps_raw_int->satellites_visible);
}

/**
 * @brief Send a gps_raw_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters) above MSL
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_raw_int_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[30];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, eph);
	_mav_put_uint16_t(buf, 22, epv);
	_mav_put_uint16_t(buf, 24, vel);
	_mav_put_uint16_t(buf, 26, cog);
	_mav_put_uint8_t(buf, 28, fix_type);
	_mav_put_uint8_t(buf, 29, satellites_visible);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW_INT, buf, 30, 24);
#else
	mavlink_gps_raw_int_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.vel = vel;
	packet.cog = cog;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW_INT, (const char *)&packet, 30, 24);
#endif
}

#endif

// MESSAGE GPS_RAW_INT UNPACKING


/**
 * @brief Get field time_usec from gps_raw_int message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_gps_raw_int_get_time_usec(const mavlink_message_t* msg)
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
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field lat from gps_raw_int message
 *
 * @return Latitude in 1E7 degrees
 */
static inline int32_t mavlink_msg_gps_raw_int_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from gps_raw_int message
 *
 * @return Longitude in 1E7 degrees
 */
static inline int32_t mavlink_msg_gps_raw_int_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from gps_raw_int message
 *
 * @return Altitude in 1E3 meters (millimeters) above MSL
 */
static inline int32_t mavlink_msg_gps_raw_int_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field eph from gps_raw_int message
 *
 * @return GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_gps_raw_int_get_eph(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field epv from gps_raw_int message
 *
 * @return GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_gps_raw_int_get_epv(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field vel from gps_raw_int message
 *
 * @return GPS ground speed (m/s * 100). If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_gps_raw_int_get_vel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field cog from gps_raw_int message
 *
 * @return Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_gps_raw_int_get_cog(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field satellites_visible from gps_raw_int message
 *
 * @return Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_gps_raw_int_get_satellites_visible(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  29);
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
	gps_raw_int->time_usec = mavlink_msg_gps_raw_int_get_time_usec(msg);
	gps_raw_int->lat = mavlink_msg_gps_raw_int_get_lat(msg);
	gps_raw_int->lon = mavlink_msg_gps_raw_int_get_lon(msg);
	gps_raw_int->alt = mavlink_msg_gps_raw_int_get_alt(msg);
	gps_raw_int->eph = mavlink_msg_gps_raw_int_get_eph(msg);
	gps_raw_int->epv = mavlink_msg_gps_raw_int_get_epv(msg);
	gps_raw_int->vel = mavlink_msg_gps_raw_int_get_vel(msg);
	gps_raw_int->cog = mavlink_msg_gps_raw_int_get_cog(msg);
	gps_raw_int->fix_type = mavlink_msg_gps_raw_int_get_fix_type(msg);
	gps_raw_int->satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(msg);
#else
	memcpy(gps_raw_int, _MAV_PAYLOAD(msg), 30);
#endif
}
