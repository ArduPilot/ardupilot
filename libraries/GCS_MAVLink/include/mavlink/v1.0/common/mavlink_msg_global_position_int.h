// MESSAGE GLOBAL_POSITION_INT PACKING

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33

typedef struct __mavlink_global_position_int_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int32_t lat; ///< Latitude, expressed as * 1E7
 int32_t lon; ///< Longitude, expressed as * 1E7
 int32_t alt; ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
 int32_t relative_alt; ///< Altitude above ground in meters, expressed as * 1000 (millimeters)
 int16_t vx; ///< Ground X Speed (Latitude), expressed as m/s * 100
 int16_t vy; ///< Ground Y Speed (Longitude), expressed as m/s * 100
 int16_t vz; ///< Ground Z Speed (Altitude), expressed as m/s * 100
 uint16_t hdg; ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
} mavlink_global_position_int_t;

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 28
#define MAVLINK_MSG_ID_33_LEN 28



#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT { \
	"GLOBAL_POSITION_INT", \
	9, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_global_position_int_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_global_position_int_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_position_int_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_position_int_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_global_position_int_t, relative_alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_global_position_int_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_global_position_int_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_global_position_int_t, vz) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_global_position_int_t, hdg) }, \
         } \
}


/**
 * @brief Pack a global_position_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_uint16_t(buf, 26, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 28);
#else
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	return mavlink_finalize_message(msg, system_id, component_id, 28, 104);
}

/**
 * @brief Pack a global_position_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,int16_t vx,int16_t vy,int16_t vz,uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_uint16_t(buf, 26, hdg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 28);
#else
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 28, 104);
}

/**
 * @brief Encode a global_position_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_int_t* global_position_int)
{
	return mavlink_msg_global_position_int_pack(system_id, component_id, msg, global_position_int->time_boot_ms, global_position_int->lat, global_position_int->lon, global_position_int->alt, global_position_int->relative_alt, global_position_int->vx, global_position_int->vy, global_position_int->vz, global_position_int->hdg);
}

/**
 * @brief Send a global_position_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, alt);
	_mav_put_int32_t(buf, 16, relative_alt);
	_mav_put_int16_t(buf, 20, vx);
	_mav_put_int16_t(buf, 22, vy);
	_mav_put_int16_t(buf, 24, vz);
	_mav_put_uint16_t(buf, 26, hdg);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, buf, 28, 104);
#else
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, (const char *)&packet, 28, 104);
#endif
}

#endif

// MESSAGE GLOBAL_POSITION_INT UNPACKING


/**
 * @brief Get field time_boot_ms from global_position_int message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_global_position_int_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from global_position_int message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_global_position_int_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from global_position_int message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_global_position_int_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from global_position_int message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), above MSL
 */
static inline int32_t mavlink_msg_global_position_int_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field relative_alt from global_position_int message
 *
 * @return Altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_global_position_int_get_relative_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field vx from global_position_int message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_global_position_int_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field vy from global_position_int message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_global_position_int_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field vz from global_position_int message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_global_position_int_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field hdg from global_position_int message
 *
 * @return Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_global_position_int_get_hdg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Decode a global_position_int message into a struct
 *
 * @param msg The message to decode
 * @param global_position_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_position_int_decode(const mavlink_message_t* msg, mavlink_global_position_int_t* global_position_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	global_position_int->time_boot_ms = mavlink_msg_global_position_int_get_time_boot_ms(msg);
	global_position_int->lat = mavlink_msg_global_position_int_get_lat(msg);
	global_position_int->lon = mavlink_msg_global_position_int_get_lon(msg);
	global_position_int->alt = mavlink_msg_global_position_int_get_alt(msg);
	global_position_int->relative_alt = mavlink_msg_global_position_int_get_relative_alt(msg);
	global_position_int->vx = mavlink_msg_global_position_int_get_vx(msg);
	global_position_int->vy = mavlink_msg_global_position_int_get_vy(msg);
	global_position_int->vz = mavlink_msg_global_position_int_get_vz(msg);
	global_position_int->hdg = mavlink_msg_global_position_int_get_hdg(msg);
#else
	memcpy(global_position_int, _MAV_PAYLOAD(msg), 28);
#endif
}
