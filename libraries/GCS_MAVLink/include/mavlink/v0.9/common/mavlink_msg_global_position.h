// MESSAGE GLOBAL_POSITION PACKING

#define MAVLINK_MSG_ID_GLOBAL_POSITION 33

typedef struct __mavlink_global_position_t
{
 uint64_t usec; ///< Timestamp (microseconds since unix epoch)
 float lat; ///< Latitude, in degrees
 float lon; ///< Longitude, in degrees
 float alt; ///< Absolute altitude, in meters
 float vx; ///< X Speed (in Latitude direction, positive: going north)
 float vy; ///< Y Speed (in Longitude direction, positive: going east)
 float vz; ///< Z Speed (in Altitude direction, positive: going up)
} mavlink_global_position_t;

#define MAVLINK_MSG_ID_GLOBAL_POSITION_LEN 32
#define MAVLINK_MSG_ID_33_LEN 32



#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION { \
	"GLOBAL_POSITION", \
	7, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_global_position_t, usec) }, \
         { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_global_position_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_global_position_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_global_position_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_global_position_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_global_position_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_global_position_t, vz) }, \
         } \
}


/**
 * @brief Pack a global_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since unix epoch)
 * @param lat Latitude, in degrees
 * @param lon Longitude, in degrees
 * @param alt Absolute altitude, in meters
 * @param vx X Speed (in Latitude direction, positive: going north)
 * @param vy Y Speed (in Longitude direction, positive: going east)
 * @param vz Z Speed (in Altitude direction, positive: going up)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t usec, float lat, float lon, float alt, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, lat);
	_mav_put_float(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, vx);
	_mav_put_float(buf, 24, vy);
	_mav_put_float(buf, 28, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 32);
#else
	mavlink_global_position_t packet;
	packet.usec = usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 32);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION;
	return mavlink_finalize_message(msg, system_id, component_id, 32);
}

/**
 * @brief Pack a global_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since unix epoch)
 * @param lat Latitude, in degrees
 * @param lon Longitude, in degrees
 * @param alt Absolute altitude, in meters
 * @param vx X Speed (in Latitude direction, positive: going north)
 * @param vy Y Speed (in Longitude direction, positive: going east)
 * @param vz Z Speed (in Altitude direction, positive: going up)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t usec,float lat,float lon,float alt,float vx,float vy,float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, lat);
	_mav_put_float(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, vx);
	_mav_put_float(buf, 24, vy);
	_mav_put_float(buf, 28, vz);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 32);
#else
	mavlink_global_position_t packet;
	packet.usec = usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 32);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32);
}

/**
 * @brief Encode a global_position struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_t* global_position)
{
	return mavlink_msg_global_position_pack(system_id, component_id, msg, global_position->usec, global_position->lat, global_position->lon, global_position->alt, global_position->vx, global_position->vy, global_position->vz);
}

/**
 * @brief Send a global_position message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since unix epoch)
 * @param lat Latitude, in degrees
 * @param lon Longitude, in degrees
 * @param alt Absolute altitude, in meters
 * @param vx X Speed (in Latitude direction, positive: going north)
 * @param vy Y Speed (in Longitude direction, positive: going east)
 * @param vz Z Speed (in Altitude direction, positive: going up)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_send(mavlink_channel_t chan, uint64_t usec, float lat, float lon, float alt, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];
	_mav_put_uint64_t(buf, 0, usec);
	_mav_put_float(buf, 8, lat);
	_mav_put_float(buf, 12, lon);
	_mav_put_float(buf, 16, alt);
	_mav_put_float(buf, 20, vx);
	_mav_put_float(buf, 24, vy);
	_mav_put_float(buf, 28, vz);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION, buf, 32);
#else
	mavlink_global_position_t packet;
	packet.usec = usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION, (const char *)&packet, 32);
#endif
}

#endif

// MESSAGE GLOBAL_POSITION UNPACKING


/**
 * @brief Get field usec from global_position message
 *
 * @return Timestamp (microseconds since unix epoch)
 */
static inline uint64_t mavlink_msg_global_position_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field lat from global_position message
 *
 * @return Latitude, in degrees
 */
static inline float mavlink_msg_global_position_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field lon from global_position message
 *
 * @return Longitude, in degrees
 */
static inline float mavlink_msg_global_position_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field alt from global_position message
 *
 * @return Absolute altitude, in meters
 */
static inline float mavlink_msg_global_position_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vx from global_position message
 *
 * @return X Speed (in Latitude direction, positive: going north)
 */
static inline float mavlink_msg_global_position_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vy from global_position message
 *
 * @return Y Speed (in Longitude direction, positive: going east)
 */
static inline float mavlink_msg_global_position_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vz from global_position message
 *
 * @return Z Speed (in Altitude direction, positive: going up)
 */
static inline float mavlink_msg_global_position_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a global_position message into a struct
 *
 * @param msg The message to decode
 * @param global_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_position_decode(const mavlink_message_t* msg, mavlink_global_position_t* global_position)
{
#if MAVLINK_NEED_BYTE_SWAP
	global_position->usec = mavlink_msg_global_position_get_usec(msg);
	global_position->lat = mavlink_msg_global_position_get_lat(msg);
	global_position->lon = mavlink_msg_global_position_get_lon(msg);
	global_position->alt = mavlink_msg_global_position_get_alt(msg);
	global_position->vx = mavlink_msg_global_position_get_vx(msg);
	global_position->vy = mavlink_msg_global_position_get_vy(msg);
	global_position->vz = mavlink_msg_global_position_get_vz(msg);
#else
	memcpy(global_position, _MAV_PAYLOAD(msg), 32);
#endif
}
