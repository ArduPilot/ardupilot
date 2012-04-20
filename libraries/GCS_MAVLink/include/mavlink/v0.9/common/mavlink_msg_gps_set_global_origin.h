// MESSAGE GPS_SET_GLOBAL_ORIGIN PACKING

#define MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN 48

typedef struct __mavlink_gps_set_global_origin_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 int32_t latitude; ///< global position * 1E7
 int32_t longitude; ///< global position * 1E7
 int32_t altitude; ///< global position * 1000
} mavlink_gps_set_global_origin_t;

#define MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN_LEN 14
#define MAVLINK_MSG_ID_48_LEN 14



#define MAVLINK_MESSAGE_INFO_GPS_SET_GLOBAL_ORIGIN { \
	"GPS_SET_GLOBAL_ORIGIN", \
	5, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_set_global_origin_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gps_set_global_origin_t, target_component) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 2, offsetof(mavlink_gps_set_global_origin_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 6, offsetof(mavlink_gps_set_global_origin_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 10, offsetof(mavlink_gps_set_global_origin_t, altitude) }, \
         } \
}


/**
 * @brief Pack a gps_set_global_origin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_set_global_origin_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_int32_t(buf, 2, latitude);
	_mav_put_int32_t(buf, 6, longitude);
	_mav_put_int32_t(buf, 10, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 14);
#else
	mavlink_gps_set_global_origin_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN;
	return mavlink_finalize_message(msg, system_id, component_id, 14);
}

/**
 * @brief Pack a gps_set_global_origin message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_set_global_origin_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,int32_t latitude,int32_t longitude,int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_int32_t(buf, 2, latitude);
	_mav_put_int32_t(buf, 6, longitude);
	_mav_put_int32_t(buf, 10, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 14);
#else
	mavlink_gps_set_global_origin_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14);
}

/**
 * @brief Encode a gps_set_global_origin struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_set_global_origin C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_set_global_origin_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_set_global_origin_t* gps_set_global_origin)
{
	return mavlink_msg_gps_set_global_origin_pack(system_id, component_id, msg, gps_set_global_origin->target_system, gps_set_global_origin->target_component, gps_set_global_origin->latitude, gps_set_global_origin->longitude, gps_set_global_origin->altitude);
}

/**
 * @brief Send a gps_set_global_origin message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_set_global_origin_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int32_t latitude, int32_t longitude, int32_t altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_int32_t(buf, 2, latitude);
	_mav_put_int32_t(buf, 6, longitude);
	_mav_put_int32_t(buf, 10, altitude);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN, buf, 14);
#else
	mavlink_gps_set_global_origin_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN, (const char *)&packet, 14);
#endif
}

#endif

// MESSAGE GPS_SET_GLOBAL_ORIGIN UNPACKING


/**
 * @brief Get field target_system from gps_set_global_origin message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gps_set_global_origin_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from gps_set_global_origin message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gps_set_global_origin_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field latitude from gps_set_global_origin message
 *
 * @return global position * 1E7
 */
static inline int32_t mavlink_msg_gps_set_global_origin_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  2);
}

/**
 * @brief Get field longitude from gps_set_global_origin message
 *
 * @return global position * 1E7
 */
static inline int32_t mavlink_msg_gps_set_global_origin_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  6);
}

/**
 * @brief Get field altitude from gps_set_global_origin message
 *
 * @return global position * 1000
 */
static inline int32_t mavlink_msg_gps_set_global_origin_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  10);
}

/**
 * @brief Decode a gps_set_global_origin message into a struct
 *
 * @param msg The message to decode
 * @param gps_set_global_origin C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_set_global_origin_decode(const mavlink_message_t* msg, mavlink_gps_set_global_origin_t* gps_set_global_origin)
{
#if MAVLINK_NEED_BYTE_SWAP
	gps_set_global_origin->target_system = mavlink_msg_gps_set_global_origin_get_target_system(msg);
	gps_set_global_origin->target_component = mavlink_msg_gps_set_global_origin_get_target_component(msg);
	gps_set_global_origin->latitude = mavlink_msg_gps_set_global_origin_get_latitude(msg);
	gps_set_global_origin->longitude = mavlink_msg_gps_set_global_origin_get_longitude(msg);
	gps_set_global_origin->altitude = mavlink_msg_gps_set_global_origin_get_altitude(msg);
#else
	memcpy(gps_set_global_origin, _MAV_PAYLOAD(msg), 14);
#endif
}
