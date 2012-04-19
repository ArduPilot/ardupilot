// MESSAGE OBS_POSITION PACKING

#define MAVLINK_MSG_ID_OBS_POSITION 170

typedef struct __mavlink_obs_position_t
{
 int32_t lon; ///< Longitude expressed in 1E7
 int32_t lat; ///< Latitude expressed in 1E7
 int32_t alt; ///< Altitude expressed in milimeters
} mavlink_obs_position_t;

#define MAVLINK_MSG_ID_OBS_POSITION_LEN 12
#define MAVLINK_MSG_ID_170_LEN 12



#define MAVLINK_MESSAGE_INFO_OBS_POSITION { \
	"OBS_POSITION", \
	3, \
	{  { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_obs_position_t, lon) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_obs_position_t, lat) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_obs_position_t, alt) }, \
         } \
}


/**
 * @brief Pack a obs_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lon Longitude expressed in 1E7
 * @param lat Latitude expressed in 1E7
 * @param alt Altitude expressed in milimeters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t lon, int32_t lat, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_int32_t(buf, 0, lon);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_obs_position_t packet;
	packet.lon = lon;
	packet.lat = lat;
	packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_POSITION;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 15);
}

/**
 * @brief Pack a obs_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param lon Longitude expressed in 1E7
 * @param lat Latitude expressed in 1E7
 * @param alt Altitude expressed in milimeters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t lon,int32_t lat,int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_int32_t(buf, 0, lon);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_obs_position_t packet;
	packet.lon = lon;
	packet.lat = lat;
	packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_POSITION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 15);
}

/**
 * @brief Encode a obs_position struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param obs_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obs_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_obs_position_t* obs_position)
{
	return mavlink_msg_obs_position_pack(system_id, component_id, msg, obs_position->lon, obs_position->lat, obs_position->alt);
}

/**
 * @brief Send a obs_position message
 * @param chan MAVLink channel to send the message
 *
 * @param lon Longitude expressed in 1E7
 * @param lat Latitude expressed in 1E7
 * @param alt Altitude expressed in milimeters
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_obs_position_send(mavlink_channel_t chan, int32_t lon, int32_t lat, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_int32_t(buf, 0, lon);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, alt);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_POSITION, buf, 12, 15);
#else
	mavlink_obs_position_t packet;
	packet.lon = lon;
	packet.lat = lat;
	packet.alt = alt;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_POSITION, (const char *)&packet, 12, 15);
#endif
}

#endif

// MESSAGE OBS_POSITION UNPACKING


/**
 * @brief Get field lon from obs_position message
 *
 * @return Longitude expressed in 1E7
 */
static inline int32_t mavlink_msg_obs_position_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lat from obs_position message
 *
 * @return Latitude expressed in 1E7
 */
static inline int32_t mavlink_msg_obs_position_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from obs_position message
 *
 * @return Altitude expressed in milimeters
 */
static inline int32_t mavlink_msg_obs_position_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a obs_position message into a struct
 *
 * @param msg The message to decode
 * @param obs_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_obs_position_decode(const mavlink_message_t* msg, mavlink_obs_position_t* obs_position)
{
#if MAVLINK_NEED_BYTE_SWAP
	obs_position->lon = mavlink_msg_obs_position_get_lon(msg);
	obs_position->lat = mavlink_msg_obs_position_get_lat(msg);
	obs_position->alt = mavlink_msg_obs_position_get_alt(msg);
#else
	memcpy(obs_position, _MAV_PAYLOAD(msg), 12);
#endif
}
