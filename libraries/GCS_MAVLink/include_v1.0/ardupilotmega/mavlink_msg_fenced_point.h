// MESSAGE FENCED_POINT PACKING

#define MAVLINK_MSG_ID_FENCED_POINT 160

typedef struct __mavlink_fenced_point_t
{
 float lat; ///< Latitude of point
 float lng; ///< Longitude of point
 uint8_t idx; ///< point index (first point is 1, 0 is for return point)
 uint8_t count; ///< total number of points (for sanity checking)
} mavlink_fenced_point_t;

#define MAVLINK_MSG_ID_FENCED_POINT_LEN 10
#define MAVLINK_MSG_ID_160_LEN 10



#define MAVLINK_MESSAGE_INFO_FENCED_POINT { \
	"FENCED_POINT", \
	4, \
	{  { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_fenced_point_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_fenced_point_t, lng) }, \
         { "idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_fenced_point_t, idx) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_fenced_point_t, count) }, \
         } \
}


/**
 * @brief Pack a fenced_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param idx point index (first point is 1, 0 is for return point)
 * @param count total number of points (for sanity checking)
 * @param lat Latitude of point
 * @param lng Longitude of point
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fenced_point_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t idx, uint8_t count, float lat, float lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_float(buf, 0, lat);
	_mav_put_float(buf, 4, lng);
	_mav_put_uint8_t(buf, 8, idx);
	_mav_put_uint8_t(buf, 9, count);

        memcpy(_MAV_PAYLOAD(msg), buf, 10);
#else
	mavlink_fenced_point_t packet;
	packet.lat = lat;
	packet.lng = lng;
	packet.idx = idx;
	packet.count = count;

        memcpy(_MAV_PAYLOAD(msg), &packet, 10);
#endif

	msg->msgid = MAVLINK_MSG_ID_FENCED_POINT;
	return mavlink_finalize_message(msg, system_id, component_id, 10, 225);
}

/**
 * @brief Pack a fenced_point message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param idx point index (first point is 1, 0 is for return point)
 * @param count total number of points (for sanity checking)
 * @param lat Latitude of point
 * @param lng Longitude of point
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fenced_point_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t idx,uint8_t count,float lat,float lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_float(buf, 0, lat);
	_mav_put_float(buf, 4, lng);
	_mav_put_uint8_t(buf, 8, idx);
	_mav_put_uint8_t(buf, 9, count);

        memcpy(_MAV_PAYLOAD(msg), buf, 10);
#else
	mavlink_fenced_point_t packet;
	packet.lat = lat;
	packet.lng = lng;
	packet.idx = idx;
	packet.count = count;

        memcpy(_MAV_PAYLOAD(msg), &packet, 10);
#endif

	msg->msgid = MAVLINK_MSG_ID_FENCED_POINT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 10, 225);
}

/**
 * @brief Encode a fenced_point struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fenced_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fenced_point_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fenced_point_t* fenced_point)
{
	return mavlink_msg_fenced_point_pack(system_id, component_id, msg, fenced_point->idx, fenced_point->count, fenced_point->lat, fenced_point->lng);
}

/**
 * @brief Send a fenced_point message
 * @param chan MAVLink channel to send the message
 *
 * @param idx point index (first point is 1, 0 is for return point)
 * @param count total number of points (for sanity checking)
 * @param lat Latitude of point
 * @param lng Longitude of point
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fenced_point_send(mavlink_channel_t chan, uint8_t idx, uint8_t count, float lat, float lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_float(buf, 0, lat);
	_mav_put_float(buf, 4, lng);
	_mav_put_uint8_t(buf, 8, idx);
	_mav_put_uint8_t(buf, 9, count);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCED_POINT, buf, 10, 225);
#else
	mavlink_fenced_point_t packet;
	packet.lat = lat;
	packet.lng = lng;
	packet.idx = idx;
	packet.count = count;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FENCED_POINT, (const char *)&packet, 10, 225);
#endif
}

#endif

// MESSAGE FENCED_POINT UNPACKING


/**
 * @brief Get field idx from fenced_point message
 *
 * @return point index (first point is 1, 0 is for return point)
 */
static inline uint8_t mavlink_msg_fenced_point_get_idx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field count from fenced_point message
 *
 * @return total number of points (for sanity checking)
 */
static inline uint8_t mavlink_msg_fenced_point_get_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field lat from fenced_point message
 *
 * @return Latitude of point
 */
static inline float mavlink_msg_fenced_point_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field lng from fenced_point message
 *
 * @return Longitude of point
 */
static inline float mavlink_msg_fenced_point_get_lng(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a fenced_point message into a struct
 *
 * @param msg The message to decode
 * @param fenced_point C-struct to decode the message contents into
 */
static inline void mavlink_msg_fenced_point_decode(const mavlink_message_t* msg, mavlink_fenced_point_t* fenced_point)
{
#if MAVLINK_NEED_BYTE_SWAP
	fenced_point->lat = mavlink_msg_fenced_point_get_lat(msg);
	fenced_point->lng = mavlink_msg_fenced_point_get_lng(msg);
	fenced_point->idx = mavlink_msg_fenced_point_get_idx(msg);
	fenced_point->count = mavlink_msg_fenced_point_get_count(msg);
#else
	memcpy(fenced_point, _MAV_PAYLOAD(msg), 10);
#endif
}
