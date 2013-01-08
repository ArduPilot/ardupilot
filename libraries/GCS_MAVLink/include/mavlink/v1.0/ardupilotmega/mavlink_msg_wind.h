// MESSAGE WIND PACKING

#define MAVLINK_MSG_ID_WIND 168

typedef struct __mavlink_wind_t
{
 float direction; ///< wind direction that wind is coming from (degrees)
 float speed; ///< wind speed in ground plane (m/s)
 float speed_z; ///< vertical wind speed (m/s)
} mavlink_wind_t;

#define MAVLINK_MSG_ID_WIND_LEN 12
#define MAVLINK_MSG_ID_168_LEN 12



#define MAVLINK_MESSAGE_INFO_WIND { \
	"WIND", \
	3, \
	{  { "direction", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_wind_t, direction) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_wind_t, speed) }, \
         { "speed_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wind_t, speed_z) }, \
         } \
}


/**
 * @brief Pack a wind message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param direction wind direction that wind is coming from (degrees)
 * @param speed wind speed in ground plane (m/s)
 * @param speed_z vertical wind speed (m/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wind_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float direction, float speed, float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, speed);
	_mav_put_float(buf, 8, speed_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_wind_t packet;
	packet.direction = direction;
	packet.speed = speed;
	packet.speed_z = speed_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_WIND;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 1);
}

/**
 * @brief Pack a wind message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param direction wind direction that wind is coming from (degrees)
 * @param speed wind speed in ground plane (m/s)
 * @param speed_z vertical wind speed (m/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wind_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float direction,float speed,float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, speed);
	_mav_put_float(buf, 8, speed_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_wind_t packet;
	packet.direction = direction;
	packet.speed = speed;
	packet.speed_z = speed_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_WIND;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 1);
}

/**
 * @brief Encode a wind struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wind C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wind_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wind_t* wind)
{
	return mavlink_msg_wind_pack(system_id, component_id, msg, wind->direction, wind->speed, wind->speed_z);
}

/**
 * @brief Send a wind message
 * @param chan MAVLink channel to send the message
 *
 * @param direction wind direction that wind is coming from (degrees)
 * @param speed wind speed in ground plane (m/s)
 * @param speed_z vertical wind speed (m/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wind_send(mavlink_channel_t chan, float direction, float speed, float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, speed);
	_mav_put_float(buf, 8, speed_z);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND, buf, 12, 1);
#else
	mavlink_wind_t packet;
	packet.direction = direction;
	packet.speed = speed;
	packet.speed_z = speed_z;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND, (const char *)&packet, 12, 1);
#endif
}

#endif

// MESSAGE WIND UNPACKING


/**
 * @brief Get field direction from wind message
 *
 * @return wind direction that wind is coming from (degrees)
 */
static inline float mavlink_msg_wind_get_direction(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field speed from wind message
 *
 * @return wind speed in ground plane (m/s)
 */
static inline float mavlink_msg_wind_get_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field speed_z from wind message
 *
 * @return vertical wind speed (m/s)
 */
static inline float mavlink_msg_wind_get_speed_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a wind message into a struct
 *
 * @param msg The message to decode
 * @param wind C-struct to decode the message contents into
 */
static inline void mavlink_msg_wind_decode(const mavlink_message_t* msg, mavlink_wind_t* wind)
{
#if MAVLINK_NEED_BYTE_SWAP
	wind->direction = mavlink_msg_wind_get_direction(msg);
	wind->speed = mavlink_msg_wind_get_speed(msg);
	wind->speed_z = mavlink_msg_wind_get_speed_z(msg);
#else
	memcpy(wind, _MAV_PAYLOAD(msg), 12);
#endif
}
