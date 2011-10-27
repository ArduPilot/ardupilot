// MESSAGE POSITION_CONTROL_SETPOINT PACKING

#define MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT 170

typedef struct __mavlink_position_control_setpoint_t
{
 float x; ///< x position
 float y; ///< y position
 float z; ///< z position
 float yaw; ///< yaw orientation in radians, 0 = NORTH
 uint16_t id; ///< ID of waypoint, 0 for plain position
} mavlink_position_control_setpoint_t;

#define MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT_LEN 18
#define MAVLINK_MSG_ID_170_LEN 18



#define MAVLINK_MESSAGE_INFO_POSITION_CONTROL_SETPOINT { \
	"POSITION_CONTROL_SETPOINT", \
	5, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_position_control_setpoint_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_position_control_setpoint_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_position_control_setpoint_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_position_control_setpoint_t, yaw) }, \
         { "id", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_position_control_setpoint_t, id) }, \
         } \
}


/**
 * @brief Pack a position_control_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_position_control_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t id, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint16_t(buf, 16, id);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_position_control_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.id = id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT;
	return mavlink_finalize_message(msg, system_id, component_id, 18, 28);
}

/**
 * @brief Pack a position_control_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_position_control_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t id,float x,float y,float z,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint16_t(buf, 16, id);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_position_control_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.id = id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 28);
}

/**
 * @brief Encode a position_control_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param position_control_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_position_control_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_position_control_setpoint_t* position_control_setpoint)
{
	return mavlink_msg_position_control_setpoint_pack(system_id, component_id, msg, position_control_setpoint->id, position_control_setpoint->x, position_control_setpoint->y, position_control_setpoint->z, position_control_setpoint->yaw);
}

/**
 * @brief Send a position_control_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_position_control_setpoint_send(mavlink_channel_t chan, uint16_t id, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint16_t(buf, 16, id);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT, buf, 18, 28);
#else
	mavlink_position_control_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.id = id;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT, (const char *)&packet, 18, 28);
#endif
}

#endif

// MESSAGE POSITION_CONTROL_SETPOINT UNPACKING


/**
 * @brief Get field id from position_control_setpoint message
 *
 * @return ID of waypoint, 0 for plain position
 */
static inline uint16_t mavlink_msg_position_control_setpoint_get_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field x from position_control_setpoint message
 *
 * @return x position
 */
static inline float mavlink_msg_position_control_setpoint_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from position_control_setpoint message
 *
 * @return y position
 */
static inline float mavlink_msg_position_control_setpoint_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from position_control_setpoint message
 *
 * @return z position
 */
static inline float mavlink_msg_position_control_setpoint_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from position_control_setpoint message
 *
 * @return yaw orientation in radians, 0 = NORTH
 */
static inline float mavlink_msg_position_control_setpoint_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a position_control_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param position_control_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_position_control_setpoint_decode(const mavlink_message_t* msg, mavlink_position_control_setpoint_t* position_control_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP
	position_control_setpoint->x = mavlink_msg_position_control_setpoint_get_x(msg);
	position_control_setpoint->y = mavlink_msg_position_control_setpoint_get_y(msg);
	position_control_setpoint->z = mavlink_msg_position_control_setpoint_get_z(msg);
	position_control_setpoint->yaw = mavlink_msg_position_control_setpoint_get_yaw(msg);
	position_control_setpoint->id = mavlink_msg_position_control_setpoint_get_id(msg);
#else
	memcpy(position_control_setpoint, _MAV_PAYLOAD(msg), 18);
#endif
}
