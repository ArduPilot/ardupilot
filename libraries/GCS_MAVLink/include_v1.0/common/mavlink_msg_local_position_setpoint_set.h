// MESSAGE LOCAL_POSITION_SETPOINT_SET PACKING

#define MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET 50

typedef struct __mavlink_local_position_setpoint_set_t
{
 float x; ///< x position
 float y; ///< y position
 float z; ///< z position
 float yaw; ///< Desired yaw angle
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_local_position_setpoint_set_t;

#define MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET_LEN 18
#define MAVLINK_MSG_ID_50_LEN 18



#define MAVLINK_MESSAGE_INFO_LOCAL_POSITION_SETPOINT_SET { \
	"LOCAL_POSITION_SETPOINT_SET", \
	6, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_local_position_setpoint_set_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_local_position_setpoint_set_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_local_position_setpoint_set_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_local_position_setpoint_set_t, yaw) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_local_position_setpoint_set_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_local_position_setpoint_set_t, target_component) }, \
         } \
}


/**
 * @brief Pack a local_position_setpoint_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_setpoint_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_local_position_setpoint_set_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET;
	return mavlink_finalize_message(msg, system_id, component_id, 18, 73);
}

/**
 * @brief Pack a local_position_setpoint_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_setpoint_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,float x,float y,float z,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_local_position_setpoint_set_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 73);
}

/**
 * @brief Encode a local_position_setpoint_set struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_position_setpoint_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_position_setpoint_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_local_position_setpoint_set_t* local_position_setpoint_set)
{
	return mavlink_msg_local_position_setpoint_set_pack(system_id, component_id, msg, local_position_setpoint_set->target_system, local_position_setpoint_set->target_component, local_position_setpoint_set->x, local_position_setpoint_set->y, local_position_setpoint_set->z, local_position_setpoint_set->yaw);
}

/**
 * @brief Send a local_position_setpoint_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_position_setpoint_set_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET, buf, 18, 73);
#else
	mavlink_local_position_setpoint_set_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET, (const char *)&packet, 18, 73);
#endif
}

#endif

// MESSAGE LOCAL_POSITION_SETPOINT_SET UNPACKING


/**
 * @brief Get field target_system from local_position_setpoint_set message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_local_position_setpoint_set_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from local_position_setpoint_set message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_local_position_setpoint_set_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field x from local_position_setpoint_set message
 *
 * @return x position
 */
static inline float mavlink_msg_local_position_setpoint_set_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from local_position_setpoint_set message
 *
 * @return y position
 */
static inline float mavlink_msg_local_position_setpoint_set_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from local_position_setpoint_set message
 *
 * @return z position
 */
static inline float mavlink_msg_local_position_setpoint_set_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from local_position_setpoint_set message
 *
 * @return Desired yaw angle
 */
static inline float mavlink_msg_local_position_setpoint_set_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a local_position_setpoint_set message into a struct
 *
 * @param msg The message to decode
 * @param local_position_setpoint_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_local_position_setpoint_set_decode(const mavlink_message_t* msg, mavlink_local_position_setpoint_set_t* local_position_setpoint_set)
{
#if MAVLINK_NEED_BYTE_SWAP
	local_position_setpoint_set->x = mavlink_msg_local_position_setpoint_set_get_x(msg);
	local_position_setpoint_set->y = mavlink_msg_local_position_setpoint_set_get_y(msg);
	local_position_setpoint_set->z = mavlink_msg_local_position_setpoint_set_get_z(msg);
	local_position_setpoint_set->yaw = mavlink_msg_local_position_setpoint_set_get_yaw(msg);
	local_position_setpoint_set->target_system = mavlink_msg_local_position_setpoint_set_get_target_system(msg);
	local_position_setpoint_set->target_component = mavlink_msg_local_position_setpoint_set_get_target_component(msg);
#else
	memcpy(local_position_setpoint_set, _MAV_PAYLOAD(msg), 18);
#endif
}
