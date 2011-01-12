// MESSAGE LOCAL_POSITION_SETPOINT_SET PACKING

#define MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET 50

typedef struct __mavlink_local_position_setpoint_set_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	float x; ///< x position 1
	float y; ///< y position 1
	float z; ///< z position 1
	float yaw; ///< x position 2

} mavlink_local_position_setpoint_set_t;



/**
 * @brief Pack a local_position_setpoint_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position 1
 * @param y y position 1
 * @param z z position 1
 * @param yaw x position 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_setpoint_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, float x, float y, float z, float yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_float_by_index(x, i, msg->payload); // x position 1
	i += put_float_by_index(y, i, msg->payload); // y position 1
	i += put_float_by_index(z, i, msg->payload); // z position 1
	i += put_float_by_index(yaw, i, msg->payload); // x position 2

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a local_position_setpoint_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position 1
 * @param y y position 1
 * @param z z position 1
 * @param yaw x position 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_setpoint_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, float x, float y, float z, float yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT_SET;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_float_by_index(x, i, msg->payload); // x position 1
	i += put_float_by_index(y, i, msg->payload); // y position 1
	i += put_float_by_index(z, i, msg->payload); // z position 1
	i += put_float_by_index(yaw, i, msg->payload); // x position 2

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
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
 * @param x x position 1
 * @param y y position 1
 * @param z z position 1
 * @param yaw x position 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_position_setpoint_set_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float x, float y, float z, float yaw)
{
	mavlink_message_t msg;
	mavlink_msg_local_position_setpoint_set_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, x, y, z, yaw);
	mavlink_send_uart(chan, &msg);
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
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from local_position_setpoint_set message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_local_position_setpoint_set_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field x from local_position_setpoint_set message
 *
 * @return x position 1
 */
static inline float mavlink_msg_local_position_setpoint_set_get_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field y from local_position_setpoint_set message
 *
 * @return y position 1
 */
static inline float mavlink_msg_local_position_setpoint_set_get_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z from local_position_setpoint_set message
 *
 * @return z position 1
 */
static inline float mavlink_msg_local_position_setpoint_set_get_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from local_position_setpoint_set message
 *
 * @return x position 2
 */
static inline float mavlink_msg_local_position_setpoint_set_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a local_position_setpoint_set message into a struct
 *
 * @param msg The message to decode
 * @param local_position_setpoint_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_local_position_setpoint_set_decode(const mavlink_message_t* msg, mavlink_local_position_setpoint_set_t* local_position_setpoint_set)
{
	local_position_setpoint_set->target_system = mavlink_msg_local_position_setpoint_set_get_target_system(msg);
	local_position_setpoint_set->target_component = mavlink_msg_local_position_setpoint_set_get_target_component(msg);
	local_position_setpoint_set->x = mavlink_msg_local_position_setpoint_set_get_x(msg);
	local_position_setpoint_set->y = mavlink_msg_local_position_setpoint_set_get_y(msg);
	local_position_setpoint_set->z = mavlink_msg_local_position_setpoint_set_get_z(msg);
	local_position_setpoint_set->yaw = mavlink_msg_local_position_setpoint_set_get_yaw(msg);
}
