// MESSAGE WAYPOINT_SET_GLOBAL_REFERENCE PACKING

#define MAVLINK_MSG_ID_WAYPOINT_SET_GLOBAL_REFERENCE 48

typedef struct __mavlink_waypoint_set_global_reference_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	float global_x; ///< global x position
	float global_y; ///< global y position
	float global_z; ///< global z position
	float global_yaw; ///< global yaw orientation in radians, 0 = NORTH
	float local_x; ///< local x position that matches the global x position
	float local_y; ///< local y position that matches the global y position
	float local_z; ///< local z position that matches the global z position
	float local_yaw; ///< local yaw that matches the global yaw orientation

} mavlink_waypoint_set_global_reference_t;



/**
 * @brief Pack a waypoint_set_global_reference message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param global_x global x position
 * @param global_y global y position
 * @param global_z global z position
 * @param global_yaw global yaw orientation in radians, 0 = NORTH
 * @param local_x local x position that matches the global x position
 * @param local_y local y position that matches the global y position
 * @param local_z local z position that matches the global z position
 * @param local_yaw local yaw that matches the global yaw orientation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_set_global_reference_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, float global_x, float global_y, float global_z, float global_yaw, float local_x, float local_y, float local_z, float local_yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_SET_GLOBAL_REFERENCE;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_float_by_index(global_x, i, msg->payload); // global x position
	i += put_float_by_index(global_y, i, msg->payload); // global y position
	i += put_float_by_index(global_z, i, msg->payload); // global z position
	i += put_float_by_index(global_yaw, i, msg->payload); // global yaw orientation in radians, 0 = NORTH
	i += put_float_by_index(local_x, i, msg->payload); // local x position that matches the global x position
	i += put_float_by_index(local_y, i, msg->payload); // local y position that matches the global y position
	i += put_float_by_index(local_z, i, msg->payload); // local z position that matches the global z position
	i += put_float_by_index(local_yaw, i, msg->payload); // local yaw that matches the global yaw orientation

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a waypoint_set_global_reference message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param global_x global x position
 * @param global_y global y position
 * @param global_z global z position
 * @param global_yaw global yaw orientation in radians, 0 = NORTH
 * @param local_x local x position that matches the global x position
 * @param local_y local y position that matches the global y position
 * @param local_z local z position that matches the global z position
 * @param local_yaw local yaw that matches the global yaw orientation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_set_global_reference_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, float global_x, float global_y, float global_z, float global_yaw, float local_x, float local_y, float local_z, float local_yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_SET_GLOBAL_REFERENCE;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_float_by_index(global_x, i, msg->payload); // global x position
	i += put_float_by_index(global_y, i, msg->payload); // global y position
	i += put_float_by_index(global_z, i, msg->payload); // global z position
	i += put_float_by_index(global_yaw, i, msg->payload); // global yaw orientation in radians, 0 = NORTH
	i += put_float_by_index(local_x, i, msg->payload); // local x position that matches the global x position
	i += put_float_by_index(local_y, i, msg->payload); // local y position that matches the global y position
	i += put_float_by_index(local_z, i, msg->payload); // local z position that matches the global z position
	i += put_float_by_index(local_yaw, i, msg->payload); // local yaw that matches the global yaw orientation

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a waypoint_set_global_reference struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_set_global_reference C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_set_global_reference_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_set_global_reference_t* waypoint_set_global_reference)
{
	return mavlink_msg_waypoint_set_global_reference_pack(system_id, component_id, msg, waypoint_set_global_reference->target_system, waypoint_set_global_reference->target_component, waypoint_set_global_reference->global_x, waypoint_set_global_reference->global_y, waypoint_set_global_reference->global_z, waypoint_set_global_reference->global_yaw, waypoint_set_global_reference->local_x, waypoint_set_global_reference->local_y, waypoint_set_global_reference->local_z, waypoint_set_global_reference->local_yaw);
}

/**
 * @brief Send a waypoint_set_global_reference message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param global_x global x position
 * @param global_y global y position
 * @param global_z global z position
 * @param global_yaw global yaw orientation in radians, 0 = NORTH
 * @param local_x local x position that matches the global x position
 * @param local_y local y position that matches the global y position
 * @param local_z local z position that matches the global z position
 * @param local_yaw local yaw that matches the global yaw orientation
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_set_global_reference_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float global_x, float global_y, float global_z, float global_yaw, float local_x, float local_y, float local_z, float local_yaw)
{
	mavlink_message_t msg;
	mavlink_msg_waypoint_set_global_reference_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, global_x, global_y, global_z, global_yaw, local_x, local_y, local_z, local_yaw);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE WAYPOINT_SET_GLOBAL_REFERENCE UNPACKING

/**
 * @brief Get field target_system from waypoint_set_global_reference message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_waypoint_set_global_reference_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from waypoint_set_global_reference message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_waypoint_set_global_reference_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field global_x from waypoint_set_global_reference message
 *
 * @return global x position
 */
static inline float mavlink_msg_waypoint_set_global_reference_get_global_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field global_y from waypoint_set_global_reference message
 *
 * @return global y position
 */
static inline float mavlink_msg_waypoint_set_global_reference_get_global_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field global_z from waypoint_set_global_reference message
 *
 * @return global z position
 */
static inline float mavlink_msg_waypoint_set_global_reference_get_global_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field global_yaw from waypoint_set_global_reference message
 *
 * @return global yaw orientation in radians, 0 = NORTH
 */
static inline float mavlink_msg_waypoint_set_global_reference_get_global_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field local_x from waypoint_set_global_reference message
 *
 * @return local x position that matches the global x position
 */
static inline float mavlink_msg_waypoint_set_global_reference_get_local_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field local_y from waypoint_set_global_reference message
 *
 * @return local y position that matches the global y position
 */
static inline float mavlink_msg_waypoint_set_global_reference_get_local_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field local_z from waypoint_set_global_reference message
 *
 * @return local z position that matches the global z position
 */
static inline float mavlink_msg_waypoint_set_global_reference_get_local_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field local_yaw from waypoint_set_global_reference message
 *
 * @return local yaw that matches the global yaw orientation
 */
static inline float mavlink_msg_waypoint_set_global_reference_get_local_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a waypoint_set_global_reference message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_set_global_reference C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_set_global_reference_decode(const mavlink_message_t* msg, mavlink_waypoint_set_global_reference_t* waypoint_set_global_reference)
{
	waypoint_set_global_reference->target_system = mavlink_msg_waypoint_set_global_reference_get_target_system(msg);
	waypoint_set_global_reference->target_component = mavlink_msg_waypoint_set_global_reference_get_target_component(msg);
	waypoint_set_global_reference->global_x = mavlink_msg_waypoint_set_global_reference_get_global_x(msg);
	waypoint_set_global_reference->global_y = mavlink_msg_waypoint_set_global_reference_get_global_y(msg);
	waypoint_set_global_reference->global_z = mavlink_msg_waypoint_set_global_reference_get_global_z(msg);
	waypoint_set_global_reference->global_yaw = mavlink_msg_waypoint_set_global_reference_get_global_yaw(msg);
	waypoint_set_global_reference->local_x = mavlink_msg_waypoint_set_global_reference_get_local_x(msg);
	waypoint_set_global_reference->local_y = mavlink_msg_waypoint_set_global_reference_get_local_y(msg);
	waypoint_set_global_reference->local_z = mavlink_msg_waypoint_set_global_reference_get_local_z(msg);
	waypoint_set_global_reference->local_yaw = mavlink_msg_waypoint_set_global_reference_get_local_yaw(msg);
}
