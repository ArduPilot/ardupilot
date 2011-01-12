// MESSAGE WAYPOINT PACKING

#define MAVLINK_MSG_ID_WAYPOINT 39

typedef struct __mavlink_waypoint_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	uint16_t seq; ///< Sequence
	uint8_t frame; ///< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	uint8_t action; ///< The scheduled action for the waypoint. see MAV_ACTION in mavlink_types.h
	float orbit; ///< Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
	uint8_t orbit_direction; ///< Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
	float param1; ///< For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
	float param2; ///< For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
	uint8_t current; ///< false:0, true:1
	float x; ///< local: x position, global: longitude
	float y; ///< y position: global: latitude
	float z; ///< z position: global: altitude
	float yaw; ///< yaw orientation in radians, 0 = NORTH
	uint8_t autocontinue; ///< autocontinue to next wp

} mavlink_waypoint_t;



/**
 * @brief Pack a waypoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 * @param action The scheduled action for the waypoint. see MAV_ACTION in mavlink_types.h
 * @param orbit Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
 * @param orbit_direction Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
 * @param param1 For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
 * @param current false:0, true:1
 * @param x local: x position, global: longitude
 * @param y y position: global: latitude
 * @param z z position: global: altitude
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @param autocontinue autocontinue to next wp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint8_t action, float orbit, uint8_t orbit_direction, float param1, float param2, uint8_t current, float x, float y, float z, float yaw, uint8_t autocontinue)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WAYPOINT;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_uint16_t_by_index(seq, i, msg->payload); // Sequence
	i += put_uint8_t_by_index(frame, i, msg->payload); // The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	i += put_uint8_t_by_index(action, i, msg->payload); // The scheduled action for the waypoint. see MAV_ACTION in mavlink_types.h
	i += put_float_by_index(orbit, i, msg->payload); // Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
	i += put_uint8_t_by_index(orbit_direction, i, msg->payload); // Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
	i += put_float_by_index(param1, i, msg->payload); // For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
	i += put_float_by_index(param2, i, msg->payload); // For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
	i += put_uint8_t_by_index(current, i, msg->payload); // false:0, true:1
	i += put_float_by_index(x, i, msg->payload); // local: x position, global: longitude
	i += put_float_by_index(y, i, msg->payload); // y position: global: latitude
	i += put_float_by_index(z, i, msg->payload); // z position: global: altitude
	i += put_float_by_index(yaw, i, msg->payload); // yaw orientation in radians, 0 = NORTH
	i += put_uint8_t_by_index(autocontinue, i, msg->payload); // autocontinue to next wp

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a waypoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 * @param action The scheduled action for the waypoint. see MAV_ACTION in mavlink_types.h
 * @param orbit Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
 * @param orbit_direction Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
 * @param param1 For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
 * @param current false:0, true:1
 * @param x local: x position, global: longitude
 * @param y y position: global: latitude
 * @param z z position: global: altitude
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @param autocontinue autocontinue to next wp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint8_t action, float orbit, uint8_t orbit_direction, float param1, float param2, uint8_t current, float x, float y, float z, float yaw, uint8_t autocontinue)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WAYPOINT;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_uint16_t_by_index(seq, i, msg->payload); // Sequence
	i += put_uint8_t_by_index(frame, i, msg->payload); // The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	i += put_uint8_t_by_index(action, i, msg->payload); // The scheduled action for the waypoint. see MAV_ACTION in mavlink_types.h
	i += put_float_by_index(orbit, i, msg->payload); // Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
	i += put_uint8_t_by_index(orbit_direction, i, msg->payload); // Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
	i += put_float_by_index(param1, i, msg->payload); // For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
	i += put_float_by_index(param2, i, msg->payload); // For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
	i += put_uint8_t_by_index(current, i, msg->payload); // false:0, true:1
	i += put_float_by_index(x, i, msg->payload); // local: x position, global: longitude
	i += put_float_by_index(y, i, msg->payload); // y position: global: latitude
	i += put_float_by_index(z, i, msg->payload); // z position: global: altitude
	i += put_float_by_index(yaw, i, msg->payload); // yaw orientation in radians, 0 = NORTH
	i += put_uint8_t_by_index(autocontinue, i, msg->payload); // autocontinue to next wp

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a waypoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_t* waypoint)
{
	return mavlink_msg_waypoint_pack(system_id, component_id, msg, waypoint->target_system, waypoint->target_component, waypoint->seq, waypoint->frame, waypoint->action, waypoint->orbit, waypoint->orbit_direction, waypoint->param1, waypoint->param2, waypoint->current, waypoint->x, waypoint->y, waypoint->z, waypoint->yaw, waypoint->autocontinue);
}

/**
 * @brief Send a waypoint message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 * @param action The scheduled action for the waypoint. see MAV_ACTION in mavlink_types.h
 * @param orbit Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
 * @param orbit_direction Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
 * @param param1 For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
 * @param current false:0, true:1
 * @param x local: x position, global: longitude
 * @param y y position: global: latitude
 * @param z z position: global: altitude
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @param autocontinue autocontinue to next wp
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint8_t action, float orbit, uint8_t orbit_direction, float param1, float param2, uint8_t current, float x, float y, float z, float yaw, uint8_t autocontinue)
{
	mavlink_message_t msg;
	mavlink_msg_waypoint_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, seq, frame, action, orbit, orbit_direction, param1, param2, current, x, y, z, yaw, autocontinue);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE WAYPOINT UNPACKING

/**
 * @brief Get field target_system from waypoint message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_waypoint_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from waypoint message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_waypoint_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field seq from waypoint message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_waypoint_get_seq(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field frame from waypoint message
 *
 * @return The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 */
static inline uint8_t mavlink_msg_waypoint_get_frame(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[0];
}

/**
 * @brief Get field action from waypoint message
 *
 * @return The scheduled action for the waypoint. see MAV_ACTION in mavlink_types.h
 */
static inline uint8_t mavlink_msg_waypoint_get_action(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field orbit from waypoint message
 *
 * @return Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
 */
static inline float mavlink_msg_waypoint_get_orbit(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field orbit_direction from waypoint message
 *
 * @return Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
 */
static inline uint8_t mavlink_msg_waypoint_get_orbit_direction(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[0];
}

/**
 * @brief Get field param1 from waypoint message
 *
 * @return For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
 */
static inline float mavlink_msg_waypoint_get_param1(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field param2 from waypoint message
 *
 * @return For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
 */
static inline float mavlink_msg_waypoint_get_param2(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field current from waypoint message
 *
 * @return false:0, true:1
 */
static inline uint8_t mavlink_msg_waypoint_get_current(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
}

/**
 * @brief Get field x from waypoint message
 *
 * @return local: x position, global: longitude
 */
static inline float mavlink_msg_waypoint_get_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field y from waypoint message
 *
 * @return y position: global: latitude
 */
static inline float mavlink_msg_waypoint_get_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z from waypoint message
 *
 * @return z position: global: altitude
 */
static inline float mavlink_msg_waypoint_get_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from waypoint message
 *
 * @return yaw orientation in radians, 0 = NORTH
 */
static inline float mavlink_msg_waypoint_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field autocontinue from waypoint message
 *
 * @return autocontinue to next wp
 */
static inline uint8_t mavlink_msg_waypoint_get_autocontinue(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
}

/**
 * @brief Decode a waypoint message into a struct
 *
 * @param msg The message to decode
 * @param waypoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_decode(const mavlink_message_t* msg, mavlink_waypoint_t* waypoint)
{
	waypoint->target_system = mavlink_msg_waypoint_get_target_system(msg);
	waypoint->target_component = mavlink_msg_waypoint_get_target_component(msg);
	waypoint->seq = mavlink_msg_waypoint_get_seq(msg);
	waypoint->frame = mavlink_msg_waypoint_get_frame(msg);
	waypoint->action = mavlink_msg_waypoint_get_action(msg);
	waypoint->orbit = mavlink_msg_waypoint_get_orbit(msg);
	waypoint->orbit_direction = mavlink_msg_waypoint_get_orbit_direction(msg);
	waypoint->param1 = mavlink_msg_waypoint_get_param1(msg);
	waypoint->param2 = mavlink_msg_waypoint_get_param2(msg);
	waypoint->current = mavlink_msg_waypoint_get_current(msg);
	waypoint->x = mavlink_msg_waypoint_get_x(msg);
	waypoint->y = mavlink_msg_waypoint_get_y(msg);
	waypoint->z = mavlink_msg_waypoint_get_z(msg);
	waypoint->yaw = mavlink_msg_waypoint_get_yaw(msg);
	waypoint->autocontinue = mavlink_msg_waypoint_get_autocontinue(msg);
}
