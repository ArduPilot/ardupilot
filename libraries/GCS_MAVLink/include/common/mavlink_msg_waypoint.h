// MESSAGE WAYPOINT PACKING

#define MAVLINK_MSG_ID_WAYPOINT 39

typedef struct __mavlink_waypoint_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	uint16_t seq; ///< Sequence
	uint8_t frame; ///< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	uint8_t command; ///< The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
	uint8_t current; ///< false:0, true:1
	uint8_t autocontinue; ///< autocontinue to next wp
	float param1; ///< PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
	float param2; ///< PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
	float param3; ///< PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
	float param4; ///< PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
	float x; ///< PARAM5 / local: x position, global: latitude
	float y; ///< PARAM6 / y position: global: longitude
	float z; ///< PARAM7 / z position: global: altitude

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
 * @param command The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint8_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WAYPOINT;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_uint16_t_by_index(seq, i, msg->payload); // Sequence
	i += put_uint8_t_by_index(frame, i, msg->payload); // The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	i += put_uint8_t_by_index(command, i, msg->payload); // The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
	i += put_uint8_t_by_index(current, i, msg->payload); // false:0, true:1
	i += put_uint8_t_by_index(autocontinue, i, msg->payload); // autocontinue to next wp
	i += put_float_by_index(param1, i, msg->payload); // PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
	i += put_float_by_index(param2, i, msg->payload); // PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
	i += put_float_by_index(param3, i, msg->payload); // PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
	i += put_float_by_index(param4, i, msg->payload); // PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
	i += put_float_by_index(x, i, msg->payload); // PARAM5 / local: x position, global: latitude
	i += put_float_by_index(y, i, msg->payload); // PARAM6 / y position: global: longitude
	i += put_float_by_index(z, i, msg->payload); // PARAM7 / z position: global: altitude

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
 * @param command The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint8_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WAYPOINT;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_uint16_t_by_index(seq, i, msg->payload); // Sequence
	i += put_uint8_t_by_index(frame, i, msg->payload); // The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
	i += put_uint8_t_by_index(command, i, msg->payload); // The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
	i += put_uint8_t_by_index(current, i, msg->payload); // false:0, true:1
	i += put_uint8_t_by_index(autocontinue, i, msg->payload); // autocontinue to next wp
	i += put_float_by_index(param1, i, msg->payload); // PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
	i += put_float_by_index(param2, i, msg->payload); // PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
	i += put_float_by_index(param3, i, msg->payload); // PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
	i += put_float_by_index(param4, i, msg->payload); // PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
	i += put_float_by_index(x, i, msg->payload); // PARAM5 / local: x position, global: latitude
	i += put_float_by_index(y, i, msg->payload); // PARAM6 / y position: global: longitude
	i += put_float_by_index(z, i, msg->payload); // PARAM7 / z position: global: altitude

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
	return mavlink_msg_waypoint_pack(system_id, component_id, msg, waypoint->target_system, waypoint->target_component, waypoint->seq, waypoint->frame, waypoint->command, waypoint->current, waypoint->autocontinue, waypoint->param1, waypoint->param2, waypoint->param3, waypoint->param4, waypoint->x, waypoint->y, waypoint->z);
}

/**
 * @brief Send a waypoint message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint8_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
{
	mavlink_message_t msg;
	mavlink_msg_waypoint_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
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
 * @brief Get field command from waypoint message
 *
 * @return The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
 */
static inline uint8_t mavlink_msg_waypoint_get_command(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field current from waypoint message
 *
 * @return false:0, true:1
 */
static inline uint8_t mavlink_msg_waypoint_get_current(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field autocontinue from waypoint message
 *
 * @return autocontinue to next wp
 */
static inline uint8_t mavlink_msg_waypoint_get_autocontinue(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field param1 from waypoint message
 *
 * @return PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
 */
static inline float mavlink_msg_waypoint_get_param1(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field param2 from waypoint message
 *
 * @return PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 */
static inline float mavlink_msg_waypoint_get_param2(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field param3 from waypoint message
 *
 * @return PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 */
static inline float mavlink_msg_waypoint_get_param3(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field param4 from waypoint message
 *
 * @return PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
 */
static inline float mavlink_msg_waypoint_get_param4(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field x from waypoint message
 *
 * @return PARAM5 / local: x position, global: latitude
 */
static inline float mavlink_msg_waypoint_get_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field y from waypoint message
 *
 * @return PARAM6 / y position: global: longitude
 */
static inline float mavlink_msg_waypoint_get_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z from waypoint message
 *
 * @return PARAM7 / z position: global: altitude
 */
static inline float mavlink_msg_waypoint_get_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
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
	waypoint->command = mavlink_msg_waypoint_get_command(msg);
	waypoint->current = mavlink_msg_waypoint_get_current(msg);
	waypoint->autocontinue = mavlink_msg_waypoint_get_autocontinue(msg);
	waypoint->param1 = mavlink_msg_waypoint_get_param1(msg);
	waypoint->param2 = mavlink_msg_waypoint_get_param2(msg);
	waypoint->param3 = mavlink_msg_waypoint_get_param3(msg);
	waypoint->param4 = mavlink_msg_waypoint_get_param4(msg);
	waypoint->x = mavlink_msg_waypoint_get_x(msg);
	waypoint->y = mavlink_msg_waypoint_get_y(msg);
	waypoint->z = mavlink_msg_waypoint_get_z(msg);
}
