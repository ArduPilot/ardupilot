// MESSAGE WAYPOINT PACKING

#define MAVLINK_MSG_ID_WAYPOINT 39

typedef struct __mavlink_waypoint_t
{
 float param1; ///< PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
 float param2; ///< PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 float param3; ///< PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 float param4; ///< PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
 float x; ///< PARAM5 / local: x position, global: latitude
 float y; ///< PARAM6 / y position: global: longitude
 float z; ///< PARAM7 / z position: global: altitude
 uint16_t seq; ///< Sequence
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t frame; ///< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 uint8_t command; ///< The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
 uint8_t current; ///< false:0, true:1
 uint8_t autocontinue; ///< autocontinue to next wp
} mavlink_waypoint_t;

#define MAVLINK_MSG_ID_WAYPOINT_LEN 36
#define MAVLINK_MSG_ID_39_LEN 36



#define MAVLINK_MESSAGE_INFO_WAYPOINT { \
	"WAYPOINT", \
	14, \
	{  { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_waypoint_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_waypoint_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_waypoint_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_waypoint_t, param4) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_waypoint_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_waypoint_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_waypoint_t, z) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_waypoint_t, seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_waypoint_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_waypoint_t, target_component) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_waypoint_t, frame) }, \
         { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_waypoint_t, command) }, \
         { "current", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_waypoint_t, current) }, \
         { "autocontinue", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_waypoint_t, autocontinue) }, \
         } \
}


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
static inline uint16_t mavlink_msg_waypoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint8_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, param1);
	_mav_put_float(buf, 4, param2);
	_mav_put_float(buf, 8, param3);
	_mav_put_float(buf, 12, param4);
	_mav_put_float(buf, 16, x);
	_mav_put_float(buf, 20, y);
	_mav_put_float(buf, 24, z);
	_mav_put_uint16_t(buf, 28, seq);
	_mav_put_uint8_t(buf, 30, target_system);
	_mav_put_uint8_t(buf, 31, target_component);
	_mav_put_uint8_t(buf, 32, frame);
	_mav_put_uint8_t(buf, 33, command);
	_mav_put_uint8_t(buf, 34, current);
	_mav_put_uint8_t(buf, 35, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
	mavlink_waypoint_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.command = command;
	packet.current = current;
	packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT;
	return mavlink_finalize_message(msg, system_id, component_id, 36, 205);
}

/**
 * @brief Pack a waypoint message on a channel
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
static inline uint16_t mavlink_msg_waypoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t frame,uint8_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,float x,float y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, param1);
	_mav_put_float(buf, 4, param2);
	_mav_put_float(buf, 8, param3);
	_mav_put_float(buf, 12, param4);
	_mav_put_float(buf, 16, x);
	_mav_put_float(buf, 20, y);
	_mav_put_float(buf, 24, z);
	_mav_put_uint16_t(buf, 28, seq);
	_mav_put_uint8_t(buf, 30, target_system);
	_mav_put_uint8_t(buf, 31, target_component);
	_mav_put_uint8_t(buf, 32, frame);
	_mav_put_uint8_t(buf, 33, command);
	_mav_put_uint8_t(buf, 34, current);
	_mav_put_uint8_t(buf, 35, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
	mavlink_waypoint_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.command = command;
	packet.current = current;
	packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 205);
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
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, param1);
	_mav_put_float(buf, 4, param2);
	_mav_put_float(buf, 8, param3);
	_mav_put_float(buf, 12, param4);
	_mav_put_float(buf, 16, x);
	_mav_put_float(buf, 20, y);
	_mav_put_float(buf, 24, z);
	_mav_put_uint16_t(buf, 28, seq);
	_mav_put_uint8_t(buf, 30, target_system);
	_mav_put_uint8_t(buf, 31, target_component);
	_mav_put_uint8_t(buf, 32, frame);
	_mav_put_uint8_t(buf, 33, command);
	_mav_put_uint8_t(buf, 34, current);
	_mav_put_uint8_t(buf, 35, autocontinue);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT, buf, 36, 205);
#else
	mavlink_waypoint_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.command = command;
	packet.current = current;
	packet.autocontinue = autocontinue;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT, (const char *)&packet, 36, 205);
#endif
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
	return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field target_component from waypoint message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_waypoint_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field seq from waypoint message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_waypoint_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field frame from waypoint message
 *
 * @return The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 */
static inline uint8_t mavlink_msg_waypoint_get_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field command from waypoint message
 *
 * @return The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
 */
static inline uint8_t mavlink_msg_waypoint_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field current from waypoint message
 *
 * @return false:0, true:1
 */
static inline uint8_t mavlink_msg_waypoint_get_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field autocontinue from waypoint message
 *
 * @return autocontinue to next wp
 */
static inline uint8_t mavlink_msg_waypoint_get_autocontinue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field param1 from waypoint message
 *
 * @return PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
 */
static inline float mavlink_msg_waypoint_get_param1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field param2 from waypoint message
 *
 * @return PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 */
static inline float mavlink_msg_waypoint_get_param2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field param3 from waypoint message
 *
 * @return PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 */
static inline float mavlink_msg_waypoint_get_param3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field param4 from waypoint message
 *
 * @return PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
 */
static inline float mavlink_msg_waypoint_get_param4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field x from waypoint message
 *
 * @return PARAM5 / local: x position, global: latitude
 */
static inline float mavlink_msg_waypoint_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field y from waypoint message
 *
 * @return PARAM6 / y position: global: longitude
 */
static inline float mavlink_msg_waypoint_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field z from waypoint message
 *
 * @return PARAM7 / z position: global: altitude
 */
static inline float mavlink_msg_waypoint_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a waypoint message into a struct
 *
 * @param msg The message to decode
 * @param waypoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_decode(const mavlink_message_t* msg, mavlink_waypoint_t* waypoint)
{
#if MAVLINK_NEED_BYTE_SWAP
	waypoint->param1 = mavlink_msg_waypoint_get_param1(msg);
	waypoint->param2 = mavlink_msg_waypoint_get_param2(msg);
	waypoint->param3 = mavlink_msg_waypoint_get_param3(msg);
	waypoint->param4 = mavlink_msg_waypoint_get_param4(msg);
	waypoint->x = mavlink_msg_waypoint_get_x(msg);
	waypoint->y = mavlink_msg_waypoint_get_y(msg);
	waypoint->z = mavlink_msg_waypoint_get_z(msg);
	waypoint->seq = mavlink_msg_waypoint_get_seq(msg);
	waypoint->target_system = mavlink_msg_waypoint_get_target_system(msg);
	waypoint->target_component = mavlink_msg_waypoint_get_target_component(msg);
	waypoint->frame = mavlink_msg_waypoint_get_frame(msg);
	waypoint->command = mavlink_msg_waypoint_get_command(msg);
	waypoint->current = mavlink_msg_waypoint_get_current(msg);
	waypoint->autocontinue = mavlink_msg_waypoint_get_autocontinue(msg);
#else
	memcpy(waypoint, _MAV_PAYLOAD(msg), 36);
#endif
}
