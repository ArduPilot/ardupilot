// MESSAGE POSITION_CONTROL_SETPOINT PACKING

#define MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT 121

typedef struct __mavlink_position_control_setpoint_t 
{
	uint16_t id; ///< ID of waypoint, 0 for plain position
	float x; ///< x position
	float y; ///< y position
	float z; ///< z position
	float yaw; ///< yaw orientation in radians, 0 = NORTH

} mavlink_position_control_setpoint_t;



/**
 * @brief Send a position_control_setpoint message
 *
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_position_control_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint16_t id, float x, float y, float z, float yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT;

	i += put_uint16_t_by_index(id, i, msg->payload); //ID of waypoint, 0 for plain position
	i += put_float_by_index(x, i, msg->payload); //x position
	i += put_float_by_index(y, i, msg->payload); //y position
	i += put_float_by_index(z, i, msg->payload); //z position
	i += put_float_by_index(yaw, i, msg->payload); //yaw orientation in radians, 0 = NORTH

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_position_control_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_position_control_setpoint_t* position_control_setpoint)
{
	return mavlink_msg_position_control_setpoint_pack(system_id, component_id, msg, position_control_setpoint->id, position_control_setpoint->x, position_control_setpoint->y, position_control_setpoint->z, position_control_setpoint->yaw);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_position_control_setpoint_send(mavlink_channel_t chan, uint16_t id, float x, float y, float z, float yaw)
{
	mavlink_message_t msg;
	mavlink_msg_position_control_setpoint_pack(mavlink_system.sysid, mavlink_system.compid, &msg, id, x, y, z, yaw);
	mavlink_send_uart(chan, &msg);
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
	generic_16bit r;
	r.b[1] = (msg->payload)[0];
	r.b[0] = (msg->payload)[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field x from position_control_setpoint message
 *
 * @return x position
 */
static inline float mavlink_msg_position_control_setpoint_get_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint16_t))[0];
	r.b[2] = (msg->payload+sizeof(uint16_t))[1];
	r.b[1] = (msg->payload+sizeof(uint16_t))[2];
	r.b[0] = (msg->payload+sizeof(uint16_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field y from position_control_setpoint message
 *
 * @return y position
 */
static inline float mavlink_msg_position_control_setpoint_get_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint16_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint16_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z from position_control_setpoint message
 *
 * @return z position
 */
static inline float mavlink_msg_position_control_setpoint_get_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint16_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint16_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from position_control_setpoint message
 *
 * @return yaw orientation in radians, 0 = NORTH
 */
static inline float mavlink_msg_position_control_setpoint_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

static inline void mavlink_msg_position_control_setpoint_decode(const mavlink_message_t* msg, mavlink_position_control_setpoint_t* position_control_setpoint)
{
	position_control_setpoint->id = mavlink_msg_position_control_setpoint_get_id(msg);
	position_control_setpoint->x = mavlink_msg_position_control_setpoint_get_x(msg);
	position_control_setpoint->y = mavlink_msg_position_control_setpoint_get_y(msg);
	position_control_setpoint->z = mavlink_msg_position_control_setpoint_get_z(msg);
	position_control_setpoint->yaw = mavlink_msg_position_control_setpoint_get_yaw(msg);
}
