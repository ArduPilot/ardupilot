// MESSAGE POSITION_CONTROL_SETPOINT_SET PACKING

#define MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT_SET 120

typedef struct __mavlink_position_control_setpoint_set_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	uint16_t id; ///< ID of waypoint, 0 for plain position
	float x; ///< x position
	float y; ///< y position
	float z; ///< z position
	float yaw; ///< yaw orientation in radians, 0 = NORTH

} mavlink_position_control_setpoint_set_t;



/**
 * @brief Send a position_control_setpoint_set message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_position_control_setpoint_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t id, float x, float y, float z, float yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT_SET;

	i += put_uint8_t_by_index(target_system, i, msg->payload); //System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); //Component ID
	i += put_uint16_t_by_index(id, i, msg->payload); //ID of waypoint, 0 for plain position
	i += put_float_by_index(x, i, msg->payload); //x position
	i += put_float_by_index(y, i, msg->payload); //y position
	i += put_float_by_index(z, i, msg->payload); //z position
	i += put_float_by_index(yaw, i, msg->payload); //yaw orientation in radians, 0 = NORTH

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_position_control_setpoint_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_position_control_setpoint_set_t* position_control_setpoint_set)
{
	return mavlink_msg_position_control_setpoint_set_pack(system_id, component_id, msg, position_control_setpoint_set->target_system, position_control_setpoint_set->target_component, position_control_setpoint_set->id, position_control_setpoint_set->x, position_control_setpoint_set->y, position_control_setpoint_set->z, position_control_setpoint_set->yaw);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_position_control_setpoint_set_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t id, float x, float y, float z, float yaw)
{
	mavlink_message_t msg;
	mavlink_msg_position_control_setpoint_set_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target_system, target_component, id, x, y, z, yaw);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE POSITION_CONTROL_SETPOINT_SET UNPACKING

/**
 * @brief Get field target_system from position_control_setpoint_set message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_position_control_setpoint_set_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from position_control_setpoint_set message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_position_control_setpoint_set_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field id from position_control_setpoint_set message
 *
 * @return ID of waypoint, 0 for plain position
 */
static inline uint16_t mavlink_msg_position_control_setpoint_set_get_id(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field x from position_control_setpoint_set message
 *
 * @return x position
 */
static inline float mavlink_msg_position_control_setpoint_set_get_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field y from position_control_setpoint_set message
 *
 * @return y position
 */
static inline float mavlink_msg_position_control_setpoint_set_get_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z from position_control_setpoint_set message
 *
 * @return z position
 */
static inline float mavlink_msg_position_control_setpoint_set_get_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from position_control_setpoint_set message
 *
 * @return yaw orientation in radians, 0 = NORTH
 */
static inline float mavlink_msg_position_control_setpoint_set_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

static inline void mavlink_msg_position_control_setpoint_set_decode(const mavlink_message_t* msg, mavlink_position_control_setpoint_set_t* position_control_setpoint_set)
{
	position_control_setpoint_set->target_system = mavlink_msg_position_control_setpoint_set_get_target_system(msg);
	position_control_setpoint_set->target_component = mavlink_msg_position_control_setpoint_set_get_target_component(msg);
	position_control_setpoint_set->id = mavlink_msg_position_control_setpoint_set_get_id(msg);
	position_control_setpoint_set->x = mavlink_msg_position_control_setpoint_set_get_x(msg);
	position_control_setpoint_set->y = mavlink_msg_position_control_setpoint_set_get_y(msg);
	position_control_setpoint_set->z = mavlink_msg_position_control_setpoint_set_get_z(msg);
	position_control_setpoint_set->yaw = mavlink_msg_position_control_setpoint_set_get_yaw(msg);
}
