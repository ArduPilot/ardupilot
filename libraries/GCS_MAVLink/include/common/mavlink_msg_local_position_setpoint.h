// MESSAGE LOCAL_POSITION_SETPOINT PACKING

#define MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT 51

typedef struct __mavlink_local_position_setpoint_t 
{
	float x; ///< x position 1
	float y; ///< y position 1
	float z; ///< z position 1
	float yaw; ///< x position 2

} mavlink_local_position_setpoint_t;



/**
 * @brief Pack a local_position_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x x position 1
 * @param y y position 1
 * @param z z position 1
 * @param yaw x position 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float x, float y, float z, float yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT;

	i += put_float_by_index(x, i, msg->payload); // x position 1
	i += put_float_by_index(y, i, msg->payload); // y position 1
	i += put_float_by_index(z, i, msg->payload); // z position 1
	i += put_float_by_index(yaw, i, msg->payload); // x position 2

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a local_position_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param x x position 1
 * @param y y position 1
 * @param z z position 1
 * @param yaw x position 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, float x, float y, float z, float yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT;

	i += put_float_by_index(x, i, msg->payload); // x position 1
	i += put_float_by_index(y, i, msg->payload); // y position 1
	i += put_float_by_index(z, i, msg->payload); // z position 1
	i += put_float_by_index(yaw, i, msg->payload); // x position 2

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a local_position_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_position_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_position_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_local_position_setpoint_t* local_position_setpoint)
{
	return mavlink_msg_local_position_setpoint_pack(system_id, component_id, msg, local_position_setpoint->x, local_position_setpoint->y, local_position_setpoint->z, local_position_setpoint->yaw);
}

/**
 * @brief Send a local_position_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param x x position 1
 * @param y y position 1
 * @param z z position 1
 * @param yaw x position 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_position_setpoint_send(mavlink_channel_t chan, float x, float y, float z, float yaw)
{
	mavlink_message_t msg;
	mavlink_msg_local_position_setpoint_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, x, y, z, yaw);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE LOCAL_POSITION_SETPOINT UNPACKING

/**
 * @brief Get field x from local_position_setpoint message
 *
 * @return x position 1
 */
static inline float mavlink_msg_local_position_setpoint_get_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (float)r.f;
}

/**
 * @brief Get field y from local_position_setpoint message
 *
 * @return y position 1
 */
static inline float mavlink_msg_local_position_setpoint_get_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z from local_position_setpoint message
 *
 * @return z position 1
 */
static inline float mavlink_msg_local_position_setpoint_get_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from local_position_setpoint message
 *
 * @return x position 2
 */
static inline float mavlink_msg_local_position_setpoint_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a local_position_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param local_position_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_local_position_setpoint_decode(const mavlink_message_t* msg, mavlink_local_position_setpoint_t* local_position_setpoint)
{
	local_position_setpoint->x = mavlink_msg_local_position_setpoint_get_x(msg);
	local_position_setpoint->y = mavlink_msg_local_position_setpoint_get_y(msg);
	local_position_setpoint->z = mavlink_msg_local_position_setpoint_get_z(msg);
	local_position_setpoint->yaw = mavlink_msg_local_position_setpoint_get_yaw(msg);
}
