// MESSAGE GPS_SET_GLOBAL_ORIGIN PACKING

#define MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN 48

typedef struct __mavlink_gps_set_global_origin_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	uint32_t global_x; ///< global x position * 1E7
	uint32_t global_y; ///< global y position * 1E7
	uint32_t global_z; ///< global z position * 1000
	float local_x; ///< local x position that matches the global x position
	float local_y; ///< local y position that matches the global y position
	float local_z; ///< local z position that matches the global z position

} mavlink_gps_set_global_origin_t;



/**
 * @brief Pack a gps_set_global_origin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param global_x global x position * 1E7
 * @param global_y global y position * 1E7
 * @param global_z global z position * 1000
 * @param local_x local x position that matches the global x position
 * @param local_y local y position that matches the global y position
 * @param local_z local z position that matches the global z position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_set_global_origin_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint32_t global_x, uint32_t global_y, uint32_t global_z, float local_x, float local_y, float local_z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_uint32_t_by_index(global_x, i, msg->payload); // global x position * 1E7
	i += put_uint32_t_by_index(global_y, i, msg->payload); // global y position * 1E7
	i += put_uint32_t_by_index(global_z, i, msg->payload); // global z position * 1000
	i += put_float_by_index(local_x, i, msg->payload); // local x position that matches the global x position
	i += put_float_by_index(local_y, i, msg->payload); // local y position that matches the global y position
	i += put_float_by_index(local_z, i, msg->payload); // local z position that matches the global z position

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a gps_set_global_origin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param global_x global x position * 1E7
 * @param global_y global y position * 1E7
 * @param global_z global z position * 1000
 * @param local_x local x position that matches the global x position
 * @param local_y local y position that matches the global y position
 * @param local_z local z position that matches the global z position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_set_global_origin_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint32_t global_x, uint32_t global_y, uint32_t global_z, float local_x, float local_y, float local_z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_uint32_t_by_index(global_x, i, msg->payload); // global x position * 1E7
	i += put_uint32_t_by_index(global_y, i, msg->payload); // global y position * 1E7
	i += put_uint32_t_by_index(global_z, i, msg->payload); // global z position * 1000
	i += put_float_by_index(local_x, i, msg->payload); // local x position that matches the global x position
	i += put_float_by_index(local_y, i, msg->payload); // local y position that matches the global y position
	i += put_float_by_index(local_z, i, msg->payload); // local z position that matches the global z position

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a gps_set_global_origin struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_set_global_origin C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_set_global_origin_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_set_global_origin_t* gps_set_global_origin)
{
	return mavlink_msg_gps_set_global_origin_pack(system_id, component_id, msg, gps_set_global_origin->target_system, gps_set_global_origin->target_component, gps_set_global_origin->global_x, gps_set_global_origin->global_y, gps_set_global_origin->global_z, gps_set_global_origin->local_x, gps_set_global_origin->local_y, gps_set_global_origin->local_z);
}

/**
 * @brief Send a gps_set_global_origin message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param global_x global x position * 1E7
 * @param global_y global y position * 1E7
 * @param global_z global z position * 1000
 * @param local_x local x position that matches the global x position
 * @param local_y local y position that matches the global y position
 * @param local_z local z position that matches the global z position
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_set_global_origin_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t global_x, uint32_t global_y, uint32_t global_z, float local_x, float local_y, float local_z)
{
	mavlink_message_t msg;
	mavlink_msg_gps_set_global_origin_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, global_x, global_y, global_z, local_x, local_y, local_z);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE GPS_SET_GLOBAL_ORIGIN UNPACKING

/**
 * @brief Get field target_system from gps_set_global_origin message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gps_set_global_origin_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from gps_set_global_origin message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gps_set_global_origin_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field global_x from gps_set_global_origin message
 *
 * @return global x position * 1E7
 */
static inline uint32_t mavlink_msg_gps_set_global_origin_get_global_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field global_y from gps_set_global_origin message
 *
 * @return global y position * 1E7
 */
static inline uint32_t mavlink_msg_gps_set_global_origin_get_global_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field global_z from gps_set_global_origin message
 *
 * @return global z position * 1000
 */
static inline uint32_t mavlink_msg_gps_set_global_origin_get_global_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field local_x from gps_set_global_origin message
 *
 * @return local x position that matches the global x position
 */
static inline float mavlink_msg_gps_set_global_origin_get_local_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field local_y from gps_set_global_origin message
 *
 * @return local y position that matches the global y position
 */
static inline float mavlink_msg_gps_set_global_origin_get_local_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field local_z from gps_set_global_origin message
 *
 * @return local z position that matches the global z position
 */
static inline float mavlink_msg_gps_set_global_origin_get_local_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a gps_set_global_origin message into a struct
 *
 * @param msg The message to decode
 * @param gps_set_global_origin C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_set_global_origin_decode(const mavlink_message_t* msg, mavlink_gps_set_global_origin_t* gps_set_global_origin)
{
	gps_set_global_origin->target_system = mavlink_msg_gps_set_global_origin_get_target_system(msg);
	gps_set_global_origin->target_component = mavlink_msg_gps_set_global_origin_get_target_component(msg);
	gps_set_global_origin->global_x = mavlink_msg_gps_set_global_origin_get_global_x(msg);
	gps_set_global_origin->global_y = mavlink_msg_gps_set_global_origin_get_global_y(msg);
	gps_set_global_origin->global_z = mavlink_msg_gps_set_global_origin_get_global_z(msg);
	gps_set_global_origin->local_x = mavlink_msg_gps_set_global_origin_get_local_x(msg);
	gps_set_global_origin->local_y = mavlink_msg_gps_set_global_origin_get_local_y(msg);
	gps_set_global_origin->local_z = mavlink_msg_gps_set_global_origin_get_local_z(msg);
}
