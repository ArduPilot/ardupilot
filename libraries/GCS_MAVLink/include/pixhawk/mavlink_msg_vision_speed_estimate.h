// MESSAGE VISION_SPEED_ESTIMATE PACKING

#define MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE 113

typedef struct __mavlink_vision_speed_estimate_t 
{
	uint64_t usec; ///< Timestamp (milliseconds)
	float x; ///< Global X speed
	float y; ///< Global Y speed
	float z; ///< Global Z speed

} mavlink_vision_speed_estimate_t;



/**
 * @brief Pack a vision_speed_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (milliseconds)
 * @param x Global X speed
 * @param y Global Y speed
 * @param z Global Z speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_speed_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float x, float y, float z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (milliseconds)
	i += put_float_by_index(x, i, msg->payload); // Global X speed
	i += put_float_by_index(y, i, msg->payload); // Global Y speed
	i += put_float_by_index(z, i, msg->payload); // Global Z speed

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a vision_speed_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (milliseconds)
 * @param x Global X speed
 * @param y Global Y speed
 * @param z Global Z speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_speed_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t usec, float x, float y, float z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (milliseconds)
	i += put_float_by_index(x, i, msg->payload); // Global X speed
	i += put_float_by_index(y, i, msg->payload); // Global Y speed
	i += put_float_by_index(z, i, msg->payload); // Global Z speed

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a vision_speed_estimate struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_speed_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_speed_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_speed_estimate_t* vision_speed_estimate)
{
	return mavlink_msg_vision_speed_estimate_pack(system_id, component_id, msg, vision_speed_estimate->usec, vision_speed_estimate->x, vision_speed_estimate->y, vision_speed_estimate->z);
}

/**
 * @brief Send a vision_speed_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (milliseconds)
 * @param x Global X speed
 * @param y Global Y speed
 * @param z Global Z speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_speed_estimate_send(mavlink_channel_t chan, uint64_t usec, float x, float y, float z)
{
	mavlink_message_t msg;
	mavlink_msg_vision_speed_estimate_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, usec, x, y, z);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE VISION_SPEED_ESTIMATE UNPACKING

/**
 * @brief Get field usec from vision_speed_estimate message
 *
 * @return Timestamp (milliseconds)
 */
static inline uint64_t mavlink_msg_vision_speed_estimate_get_usec(const mavlink_message_t* msg)
{
	generic_64bit r;
	r.b[7] = (msg->payload)[0];
	r.b[6] = (msg->payload)[1];
	r.b[5] = (msg->payload)[2];
	r.b[4] = (msg->payload)[3];
	r.b[3] = (msg->payload)[4];
	r.b[2] = (msg->payload)[5];
	r.b[1] = (msg->payload)[6];
	r.b[0] = (msg->payload)[7];
	return (uint64_t)r.ll;
}

/**
 * @brief Get field x from vision_speed_estimate message
 *
 * @return Global X speed
 */
static inline float mavlink_msg_vision_speed_estimate_get_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field y from vision_speed_estimate message
 *
 * @return Global Y speed
 */
static inline float mavlink_msg_vision_speed_estimate_get_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z from vision_speed_estimate message
 *
 * @return Global Z speed
 */
static inline float mavlink_msg_vision_speed_estimate_get_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a vision_speed_estimate message into a struct
 *
 * @param msg The message to decode
 * @param vision_speed_estimate C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_speed_estimate_decode(const mavlink_message_t* msg, mavlink_vision_speed_estimate_t* vision_speed_estimate)
{
	vision_speed_estimate->usec = mavlink_msg_vision_speed_estimate_get_usec(msg);
	vision_speed_estimate->x = mavlink_msg_vision_speed_estimate_get_x(msg);
	vision_speed_estimate->y = mavlink_msg_vision_speed_estimate_get_y(msg);
	vision_speed_estimate->z = mavlink_msg_vision_speed_estimate_get_z(msg);
}
