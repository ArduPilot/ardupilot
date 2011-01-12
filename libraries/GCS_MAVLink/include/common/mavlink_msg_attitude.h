// MESSAGE ATTITUDE PACKING

#define MAVLINK_MSG_ID_ATTITUDE 30

typedef struct __mavlink_attitude_t 
{
	uint64_t usec; ///< Timestamp (microseconds)
	float roll; ///< Roll angle (rad)
	float pitch; ///< Pitch angle (rad)
	float yaw; ///< Yaw angle (rad)
	float rollspeed; ///< Roll angular speed (rad/s)
	float pitchspeed; ///< Pitch angular speed (rad/s)
	float yawspeed; ///< Yaw angular speed (rad/s)

} mavlink_attitude_t;



/**
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_ATTITUDE;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds)
	i += put_float_by_index(roll, i, msg->payload); // Roll angle (rad)
	i += put_float_by_index(pitch, i, msg->payload); // Pitch angle (rad)
	i += put_float_by_index(yaw, i, msg->payload); // Yaw angle (rad)
	i += put_float_by_index(rollspeed, i, msg->payload); // Roll angular speed (rad/s)
	i += put_float_by_index(pitchspeed, i, msg->payload); // Pitch angular speed (rad/s)
	i += put_float_by_index(yawspeed, i, msg->payload); // Yaw angular speed (rad/s)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_ATTITUDE;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds)
	i += put_float_by_index(roll, i, msg->payload); // Roll angle (rad)
	i += put_float_by_index(pitch, i, msg->payload); // Pitch angle (rad)
	i += put_float_by_index(yaw, i, msg->payload); // Yaw angle (rad)
	i += put_float_by_index(rollspeed, i, msg->payload); // Roll angular speed (rad/s)
	i += put_float_by_index(pitchspeed, i, msg->payload); // Pitch angular speed (rad/s)
	i += put_float_by_index(yawspeed, i, msg->payload); // Yaw angular speed (rad/s)

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a attitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_t* attitude)
{
	return mavlink_msg_attitude_pack(system_id, component_id, msg, attitude->usec, attitude->roll, attitude->pitch, attitude->yaw, attitude->rollspeed, attitude->pitchspeed, attitude->yawspeed);
}

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_send(mavlink_channel_t chan, uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
	mavlink_message_t msg;
	mavlink_msg_attitude_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, usec, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE ATTITUDE UNPACKING

/**
 * @brief Get field usec from attitude message
 *
 * @return Timestamp (microseconds)
 */
static inline uint64_t mavlink_msg_attitude_get_usec(const mavlink_message_t* msg)
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
 * @brief Get field roll from attitude message
 *
 * @return Roll angle (rad)
 */
static inline float mavlink_msg_attitude_get_roll(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitch from attitude message
 *
 * @return Pitch angle (rad)
 */
static inline float mavlink_msg_attitude_get_pitch(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from attitude message
 *
 * @return Yaw angle (rad)
 */
static inline float mavlink_msg_attitude_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field rollspeed from attitude message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_get_rollspeed(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitchspeed from attitude message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_get_pitchspeed(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yawspeed from attitude message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_get_yawspeed(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a attitude message into a struct
 *
 * @param msg The message to decode
 * @param attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
{
	attitude->usec = mavlink_msg_attitude_get_usec(msg);
	attitude->roll = mavlink_msg_attitude_get_roll(msg);
	attitude->pitch = mavlink_msg_attitude_get_pitch(msg);
	attitude->yaw = mavlink_msg_attitude_get_yaw(msg);
	attitude->rollspeed = mavlink_msg_attitude_get_rollspeed(msg);
	attitude->pitchspeed = mavlink_msg_attitude_get_pitchspeed(msg);
	attitude->yawspeed = mavlink_msg_attitude_get_yawspeed(msg);
}
