// MESSAGE ATTITUDE PACKING

#define MAVLINK_MSG_ID_ATTITUDE_NEW 30
#define MAVLINK_MSG_LENGTH_ATTITUDE_NEW 8+4+4+4+4+4+4

#ifndef MAVLINK_DEACTIVATE_STRUCTS

typedef struct __mavlink_attitude_new_t 
{
	uint64_t usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	float roll; ///< Roll angle (rad)
	float pitch; ///< Pitch angle (rad)
	float yaw; ///< Yaw angle (rad)
	float rollspeed; ///< Roll angular speed (rad/s)
	float pitchspeed; ///< Pitch angular speed (rad/s)
	float yawspeed; ///< Yaw angular speed (rad/s)

} mavlink_attitude_new_t;



/**
 * @brief Pack a attitude_new message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_new_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_ATTITUDE;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	i += put_float_by_index(roll, i, msg->payload); // Roll angle (rad)
	i += put_float_by_index(pitch, i, msg->payload); // Pitch angle (rad)
	i += put_float_by_index(yaw, i, msg->payload); // Yaw angle (rad)
	i += put_float_by_index(rollspeed, i, msg->payload); // Roll angular speed (rad/s)
	i += put_float_by_index(pitchspeed, i, msg->payload); // Pitch angular speed (rad/s)
	i += put_float_by_index(yawspeed, i, msg->payload); // Yaw angular speed (rad/s)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a attitude_new message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_new_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_ATTITUDE;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	i += put_float_by_index(roll, i, msg->payload); // Roll angle (rad)
	i += put_float_by_index(pitch, i, msg->payload); // Pitch angle (rad)
	i += put_float_by_index(yaw, i, msg->payload); // Yaw angle (rad)
	i += put_float_by_index(rollspeed, i, msg->payload); // Roll angular speed (rad/s)
	i += put_float_by_index(pitchspeed, i, msg->payload); // Pitch angular speed (rad/s)
	i += put_float_by_index(yawspeed, i, msg->payload); // Yaw angular speed (rad/s)

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a attitude_new struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_new C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_new_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_new_t* attitude_new)
{
	return mavlink_msg_attitude_new_pack(system_id, component_id, msg, attitude_new->usec, attitude_new->roll, attitude_new->pitch, attitude_new->yaw, attitude_new->rollspeed, attitude_new->pitchspeed, attitude_new->yawspeed);
}

#endif

/**
 * @brief Send a attitude_new message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_new_send(mavlink_channel_t chan, uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
	uint16_t checksum;
	comm_send_ch(chan, MAVLINK_STX);
	crc_init(&checksum);
	mavlink_send_uart_uint8_t(chan, MAVLINK_MSG_LENGTH_ATTITUDE_NEW, &checksum);
	mavlink_send_uart_uint8_t(chan, mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq, &checksum);
	// Increase sequence
    mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq = mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq+1;
	mavlink_send_uart_uint8_t(chan, mavlink_system.sysid, &checksum);
	mavlink_send_uart_uint8_t(chan, mavlink_system.compid, &checksum);
	mavlink_send_uart_uint8_t(chan, MAVLINK_MSG_ID_ATTITUDE_NEW, &checksum);
	mavlink_send_uart_uint64_t(chan, usec, &checksum);
	mavlink_send_uart_float(chan, roll, &checksum);
	mavlink_send_uart_float(chan, pitch, &checksum);
	mavlink_send_uart_float(chan, yaw, &checksum);
	mavlink_send_uart_float(chan, rollspeed, &checksum);
	mavlink_send_uart_float(chan, pitchspeed, &checksum);
	mavlink_send_uart_float(chan, yawspeed, &checksum);
	// Checksum complete, send it
	comm_send_ch(chan, (uint8_t)(checksum & 0xFF));
	comm_send_ch(chan, (uint8_t)(checksum >> 8));
}

#endif
// MESSAGE ATTITUDE UNPACKING

/**
 * @brief Get field usec from attitude_new message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_attitude_new_get_usec(const mavlink_message_t* msg)
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
 * @brief Get field roll from attitude_new message
 *
 * @return Roll angle (rad)
 */
static inline float mavlink_msg_attitude_new_get_roll(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitch from attitude_new message
 *
 * @return Pitch angle (rad)
 */
static inline float mavlink_msg_attitude_new_get_pitch(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from attitude_new message
 *
 * @return Yaw angle (rad)
 */
static inline float mavlink_msg_attitude_new_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field rollspeed from attitude_new message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_new_get_rollspeed(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitchspeed from attitude_new message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_new_get_pitchspeed(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yawspeed from attitude_new message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_attitude_new_get_yawspeed(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

#ifndef MAVLINK_DEACTIVATE_STRUCTS

/**
 * @brief Decode a attitude_new message into a struct
 *
 * @param msg The message to decode
 * @param attitude_new C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_new_decode(const mavlink_message_t* msg, mavlink_attitude_new_t* attitude_new)
{
	attitude_new->usec = mavlink_msg_attitude_new_get_usec(msg);
	attitude_new->roll = mavlink_msg_attitude_new_get_roll(msg);
	attitude_new->pitch = mavlink_msg_attitude_new_get_pitch(msg);
	attitude_new->yaw = mavlink_msg_attitude_new_get_yaw(msg);
	attitude_new->rollspeed = mavlink_msg_attitude_new_get_rollspeed(msg);
	attitude_new->pitchspeed = mavlink_msg_attitude_new_get_pitchspeed(msg);
	attitude_new->yawspeed = mavlink_msg_attitude_new_get_yawspeed(msg);
}

#endif
