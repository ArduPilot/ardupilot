// MESSAGE SET_ROLL_PITCH_YAW PACKING

#define MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW 55

typedef struct __mavlink_set_roll_pitch_yaw_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	float roll; ///< Desired roll angle in radians
	float pitch; ///< Desired pitch angle in radians
	float yaw; ///< Desired yaw angle in radians

} mavlink_set_roll_pitch_yaw_t;



/**
 * @brief Pack a set_roll_pitch_yaw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, float roll, float pitch, float yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_float_by_index(roll, i, msg->payload); // Desired roll angle in radians
	i += put_float_by_index(pitch, i, msg->payload); // Desired pitch angle in radians
	i += put_float_by_index(yaw, i, msg->payload); // Desired yaw angle in radians

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a set_roll_pitch_yaw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, float roll, float pitch, float yaw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_float_by_index(roll, i, msg->payload); // Desired roll angle in radians
	i += put_float_by_index(pitch, i, msg->payload); // Desired pitch angle in radians
	i += put_float_by_index(yaw, i, msg->payload); // Desired yaw angle in radians

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a set_roll_pitch_yaw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_roll_pitch_yaw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_roll_pitch_yaw_t* set_roll_pitch_yaw)
{
	return mavlink_msg_set_roll_pitch_yaw_pack(system_id, component_id, msg, set_roll_pitch_yaw->target_system, set_roll_pitch_yaw->target_component, set_roll_pitch_yaw->roll, set_roll_pitch_yaw->pitch, set_roll_pitch_yaw->yaw);
}

/**
 * @brief Send a set_roll_pitch_yaw message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_roll_pitch_yaw_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float roll, float pitch, float yaw)
{
	mavlink_message_t msg;
	mavlink_msg_set_roll_pitch_yaw_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, roll, pitch, yaw);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SET_ROLL_PITCH_YAW UNPACKING

/**
 * @brief Get field target_system from set_roll_pitch_yaw message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_roll_pitch_yaw_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from set_roll_pitch_yaw message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_roll_pitch_yaw_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field roll from set_roll_pitch_yaw message
 *
 * @return Desired roll angle in radians
 */
static inline float mavlink_msg_set_roll_pitch_yaw_get_roll(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitch from set_roll_pitch_yaw message
 *
 * @return Desired pitch angle in radians
 */
static inline float mavlink_msg_set_roll_pitch_yaw_get_pitch(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from set_roll_pitch_yaw message
 *
 * @return Desired yaw angle in radians
 */
static inline float mavlink_msg_set_roll_pitch_yaw_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a set_roll_pitch_yaw message into a struct
 *
 * @param msg The message to decode
 * @param set_roll_pitch_yaw C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_roll_pitch_yaw_decode(const mavlink_message_t* msg, mavlink_set_roll_pitch_yaw_t* set_roll_pitch_yaw)
{
	set_roll_pitch_yaw->target_system = mavlink_msg_set_roll_pitch_yaw_get_target_system(msg);
	set_roll_pitch_yaw->target_component = mavlink_msg_set_roll_pitch_yaw_get_target_component(msg);
	set_roll_pitch_yaw->roll = mavlink_msg_set_roll_pitch_yaw_get_roll(msg);
	set_roll_pitch_yaw->pitch = mavlink_msg_set_roll_pitch_yaw_get_pitch(msg);
	set_roll_pitch_yaw->yaw = mavlink_msg_set_roll_pitch_yaw_get_yaw(msg);
}
