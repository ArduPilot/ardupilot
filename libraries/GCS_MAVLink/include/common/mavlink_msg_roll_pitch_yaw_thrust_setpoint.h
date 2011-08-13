// MESSAGE ROLL_PITCH_YAW_THRUST_SETPOINT PACKING

#define MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT 57

typedef struct __mavlink_roll_pitch_yaw_thrust_setpoint_t 
{
	uint32_t time_ms; ///< Timestamp in milliseconds since system boot
	float roll; ///< Desired roll angle in radians
	float pitch; ///< Desired pitch angle in radians
	float yaw; ///< Desired yaw angle in radians
	float thrust; ///< Collective thrust, normalized to 0 .. 1

} mavlink_roll_pitch_yaw_thrust_setpoint_t;



/**
 * @brief Pack a roll_pitch_yaw_thrust_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_ms Timestamp in milliseconds since system boot
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time_ms, float roll, float pitch, float yaw, float thrust)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT;

	i += put_uint32_t_by_index(time_ms, i, msg->payload); // Timestamp in milliseconds since system boot
	i += put_float_by_index(roll, i, msg->payload); // Desired roll angle in radians
	i += put_float_by_index(pitch, i, msg->payload); // Desired pitch angle in radians
	i += put_float_by_index(yaw, i, msg->payload); // Desired yaw angle in radians
	i += put_float_by_index(thrust, i, msg->payload); // Collective thrust, normalized to 0 .. 1

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a roll_pitch_yaw_thrust_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_ms Timestamp in milliseconds since system boot
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint32_t time_ms, float roll, float pitch, float yaw, float thrust)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT;

	i += put_uint32_t_by_index(time_ms, i, msg->payload); // Timestamp in milliseconds since system boot
	i += put_float_by_index(roll, i, msg->payload); // Desired roll angle in radians
	i += put_float_by_index(pitch, i, msg->payload); // Desired pitch angle in radians
	i += put_float_by_index(yaw, i, msg->payload); // Desired yaw angle in radians
	i += put_float_by_index(thrust, i, msg->payload); // Collective thrust, normalized to 0 .. 1

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a roll_pitch_yaw_thrust_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_yaw_thrust_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_roll_pitch_yaw_thrust_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_roll_pitch_yaw_thrust_setpoint_t* roll_pitch_yaw_thrust_setpoint)
{
	return mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack(system_id, component_id, msg, roll_pitch_yaw_thrust_setpoint->time_ms, roll_pitch_yaw_thrust_setpoint->roll, roll_pitch_yaw_thrust_setpoint->pitch, roll_pitch_yaw_thrust_setpoint->yaw, roll_pitch_yaw_thrust_setpoint->thrust);
}

/**
 * @brief Send a roll_pitch_yaw_thrust_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param time_ms Timestamp in milliseconds since system boot
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(mavlink_channel_t chan, uint32_t time_ms, float roll, float pitch, float yaw, float thrust)
{
	mavlink_message_t msg;
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, time_ms, roll, pitch, yaw, thrust);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE ROLL_PITCH_YAW_THRUST_SETPOINT UNPACKING

/**
 * @brief Get field time_ms from roll_pitch_yaw_thrust_setpoint message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint32_t mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_time_ms(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field roll from roll_pitch_yaw_thrust_setpoint message
 *
 * @return Desired roll angle in radians
 */
static inline float mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_roll(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint32_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitch from roll_pitch_yaw_thrust_setpoint message
 *
 * @return Desired pitch angle in radians
 */
static inline float mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_pitch(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint32_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint32_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint32_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint32_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from roll_pitch_yaw_thrust_setpoint message
 *
 * @return Desired yaw angle in radians
 */
static inline float mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint32_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint32_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint32_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint32_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field thrust from roll_pitch_yaw_thrust_setpoint message
 *
 * @return Collective thrust, normalized to 0 .. 1
 */
static inline float mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_thrust(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a roll_pitch_yaw_thrust_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param roll_pitch_yaw_thrust_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(const mavlink_message_t* msg, mavlink_roll_pitch_yaw_thrust_setpoint_t* roll_pitch_yaw_thrust_setpoint)
{
	roll_pitch_yaw_thrust_setpoint->time_ms = mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_time_ms(msg);
	roll_pitch_yaw_thrust_setpoint->roll = mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_roll(msg);
	roll_pitch_yaw_thrust_setpoint->pitch = mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_pitch(msg);
	roll_pitch_yaw_thrust_setpoint->yaw = mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_yaw(msg);
	roll_pitch_yaw_thrust_setpoint->thrust = mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_thrust(msg);
}
