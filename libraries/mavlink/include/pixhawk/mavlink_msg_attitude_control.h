// MESSAGE ATTITUDE_CONTROL PACKING

#define MAVLINK_MSG_ID_ATTITUDE_CONTROL 85

typedef struct __mavlink_attitude_control_t 
{
	uint8_t target; ///< The system to be controlled
	float roll; ///< roll
	float pitch; ///< pitch
	float yaw; ///< yaw
	float thrust; ///< thrust
	uint8_t roll_manual; ///< roll control enabled auto:0, manual:1
	uint8_t pitch_manual; ///< pitch auto:0, manual:1
	uint8_t yaw_manual; ///< yaw auto:0, manual:1
	uint8_t thrust_manual; ///< thrust auto:0, manual:1

} mavlink_attitude_control_t;



/**
 * @brief Send a attitude_control message
 *
 * @param target The system to be controlled
 * @param roll roll
 * @param pitch pitch
 * @param yaw yaw
 * @param thrust thrust
 * @param roll_manual roll control enabled auto:0, manual:1
 * @param pitch_manual pitch auto:0, manual:1
 * @param yaw_manual yaw auto:0, manual:1
 * @param thrust_manual thrust auto:0, manual:1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, float roll, float pitch, float yaw, float thrust, uint8_t roll_manual, uint8_t pitch_manual, uint8_t yaw_manual, uint8_t thrust_manual)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_ATTITUDE_CONTROL;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system to be controlled
	i += put_float_by_index(roll, i, msg->payload); //roll
	i += put_float_by_index(pitch, i, msg->payload); //pitch
	i += put_float_by_index(yaw, i, msg->payload); //yaw
	i += put_float_by_index(thrust, i, msg->payload); //thrust
	i += put_uint8_t_by_index(roll_manual, i, msg->payload); //roll control enabled auto:0, manual:1
	i += put_uint8_t_by_index(pitch_manual, i, msg->payload); //pitch auto:0, manual:1
	i += put_uint8_t_by_index(yaw_manual, i, msg->payload); //yaw auto:0, manual:1
	i += put_uint8_t_by_index(thrust_manual, i, msg->payload); //thrust auto:0, manual:1

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_attitude_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_control_t* attitude_control)
{
	return mavlink_msg_attitude_control_pack(system_id, component_id, msg, attitude_control->target, attitude_control->roll, attitude_control->pitch, attitude_control->yaw, attitude_control->thrust, attitude_control->roll_manual, attitude_control->pitch_manual, attitude_control->yaw_manual, attitude_control->thrust_manual);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_control_send(mavlink_channel_t chan, uint8_t target, float roll, float pitch, float yaw, float thrust, uint8_t roll_manual, uint8_t pitch_manual, uint8_t yaw_manual, uint8_t thrust_manual)
{
	mavlink_message_t msg;
	mavlink_msg_attitude_control_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, roll, pitch, yaw, thrust, roll_manual, pitch_manual, yaw_manual, thrust_manual);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE ATTITUDE_CONTROL UNPACKING

/**
 * @brief Get field target from attitude_control message
 *
 * @return The system to be controlled
 */
static inline uint8_t mavlink_msg_attitude_control_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field roll from attitude_control message
 *
 * @return roll
 */
static inline float mavlink_msg_attitude_control_get_roll(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitch from attitude_control message
 *
 * @return pitch
 */
static inline float mavlink_msg_attitude_control_get_pitch(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from attitude_control message
 *
 * @return yaw
 */
static inline float mavlink_msg_attitude_control_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field thrust from attitude_control message
 *
 * @return thrust
 */
static inline float mavlink_msg_attitude_control_get_thrust(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field roll_manual from attitude_control message
 *
 * @return roll control enabled auto:0, manual:1
 */
static inline uint8_t mavlink_msg_attitude_control_get_roll_manual(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
}

/**
 * @brief Get field pitch_manual from attitude_control message
 *
 * @return pitch auto:0, manual:1
 */
static inline uint8_t mavlink_msg_attitude_control_get_pitch_manual(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field yaw_manual from attitude_control message
 *
 * @return yaw auto:0, manual:1
 */
static inline uint8_t mavlink_msg_attitude_control_get_yaw_manual(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field thrust_manual from attitude_control message
 *
 * @return thrust auto:0, manual:1
 */
static inline uint8_t mavlink_msg_attitude_control_get_thrust_manual(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

static inline void mavlink_msg_attitude_control_decode(const mavlink_message_t* msg, mavlink_attitude_control_t* attitude_control)
{
	attitude_control->target = mavlink_msg_attitude_control_get_target(msg);
	attitude_control->roll = mavlink_msg_attitude_control_get_roll(msg);
	attitude_control->pitch = mavlink_msg_attitude_control_get_pitch(msg);
	attitude_control->yaw = mavlink_msg_attitude_control_get_yaw(msg);
	attitude_control->thrust = mavlink_msg_attitude_control_get_thrust(msg);
	attitude_control->roll_manual = mavlink_msg_attitude_control_get_roll_manual(msg);
	attitude_control->pitch_manual = mavlink_msg_attitude_control_get_pitch_manual(msg);
	attitude_control->yaw_manual = mavlink_msg_attitude_control_get_yaw_manual(msg);
	attitude_control->thrust_manual = mavlink_msg_attitude_control_get_thrust_manual(msg);
}
