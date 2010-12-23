// MESSAGE PID PACKING

#define MAVLINK_MSG_ID_PID 182

typedef struct __mavlink_pid_t 
{
	uint8_t target; ///< The system setting the PID values
	float pVal; ///< Proportional gain
	float iVal; ///< Integral gain
	float dVal; ///< Derivative gain
	uint8_t idx; ///< PID loop index

} mavlink_pid_t;



/**
 * @brief Send a pid message
 *
 * @param target The system setting the PID values
 * @param pVal Proportional gain
 * @param iVal Integral gain
 * @param dVal Derivative gain
 * @param idx PID loop index
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, float pVal, float iVal, float dVal, uint8_t idx)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_PID;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system setting the PID values
	i += put_float_by_index(pVal, i, msg->payload); //Proportional gain
	i += put_float_by_index(iVal, i, msg->payload); //Integral gain
	i += put_float_by_index(dVal, i, msg->payload); //Derivative gain
	i += put_uint8_t_by_index(idx, i, msg->payload); //PID loop index

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_pid_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pid_t* pid)
{
	return mavlink_msg_pid_pack(system_id, component_id, msg, pid->target, pid->pVal, pid->iVal, pid->dVal, pid->idx);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pid_send(mavlink_channel_t chan, uint8_t target, float pVal, float iVal, float dVal, uint8_t idx)
{
	mavlink_message_t msg;
	mavlink_msg_pid_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, pVal, iVal, dVal, idx);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE PID UNPACKING

/**
 * @brief Get field target from pid message
 *
 * @return The system setting the PID values
 */
static inline uint8_t mavlink_msg_pid_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field pVal from pid message
 *
 * @return Proportional gain
 */
static inline float mavlink_msg_pid_get_pVal(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field iVal from pid message
 *
 * @return Integral gain
 */
static inline float mavlink_msg_pid_get_iVal(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field dVal from pid message
 *
 * @return Derivative gain
 */
static inline float mavlink_msg_pid_get_dVal(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field idx from pid message
 *
 * @return PID loop index
 */
static inline uint8_t mavlink_msg_pid_get_idx(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
}

static inline void mavlink_msg_pid_decode(const mavlink_message_t* msg, mavlink_pid_t* pid)
{
	pid->target = mavlink_msg_pid_get_target(msg);
	pid->pVal = mavlink_msg_pid_get_pVal(msg);
	pid->iVal = mavlink_msg_pid_get_iVal(msg);
	pid->dVal = mavlink_msg_pid_get_dVal(msg);
	pid->idx = mavlink_msg_pid_get_idx(msg);
}
