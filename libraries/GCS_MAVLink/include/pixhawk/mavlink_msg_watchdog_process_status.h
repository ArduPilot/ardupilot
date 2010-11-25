// MESSAGE WATCHDOG_PROCESS_STATUS PACKING

#define MAVLINK_MSG_ID_WATCHDOG_PROCESS_STATUS 152

typedef struct __mavlink_watchdog_process_status_t 
{
	uint16_t watchdog_id; ///< Watchdog ID
	uint16_t process_id; ///< Process ID
	uint8_t state; ///< Is running / finished / suspended / crashed
	uint8_t muted; ///< Is muted
	int32_t pid; ///< PID
	uint16_t crashes; ///< Number of crashes

} mavlink_watchdog_process_status_t;



/**
 * @brief Send a watchdog_process_status message
 *
 * @param watchdog_id Watchdog ID
 * @param process_id Process ID
 * @param state Is running / finished / suspended / crashed
 * @param muted Is muted
 * @param pid PID
 * @param crashes Number of crashes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_watchdog_process_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint16_t watchdog_id, uint16_t process_id, uint8_t state, uint8_t muted, int32_t pid, uint16_t crashes)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WATCHDOG_PROCESS_STATUS;

	i += put_uint16_t_by_index(watchdog_id, i, msg->payload); //Watchdog ID
	i += put_uint16_t_by_index(process_id, i, msg->payload); //Process ID
	i += put_uint8_t_by_index(state, i, msg->payload); //Is running / finished / suspended / crashed
	i += put_uint8_t_by_index(muted, i, msg->payload); //Is muted
	i += put_int32_t_by_index(pid, i, msg->payload); //PID
	i += put_uint16_t_by_index(crashes, i, msg->payload); //Number of crashes

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_watchdog_process_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_watchdog_process_status_t* watchdog_process_status)
{
	return mavlink_msg_watchdog_process_status_pack(system_id, component_id, msg, watchdog_process_status->watchdog_id, watchdog_process_status->process_id, watchdog_process_status->state, watchdog_process_status->muted, watchdog_process_status->pid, watchdog_process_status->crashes);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_watchdog_process_status_send(mavlink_channel_t chan, uint16_t watchdog_id, uint16_t process_id, uint8_t state, uint8_t muted, int32_t pid, uint16_t crashes)
{
	mavlink_message_t msg;
	mavlink_msg_watchdog_process_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, watchdog_id, process_id, state, muted, pid, crashes);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE WATCHDOG_PROCESS_STATUS UNPACKING

/**
 * @brief Get field watchdog_id from watchdog_process_status message
 *
 * @return Watchdog ID
 */
static inline uint16_t mavlink_msg_watchdog_process_status_get_watchdog_id(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload)[0];
	r.b[0] = (msg->payload)[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field process_id from watchdog_process_status message
 *
 * @return Process ID
 */
static inline uint16_t mavlink_msg_watchdog_process_status_get_process_id(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field state from watchdog_process_status message
 *
 * @return Is running / finished / suspended / crashed
 */
static inline uint8_t mavlink_msg_watchdog_process_status_get_state(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint16_t)+sizeof(uint16_t))[0];
}

/**
 * @brief Get field muted from watchdog_process_status message
 *
 * @return Is muted
 */
static inline uint8_t mavlink_msg_watchdog_process_status_get_muted(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field pid from watchdog_process_status message
 *
 * @return PID
 */
static inline int32_t mavlink_msg_watchdog_process_status_get_pid(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field crashes from watchdog_process_status message
 *
 * @return Number of crashes
 */
static inline uint16_t mavlink_msg_watchdog_process_status_get_crashes(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int32_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int32_t))[1];
	return (uint16_t)r.s;
}

static inline void mavlink_msg_watchdog_process_status_decode(const mavlink_message_t* msg, mavlink_watchdog_process_status_t* watchdog_process_status)
{
	watchdog_process_status->watchdog_id = mavlink_msg_watchdog_process_status_get_watchdog_id(msg);
	watchdog_process_status->process_id = mavlink_msg_watchdog_process_status_get_process_id(msg);
	watchdog_process_status->state = mavlink_msg_watchdog_process_status_get_state(msg);
	watchdog_process_status->muted = mavlink_msg_watchdog_process_status_get_muted(msg);
	watchdog_process_status->pid = mavlink_msg_watchdog_process_status_get_pid(msg);
	watchdog_process_status->crashes = mavlink_msg_watchdog_process_status_get_crashes(msg);
}
