// MESSAGE WATCHDOG_PROCESS_INFO PACKING

#define MAVLINK_MSG_ID_WATCHDOG_PROCESS_INFO 151

typedef struct __mavlink_watchdog_process_info_t 
{
	uint16_t watchdog_id; ///< Watchdog ID
	uint16_t process_id; ///< Process ID
	int8_t name[100]; ///< Process name
	int8_t arguments[147]; ///< Process arguments
	int32_t timeout; ///< Timeout (seconds)

} mavlink_watchdog_process_info_t;

#define MAVLINK_MSG_WATCHDOG_PROCESS_INFO_FIELD_NAME_LEN 100
#define MAVLINK_MSG_WATCHDOG_PROCESS_INFO_FIELD_ARGUMENTS_LEN 147


/**
 * @brief Send a watchdog_process_info message
 *
 * @param watchdog_id Watchdog ID
 * @param process_id Process ID
 * @param name Process name
 * @param arguments Process arguments
 * @param timeout Timeout (seconds)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_watchdog_process_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint16_t watchdog_id, uint16_t process_id, const int8_t* name, const int8_t* arguments, int32_t timeout)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_WATCHDOG_PROCESS_INFO;

	i += put_uint16_t_by_index(watchdog_id, i, msg->payload); //Watchdog ID
	i += put_uint16_t_by_index(process_id, i, msg->payload); //Process ID
	i += put_array_by_index(name, 100, i, msg->payload); //Process name
	i += put_array_by_index(arguments, 147, i, msg->payload); //Process arguments
	i += put_int32_t_by_index(timeout, i, msg->payload); //Timeout (seconds)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_watchdog_process_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_watchdog_process_info_t* watchdog_process_info)
{
	return mavlink_msg_watchdog_process_info_pack(system_id, component_id, msg, watchdog_process_info->watchdog_id, watchdog_process_info->process_id, watchdog_process_info->name, watchdog_process_info->arguments, watchdog_process_info->timeout);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_watchdog_process_info_send(mavlink_channel_t chan, uint16_t watchdog_id, uint16_t process_id, const int8_t* name, const int8_t* arguments, int32_t timeout)
{
	mavlink_message_t msg;
	mavlink_msg_watchdog_process_info_pack(mavlink_system.sysid, mavlink_system.compid, &msg, watchdog_id, process_id, name, arguments, timeout);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE WATCHDOG_PROCESS_INFO UNPACKING

/**
 * @brief Get field watchdog_id from watchdog_process_info message
 *
 * @return Watchdog ID
 */
static inline uint16_t mavlink_msg_watchdog_process_info_get_watchdog_id(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload)[0];
	r.b[0] = (msg->payload)[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field process_id from watchdog_process_info message
 *
 * @return Process ID
 */
static inline uint16_t mavlink_msg_watchdog_process_info_get_process_id(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field name from watchdog_process_info message
 *
 * @return Process name
 */
static inline uint16_t mavlink_msg_watchdog_process_info_get_name(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint16_t)+sizeof(uint16_t), 100);
	return 100;
}

/**
 * @brief Get field arguments from watchdog_process_info message
 *
 * @return Process arguments
 */
static inline uint16_t mavlink_msg_watchdog_process_info_get_arguments(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+100, 147);
	return 147;
}

/**
 * @brief Get field timeout from watchdog_process_info message
 *
 * @return Timeout (seconds)
 */
static inline int32_t mavlink_msg_watchdog_process_info_get_timeout(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+100+147)[0];
	r.b[2] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+100+147)[1];
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+100+147)[2];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+100+147)[3];
	return (int32_t)r.i;
}

static inline void mavlink_msg_watchdog_process_info_decode(const mavlink_message_t* msg, mavlink_watchdog_process_info_t* watchdog_process_info)
{
	watchdog_process_info->watchdog_id = mavlink_msg_watchdog_process_info_get_watchdog_id(msg);
	watchdog_process_info->process_id = mavlink_msg_watchdog_process_info_get_process_id(msg);
	mavlink_msg_watchdog_process_info_get_name(msg, watchdog_process_info->name);
	mavlink_msg_watchdog_process_info_get_arguments(msg, watchdog_process_info->arguments);
	watchdog_process_info->timeout = mavlink_msg_watchdog_process_info_get_timeout(msg);
}
