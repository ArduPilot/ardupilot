// MESSAGE CPU_LOAD PACKING

#define MAVLINK_MSG_ID_CPU_LOAD 190

typedef struct __mavlink_cpu_load_t 
{
	uint8_t target; ///< The system reporting the CPU load
	uint8_t sensLoad; ///< Sensor DSC Load
	uint8_t ctrlLoad; ///< Control DSC Load
	uint16_t batVolt; ///< Battery Voltage in millivolts

} mavlink_cpu_load_t;



/**
 * @brief Send a cpu_load message
 *
 * @param target The system reporting the CPU load
 * @param sensLoad Sensor DSC Load
 * @param ctrlLoad Control DSC Load
 * @param batVolt Battery Voltage in millivolts
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cpu_load_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, uint8_t sensLoad, uint8_t ctrlLoad, uint16_t batVolt)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_CPU_LOAD;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system reporting the CPU load
	i += put_uint8_t_by_index(sensLoad, i, msg->payload); //Sensor DSC Load
	i += put_uint8_t_by_index(ctrlLoad, i, msg->payload); //Control DSC Load
	i += put_uint16_t_by_index(batVolt, i, msg->payload); //Battery Voltage in millivolts

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_cpu_load_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cpu_load_t* cpu_load)
{
	return mavlink_msg_cpu_load_pack(system_id, component_id, msg, cpu_load->target, cpu_load->sensLoad, cpu_load->ctrlLoad, cpu_load->batVolt);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cpu_load_send(mavlink_channel_t chan, uint8_t target, uint8_t sensLoad, uint8_t ctrlLoad, uint16_t batVolt)
{
	mavlink_message_t msg;
	mavlink_msg_cpu_load_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, sensLoad, ctrlLoad, batVolt);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE CPU_LOAD UNPACKING

/**
 * @brief Get field target from cpu_load message
 *
 * @return The system reporting the CPU load
 */
static inline uint8_t mavlink_msg_cpu_load_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field sensLoad from cpu_load message
 *
 * @return Sensor DSC Load
 */
static inline uint8_t mavlink_msg_cpu_load_get_sensLoad(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field ctrlLoad from cpu_load message
 *
 * @return Control DSC Load
 */
static inline uint8_t mavlink_msg_cpu_load_get_ctrlLoad(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field batVolt from cpu_load message
 *
 * @return Battery Voltage in millivolts
 */
static inline uint16_t mavlink_msg_cpu_load_get_batVolt(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

static inline void mavlink_msg_cpu_load_decode(const mavlink_message_t* msg, mavlink_cpu_load_t* cpu_load)
{
	cpu_load->target = mavlink_msg_cpu_load_get_target(msg);
	cpu_load->sensLoad = mavlink_msg_cpu_load_get_sensLoad(msg);
	cpu_load->ctrlLoad = mavlink_msg_cpu_load_get_ctrlLoad(msg);
	cpu_load->batVolt = mavlink_msg_cpu_load_get_batVolt(msg);
}
