// MESSAGE AIR_DATA PACKING

#define MAVLINK_MSG_ID_AIR_DATA 191

typedef struct __mavlink_air_data_t 
{
	uint8_t target; ///< The system reporting the air data
	float dynamicPressure; ///< Dynamic pressure (Pa)
	float staticPressure; ///< Static pressure (Pa)
	uint16_t temperature; ///< Board temperature

} mavlink_air_data_t;



/**
 * @brief Send a air_data message
 *
 * @param target The system reporting the air data
 * @param dynamicPressure Dynamic pressure (Pa)
 * @param staticPressure Static pressure (Pa)
 * @param temperature Board temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_air_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, float dynamicPressure, float staticPressure, uint16_t temperature)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_AIR_DATA;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system reporting the air data
	i += put_float_by_index(dynamicPressure, i, msg->payload); //Dynamic pressure (Pa)
	i += put_float_by_index(staticPressure, i, msg->payload); //Static pressure (Pa)
	i += put_uint16_t_by_index(temperature, i, msg->payload); //Board temperature

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_air_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_air_data_t* air_data)
{
	return mavlink_msg_air_data_pack(system_id, component_id, msg, air_data->target, air_data->dynamicPressure, air_data->staticPressure, air_data->temperature);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_air_data_send(mavlink_channel_t chan, uint8_t target, float dynamicPressure, float staticPressure, uint16_t temperature)
{
	mavlink_message_t msg;
	mavlink_msg_air_data_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, dynamicPressure, staticPressure, temperature);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE AIR_DATA UNPACKING

/**
 * @brief Get field target from air_data message
 *
 * @return The system reporting the air data
 */
static inline uint8_t mavlink_msg_air_data_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field dynamicPressure from air_data message
 *
 * @return Dynamic pressure (Pa)
 */
static inline float mavlink_msg_air_data_get_dynamicPressure(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field staticPressure from air_data message
 *
 * @return Static pressure (Pa)
 */
static inline float mavlink_msg_air_data_get_staticPressure(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field temperature from air_data message
 *
 * @return Board temperature
 */
static inline uint16_t mavlink_msg_air_data_get_temperature(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	return (uint16_t)r.s;
}

static inline void mavlink_msg_air_data_decode(const mavlink_message_t* msg, mavlink_air_data_t* air_data)
{
	air_data->target = mavlink_msg_air_data_get_target(msg);
	air_data->dynamicPressure = mavlink_msg_air_data_get_dynamicPressure(msg);
	air_data->staticPressure = mavlink_msg_air_data_get_staticPressure(msg);
	air_data->temperature = mavlink_msg_air_data_get_temperature(msg);
}
