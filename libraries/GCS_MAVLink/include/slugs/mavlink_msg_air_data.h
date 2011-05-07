// MESSAGE AIR_DATA PACKING

#define MAVLINK_MSG_ID_AIR_DATA 171

typedef struct __mavlink_air_data_t 
{
	float dynamicPressure; ///< Dynamic pressure (Pa)
	float staticPressure; ///< Static pressure (Pa)
	uint16_t temperature; ///< Board temperature

} mavlink_air_data_t;



/**
 * @brief Pack a air_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param dynamicPressure Dynamic pressure (Pa)
 * @param staticPressure Static pressure (Pa)
 * @param temperature Board temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_air_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float dynamicPressure, float staticPressure, uint16_t temperature)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_AIR_DATA;

	i += put_float_by_index(dynamicPressure, i, msg->payload); // Dynamic pressure (Pa)
	i += put_float_by_index(staticPressure, i, msg->payload); // Static pressure (Pa)
	i += put_uint16_t_by_index(temperature, i, msg->payload); // Board temperature

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a air_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param dynamicPressure Dynamic pressure (Pa)
 * @param staticPressure Static pressure (Pa)
 * @param temperature Board temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_air_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, float dynamicPressure, float staticPressure, uint16_t temperature)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_AIR_DATA;

	i += put_float_by_index(dynamicPressure, i, msg->payload); // Dynamic pressure (Pa)
	i += put_float_by_index(staticPressure, i, msg->payload); // Static pressure (Pa)
	i += put_uint16_t_by_index(temperature, i, msg->payload); // Board temperature

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a air_data struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param air_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_air_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_air_data_t* air_data)
{
	return mavlink_msg_air_data_pack(system_id, component_id, msg, air_data->dynamicPressure, air_data->staticPressure, air_data->temperature);
}

/**
 * @brief Send a air_data message
 * @param chan MAVLink channel to send the message
 *
 * @param dynamicPressure Dynamic pressure (Pa)
 * @param staticPressure Static pressure (Pa)
 * @param temperature Board temperature
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_air_data_send(mavlink_channel_t chan, float dynamicPressure, float staticPressure, uint16_t temperature)
{
	mavlink_message_t msg;
	mavlink_msg_air_data_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, dynamicPressure, staticPressure, temperature);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE AIR_DATA UNPACKING

/**
 * @brief Get field dynamicPressure from air_data message
 *
 * @return Dynamic pressure (Pa)
 */
static inline float mavlink_msg_air_data_get_dynamicPressure(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
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
	r.b[3] = (msg->payload+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float))[3];
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
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float))[0];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Decode a air_data message into a struct
 *
 * @param msg The message to decode
 * @param air_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_air_data_decode(const mavlink_message_t* msg, mavlink_air_data_t* air_data)
{
	air_data->dynamicPressure = mavlink_msg_air_data_get_dynamicPressure(msg);
	air_data->staticPressure = mavlink_msg_air_data_get_staticPressure(msg);
	air_data->temperature = mavlink_msg_air_data_get_temperature(msg);
}
