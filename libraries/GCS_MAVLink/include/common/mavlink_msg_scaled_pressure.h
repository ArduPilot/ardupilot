// MESSAGE SCALED_PRESSURE PACKING

#define MAVLINK_MSG_ID_SCALED_PRESSURE 38

typedef struct __mavlink_scaled_pressure_t 
{
	uint64_t usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	float press_abs; ///< Absolute pressure (hectopascal)
	float press_diff; ///< Differential pressure 1 (hectopascal)
	int16_t temperature; ///< Temperature measurement (0.01 degrees celsius)

} mavlink_scaled_pressure_t;



/**
 * @brief Pack a scaled_pressure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_pressure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float press_abs, float press_diff, int16_t temperature)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SCALED_PRESSURE;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	i += put_float_by_index(press_abs, i, msg->payload); // Absolute pressure (hectopascal)
	i += put_float_by_index(press_diff, i, msg->payload); // Differential pressure 1 (hectopascal)
	i += put_int16_t_by_index(temperature, i, msg->payload); // Temperature measurement (0.01 degrees celsius)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a scaled_pressure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_pressure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t usec, float press_abs, float press_diff, int16_t temperature)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SCALED_PRESSURE;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	i += put_float_by_index(press_abs, i, msg->payload); // Absolute pressure (hectopascal)
	i += put_float_by_index(press_diff, i, msg->payload); // Differential pressure 1 (hectopascal)
	i += put_int16_t_by_index(temperature, i, msg->payload); // Temperature measurement (0.01 degrees celsius)

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a scaled_pressure struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scaled_pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_pressure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scaled_pressure_t* scaled_pressure)
{
	return mavlink_msg_scaled_pressure_pack(system_id, component_id, msg, scaled_pressure->usec, scaled_pressure->press_abs, scaled_pressure->press_diff, scaled_pressure->temperature);
}

/**
 * @brief Send a scaled_pressure message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_scaled_pressure_send(mavlink_channel_t chan, uint64_t usec, float press_abs, float press_diff, int16_t temperature)
{
	mavlink_message_t msg;
	mavlink_msg_scaled_pressure_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, usec, press_abs, press_diff, temperature);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SCALED_PRESSURE UNPACKING

/**
 * @brief Get field usec from scaled_pressure message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_scaled_pressure_get_usec(const mavlink_message_t* msg)
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
 * @brief Get field press_abs from scaled_pressure message
 *
 * @return Absolute pressure (hectopascal)
 */
static inline float mavlink_msg_scaled_pressure_get_press_abs(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field press_diff from scaled_pressure message
 *
 * @return Differential pressure 1 (hectopascal)
 */
static inline float mavlink_msg_scaled_pressure_get_press_diff(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field temperature from scaled_pressure message
 *
 * @return Temperature measurement (0.01 degrees celsius)
 */
static inline int16_t mavlink_msg_scaled_pressure_get_temperature(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[1];
	return (int16_t)r.s;
}

/**
 * @brief Decode a scaled_pressure message into a struct
 *
 * @param msg The message to decode
 * @param scaled_pressure C-struct to decode the message contents into
 */
static inline void mavlink_msg_scaled_pressure_decode(const mavlink_message_t* msg, mavlink_scaled_pressure_t* scaled_pressure)
{
	scaled_pressure->usec = mavlink_msg_scaled_pressure_get_usec(msg);
	scaled_pressure->press_abs = mavlink_msg_scaled_pressure_get_press_abs(msg);
	scaled_pressure->press_diff = mavlink_msg_scaled_pressure_get_press_diff(msg);
	scaled_pressure->temperature = mavlink_msg_scaled_pressure_get_temperature(msg);
}
