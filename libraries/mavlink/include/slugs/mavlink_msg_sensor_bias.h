// MESSAGE SENSOR_BIAS PACKING

#define MAVLINK_MSG_ID_SENSOR_BIAS 192

typedef struct __mavlink_sensor_bias_t 
{
	uint8_t target; ///< The system reporting the biases
	float axBias; ///< Accelerometer X bias (m/s)
	float ayBias; ///< Accelerometer Y bias (m/s)
	float azBias; ///< Accelerometer Z bias (m/s)
	float gxBias; ///< Gyro X bias (rad/s)
	float gyBias; ///< Gyro Y bias (rad/s)
	float gzBias; ///< Gyro Z bias (rad/s)

} mavlink_sensor_bias_t;



/**
 * @brief Send a sensor_bias message
 *
 * @param target The system reporting the biases
 * @param axBias Accelerometer X bias (m/s)
 * @param ayBias Accelerometer Y bias (m/s)
 * @param azBias Accelerometer Z bias (m/s)
 * @param gxBias Gyro X bias (rad/s)
 * @param gyBias Gyro Y bias (rad/s)
 * @param gzBias Gyro Z bias (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_bias_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, float axBias, float ayBias, float azBias, float gxBias, float gyBias, float gzBias)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SENSOR_BIAS;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system reporting the biases
	i += put_float_by_index(axBias, i, msg->payload); //Accelerometer X bias (m/s)
	i += put_float_by_index(ayBias, i, msg->payload); //Accelerometer Y bias (m/s)
	i += put_float_by_index(azBias, i, msg->payload); //Accelerometer Z bias (m/s)
	i += put_float_by_index(gxBias, i, msg->payload); //Gyro X bias (rad/s)
	i += put_float_by_index(gyBias, i, msg->payload); //Gyro Y bias (rad/s)
	i += put_float_by_index(gzBias, i, msg->payload); //Gyro Z bias (rad/s)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_sensor_bias_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_bias_t* sensor_bias)
{
	return mavlink_msg_sensor_bias_pack(system_id, component_id, msg, sensor_bias->target, sensor_bias->axBias, sensor_bias->ayBias, sensor_bias->azBias, sensor_bias->gxBias, sensor_bias->gyBias, sensor_bias->gzBias);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_bias_send(mavlink_channel_t chan, uint8_t target, float axBias, float ayBias, float azBias, float gxBias, float gyBias, float gzBias)
{
	mavlink_message_t msg;
	mavlink_msg_sensor_bias_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, axBias, ayBias, azBias, gxBias, gyBias, gzBias);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SENSOR_BIAS UNPACKING

/**
 * @brief Get field target from sensor_bias message
 *
 * @return The system reporting the biases
 */
static inline uint8_t mavlink_msg_sensor_bias_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field axBias from sensor_bias message
 *
 * @return Accelerometer X bias (m/s)
 */
static inline float mavlink_msg_sensor_bias_get_axBias(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field ayBias from sensor_bias message
 *
 * @return Accelerometer Y bias (m/s)
 */
static inline float mavlink_msg_sensor_bias_get_ayBias(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field azBias from sensor_bias message
 *
 * @return Accelerometer Z bias (m/s)
 */
static inline float mavlink_msg_sensor_bias_get_azBias(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field gxBias from sensor_bias message
 *
 * @return Gyro X bias (rad/s)
 */
static inline float mavlink_msg_sensor_bias_get_gxBias(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field gyBias from sensor_bias message
 *
 * @return Gyro Y bias (rad/s)
 */
static inline float mavlink_msg_sensor_bias_get_gyBias(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field gzBias from sensor_bias message
 *
 * @return Gyro Z bias (rad/s)
 */
static inline float mavlink_msg_sensor_bias_get_gzBias(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

static inline void mavlink_msg_sensor_bias_decode(const mavlink_message_t* msg, mavlink_sensor_bias_t* sensor_bias)
{
	sensor_bias->target = mavlink_msg_sensor_bias_get_target(msg);
	sensor_bias->axBias = mavlink_msg_sensor_bias_get_axBias(msg);
	sensor_bias->ayBias = mavlink_msg_sensor_bias_get_ayBias(msg);
	sensor_bias->azBias = mavlink_msg_sensor_bias_get_azBias(msg);
	sensor_bias->gxBias = mavlink_msg_sensor_bias_get_gxBias(msg);
	sensor_bias->gyBias = mavlink_msg_sensor_bias_get_gyBias(msg);
	sensor_bias->gzBias = mavlink_msg_sensor_bias_get_gzBias(msg);
}
