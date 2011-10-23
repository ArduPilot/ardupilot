// MESSAGE SENSOR_BIAS PACKING

#define MAVLINK_MSG_ID_SENSOR_BIAS 172

typedef struct __mavlink_sensor_bias_t
{
 float axBias; ///< Accelerometer X bias (m/s)
 float ayBias; ///< Accelerometer Y bias (m/s)
 float azBias; ///< Accelerometer Z bias (m/s)
 float gxBias; ///< Gyro X bias (rad/s)
 float gyBias; ///< Gyro Y bias (rad/s)
 float gzBias; ///< Gyro Z bias (rad/s)
} mavlink_sensor_bias_t;

#define MAVLINK_MSG_ID_SENSOR_BIAS_LEN 24
#define MAVLINK_MSG_ID_172_LEN 24



#define MAVLINK_MESSAGE_INFO_SENSOR_BIAS { \
	"SENSOR_BIAS", \
	6, \
	{  { "axBias", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_bias_t, axBias) }, \
         { "ayBias", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sensor_bias_t, ayBias) }, \
         { "azBias", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sensor_bias_t, azBias) }, \
         { "gxBias", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_bias_t, gxBias) }, \
         { "gyBias", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_bias_t, gyBias) }, \
         { "gzBias", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_bias_t, gzBias) }, \
         } \
}


/**
 * @brief Pack a sensor_bias message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param axBias Accelerometer X bias (m/s)
 * @param ayBias Accelerometer Y bias (m/s)
 * @param azBias Accelerometer Z bias (m/s)
 * @param gxBias Gyro X bias (rad/s)
 * @param gyBias Gyro Y bias (rad/s)
 * @param gzBias Gyro Z bias (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_bias_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float axBias, float ayBias, float azBias, float gxBias, float gyBias, float gzBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_float(buf, 0, axBias);
	_mav_put_float(buf, 4, ayBias);
	_mav_put_float(buf, 8, azBias);
	_mav_put_float(buf, 12, gxBias);
	_mav_put_float(buf, 16, gyBias);
	_mav_put_float(buf, 20, gzBias);

        memcpy(_MAV_PAYLOAD(msg), buf, 24);
#else
	mavlink_sensor_bias_t packet;
	packet.axBias = axBias;
	packet.ayBias = ayBias;
	packet.azBias = azBias;
	packet.gxBias = gxBias;
	packet.gyBias = gyBias;
	packet.gzBias = gzBias;

        memcpy(_MAV_PAYLOAD(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_SENSOR_BIAS;
	return mavlink_finalize_message(msg, system_id, component_id, 24, 168);
}

/**
 * @brief Pack a sensor_bias message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param axBias Accelerometer X bias (m/s)
 * @param ayBias Accelerometer Y bias (m/s)
 * @param azBias Accelerometer Z bias (m/s)
 * @param gxBias Gyro X bias (rad/s)
 * @param gyBias Gyro Y bias (rad/s)
 * @param gzBias Gyro Z bias (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_bias_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float axBias,float ayBias,float azBias,float gxBias,float gyBias,float gzBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_float(buf, 0, axBias);
	_mav_put_float(buf, 4, ayBias);
	_mav_put_float(buf, 8, azBias);
	_mav_put_float(buf, 12, gxBias);
	_mav_put_float(buf, 16, gyBias);
	_mav_put_float(buf, 20, gzBias);

        memcpy(_MAV_PAYLOAD(msg), buf, 24);
#else
	mavlink_sensor_bias_t packet;
	packet.axBias = axBias;
	packet.ayBias = ayBias;
	packet.azBias = azBias;
	packet.gxBias = gxBias;
	packet.gyBias = gyBias;
	packet.gzBias = gzBias;

        memcpy(_MAV_PAYLOAD(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_SENSOR_BIAS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 24, 168);
}

/**
 * @brief Encode a sensor_bias struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_bias C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_bias_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_bias_t* sensor_bias)
{
	return mavlink_msg_sensor_bias_pack(system_id, component_id, msg, sensor_bias->axBias, sensor_bias->ayBias, sensor_bias->azBias, sensor_bias->gxBias, sensor_bias->gyBias, sensor_bias->gzBias);
}

/**
 * @brief Send a sensor_bias message
 * @param chan MAVLink channel to send the message
 *
 * @param axBias Accelerometer X bias (m/s)
 * @param ayBias Accelerometer Y bias (m/s)
 * @param azBias Accelerometer Z bias (m/s)
 * @param gxBias Gyro X bias (rad/s)
 * @param gyBias Gyro Y bias (rad/s)
 * @param gzBias Gyro Z bias (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_bias_send(mavlink_channel_t chan, float axBias, float ayBias, float azBias, float gxBias, float gyBias, float gzBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_float(buf, 0, axBias);
	_mav_put_float(buf, 4, ayBias);
	_mav_put_float(buf, 8, azBias);
	_mav_put_float(buf, 12, gxBias);
	_mav_put_float(buf, 16, gyBias);
	_mav_put_float(buf, 20, gzBias);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_BIAS, buf, 24, 168);
#else
	mavlink_sensor_bias_t packet;
	packet.axBias = axBias;
	packet.ayBias = ayBias;
	packet.azBias = azBias;
	packet.gxBias = gxBias;
	packet.gyBias = gyBias;
	packet.gzBias = gzBias;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_BIAS, (const char *)&packet, 24, 168);
#endif
}

#endif

// MESSAGE SENSOR_BIAS UNPACKING


/**
 * @brief Get field axBias from sensor_bias message
 *
 * @return Accelerometer X bias (m/s)
 */
static inline float mavlink_msg_sensor_bias_get_axBias(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ayBias from sensor_bias message
 *
 * @return Accelerometer Y bias (m/s)
 */
static inline float mavlink_msg_sensor_bias_get_ayBias(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field azBias from sensor_bias message
 *
 * @return Accelerometer Z bias (m/s)
 */
static inline float mavlink_msg_sensor_bias_get_azBias(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gxBias from sensor_bias message
 *
 * @return Gyro X bias (rad/s)
 */
static inline float mavlink_msg_sensor_bias_get_gxBias(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyBias from sensor_bias message
 *
 * @return Gyro Y bias (rad/s)
 */
static inline float mavlink_msg_sensor_bias_get_gyBias(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gzBias from sensor_bias message
 *
 * @return Gyro Z bias (rad/s)
 */
static inline float mavlink_msg_sensor_bias_get_gzBias(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a sensor_bias message into a struct
 *
 * @param msg The message to decode
 * @param sensor_bias C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_bias_decode(const mavlink_message_t* msg, mavlink_sensor_bias_t* sensor_bias)
{
#if MAVLINK_NEED_BYTE_SWAP
	sensor_bias->axBias = mavlink_msg_sensor_bias_get_axBias(msg);
	sensor_bias->ayBias = mavlink_msg_sensor_bias_get_ayBias(msg);
	sensor_bias->azBias = mavlink_msg_sensor_bias_get_azBias(msg);
	sensor_bias->gxBias = mavlink_msg_sensor_bias_get_gxBias(msg);
	sensor_bias->gyBias = mavlink_msg_sensor_bias_get_gyBias(msg);
	sensor_bias->gzBias = mavlink_msg_sensor_bias_get_gzBias(msg);
#else
	memcpy(sensor_bias, _MAV_PAYLOAD(msg), 24);
#endif
}
