// MESSAGE OBS_BIAS PACKING

#define MAVLINK_MSG_ID_OBS_BIAS 180

typedef struct __mavlink_obs_bias_t
{
 float accBias[3]; ///< accelerometer bias
 float gyroBias[3]; ///< gyroscope bias
} mavlink_obs_bias_t;

#define MAVLINK_MSG_ID_OBS_BIAS_LEN 24
#define MAVLINK_MSG_ID_180_LEN 24

#define MAVLINK_MSG_OBS_BIAS_FIELD_ACCBIAS_LEN 3
#define MAVLINK_MSG_OBS_BIAS_FIELD_GYROBIAS_LEN 3

#define MAVLINK_MESSAGE_INFO_OBS_BIAS { \
	"OBS_BIAS", \
	2, \
	{  { "accBias", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_obs_bias_t, accBias) }, \
         { "gyroBias", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_obs_bias_t, gyroBias) }, \
         } \
}


/**
 * @brief Pack a obs_bias message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param accBias accelerometer bias
 * @param gyroBias gyroscope bias
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_bias_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const float *accBias, const float *gyroBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];

	_mav_put_float_array(buf, 0, accBias, 3);
	_mav_put_float_array(buf, 12, gyroBias, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 24);
#else
	mavlink_obs_bias_t packet;

	mav_array_memcpy(packet.accBias, accBias, sizeof(float)*3);
	mav_array_memcpy(packet.gyroBias, gyroBias, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_BIAS;
	return mavlink_finalize_message(msg, system_id, component_id, 24, 159);
}

/**
 * @brief Pack a obs_bias message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param accBias accelerometer bias
 * @param gyroBias gyroscope bias
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_bias_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const float *accBias,const float *gyroBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];

	_mav_put_float_array(buf, 0, accBias, 3);
	_mav_put_float_array(buf, 12, gyroBias, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 24);
#else
	mavlink_obs_bias_t packet;

	mav_array_memcpy(packet.accBias, accBias, sizeof(float)*3);
	mav_array_memcpy(packet.gyroBias, gyroBias, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_BIAS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 24, 159);
}

/**
 * @brief Encode a obs_bias struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param obs_bias C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obs_bias_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_obs_bias_t* obs_bias)
{
	return mavlink_msg_obs_bias_pack(system_id, component_id, msg, obs_bias->accBias, obs_bias->gyroBias);
}

/**
 * @brief Send a obs_bias message
 * @param chan MAVLink channel to send the message
 *
 * @param accBias accelerometer bias
 * @param gyroBias gyroscope bias
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_obs_bias_send(mavlink_channel_t chan, const float *accBias, const float *gyroBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];

	_mav_put_float_array(buf, 0, accBias, 3);
	_mav_put_float_array(buf, 12, gyroBias, 3);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_BIAS, buf, 24, 159);
#else
	mavlink_obs_bias_t packet;

	mav_array_memcpy(packet.accBias, accBias, sizeof(float)*3);
	mav_array_memcpy(packet.gyroBias, gyroBias, sizeof(float)*3);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_BIAS, (const char *)&packet, 24, 159);
#endif
}

#endif

// MESSAGE OBS_BIAS UNPACKING


/**
 * @brief Get field accBias from obs_bias message
 *
 * @return accelerometer bias
 */
static inline uint16_t mavlink_msg_obs_bias_get_accBias(const mavlink_message_t* msg, float *accBias)
{
	return _MAV_RETURN_float_array(msg, accBias, 3,  0);
}

/**
 * @brief Get field gyroBias from obs_bias message
 *
 * @return gyroscope bias
 */
static inline uint16_t mavlink_msg_obs_bias_get_gyroBias(const mavlink_message_t* msg, float *gyroBias)
{
	return _MAV_RETURN_float_array(msg, gyroBias, 3,  12);
}

/**
 * @brief Decode a obs_bias message into a struct
 *
 * @param msg The message to decode
 * @param obs_bias C-struct to decode the message contents into
 */
static inline void mavlink_msg_obs_bias_decode(const mavlink_message_t* msg, mavlink_obs_bias_t* obs_bias)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_obs_bias_get_accBias(msg, obs_bias->accBias);
	mavlink_msg_obs_bias_get_gyroBias(msg, obs_bias->gyroBias);
#else
	memcpy(obs_bias, _MAV_PAYLOAD(msg), 24);
#endif
}
