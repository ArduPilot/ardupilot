// MESSAGE OBS_ATTITUDE PACKING

#define MAVLINK_MSG_ID_OBS_ATTITUDE 174

typedef struct __mavlink_obs_attitude_t
{
 double quat[4]; ///< Quaternion re;im
} mavlink_obs_attitude_t;

#define MAVLINK_MSG_ID_OBS_ATTITUDE_LEN 32
#define MAVLINK_MSG_ID_174_LEN 32

#define MAVLINK_MSG_OBS_ATTITUDE_FIELD_QUAT_LEN 4

#define MAVLINK_MESSAGE_INFO_OBS_ATTITUDE { \
	"OBS_ATTITUDE", \
	1, \
	{  { "quat", NULL, MAVLINK_TYPE_DOUBLE, 4, 0, offsetof(mavlink_obs_attitude_t, quat) }, \
         } \
}


/**
 * @brief Pack a obs_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param quat Quaternion re;im
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const double *quat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];

	_mav_put_double_array(buf, 0, quat, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 32);
#else
	mavlink_obs_attitude_t packet;

	mav_array_memcpy(packet.quat, quat, sizeof(double)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 32);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_ATTITUDE;
	return mavlink_finalize_message(msg, system_id, component_id, 32, 146);
}

/**
 * @brief Pack a obs_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param quat Quaternion re;im
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const double *quat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];

	_mav_put_double_array(buf, 0, quat, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 32);
#else
	mavlink_obs_attitude_t packet;

	mav_array_memcpy(packet.quat, quat, sizeof(double)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 32);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_ATTITUDE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32, 146);
}

/**
 * @brief Encode a obs_attitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param obs_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obs_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_obs_attitude_t* obs_attitude)
{
	return mavlink_msg_obs_attitude_pack(system_id, component_id, msg, obs_attitude->quat);
}

/**
 * @brief Send a obs_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param quat Quaternion re;im
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_obs_attitude_send(mavlink_channel_t chan, const double *quat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[32];

	_mav_put_double_array(buf, 0, quat, 4);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_ATTITUDE, buf, 32, 146);
#else
	mavlink_obs_attitude_t packet;

	mav_array_memcpy(packet.quat, quat, sizeof(double)*4);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_ATTITUDE, (const char *)&packet, 32, 146);
#endif
}

#endif

// MESSAGE OBS_ATTITUDE UNPACKING


/**
 * @brief Get field quat from obs_attitude message
 *
 * @return Quaternion re;im
 */
static inline uint16_t mavlink_msg_obs_attitude_get_quat(const mavlink_message_t* msg, double *quat)
{
	return _MAV_RETURN_double_array(msg, quat, 4,  0);
}

/**
 * @brief Decode a obs_attitude message into a struct
 *
 * @param msg The message to decode
 * @param obs_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_obs_attitude_decode(const mavlink_message_t* msg, mavlink_obs_attitude_t* obs_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_obs_attitude_get_quat(msg, obs_attitude->quat);
#else
	memcpy(obs_attitude, _MAV_PAYLOAD(msg), 32);
#endif
}
