// MESSAGE DCM PACKING

#define MAVLINK_MSG_ID_DCM 163

typedef struct __mavlink_dcm_t
{
 float omegaIx; ///< X gyro drift estimate rad/s
 float omegaIy; ///< Y gyro drift estimate rad/s
 float omegaIz; ///< Z gyro drift estimate rad/s
 float accel_weight; ///< average accel_weight
 float renorm_val; ///< average renormalisation value
 float error_rp; ///< average error_roll_pitch value
 float error_yaw; ///< average error_yaw value
} mavlink_dcm_t;

#define MAVLINK_MSG_ID_DCM_LEN 28
#define MAVLINK_MSG_ID_163_LEN 28



#define MAVLINK_MESSAGE_INFO_DCM { \
	"DCM", \
	7, \
	{  { "omegaIx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_dcm_t, omegaIx) }, \
         { "omegaIy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_dcm_t, omegaIy) }, \
         { "omegaIz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_dcm_t, omegaIz) }, \
         { "accel_weight", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_dcm_t, accel_weight) }, \
         { "renorm_val", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_dcm_t, renorm_val) }, \
         { "error_rp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_dcm_t, error_rp) }, \
         { "error_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_dcm_t, error_yaw) }, \
         } \
}


/**
 * @brief Pack a dcm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param omegaIx X gyro drift estimate rad/s
 * @param omegaIy Y gyro drift estimate rad/s
 * @param omegaIz Z gyro drift estimate rad/s
 * @param accel_weight average accel_weight
 * @param renorm_val average renormalisation value
 * @param error_rp average error_roll_pitch value
 * @param error_yaw average error_yaw value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dcm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_float(buf, 0, omegaIx);
	_mav_put_float(buf, 4, omegaIy);
	_mav_put_float(buf, 8, omegaIz);
	_mav_put_float(buf, 12, accel_weight);
	_mav_put_float(buf, 16, renorm_val);
	_mav_put_float(buf, 20, error_rp);
	_mav_put_float(buf, 24, error_yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 28);
#else
	mavlink_dcm_t packet;
	packet.omegaIx = omegaIx;
	packet.omegaIy = omegaIy;
	packet.omegaIz = omegaIz;
	packet.accel_weight = accel_weight;
	packet.renorm_val = renorm_val;
	packet.error_rp = error_rp;
	packet.error_yaw = error_yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_DCM;
	return mavlink_finalize_message(msg, system_id, component_id, 28, 205);
}

/**
 * @brief Pack a dcm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param omegaIx X gyro drift estimate rad/s
 * @param omegaIy Y gyro drift estimate rad/s
 * @param omegaIz Z gyro drift estimate rad/s
 * @param accel_weight average accel_weight
 * @param renorm_val average renormalisation value
 * @param error_rp average error_roll_pitch value
 * @param error_yaw average error_yaw value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dcm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float omegaIx,float omegaIy,float omegaIz,float accel_weight,float renorm_val,float error_rp,float error_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_float(buf, 0, omegaIx);
	_mav_put_float(buf, 4, omegaIy);
	_mav_put_float(buf, 8, omegaIz);
	_mav_put_float(buf, 12, accel_weight);
	_mav_put_float(buf, 16, renorm_val);
	_mav_put_float(buf, 20, error_rp);
	_mav_put_float(buf, 24, error_yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 28);
#else
	mavlink_dcm_t packet;
	packet.omegaIx = omegaIx;
	packet.omegaIy = omegaIy;
	packet.omegaIz = omegaIz;
	packet.accel_weight = accel_weight;
	packet.renorm_val = renorm_val;
	packet.error_rp = error_rp;
	packet.error_yaw = error_yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_DCM;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 28, 205);
}

/**
 * @brief Encode a dcm struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dcm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dcm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dcm_t* dcm)
{
	return mavlink_msg_dcm_pack(system_id, component_id, msg, dcm->omegaIx, dcm->omegaIy, dcm->omegaIz, dcm->accel_weight, dcm->renorm_val, dcm->error_rp, dcm->error_yaw);
}

/**
 * @brief Send a dcm message
 * @param chan MAVLink channel to send the message
 *
 * @param omegaIx X gyro drift estimate rad/s
 * @param omegaIy Y gyro drift estimate rad/s
 * @param omegaIz Z gyro drift estimate rad/s
 * @param accel_weight average accel_weight
 * @param renorm_val average renormalisation value
 * @param error_rp average error_roll_pitch value
 * @param error_yaw average error_yaw value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dcm_send(mavlink_channel_t chan, float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_float(buf, 0, omegaIx);
	_mav_put_float(buf, 4, omegaIy);
	_mav_put_float(buf, 8, omegaIz);
	_mav_put_float(buf, 12, accel_weight);
	_mav_put_float(buf, 16, renorm_val);
	_mav_put_float(buf, 20, error_rp);
	_mav_put_float(buf, 24, error_yaw);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DCM, buf, 28, 205);
#else
	mavlink_dcm_t packet;
	packet.omegaIx = omegaIx;
	packet.omegaIy = omegaIy;
	packet.omegaIz = omegaIz;
	packet.accel_weight = accel_weight;
	packet.renorm_val = renorm_val;
	packet.error_rp = error_rp;
	packet.error_yaw = error_yaw;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DCM, (const char *)&packet, 28, 205);
#endif
}

#endif

// MESSAGE DCM UNPACKING


/**
 * @brief Get field omegaIx from dcm message
 *
 * @return X gyro drift estimate rad/s
 */
static inline float mavlink_msg_dcm_get_omegaIx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field omegaIy from dcm message
 *
 * @return Y gyro drift estimate rad/s
 */
static inline float mavlink_msg_dcm_get_omegaIy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field omegaIz from dcm message
 *
 * @return Z gyro drift estimate rad/s
 */
static inline float mavlink_msg_dcm_get_omegaIz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field accel_weight from dcm message
 *
 * @return average accel_weight
 */
static inline float mavlink_msg_dcm_get_accel_weight(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field renorm_val from dcm message
 *
 * @return average renormalisation value
 */
static inline float mavlink_msg_dcm_get_renorm_val(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field error_rp from dcm message
 *
 * @return average error_roll_pitch value
 */
static inline float mavlink_msg_dcm_get_error_rp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field error_yaw from dcm message
 *
 * @return average error_yaw value
 */
static inline float mavlink_msg_dcm_get_error_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a dcm message into a struct
 *
 * @param msg The message to decode
 * @param dcm C-struct to decode the message contents into
 */
static inline void mavlink_msg_dcm_decode(const mavlink_message_t* msg, mavlink_dcm_t* dcm)
{
#if MAVLINK_NEED_BYTE_SWAP
	dcm->omegaIx = mavlink_msg_dcm_get_omegaIx(msg);
	dcm->omegaIy = mavlink_msg_dcm_get_omegaIy(msg);
	dcm->omegaIz = mavlink_msg_dcm_get_omegaIz(msg);
	dcm->accel_weight = mavlink_msg_dcm_get_accel_weight(msg);
	dcm->renorm_val = mavlink_msg_dcm_get_renorm_val(msg);
	dcm->error_rp = mavlink_msg_dcm_get_error_rp(msg);
	dcm->error_yaw = mavlink_msg_dcm_get_error_yaw(msg);
#else
	memcpy(dcm, _MAV_PAYLOAD(msg), 28);
#endif
}
