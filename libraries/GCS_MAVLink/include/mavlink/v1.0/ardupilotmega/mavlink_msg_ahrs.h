// MESSAGE AHRS PACKING

#define MAVLINK_MSG_ID_AHRS 163

typedef struct __mavlink_ahrs_t
{
 float omegaIx; ///< X gyro drift estimate rad/s
 float omegaIy; ///< Y gyro drift estimate rad/s
 float omegaIz; ///< Z gyro drift estimate rad/s
 float accel_weight; ///< average accel_weight
 float renorm_val; ///< average renormalisation value
 float error_rp; ///< average error_roll_pitch value
 float error_yaw; ///< average error_yaw value
} mavlink_ahrs_t;

#define MAVLINK_MSG_ID_AHRS_LEN 28
#define MAVLINK_MSG_ID_163_LEN 28

#define MAVLINK_MSG_ID_AHRS_CRC 127
#define MAVLINK_MSG_ID_163_CRC 127



#define MAVLINK_MESSAGE_INFO_AHRS { \
	"AHRS", \
	7, \
	{  { "omegaIx", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ahrs_t, omegaIx) }, \
         { "omegaIy", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ahrs_t, omegaIy) }, \
         { "omegaIz", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ahrs_t, omegaIz) }, \
         { "accel_weight", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ahrs_t, accel_weight) }, \
         { "renorm_val", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ahrs_t, renorm_val) }, \
         { "error_rp", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ahrs_t, error_rp) }, \
         { "error_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ahrs_t, error_yaw) }, \
         } \
}


/**
 * @brief Pack a ahrs message
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
static inline uint16_t mavlink_msg_ahrs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AHRS_LEN];
	_mav_put_float(buf, 0, omegaIx);
	_mav_put_float(buf, 4, omegaIy);
	_mav_put_float(buf, 8, omegaIz);
	_mav_put_float(buf, 12, accel_weight);
	_mav_put_float(buf, 16, renorm_val);
	_mav_put_float(buf, 20, error_rp);
	_mav_put_float(buf, 24, error_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AHRS_LEN);
#else
	mavlink_ahrs_t packet;
	packet.omegaIx = omegaIx;
	packet.omegaIy = omegaIy;
	packet.omegaIz = omegaIz;
	packet.accel_weight = accel_weight;
	packet.renorm_val = renorm_val;
	packet.error_rp = error_rp;
	packet.error_yaw = error_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AHRS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AHRS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AHRS_LEN);
#endif
}

/**
 * @brief Pack a ahrs message on a channel
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
static inline uint16_t mavlink_msg_ahrs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float omegaIx,float omegaIy,float omegaIz,float accel_weight,float renorm_val,float error_rp,float error_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AHRS_LEN];
	_mav_put_float(buf, 0, omegaIx);
	_mav_put_float(buf, 4, omegaIy);
	_mav_put_float(buf, 8, omegaIz);
	_mav_put_float(buf, 12, accel_weight);
	_mav_put_float(buf, 16, renorm_val);
	_mav_put_float(buf, 20, error_rp);
	_mav_put_float(buf, 24, error_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AHRS_LEN);
#else
	mavlink_ahrs_t packet;
	packet.omegaIx = omegaIx;
	packet.omegaIy = omegaIy;
	packet.omegaIz = omegaIz;
	packet.accel_weight = accel_weight;
	packet.renorm_val = renorm_val;
	packet.error_rp = error_rp;
	packet.error_yaw = error_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AHRS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AHRS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AHRS_LEN);
#endif
}

/**
 * @brief Encode a ahrs struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ahrs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ahrs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ahrs_t* ahrs)
{
	return mavlink_msg_ahrs_pack(system_id, component_id, msg, ahrs->omegaIx, ahrs->omegaIy, ahrs->omegaIz, ahrs->accel_weight, ahrs->renorm_val, ahrs->error_rp, ahrs->error_yaw);
}

/**
 * @brief Send a ahrs message
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

static inline void mavlink_msg_ahrs_send(mavlink_channel_t chan, float omegaIx, float omegaIy, float omegaIz, float accel_weight, float renorm_val, float error_rp, float error_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AHRS_LEN];
	_mav_put_float(buf, 0, omegaIx);
	_mav_put_float(buf, 4, omegaIy);
	_mav_put_float(buf, 8, omegaIz);
	_mav_put_float(buf, 12, accel_weight);
	_mav_put_float(buf, 16, renorm_val);
	_mav_put_float(buf, 20, error_rp);
	_mav_put_float(buf, 24, error_yaw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS, buf, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS, buf, MAVLINK_MSG_ID_AHRS_LEN);
#endif
#else
	mavlink_ahrs_t packet;
	packet.omegaIx = omegaIx;
	packet.omegaIy = omegaIy;
	packet.omegaIz = omegaIz;
	packet.accel_weight = accel_weight;
	packet.renorm_val = renorm_val;
	packet.error_rp = error_rp;
	packet.error_yaw = error_yaw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS, (const char *)&packet, MAVLINK_MSG_ID_AHRS_LEN, MAVLINK_MSG_ID_AHRS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AHRS, (const char *)&packet, MAVLINK_MSG_ID_AHRS_LEN);
#endif
#endif
}

#endif

// MESSAGE AHRS UNPACKING


/**
 * @brief Get field omegaIx from ahrs message
 *
 * @return X gyro drift estimate rad/s
 */
static inline float mavlink_msg_ahrs_get_omegaIx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field omegaIy from ahrs message
 *
 * @return Y gyro drift estimate rad/s
 */
static inline float mavlink_msg_ahrs_get_omegaIy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field omegaIz from ahrs message
 *
 * @return Z gyro drift estimate rad/s
 */
static inline float mavlink_msg_ahrs_get_omegaIz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field accel_weight from ahrs message
 *
 * @return average accel_weight
 */
static inline float mavlink_msg_ahrs_get_accel_weight(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field renorm_val from ahrs message
 *
 * @return average renormalisation value
 */
static inline float mavlink_msg_ahrs_get_renorm_val(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field error_rp from ahrs message
 *
 * @return average error_roll_pitch value
 */
static inline float mavlink_msg_ahrs_get_error_rp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field error_yaw from ahrs message
 *
 * @return average error_yaw value
 */
static inline float mavlink_msg_ahrs_get_error_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a ahrs message into a struct
 *
 * @param msg The message to decode
 * @param ahrs C-struct to decode the message contents into
 */
static inline void mavlink_msg_ahrs_decode(const mavlink_message_t* msg, mavlink_ahrs_t* ahrs)
{
#if MAVLINK_NEED_BYTE_SWAP
	ahrs->omegaIx = mavlink_msg_ahrs_get_omegaIx(msg);
	ahrs->omegaIy = mavlink_msg_ahrs_get_omegaIy(msg);
	ahrs->omegaIz = mavlink_msg_ahrs_get_omegaIz(msg);
	ahrs->accel_weight = mavlink_msg_ahrs_get_accel_weight(msg);
	ahrs->renorm_val = mavlink_msg_ahrs_get_renorm_val(msg);
	ahrs->error_rp = mavlink_msg_ahrs_get_error_rp(msg);
	ahrs->error_yaw = mavlink_msg_ahrs_get_error_yaw(msg);
#else
	memcpy(ahrs, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_AHRS_LEN);
#endif
}
