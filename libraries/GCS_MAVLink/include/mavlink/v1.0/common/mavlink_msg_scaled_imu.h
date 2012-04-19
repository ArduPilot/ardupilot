// MESSAGE SCALED_IMU PACKING

#define MAVLINK_MSG_ID_SCALED_IMU 26

typedef struct __mavlink_scaled_imu_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int16_t xacc; ///< X acceleration (mg)
 int16_t yacc; ///< Y acceleration (mg)
 int16_t zacc; ///< Z acceleration (mg)
 int16_t xgyro; ///< Angular speed around X axis (millirad /sec)
 int16_t ygyro; ///< Angular speed around Y axis (millirad /sec)
 int16_t zgyro; ///< Angular speed around Z axis (millirad /sec)
 int16_t xmag; ///< X Magnetic field (milli tesla)
 int16_t ymag; ///< Y Magnetic field (milli tesla)
 int16_t zmag; ///< Z Magnetic field (milli tesla)
} mavlink_scaled_imu_t;

#define MAVLINK_MSG_ID_SCALED_IMU_LEN 22
#define MAVLINK_MSG_ID_26_LEN 22



#define MAVLINK_MESSAGE_INFO_SCALED_IMU { \
	"SCALED_IMU", \
	10, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_scaled_imu_t, time_boot_ms) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_scaled_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_scaled_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_scaled_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_scaled_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_scaled_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_scaled_imu_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_scaled_imu_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_scaled_imu_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_scaled_imu_t, zmag) }, \
         } \
}


/**
 * @brief Pack a scaled_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[22];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int16_t(buf, 4, xacc);
	_mav_put_int16_t(buf, 6, yacc);
	_mav_put_int16_t(buf, 8, zacc);
	_mav_put_int16_t(buf, 10, xgyro);
	_mav_put_int16_t(buf, 12, ygyro);
	_mav_put_int16_t(buf, 14, zgyro);
	_mav_put_int16_t(buf, 16, xmag);
	_mav_put_int16_t(buf, 18, ymag);
	_mav_put_int16_t(buf, 20, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 22);
#else
	mavlink_scaled_imu_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 22);
#endif

	msg->msgid = MAVLINK_MSG_ID_SCALED_IMU;
	return mavlink_finalize_message(msg, system_id, component_id, 22, 170);
}

/**
 * @brief Pack a scaled_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int16_t xacc,int16_t yacc,int16_t zacc,int16_t xgyro,int16_t ygyro,int16_t zgyro,int16_t xmag,int16_t ymag,int16_t zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[22];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int16_t(buf, 4, xacc);
	_mav_put_int16_t(buf, 6, yacc);
	_mav_put_int16_t(buf, 8, zacc);
	_mav_put_int16_t(buf, 10, xgyro);
	_mav_put_int16_t(buf, 12, ygyro);
	_mav_put_int16_t(buf, 14, zgyro);
	_mav_put_int16_t(buf, 16, xmag);
	_mav_put_int16_t(buf, 18, ymag);
	_mav_put_int16_t(buf, 20, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 22);
#else
	mavlink_scaled_imu_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 22);
#endif

	msg->msgid = MAVLINK_MSG_ID_SCALED_IMU;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 22, 170);
}

/**
 * @brief Encode a scaled_imu struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scaled_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scaled_imu_t* scaled_imu)
{
	return mavlink_msg_scaled_imu_pack(system_id, component_id, msg, scaled_imu->time_boot_ms, scaled_imu->xacc, scaled_imu->yacc, scaled_imu->zacc, scaled_imu->xgyro, scaled_imu->ygyro, scaled_imu->zgyro, scaled_imu->xmag, scaled_imu->ymag, scaled_imu->zmag);
}

/**
 * @brief Send a scaled_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_scaled_imu_send(mavlink_channel_t chan, uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[22];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int16_t(buf, 4, xacc);
	_mav_put_int16_t(buf, 6, yacc);
	_mav_put_int16_t(buf, 8, zacc);
	_mav_put_int16_t(buf, 10, xgyro);
	_mav_put_int16_t(buf, 12, ygyro);
	_mav_put_int16_t(buf, 14, zgyro);
	_mav_put_int16_t(buf, 16, xmag);
	_mav_put_int16_t(buf, 18, ymag);
	_mav_put_int16_t(buf, 20, zmag);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU, buf, 22, 170);
#else
	mavlink_scaled_imu_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU, (const char *)&packet, 22, 170);
#endif
}

#endif

// MESSAGE SCALED_IMU UNPACKING


/**
 * @brief Get field time_boot_ms from scaled_imu message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_scaled_imu_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field xacc from scaled_imu message
 *
 * @return X acceleration (mg)
 */
static inline int16_t mavlink_msg_scaled_imu_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field yacc from scaled_imu message
 *
 * @return Y acceleration (mg)
 */
static inline int16_t mavlink_msg_scaled_imu_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field zacc from scaled_imu message
 *
 * @return Z acceleration (mg)
 */
static inline int16_t mavlink_msg_scaled_imu_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field xgyro from scaled_imu message
 *
 * @return Angular speed around X axis (millirad /sec)
 */
static inline int16_t mavlink_msg_scaled_imu_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field ygyro from scaled_imu message
 *
 * @return Angular speed around Y axis (millirad /sec)
 */
static inline int16_t mavlink_msg_scaled_imu_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field zgyro from scaled_imu message
 *
 * @return Angular speed around Z axis (millirad /sec)
 */
static inline int16_t mavlink_msg_scaled_imu_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field xmag from scaled_imu message
 *
 * @return X Magnetic field (milli tesla)
 */
static inline int16_t mavlink_msg_scaled_imu_get_xmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field ymag from scaled_imu message
 *
 * @return Y Magnetic field (milli tesla)
 */
static inline int16_t mavlink_msg_scaled_imu_get_ymag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field zmag from scaled_imu message
 *
 * @return Z Magnetic field (milli tesla)
 */
static inline int16_t mavlink_msg_scaled_imu_get_zmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Decode a scaled_imu message into a struct
 *
 * @param msg The message to decode
 * @param scaled_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_scaled_imu_decode(const mavlink_message_t* msg, mavlink_scaled_imu_t* scaled_imu)
{
#if MAVLINK_NEED_BYTE_SWAP
	scaled_imu->time_boot_ms = mavlink_msg_scaled_imu_get_time_boot_ms(msg);
	scaled_imu->xacc = mavlink_msg_scaled_imu_get_xacc(msg);
	scaled_imu->yacc = mavlink_msg_scaled_imu_get_yacc(msg);
	scaled_imu->zacc = mavlink_msg_scaled_imu_get_zacc(msg);
	scaled_imu->xgyro = mavlink_msg_scaled_imu_get_xgyro(msg);
	scaled_imu->ygyro = mavlink_msg_scaled_imu_get_ygyro(msg);
	scaled_imu->zgyro = mavlink_msg_scaled_imu_get_zgyro(msg);
	scaled_imu->xmag = mavlink_msg_scaled_imu_get_xmag(msg);
	scaled_imu->ymag = mavlink_msg_scaled_imu_get_ymag(msg);
	scaled_imu->zmag = mavlink_msg_scaled_imu_get_zmag(msg);
#else
	memcpy(scaled_imu, _MAV_PAYLOAD(msg), 22);
#endif
}
