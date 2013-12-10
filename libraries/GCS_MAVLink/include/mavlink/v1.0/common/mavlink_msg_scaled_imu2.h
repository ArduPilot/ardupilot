// MESSAGE SCALED_IMU2 PACKING

#define MAVLINK_MSG_ID_SCALED_IMU2 116

typedef struct __mavlink_scaled_imu2_t
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
} mavlink_scaled_imu2_t;

#define MAVLINK_MSG_ID_SCALED_IMU2_LEN 22
#define MAVLINK_MSG_ID_116_LEN 22

#define MAVLINK_MSG_ID_SCALED_IMU2_CRC 76
#define MAVLINK_MSG_ID_116_CRC 76



#define MAVLINK_MESSAGE_INFO_SCALED_IMU2 { \
	"SCALED_IMU2", \
	10, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_scaled_imu2_t, time_boot_ms) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_scaled_imu2_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_scaled_imu2_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_scaled_imu2_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_scaled_imu2_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_scaled_imu2_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_scaled_imu2_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_scaled_imu2_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_scaled_imu2_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_scaled_imu2_t, zmag) }, \
         } \
}


/**
 * @brief Pack a scaled_imu2 message
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
static inline uint16_t mavlink_msg_scaled_imu2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SCALED_IMU2_LEN];
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCALED_IMU2_LEN);
#else
	mavlink_scaled_imu2_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCALED_IMU2_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SCALED_IMU2;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SCALED_IMU2_LEN, MAVLINK_MSG_ID_SCALED_IMU2_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SCALED_IMU2_LEN);
#endif
}

/**
 * @brief Pack a scaled_imu2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
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
static inline uint16_t mavlink_msg_scaled_imu2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int16_t xacc,int16_t yacc,int16_t zacc,int16_t xgyro,int16_t ygyro,int16_t zgyro,int16_t xmag,int16_t ymag,int16_t zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SCALED_IMU2_LEN];
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCALED_IMU2_LEN);
#else
	mavlink_scaled_imu2_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCALED_IMU2_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SCALED_IMU2;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SCALED_IMU2_LEN, MAVLINK_MSG_ID_SCALED_IMU2_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SCALED_IMU2_LEN);
#endif
}

/**
 * @brief Encode a scaled_imu2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scaled_imu2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_imu2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scaled_imu2_t* scaled_imu2)
{
	return mavlink_msg_scaled_imu2_pack(system_id, component_id, msg, scaled_imu2->time_boot_ms, scaled_imu2->xacc, scaled_imu2->yacc, scaled_imu2->zacc, scaled_imu2->xgyro, scaled_imu2->ygyro, scaled_imu2->zgyro, scaled_imu2->xmag, scaled_imu2->ymag, scaled_imu2->zmag);
}

/**
 * @brief Encode a scaled_imu2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param scaled_imu2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_imu2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_scaled_imu2_t* scaled_imu2)
{
	return mavlink_msg_scaled_imu2_pack_chan(system_id, component_id, chan, msg, scaled_imu2->time_boot_ms, scaled_imu2->xacc, scaled_imu2->yacc, scaled_imu2->zacc, scaled_imu2->xgyro, scaled_imu2->ygyro, scaled_imu2->zgyro, scaled_imu2->xmag, scaled_imu2->ymag, scaled_imu2->zmag);
}

/**
 * @brief Send a scaled_imu2 message
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

static inline void mavlink_msg_scaled_imu2_send(mavlink_channel_t chan, uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SCALED_IMU2_LEN];
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

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU2, buf, MAVLINK_MSG_ID_SCALED_IMU2_LEN, MAVLINK_MSG_ID_SCALED_IMU2_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU2, buf, MAVLINK_MSG_ID_SCALED_IMU2_LEN);
#endif
#else
	mavlink_scaled_imu2_t packet;
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

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU2, (const char *)&packet, MAVLINK_MSG_ID_SCALED_IMU2_LEN, MAVLINK_MSG_ID_SCALED_IMU2_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU2, (const char *)&packet, MAVLINK_MSG_ID_SCALED_IMU2_LEN);
#endif
#endif
}

#endif

// MESSAGE SCALED_IMU2 UNPACKING


/**
 * @brief Get field time_boot_ms from scaled_imu2 message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_scaled_imu2_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field xacc from scaled_imu2 message
 *
 * @return X acceleration (mg)
 */
static inline int16_t mavlink_msg_scaled_imu2_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field yacc from scaled_imu2 message
 *
 * @return Y acceleration (mg)
 */
static inline int16_t mavlink_msg_scaled_imu2_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field zacc from scaled_imu2 message
 *
 * @return Z acceleration (mg)
 */
static inline int16_t mavlink_msg_scaled_imu2_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field xgyro from scaled_imu2 message
 *
 * @return Angular speed around X axis (millirad /sec)
 */
static inline int16_t mavlink_msg_scaled_imu2_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field ygyro from scaled_imu2 message
 *
 * @return Angular speed around Y axis (millirad /sec)
 */
static inline int16_t mavlink_msg_scaled_imu2_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field zgyro from scaled_imu2 message
 *
 * @return Angular speed around Z axis (millirad /sec)
 */
static inline int16_t mavlink_msg_scaled_imu2_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field xmag from scaled_imu2 message
 *
 * @return X Magnetic field (milli tesla)
 */
static inline int16_t mavlink_msg_scaled_imu2_get_xmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field ymag from scaled_imu2 message
 *
 * @return Y Magnetic field (milli tesla)
 */
static inline int16_t mavlink_msg_scaled_imu2_get_ymag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field zmag from scaled_imu2 message
 *
 * @return Z Magnetic field (milli tesla)
 */
static inline int16_t mavlink_msg_scaled_imu2_get_zmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Decode a scaled_imu2 message into a struct
 *
 * @param msg The message to decode
 * @param scaled_imu2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_scaled_imu2_decode(const mavlink_message_t* msg, mavlink_scaled_imu2_t* scaled_imu2)
{
#if MAVLINK_NEED_BYTE_SWAP
	scaled_imu2->time_boot_ms = mavlink_msg_scaled_imu2_get_time_boot_ms(msg);
	scaled_imu2->xacc = mavlink_msg_scaled_imu2_get_xacc(msg);
	scaled_imu2->yacc = mavlink_msg_scaled_imu2_get_yacc(msg);
	scaled_imu2->zacc = mavlink_msg_scaled_imu2_get_zacc(msg);
	scaled_imu2->xgyro = mavlink_msg_scaled_imu2_get_xgyro(msg);
	scaled_imu2->ygyro = mavlink_msg_scaled_imu2_get_ygyro(msg);
	scaled_imu2->zgyro = mavlink_msg_scaled_imu2_get_zgyro(msg);
	scaled_imu2->xmag = mavlink_msg_scaled_imu2_get_xmag(msg);
	scaled_imu2->ymag = mavlink_msg_scaled_imu2_get_ymag(msg);
	scaled_imu2->zmag = mavlink_msg_scaled_imu2_get_zmag(msg);
#else
	memcpy(scaled_imu2, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SCALED_IMU2_LEN);
#endif
}
