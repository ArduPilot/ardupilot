// MESSAGE RAW_IMU PACKING

#define MAVLINK_MSG_ID_RAW_IMU 28

typedef struct __mavlink_raw_imu_t 
{
	uint64_t usec; ///< Timestamp (microseconds since UNIX epoch)
	int16_t xacc; ///< X acceleration (mg raw)
	int16_t yacc; ///< Y acceleration (mg raw)
	int16_t zacc; ///< Z acceleration (mg raw)
	int16_t xgyro; ///< Angular speed around X axis (millirad /sec)
	int16_t ygyro; ///< Angular speed around Y axis (millirad /sec)
	int16_t zgyro; ///< Angular speed around Z axis (millirad /sec)
	int16_t xmag; ///< X Magnetic field (milli tesla)
	int16_t ymag; ///< Y Magnetic field (milli tesla)
	int16_t zmag; ///< Z Magnetic field (milli tesla)

} mavlink_raw_imu_t;



/**
 * @brief Pack a raw_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch)
 * @param xacc X acceleration (mg raw)
 * @param yacc Y acceleration (mg raw)
 * @param zacc Z acceleration (mg raw)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_RAW_IMU;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds since UNIX epoch)
	i += put_int16_t_by_index(xacc, i, msg->payload); // X acceleration (mg raw)
	i += put_int16_t_by_index(yacc, i, msg->payload); // Y acceleration (mg raw)
	i += put_int16_t_by_index(zacc, i, msg->payload); // Z acceleration (mg raw)
	i += put_int16_t_by_index(xgyro, i, msg->payload); // Angular speed around X axis (millirad /sec)
	i += put_int16_t_by_index(ygyro, i, msg->payload); // Angular speed around Y axis (millirad /sec)
	i += put_int16_t_by_index(zgyro, i, msg->payload); // Angular speed around Z axis (millirad /sec)
	i += put_int16_t_by_index(xmag, i, msg->payload); // X Magnetic field (milli tesla)
	i += put_int16_t_by_index(ymag, i, msg->payload); // Y Magnetic field (milli tesla)
	i += put_int16_t_by_index(zmag, i, msg->payload); // Z Magnetic field (milli tesla)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a raw_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch)
 * @param xacc X acceleration (mg raw)
 * @param yacc Y acceleration (mg raw)
 * @param zacc Z acceleration (mg raw)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_RAW_IMU;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds since UNIX epoch)
	i += put_int16_t_by_index(xacc, i, msg->payload); // X acceleration (mg raw)
	i += put_int16_t_by_index(yacc, i, msg->payload); // Y acceleration (mg raw)
	i += put_int16_t_by_index(zacc, i, msg->payload); // Z acceleration (mg raw)
	i += put_int16_t_by_index(xgyro, i, msg->payload); // Angular speed around X axis (millirad /sec)
	i += put_int16_t_by_index(ygyro, i, msg->payload); // Angular speed around Y axis (millirad /sec)
	i += put_int16_t_by_index(zgyro, i, msg->payload); // Angular speed around Z axis (millirad /sec)
	i += put_int16_t_by_index(xmag, i, msg->payload); // X Magnetic field (milli tesla)
	i += put_int16_t_by_index(ymag, i, msg->payload); // Y Magnetic field (milli tesla)
	i += put_int16_t_by_index(zmag, i, msg->payload); // Z Magnetic field (milli tesla)

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a raw_imu struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_imu_t* raw_imu)
{
	return mavlink_msg_raw_imu_pack(system_id, component_id, msg, raw_imu->usec, raw_imu->xacc, raw_imu->yacc, raw_imu->zacc, raw_imu->xgyro, raw_imu->ygyro, raw_imu->zgyro, raw_imu->xmag, raw_imu->ymag, raw_imu->zmag);
}

/**
 * @brief Send a raw_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch)
 * @param xacc X acceleration (mg raw)
 * @param yacc Y acceleration (mg raw)
 * @param zacc Z acceleration (mg raw)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_imu_send(mavlink_channel_t chan, uint64_t usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag)
{
	mavlink_message_t msg;
	mavlink_msg_raw_imu_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, usec, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE RAW_IMU UNPACKING

/**
 * @brief Get field usec from raw_imu message
 *
 * @return Timestamp (microseconds since UNIX epoch)
 */
static inline uint64_t mavlink_msg_raw_imu_get_usec(const mavlink_message_t* msg)
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
 * @brief Get field xacc from raw_imu message
 *
 * @return X acceleration (mg raw)
 */
static inline int16_t mavlink_msg_raw_imu_get_xacc(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field yacc from raw_imu message
 *
 * @return Y acceleration (mg raw)
 */
static inline int16_t mavlink_msg_raw_imu_get_yacc(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field zacc from raw_imu message
 *
 * @return Z acceleration (mg raw)
 */
static inline int16_t mavlink_msg_raw_imu_get_zacc(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field xgyro from raw_imu message
 *
 * @return Angular speed around X axis (millirad /sec)
 */
static inline int16_t mavlink_msg_raw_imu_get_xgyro(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field ygyro from raw_imu message
 *
 * @return Angular speed around Y axis (millirad /sec)
 */
static inline int16_t mavlink_msg_raw_imu_get_ygyro(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field zgyro from raw_imu message
 *
 * @return Angular speed around Z axis (millirad /sec)
 */
static inline int16_t mavlink_msg_raw_imu_get_zgyro(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field xmag from raw_imu message
 *
 * @return X Magnetic field (milli tesla)
 */
static inline int16_t mavlink_msg_raw_imu_get_xmag(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field ymag from raw_imu message
 *
 * @return Y Magnetic field (milli tesla)
 */
static inline int16_t mavlink_msg_raw_imu_get_ymag(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field zmag from raw_imu message
 *
 * @return Z Magnetic field (milli tesla)
 */
static inline int16_t mavlink_msg_raw_imu_get_zmag(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Decode a raw_imu message into a struct
 *
 * @param msg The message to decode
 * @param raw_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
{
	raw_imu->usec = mavlink_msg_raw_imu_get_usec(msg);
	raw_imu->xacc = mavlink_msg_raw_imu_get_xacc(msg);
	raw_imu->yacc = mavlink_msg_raw_imu_get_yacc(msg);
	raw_imu->zacc = mavlink_msg_raw_imu_get_zacc(msg);
	raw_imu->xgyro = mavlink_msg_raw_imu_get_xgyro(msg);
	raw_imu->ygyro = mavlink_msg_raw_imu_get_ygyro(msg);
	raw_imu->zgyro = mavlink_msg_raw_imu_get_zgyro(msg);
	raw_imu->xmag = mavlink_msg_raw_imu_get_xmag(msg);
	raw_imu->ymag = mavlink_msg_raw_imu_get_ymag(msg);
	raw_imu->zmag = mavlink_msg_raw_imu_get_zmag(msg);
}
