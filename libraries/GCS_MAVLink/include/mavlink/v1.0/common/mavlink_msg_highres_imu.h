// MESSAGE HIGHRES_IMU PACKING

#define MAVLINK_MSG_ID_HIGHRES_IMU 105

typedef struct __mavlink_highres_imu_t
{
 uint64_t time_usec; ///< Timestamp (microseconds, synced to UNIX time or since system boot)
 float xacc; ///< X acceleration (m/s^2)
 float yacc; ///< Y acceleration (m/s^2)
 float zacc; ///< Z acceleration (m/s^2)
 float xgyro; ///< Angular speed around X axis (rad / sec)
 float ygyro; ///< Angular speed around Y axis (rad / sec)
 float zgyro; ///< Angular speed around Z axis (rad / sec)
 float xmag; ///< X Magnetic field (Gauss)
 float ymag; ///< Y Magnetic field (Gauss)
 float zmag; ///< Z Magnetic field (Gauss)
 float abs_pressure; ///< Absolute pressure in millibar
 float diff_pressure; ///< Differential pressure in millibar
 float pressure_alt; ///< Altitude calculated from pressure
 float temperature; ///< Temperature in degrees celsius
 uint16_t fields_updated; ///< Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
} mavlink_highres_imu_t;

#define MAVLINK_MSG_ID_HIGHRES_IMU_LEN 62
#define MAVLINK_MSG_ID_105_LEN 62



#define MAVLINK_MESSAGE_INFO_HIGHRES_IMU { \
	"HIGHRES_IMU", \
	15, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_highres_imu_t, time_usec) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_highres_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_highres_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_highres_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_highres_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_highres_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_highres_imu_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_highres_imu_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_highres_imu_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_highres_imu_t, zmag) }, \
         { "abs_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_highres_imu_t, abs_pressure) }, \
         { "diff_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_highres_imu_t, diff_pressure) }, \
         { "pressure_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_highres_imu_t, pressure_alt) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_highres_imu_t, temperature) }, \
         { "fields_updated", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_highres_imu_t, fields_updated) }, \
         } \
}


/**
 * @brief Pack a highres_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param xgyro Angular speed around X axis (rad / sec)
 * @param ygyro Angular speed around Y axis (rad / sec)
 * @param zgyro Angular speed around Z axis (rad / sec)
 * @param xmag X Magnetic field (Gauss)
 * @param ymag Y Magnetic field (Gauss)
 * @param zmag Z Magnetic field (Gauss)
 * @param abs_pressure Absolute pressure in millibar
 * @param diff_pressure Differential pressure in millibar
 * @param pressure_alt Altitude calculated from pressure
 * @param temperature Temperature in degrees celsius
 * @param fields_updated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_highres_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint16_t fields_updated)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[62];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, xacc);
	_mav_put_float(buf, 12, yacc);
	_mav_put_float(buf, 16, zacc);
	_mav_put_float(buf, 20, xgyro);
	_mav_put_float(buf, 24, ygyro);
	_mav_put_float(buf, 28, zgyro);
	_mav_put_float(buf, 32, xmag);
	_mav_put_float(buf, 36, ymag);
	_mav_put_float(buf, 40, zmag);
	_mav_put_float(buf, 44, abs_pressure);
	_mav_put_float(buf, 48, diff_pressure);
	_mav_put_float(buf, 52, pressure_alt);
	_mav_put_float(buf, 56, temperature);
	_mav_put_uint16_t(buf, 60, fields_updated);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 62);
#else
	mavlink_highres_imu_t packet;
	packet.time_usec = time_usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;
	packet.abs_pressure = abs_pressure;
	packet.diff_pressure = diff_pressure;
	packet.pressure_alt = pressure_alt;
	packet.temperature = temperature;
	packet.fields_updated = fields_updated;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 62);
#endif

	msg->msgid = MAVLINK_MSG_ID_HIGHRES_IMU;
	return mavlink_finalize_message(msg, system_id, component_id, 62, 93);
}

/**
 * @brief Pack a highres_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param xgyro Angular speed around X axis (rad / sec)
 * @param ygyro Angular speed around Y axis (rad / sec)
 * @param zgyro Angular speed around Z axis (rad / sec)
 * @param xmag X Magnetic field (Gauss)
 * @param ymag Y Magnetic field (Gauss)
 * @param zmag Z Magnetic field (Gauss)
 * @param abs_pressure Absolute pressure in millibar
 * @param diff_pressure Differential pressure in millibar
 * @param pressure_alt Altitude calculated from pressure
 * @param temperature Temperature in degrees celsius
 * @param fields_updated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_highres_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint16_t fields_updated)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[62];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, xacc);
	_mav_put_float(buf, 12, yacc);
	_mav_put_float(buf, 16, zacc);
	_mav_put_float(buf, 20, xgyro);
	_mav_put_float(buf, 24, ygyro);
	_mav_put_float(buf, 28, zgyro);
	_mav_put_float(buf, 32, xmag);
	_mav_put_float(buf, 36, ymag);
	_mav_put_float(buf, 40, zmag);
	_mav_put_float(buf, 44, abs_pressure);
	_mav_put_float(buf, 48, diff_pressure);
	_mav_put_float(buf, 52, pressure_alt);
	_mav_put_float(buf, 56, temperature);
	_mav_put_uint16_t(buf, 60, fields_updated);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 62);
#else
	mavlink_highres_imu_t packet;
	packet.time_usec = time_usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;
	packet.abs_pressure = abs_pressure;
	packet.diff_pressure = diff_pressure;
	packet.pressure_alt = pressure_alt;
	packet.temperature = temperature;
	packet.fields_updated = fields_updated;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 62);
#endif

	msg->msgid = MAVLINK_MSG_ID_HIGHRES_IMU;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 62, 93);
}

/**
 * @brief Encode a highres_imu struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param highres_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_highres_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_highres_imu_t* highres_imu)
{
	return mavlink_msg_highres_imu_pack(system_id, component_id, msg, highres_imu->time_usec, highres_imu->xacc, highres_imu->yacc, highres_imu->zacc, highres_imu->xgyro, highres_imu->ygyro, highres_imu->zgyro, highres_imu->xmag, highres_imu->ymag, highres_imu->zmag, highres_imu->abs_pressure, highres_imu->diff_pressure, highres_imu->pressure_alt, highres_imu->temperature, highres_imu->fields_updated);
}

/**
 * @brief Send a highres_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param xacc X acceleration (m/s^2)
 * @param yacc Y acceleration (m/s^2)
 * @param zacc Z acceleration (m/s^2)
 * @param xgyro Angular speed around X axis (rad / sec)
 * @param ygyro Angular speed around Y axis (rad / sec)
 * @param zgyro Angular speed around Z axis (rad / sec)
 * @param xmag X Magnetic field (Gauss)
 * @param ymag Y Magnetic field (Gauss)
 * @param zmag Z Magnetic field (Gauss)
 * @param abs_pressure Absolute pressure in millibar
 * @param diff_pressure Differential pressure in millibar
 * @param pressure_alt Altitude calculated from pressure
 * @param temperature Temperature in degrees celsius
 * @param fields_updated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_highres_imu_send(mavlink_channel_t chan, uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint16_t fields_updated)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[62];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, xacc);
	_mav_put_float(buf, 12, yacc);
	_mav_put_float(buf, 16, zacc);
	_mav_put_float(buf, 20, xgyro);
	_mav_put_float(buf, 24, ygyro);
	_mav_put_float(buf, 28, zgyro);
	_mav_put_float(buf, 32, xmag);
	_mav_put_float(buf, 36, ymag);
	_mav_put_float(buf, 40, zmag);
	_mav_put_float(buf, 44, abs_pressure);
	_mav_put_float(buf, 48, diff_pressure);
	_mav_put_float(buf, 52, pressure_alt);
	_mav_put_float(buf, 56, temperature);
	_mav_put_uint16_t(buf, 60, fields_updated);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGHRES_IMU, buf, 62, 93);
#else
	mavlink_highres_imu_t packet;
	packet.time_usec = time_usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;
	packet.abs_pressure = abs_pressure;
	packet.diff_pressure = diff_pressure;
	packet.pressure_alt = pressure_alt;
	packet.temperature = temperature;
	packet.fields_updated = fields_updated;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIGHRES_IMU, (const char *)&packet, 62, 93);
#endif
}

#endif

// MESSAGE HIGHRES_IMU UNPACKING


/**
 * @brief Get field time_usec from highres_imu message
 *
 * @return Timestamp (microseconds, synced to UNIX time or since system boot)
 */
static inline uint64_t mavlink_msg_highres_imu_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field xacc from highres_imu message
 *
 * @return X acceleration (m/s^2)
 */
static inline float mavlink_msg_highres_imu_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yacc from highres_imu message
 *
 * @return Y acceleration (m/s^2)
 */
static inline float mavlink_msg_highres_imu_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field zacc from highres_imu message
 *
 * @return Z acceleration (m/s^2)
 */
static inline float mavlink_msg_highres_imu_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field xgyro from highres_imu message
 *
 * @return Angular speed around X axis (rad / sec)
 */
static inline float mavlink_msg_highres_imu_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ygyro from highres_imu message
 *
 * @return Angular speed around Y axis (rad / sec)
 */
static inline float mavlink_msg_highres_imu_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field zgyro from highres_imu message
 *
 * @return Angular speed around Z axis (rad / sec)
 */
static inline float mavlink_msg_highres_imu_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field xmag from highres_imu message
 *
 * @return X Magnetic field (Gauss)
 */
static inline float mavlink_msg_highres_imu_get_xmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field ymag from highres_imu message
 *
 * @return Y Magnetic field (Gauss)
 */
static inline float mavlink_msg_highres_imu_get_ymag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field zmag from highres_imu message
 *
 * @return Z Magnetic field (Gauss)
 */
static inline float mavlink_msg_highres_imu_get_zmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field abs_pressure from highres_imu message
 *
 * @return Absolute pressure in millibar
 */
static inline float mavlink_msg_highres_imu_get_abs_pressure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field diff_pressure from highres_imu message
 *
 * @return Differential pressure in millibar
 */
static inline float mavlink_msg_highres_imu_get_diff_pressure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field pressure_alt from highres_imu message
 *
 * @return Altitude calculated from pressure
 */
static inline float mavlink_msg_highres_imu_get_pressure_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field temperature from highres_imu message
 *
 * @return Temperature in degrees celsius
 */
static inline float mavlink_msg_highres_imu_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field fields_updated from highres_imu message
 *
 * @return Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
 */
static inline uint16_t mavlink_msg_highres_imu_get_fields_updated(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  60);
}

/**
 * @brief Decode a highres_imu message into a struct
 *
 * @param msg The message to decode
 * @param highres_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_highres_imu_decode(const mavlink_message_t* msg, mavlink_highres_imu_t* highres_imu)
{
#if MAVLINK_NEED_BYTE_SWAP
	highres_imu->time_usec = mavlink_msg_highres_imu_get_time_usec(msg);
	highres_imu->xacc = mavlink_msg_highres_imu_get_xacc(msg);
	highres_imu->yacc = mavlink_msg_highres_imu_get_yacc(msg);
	highres_imu->zacc = mavlink_msg_highres_imu_get_zacc(msg);
	highres_imu->xgyro = mavlink_msg_highres_imu_get_xgyro(msg);
	highres_imu->ygyro = mavlink_msg_highres_imu_get_ygyro(msg);
	highres_imu->zgyro = mavlink_msg_highres_imu_get_zgyro(msg);
	highres_imu->xmag = mavlink_msg_highres_imu_get_xmag(msg);
	highres_imu->ymag = mavlink_msg_highres_imu_get_ymag(msg);
	highres_imu->zmag = mavlink_msg_highres_imu_get_zmag(msg);
	highres_imu->abs_pressure = mavlink_msg_highres_imu_get_abs_pressure(msg);
	highres_imu->diff_pressure = mavlink_msg_highres_imu_get_diff_pressure(msg);
	highres_imu->pressure_alt = mavlink_msg_highres_imu_get_pressure_alt(msg);
	highres_imu->temperature = mavlink_msg_highres_imu_get_temperature(msg);
	highres_imu->fields_updated = mavlink_msg_highres_imu_get_fields_updated(msg);
#else
	memcpy(highres_imu, _MAV_PAYLOAD(msg), 62);
#endif
}
