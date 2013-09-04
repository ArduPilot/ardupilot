// MESSAGE SENSOR_OFFSETS PACKING

#define MAVLINK_MSG_ID_SENSOR_OFFSETS 150

typedef struct __mavlink_sensor_offsets_t
{
 float mag_declination; ///< magnetic declination (radians)
 int32_t raw_press; ///< raw pressure from barometer
 int32_t raw_temp; ///< raw temperature from barometer
 float gyro_cal_x; ///< gyro X calibration
 float gyro_cal_y; ///< gyro Y calibration
 float gyro_cal_z; ///< gyro Z calibration
 float accel_cal_x; ///< accel X calibration
 float accel_cal_y; ///< accel Y calibration
 float accel_cal_z; ///< accel Z calibration
 int16_t mag_ofs_x; ///< magnetometer X offset
 int16_t mag_ofs_y; ///< magnetometer Y offset
 int16_t mag_ofs_z; ///< magnetometer Z offset
} mavlink_sensor_offsets_t;

#define MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN 42
#define MAVLINK_MSG_ID_150_LEN 42

#define MAVLINK_MSG_ID_SENSOR_OFFSETS_CRC 134
#define MAVLINK_MSG_ID_150_CRC 134



#define MAVLINK_MESSAGE_INFO_SENSOR_OFFSETS { \
	"SENSOR_OFFSETS", \
	12, \
	{  { "mag_declination", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sensor_offsets_t, mag_declination) }, \
         { "raw_press", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_sensor_offsets_t, raw_press) }, \
         { "raw_temp", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_sensor_offsets_t, raw_temp) }, \
         { "gyro_cal_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sensor_offsets_t, gyro_cal_x) }, \
         { "gyro_cal_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sensor_offsets_t, gyro_cal_y) }, \
         { "gyro_cal_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sensor_offsets_t, gyro_cal_z) }, \
         { "accel_cal_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sensor_offsets_t, accel_cal_x) }, \
         { "accel_cal_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sensor_offsets_t, accel_cal_y) }, \
         { "accel_cal_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sensor_offsets_t, accel_cal_z) }, \
         { "mag_ofs_x", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_sensor_offsets_t, mag_ofs_x) }, \
         { "mag_ofs_y", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_sensor_offsets_t, mag_ofs_y) }, \
         { "mag_ofs_z", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_sensor_offsets_t, mag_ofs_z) }, \
         } \
}


/**
 * @brief Pack a sensor_offsets message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @param mag_declination magnetic declination (radians)
 * @param raw_press raw pressure from barometer
 * @param raw_temp raw temperature from barometer
 * @param gyro_cal_x gyro X calibration
 * @param gyro_cal_y gyro Y calibration
 * @param gyro_cal_z gyro Z calibration
 * @param accel_cal_x accel X calibration
 * @param accel_cal_y accel Y calibration
 * @param accel_cal_z accel Z calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_offsets_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN];
	_mav_put_float(buf, 0, mag_declination);
	_mav_put_int32_t(buf, 4, raw_press);
	_mav_put_int32_t(buf, 8, raw_temp);
	_mav_put_float(buf, 12, gyro_cal_x);
	_mav_put_float(buf, 16, gyro_cal_y);
	_mav_put_float(buf, 20, gyro_cal_z);
	_mav_put_float(buf, 24, accel_cal_x);
	_mav_put_float(buf, 28, accel_cal_y);
	_mav_put_float(buf, 32, accel_cal_z);
	_mav_put_int16_t(buf, 36, mag_ofs_x);
	_mav_put_int16_t(buf, 38, mag_ofs_y);
	_mav_put_int16_t(buf, 40, mag_ofs_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN);
#else
	mavlink_sensor_offsets_t packet;
	packet.mag_declination = mag_declination;
	packet.raw_press = raw_press;
	packet.raw_temp = raw_temp;
	packet.gyro_cal_x = gyro_cal_x;
	packet.gyro_cal_y = gyro_cal_y;
	packet.gyro_cal_z = gyro_cal_z;
	packet.accel_cal_x = accel_cal_x;
	packet.accel_cal_y = accel_cal_y;
	packet.accel_cal_z = accel_cal_z;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SENSOR_OFFSETS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN, MAVLINK_MSG_ID_SENSOR_OFFSETS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN);
#endif
}

/**
 * @brief Pack a sensor_offsets message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @param mag_declination magnetic declination (radians)
 * @param raw_press raw pressure from barometer
 * @param raw_temp raw temperature from barometer
 * @param gyro_cal_x gyro X calibration
 * @param gyro_cal_y gyro Y calibration
 * @param gyro_cal_z gyro Z calibration
 * @param accel_cal_x accel X calibration
 * @param accel_cal_y accel Y calibration
 * @param accel_cal_z accel Z calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_offsets_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t mag_ofs_x,int16_t mag_ofs_y,int16_t mag_ofs_z,float mag_declination,int32_t raw_press,int32_t raw_temp,float gyro_cal_x,float gyro_cal_y,float gyro_cal_z,float accel_cal_x,float accel_cal_y,float accel_cal_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN];
	_mav_put_float(buf, 0, mag_declination);
	_mav_put_int32_t(buf, 4, raw_press);
	_mav_put_int32_t(buf, 8, raw_temp);
	_mav_put_float(buf, 12, gyro_cal_x);
	_mav_put_float(buf, 16, gyro_cal_y);
	_mav_put_float(buf, 20, gyro_cal_z);
	_mav_put_float(buf, 24, accel_cal_x);
	_mav_put_float(buf, 28, accel_cal_y);
	_mav_put_float(buf, 32, accel_cal_z);
	_mav_put_int16_t(buf, 36, mag_ofs_x);
	_mav_put_int16_t(buf, 38, mag_ofs_y);
	_mav_put_int16_t(buf, 40, mag_ofs_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN);
#else
	mavlink_sensor_offsets_t packet;
	packet.mag_declination = mag_declination;
	packet.raw_press = raw_press;
	packet.raw_temp = raw_temp;
	packet.gyro_cal_x = gyro_cal_x;
	packet.gyro_cal_y = gyro_cal_y;
	packet.gyro_cal_z = gyro_cal_z;
	packet.accel_cal_x = accel_cal_x;
	packet.accel_cal_y = accel_cal_y;
	packet.accel_cal_z = accel_cal_z;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SENSOR_OFFSETS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN, MAVLINK_MSG_ID_SENSOR_OFFSETS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN);
#endif
}

/**
 * @brief Encode a sensor_offsets struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_offsets C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_offsets_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_offsets_t* sensor_offsets)
{
	return mavlink_msg_sensor_offsets_pack(system_id, component_id, msg, sensor_offsets->mag_ofs_x, sensor_offsets->mag_ofs_y, sensor_offsets->mag_ofs_z, sensor_offsets->mag_declination, sensor_offsets->raw_press, sensor_offsets->raw_temp, sensor_offsets->gyro_cal_x, sensor_offsets->gyro_cal_y, sensor_offsets->gyro_cal_z, sensor_offsets->accel_cal_x, sensor_offsets->accel_cal_y, sensor_offsets->accel_cal_z);
}

/**
 * @brief Encode a sensor_offsets struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor_offsets C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_offsets_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_offsets_t* sensor_offsets)
{
	return mavlink_msg_sensor_offsets_pack_chan(system_id, component_id, chan, msg, sensor_offsets->mag_ofs_x, sensor_offsets->mag_ofs_y, sensor_offsets->mag_ofs_z, sensor_offsets->mag_declination, sensor_offsets->raw_press, sensor_offsets->raw_temp, sensor_offsets->gyro_cal_x, sensor_offsets->gyro_cal_y, sensor_offsets->gyro_cal_z, sensor_offsets->accel_cal_x, sensor_offsets->accel_cal_y, sensor_offsets->accel_cal_z);
}

/**
 * @brief Send a sensor_offsets message
 * @param chan MAVLink channel to send the message
 *
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @param mag_declination magnetic declination (radians)
 * @param raw_press raw pressure from barometer
 * @param raw_temp raw temperature from barometer
 * @param gyro_cal_x gyro X calibration
 * @param gyro_cal_y gyro Y calibration
 * @param gyro_cal_z gyro Z calibration
 * @param accel_cal_x accel X calibration
 * @param accel_cal_y accel Y calibration
 * @param accel_cal_z accel Z calibration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_offsets_send(mavlink_channel_t chan, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN];
	_mav_put_float(buf, 0, mag_declination);
	_mav_put_int32_t(buf, 4, raw_press);
	_mav_put_int32_t(buf, 8, raw_temp);
	_mav_put_float(buf, 12, gyro_cal_x);
	_mav_put_float(buf, 16, gyro_cal_y);
	_mav_put_float(buf, 20, gyro_cal_z);
	_mav_put_float(buf, 24, accel_cal_x);
	_mav_put_float(buf, 28, accel_cal_y);
	_mav_put_float(buf, 32, accel_cal_z);
	_mav_put_int16_t(buf, 36, mag_ofs_x);
	_mav_put_int16_t(buf, 38, mag_ofs_y);
	_mav_put_int16_t(buf, 40, mag_ofs_z);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_OFFSETS, buf, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN, MAVLINK_MSG_ID_SENSOR_OFFSETS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_OFFSETS, buf, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN);
#endif
#else
	mavlink_sensor_offsets_t packet;
	packet.mag_declination = mag_declination;
	packet.raw_press = raw_press;
	packet.raw_temp = raw_temp;
	packet.gyro_cal_x = gyro_cal_x;
	packet.gyro_cal_y = gyro_cal_y;
	packet.gyro_cal_z = gyro_cal_z;
	packet.accel_cal_x = accel_cal_x;
	packet.accel_cal_y = accel_cal_y;
	packet.accel_cal_z = accel_cal_z;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_OFFSETS, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN, MAVLINK_MSG_ID_SENSOR_OFFSETS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_OFFSETS, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN);
#endif
#endif
}

#endif

// MESSAGE SENSOR_OFFSETS UNPACKING


/**
 * @brief Get field mag_ofs_x from sensor_offsets message
 *
 * @return magnetometer X offset
 */
static inline int16_t mavlink_msg_sensor_offsets_get_mag_ofs_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  36);
}

/**
 * @brief Get field mag_ofs_y from sensor_offsets message
 *
 * @return magnetometer Y offset
 */
static inline int16_t mavlink_msg_sensor_offsets_get_mag_ofs_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field mag_ofs_z from sensor_offsets message
 *
 * @return magnetometer Z offset
 */
static inline int16_t mavlink_msg_sensor_offsets_get_mag_ofs_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Get field mag_declination from sensor_offsets message
 *
 * @return magnetic declination (radians)
 */
static inline float mavlink_msg_sensor_offsets_get_mag_declination(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field raw_press from sensor_offsets message
 *
 * @return raw pressure from barometer
 */
static inline int32_t mavlink_msg_sensor_offsets_get_raw_press(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field raw_temp from sensor_offsets message
 *
 * @return raw temperature from barometer
 */
static inline int32_t mavlink_msg_sensor_offsets_get_raw_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field gyro_cal_x from sensor_offsets message
 *
 * @return gyro X calibration
 */
static inline float mavlink_msg_sensor_offsets_get_gyro_cal_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyro_cal_y from sensor_offsets message
 *
 * @return gyro Y calibration
 */
static inline float mavlink_msg_sensor_offsets_get_gyro_cal_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyro_cal_z from sensor_offsets message
 *
 * @return gyro Z calibration
 */
static inline float mavlink_msg_sensor_offsets_get_gyro_cal_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field accel_cal_x from sensor_offsets message
 *
 * @return accel X calibration
 */
static inline float mavlink_msg_sensor_offsets_get_accel_cal_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field accel_cal_y from sensor_offsets message
 *
 * @return accel Y calibration
 */
static inline float mavlink_msg_sensor_offsets_get_accel_cal_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field accel_cal_z from sensor_offsets message
 *
 * @return accel Z calibration
 */
static inline float mavlink_msg_sensor_offsets_get_accel_cal_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a sensor_offsets message into a struct
 *
 * @param msg The message to decode
 * @param sensor_offsets C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_offsets_decode(const mavlink_message_t* msg, mavlink_sensor_offsets_t* sensor_offsets)
{
#if MAVLINK_NEED_BYTE_SWAP
	sensor_offsets->mag_declination = mavlink_msg_sensor_offsets_get_mag_declination(msg);
	sensor_offsets->raw_press = mavlink_msg_sensor_offsets_get_raw_press(msg);
	sensor_offsets->raw_temp = mavlink_msg_sensor_offsets_get_raw_temp(msg);
	sensor_offsets->gyro_cal_x = mavlink_msg_sensor_offsets_get_gyro_cal_x(msg);
	sensor_offsets->gyro_cal_y = mavlink_msg_sensor_offsets_get_gyro_cal_y(msg);
	sensor_offsets->gyro_cal_z = mavlink_msg_sensor_offsets_get_gyro_cal_z(msg);
	sensor_offsets->accel_cal_x = mavlink_msg_sensor_offsets_get_accel_cal_x(msg);
	sensor_offsets->accel_cal_y = mavlink_msg_sensor_offsets_get_accel_cal_y(msg);
	sensor_offsets->accel_cal_z = mavlink_msg_sensor_offsets_get_accel_cal_z(msg);
	sensor_offsets->mag_ofs_x = mavlink_msg_sensor_offsets_get_mag_ofs_x(msg);
	sensor_offsets->mag_ofs_y = mavlink_msg_sensor_offsets_get_mag_ofs_y(msg);
	sensor_offsets->mag_ofs_z = mavlink_msg_sensor_offsets_get_mag_ofs_z(msg);
#else
	memcpy(sensor_offsets, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SENSOR_OFFSETS_LEN);
#endif
}
