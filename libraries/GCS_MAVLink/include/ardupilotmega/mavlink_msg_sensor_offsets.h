// MESSAGE SENSOR_OFFSETS PACKING

#define MAVLINK_MSG_ID_SENSOR_OFFSETS 150

typedef struct __mavlink_sensor_offsets_t 
{
	int16_t mag_ofs_x; ///< magnetometer X offset
	int16_t mag_ofs_y; ///< magnetometer Y offset
	int16_t mag_ofs_z; ///< magnetometer Z offset
	float mag_declination; ///< magnetic declination (radians)
	int32_t raw_press; ///< raw pressure from barometer
	int32_t raw_temp; ///< raw temperature from barometer
	float gyro_cal_x; ///< gyro X calibration
	float gyro_cal_y; ///< gyro Y calibration
	float gyro_cal_z; ///< gyro Z calibration
	float accel_cal_x; ///< accel X calibration
	float accel_cal_y; ///< accel Y calibration
	float accel_cal_z; ///< accel Z calibration

} mavlink_sensor_offsets_t;



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
static inline uint16_t mavlink_msg_sensor_offsets_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SENSOR_OFFSETS;

	i += put_int16_t_by_index(mag_ofs_x, i, msg->payload); // magnetometer X offset
	i += put_int16_t_by_index(mag_ofs_y, i, msg->payload); // magnetometer Y offset
	i += put_int16_t_by_index(mag_ofs_z, i, msg->payload); // magnetometer Z offset
	i += put_float_by_index(mag_declination, i, msg->payload); // magnetic declination (radians)
	i += put_int32_t_by_index(raw_press, i, msg->payload); // raw pressure from barometer
	i += put_int32_t_by_index(raw_temp, i, msg->payload); // raw temperature from barometer
	i += put_float_by_index(gyro_cal_x, i, msg->payload); // gyro X calibration
	i += put_float_by_index(gyro_cal_y, i, msg->payload); // gyro Y calibration
	i += put_float_by_index(gyro_cal_z, i, msg->payload); // gyro Z calibration
	i += put_float_by_index(accel_cal_x, i, msg->payload); // accel X calibration
	i += put_float_by_index(accel_cal_y, i, msg->payload); // accel Y calibration
	i += put_float_by_index(accel_cal_z, i, msg->payload); // accel Z calibration

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a sensor_offsets message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
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
static inline uint16_t mavlink_msg_sensor_offsets_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z, float mag_declination, int32_t raw_press, int32_t raw_temp, float gyro_cal_x, float gyro_cal_y, float gyro_cal_z, float accel_cal_x, float accel_cal_y, float accel_cal_z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SENSOR_OFFSETS;

	i += put_int16_t_by_index(mag_ofs_x, i, msg->payload); // magnetometer X offset
	i += put_int16_t_by_index(mag_ofs_y, i, msg->payload); // magnetometer Y offset
	i += put_int16_t_by_index(mag_ofs_z, i, msg->payload); // magnetometer Z offset
	i += put_float_by_index(mag_declination, i, msg->payload); // magnetic declination (radians)
	i += put_int32_t_by_index(raw_press, i, msg->payload); // raw pressure from barometer
	i += put_int32_t_by_index(raw_temp, i, msg->payload); // raw temperature from barometer
	i += put_float_by_index(gyro_cal_x, i, msg->payload); // gyro X calibration
	i += put_float_by_index(gyro_cal_y, i, msg->payload); // gyro Y calibration
	i += put_float_by_index(gyro_cal_z, i, msg->payload); // gyro Z calibration
	i += put_float_by_index(accel_cal_x, i, msg->payload); // accel X calibration
	i += put_float_by_index(accel_cal_y, i, msg->payload); // accel Y calibration
	i += put_float_by_index(accel_cal_z, i, msg->payload); // accel Z calibration

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a sensor_offsets struct into a message
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
	mavlink_message_t msg;
	mavlink_msg_sensor_offsets_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, mag_ofs_x, mag_ofs_y, mag_ofs_z, mag_declination, raw_press, raw_temp, gyro_cal_x, gyro_cal_y, gyro_cal_z, accel_cal_x, accel_cal_y, accel_cal_z);
	mavlink_send_uart(chan, &msg);
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
	generic_16bit r;
	r.b[1] = (msg->payload)[0];
	r.b[0] = (msg->payload)[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field mag_ofs_y from sensor_offsets message
 *
 * @return magnetometer Y offset
 */
static inline int16_t mavlink_msg_sensor_offsets_get_mag_ofs_y(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field mag_ofs_z from sensor_offsets message
 *
 * @return magnetometer Z offset
 */
static inline int16_t mavlink_msg_sensor_offsets_get_mag_ofs_z(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field mag_declination from sensor_offsets message
 *
 * @return magnetic declination (radians)
 */
static inline float mavlink_msg_sensor_offsets_get_mag_declination(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[2] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	r.b[1] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[2];
	r.b[0] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field raw_press from sensor_offsets message
 *
 * @return raw pressure from barometer
 */
static inline int32_t mavlink_msg_sensor_offsets_get_raw_press(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field raw_temp from sensor_offsets message
 *
 * @return raw temperature from barometer
 */
static inline int32_t mavlink_msg_sensor_offsets_get_raw_temp(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t))[0];
	r.b[2] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t))[1];
	r.b[1] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t))[2];
	r.b[0] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field gyro_cal_x from sensor_offsets message
 *
 * @return gyro X calibration
 */
static inline float mavlink_msg_sensor_offsets_get_gyro_cal_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t))[0];
	r.b[2] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t))[1];
	r.b[1] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t))[2];
	r.b[0] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field gyro_cal_y from sensor_offsets message
 *
 * @return gyro Y calibration
 */
static inline float mavlink_msg_sensor_offsets_get_gyro_cal_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field gyro_cal_z from sensor_offsets message
 *
 * @return gyro Z calibration
 */
static inline float mavlink_msg_sensor_offsets_get_gyro_cal_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field accel_cal_x from sensor_offsets message
 *
 * @return accel X calibration
 */
static inline float mavlink_msg_sensor_offsets_get_accel_cal_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field accel_cal_y from sensor_offsets message
 *
 * @return accel Y calibration
 */
static inline float mavlink_msg_sensor_offsets_get_accel_cal_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field accel_cal_z from sensor_offsets message
 *
 * @return accel Z calibration
 */
static inline float mavlink_msg_sensor_offsets_get_accel_cal_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a sensor_offsets message into a struct
 *
 * @param msg The message to decode
 * @param sensor_offsets C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_offsets_decode(const mavlink_message_t* msg, mavlink_sensor_offsets_t* sensor_offsets)
{
	sensor_offsets->mag_ofs_x = mavlink_msg_sensor_offsets_get_mag_ofs_x(msg);
	sensor_offsets->mag_ofs_y = mavlink_msg_sensor_offsets_get_mag_ofs_y(msg);
	sensor_offsets->mag_ofs_z = mavlink_msg_sensor_offsets_get_mag_ofs_z(msg);
	sensor_offsets->mag_declination = mavlink_msg_sensor_offsets_get_mag_declination(msg);
	sensor_offsets->raw_press = mavlink_msg_sensor_offsets_get_raw_press(msg);
	sensor_offsets->raw_temp = mavlink_msg_sensor_offsets_get_raw_temp(msg);
	sensor_offsets->gyro_cal_x = mavlink_msg_sensor_offsets_get_gyro_cal_x(msg);
	sensor_offsets->gyro_cal_y = mavlink_msg_sensor_offsets_get_gyro_cal_y(msg);
	sensor_offsets->gyro_cal_z = mavlink_msg_sensor_offsets_get_gyro_cal_z(msg);
	sensor_offsets->accel_cal_x = mavlink_msg_sensor_offsets_get_accel_cal_x(msg);
	sensor_offsets->accel_cal_y = mavlink_msg_sensor_offsets_get_accel_cal_y(msg);
	sensor_offsets->accel_cal_z = mavlink_msg_sensor_offsets_get_accel_cal_z(msg);
}
