// MESSAGE EKF PACKING

#define MAVLINK_MSG_ID_EKF 178

typedef struct __mavlink_ekf_t
{
 float roll; ///< Roll angle (rad)
 float pitch; ///< Pitch angle (rad)
 float yaw; ///< Yaw angle (rad)
 float altitude; ///< Altitude (MSL)
 int32_t lat; ///< Latitude in degrees * 1E7
 int32_t lng; ///< Longitude in degrees * 1E7
} mavlink_ekf_t;

#define MAVLINK_MSG_ID_EKF_LEN 24
#define MAVLINK_MSG_ID_178_LEN 24

#define MAVLINK_MSG_ID_EKF_CRC 194
#define MAVLINK_MSG_ID_178_CRC 194



#define MAVLINK_MESSAGE_INFO_EKF { \
	"EKF", \
	6, \
	{  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ekf_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ekf_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ekf_t, yaw) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ekf_t, altitude) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_ekf_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_ekf_t, lng) }, \
         } \
}


/**
 * @brief Pack a ekf message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param altitude Altitude (MSL)
 * @param lat Latitude in degrees * 1E7
 * @param lng Longitude in degrees * 1E7
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ekf_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EKF_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lng);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EKF_LEN);
#else
	mavlink_ekf_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.altitude = altitude;
	packet.lat = lat;
	packet.lng = lng;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EKF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EKF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EKF_LEN, MAVLINK_MSG_ID_EKF_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EKF_LEN);
#endif
}

/**
 * @brief Pack a ekf message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param altitude Altitude (MSL)
 * @param lat Latitude in degrees * 1E7
 * @param lng Longitude in degrees * 1E7
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ekf_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float roll,float pitch,float yaw,float altitude,int32_t lat,int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EKF_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lng);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EKF_LEN);
#else
	mavlink_ekf_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.altitude = altitude;
	packet.lat = lat;
	packet.lng = lng;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EKF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EKF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EKF_LEN, MAVLINK_MSG_ID_EKF_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EKF_LEN);
#endif
}

/**
 * @brief Encode a ekf struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ekf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ekf_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ekf_t* ekf)
{
	return mavlink_msg_ekf_pack(system_id, component_id, msg, ekf->roll, ekf->pitch, ekf->yaw, ekf->altitude, ekf->lat, ekf->lng);
}

/**
 * @brief Encode a ekf struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ekf C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ekf_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ekf_t* ekf)
{
	return mavlink_msg_ekf_pack_chan(system_id, component_id, chan, msg, ekf->roll, ekf->pitch, ekf->yaw, ekf->altitude, ekf->lat, ekf->lng);
}

/**
 * @brief Send a ekf message
 * @param chan MAVLink channel to send the message
 *
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param altitude Altitude (MSL)
 * @param lat Latitude in degrees * 1E7
 * @param lng Longitude in degrees * 1E7
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ekf_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float altitude, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EKF_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, lat);
	_mav_put_int32_t(buf, 20, lng);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF, buf, MAVLINK_MSG_ID_EKF_LEN, MAVLINK_MSG_ID_EKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF, buf, MAVLINK_MSG_ID_EKF_LEN);
#endif
#else
	mavlink_ekf_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.altitude = altitude;
	packet.lat = lat;
	packet.lng = lng;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF, (const char *)&packet, MAVLINK_MSG_ID_EKF_LEN, MAVLINK_MSG_ID_EKF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF, (const char *)&packet, MAVLINK_MSG_ID_EKF_LEN);
#endif
#endif
}

#endif

// MESSAGE EKF UNPACKING


/**
 * @brief Get field roll from ekf message
 *
 * @return Roll angle (rad)
 */
static inline float mavlink_msg_ekf_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from ekf message
 *
 * @return Pitch angle (rad)
 */
static inline float mavlink_msg_ekf_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from ekf message
 *
 * @return Yaw angle (rad)
 */
static inline float mavlink_msg_ekf_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field altitude from ekf message
 *
 * @return Altitude (MSL)
 */
static inline float mavlink_msg_ekf_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field lat from ekf message
 *
 * @return Latitude in degrees * 1E7
 */
static inline int32_t mavlink_msg_ekf_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field lng from ekf message
 *
 * @return Longitude in degrees * 1E7
 */
static inline int32_t mavlink_msg_ekf_get_lng(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Decode a ekf message into a struct
 *
 * @param msg The message to decode
 * @param ekf C-struct to decode the message contents into
 */
static inline void mavlink_msg_ekf_decode(const mavlink_message_t* msg, mavlink_ekf_t* ekf)
{
#if MAVLINK_NEED_BYTE_SWAP
	ekf->roll = mavlink_msg_ekf_get_roll(msg);
	ekf->pitch = mavlink_msg_ekf_get_pitch(msg);
	ekf->yaw = mavlink_msg_ekf_get_yaw(msg);
	ekf->altitude = mavlink_msg_ekf_get_altitude(msg);
	ekf->lat = mavlink_msg_ekf_get_lat(msg);
	ekf->lng = mavlink_msg_ekf_get_lng(msg);
#else
	memcpy(ekf, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_EKF_LEN);
#endif
}
