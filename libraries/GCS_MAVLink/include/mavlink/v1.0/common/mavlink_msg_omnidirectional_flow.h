// MESSAGE OMNIDIRECTIONAL_FLOW PACKING

#define MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW 106

typedef struct __mavlink_omnidirectional_flow_t
{
 uint64_t time_usec; ///< Timestamp (microseconds, synced to UNIX time or since system boot)
 float front_distance_m; ///< Front distance in meters. Positive value (including zero): distance known. Negative value: Unknown distance
 int16_t left[10]; ///< Flow in deci pixels (1 = 0.1 pixel) on left hemisphere
 int16_t right[10]; ///< Flow in deci pixels (1 = 0.1 pixel) on right hemisphere
 uint8_t sensor_id; ///< Sensor ID
 uint8_t quality; ///< Optical flow quality / confidence. 0: bad, 255: maximum quality
} mavlink_omnidirectional_flow_t;

#define MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN 54
#define MAVLINK_MSG_ID_106_LEN 54

#define MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_CRC 211
#define MAVLINK_MSG_ID_106_CRC 211

#define MAVLINK_MSG_OMNIDIRECTIONAL_FLOW_FIELD_LEFT_LEN 10
#define MAVLINK_MSG_OMNIDIRECTIONAL_FLOW_FIELD_RIGHT_LEN 10

#define MAVLINK_MESSAGE_INFO_OMNIDIRECTIONAL_FLOW { \
	"OMNIDIRECTIONAL_FLOW", \
	6, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_omnidirectional_flow_t, time_usec) }, \
         { "front_distance_m", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_omnidirectional_flow_t, front_distance_m) }, \
         { "left", NULL, MAVLINK_TYPE_INT16_T, 10, 12, offsetof(mavlink_omnidirectional_flow_t, left) }, \
         { "right", NULL, MAVLINK_TYPE_INT16_T, 10, 32, offsetof(mavlink_omnidirectional_flow_t, right) }, \
         { "sensor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_omnidirectional_flow_t, sensor_id) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_omnidirectional_flow_t, quality) }, \
         } \
}


/**
 * @brief Pack a omnidirectional_flow message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param sensor_id Sensor ID
 * @param left Flow in deci pixels (1 = 0.1 pixel) on left hemisphere
 * @param right Flow in deci pixels (1 = 0.1 pixel) on right hemisphere
 * @param quality Optical flow quality / confidence. 0: bad, 255: maximum quality
 * @param front_distance_m Front distance in meters. Positive value (including zero): distance known. Negative value: Unknown distance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_omnidirectional_flow_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t sensor_id, const int16_t *left, const int16_t *right, uint8_t quality, float front_distance_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, front_distance_m);
	_mav_put_uint8_t(buf, 52, sensor_id);
	_mav_put_uint8_t(buf, 53, quality);
	_mav_put_int16_t_array(buf, 12, left, 10);
	_mav_put_int16_t_array(buf, 32, right, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN);
#else
	mavlink_omnidirectional_flow_t packet;
	packet.time_usec = time_usec;
	packet.front_distance_m = front_distance_m;
	packet.sensor_id = sensor_id;
	packet.quality = quality;
	mav_array_memcpy(packet.left, left, sizeof(int16_t)*10);
	mav_array_memcpy(packet.right, right, sizeof(int16_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN);
#endif
}

/**
 * @brief Pack a omnidirectional_flow message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param sensor_id Sensor ID
 * @param left Flow in deci pixels (1 = 0.1 pixel) on left hemisphere
 * @param right Flow in deci pixels (1 = 0.1 pixel) on right hemisphere
 * @param quality Optical flow quality / confidence. 0: bad, 255: maximum quality
 * @param front_distance_m Front distance in meters. Positive value (including zero): distance known. Negative value: Unknown distance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_omnidirectional_flow_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t sensor_id,const int16_t *left,const int16_t *right,uint8_t quality,float front_distance_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, front_distance_m);
	_mav_put_uint8_t(buf, 52, sensor_id);
	_mav_put_uint8_t(buf, 53, quality);
	_mav_put_int16_t_array(buf, 12, left, 10);
	_mav_put_int16_t_array(buf, 32, right, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN);
#else
	mavlink_omnidirectional_flow_t packet;
	packet.time_usec = time_usec;
	packet.front_distance_m = front_distance_m;
	packet.sensor_id = sensor_id;
	packet.quality = quality;
	mav_array_memcpy(packet.left, left, sizeof(int16_t)*10);
	mav_array_memcpy(packet.right, right, sizeof(int16_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN);
#endif
}

/**
 * @brief Encode a omnidirectional_flow struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param omnidirectional_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_omnidirectional_flow_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_omnidirectional_flow_t* omnidirectional_flow)
{
	return mavlink_msg_omnidirectional_flow_pack(system_id, component_id, msg, omnidirectional_flow->time_usec, omnidirectional_flow->sensor_id, omnidirectional_flow->left, omnidirectional_flow->right, omnidirectional_flow->quality, omnidirectional_flow->front_distance_m);
}

/**
 * @brief Send a omnidirectional_flow message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
 * @param sensor_id Sensor ID
 * @param left Flow in deci pixels (1 = 0.1 pixel) on left hemisphere
 * @param right Flow in deci pixels (1 = 0.1 pixel) on right hemisphere
 * @param quality Optical flow quality / confidence. 0: bad, 255: maximum quality
 * @param front_distance_m Front distance in meters. Positive value (including zero): distance known. Negative value: Unknown distance
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_omnidirectional_flow_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t sensor_id, const int16_t *left, const int16_t *right, uint8_t quality, float front_distance_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, front_distance_m);
	_mav_put_uint8_t(buf, 52, sensor_id);
	_mav_put_uint8_t(buf, 53, quality);
	_mav_put_int16_t_array(buf, 12, left, 10);
	_mav_put_int16_t_array(buf, 32, right, 10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW, buf, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW, buf, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN);
#endif
#else
	mavlink_omnidirectional_flow_t packet;
	packet.time_usec = time_usec;
	packet.front_distance_m = front_distance_m;
	packet.sensor_id = sensor_id;
	packet.quality = quality;
	mav_array_memcpy(packet.left, left, sizeof(int16_t)*10);
	mav_array_memcpy(packet.right, right, sizeof(int16_t)*10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW, (const char *)&packet, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW, (const char *)&packet, MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN);
#endif
#endif
}

#endif

// MESSAGE OMNIDIRECTIONAL_FLOW UNPACKING


/**
 * @brief Get field time_usec from omnidirectional_flow message
 *
 * @return Timestamp (microseconds, synced to UNIX time or since system boot)
 */
static inline uint64_t mavlink_msg_omnidirectional_flow_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_id from omnidirectional_flow message
 *
 * @return Sensor ID
 */
static inline uint8_t mavlink_msg_omnidirectional_flow_get_sensor_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field left from omnidirectional_flow message
 *
 * @return Flow in deci pixels (1 = 0.1 pixel) on left hemisphere
 */
static inline uint16_t mavlink_msg_omnidirectional_flow_get_left(const mavlink_message_t* msg, int16_t *left)
{
	return _MAV_RETURN_int16_t_array(msg, left, 10,  12);
}

/**
 * @brief Get field right from omnidirectional_flow message
 *
 * @return Flow in deci pixels (1 = 0.1 pixel) on right hemisphere
 */
static inline uint16_t mavlink_msg_omnidirectional_flow_get_right(const mavlink_message_t* msg, int16_t *right)
{
	return _MAV_RETURN_int16_t_array(msg, right, 10,  32);
}

/**
 * @brief Get field quality from omnidirectional_flow message
 *
 * @return Optical flow quality / confidence. 0: bad, 255: maximum quality
 */
static inline uint8_t mavlink_msg_omnidirectional_flow_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Get field front_distance_m from omnidirectional_flow message
 *
 * @return Front distance in meters. Positive value (including zero): distance known. Negative value: Unknown distance
 */
static inline float mavlink_msg_omnidirectional_flow_get_front_distance_m(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a omnidirectional_flow message into a struct
 *
 * @param msg The message to decode
 * @param omnidirectional_flow C-struct to decode the message contents into
 */
static inline void mavlink_msg_omnidirectional_flow_decode(const mavlink_message_t* msg, mavlink_omnidirectional_flow_t* omnidirectional_flow)
{
#if MAVLINK_NEED_BYTE_SWAP
	omnidirectional_flow->time_usec = mavlink_msg_omnidirectional_flow_get_time_usec(msg);
	omnidirectional_flow->front_distance_m = mavlink_msg_omnidirectional_flow_get_front_distance_m(msg);
	mavlink_msg_omnidirectional_flow_get_left(msg, omnidirectional_flow->left);
	mavlink_msg_omnidirectional_flow_get_right(msg, omnidirectional_flow->right);
	omnidirectional_flow->sensor_id = mavlink_msg_omnidirectional_flow_get_sensor_id(msg);
	omnidirectional_flow->quality = mavlink_msg_omnidirectional_flow_get_quality(msg);
#else
	memcpy(omnidirectional_flow, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW_LEN);
#endif
}
