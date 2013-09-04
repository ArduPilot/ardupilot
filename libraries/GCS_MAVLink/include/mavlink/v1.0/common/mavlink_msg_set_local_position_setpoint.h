// MESSAGE SET_LOCAL_POSITION_SETPOINT PACKING

#define MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT 50

typedef struct __mavlink_set_local_position_setpoint_t
{
 float x; ///< x position
 float y; ///< y position
 float z; ///< z position
 float yaw; ///< Desired yaw angle
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t coordinate_frame; ///< Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
} mavlink_set_local_position_setpoint_t;

#define MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN 19
#define MAVLINK_MSG_ID_50_LEN 19

#define MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_CRC 214
#define MAVLINK_MSG_ID_50_CRC 214



#define MAVLINK_MESSAGE_INFO_SET_LOCAL_POSITION_SETPOINT { \
	"SET_LOCAL_POSITION_SETPOINT", \
	7, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_local_position_setpoint_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_local_position_setpoint_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_local_position_setpoint_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_local_position_setpoint_t, yaw) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_set_local_position_setpoint_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_set_local_position_setpoint_t, target_component) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_set_local_position_setpoint_t, coordinate_frame) }, \
         } \
}


/**
 * @brief Pack a set_local_position_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_local_position_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);
	_mav_put_uint8_t(buf, 18, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN);
#else
	mavlink_set_local_position_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN);
#endif
}

/**
 * @brief Pack a set_local_position_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_local_position_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t coordinate_frame,float x,float y,float z,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);
	_mav_put_uint8_t(buf, 18, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN);
#else
	mavlink_set_local_position_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN);
#endif
}

/**
 * @brief Encode a set_local_position_setpoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_local_position_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_local_position_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_local_position_setpoint_t* set_local_position_setpoint)
{
	return mavlink_msg_set_local_position_setpoint_pack(system_id, component_id, msg, set_local_position_setpoint->target_system, set_local_position_setpoint->target_component, set_local_position_setpoint->coordinate_frame, set_local_position_setpoint->x, set_local_position_setpoint->y, set_local_position_setpoint->z, set_local_position_setpoint->yaw);
}

/**
 * @brief Encode a set_local_position_setpoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_local_position_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_local_position_setpoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_local_position_setpoint_t* set_local_position_setpoint)
{
	return mavlink_msg_set_local_position_setpoint_pack_chan(system_id, component_id, chan, msg, set_local_position_setpoint->target_system, set_local_position_setpoint->target_component, set_local_position_setpoint->coordinate_frame, set_local_position_setpoint->x, set_local_position_setpoint->y, set_local_position_setpoint->z, set_local_position_setpoint->yaw);
}

/**
 * @brief Send a set_local_position_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_local_position_setpoint_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, yaw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);
	_mav_put_uint8_t(buf, 18, coordinate_frame);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT, buf, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT, buf, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN);
#endif
#else
	mavlink_set_local_position_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT, (const char *)&packet, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT, (const char *)&packet, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN);
#endif
#endif
}

#endif

// MESSAGE SET_LOCAL_POSITION_SETPOINT UNPACKING


/**
 * @brief Get field target_system from set_local_position_setpoint message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_local_position_setpoint_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from set_local_position_setpoint message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_local_position_setpoint_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field coordinate_frame from set_local_position_setpoint message
 *
 * @return Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
 */
static inline uint8_t mavlink_msg_set_local_position_setpoint_get_coordinate_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field x from set_local_position_setpoint message
 *
 * @return x position
 */
static inline float mavlink_msg_set_local_position_setpoint_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from set_local_position_setpoint message
 *
 * @return y position
 */
static inline float mavlink_msg_set_local_position_setpoint_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from set_local_position_setpoint message
 *
 * @return z position
 */
static inline float mavlink_msg_set_local_position_setpoint_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from set_local_position_setpoint message
 *
 * @return Desired yaw angle
 */
static inline float mavlink_msg_set_local_position_setpoint_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a set_local_position_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param set_local_position_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_local_position_setpoint_decode(const mavlink_message_t* msg, mavlink_set_local_position_setpoint_t* set_local_position_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_local_position_setpoint->x = mavlink_msg_set_local_position_setpoint_get_x(msg);
	set_local_position_setpoint->y = mavlink_msg_set_local_position_setpoint_get_y(msg);
	set_local_position_setpoint->z = mavlink_msg_set_local_position_setpoint_get_z(msg);
	set_local_position_setpoint->yaw = mavlink_msg_set_local_position_setpoint_get_yaw(msg);
	set_local_position_setpoint->target_system = mavlink_msg_set_local_position_setpoint_get_target_system(msg);
	set_local_position_setpoint->target_component = mavlink_msg_set_local_position_setpoint_get_target_component(msg);
	set_local_position_setpoint->coordinate_frame = mavlink_msg_set_local_position_setpoint_get_coordinate_frame(msg);
#else
	memcpy(set_local_position_setpoint, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT_LEN);
#endif
}
