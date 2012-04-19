// MESSAGE SET_QUAD_MOTORS_SETPOINT PACKING

#define MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT 60

typedef struct __mavlink_set_quad_motors_setpoint_t
{
 uint16_t motor_front_nw; ///< Front motor in + configuration, front left motor in x configuration
 uint16_t motor_right_ne; ///< Right motor in + configuration, front right motor in x configuration
 uint16_t motor_back_se; ///< Back motor in + configuration, back right motor in x configuration
 uint16_t motor_left_sw; ///< Left motor in + configuration, back left motor in x configuration
 uint8_t target_system; ///< System ID of the system that should set these motor commands
} mavlink_set_quad_motors_setpoint_t;

#define MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT_LEN 9
#define MAVLINK_MSG_ID_60_LEN 9



#define MAVLINK_MESSAGE_INFO_SET_QUAD_MOTORS_SETPOINT { \
	"SET_QUAD_MOTORS_SETPOINT", \
	5, \
	{  { "motor_front_nw", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_set_quad_motors_setpoint_t, motor_front_nw) }, \
         { "motor_right_ne", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_set_quad_motors_setpoint_t, motor_right_ne) }, \
         { "motor_back_se", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_set_quad_motors_setpoint_t, motor_back_se) }, \
         { "motor_left_sw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_set_quad_motors_setpoint_t, motor_left_sw) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_set_quad_motors_setpoint_t, target_system) }, \
         } \
}


/**
 * @brief Pack a set_quad_motors_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID of the system that should set these motor commands
 * @param motor_front_nw Front motor in + configuration, front left motor in x configuration
 * @param motor_right_ne Right motor in + configuration, front right motor in x configuration
 * @param motor_back_se Back motor in + configuration, back right motor in x configuration
 * @param motor_left_sw Left motor in + configuration, back left motor in x configuration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_quad_motors_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint16_t motor_front_nw, uint16_t motor_right_ne, uint16_t motor_back_se, uint16_t motor_left_sw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_uint16_t(buf, 0, motor_front_nw);
	_mav_put_uint16_t(buf, 2, motor_right_ne);
	_mav_put_uint16_t(buf, 4, motor_back_se);
	_mav_put_uint16_t(buf, 6, motor_left_sw);
	_mav_put_uint8_t(buf, 8, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 9);
#else
	mavlink_set_quad_motors_setpoint_t packet;
	packet.motor_front_nw = motor_front_nw;
	packet.motor_right_ne = motor_right_ne;
	packet.motor_back_se = motor_back_se;
	packet.motor_left_sw = motor_left_sw;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT;
	return mavlink_finalize_message(msg, system_id, component_id, 9, 30);
}

/**
 * @brief Pack a set_quad_motors_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID of the system that should set these motor commands
 * @param motor_front_nw Front motor in + configuration, front left motor in x configuration
 * @param motor_right_ne Right motor in + configuration, front right motor in x configuration
 * @param motor_back_se Back motor in + configuration, back right motor in x configuration
 * @param motor_left_sw Left motor in + configuration, back left motor in x configuration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_quad_motors_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint16_t motor_front_nw,uint16_t motor_right_ne,uint16_t motor_back_se,uint16_t motor_left_sw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_uint16_t(buf, 0, motor_front_nw);
	_mav_put_uint16_t(buf, 2, motor_right_ne);
	_mav_put_uint16_t(buf, 4, motor_back_se);
	_mav_put_uint16_t(buf, 6, motor_left_sw);
	_mav_put_uint8_t(buf, 8, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 9);
#else
	mavlink_set_quad_motors_setpoint_t packet;
	packet.motor_front_nw = motor_front_nw;
	packet.motor_right_ne = motor_right_ne;
	packet.motor_back_se = motor_back_se;
	packet.motor_left_sw = motor_left_sw;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 9, 30);
}

/**
 * @brief Encode a set_quad_motors_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_quad_motors_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_quad_motors_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_quad_motors_setpoint_t* set_quad_motors_setpoint)
{
	return mavlink_msg_set_quad_motors_setpoint_pack(system_id, component_id, msg, set_quad_motors_setpoint->target_system, set_quad_motors_setpoint->motor_front_nw, set_quad_motors_setpoint->motor_right_ne, set_quad_motors_setpoint->motor_back_se, set_quad_motors_setpoint->motor_left_sw);
}

/**
 * @brief Send a set_quad_motors_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID of the system that should set these motor commands
 * @param motor_front_nw Front motor in + configuration, front left motor in x configuration
 * @param motor_right_ne Right motor in + configuration, front right motor in x configuration
 * @param motor_back_se Back motor in + configuration, back right motor in x configuration
 * @param motor_left_sw Left motor in + configuration, back left motor in x configuration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_quad_motors_setpoint_send(mavlink_channel_t chan, uint8_t target_system, uint16_t motor_front_nw, uint16_t motor_right_ne, uint16_t motor_back_se, uint16_t motor_left_sw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_uint16_t(buf, 0, motor_front_nw);
	_mav_put_uint16_t(buf, 2, motor_right_ne);
	_mav_put_uint16_t(buf, 4, motor_back_se);
	_mav_put_uint16_t(buf, 6, motor_left_sw);
	_mav_put_uint8_t(buf, 8, target_system);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT, buf, 9, 30);
#else
	mavlink_set_quad_motors_setpoint_t packet;
	packet.motor_front_nw = motor_front_nw;
	packet.motor_right_ne = motor_right_ne;
	packet.motor_back_se = motor_back_se;
	packet.motor_left_sw = motor_left_sw;
	packet.target_system = target_system;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT, (const char *)&packet, 9, 30);
#endif
}

#endif

// MESSAGE SET_QUAD_MOTORS_SETPOINT UNPACKING


/**
 * @brief Get field target_system from set_quad_motors_setpoint message
 *
 * @return System ID of the system that should set these motor commands
 */
static inline uint8_t mavlink_msg_set_quad_motors_setpoint_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field motor_front_nw from set_quad_motors_setpoint message
 *
 * @return Front motor in + configuration, front left motor in x configuration
 */
static inline uint16_t mavlink_msg_set_quad_motors_setpoint_get_motor_front_nw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field motor_right_ne from set_quad_motors_setpoint message
 *
 * @return Right motor in + configuration, front right motor in x configuration
 */
static inline uint16_t mavlink_msg_set_quad_motors_setpoint_get_motor_right_ne(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field motor_back_se from set_quad_motors_setpoint message
 *
 * @return Back motor in + configuration, back right motor in x configuration
 */
static inline uint16_t mavlink_msg_set_quad_motors_setpoint_get_motor_back_se(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field motor_left_sw from set_quad_motors_setpoint message
 *
 * @return Left motor in + configuration, back left motor in x configuration
 */
static inline uint16_t mavlink_msg_set_quad_motors_setpoint_get_motor_left_sw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a set_quad_motors_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param set_quad_motors_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_quad_motors_setpoint_decode(const mavlink_message_t* msg, mavlink_set_quad_motors_setpoint_t* set_quad_motors_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_quad_motors_setpoint->motor_front_nw = mavlink_msg_set_quad_motors_setpoint_get_motor_front_nw(msg);
	set_quad_motors_setpoint->motor_right_ne = mavlink_msg_set_quad_motors_setpoint_get_motor_right_ne(msg);
	set_quad_motors_setpoint->motor_back_se = mavlink_msg_set_quad_motors_setpoint_get_motor_back_se(msg);
	set_quad_motors_setpoint->motor_left_sw = mavlink_msg_set_quad_motors_setpoint_get_motor_left_sw(msg);
	set_quad_motors_setpoint->target_system = mavlink_msg_set_quad_motors_setpoint_get_target_system(msg);
#else
	memcpy(set_quad_motors_setpoint, _MAV_PAYLOAD(msg), 9);
#endif
}
