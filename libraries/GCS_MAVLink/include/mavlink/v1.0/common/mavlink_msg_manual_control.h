// MESSAGE MANUAL_CONTROL PACKING

#define MAVLINK_MSG_ID_MANUAL_CONTROL 69

typedef struct __mavlink_manual_control_t
{
 float roll; ///< roll
 float pitch; ///< pitch
 float yaw; ///< yaw
 float thrust; ///< thrust
 uint8_t target; ///< The system to be controlled
 uint8_t roll_manual; ///< roll control enabled auto:0, manual:1
 uint8_t pitch_manual; ///< pitch auto:0, manual:1
 uint8_t yaw_manual; ///< yaw auto:0, manual:1
 uint8_t thrust_manual; ///< thrust auto:0, manual:1
} mavlink_manual_control_t;

#define MAVLINK_MSG_ID_MANUAL_CONTROL_LEN 21
#define MAVLINK_MSG_ID_69_LEN 21



#define MAVLINK_MESSAGE_INFO_MANUAL_CONTROL { \
	"MANUAL_CONTROL", \
	9, \
	{  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_manual_control_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_manual_control_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_manual_control_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_manual_control_t, thrust) }, \
         { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_manual_control_t, target) }, \
         { "roll_manual", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_manual_control_t, roll_manual) }, \
         { "pitch_manual", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_manual_control_t, pitch_manual) }, \
         { "yaw_manual", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_manual_control_t, yaw_manual) }, \
         { "thrust_manual", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_manual_control_t, thrust_manual) }, \
         } \
}


/**
 * @brief Pack a manual_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system to be controlled
 * @param roll roll
 * @param pitch pitch
 * @param yaw yaw
 * @param thrust thrust
 * @param roll_manual roll control enabled auto:0, manual:1
 * @param pitch_manual pitch auto:0, manual:1
 * @param yaw_manual yaw auto:0, manual:1
 * @param thrust_manual thrust auto:0, manual:1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_manual_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target, float roll, float pitch, float yaw, float thrust, uint8_t roll_manual, uint8_t pitch_manual, uint8_t yaw_manual, uint8_t thrust_manual)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[21];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, thrust);
	_mav_put_uint8_t(buf, 16, target);
	_mav_put_uint8_t(buf, 17, roll_manual);
	_mav_put_uint8_t(buf, 18, pitch_manual);
	_mav_put_uint8_t(buf, 19, yaw_manual);
	_mav_put_uint8_t(buf, 20, thrust_manual);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 21);
#else
	mavlink_manual_control_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target = target;
	packet.roll_manual = roll_manual;
	packet.pitch_manual = pitch_manual;
	packet.yaw_manual = yaw_manual;
	packet.thrust_manual = thrust_manual;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 21);
#endif

	msg->msgid = MAVLINK_MSG_ID_MANUAL_CONTROL;
	return mavlink_finalize_message(msg, system_id, component_id, 21, 52);
}

/**
 * @brief Pack a manual_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system to be controlled
 * @param roll roll
 * @param pitch pitch
 * @param yaw yaw
 * @param thrust thrust
 * @param roll_manual roll control enabled auto:0, manual:1
 * @param pitch_manual pitch auto:0, manual:1
 * @param yaw_manual yaw auto:0, manual:1
 * @param thrust_manual thrust auto:0, manual:1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_manual_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target,float roll,float pitch,float yaw,float thrust,uint8_t roll_manual,uint8_t pitch_manual,uint8_t yaw_manual,uint8_t thrust_manual)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[21];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, thrust);
	_mav_put_uint8_t(buf, 16, target);
	_mav_put_uint8_t(buf, 17, roll_manual);
	_mav_put_uint8_t(buf, 18, pitch_manual);
	_mav_put_uint8_t(buf, 19, yaw_manual);
	_mav_put_uint8_t(buf, 20, thrust_manual);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 21);
#else
	mavlink_manual_control_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target = target;
	packet.roll_manual = roll_manual;
	packet.pitch_manual = pitch_manual;
	packet.yaw_manual = yaw_manual;
	packet.thrust_manual = thrust_manual;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 21);
#endif

	msg->msgid = MAVLINK_MSG_ID_MANUAL_CONTROL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 21, 52);
}

/**
 * @brief Encode a manual_control struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param manual_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_manual_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_manual_control_t* manual_control)
{
	return mavlink_msg_manual_control_pack(system_id, component_id, msg, manual_control->target, manual_control->roll, manual_control->pitch, manual_control->yaw, manual_control->thrust, manual_control->roll_manual, manual_control->pitch_manual, manual_control->yaw_manual, manual_control->thrust_manual);
}

/**
 * @brief Send a manual_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system to be controlled
 * @param roll roll
 * @param pitch pitch
 * @param yaw yaw
 * @param thrust thrust
 * @param roll_manual roll control enabled auto:0, manual:1
 * @param pitch_manual pitch auto:0, manual:1
 * @param yaw_manual yaw auto:0, manual:1
 * @param thrust_manual thrust auto:0, manual:1
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_manual_control_send(mavlink_channel_t chan, uint8_t target, float roll, float pitch, float yaw, float thrust, uint8_t roll_manual, uint8_t pitch_manual, uint8_t yaw_manual, uint8_t thrust_manual)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[21];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, thrust);
	_mav_put_uint8_t(buf, 16, target);
	_mav_put_uint8_t(buf, 17, roll_manual);
	_mav_put_uint8_t(buf, 18, pitch_manual);
	_mav_put_uint8_t(buf, 19, yaw_manual);
	_mav_put_uint8_t(buf, 20, thrust_manual);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_CONTROL, buf, 21, 52);
#else
	mavlink_manual_control_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target = target;
	packet.roll_manual = roll_manual;
	packet.pitch_manual = pitch_manual;
	packet.yaw_manual = yaw_manual;
	packet.thrust_manual = thrust_manual;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_CONTROL, (const char *)&packet, 21, 52);
#endif
}

#endif

// MESSAGE MANUAL_CONTROL UNPACKING


/**
 * @brief Get field target from manual_control message
 *
 * @return The system to be controlled
 */
static inline uint8_t mavlink_msg_manual_control_get_target(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field roll from manual_control message
 *
 * @return roll
 */
static inline float mavlink_msg_manual_control_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from manual_control message
 *
 * @return pitch
 */
static inline float mavlink_msg_manual_control_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from manual_control message
 *
 * @return yaw
 */
static inline float mavlink_msg_manual_control_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field thrust from manual_control message
 *
 * @return thrust
 */
static inline float mavlink_msg_manual_control_get_thrust(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll_manual from manual_control message
 *
 * @return roll control enabled auto:0, manual:1
 */
static inline uint8_t mavlink_msg_manual_control_get_roll_manual(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field pitch_manual from manual_control message
 *
 * @return pitch auto:0, manual:1
 */
static inline uint8_t mavlink_msg_manual_control_get_pitch_manual(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field yaw_manual from manual_control message
 *
 * @return yaw auto:0, manual:1
 */
static inline uint8_t mavlink_msg_manual_control_get_yaw_manual(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field thrust_manual from manual_control message
 *
 * @return thrust auto:0, manual:1
 */
static inline uint8_t mavlink_msg_manual_control_get_thrust_manual(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a manual_control message into a struct
 *
 * @param msg The message to decode
 * @param manual_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_manual_control_decode(const mavlink_message_t* msg, mavlink_manual_control_t* manual_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	manual_control->roll = mavlink_msg_manual_control_get_roll(msg);
	manual_control->pitch = mavlink_msg_manual_control_get_pitch(msg);
	manual_control->yaw = mavlink_msg_manual_control_get_yaw(msg);
	manual_control->thrust = mavlink_msg_manual_control_get_thrust(msg);
	manual_control->target = mavlink_msg_manual_control_get_target(msg);
	manual_control->roll_manual = mavlink_msg_manual_control_get_roll_manual(msg);
	manual_control->pitch_manual = mavlink_msg_manual_control_get_pitch_manual(msg);
	manual_control->yaw_manual = mavlink_msg_manual_control_get_yaw_manual(msg);
	manual_control->thrust_manual = mavlink_msg_manual_control_get_thrust_manual(msg);
#else
	memcpy(manual_control, _MAV_PAYLOAD(msg), 21);
#endif
}
