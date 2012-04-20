// MESSAGE CONTROL_STATUS PACKING

#define MAVLINK_MSG_ID_CONTROL_STATUS 52

typedef struct __mavlink_control_status_t
{
 uint8_t position_fix; ///< Position fix: 0: lost, 2: 2D position fix, 3: 3D position fix 
 uint8_t vision_fix; ///< Vision position fix: 0: lost, 1: 2D local position hold, 2: 2D global position fix, 3: 3D global position fix 
 uint8_t gps_fix; ///< GPS position fix: 0: no reception, 1: Minimum 1 satellite, but no position fix, 2: 2D position fix, 3: 3D position fix 
 uint8_t ahrs_health; ///< Attitude estimation health: 0: poor, 255: excellent
 uint8_t control_att; ///< 0: Attitude control disabled, 1: enabled
 uint8_t control_pos_xy; ///< 0: X, Y position control disabled, 1: enabled
 uint8_t control_pos_z; ///< 0: Z position control disabled, 1: enabled
 uint8_t control_pos_yaw; ///< 0: Yaw angle control disabled, 1: enabled
} mavlink_control_status_t;

#define MAVLINK_MSG_ID_CONTROL_STATUS_LEN 8
#define MAVLINK_MSG_ID_52_LEN 8



#define MAVLINK_MESSAGE_INFO_CONTROL_STATUS { \
	"CONTROL_STATUS", \
	8, \
	{  { "position_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_control_status_t, position_fix) }, \
         { "vision_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_control_status_t, vision_fix) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_control_status_t, gps_fix) }, \
         { "ahrs_health", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_control_status_t, ahrs_health) }, \
         { "control_att", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_control_status_t, control_att) }, \
         { "control_pos_xy", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_control_status_t, control_pos_xy) }, \
         { "control_pos_z", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_control_status_t, control_pos_z) }, \
         { "control_pos_yaw", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_control_status_t, control_pos_yaw) }, \
         } \
}


/**
 * @brief Pack a control_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param position_fix Position fix: 0: lost, 2: 2D position fix, 3: 3D position fix 
 * @param vision_fix Vision position fix: 0: lost, 1: 2D local position hold, 2: 2D global position fix, 3: 3D global position fix 
 * @param gps_fix GPS position fix: 0: no reception, 1: Minimum 1 satellite, but no position fix, 2: 2D position fix, 3: 3D position fix 
 * @param ahrs_health Attitude estimation health: 0: poor, 255: excellent
 * @param control_att 0: Attitude control disabled, 1: enabled
 * @param control_pos_xy 0: X, Y position control disabled, 1: enabled
 * @param control_pos_z 0: Z position control disabled, 1: enabled
 * @param control_pos_yaw 0: Yaw angle control disabled, 1: enabled
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t position_fix, uint8_t vision_fix, uint8_t gps_fix, uint8_t ahrs_health, uint8_t control_att, uint8_t control_pos_xy, uint8_t control_pos_z, uint8_t control_pos_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint8_t(buf, 0, position_fix);
	_mav_put_uint8_t(buf, 1, vision_fix);
	_mav_put_uint8_t(buf, 2, gps_fix);
	_mav_put_uint8_t(buf, 3, ahrs_health);
	_mav_put_uint8_t(buf, 4, control_att);
	_mav_put_uint8_t(buf, 5, control_pos_xy);
	_mav_put_uint8_t(buf, 6, control_pos_z);
	_mav_put_uint8_t(buf, 7, control_pos_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 8);
#else
	mavlink_control_status_t packet;
	packet.position_fix = position_fix;
	packet.vision_fix = vision_fix;
	packet.gps_fix = gps_fix;
	packet.ahrs_health = ahrs_health;
	packet.control_att = control_att;
	packet.control_pos_xy = control_pos_xy;
	packet.control_pos_z = control_pos_z;
	packet.control_pos_yaw = control_pos_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONTROL_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 8);
}

/**
 * @brief Pack a control_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param position_fix Position fix: 0: lost, 2: 2D position fix, 3: 3D position fix 
 * @param vision_fix Vision position fix: 0: lost, 1: 2D local position hold, 2: 2D global position fix, 3: 3D global position fix 
 * @param gps_fix GPS position fix: 0: no reception, 1: Minimum 1 satellite, but no position fix, 2: 2D position fix, 3: 3D position fix 
 * @param ahrs_health Attitude estimation health: 0: poor, 255: excellent
 * @param control_att 0: Attitude control disabled, 1: enabled
 * @param control_pos_xy 0: X, Y position control disabled, 1: enabled
 * @param control_pos_z 0: Z position control disabled, 1: enabled
 * @param control_pos_yaw 0: Yaw angle control disabled, 1: enabled
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t position_fix,uint8_t vision_fix,uint8_t gps_fix,uint8_t ahrs_health,uint8_t control_att,uint8_t control_pos_xy,uint8_t control_pos_z,uint8_t control_pos_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint8_t(buf, 0, position_fix);
	_mav_put_uint8_t(buf, 1, vision_fix);
	_mav_put_uint8_t(buf, 2, gps_fix);
	_mav_put_uint8_t(buf, 3, ahrs_health);
	_mav_put_uint8_t(buf, 4, control_att);
	_mav_put_uint8_t(buf, 5, control_pos_xy);
	_mav_put_uint8_t(buf, 6, control_pos_z);
	_mav_put_uint8_t(buf, 7, control_pos_yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 8);
#else
	mavlink_control_status_t packet;
	packet.position_fix = position_fix;
	packet.vision_fix = vision_fix;
	packet.gps_fix = gps_fix;
	packet.ahrs_health = ahrs_health;
	packet.control_att = control_att;
	packet.control_pos_xy = control_pos_xy;
	packet.control_pos_z = control_pos_z;
	packet.control_pos_yaw = control_pos_yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_CONTROL_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8);
}

/**
 * @brief Encode a control_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param control_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_control_status_t* control_status)
{
	return mavlink_msg_control_status_pack(system_id, component_id, msg, control_status->position_fix, control_status->vision_fix, control_status->gps_fix, control_status->ahrs_health, control_status->control_att, control_status->control_pos_xy, control_status->control_pos_z, control_status->control_pos_yaw);
}

/**
 * @brief Send a control_status message
 * @param chan MAVLink channel to send the message
 *
 * @param position_fix Position fix: 0: lost, 2: 2D position fix, 3: 3D position fix 
 * @param vision_fix Vision position fix: 0: lost, 1: 2D local position hold, 2: 2D global position fix, 3: 3D global position fix 
 * @param gps_fix GPS position fix: 0: no reception, 1: Minimum 1 satellite, but no position fix, 2: 2D position fix, 3: 3D position fix 
 * @param ahrs_health Attitude estimation health: 0: poor, 255: excellent
 * @param control_att 0: Attitude control disabled, 1: enabled
 * @param control_pos_xy 0: X, Y position control disabled, 1: enabled
 * @param control_pos_z 0: Z position control disabled, 1: enabled
 * @param control_pos_yaw 0: Yaw angle control disabled, 1: enabled
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_control_status_send(mavlink_channel_t chan, uint8_t position_fix, uint8_t vision_fix, uint8_t gps_fix, uint8_t ahrs_health, uint8_t control_att, uint8_t control_pos_xy, uint8_t control_pos_z, uint8_t control_pos_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_uint8_t(buf, 0, position_fix);
	_mav_put_uint8_t(buf, 1, vision_fix);
	_mav_put_uint8_t(buf, 2, gps_fix);
	_mav_put_uint8_t(buf, 3, ahrs_health);
	_mav_put_uint8_t(buf, 4, control_att);
	_mav_put_uint8_t(buf, 5, control_pos_xy);
	_mav_put_uint8_t(buf, 6, control_pos_z);
	_mav_put_uint8_t(buf, 7, control_pos_yaw);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_STATUS, buf, 8);
#else
	mavlink_control_status_t packet;
	packet.position_fix = position_fix;
	packet.vision_fix = vision_fix;
	packet.gps_fix = gps_fix;
	packet.ahrs_health = ahrs_health;
	packet.control_att = control_att;
	packet.control_pos_xy = control_pos_xy;
	packet.control_pos_z = control_pos_z;
	packet.control_pos_yaw = control_pos_yaw;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_STATUS, (const char *)&packet, 8);
#endif
}

#endif

// MESSAGE CONTROL_STATUS UNPACKING


/**
 * @brief Get field position_fix from control_status message
 *
 * @return Position fix: 0: lost, 2: 2D position fix, 3: 3D position fix 
 */
static inline uint8_t mavlink_msg_control_status_get_position_fix(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field vision_fix from control_status message
 *
 * @return Vision position fix: 0: lost, 1: 2D local position hold, 2: 2D global position fix, 3: 3D global position fix 
 */
static inline uint8_t mavlink_msg_control_status_get_vision_fix(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field gps_fix from control_status message
 *
 * @return GPS position fix: 0: no reception, 1: Minimum 1 satellite, but no position fix, 2: 2D position fix, 3: 3D position fix 
 */
static inline uint8_t mavlink_msg_control_status_get_gps_fix(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field ahrs_health from control_status message
 *
 * @return Attitude estimation health: 0: poor, 255: excellent
 */
static inline uint8_t mavlink_msg_control_status_get_ahrs_health(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field control_att from control_status message
 *
 * @return 0: Attitude control disabled, 1: enabled
 */
static inline uint8_t mavlink_msg_control_status_get_control_att(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field control_pos_xy from control_status message
 *
 * @return 0: X, Y position control disabled, 1: enabled
 */
static inline uint8_t mavlink_msg_control_status_get_control_pos_xy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field control_pos_z from control_status message
 *
 * @return 0: Z position control disabled, 1: enabled
 */
static inline uint8_t mavlink_msg_control_status_get_control_pos_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field control_pos_yaw from control_status message
 *
 * @return 0: Yaw angle control disabled, 1: enabled
 */
static inline uint8_t mavlink_msg_control_status_get_control_pos_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Decode a control_status message into a struct
 *
 * @param msg The message to decode
 * @param control_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_control_status_decode(const mavlink_message_t* msg, mavlink_control_status_t* control_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	control_status->position_fix = mavlink_msg_control_status_get_position_fix(msg);
	control_status->vision_fix = mavlink_msg_control_status_get_vision_fix(msg);
	control_status->gps_fix = mavlink_msg_control_status_get_gps_fix(msg);
	control_status->ahrs_health = mavlink_msg_control_status_get_ahrs_health(msg);
	control_status->control_att = mavlink_msg_control_status_get_control_att(msg);
	control_status->control_pos_xy = mavlink_msg_control_status_get_control_pos_xy(msg);
	control_status->control_pos_z = mavlink_msg_control_status_get_control_pos_z(msg);
	control_status->control_pos_yaw = mavlink_msg_control_status_get_control_pos_yaw(msg);
#else
	memcpy(control_status, _MAV_PAYLOAD(msg), 8);
#endif
}
