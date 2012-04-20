// MESSAGE MOUNT_STATUS PACKING

#define MAVLINK_MSG_ID_MOUNT_STATUS 158

typedef struct __mavlink_mount_status_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 int32_t pointing_a; ///< pitch(deg*100) or lat, depending on mount mode
 int32_t pointing_b; ///< roll(deg*100) or lon depending on mount mode
 int32_t pointing_c; ///< yaw(deg*100) or alt (in cm) depending on mount mode
} mavlink_mount_status_t;

#define MAVLINK_MSG_ID_MOUNT_STATUS_LEN 14
#define MAVLINK_MSG_ID_158_LEN 14



#define MAVLINK_MESSAGE_INFO_MOUNT_STATUS { \
	"MOUNT_STATUS", \
	5, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mount_status_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mount_status_t, target_component) }, \
         { "pointing_a", NULL, MAVLINK_TYPE_INT32_T, 0, 2, offsetof(mavlink_mount_status_t, pointing_a) }, \
         { "pointing_b", NULL, MAVLINK_TYPE_INT32_T, 0, 6, offsetof(mavlink_mount_status_t, pointing_b) }, \
         { "pointing_c", NULL, MAVLINK_TYPE_INT32_T, 0, 10, offsetof(mavlink_mount_status_t, pointing_c) }, \
         } \
}


/**
 * @brief Pack a mount_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param pointing_a pitch(deg*100) or lat, depending on mount mode
 * @param pointing_b roll(deg*100) or lon depending on mount mode
 * @param pointing_c yaw(deg*100) or alt (in cm) depending on mount mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_int32_t(buf, 2, pointing_a);
	_mav_put_int32_t(buf, 6, pointing_b);
	_mav_put_int32_t(buf, 10, pointing_c);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 14);
#else
	mavlink_mount_status_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.pointing_a = pointing_a;
	packet.pointing_b = pointing_b;
	packet.pointing_c = pointing_c;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOUNT_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 14);
}

/**
 * @brief Pack a mount_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param pointing_a pitch(deg*100) or lat, depending on mount mode
 * @param pointing_b roll(deg*100) or lon depending on mount mode
 * @param pointing_c yaw(deg*100) or alt (in cm) depending on mount mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,int32_t pointing_a,int32_t pointing_b,int32_t pointing_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_int32_t(buf, 2, pointing_a);
	_mav_put_int32_t(buf, 6, pointing_b);
	_mav_put_int32_t(buf, 10, pointing_c);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 14);
#else
	mavlink_mount_status_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.pointing_a = pointing_a;
	packet.pointing_b = pointing_b;
	packet.pointing_c = pointing_c;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOUNT_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14);
}

/**
 * @brief Encode a mount_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mount_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mount_status_t* mount_status)
{
	return mavlink_msg_mount_status_pack(system_id, component_id, msg, mount_status->target_system, mount_status->target_component, mount_status->pointing_a, mount_status->pointing_b, mount_status->pointing_c);
}

/**
 * @brief Send a mount_status message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param pointing_a pitch(deg*100) or lat, depending on mount mode
 * @param pointing_b roll(deg*100) or lon depending on mount mode
 * @param pointing_c yaw(deg*100) or alt (in cm) depending on mount mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mount_status_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int32_t pointing_a, int32_t pointing_b, int32_t pointing_c)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_int32_t(buf, 2, pointing_a);
	_mav_put_int32_t(buf, 6, pointing_b);
	_mav_put_int32_t(buf, 10, pointing_c);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, buf, 14);
#else
	mavlink_mount_status_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.pointing_a = pointing_a;
	packet.pointing_b = pointing_b;
	packet.pointing_c = pointing_c;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_STATUS, (const char *)&packet, 14);
#endif
}

#endif

// MESSAGE MOUNT_STATUS UNPACKING


/**
 * @brief Get field target_system from mount_status message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mount_status_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from mount_status message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_mount_status_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field pointing_a from mount_status message
 *
 * @return pitch(deg*100) or lat, depending on mount mode
 */
static inline int32_t mavlink_msg_mount_status_get_pointing_a(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  2);
}

/**
 * @brief Get field pointing_b from mount_status message
 *
 * @return roll(deg*100) or lon depending on mount mode
 */
static inline int32_t mavlink_msg_mount_status_get_pointing_b(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  6);
}

/**
 * @brief Get field pointing_c from mount_status message
 *
 * @return yaw(deg*100) or alt (in cm) depending on mount mode
 */
static inline int32_t mavlink_msg_mount_status_get_pointing_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  10);
}

/**
 * @brief Decode a mount_status message into a struct
 *
 * @param msg The message to decode
 * @param mount_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mount_status_decode(const mavlink_message_t* msg, mavlink_mount_status_t* mount_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	mount_status->target_system = mavlink_msg_mount_status_get_target_system(msg);
	mount_status->target_component = mavlink_msg_mount_status_get_target_component(msg);
	mount_status->pointing_a = mavlink_msg_mount_status_get_pointing_a(msg);
	mount_status->pointing_b = mavlink_msg_mount_status_get_pointing_b(msg);
	mount_status->pointing_c = mavlink_msg_mount_status_get_pointing_c(msg);
#else
	memcpy(mount_status, _MAV_PAYLOAD(msg), 14);
#endif
}
