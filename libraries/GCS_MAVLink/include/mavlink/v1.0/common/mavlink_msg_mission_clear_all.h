// MESSAGE MISSION_CLEAR_ALL PACKING

#define MAVLINK_MSG_ID_MISSION_CLEAR_ALL 45

typedef struct __mavlink_mission_clear_all_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_mission_clear_all_t;

#define MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN 2
#define MAVLINK_MSG_ID_45_LEN 2

#define MAVLINK_MSG_ID_MISSION_CLEAR_ALL_CRC 232
#define MAVLINK_MSG_ID_45_CRC 232



#define MAVLINK_MESSAGE_INFO_MISSION_CLEAR_ALL { \
	"MISSION_CLEAR_ALL", \
	2, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_clear_all_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_clear_all_t, target_component) }, \
         } \
}


/**
 * @brief Pack a mission_clear_all message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_clear_all_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#else
	mavlink_mission_clear_all_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_CLEAR_ALL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#endif
}

/**
 * @brief Pack a mission_clear_all message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_clear_all_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#else
	mavlink_mission_clear_all_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_CLEAR_ALL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#endif
}

/**
 * @brief Encode a mission_clear_all struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_clear_all C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_clear_all_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_clear_all_t* mission_clear_all)
{
	return mavlink_msg_mission_clear_all_pack(system_id, component_id, msg, mission_clear_all->target_system, mission_clear_all->target_component);
}

/**
 * @brief Encode a mission_clear_all struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_clear_all C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_clear_all_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_clear_all_t* mission_clear_all)
{
	return mavlink_msg_mission_clear_all_pack_chan(system_id, component_id, chan, msg, mission_clear_all->target_system, mission_clear_all->target_component);
}

/**
 * @brief Send a mission_clear_all message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_clear_all_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, buf, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, buf, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#endif
#else
	mavlink_mission_clear_all_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, (const char *)&packet, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, (const char *)&packet, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_clear_all_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, buf, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, buf, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#endif
#else
	mavlink_mission_clear_all_t *packet = (mavlink_mission_clear_all_t *)msgbuf;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, (const char *)packet, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, (const char *)packet, MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MISSION_CLEAR_ALL UNPACKING


/**
 * @brief Get field target_system from mission_clear_all message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mission_clear_all_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from mission_clear_all message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_mission_clear_all_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a mission_clear_all message into a struct
 *
 * @param msg The message to decode
 * @param mission_clear_all C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_clear_all_decode(const mavlink_message_t* msg, mavlink_mission_clear_all_t* mission_clear_all)
{
#if MAVLINK_NEED_BYTE_SWAP
	mission_clear_all->target_system = mavlink_msg_mission_clear_all_get_target_system(msg);
	mission_clear_all->target_component = mavlink_msg_mission_clear_all_get_target_component(msg);
#else
	memcpy(mission_clear_all, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN);
#endif
}
