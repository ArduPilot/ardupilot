// MESSAGE MISSION_ACK PACKING

#define MAVLINK_MSG_ID_MISSION_ACK 47

typedef struct __mavlink_mission_ack_t
{
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t type; /*< See MAV_MISSION_RESULT enum*/
} mavlink_mission_ack_t;

#define MAVLINK_MSG_ID_MISSION_ACK_LEN 3
#define MAVLINK_MSG_ID_47_LEN 3

#define MAVLINK_MSG_ID_MISSION_ACK_CRC 153
#define MAVLINK_MSG_ID_47_CRC 153



#define MAVLINK_MESSAGE_INFO_MISSION_ACK { \
	"MISSION_ACK", \
	3, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_ack_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_ack_t, target_component) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_ack_t, type) }, \
         } \
}


/**
 * @brief Pack a mission_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param type See MAV_MISSION_RESULT enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_ACK_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_ACK_LEN);
#else
	mavlink_mission_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_ACK_LEN, MAVLINK_MSG_ID_MISSION_ACK_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_ACK_LEN);
#endif
}

/**
 * @brief Pack a mission_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param type See MAV_MISSION_RESULT enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_ACK_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_ACK_LEN);
#else
	mavlink_mission_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.type = type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_ACK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_ACK_LEN, MAVLINK_MSG_ID_MISSION_ACK_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_ACK_LEN);
#endif
}

/**
 * @brief Encode a mission_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_ack_t* mission_ack)
{
	return mavlink_msg_mission_ack_pack(system_id, component_id, msg, mission_ack->target_system, mission_ack->target_component, mission_ack->type);
}

/**
 * @brief Encode a mission_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_ack_t* mission_ack)
{
	return mavlink_msg_mission_ack_pack_chan(system_id, component_id, chan, msg, mission_ack->target_system, mission_ack->target_component, mission_ack->type);
}

/**
 * @brief Send a mission_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param type See MAV_MISSION_RESULT enum
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_ack_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_ACK_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ACK, buf, MAVLINK_MSG_ID_MISSION_ACK_LEN, MAVLINK_MSG_ID_MISSION_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ACK, buf, MAVLINK_MSG_ID_MISSION_ACK_LEN);
#endif
#else
	mavlink_mission_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.type = type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ACK, (const char *)&packet, MAVLINK_MSG_ID_MISSION_ACK_LEN, MAVLINK_MSG_ID_MISSION_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ACK, (const char *)&packet, MAVLINK_MSG_ID_MISSION_ACK_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MISSION_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ACK, buf, MAVLINK_MSG_ID_MISSION_ACK_LEN, MAVLINK_MSG_ID_MISSION_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ACK, buf, MAVLINK_MSG_ID_MISSION_ACK_LEN);
#endif
#else
	mavlink_mission_ack_t *packet = (mavlink_mission_ack_t *)msgbuf;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->type = type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ACK, (const char *)packet, MAVLINK_MSG_ID_MISSION_ACK_LEN, MAVLINK_MSG_ID_MISSION_ACK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ACK, (const char *)packet, MAVLINK_MSG_ID_MISSION_ACK_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MISSION_ACK UNPACKING


/**
 * @brief Get field target_system from mission_ack message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mission_ack_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from mission_ack message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_mission_ack_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field type from mission_ack message
 *
 * @return See MAV_MISSION_RESULT enum
 */
static inline uint8_t mavlink_msg_mission_ack_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a mission_ack message into a struct
 *
 * @param msg The message to decode
 * @param mission_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_ack_decode(const mavlink_message_t* msg, mavlink_mission_ack_t* mission_ack)
{
#if MAVLINK_NEED_BYTE_SWAP
	mission_ack->target_system = mavlink_msg_mission_ack_get_target_system(msg);
	mission_ack->target_component = mavlink_msg_mission_ack_get_target_component(msg);
	mission_ack->type = mavlink_msg_mission_ack_get_type(msg);
#else
	memcpy(mission_ack, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MISSION_ACK_LEN);
#endif
}
