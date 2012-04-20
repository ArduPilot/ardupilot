// MESSAGE MISSION_REQUEST PACKING

#define MAVLINK_MSG_ID_MISSION_REQUEST 40

typedef struct __mavlink_mission_request_t
{
 uint16_t seq; ///< Sequence
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_mission_request_t;

#define MAVLINK_MSG_ID_MISSION_REQUEST_LEN 4
#define MAVLINK_MSG_ID_40_LEN 4



#define MAVLINK_MESSAGE_INFO_MISSION_REQUEST { \
	"MISSION_REQUEST", \
	3, \
	{  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mission_request_t, seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_request_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mission_request_t, target_component) }, \
         } \
}


/**
 * @brief Pack a mission_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, seq);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_mission_request_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 230);
}

/**
 * @brief Pack a mission_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, seq);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_mission_request_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 230);
}

/**
 * @brief Encode a mission_request struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_request_t* mission_request)
{
	return mavlink_msg_mission_request_pack(system_id, component_id, msg, mission_request->target_system, mission_request->target_component, mission_request->seq);
}

/**
 * @brief Send a mission_request message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_request_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, seq);
	_mav_put_uint8_t(buf, 2, target_system);
	_mav_put_uint8_t(buf, 3, target_component);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST, buf, 4, 230);
#else
	mavlink_mission_request_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST, (const char *)&packet, 4, 230);
#endif
}

#endif

// MESSAGE MISSION_REQUEST UNPACKING


/**
 * @brief Get field target_system from mission_request message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mission_request_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field target_component from mission_request message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_mission_request_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field seq from mission_request message
 *
 * @return Sequence
 */
static inline uint16_t mavlink_msg_mission_request_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a mission_request message into a struct
 *
 * @param msg The message to decode
 * @param mission_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_request_decode(const mavlink_message_t* msg, mavlink_mission_request_t* mission_request)
{
#if MAVLINK_NEED_BYTE_SWAP
	mission_request->seq = mavlink_msg_mission_request_get_seq(msg);
	mission_request->target_system = mavlink_msg_mission_request_get_target_system(msg);
	mission_request->target_component = mavlink_msg_mission_request_get_target_component(msg);
#else
	memcpy(mission_request, _MAV_PAYLOAD(msg), 4);
#endif
}
