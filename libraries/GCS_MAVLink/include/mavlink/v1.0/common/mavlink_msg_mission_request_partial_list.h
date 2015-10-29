// MESSAGE MISSION_REQUEST_PARTIAL_LIST PACKING

#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST 37

typedef struct __mavlink_mission_request_partial_list_t
{
 int16_t start_index; /*< Start index, 0 by default*/
 int16_t end_index; /*< End index, -1 by default (-1: send list to end). Else a valid index of the list*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
} mavlink_mission_request_partial_list_t;

#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN 6
#define MAVLINK_MSG_ID_37_LEN 6

#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC 212
#define MAVLINK_MSG_ID_37_CRC 212



#define MAVLINK_MESSAGE_INFO_MISSION_REQUEST_PARTIAL_LIST { \
	"MISSION_REQUEST_PARTIAL_LIST", \
	4, \
	{  { "start_index", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_mission_request_partial_list_t, start_index) }, \
         { "end_index", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_mission_request_partial_list_t, end_index) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mission_request_partial_list_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mission_request_partial_list_t, target_component) }, \
         } \
}


/**
 * @brief Pack a mission_request_partial_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param start_index Start index, 0 by default
 * @param end_index End index, -1 by default (-1: send list to end). Else a valid index of the list
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_partial_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN];
	_mav_put_int16_t(buf, 0, start_index);
	_mav_put_int16_t(buf, 2, end_index);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#else
	mavlink_mission_request_partial_list_t packet;
	packet.start_index = start_index;
	packet.end_index = end_index;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif
}

/**
 * @brief Pack a mission_request_partial_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param start_index Start index, 0 by default
 * @param end_index End index, -1 by default (-1: send list to end). Else a valid index of the list
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_request_partial_list_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,int16_t start_index,int16_t end_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN];
	_mav_put_int16_t(buf, 0, start_index);
	_mav_put_int16_t(buf, 2, end_index);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#else
	mavlink_mission_request_partial_list_t packet;
	packet.start_index = start_index;
	packet.end_index = end_index;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif
}

/**
 * @brief Encode a mission_request_partial_list struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_partial_list C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_request_partial_list_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_request_partial_list_t* mission_request_partial_list)
{
	return mavlink_msg_mission_request_partial_list_pack(system_id, component_id, msg, mission_request_partial_list->target_system, mission_request_partial_list->target_component, mission_request_partial_list->start_index, mission_request_partial_list->end_index);
}

/**
 * @brief Encode a mission_request_partial_list struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_request_partial_list C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_request_partial_list_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_request_partial_list_t* mission_request_partial_list)
{
	return mavlink_msg_mission_request_partial_list_pack_chan(system_id, component_id, chan, msg, mission_request_partial_list->target_system, mission_request_partial_list->target_component, mission_request_partial_list->start_index, mission_request_partial_list->end_index);
}

/**
 * @brief Send a mission_request_partial_list message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param start_index Start index, 0 by default
 * @param end_index End index, -1 by default (-1: send list to end). Else a valid index of the list
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_request_partial_list_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN];
	_mav_put_int16_t(buf, 0, start_index);
	_mav_put_int16_t(buf, 2, end_index);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, buf, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, buf, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif
#else
	mavlink_mission_request_partial_list_t packet;
	packet.start_index = start_index;
	packet.end_index = end_index;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, (const char *)&packet, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, (const char *)&packet, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_request_partial_list_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, int16_t start_index, int16_t end_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int16_t(buf, 0, start_index);
	_mav_put_int16_t(buf, 2, end_index);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, buf, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, buf, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif
#else
	mavlink_mission_request_partial_list_t *packet = (mavlink_mission_request_partial_list_t *)msgbuf;
	packet->start_index = start_index;
	packet->end_index = end_index;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, (const char *)packet, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST, (const char *)packet, MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MISSION_REQUEST_PARTIAL_LIST UNPACKING


/**
 * @brief Get field target_system from mission_request_partial_list message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_mission_request_partial_list_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from mission_request_partial_list message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_mission_request_partial_list_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field start_index from mission_request_partial_list message
 *
 * @return Start index, 0 by default
 */
static inline int16_t mavlink_msg_mission_request_partial_list_get_start_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field end_index from mission_request_partial_list message
 *
 * @return End index, -1 by default (-1: send list to end). Else a valid index of the list
 */
static inline int16_t mavlink_msg_mission_request_partial_list_get_end_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a mission_request_partial_list message into a struct
 *
 * @param msg The message to decode
 * @param mission_request_partial_list C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_request_partial_list_decode(const mavlink_message_t* msg, mavlink_mission_request_partial_list_t* mission_request_partial_list)
{
#if MAVLINK_NEED_BYTE_SWAP
	mission_request_partial_list->start_index = mavlink_msg_mission_request_partial_list_get_start_index(msg);
	mission_request_partial_list->end_index = mavlink_msg_mission_request_partial_list_get_end_index(msg);
	mission_request_partial_list->target_system = mavlink_msg_mission_request_partial_list_get_target_system(msg);
	mission_request_partial_list->target_component = mavlink_msg_mission_request_partial_list_get_target_component(msg);
#else
	memcpy(mission_request_partial_list, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN);
#endif
}
