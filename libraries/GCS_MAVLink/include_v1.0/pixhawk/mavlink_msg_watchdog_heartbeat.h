// MESSAGE WATCHDOG_HEARTBEAT PACKING

#define MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT 180

typedef struct __mavlink_watchdog_heartbeat_t
{
 uint16_t watchdog_id; ///< Watchdog ID
 uint16_t process_count; ///< Number of processes
} mavlink_watchdog_heartbeat_t;

#define MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT_LEN 4
#define MAVLINK_MSG_ID_180_LEN 4



#define MAVLINK_MESSAGE_INFO_WATCHDOG_HEARTBEAT { \
	"WATCHDOG_HEARTBEAT", \
	2, \
	{  { "watchdog_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_watchdog_heartbeat_t, watchdog_id) }, \
         { "process_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_watchdog_heartbeat_t, process_count) }, \
         } \
}


/**
 * @brief Pack a watchdog_heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param watchdog_id Watchdog ID
 * @param process_count Number of processes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_watchdog_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t watchdog_id, uint16_t process_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, watchdog_id);
	_mav_put_uint16_t(buf, 2, process_count);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_watchdog_heartbeat_t packet;
	packet.watchdog_id = watchdog_id;
	packet.process_count = process_count;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 153);
}

/**
 * @brief Pack a watchdog_heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param watchdog_id Watchdog ID
 * @param process_count Number of processes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_watchdog_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t watchdog_id,uint16_t process_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, watchdog_id);
	_mav_put_uint16_t(buf, 2, process_count);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_watchdog_heartbeat_t packet;
	packet.watchdog_id = watchdog_id;
	packet.process_count = process_count;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 153);
}

/**
 * @brief Encode a watchdog_heartbeat struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param watchdog_heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_watchdog_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_watchdog_heartbeat_t* watchdog_heartbeat)
{
	return mavlink_msg_watchdog_heartbeat_pack(system_id, component_id, msg, watchdog_heartbeat->watchdog_id, watchdog_heartbeat->process_count);
}

/**
 * @brief Send a watchdog_heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param watchdog_id Watchdog ID
 * @param process_count Number of processes
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_watchdog_heartbeat_send(mavlink_channel_t chan, uint16_t watchdog_id, uint16_t process_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, watchdog_id);
	_mav_put_uint16_t(buf, 2, process_count);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT, buf, 4, 153);
#else
	mavlink_watchdog_heartbeat_t packet;
	packet.watchdog_id = watchdog_id;
	packet.process_count = process_count;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT, (const char *)&packet, 4, 153);
#endif
}

#endif

// MESSAGE WATCHDOG_HEARTBEAT UNPACKING


/**
 * @brief Get field watchdog_id from watchdog_heartbeat message
 *
 * @return Watchdog ID
 */
static inline uint16_t mavlink_msg_watchdog_heartbeat_get_watchdog_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field process_count from watchdog_heartbeat message
 *
 * @return Number of processes
 */
static inline uint16_t mavlink_msg_watchdog_heartbeat_get_process_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a watchdog_heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param watchdog_heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_watchdog_heartbeat_decode(const mavlink_message_t* msg, mavlink_watchdog_heartbeat_t* watchdog_heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP
	watchdog_heartbeat->watchdog_id = mavlink_msg_watchdog_heartbeat_get_watchdog_id(msg);
	watchdog_heartbeat->process_count = mavlink_msg_watchdog_heartbeat_get_process_count(msg);
#else
	memcpy(watchdog_heartbeat, _MAV_PAYLOAD(msg), 4);
#endif
}
