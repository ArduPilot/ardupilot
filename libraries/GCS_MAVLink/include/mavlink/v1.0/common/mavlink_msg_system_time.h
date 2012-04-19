// MESSAGE SYSTEM_TIME PACKING

#define MAVLINK_MSG_ID_SYSTEM_TIME 2

typedef struct __mavlink_system_time_t
{
 uint64_t time_unix_usec; ///< Timestamp of the master clock in microseconds since UNIX epoch.
 uint32_t time_boot_ms; ///< Timestamp of the component clock since boot time in milliseconds.
} mavlink_system_time_t;

#define MAVLINK_MSG_ID_SYSTEM_TIME_LEN 12
#define MAVLINK_MSG_ID_2_LEN 12



#define MAVLINK_MESSAGE_INFO_SYSTEM_TIME { \
	"SYSTEM_TIME", \
	2, \
	{  { "time_unix_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_system_time_t, time_unix_usec) }, \
         { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_system_time_t, time_boot_ms) }, \
         } \
}


/**
 * @brief Pack a system_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_time_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_unix_usec, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, time_unix_usec);
	_mav_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_system_time_t packet;
	packet.time_unix_usec = time_unix_usec;
	packet.time_boot_ms = time_boot_ms;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 137);
}

/**
 * @brief Pack a system_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_unix_usec,uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, time_unix_usec);
	_mav_put_uint32_t(buf, 8, time_boot_ms);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_system_time_t packet;
	packet.time_unix_usec = time_unix_usec;
	packet.time_boot_ms = time_boot_ms;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 137);
}

/**
 * @brief Encode a system_time struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_system_time_t* system_time)
{
	return mavlink_msg_system_time_pack(system_id, component_id, msg, system_time->time_unix_usec, system_time->time_boot_ms);
}

/**
 * @brief Send a system_time message
 * @param chan MAVLink channel to send the message
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_system_time_send(mavlink_channel_t chan, uint64_t time_unix_usec, uint32_t time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];
	_mav_put_uint64_t(buf, 0, time_unix_usec);
	_mav_put_uint32_t(buf, 8, time_boot_ms);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, buf, 12, 137);
#else
	mavlink_system_time_t packet;
	packet.time_unix_usec = time_unix_usec;
	packet.time_boot_ms = time_boot_ms;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, (const char *)&packet, 12, 137);
#endif
}

#endif

// MESSAGE SYSTEM_TIME UNPACKING


/**
 * @brief Get field time_unix_usec from system_time message
 *
 * @return Timestamp of the master clock in microseconds since UNIX epoch.
 */
static inline uint64_t mavlink_msg_system_time_get_time_unix_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_boot_ms from system_time message
 *
 * @return Timestamp of the component clock since boot time in milliseconds.
 */
static inline uint32_t mavlink_msg_system_time_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a system_time message into a struct
 *
 * @param msg The message to decode
 * @param system_time C-struct to decode the message contents into
 */
static inline void mavlink_msg_system_time_decode(const mavlink_message_t* msg, mavlink_system_time_t* system_time)
{
#if MAVLINK_NEED_BYTE_SWAP
	system_time->time_unix_usec = mavlink_msg_system_time_get_time_unix_usec(msg);
	system_time->time_boot_ms = mavlink_msg_system_time_get_time_boot_ms(msg);
#else
	memcpy(system_time, _MAV_PAYLOAD(msg), 12);
#endif
}
