// MESSAGE BOOT PACKING

#define MAVLINK_MSG_ID_BOOT 1

typedef struct __mavlink_boot_t
{
 uint32_t version; ///< The onboard software version
} mavlink_boot_t;

#define MAVLINK_MSG_ID_BOOT_LEN 4
#define MAVLINK_MSG_ID_1_LEN 4



#define MAVLINK_MESSAGE_INFO_BOOT { \
	"BOOT", \
	1, \
	{  { "version", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_boot_t, version) }, \
         } \
}


/**
 * @brief Pack a boot message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param version The onboard software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boot_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint32_t(buf, 0, version);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_boot_t packet;
	packet.version = version;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_BOOT;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 39);
}

/**
 * @brief Pack a boot message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param version The onboard software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boot_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint32_t(buf, 0, version);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_boot_t packet;
	packet.version = version;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_BOOT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 39);
}

/**
 * @brief Encode a boot struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boot C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_boot_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_boot_t* boot)
{
	return mavlink_msg_boot_pack(system_id, component_id, msg, boot->version);
}

/**
 * @brief Send a boot message
 * @param chan MAVLink channel to send the message
 *
 * @param version The onboard software version
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_boot_send(mavlink_channel_t chan, uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint32_t(buf, 0, version);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT, buf, 4, 39);
#else
	mavlink_boot_t packet;
	packet.version = version;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT, (const char *)&packet, 4, 39);
#endif
}

#endif

// MESSAGE BOOT UNPACKING


/**
 * @brief Get field version from boot message
 *
 * @return The onboard software version
 */
static inline uint32_t mavlink_msg_boot_get_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a boot message into a struct
 *
 * @param msg The message to decode
 * @param boot C-struct to decode the message contents into
 */
static inline void mavlink_msg_boot_decode(const mavlink_message_t* msg, mavlink_boot_t* boot)
{
#if MAVLINK_NEED_BYTE_SWAP
	boot->version = mavlink_msg_boot_get_version(msg);
#else
	memcpy(boot, _MAV_PAYLOAD(msg), 4);
#endif
}
