// MESSAGE MEMINFO PACKING

#define MAVLINK_MSG_ID_MEMINFO 152

typedef struct __mavlink_meminfo_t
{
 uint16_t brkval; ///< heap top
 uint16_t freemem; ///< free memory
} mavlink_meminfo_t;

#define MAVLINK_MSG_ID_MEMINFO_LEN 4
#define MAVLINK_MSG_ID_152_LEN 4



#define MAVLINK_MESSAGE_INFO_MEMINFO { \
	"MEMINFO", \
	2, \
	{  { "brkval", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_meminfo_t, brkval) }, \
         { "freemem", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_meminfo_t, freemem) }, \
         } \
}


/**
 * @brief Pack a meminfo message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param brkval heap top
 * @param freemem free memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_meminfo_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t brkval, uint16_t freemem)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, brkval);
	_mav_put_uint16_t(buf, 2, freemem);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_meminfo_t packet;
	packet.brkval = brkval;
	packet.freemem = freemem;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_MEMINFO;
	return mavlink_finalize_message(msg, system_id, component_id, 4);
}

/**
 * @brief Pack a meminfo message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param brkval heap top
 * @param freemem free memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_meminfo_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t brkval,uint16_t freemem)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, brkval);
	_mav_put_uint16_t(buf, 2, freemem);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_meminfo_t packet;
	packet.brkval = brkval;
	packet.freemem = freemem;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_MEMINFO;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4);
}

/**
 * @brief Encode a meminfo struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param meminfo C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_meminfo_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_meminfo_t* meminfo)
{
	return mavlink_msg_meminfo_pack(system_id, component_id, msg, meminfo->brkval, meminfo->freemem);
}

/**
 * @brief Send a meminfo message
 * @param chan MAVLink channel to send the message
 *
 * @param brkval heap top
 * @param freemem free memory
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_meminfo_send(mavlink_channel_t chan, uint16_t brkval, uint16_t freemem)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, brkval);
	_mav_put_uint16_t(buf, 2, freemem);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEMINFO, buf, 4);
#else
	mavlink_meminfo_t packet;
	packet.brkval = brkval;
	packet.freemem = freemem;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEMINFO, (const char *)&packet, 4);
#endif
}

#endif

// MESSAGE MEMINFO UNPACKING


/**
 * @brief Get field brkval from meminfo message
 *
 * @return heap top
 */
static inline uint16_t mavlink_msg_meminfo_get_brkval(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field freemem from meminfo message
 *
 * @return free memory
 */
static inline uint16_t mavlink_msg_meminfo_get_freemem(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a meminfo message into a struct
 *
 * @param msg The message to decode
 * @param meminfo C-struct to decode the message contents into
 */
static inline void mavlink_msg_meminfo_decode(const mavlink_message_t* msg, mavlink_meminfo_t* meminfo)
{
#if MAVLINK_NEED_BYTE_SWAP
	meminfo->brkval = mavlink_msg_meminfo_get_brkval(msg);
	meminfo->freemem = mavlink_msg_meminfo_get_freemem(msg);
#else
	memcpy(meminfo, _MAV_PAYLOAD(msg), 4);
#endif
}
