// MESSAGE FILE_TRANSFER_START PACKING

#define MAVLINK_MSG_ID_FILE_TRANSFER_START 110

typedef struct __mavlink_file_transfer_start_t
{
 uint64_t transfer_uid; ///< Unique transfer ID
 uint32_t file_size; ///< File size in bytes
 char dest_path[240]; ///< Destination path
 uint8_t direction; ///< Transfer direction: 0: from requester, 1: to requester
 uint8_t flags; ///< RESERVED
} mavlink_file_transfer_start_t;

#define MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN 254
#define MAVLINK_MSG_ID_110_LEN 254

#define MAVLINK_MSG_ID_FILE_TRANSFER_START_CRC 235
#define MAVLINK_MSG_ID_110_CRC 235

#define MAVLINK_MSG_FILE_TRANSFER_START_FIELD_DEST_PATH_LEN 240

#define MAVLINK_MESSAGE_INFO_FILE_TRANSFER_START { \
	"FILE_TRANSFER_START", \
	5, \
	{  { "transfer_uid", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_file_transfer_start_t, transfer_uid) }, \
         { "file_size", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_file_transfer_start_t, file_size) }, \
         { "dest_path", NULL, MAVLINK_TYPE_CHAR, 240, 12, offsetof(mavlink_file_transfer_start_t, dest_path) }, \
         { "direction", NULL, MAVLINK_TYPE_UINT8_T, 0, 252, offsetof(mavlink_file_transfer_start_t, direction) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 253, offsetof(mavlink_file_transfer_start_t, flags) }, \
         } \
}


/**
 * @brief Pack a file_transfer_start message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param transfer_uid Unique transfer ID
 * @param dest_path Destination path
 * @param direction Transfer direction: 0: from requester, 1: to requester
 * @param file_size File size in bytes
 * @param flags RESERVED
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_transfer_start_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t transfer_uid, const char *dest_path, uint8_t direction, uint32_t file_size, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN];
	_mav_put_uint64_t(buf, 0, transfer_uid);
	_mav_put_uint32_t(buf, 8, file_size);
	_mav_put_uint8_t(buf, 252, direction);
	_mav_put_uint8_t(buf, 253, flags);
	_mav_put_char_array(buf, 12, dest_path, 240);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN);
#else
	mavlink_file_transfer_start_t packet;
	packet.transfer_uid = transfer_uid;
	packet.file_size = file_size;
	packet.direction = direction;
	packet.flags = flags;
	mav_array_memcpy(packet.dest_path, dest_path, sizeof(char)*240);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FILE_TRANSFER_START;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_START_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN);
#endif
}

/**
 * @brief Pack a file_transfer_start message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param transfer_uid Unique transfer ID
 * @param dest_path Destination path
 * @param direction Transfer direction: 0: from requester, 1: to requester
 * @param file_size File size in bytes
 * @param flags RESERVED
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_transfer_start_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t transfer_uid,const char *dest_path,uint8_t direction,uint32_t file_size,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN];
	_mav_put_uint64_t(buf, 0, transfer_uid);
	_mav_put_uint32_t(buf, 8, file_size);
	_mav_put_uint8_t(buf, 252, direction);
	_mav_put_uint8_t(buf, 253, flags);
	_mav_put_char_array(buf, 12, dest_path, 240);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN);
#else
	mavlink_file_transfer_start_t packet;
	packet.transfer_uid = transfer_uid;
	packet.file_size = file_size;
	packet.direction = direction;
	packet.flags = flags;
	mav_array_memcpy(packet.dest_path, dest_path, sizeof(char)*240);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FILE_TRANSFER_START;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_START_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN);
#endif
}

/**
 * @brief Encode a file_transfer_start struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param file_transfer_start C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_file_transfer_start_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_file_transfer_start_t* file_transfer_start)
{
	return mavlink_msg_file_transfer_start_pack(system_id, component_id, msg, file_transfer_start->transfer_uid, file_transfer_start->dest_path, file_transfer_start->direction, file_transfer_start->file_size, file_transfer_start->flags);
}

/**
 * @brief Send a file_transfer_start message
 * @param chan MAVLink channel to send the message
 *
 * @param transfer_uid Unique transfer ID
 * @param dest_path Destination path
 * @param direction Transfer direction: 0: from requester, 1: to requester
 * @param file_size File size in bytes
 * @param flags RESERVED
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_file_transfer_start_send(mavlink_channel_t chan, uint64_t transfer_uid, const char *dest_path, uint8_t direction, uint32_t file_size, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN];
	_mav_put_uint64_t(buf, 0, transfer_uid);
	_mav_put_uint32_t(buf, 8, file_size);
	_mav_put_uint8_t(buf, 252, direction);
	_mav_put_uint8_t(buf, 253, flags);
	_mav_put_char_array(buf, 12, dest_path, 240);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_START, buf, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_START_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_START, buf, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN);
#endif
#else
	mavlink_file_transfer_start_t packet;
	packet.transfer_uid = transfer_uid;
	packet.file_size = file_size;
	packet.direction = direction;
	packet.flags = flags;
	mav_array_memcpy(packet.dest_path, dest_path, sizeof(char)*240);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_START, (const char *)&packet, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_START_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_START, (const char *)&packet, MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN);
#endif
#endif
}

#endif

// MESSAGE FILE_TRANSFER_START UNPACKING


/**
 * @brief Get field transfer_uid from file_transfer_start message
 *
 * @return Unique transfer ID
 */
static inline uint64_t mavlink_msg_file_transfer_start_get_transfer_uid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field dest_path from file_transfer_start message
 *
 * @return Destination path
 */
static inline uint16_t mavlink_msg_file_transfer_start_get_dest_path(const mavlink_message_t* msg, char *dest_path)
{
	return _MAV_RETURN_char_array(msg, dest_path, 240,  12);
}

/**
 * @brief Get field direction from file_transfer_start message
 *
 * @return Transfer direction: 0: from requester, 1: to requester
 */
static inline uint8_t mavlink_msg_file_transfer_start_get_direction(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  252);
}

/**
 * @brief Get field file_size from file_transfer_start message
 *
 * @return File size in bytes
 */
static inline uint32_t mavlink_msg_file_transfer_start_get_file_size(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field flags from file_transfer_start message
 *
 * @return RESERVED
 */
static inline uint8_t mavlink_msg_file_transfer_start_get_flags(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  253);
}

/**
 * @brief Decode a file_transfer_start message into a struct
 *
 * @param msg The message to decode
 * @param file_transfer_start C-struct to decode the message contents into
 */
static inline void mavlink_msg_file_transfer_start_decode(const mavlink_message_t* msg, mavlink_file_transfer_start_t* file_transfer_start)
{
#if MAVLINK_NEED_BYTE_SWAP
	file_transfer_start->transfer_uid = mavlink_msg_file_transfer_start_get_transfer_uid(msg);
	file_transfer_start->file_size = mavlink_msg_file_transfer_start_get_file_size(msg);
	mavlink_msg_file_transfer_start_get_dest_path(msg, file_transfer_start->dest_path);
	file_transfer_start->direction = mavlink_msg_file_transfer_start_get_direction(msg);
	file_transfer_start->flags = mavlink_msg_file_transfer_start_get_flags(msg);
#else
	memcpy(file_transfer_start, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_FILE_TRANSFER_START_LEN);
#endif
}
