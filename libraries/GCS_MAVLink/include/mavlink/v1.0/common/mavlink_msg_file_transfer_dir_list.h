// MESSAGE FILE_TRANSFER_DIR_LIST PACKING

#define MAVLINK_MSG_ID_FILE_TRANSFER_DIR_LIST 111

typedef struct __mavlink_file_transfer_dir_list_t
{
 uint64_t transfer_uid; ///< Unique transfer ID
 char dir_path[240]; ///< Directory path to list
 uint8_t flags; ///< RESERVED
} mavlink_file_transfer_dir_list_t;

#define MAVLINK_MSG_ID_FILE_TRANSFER_DIR_LIST_LEN 249
#define MAVLINK_MSG_ID_111_LEN 249

#define MAVLINK_MSG_FILE_TRANSFER_DIR_LIST_FIELD_DIR_PATH_LEN 240

#define MAVLINK_MESSAGE_INFO_FILE_TRANSFER_DIR_LIST { \
	"FILE_TRANSFER_DIR_LIST", \
	3, \
	{  { "transfer_uid", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_file_transfer_dir_list_t, transfer_uid) }, \
         { "dir_path", NULL, MAVLINK_TYPE_CHAR, 240, 8, offsetof(mavlink_file_transfer_dir_list_t, dir_path) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 248, offsetof(mavlink_file_transfer_dir_list_t, flags) }, \
         } \
}


/**
 * @brief Pack a file_transfer_dir_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param transfer_uid Unique transfer ID
 * @param dir_path Directory path to list
 * @param flags RESERVED
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_transfer_dir_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t transfer_uid, const char *dir_path, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[249];
	_mav_put_uint64_t(buf, 0, transfer_uid);
	_mav_put_uint8_t(buf, 248, flags);
	_mav_put_char_array(buf, 8, dir_path, 240);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 249);
#else
	mavlink_file_transfer_dir_list_t packet;
	packet.transfer_uid = transfer_uid;
	packet.flags = flags;
	mav_array_memcpy(packet.dir_path, dir_path, sizeof(char)*240);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 249);
#endif

	msg->msgid = MAVLINK_MSG_ID_FILE_TRANSFER_DIR_LIST;
	return mavlink_finalize_message(msg, system_id, component_id, 249, 93);
}

/**
 * @brief Pack a file_transfer_dir_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param transfer_uid Unique transfer ID
 * @param dir_path Directory path to list
 * @param flags RESERVED
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_transfer_dir_list_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t transfer_uid,const char *dir_path,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[249];
	_mav_put_uint64_t(buf, 0, transfer_uid);
	_mav_put_uint8_t(buf, 248, flags);
	_mav_put_char_array(buf, 8, dir_path, 240);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 249);
#else
	mavlink_file_transfer_dir_list_t packet;
	packet.transfer_uid = transfer_uid;
	packet.flags = flags;
	mav_array_memcpy(packet.dir_path, dir_path, sizeof(char)*240);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 249);
#endif

	msg->msgid = MAVLINK_MSG_ID_FILE_TRANSFER_DIR_LIST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 249, 93);
}

/**
 * @brief Encode a file_transfer_dir_list struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param file_transfer_dir_list C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_file_transfer_dir_list_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_file_transfer_dir_list_t* file_transfer_dir_list)
{
	return mavlink_msg_file_transfer_dir_list_pack(system_id, component_id, msg, file_transfer_dir_list->transfer_uid, file_transfer_dir_list->dir_path, file_transfer_dir_list->flags);
}

/**
 * @brief Send a file_transfer_dir_list message
 * @param chan MAVLink channel to send the message
 *
 * @param transfer_uid Unique transfer ID
 * @param dir_path Directory path to list
 * @param flags RESERVED
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_file_transfer_dir_list_send(mavlink_channel_t chan, uint64_t transfer_uid, const char *dir_path, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[249];
	_mav_put_uint64_t(buf, 0, transfer_uid);
	_mav_put_uint8_t(buf, 248, flags);
	_mav_put_char_array(buf, 8, dir_path, 240);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_DIR_LIST, buf, 249, 93);
#else
	mavlink_file_transfer_dir_list_t packet;
	packet.transfer_uid = transfer_uid;
	packet.flags = flags;
	mav_array_memcpy(packet.dir_path, dir_path, sizeof(char)*240);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_DIR_LIST, (const char *)&packet, 249, 93);
#endif
}

#endif

// MESSAGE FILE_TRANSFER_DIR_LIST UNPACKING


/**
 * @brief Get field transfer_uid from file_transfer_dir_list message
 *
 * @return Unique transfer ID
 */
static inline uint64_t mavlink_msg_file_transfer_dir_list_get_transfer_uid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field dir_path from file_transfer_dir_list message
 *
 * @return Directory path to list
 */
static inline uint16_t mavlink_msg_file_transfer_dir_list_get_dir_path(const mavlink_message_t* msg, char *dir_path)
{
	return _MAV_RETURN_char_array(msg, dir_path, 240,  8);
}

/**
 * @brief Get field flags from file_transfer_dir_list message
 *
 * @return RESERVED
 */
static inline uint8_t mavlink_msg_file_transfer_dir_list_get_flags(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  248);
}

/**
 * @brief Decode a file_transfer_dir_list message into a struct
 *
 * @param msg The message to decode
 * @param file_transfer_dir_list C-struct to decode the message contents into
 */
static inline void mavlink_msg_file_transfer_dir_list_decode(const mavlink_message_t* msg, mavlink_file_transfer_dir_list_t* file_transfer_dir_list)
{
#if MAVLINK_NEED_BYTE_SWAP
	file_transfer_dir_list->transfer_uid = mavlink_msg_file_transfer_dir_list_get_transfer_uid(msg);
	mavlink_msg_file_transfer_dir_list_get_dir_path(msg, file_transfer_dir_list->dir_path);
	file_transfer_dir_list->flags = mavlink_msg_file_transfer_dir_list_get_flags(msg);
#else
	memcpy(file_transfer_dir_list, _MAV_PAYLOAD(msg), 249);
#endif
}
