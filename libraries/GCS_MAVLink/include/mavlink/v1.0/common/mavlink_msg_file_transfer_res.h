// MESSAGE FILE_TRANSFER_RES PACKING

#define MAVLINK_MSG_ID_FILE_TRANSFER_RES 112

typedef struct __mavlink_file_transfer_res_t
{
 uint64_t transfer_uid; ///< Unique transfer ID
 uint8_t result; ///< 0: OK, 1: not permitted, 2: bad path / file name, 3: no space left on device
} mavlink_file_transfer_res_t;

#define MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN 9
#define MAVLINK_MSG_ID_112_LEN 9

#define MAVLINK_MSG_ID_FILE_TRANSFER_RES_CRC 124
#define MAVLINK_MSG_ID_112_CRC 124



#define MAVLINK_MESSAGE_INFO_FILE_TRANSFER_RES { \
	"FILE_TRANSFER_RES", \
	2, \
	{  { "transfer_uid", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_file_transfer_res_t, transfer_uid) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_file_transfer_res_t, result) }, \
         } \
}


/**
 * @brief Pack a file_transfer_res message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param transfer_uid Unique transfer ID
 * @param result 0: OK, 1: not permitted, 2: bad path / file name, 3: no space left on device
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_transfer_res_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t transfer_uid, uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN];
	_mav_put_uint64_t(buf, 0, transfer_uid);
	_mav_put_uint8_t(buf, 8, result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN);
#else
	mavlink_file_transfer_res_t packet;
	packet.transfer_uid = transfer_uid;
	packet.result = result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FILE_TRANSFER_RES;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_RES_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN);
#endif
}

/**
 * @brief Pack a file_transfer_res message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param transfer_uid Unique transfer ID
 * @param result 0: OK, 1: not permitted, 2: bad path / file name, 3: no space left on device
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_file_transfer_res_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t transfer_uid,uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN];
	_mav_put_uint64_t(buf, 0, transfer_uid);
	_mav_put_uint8_t(buf, 8, result);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN);
#else
	mavlink_file_transfer_res_t packet;
	packet.transfer_uid = transfer_uid;
	packet.result = result;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FILE_TRANSFER_RES;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_RES_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN);
#endif
}

/**
 * @brief Encode a file_transfer_res struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param file_transfer_res C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_file_transfer_res_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_file_transfer_res_t* file_transfer_res)
{
	return mavlink_msg_file_transfer_res_pack(system_id, component_id, msg, file_transfer_res->transfer_uid, file_transfer_res->result);
}

/**
 * @brief Send a file_transfer_res message
 * @param chan MAVLink channel to send the message
 *
 * @param transfer_uid Unique transfer ID
 * @param result 0: OK, 1: not permitted, 2: bad path / file name, 3: no space left on device
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_file_transfer_res_send(mavlink_channel_t chan, uint64_t transfer_uid, uint8_t result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN];
	_mav_put_uint64_t(buf, 0, transfer_uid);
	_mav_put_uint8_t(buf, 8, result);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_RES, buf, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_RES_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_RES, buf, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN);
#endif
#else
	mavlink_file_transfer_res_t packet;
	packet.transfer_uid = transfer_uid;
	packet.result = result;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_RES, (const char *)&packet, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN, MAVLINK_MSG_ID_FILE_TRANSFER_RES_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILE_TRANSFER_RES, (const char *)&packet, MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN);
#endif
#endif
}

#endif

// MESSAGE FILE_TRANSFER_RES UNPACKING


/**
 * @brief Get field transfer_uid from file_transfer_res message
 *
 * @return Unique transfer ID
 */
static inline uint64_t mavlink_msg_file_transfer_res_get_transfer_uid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field result from file_transfer_res message
 *
 * @return 0: OK, 1: not permitted, 2: bad path / file name, 3: no space left on device
 */
static inline uint8_t mavlink_msg_file_transfer_res_get_result(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a file_transfer_res message into a struct
 *
 * @param msg The message to decode
 * @param file_transfer_res C-struct to decode the message contents into
 */
static inline void mavlink_msg_file_transfer_res_decode(const mavlink_message_t* msg, mavlink_file_transfer_res_t* file_transfer_res)
{
#if MAVLINK_NEED_BYTE_SWAP
	file_transfer_res->transfer_uid = mavlink_msg_file_transfer_res_get_transfer_uid(msg);
	file_transfer_res->result = mavlink_msg_file_transfer_res_get_result(msg);
#else
	memcpy(file_transfer_res, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_FILE_TRANSFER_RES_LEN);
#endif
}
