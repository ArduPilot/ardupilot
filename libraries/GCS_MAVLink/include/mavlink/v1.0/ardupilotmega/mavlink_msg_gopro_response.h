// MESSAGE GOPRO_RESPONSE PACKING

#define MAVLINK_MSG_ID_GOPRO_RESPONSE 218

typedef struct __mavlink_gopro_response_t
{
 uint16_t gp_cmd_result; ///< Result of the command attempt to the GoPro, as defined by GOPRO_CMD_RESULT enum.
 uint8_t gp_cmd_name_1; ///< First character of the 2 character GoPro command that generated this response
 uint8_t gp_cmd_name_2; ///< Second character of the 2 character GoPro command that generated this response
 uint8_t gp_cmd_response_status; ///< Response byte from the GoPro's response to the command.  0 = Success, 1 = Failure
 uint8_t gp_cmd_response_argument; ///< Response argument from the GoPro's response to the command
} mavlink_gopro_response_t;

#define MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN 6
#define MAVLINK_MSG_ID_218_LEN 6

#define MAVLINK_MSG_ID_GOPRO_RESPONSE_CRC 149
#define MAVLINK_MSG_ID_218_CRC 149



#define MAVLINK_MESSAGE_INFO_GOPRO_RESPONSE { \
	"GOPRO_RESPONSE", \
	5, \
	{  { "gp_cmd_result", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_gopro_response_t, gp_cmd_result) }, \
         { "gp_cmd_name_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gopro_response_t, gp_cmd_name_1) }, \
         { "gp_cmd_name_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_gopro_response_t, gp_cmd_name_2) }, \
         { "gp_cmd_response_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_gopro_response_t, gp_cmd_response_status) }, \
         { "gp_cmd_response_argument", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_gopro_response_t, gp_cmd_response_argument) }, \
         } \
}


/**
 * @brief Pack a gopro_response message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gp_cmd_name_1 First character of the 2 character GoPro command that generated this response
 * @param gp_cmd_name_2 Second character of the 2 character GoPro command that generated this response
 * @param gp_cmd_response_status Response byte from the GoPro's response to the command.  0 = Success, 1 = Failure
 * @param gp_cmd_response_argument Response argument from the GoPro's response to the command
 * @param gp_cmd_result Result of the command attempt to the GoPro, as defined by GOPRO_CMD_RESULT enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gopro_response_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t gp_cmd_name_1, uint8_t gp_cmd_name_2, uint8_t gp_cmd_response_status, uint8_t gp_cmd_response_argument, uint16_t gp_cmd_result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN];
	_mav_put_uint16_t(buf, 0, gp_cmd_result);
	_mav_put_uint8_t(buf, 2, gp_cmd_name_1);
	_mav_put_uint8_t(buf, 3, gp_cmd_name_2);
	_mav_put_uint8_t(buf, 4, gp_cmd_response_status);
	_mav_put_uint8_t(buf, 5, gp_cmd_response_argument);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#else
	mavlink_gopro_response_t packet;
	packet.gp_cmd_result = gp_cmd_result;
	packet.gp_cmd_name_1 = gp_cmd_name_1;
	packet.gp_cmd_name_2 = gp_cmd_name_2;
	packet.gp_cmd_response_status = gp_cmd_response_status;
	packet.gp_cmd_response_argument = gp_cmd_response_argument;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GOPRO_RESPONSE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN, MAVLINK_MSG_ID_GOPRO_RESPONSE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#endif
}

/**
 * @brief Pack a gopro_response message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gp_cmd_name_1 First character of the 2 character GoPro command that generated this response
 * @param gp_cmd_name_2 Second character of the 2 character GoPro command that generated this response
 * @param gp_cmd_response_status Response byte from the GoPro's response to the command.  0 = Success, 1 = Failure
 * @param gp_cmd_response_argument Response argument from the GoPro's response to the command
 * @param gp_cmd_result Result of the command attempt to the GoPro, as defined by GOPRO_CMD_RESULT enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gopro_response_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t gp_cmd_name_1,uint8_t gp_cmd_name_2,uint8_t gp_cmd_response_status,uint8_t gp_cmd_response_argument,uint16_t gp_cmd_result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN];
	_mav_put_uint16_t(buf, 0, gp_cmd_result);
	_mav_put_uint8_t(buf, 2, gp_cmd_name_1);
	_mav_put_uint8_t(buf, 3, gp_cmd_name_2);
	_mav_put_uint8_t(buf, 4, gp_cmd_response_status);
	_mav_put_uint8_t(buf, 5, gp_cmd_response_argument);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#else
	mavlink_gopro_response_t packet;
	packet.gp_cmd_result = gp_cmd_result;
	packet.gp_cmd_name_1 = gp_cmd_name_1;
	packet.gp_cmd_name_2 = gp_cmd_name_2;
	packet.gp_cmd_response_status = gp_cmd_response_status;
	packet.gp_cmd_response_argument = gp_cmd_response_argument;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GOPRO_RESPONSE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN, MAVLINK_MSG_ID_GOPRO_RESPONSE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#endif
}

/**
 * @brief Encode a gopro_response struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gopro_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gopro_response_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gopro_response_t* gopro_response)
{
	return mavlink_msg_gopro_response_pack(system_id, component_id, msg, gopro_response->gp_cmd_name_1, gopro_response->gp_cmd_name_2, gopro_response->gp_cmd_response_status, gopro_response->gp_cmd_response_argument, gopro_response->gp_cmd_result);
}

/**
 * @brief Encode a gopro_response struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gopro_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gopro_response_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gopro_response_t* gopro_response)
{
	return mavlink_msg_gopro_response_pack_chan(system_id, component_id, chan, msg, gopro_response->gp_cmd_name_1, gopro_response->gp_cmd_name_2, gopro_response->gp_cmd_response_status, gopro_response->gp_cmd_response_argument, gopro_response->gp_cmd_result);
}

/**
 * @brief Send a gopro_response message
 * @param chan MAVLink channel to send the message
 *
 * @param gp_cmd_name_1 First character of the 2 character GoPro command that generated this response
 * @param gp_cmd_name_2 Second character of the 2 character GoPro command that generated this response
 * @param gp_cmd_response_status Response byte from the GoPro's response to the command.  0 = Success, 1 = Failure
 * @param gp_cmd_response_argument Response argument from the GoPro's response to the command
 * @param gp_cmd_result Result of the command attempt to the GoPro, as defined by GOPRO_CMD_RESULT enum.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gopro_response_send(mavlink_channel_t chan, uint8_t gp_cmd_name_1, uint8_t gp_cmd_name_2, uint8_t gp_cmd_response_status, uint8_t gp_cmd_response_argument, uint16_t gp_cmd_result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN];
	_mav_put_uint16_t(buf, 0, gp_cmd_result);
	_mav_put_uint8_t(buf, 2, gp_cmd_name_1);
	_mav_put_uint8_t(buf, 3, gp_cmd_name_2);
	_mav_put_uint8_t(buf, 4, gp_cmd_response_status);
	_mav_put_uint8_t(buf, 5, gp_cmd_response_argument);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_RESPONSE, buf, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN, MAVLINK_MSG_ID_GOPRO_RESPONSE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_RESPONSE, buf, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#endif
#else
	mavlink_gopro_response_t packet;
	packet.gp_cmd_result = gp_cmd_result;
	packet.gp_cmd_name_1 = gp_cmd_name_1;
	packet.gp_cmd_name_2 = gp_cmd_name_2;
	packet.gp_cmd_response_status = gp_cmd_response_status;
	packet.gp_cmd_response_argument = gp_cmd_response_argument;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_RESPONSE, (const char *)&packet, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN, MAVLINK_MSG_ID_GOPRO_RESPONSE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_RESPONSE, (const char *)&packet, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gopro_response_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t gp_cmd_name_1, uint8_t gp_cmd_name_2, uint8_t gp_cmd_response_status, uint8_t gp_cmd_response_argument, uint16_t gp_cmd_result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, gp_cmd_result);
	_mav_put_uint8_t(buf, 2, gp_cmd_name_1);
	_mav_put_uint8_t(buf, 3, gp_cmd_name_2);
	_mav_put_uint8_t(buf, 4, gp_cmd_response_status);
	_mav_put_uint8_t(buf, 5, gp_cmd_response_argument);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_RESPONSE, buf, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN, MAVLINK_MSG_ID_GOPRO_RESPONSE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_RESPONSE, buf, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#endif
#else
	mavlink_gopro_response_t *packet = (mavlink_gopro_response_t *)msgbuf;
	packet->gp_cmd_result = gp_cmd_result;
	packet->gp_cmd_name_1 = gp_cmd_name_1;
	packet->gp_cmd_name_2 = gp_cmd_name_2;
	packet->gp_cmd_response_status = gp_cmd_response_status;
	packet->gp_cmd_response_argument = gp_cmd_response_argument;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_RESPONSE, (const char *)packet, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN, MAVLINK_MSG_ID_GOPRO_RESPONSE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GOPRO_RESPONSE, (const char *)packet, MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GOPRO_RESPONSE UNPACKING


/**
 * @brief Get field gp_cmd_name_1 from gopro_response message
 *
 * @return First character of the 2 character GoPro command that generated this response
 */
static inline uint8_t mavlink_msg_gopro_response_get_gp_cmd_name_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field gp_cmd_name_2 from gopro_response message
 *
 * @return Second character of the 2 character GoPro command that generated this response
 */
static inline uint8_t mavlink_msg_gopro_response_get_gp_cmd_name_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field gp_cmd_response_status from gopro_response message
 *
 * @return Response byte from the GoPro's response to the command.  0 = Success, 1 = Failure
 */
static inline uint8_t mavlink_msg_gopro_response_get_gp_cmd_response_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field gp_cmd_response_argument from gopro_response message
 *
 * @return Response argument from the GoPro's response to the command
 */
static inline uint8_t mavlink_msg_gopro_response_get_gp_cmd_response_argument(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field gp_cmd_result from gopro_response message
 *
 * @return Result of the command attempt to the GoPro, as defined by GOPRO_CMD_RESULT enum.
 */
static inline uint16_t mavlink_msg_gopro_response_get_gp_cmd_result(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a gopro_response message into a struct
 *
 * @param msg The message to decode
 * @param gopro_response C-struct to decode the message contents into
 */
static inline void mavlink_msg_gopro_response_decode(const mavlink_message_t* msg, mavlink_gopro_response_t* gopro_response)
{
#if MAVLINK_NEED_BYTE_SWAP
	gopro_response->gp_cmd_result = mavlink_msg_gopro_response_get_gp_cmd_result(msg);
	gopro_response->gp_cmd_name_1 = mavlink_msg_gopro_response_get_gp_cmd_name_1(msg);
	gopro_response->gp_cmd_name_2 = mavlink_msg_gopro_response_get_gp_cmd_name_2(msg);
	gopro_response->gp_cmd_response_status = mavlink_msg_gopro_response_get_gp_cmd_response_status(msg);
	gopro_response->gp_cmd_response_argument = mavlink_msg_gopro_response_get_gp_cmd_response_argument(msg);
#else
	memcpy(gopro_response, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GOPRO_RESPONSE_LEN);
#endif
}
