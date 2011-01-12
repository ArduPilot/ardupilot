// MESSAGE STATUSTEXT PACKING

#define MAVLINK_MSG_ID_STATUSTEXT 254

typedef struct __mavlink_statustext_t 
{
	uint8_t severity; ///< Severity of status, 0 = info message, 255 = critical fault
	int8_t text[50]; ///< Status text message, without null termination character

} mavlink_statustext_t;

#define MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN 50


/**
 * @brief Pack a statustext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param severity Severity of status, 0 = info message, 255 = critical fault
 * @param text Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t severity, const int8_t* text)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;

	i += put_uint8_t_by_index(severity, i, msg->payload); // Severity of status, 0 = info message, 255 = critical fault
	i += put_array_by_index(text, 50, i, msg->payload); // Status text message, without null termination character

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a statustext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param severity Severity of status, 0 = info message, 255 = critical fault
 * @param text Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t severity, const int8_t* text)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;

	i += put_uint8_t_by_index(severity, i, msg->payload); // Severity of status, 0 = info message, 255 = critical fault
	i += put_array_by_index(text, 50, i, msg->payload); // Status text message, without null termination character

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a statustext struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_statustext_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_statustext_t* statustext)
{
	return mavlink_msg_statustext_pack(system_id, component_id, msg, statustext->severity, statustext->text);
}

/**
 * @brief Send a statustext message
 * @param chan MAVLink channel to send the message
 *
 * @param severity Severity of status, 0 = info message, 255 = critical fault
 * @param text Status text message, without null termination character
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_statustext_send(mavlink_channel_t chan, uint8_t severity, const int8_t* text)
{
	mavlink_message_t msg;
	mavlink_msg_statustext_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, severity, text);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE STATUSTEXT UNPACKING

/**
 * @brief Get field severity from statustext message
 *
 * @return Severity of status, 0 = info message, 255 = critical fault
 */
static inline uint8_t mavlink_msg_statustext_get_severity(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field text from statustext message
 *
 * @return Status text message, without null termination character
 */
static inline uint16_t mavlink_msg_statustext_get_text(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint8_t), 50);
	return 50;
}

/**
 * @brief Decode a statustext message into a struct
 *
 * @param msg The message to decode
 * @param statustext C-struct to decode the message contents into
 */
static inline void mavlink_msg_statustext_decode(const mavlink_message_t* msg, mavlink_statustext_t* statustext)
{
	statustext->severity = mavlink_msg_statustext_get_severity(msg);
	mavlink_msg_statustext_get_text(msg, statustext->text);
}
