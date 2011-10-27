// MESSAGE DIAGNOSTIC PACKING

#define MAVLINK_MSG_ID_DIAGNOSTIC 173

typedef struct __mavlink_diagnostic_t
{
 float diagFl1; ///< Diagnostic float 1
 float diagFl2; ///< Diagnostic float 2
 float diagFl3; ///< Diagnostic float 3
 int16_t diagSh1; ///< Diagnostic short 1
 int16_t diagSh2; ///< Diagnostic short 2
 int16_t diagSh3; ///< Diagnostic short 3
} mavlink_diagnostic_t;

#define MAVLINK_MSG_ID_DIAGNOSTIC_LEN 18
#define MAVLINK_MSG_ID_173_LEN 18



#define MAVLINK_MESSAGE_INFO_DIAGNOSTIC { \
	"DIAGNOSTIC", \
	6, \
	{  { "diagFl1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_diagnostic_t, diagFl1) }, \
         { "diagFl2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_diagnostic_t, diagFl2) }, \
         { "diagFl3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_diagnostic_t, diagFl3) }, \
         { "diagSh1", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_diagnostic_t, diagSh1) }, \
         { "diagSh2", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_diagnostic_t, diagSh2) }, \
         { "diagSh3", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_diagnostic_t, diagSh3) }, \
         } \
}


/**
 * @brief Pack a diagnostic message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param diagFl1 Diagnostic float 1
 * @param diagFl2 Diagnostic float 2
 * @param diagFl3 Diagnostic float 3
 * @param diagSh1 Diagnostic short 1
 * @param diagSh2 Diagnostic short 2
 * @param diagSh3 Diagnostic short 3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_diagnostic_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float diagFl1, float diagFl2, float diagFl3, int16_t diagSh1, int16_t diagSh2, int16_t diagSh3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, diagFl1);
	_mav_put_float(buf, 4, diagFl2);
	_mav_put_float(buf, 8, diagFl3);
	_mav_put_int16_t(buf, 12, diagSh1);
	_mav_put_int16_t(buf, 14, diagSh2);
	_mav_put_int16_t(buf, 16, diagSh3);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_diagnostic_t packet;
	packet.diagFl1 = diagFl1;
	packet.diagFl2 = diagFl2;
	packet.diagFl3 = diagFl3;
	packet.diagSh1 = diagSh1;
	packet.diagSh2 = diagSh2;
	packet.diagSh3 = diagSh3;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_DIAGNOSTIC;
	return mavlink_finalize_message(msg, system_id, component_id, 18, 2);
}

/**
 * @brief Pack a diagnostic message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param diagFl1 Diagnostic float 1
 * @param diagFl2 Diagnostic float 2
 * @param diagFl3 Diagnostic float 3
 * @param diagSh1 Diagnostic short 1
 * @param diagSh2 Diagnostic short 2
 * @param diagSh3 Diagnostic short 3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_diagnostic_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float diagFl1,float diagFl2,float diagFl3,int16_t diagSh1,int16_t diagSh2,int16_t diagSh3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, diagFl1);
	_mav_put_float(buf, 4, diagFl2);
	_mav_put_float(buf, 8, diagFl3);
	_mav_put_int16_t(buf, 12, diagSh1);
	_mav_put_int16_t(buf, 14, diagSh2);
	_mav_put_int16_t(buf, 16, diagSh3);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_diagnostic_t packet;
	packet.diagFl1 = diagFl1;
	packet.diagFl2 = diagFl2;
	packet.diagFl3 = diagFl3;
	packet.diagSh1 = diagSh1;
	packet.diagSh2 = diagSh2;
	packet.diagSh3 = diagSh3;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_DIAGNOSTIC;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 2);
}

/**
 * @brief Encode a diagnostic struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param diagnostic C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_diagnostic_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_diagnostic_t* diagnostic)
{
	return mavlink_msg_diagnostic_pack(system_id, component_id, msg, diagnostic->diagFl1, diagnostic->diagFl2, diagnostic->diagFl3, diagnostic->diagSh1, diagnostic->diagSh2, diagnostic->diagSh3);
}

/**
 * @brief Send a diagnostic message
 * @param chan MAVLink channel to send the message
 *
 * @param diagFl1 Diagnostic float 1
 * @param diagFl2 Diagnostic float 2
 * @param diagFl3 Diagnostic float 3
 * @param diagSh1 Diagnostic short 1
 * @param diagSh2 Diagnostic short 2
 * @param diagSh3 Diagnostic short 3
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_diagnostic_send(mavlink_channel_t chan, float diagFl1, float diagFl2, float diagFl3, int16_t diagSh1, int16_t diagSh2, int16_t diagSh3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_float(buf, 0, diagFl1);
	_mav_put_float(buf, 4, diagFl2);
	_mav_put_float(buf, 8, diagFl3);
	_mav_put_int16_t(buf, 12, diagSh1);
	_mav_put_int16_t(buf, 14, diagSh2);
	_mav_put_int16_t(buf, 16, diagSh3);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DIAGNOSTIC, buf, 18, 2);
#else
	mavlink_diagnostic_t packet;
	packet.diagFl1 = diagFl1;
	packet.diagFl2 = diagFl2;
	packet.diagFl3 = diagFl3;
	packet.diagSh1 = diagSh1;
	packet.diagSh2 = diagSh2;
	packet.diagSh3 = diagSh3;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DIAGNOSTIC, (const char *)&packet, 18, 2);
#endif
}

#endif

// MESSAGE DIAGNOSTIC UNPACKING


/**
 * @brief Get field diagFl1 from diagnostic message
 *
 * @return Diagnostic float 1
 */
static inline float mavlink_msg_diagnostic_get_diagFl1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field diagFl2 from diagnostic message
 *
 * @return Diagnostic float 2
 */
static inline float mavlink_msg_diagnostic_get_diagFl2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field diagFl3 from diagnostic message
 *
 * @return Diagnostic float 3
 */
static inline float mavlink_msg_diagnostic_get_diagFl3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field diagSh1 from diagnostic message
 *
 * @return Diagnostic short 1
 */
static inline int16_t mavlink_msg_diagnostic_get_diagSh1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field diagSh2 from diagnostic message
 *
 * @return Diagnostic short 2
 */
static inline int16_t mavlink_msg_diagnostic_get_diagSh2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field diagSh3 from diagnostic message
 *
 * @return Diagnostic short 3
 */
static inline int16_t mavlink_msg_diagnostic_get_diagSh3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Decode a diagnostic message into a struct
 *
 * @param msg The message to decode
 * @param diagnostic C-struct to decode the message contents into
 */
static inline void mavlink_msg_diagnostic_decode(const mavlink_message_t* msg, mavlink_diagnostic_t* diagnostic)
{
#if MAVLINK_NEED_BYTE_SWAP
	diagnostic->diagFl1 = mavlink_msg_diagnostic_get_diagFl1(msg);
	diagnostic->diagFl2 = mavlink_msg_diagnostic_get_diagFl2(msg);
	diagnostic->diagFl3 = mavlink_msg_diagnostic_get_diagFl3(msg);
	diagnostic->diagSh1 = mavlink_msg_diagnostic_get_diagSh1(msg);
	diagnostic->diagSh2 = mavlink_msg_diagnostic_get_diagSh2(msg);
	diagnostic->diagSh3 = mavlink_msg_diagnostic_get_diagSh3(msg);
#else
	memcpy(diagnostic, _MAV_PAYLOAD(msg), 18);
#endif
}
