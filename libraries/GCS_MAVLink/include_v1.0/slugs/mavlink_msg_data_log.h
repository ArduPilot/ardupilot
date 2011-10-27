// MESSAGE DATA_LOG PACKING

#define MAVLINK_MSG_ID_DATA_LOG 177

typedef struct __mavlink_data_log_t
{
 float fl_1; ///< Log value 1 
 float fl_2; ///< Log value 2 
 float fl_3; ///< Log value 3 
 float fl_4; ///< Log value 4 
 float fl_5; ///< Log value 5 
 float fl_6; ///< Log value 6 
} mavlink_data_log_t;

#define MAVLINK_MSG_ID_DATA_LOG_LEN 24
#define MAVLINK_MSG_ID_177_LEN 24



#define MAVLINK_MESSAGE_INFO_DATA_LOG { \
	"DATA_LOG", \
	6, \
	{  { "fl_1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_data_log_t, fl_1) }, \
         { "fl_2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_data_log_t, fl_2) }, \
         { "fl_3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_data_log_t, fl_3) }, \
         { "fl_4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_data_log_t, fl_4) }, \
         { "fl_5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_data_log_t, fl_5) }, \
         { "fl_6", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_data_log_t, fl_6) }, \
         } \
}


/**
 * @brief Pack a data_log message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param fl_1 Log value 1 
 * @param fl_2 Log value 2 
 * @param fl_3 Log value 3 
 * @param fl_4 Log value 4 
 * @param fl_5 Log value 5 
 * @param fl_6 Log value 6 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_log_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float fl_1, float fl_2, float fl_3, float fl_4, float fl_5, float fl_6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_float(buf, 0, fl_1);
	_mav_put_float(buf, 4, fl_2);
	_mav_put_float(buf, 8, fl_3);
	_mav_put_float(buf, 12, fl_4);
	_mav_put_float(buf, 16, fl_5);
	_mav_put_float(buf, 20, fl_6);

        memcpy(_MAV_PAYLOAD(msg), buf, 24);
#else
	mavlink_data_log_t packet;
	packet.fl_1 = fl_1;
	packet.fl_2 = fl_2;
	packet.fl_3 = fl_3;
	packet.fl_4 = fl_4;
	packet.fl_5 = fl_5;
	packet.fl_6 = fl_6;

        memcpy(_MAV_PAYLOAD(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA_LOG;
	return mavlink_finalize_message(msg, system_id, component_id, 24, 167);
}

/**
 * @brief Pack a data_log message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param fl_1 Log value 1 
 * @param fl_2 Log value 2 
 * @param fl_3 Log value 3 
 * @param fl_4 Log value 4 
 * @param fl_5 Log value 5 
 * @param fl_6 Log value 6 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_data_log_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float fl_1,float fl_2,float fl_3,float fl_4,float fl_5,float fl_6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_float(buf, 0, fl_1);
	_mav_put_float(buf, 4, fl_2);
	_mav_put_float(buf, 8, fl_3);
	_mav_put_float(buf, 12, fl_4);
	_mav_put_float(buf, 16, fl_5);
	_mav_put_float(buf, 20, fl_6);

        memcpy(_MAV_PAYLOAD(msg), buf, 24);
#else
	mavlink_data_log_t packet;
	packet.fl_1 = fl_1;
	packet.fl_2 = fl_2;
	packet.fl_3 = fl_3;
	packet.fl_4 = fl_4;
	packet.fl_5 = fl_5;
	packet.fl_6 = fl_6;

        memcpy(_MAV_PAYLOAD(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_DATA_LOG;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 24, 167);
}

/**
 * @brief Encode a data_log struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data_log C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_data_log_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data_log_t* data_log)
{
	return mavlink_msg_data_log_pack(system_id, component_id, msg, data_log->fl_1, data_log->fl_2, data_log->fl_3, data_log->fl_4, data_log->fl_5, data_log->fl_6);
}

/**
 * @brief Send a data_log message
 * @param chan MAVLink channel to send the message
 *
 * @param fl_1 Log value 1 
 * @param fl_2 Log value 2 
 * @param fl_3 Log value 3 
 * @param fl_4 Log value 4 
 * @param fl_5 Log value 5 
 * @param fl_6 Log value 6 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data_log_send(mavlink_channel_t chan, float fl_1, float fl_2, float fl_3, float fl_4, float fl_5, float fl_6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_float(buf, 0, fl_1);
	_mav_put_float(buf, 4, fl_2);
	_mav_put_float(buf, 8, fl_3);
	_mav_put_float(buf, 12, fl_4);
	_mav_put_float(buf, 16, fl_5);
	_mav_put_float(buf, 20, fl_6);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_LOG, buf, 24, 167);
#else
	mavlink_data_log_t packet;
	packet.fl_1 = fl_1;
	packet.fl_2 = fl_2;
	packet.fl_3 = fl_3;
	packet.fl_4 = fl_4;
	packet.fl_5 = fl_5;
	packet.fl_6 = fl_6;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_LOG, (const char *)&packet, 24, 167);
#endif
}

#endif

// MESSAGE DATA_LOG UNPACKING


/**
 * @brief Get field fl_1 from data_log message
 *
 * @return Log value 1 
 */
static inline float mavlink_msg_data_log_get_fl_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field fl_2 from data_log message
 *
 * @return Log value 2 
 */
static inline float mavlink_msg_data_log_get_fl_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field fl_3 from data_log message
 *
 * @return Log value 3 
 */
static inline float mavlink_msg_data_log_get_fl_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field fl_4 from data_log message
 *
 * @return Log value 4 
 */
static inline float mavlink_msg_data_log_get_fl_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field fl_5 from data_log message
 *
 * @return Log value 5 
 */
static inline float mavlink_msg_data_log_get_fl_5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field fl_6 from data_log message
 *
 * @return Log value 6 
 */
static inline float mavlink_msg_data_log_get_fl_6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a data_log message into a struct
 *
 * @param msg The message to decode
 * @param data_log C-struct to decode the message contents into
 */
static inline void mavlink_msg_data_log_decode(const mavlink_message_t* msg, mavlink_data_log_t* data_log)
{
#if MAVLINK_NEED_BYTE_SWAP
	data_log->fl_1 = mavlink_msg_data_log_get_fl_1(msg);
	data_log->fl_2 = mavlink_msg_data_log_get_fl_2(msg);
	data_log->fl_3 = mavlink_msg_data_log_get_fl_3(msg);
	data_log->fl_4 = mavlink_msg_data_log_get_fl_4(msg);
	data_log->fl_5 = mavlink_msg_data_log_get_fl_5(msg);
	data_log->fl_6 = mavlink_msg_data_log_get_fl_6(msg);
#else
	memcpy(data_log, _MAV_PAYLOAD(msg), 24);
#endif
}
