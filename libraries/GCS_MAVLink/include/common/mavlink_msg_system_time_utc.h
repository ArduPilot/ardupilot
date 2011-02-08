// MESSAGE SYSTEM_TIME_UTC PACKING

#define MAVLINK_MSG_ID_SYSTEM_TIME_UTC 4

typedef struct __mavlink_system_time_utc_t 
{
	uint32_t utc_date; ///< GPS UTC date ddmmyy
	uint32_t utc_time; ///< GPS UTC time hhmmss

} mavlink_system_time_utc_t;



/**
 * @brief Pack a system_time_utc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param utc_date GPS UTC date ddmmyy
 * @param utc_time GPS UTC time hhmmss
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_time_utc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t utc_date, uint32_t utc_time)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME_UTC;

	i += put_uint32_t_by_index(utc_date, i, msg->payload); // GPS UTC date ddmmyy
	i += put_uint32_t_by_index(utc_time, i, msg->payload); // GPS UTC time hhmmss

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a system_time_utc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param utc_date GPS UTC date ddmmyy
 * @param utc_time GPS UTC time hhmmss
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_time_utc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint32_t utc_date, uint32_t utc_time)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME_UTC;

	i += put_uint32_t_by_index(utc_date, i, msg->payload); // GPS UTC date ddmmyy
	i += put_uint32_t_by_index(utc_time, i, msg->payload); // GPS UTC time hhmmss

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a system_time_utc struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_time_utc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_time_utc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_system_time_utc_t* system_time_utc)
{
	return mavlink_msg_system_time_utc_pack(system_id, component_id, msg, system_time_utc->utc_date, system_time_utc->utc_time);
}

/**
 * @brief Send a system_time_utc message
 * @param chan MAVLink channel to send the message
 *
 * @param utc_date GPS UTC date ddmmyy
 * @param utc_time GPS UTC time hhmmss
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_system_time_utc_send(mavlink_channel_t chan, uint32_t utc_date, uint32_t utc_time)
{
	mavlink_message_t msg;
	mavlink_msg_system_time_utc_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, utc_date, utc_time);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SYSTEM_TIME_UTC UNPACKING

/**
 * @brief Get field utc_date from system_time_utc message
 *
 * @return GPS UTC date ddmmyy
 */
static inline uint32_t mavlink_msg_system_time_utc_get_utc_date(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field utc_time from system_time_utc message
 *
 * @return GPS UTC time hhmmss
 */
static inline uint32_t mavlink_msg_system_time_utc_get_utc_time(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint32_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Decode a system_time_utc message into a struct
 *
 * @param msg The message to decode
 * @param system_time_utc C-struct to decode the message contents into
 */
static inline void mavlink_msg_system_time_utc_decode(const mavlink_message_t* msg, mavlink_system_time_utc_t* system_time_utc)
{
	system_time_utc->utc_date = mavlink_msg_system_time_utc_get_utc_date(msg);
	system_time_utc->utc_time = mavlink_msg_system_time_utc_get_utc_time(msg);
}
