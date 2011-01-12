// MESSAGE RC_CHANNELS_RAW PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35

typedef struct __mavlink_rc_channels_raw_t 
{
	uint16_t chan1_raw; ///< RC channel 1 value, in microseconds
	uint16_t chan2_raw; ///< RC channel 2 value, in microseconds
	uint16_t chan3_raw; ///< RC channel 3 value, in microseconds
	uint16_t chan4_raw; ///< RC channel 4 value, in microseconds
	uint16_t chan5_raw; ///< RC channel 5 value, in microseconds
	uint16_t chan6_raw; ///< RC channel 6 value, in microseconds
	uint16_t chan7_raw; ///< RC channel 7 value, in microseconds
	uint16_t chan8_raw; ///< RC channel 8 value, in microseconds
	uint8_t rssi; ///< Receive signal strength indicator, 0: 0%, 255: 100%

} mavlink_rc_channels_raw_t;



/**
 * @brief Pack a rc_channels_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;

	i += put_uint16_t_by_index(chan1_raw, i, msg->payload); // RC channel 1 value, in microseconds
	i += put_uint16_t_by_index(chan2_raw, i, msg->payload); // RC channel 2 value, in microseconds
	i += put_uint16_t_by_index(chan3_raw, i, msg->payload); // RC channel 3 value, in microseconds
	i += put_uint16_t_by_index(chan4_raw, i, msg->payload); // RC channel 4 value, in microseconds
	i += put_uint16_t_by_index(chan5_raw, i, msg->payload); // RC channel 5 value, in microseconds
	i += put_uint16_t_by_index(chan6_raw, i, msg->payload); // RC channel 6 value, in microseconds
	i += put_uint16_t_by_index(chan7_raw, i, msg->payload); // RC channel 7 value, in microseconds
	i += put_uint16_t_by_index(chan8_raw, i, msg->payload); // RC channel 8 value, in microseconds
	i += put_uint8_t_by_index(rssi, i, msg->payload); // Receive signal strength indicator, 0: 0%, 255: 100%

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a rc_channels_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;

	i += put_uint16_t_by_index(chan1_raw, i, msg->payload); // RC channel 1 value, in microseconds
	i += put_uint16_t_by_index(chan2_raw, i, msg->payload); // RC channel 2 value, in microseconds
	i += put_uint16_t_by_index(chan3_raw, i, msg->payload); // RC channel 3 value, in microseconds
	i += put_uint16_t_by_index(chan4_raw, i, msg->payload); // RC channel 4 value, in microseconds
	i += put_uint16_t_by_index(chan5_raw, i, msg->payload); // RC channel 5 value, in microseconds
	i += put_uint16_t_by_index(chan6_raw, i, msg->payload); // RC channel 6 value, in microseconds
	i += put_uint16_t_by_index(chan7_raw, i, msg->payload); // RC channel 7 value, in microseconds
	i += put_uint16_t_by_index(chan8_raw, i, msg->payload); // RC channel 8 value, in microseconds
	i += put_uint8_t_by_index(rssi, i, msg->payload); // Receive signal strength indicator, 0: 0%, 255: 100%

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a rc_channels_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_raw_t* rc_channels_raw)
{
	return mavlink_msg_rc_channels_raw_pack(system_id, component_id, msg, rc_channels_raw->chan1_raw, rc_channels_raw->chan2_raw, rc_channels_raw->chan3_raw, rc_channels_raw->chan4_raw, rc_channels_raw->chan5_raw, rc_channels_raw->chan6_raw, rc_channels_raw->chan7_raw, rc_channels_raw->chan8_raw, rc_channels_raw->rssi);
}

/**
 * @brief Send a rc_channels_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_raw_send(mavlink_channel_t chan, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)
{
	mavlink_message_t msg;
	mavlink_msg_rc_channels_raw_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, rssi);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE RC_CHANNELS_RAW UNPACKING

/**
 * @brief Get field chan1_raw from rc_channels_raw message
 *
 * @return RC channel 1 value, in microseconds
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan1_raw(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload)[0];
	r.b[0] = (msg->payload)[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field chan2_raw from rc_channels_raw message
 *
 * @return RC channel 2 value, in microseconds
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan2_raw(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field chan3_raw from rc_channels_raw message
 *
 * @return RC channel 3 value, in microseconds
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan3_raw(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field chan4_raw from rc_channels_raw message
 *
 * @return RC channel 4 value, in microseconds
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan4_raw(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field chan5_raw from rc_channels_raw message
 *
 * @return RC channel 5 value, in microseconds
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan5_raw(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field chan6_raw from rc_channels_raw message
 *
 * @return RC channel 6 value, in microseconds
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan6_raw(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field chan7_raw from rc_channels_raw message
 *
 * @return RC channel 7 value, in microseconds
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan7_raw(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field chan8_raw from rc_channels_raw message
 *
 * @return RC channel 8 value, in microseconds
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan8_raw(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field rssi from rc_channels_raw message
 *
 * @return Receive signal strength indicator, 0: 0%, 255: 100%
 */
static inline uint8_t mavlink_msg_rc_channels_raw_get_rssi(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
}

/**
 * @brief Decode a rc_channels_raw message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_raw_decode(const mavlink_message_t* msg, mavlink_rc_channels_raw_t* rc_channels_raw)
{
	rc_channels_raw->chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(msg);
	rc_channels_raw->chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(msg);
	rc_channels_raw->chan3_raw = mavlink_msg_rc_channels_raw_get_chan3_raw(msg);
	rc_channels_raw->chan4_raw = mavlink_msg_rc_channels_raw_get_chan4_raw(msg);
	rc_channels_raw->chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(msg);
	rc_channels_raw->chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(msg);
	rc_channels_raw->chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(msg);
	rc_channels_raw->chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(msg);
	rc_channels_raw->rssi = mavlink_msg_rc_channels_raw_get_rssi(msg);
}
