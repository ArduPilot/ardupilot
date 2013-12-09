// MESSAGE RC_CHANNELS_RAW PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35

typedef struct __mavlink_rc_channels_raw_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 uint16_t chan1_raw; ///< RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 uint16_t chan2_raw; ///< RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 uint16_t chan3_raw; ///< RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 uint16_t chan4_raw; ///< RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 uint16_t chan5_raw; ///< RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 uint16_t chan6_raw; ///< RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 uint16_t chan7_raw; ///< RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 uint16_t chan8_raw; ///< RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 uint8_t port; ///< Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
 uint8_t rssi; ///< Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
} mavlink_rc_channels_raw_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN 22
#define MAVLINK_MSG_ID_35_LEN 22

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_CRC 244
#define MAVLINK_MSG_ID_35_CRC 244



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_RAW { \
	"RC_CHANNELS_RAW", \
	11, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_rc_channels_raw_t, time_boot_ms) }, \
         { "chan1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_rc_channels_raw_t, chan1_raw) }, \
         { "chan2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_rc_channels_raw_t, chan2_raw) }, \
         { "chan3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_rc_channels_raw_t, chan3_raw) }, \
         { "chan4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_rc_channels_raw_t, chan4_raw) }, \
         { "chan5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_rc_channels_raw_t, chan5_raw) }, \
         { "chan6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_rc_channels_raw_t, chan6_raw) }, \
         { "chan7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_rc_channels_raw_t, chan7_raw) }, \
         { "chan8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_rc_channels_raw_t, chan8_raw) }, \
         { "port", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_rc_channels_raw_t, port) }, \
         { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_rc_channels_raw_t, rssi) }, \
         } \
}


/**
 * @brief Pack a rc_channels_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param rssi Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint16_t(buf, 4, chan1_raw);
	_mav_put_uint16_t(buf, 6, chan2_raw);
	_mav_put_uint16_t(buf, 8, chan3_raw);
	_mav_put_uint16_t(buf, 10, chan4_raw);
	_mav_put_uint16_t(buf, 12, chan5_raw);
	_mav_put_uint16_t(buf, 14, chan6_raw);
	_mav_put_uint16_t(buf, 16, chan7_raw);
	_mav_put_uint16_t(buf, 18, chan8_raw);
	_mav_put_uint8_t(buf, 20, port);
	_mav_put_uint8_t(buf, 21, rssi);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN);
#else
	mavlink_rc_channels_raw_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.port = port;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN, MAVLINK_MSG_ID_RC_CHANNELS_RAW_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN);
#endif
}

/**
 * @brief Pack a rc_channels_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param rssi Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t port,uint16_t chan1_raw,uint16_t chan2_raw,uint16_t chan3_raw,uint16_t chan4_raw,uint16_t chan5_raw,uint16_t chan6_raw,uint16_t chan7_raw,uint16_t chan8_raw,uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint16_t(buf, 4, chan1_raw);
	_mav_put_uint16_t(buf, 6, chan2_raw);
	_mav_put_uint16_t(buf, 8, chan3_raw);
	_mav_put_uint16_t(buf, 10, chan4_raw);
	_mav_put_uint16_t(buf, 12, chan5_raw);
	_mav_put_uint16_t(buf, 14, chan6_raw);
	_mav_put_uint16_t(buf, 16, chan7_raw);
	_mav_put_uint16_t(buf, 18, chan8_raw);
	_mav_put_uint8_t(buf, 20, port);
	_mav_put_uint8_t(buf, 21, rssi);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN);
#else
	mavlink_rc_channels_raw_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.port = port;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN, MAVLINK_MSG_ID_RC_CHANNELS_RAW_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN);
#endif
}

/**
 * @brief Encode a rc_channels_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_raw_t* rc_channels_raw)
{
	return mavlink_msg_rc_channels_raw_pack(system_id, component_id, msg, rc_channels_raw->time_boot_ms, rc_channels_raw->port, rc_channels_raw->chan1_raw, rc_channels_raw->chan2_raw, rc_channels_raw->chan3_raw, rc_channels_raw->chan4_raw, rc_channels_raw->chan5_raw, rc_channels_raw->chan6_raw, rc_channels_raw->chan7_raw, rc_channels_raw->chan8_raw, rc_channels_raw->rssi);
}

/**
 * @brief Encode a rc_channels_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rc_channels_raw_t* rc_channels_raw)
{
	return mavlink_msg_rc_channels_raw_pack_chan(system_id, component_id, chan, msg, rc_channels_raw->time_boot_ms, rc_channels_raw->port, rc_channels_raw->chan1_raw, rc_channels_raw->chan2_raw, rc_channels_raw->chan3_raw, rc_channels_raw->chan4_raw, rc_channels_raw->chan5_raw, rc_channels_raw->chan6_raw, rc_channels_raw->chan7_raw, rc_channels_raw->chan8_raw, rc_channels_raw->rssi);
}

/**
 * @brief Send a rc_channels_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param rssi Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_raw_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t port, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint16_t(buf, 4, chan1_raw);
	_mav_put_uint16_t(buf, 6, chan2_raw);
	_mav_put_uint16_t(buf, 8, chan3_raw);
	_mav_put_uint16_t(buf, 10, chan4_raw);
	_mav_put_uint16_t(buf, 12, chan5_raw);
	_mav_put_uint16_t(buf, 14, chan6_raw);
	_mav_put_uint16_t(buf, 16, chan7_raw);
	_mav_put_uint16_t(buf, 18, chan8_raw);
	_mav_put_uint8_t(buf, 20, port);
	_mav_put_uint8_t(buf, 21, rssi);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_RAW, buf, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN, MAVLINK_MSG_ID_RC_CHANNELS_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_RAW, buf, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN);
#endif
#else
	mavlink_rc_channels_raw_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.port = port;
	packet.rssi = rssi;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_RAW, (const char *)&packet, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN, MAVLINK_MSG_ID_RC_CHANNELS_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_RAW, (const char *)&packet, MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN);
#endif
#endif
}

#endif

// MESSAGE RC_CHANNELS_RAW UNPACKING


/**
 * @brief Get field time_boot_ms from rc_channels_raw message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_rc_channels_raw_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field port from rc_channels_raw message
 *
 * @return Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
 */
static inline uint8_t mavlink_msg_rc_channels_raw_get_port(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field chan1_raw from rc_channels_raw message
 *
 * @return RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan1_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field chan2_raw from rc_channels_raw message
 *
 * @return RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan2_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field chan3_raw from rc_channels_raw message
 *
 * @return RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan3_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field chan4_raw from rc_channels_raw message
 *
 * @return RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan4_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field chan5_raw from rc_channels_raw message
 *
 * @return RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan5_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field chan6_raw from rc_channels_raw message
 *
 * @return RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan6_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field chan7_raw from rc_channels_raw message
 *
 * @return RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan7_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field chan8_raw from rc_channels_raw message
 *
 * @return RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_raw_get_chan8_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field rssi from rc_channels_raw message
 *
 * @return Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 */
static inline uint8_t mavlink_msg_rc_channels_raw_get_rssi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Decode a rc_channels_raw message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_raw_decode(const mavlink_message_t* msg, mavlink_rc_channels_raw_t* rc_channels_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_channels_raw->time_boot_ms = mavlink_msg_rc_channels_raw_get_time_boot_ms(msg);
	rc_channels_raw->chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(msg);
	rc_channels_raw->chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(msg);
	rc_channels_raw->chan3_raw = mavlink_msg_rc_channels_raw_get_chan3_raw(msg);
	rc_channels_raw->chan4_raw = mavlink_msg_rc_channels_raw_get_chan4_raw(msg);
	rc_channels_raw->chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(msg);
	rc_channels_raw->chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(msg);
	rc_channels_raw->chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(msg);
	rc_channels_raw->chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(msg);
	rc_channels_raw->port = mavlink_msg_rc_channels_raw_get_port(msg);
	rc_channels_raw->rssi = mavlink_msg_rc_channels_raw_get_rssi(msg);
#else
	memcpy(rc_channels_raw, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN);
#endif
}
