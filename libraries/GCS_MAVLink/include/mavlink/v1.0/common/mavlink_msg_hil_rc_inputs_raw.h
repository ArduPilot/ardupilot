// MESSAGE HIL_RC_INPUTS_RAW PACKING

#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW 92

typedef struct __mavlink_hil_rc_inputs_raw_t
{
 uint64_t time_usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 uint16_t chan1_raw; ///< RC channel 1 value, in microseconds
 uint16_t chan2_raw; ///< RC channel 2 value, in microseconds
 uint16_t chan3_raw; ///< RC channel 3 value, in microseconds
 uint16_t chan4_raw; ///< RC channel 4 value, in microseconds
 uint16_t chan5_raw; ///< RC channel 5 value, in microseconds
 uint16_t chan6_raw; ///< RC channel 6 value, in microseconds
 uint16_t chan7_raw; ///< RC channel 7 value, in microseconds
 uint16_t chan8_raw; ///< RC channel 8 value, in microseconds
 uint16_t chan9_raw; ///< RC channel 9 value, in microseconds
 uint16_t chan10_raw; ///< RC channel 10 value, in microseconds
 uint16_t chan11_raw; ///< RC channel 11 value, in microseconds
 uint16_t chan12_raw; ///< RC channel 12 value, in microseconds
 uint8_t rssi; ///< Receive signal strength indicator, 0: 0%, 255: 100%
} mavlink_hil_rc_inputs_raw_t;

#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN 33
#define MAVLINK_MSG_ID_92_LEN 33

#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_CRC 54
#define MAVLINK_MSG_ID_92_CRC 54



#define MAVLINK_MESSAGE_INFO_HIL_RC_INPUTS_RAW { \
	"HIL_RC_INPUTS_RAW", \
	14, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_rc_inputs_raw_t, time_usec) }, \
         { "chan1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_hil_rc_inputs_raw_t, chan1_raw) }, \
         { "chan2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_hil_rc_inputs_raw_t, chan2_raw) }, \
         { "chan3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_hil_rc_inputs_raw_t, chan3_raw) }, \
         { "chan4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_hil_rc_inputs_raw_t, chan4_raw) }, \
         { "chan5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_hil_rc_inputs_raw_t, chan5_raw) }, \
         { "chan6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_hil_rc_inputs_raw_t, chan6_raw) }, \
         { "chan7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_hil_rc_inputs_raw_t, chan7_raw) }, \
         { "chan8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_hil_rc_inputs_raw_t, chan8_raw) }, \
         { "chan9_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_hil_rc_inputs_raw_t, chan9_raw) }, \
         { "chan10_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_hil_rc_inputs_raw_t, chan10_raw) }, \
         { "chan11_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_hil_rc_inputs_raw_t, chan11_raw) }, \
         { "chan12_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_hil_rc_inputs_raw_t, chan12_raw) }, \
         { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_hil_rc_inputs_raw_t, rssi) }, \
         } \
}


/**
 * @brief Pack a hil_rc_inputs_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param chan9_raw RC channel 9 value, in microseconds
 * @param chan10_raw RC channel 10 value, in microseconds
 * @param chan11_raw RC channel 11 value, in microseconds
 * @param chan12_raw RC channel 12 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint16_t(buf, 8, chan1_raw);
	_mav_put_uint16_t(buf, 10, chan2_raw);
	_mav_put_uint16_t(buf, 12, chan3_raw);
	_mav_put_uint16_t(buf, 14, chan4_raw);
	_mav_put_uint16_t(buf, 16, chan5_raw);
	_mav_put_uint16_t(buf, 18, chan6_raw);
	_mav_put_uint16_t(buf, 20, chan7_raw);
	_mav_put_uint16_t(buf, 22, chan8_raw);
	_mav_put_uint16_t(buf, 24, chan9_raw);
	_mav_put_uint16_t(buf, 26, chan10_raw);
	_mav_put_uint16_t(buf, 28, chan11_raw);
	_mav_put_uint16_t(buf, 30, chan12_raw);
	_mav_put_uint8_t(buf, 32, rssi);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN);
#else
	mavlink_hil_rc_inputs_raw_t packet;
	packet.time_usec = time_usec;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.chan9_raw = chan9_raw;
	packet.chan10_raw = chan10_raw;
	packet.chan11_raw = chan11_raw;
	packet.chan12_raw = chan12_raw;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN);
#endif
}

/**
 * @brief Pack a hil_rc_inputs_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param chan9_raw RC channel 9 value, in microseconds
 * @param chan10_raw RC channel 10 value, in microseconds
 * @param chan11_raw RC channel 11 value, in microseconds
 * @param chan12_raw RC channel 12 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint16_t chan1_raw,uint16_t chan2_raw,uint16_t chan3_raw,uint16_t chan4_raw,uint16_t chan5_raw,uint16_t chan6_raw,uint16_t chan7_raw,uint16_t chan8_raw,uint16_t chan9_raw,uint16_t chan10_raw,uint16_t chan11_raw,uint16_t chan12_raw,uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint16_t(buf, 8, chan1_raw);
	_mav_put_uint16_t(buf, 10, chan2_raw);
	_mav_put_uint16_t(buf, 12, chan3_raw);
	_mav_put_uint16_t(buf, 14, chan4_raw);
	_mav_put_uint16_t(buf, 16, chan5_raw);
	_mav_put_uint16_t(buf, 18, chan6_raw);
	_mav_put_uint16_t(buf, 20, chan7_raw);
	_mav_put_uint16_t(buf, 22, chan8_raw);
	_mav_put_uint16_t(buf, 24, chan9_raw);
	_mav_put_uint16_t(buf, 26, chan10_raw);
	_mav_put_uint16_t(buf, 28, chan11_raw);
	_mav_put_uint16_t(buf, 30, chan12_raw);
	_mav_put_uint8_t(buf, 32, rssi);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN);
#else
	mavlink_hil_rc_inputs_raw_t packet;
	packet.time_usec = time_usec;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.chan9_raw = chan9_raw;
	packet.chan10_raw = chan10_raw;
	packet.chan11_raw = chan11_raw;
	packet.chan12_raw = chan12_raw;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN);
#endif
}

/**
 * @brief Encode a hil_rc_inputs_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_rc_inputs_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_rc_inputs_raw_t* hil_rc_inputs_raw)
{
	return mavlink_msg_hil_rc_inputs_raw_pack(system_id, component_id, msg, hil_rc_inputs_raw->time_usec, hil_rc_inputs_raw->chan1_raw, hil_rc_inputs_raw->chan2_raw, hil_rc_inputs_raw->chan3_raw, hil_rc_inputs_raw->chan4_raw, hil_rc_inputs_raw->chan5_raw, hil_rc_inputs_raw->chan6_raw, hil_rc_inputs_raw->chan7_raw, hil_rc_inputs_raw->chan8_raw, hil_rc_inputs_raw->chan9_raw, hil_rc_inputs_raw->chan10_raw, hil_rc_inputs_raw->chan11_raw, hil_rc_inputs_raw->chan12_raw, hil_rc_inputs_raw->rssi);
}

/**
 * @brief Send a hil_rc_inputs_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param chan9_raw RC channel 9 value, in microseconds
 * @param chan10_raw RC channel 10 value, in microseconds
 * @param chan11_raw RC channel 11 value, in microseconds
 * @param chan12_raw RC channel 12 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_rc_inputs_raw_send(mavlink_channel_t chan, uint64_t time_usec, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint16_t(buf, 8, chan1_raw);
	_mav_put_uint16_t(buf, 10, chan2_raw);
	_mav_put_uint16_t(buf, 12, chan3_raw);
	_mav_put_uint16_t(buf, 14, chan4_raw);
	_mav_put_uint16_t(buf, 16, chan5_raw);
	_mav_put_uint16_t(buf, 18, chan6_raw);
	_mav_put_uint16_t(buf, 20, chan7_raw);
	_mav_put_uint16_t(buf, 22, chan8_raw);
	_mav_put_uint16_t(buf, 24, chan9_raw);
	_mav_put_uint16_t(buf, 26, chan10_raw);
	_mav_put_uint16_t(buf, 28, chan11_raw);
	_mav_put_uint16_t(buf, 30, chan12_raw);
	_mav_put_uint8_t(buf, 32, rssi);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW, buf, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW, buf, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN);
#endif
#else
	mavlink_hil_rc_inputs_raw_t packet;
	packet.time_usec = time_usec;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.chan9_raw = chan9_raw;
	packet.chan10_raw = chan10_raw;
	packet.chan11_raw = chan11_raw;
	packet.chan12_raw = chan12_raw;
	packet.rssi = rssi;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW, (const char *)&packet, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW, (const char *)&packet, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN);
#endif
#endif
}

#endif

// MESSAGE HIL_RC_INPUTS_RAW UNPACKING


/**
 * @brief Get field time_usec from hil_rc_inputs_raw message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_hil_rc_inputs_raw_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field chan1_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 1 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan1_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field chan2_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 2 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan2_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field chan3_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 3 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan3_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field chan4_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 4 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan4_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field chan5_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 5 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan5_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field chan6_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 6 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan6_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field chan7_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 7 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan7_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field chan8_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 8 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan8_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field chan9_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 9 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan9_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field chan10_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 10 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan10_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field chan11_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 11 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan11_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field chan12_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 12 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan12_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field rssi from hil_rc_inputs_raw message
 *
 * @return Receive signal strength indicator, 0: 0%, 255: 100%
 */
static inline uint8_t mavlink_msg_hil_rc_inputs_raw_get_rssi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Decode a hil_rc_inputs_raw message into a struct
 *
 * @param msg The message to decode
 * @param hil_rc_inputs_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_rc_inputs_raw_decode(const mavlink_message_t* msg, mavlink_hil_rc_inputs_raw_t* hil_rc_inputs_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	hil_rc_inputs_raw->time_usec = mavlink_msg_hil_rc_inputs_raw_get_time_usec(msg);
	hil_rc_inputs_raw->chan1_raw = mavlink_msg_hil_rc_inputs_raw_get_chan1_raw(msg);
	hil_rc_inputs_raw->chan2_raw = mavlink_msg_hil_rc_inputs_raw_get_chan2_raw(msg);
	hil_rc_inputs_raw->chan3_raw = mavlink_msg_hil_rc_inputs_raw_get_chan3_raw(msg);
	hil_rc_inputs_raw->chan4_raw = mavlink_msg_hil_rc_inputs_raw_get_chan4_raw(msg);
	hil_rc_inputs_raw->chan5_raw = mavlink_msg_hil_rc_inputs_raw_get_chan5_raw(msg);
	hil_rc_inputs_raw->chan6_raw = mavlink_msg_hil_rc_inputs_raw_get_chan6_raw(msg);
	hil_rc_inputs_raw->chan7_raw = mavlink_msg_hil_rc_inputs_raw_get_chan7_raw(msg);
	hil_rc_inputs_raw->chan8_raw = mavlink_msg_hil_rc_inputs_raw_get_chan8_raw(msg);
	hil_rc_inputs_raw->chan9_raw = mavlink_msg_hil_rc_inputs_raw_get_chan9_raw(msg);
	hil_rc_inputs_raw->chan10_raw = mavlink_msg_hil_rc_inputs_raw_get_chan10_raw(msg);
	hil_rc_inputs_raw->chan11_raw = mavlink_msg_hil_rc_inputs_raw_get_chan11_raw(msg);
	hil_rc_inputs_raw->chan12_raw = mavlink_msg_hil_rc_inputs_raw_get_chan12_raw(msg);
	hil_rc_inputs_raw->rssi = mavlink_msg_hil_rc_inputs_raw_get_rssi(msg);
#else
	memcpy(hil_rc_inputs_raw, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW_LEN);
#endif
}
