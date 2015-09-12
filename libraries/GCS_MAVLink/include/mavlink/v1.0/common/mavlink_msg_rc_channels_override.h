// MESSAGE RC_CHANNELS_OVERRIDE PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE 70

typedef struct __mavlink_rc_channels_override_t
{
 uint16_t chan1_raw; /*< RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan2_raw; /*< RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan3_raw; /*< RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan4_raw; /*< RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan5_raw; /*< RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan6_raw; /*< RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan7_raw; /*< RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint16_t chan8_raw; /*< RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
} mavlink_rc_channels_override_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN 18
#define MAVLINK_MSG_ID_70_LEN 18

#define MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC 124
#define MAVLINK_MSG_ID_70_CRC 124



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_OVERRIDE { \
	"RC_CHANNELS_OVERRIDE", \
	10, \
	{  { "chan1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_rc_channels_override_t, chan1_raw) }, \
         { "chan2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_rc_channels_override_t, chan2_raw) }, \
         { "chan3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_rc_channels_override_t, chan3_raw) }, \
         { "chan4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_rc_channels_override_t, chan4_raw) }, \
         { "chan5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_rc_channels_override_t, chan5_raw) }, \
         { "chan6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_rc_channels_override_t, chan6_raw) }, \
         { "chan7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_rc_channels_override_t, chan7_raw) }, \
         { "chan8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_rc_channels_override_t, chan8_raw) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_rc_channels_override_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_rc_channels_override_t, target_component) }, \
         } \
}


/**
 * @brief Pack a rc_channels_override message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_override_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN];
	_mav_put_uint16_t(buf, 0, chan1_raw);
	_mav_put_uint16_t(buf, 2, chan2_raw);
	_mav_put_uint16_t(buf, 4, chan3_raw);
	_mav_put_uint16_t(buf, 6, chan4_raw);
	_mav_put_uint16_t(buf, 8, chan5_raw);
	_mav_put_uint16_t(buf, 10, chan6_raw);
	_mav_put_uint16_t(buf, 12, chan7_raw);
	_mav_put_uint16_t(buf, 14, chan8_raw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#else
	mavlink_rc_channels_override_t packet;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif
}

/**
 * @brief Pack a rc_channels_override message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_override_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint16_t chan1_raw,uint16_t chan2_raw,uint16_t chan3_raw,uint16_t chan4_raw,uint16_t chan5_raw,uint16_t chan6_raw,uint16_t chan7_raw,uint16_t chan8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN];
	_mav_put_uint16_t(buf, 0, chan1_raw);
	_mav_put_uint16_t(buf, 2, chan2_raw);
	_mav_put_uint16_t(buf, 4, chan3_raw);
	_mav_put_uint16_t(buf, 6, chan4_raw);
	_mav_put_uint16_t(buf, 8, chan5_raw);
	_mav_put_uint16_t(buf, 10, chan6_raw);
	_mav_put_uint16_t(buf, 12, chan7_raw);
	_mav_put_uint16_t(buf, 14, chan8_raw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#else
	mavlink_rc_channels_override_t packet;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif
}

/**
 * @brief Encode a rc_channels_override struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_override C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_override_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_override_t* rc_channels_override)
{
	return mavlink_msg_rc_channels_override_pack(system_id, component_id, msg, rc_channels_override->target_system, rc_channels_override->target_component, rc_channels_override->chan1_raw, rc_channels_override->chan2_raw, rc_channels_override->chan3_raw, rc_channels_override->chan4_raw, rc_channels_override->chan5_raw, rc_channels_override->chan6_raw, rc_channels_override->chan7_raw, rc_channels_override->chan8_raw);
}

/**
 * @brief Encode a rc_channels_override struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_override C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_override_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rc_channels_override_t* rc_channels_override)
{
	return mavlink_msg_rc_channels_override_pack_chan(system_id, component_id, chan, msg, rc_channels_override->target_system, rc_channels_override->target_component, rc_channels_override->chan1_raw, rc_channels_override->chan2_raw, rc_channels_override->chan3_raw, rc_channels_override->chan4_raw, rc_channels_override->chan5_raw, rc_channels_override->chan6_raw, rc_channels_override->chan7_raw, rc_channels_override->chan8_raw);
}

/**
 * @brief Send a rc_channels_override message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_override_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN];
	_mav_put_uint16_t(buf, 0, chan1_raw);
	_mav_put_uint16_t(buf, 2, chan2_raw);
	_mav_put_uint16_t(buf, 4, chan3_raw);
	_mav_put_uint16_t(buf, 6, chan4_raw);
	_mav_put_uint16_t(buf, 8, chan5_raw);
	_mav_put_uint16_t(buf, 10, chan6_raw);
	_mav_put_uint16_t(buf, 12, chan7_raw);
	_mav_put_uint16_t(buf, 14, chan8_raw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif
#else
	mavlink_rc_channels_override_t packet;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, (const char *)&packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, (const char *)&packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rc_channels_override_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, chan1_raw);
	_mav_put_uint16_t(buf, 2, chan2_raw);
	_mav_put_uint16_t(buf, 4, chan3_raw);
	_mav_put_uint16_t(buf, 6, chan4_raw);
	_mav_put_uint16_t(buf, 8, chan5_raw);
	_mav_put_uint16_t(buf, 10, chan6_raw);
	_mav_put_uint16_t(buf, 12, chan7_raw);
	_mav_put_uint16_t(buf, 14, chan8_raw);
	_mav_put_uint8_t(buf, 16, target_system);
	_mav_put_uint8_t(buf, 17, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, buf, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif
#else
	mavlink_rc_channels_override_t *packet = (mavlink_rc_channels_override_t *)msgbuf;
	packet->chan1_raw = chan1_raw;
	packet->chan2_raw = chan2_raw;
	packet->chan3_raw = chan3_raw;
	packet->chan4_raw = chan4_raw;
	packet->chan5_raw = chan5_raw;
	packet->chan6_raw = chan6_raw;
	packet->chan7_raw = chan7_raw;
	packet->chan8_raw = chan8_raw;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, (const char *)packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, (const char *)packet, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RC_CHANNELS_OVERRIDE UNPACKING


/**
 * @brief Get field target_system from rc_channels_override message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_rc_channels_override_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from rc_channels_override message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_rc_channels_override_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field chan1_raw from rc_channels_override message
 *
 * @return RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan1_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field chan2_raw from rc_channels_override message
 *
 * @return RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan2_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field chan3_raw from rc_channels_override message
 *
 * @return RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan3_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field chan4_raw from rc_channels_override message
 *
 * @return RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan4_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field chan5_raw from rc_channels_override message
 *
 * @return RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan5_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field chan6_raw from rc_channels_override message
 *
 * @return RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan6_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field chan7_raw from rc_channels_override message
 *
 * @return RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan7_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field chan8_raw from rc_channels_override message
 *
 * @return RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
 */
static inline uint16_t mavlink_msg_rc_channels_override_get_chan8_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Decode a rc_channels_override message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_override C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_override_decode(const mavlink_message_t* msg, mavlink_rc_channels_override_t* rc_channels_override)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_channels_override->chan1_raw = mavlink_msg_rc_channels_override_get_chan1_raw(msg);
	rc_channels_override->chan2_raw = mavlink_msg_rc_channels_override_get_chan2_raw(msg);
	rc_channels_override->chan3_raw = mavlink_msg_rc_channels_override_get_chan3_raw(msg);
	rc_channels_override->chan4_raw = mavlink_msg_rc_channels_override_get_chan4_raw(msg);
	rc_channels_override->chan5_raw = mavlink_msg_rc_channels_override_get_chan5_raw(msg);
	rc_channels_override->chan6_raw = mavlink_msg_rc_channels_override_get_chan6_raw(msg);
	rc_channels_override->chan7_raw = mavlink_msg_rc_channels_override_get_chan7_raw(msg);
	rc_channels_override->chan8_raw = mavlink_msg_rc_channels_override_get_chan8_raw(msg);
	rc_channels_override->target_system = mavlink_msg_rc_channels_override_get_target_system(msg);
	rc_channels_override->target_component = mavlink_msg_rc_channels_override_get_target_component(msg);
#else
	memcpy(rc_channels_override, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN);
#endif
}
