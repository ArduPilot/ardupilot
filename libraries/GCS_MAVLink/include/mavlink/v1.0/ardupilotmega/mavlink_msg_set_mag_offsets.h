// MESSAGE SET_MAG_OFFSETS PACKING

#define MAVLINK_MSG_ID_SET_MAG_OFFSETS 151

typedef struct __mavlink_set_mag_offsets_t
{
 int16_t mag_ofs_x; ///< magnetometer X offset
 int16_t mag_ofs_y; ///< magnetometer Y offset
 int16_t mag_ofs_z; ///< magnetometer Z offset
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_set_mag_offsets_t;

#define MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN 8
#define MAVLINK_MSG_ID_151_LEN 8

#define MAVLINK_MSG_ID_SET_MAG_OFFSETS_CRC 219
#define MAVLINK_MSG_ID_151_CRC 219



#define MAVLINK_MESSAGE_INFO_SET_MAG_OFFSETS { \
	"SET_MAG_OFFSETS", \
	5, \
	{  { "mag_ofs_x", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_set_mag_offsets_t, mag_ofs_x) }, \
         { "mag_ofs_y", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_set_mag_offsets_t, mag_ofs_y) }, \
         { "mag_ofs_z", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_set_mag_offsets_t, mag_ofs_z) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_set_mag_offsets_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_set_mag_offsets_t, target_component) }, \
         } \
}


/**
 * @brief Pack a set_mag_offsets message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mag_offsets_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN];
	_mav_put_int16_t(buf, 0, mag_ofs_x);
	_mav_put_int16_t(buf, 2, mag_ofs_y);
	_mav_put_int16_t(buf, 4, mag_ofs_z);
	_mav_put_uint8_t(buf, 6, target_system);
	_mav_put_uint8_t(buf, 7, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#else
	mavlink_set_mag_offsets_t packet;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_MAG_OFFSETS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN, MAVLINK_MSG_ID_SET_MAG_OFFSETS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#endif
}

/**
 * @brief Pack a set_mag_offsets message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mag_offsets_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,int16_t mag_ofs_x,int16_t mag_ofs_y,int16_t mag_ofs_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN];
	_mav_put_int16_t(buf, 0, mag_ofs_x);
	_mav_put_int16_t(buf, 2, mag_ofs_y);
	_mav_put_int16_t(buf, 4, mag_ofs_z);
	_mav_put_uint8_t(buf, 6, target_system);
	_mav_put_uint8_t(buf, 7, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#else
	mavlink_set_mag_offsets_t packet;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_MAG_OFFSETS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN, MAVLINK_MSG_ID_SET_MAG_OFFSETS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#endif
}

/**
 * @brief Encode a set_mag_offsets struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_mag_offsets C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_mag_offsets_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_mag_offsets_t* set_mag_offsets)
{
	return mavlink_msg_set_mag_offsets_pack(system_id, component_id, msg, set_mag_offsets->target_system, set_mag_offsets->target_component, set_mag_offsets->mag_ofs_x, set_mag_offsets->mag_ofs_y, set_mag_offsets->mag_ofs_z);
}

/**
 * @brief Encode a set_mag_offsets struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_mag_offsets C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_mag_offsets_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_mag_offsets_t* set_mag_offsets)
{
	return mavlink_msg_set_mag_offsets_pack_chan(system_id, component_id, chan, msg, set_mag_offsets->target_system, set_mag_offsets->target_component, set_mag_offsets->mag_ofs_x, set_mag_offsets->mag_ofs_y, set_mag_offsets->mag_ofs_z);
}

/**
 * @brief Send a set_mag_offsets message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_mag_offsets_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN];
	_mav_put_int16_t(buf, 0, mag_ofs_x);
	_mav_put_int16_t(buf, 2, mag_ofs_y);
	_mav_put_int16_t(buf, 4, mag_ofs_z);
	_mav_put_uint8_t(buf, 6, target_system);
	_mav_put_uint8_t(buf, 7, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS, buf, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN, MAVLINK_MSG_ID_SET_MAG_OFFSETS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS, buf, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#endif
#else
	mavlink_set_mag_offsets_t packet;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS, (const char *)&packet, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN, MAVLINK_MSG_ID_SET_MAG_OFFSETS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS, (const char *)&packet, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_mag_offsets_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int16_t(buf, 0, mag_ofs_x);
	_mav_put_int16_t(buf, 2, mag_ofs_y);
	_mav_put_int16_t(buf, 4, mag_ofs_z);
	_mav_put_uint8_t(buf, 6, target_system);
	_mav_put_uint8_t(buf, 7, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS, buf, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN, MAVLINK_MSG_ID_SET_MAG_OFFSETS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS, buf, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#endif
#else
	mavlink_set_mag_offsets_t *packet = (mavlink_set_mag_offsets_t *)msgbuf;
	packet->mag_ofs_x = mag_ofs_x;
	packet->mag_ofs_y = mag_ofs_y;
	packet->mag_ofs_z = mag_ofs_z;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS, (const char *)packet, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN, MAVLINK_MSG_ID_SET_MAG_OFFSETS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS, (const char *)packet, MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_MAG_OFFSETS UNPACKING


/**
 * @brief Get field target_system from set_mag_offsets message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_mag_offsets_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field target_component from set_mag_offsets message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_mag_offsets_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field mag_ofs_x from set_mag_offsets message
 *
 * @return magnetometer X offset
 */
static inline int16_t mavlink_msg_set_mag_offsets_get_mag_ofs_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field mag_ofs_y from set_mag_offsets message
 *
 * @return magnetometer Y offset
 */
static inline int16_t mavlink_msg_set_mag_offsets_get_mag_ofs_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field mag_ofs_z from set_mag_offsets message
 *
 * @return magnetometer Z offset
 */
static inline int16_t mavlink_msg_set_mag_offsets_get_mag_ofs_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Decode a set_mag_offsets message into a struct
 *
 * @param msg The message to decode
 * @param set_mag_offsets C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_mag_offsets_decode(const mavlink_message_t* msg, mavlink_set_mag_offsets_t* set_mag_offsets)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_mag_offsets->mag_ofs_x = mavlink_msg_set_mag_offsets_get_mag_ofs_x(msg);
	set_mag_offsets->mag_ofs_y = mavlink_msg_set_mag_offsets_get_mag_ofs_y(msg);
	set_mag_offsets->mag_ofs_z = mavlink_msg_set_mag_offsets_get_mag_ofs_z(msg);
	set_mag_offsets->target_system = mavlink_msg_set_mag_offsets_get_target_system(msg);
	set_mag_offsets->target_component = mavlink_msg_set_mag_offsets_get_target_component(msg);
#else
	memcpy(set_mag_offsets, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN);
#endif
}
