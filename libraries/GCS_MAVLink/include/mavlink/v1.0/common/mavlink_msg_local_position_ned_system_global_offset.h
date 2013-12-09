// MESSAGE LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET PACKING

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET 89

typedef struct __mavlink_local_position_ned_system_global_offset_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float x; ///< X Position
 float y; ///< Y Position
 float z; ///< Z Position
 float roll; ///< Roll
 float pitch; ///< Pitch
 float yaw; ///< Yaw
} mavlink_local_position_ned_system_global_offset_t;

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN 28
#define MAVLINK_MSG_ID_89_LEN 28

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_CRC 231
#define MAVLINK_MSG_ID_89_CRC 231



#define MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET { \
	"LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET", \
	7, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_local_position_ned_system_global_offset_t, time_boot_ms) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_local_position_ned_system_global_offset_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_local_position_ned_system_global_offset_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_local_position_ned_system_global_offset_t, z) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_local_position_ned_system_global_offset_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_local_position_ned_system_global_offset_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_local_position_ned_system_global_offset_t, yaw) }, \
         } \
}


/**
 * @brief Pack a local_position_ned_system_global_offset message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param roll Roll
 * @param pitch Pitch
 * @param yaw Yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_ned_system_global_offset_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_float(buf, 24, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN);
#else
	mavlink_local_position_ned_system_global_offset_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN);
#endif
}

/**
 * @brief Pack a local_position_ned_system_global_offset message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param roll Roll
 * @param pitch Pitch
 * @param yaw Yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_ned_system_global_offset_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float x,float y,float z,float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_float(buf, 24, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN);
#else
	mavlink_local_position_ned_system_global_offset_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN);
#endif
}

/**
 * @brief Encode a local_position_ned_system_global_offset struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_position_ned_system_global_offset C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_position_ned_system_global_offset_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_local_position_ned_system_global_offset_t* local_position_ned_system_global_offset)
{
	return mavlink_msg_local_position_ned_system_global_offset_pack(system_id, component_id, msg, local_position_ned_system_global_offset->time_boot_ms, local_position_ned_system_global_offset->x, local_position_ned_system_global_offset->y, local_position_ned_system_global_offset->z, local_position_ned_system_global_offset->roll, local_position_ned_system_global_offset->pitch, local_position_ned_system_global_offset->yaw);
}

/**
 * @brief Encode a local_position_ned_system_global_offset struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param local_position_ned_system_global_offset C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_position_ned_system_global_offset_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_local_position_ned_system_global_offset_t* local_position_ned_system_global_offset)
{
	return mavlink_msg_local_position_ned_system_global_offset_pack_chan(system_id, component_id, chan, msg, local_position_ned_system_global_offset->time_boot_ms, local_position_ned_system_global_offset->x, local_position_ned_system_global_offset->y, local_position_ned_system_global_offset->z, local_position_ned_system_global_offset->roll, local_position_ned_system_global_offset->pitch, local_position_ned_system_global_offset->yaw);
}

/**
 * @brief Send a local_position_ned_system_global_offset message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param roll Roll
 * @param pitch Pitch
 * @param yaw Yaw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_position_ned_system_global_offset_send(mavlink_channel_t chan, uint32_t time_boot_ms, float x, float y, float z, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, roll);
	_mav_put_float(buf, 20, pitch);
	_mav_put_float(buf, 24, yaw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN);
#endif
#else
	mavlink_local_position_ned_system_global_offset_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, (const char *)&packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, (const char *)&packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN);
#endif
#endif
}

#endif

// MESSAGE LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET UNPACKING


/**
 * @brief Get field time_boot_ms from local_position_ned_system_global_offset message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_local_position_ned_system_global_offset_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field x from local_position_ned_system_global_offset message
 *
 * @return X Position
 */
static inline float mavlink_msg_local_position_ned_system_global_offset_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from local_position_ned_system_global_offset message
 *
 * @return Y Position
 */
static inline float mavlink_msg_local_position_ned_system_global_offset_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from local_position_ned_system_global_offset message
 *
 * @return Z Position
 */
static inline float mavlink_msg_local_position_ned_system_global_offset_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll from local_position_ned_system_global_offset message
 *
 * @return Roll
 */
static inline float mavlink_msg_local_position_ned_system_global_offset_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitch from local_position_ned_system_global_offset message
 *
 * @return Pitch
 */
static inline float mavlink_msg_local_position_ned_system_global_offset_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yaw from local_position_ned_system_global_offset message
 *
 * @return Yaw
 */
static inline float mavlink_msg_local_position_ned_system_global_offset_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a local_position_ned_system_global_offset message into a struct
 *
 * @param msg The message to decode
 * @param local_position_ned_system_global_offset C-struct to decode the message contents into
 */
static inline void mavlink_msg_local_position_ned_system_global_offset_decode(const mavlink_message_t* msg, mavlink_local_position_ned_system_global_offset_t* local_position_ned_system_global_offset)
{
#if MAVLINK_NEED_BYTE_SWAP
	local_position_ned_system_global_offset->time_boot_ms = mavlink_msg_local_position_ned_system_global_offset_get_time_boot_ms(msg);
	local_position_ned_system_global_offset->x = mavlink_msg_local_position_ned_system_global_offset_get_x(msg);
	local_position_ned_system_global_offset->y = mavlink_msg_local_position_ned_system_global_offset_get_y(msg);
	local_position_ned_system_global_offset->z = mavlink_msg_local_position_ned_system_global_offset_get_z(msg);
	local_position_ned_system_global_offset->roll = mavlink_msg_local_position_ned_system_global_offset_get_roll(msg);
	local_position_ned_system_global_offset->pitch = mavlink_msg_local_position_ned_system_global_offset_get_pitch(msg);
	local_position_ned_system_global_offset->yaw = mavlink_msg_local_position_ned_system_global_offset_get_yaw(msg);
#else
	memcpy(local_position_ned_system_global_offset, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET_LEN);
#endif
}
