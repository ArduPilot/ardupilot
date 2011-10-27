// MESSAGE LOCAL_POSITION PACKING

#define MAVLINK_MSG_ID_LOCAL_POSITION 32

typedef struct __mavlink_local_position_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float x; ///< X Position
 float y; ///< Y Position
 float z; ///< Z Position
 float vx; ///< X Speed
 float vy; ///< Y Speed
 float vz; ///< Z Speed
} mavlink_local_position_t;

#define MAVLINK_MSG_ID_LOCAL_POSITION_LEN 28
#define MAVLINK_MSG_ID_32_LEN 28



#define MAVLINK_MESSAGE_INFO_LOCAL_POSITION { \
	"LOCAL_POSITION", \
	7, \
<<<<<<< HEAD
	{  { "time_boot_ms", MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_local_position_t, time_boot_ms) }, \
         { "x", MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_local_position_t, x) }, \
         { "y", MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_local_position_t, y) }, \
         { "z", MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_local_position_t, z) }, \
         { "vx", MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_local_position_t, vx) }, \
         { "vy", MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_local_position_t, vy) }, \
         { "vz", MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_local_position_t, vz) }, \
=======
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_local_position_t, usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_local_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_local_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_local_position_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_local_position_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_local_position_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_local_position_t, vz) }, \
>>>>>>> 077b901fe072bb5fefdaca3680b3341c329395cb
         } \
}


/**
 * @brief Pack a local_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);

        memcpy(_MAV_PAYLOAD(msg), buf, 28);
#else
	mavlink_local_position_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION;
	return mavlink_finalize_message(msg, system_id, component_id, 28, 231);
}

/**
 * @brief Pack a local_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float x,float y,float z,float vx,float vy,float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);

        memcpy(_MAV_PAYLOAD(msg), buf, 28);
#else
	mavlink_local_position_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 28, 231);
}

/**
 * @brief Encode a local_position struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_local_position_t* local_position)
{
	return mavlink_msg_local_position_pack(system_id, component_id, msg, local_position->time_boot_ms, local_position->x, local_position->y, local_position->z, local_position->vx, local_position->vy, local_position->vz);
}

/**
 * @brief Send a local_position message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_position_send(mavlink_channel_t chan, uint32_t time_boot_ms, float x, float y, float z, float vx, float vy, float vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION, buf, 28, 231);
#else
	mavlink_local_position_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION, (const char *)&packet, 28, 231);
#endif
}

#endif

// MESSAGE LOCAL_POSITION UNPACKING


/**
 * @brief Get field time_boot_ms from local_position message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_local_position_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field x from local_position message
 *
 * @return X Position
 */
static inline float mavlink_msg_local_position_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from local_position message
 *
 * @return Y Position
 */
static inline float mavlink_msg_local_position_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from local_position message
 *
 * @return Z Position
 */
static inline float mavlink_msg_local_position_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vx from local_position message
 *
 * @return X Speed
 */
static inline float mavlink_msg_local_position_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vy from local_position message
 *
 * @return Y Speed
 */
static inline float mavlink_msg_local_position_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vz from local_position message
 *
 * @return Z Speed
 */
static inline float mavlink_msg_local_position_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a local_position message into a struct
 *
 * @param msg The message to decode
 * @param local_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_local_position_decode(const mavlink_message_t* msg, mavlink_local_position_t* local_position)
{
#if MAVLINK_NEED_BYTE_SWAP
	local_position->time_boot_ms = mavlink_msg_local_position_get_time_boot_ms(msg);
	local_position->x = mavlink_msg_local_position_get_x(msg);
	local_position->y = mavlink_msg_local_position_get_y(msg);
	local_position->z = mavlink_msg_local_position_get_z(msg);
	local_position->vx = mavlink_msg_local_position_get_vx(msg);
	local_position->vy = mavlink_msg_local_position_get_vy(msg);
	local_position->vz = mavlink_msg_local_position_get_vz(msg);
#else
	memcpy(local_position, _MAV_PAYLOAD(msg), 28);
#endif
}
