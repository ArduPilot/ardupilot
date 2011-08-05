// MESSAGE IMAGE_TRIGGERED PACKING

#define MAVLINK_MSG_ID_IMAGE_TRIGGERED 101

typedef struct __mavlink_image_triggered_t 
{
	uint64_t timestamp; ///< Timestamp
	uint32_t seq; ///< IMU seq
	float roll; ///< Roll angle in rad
	float pitch; ///< Pitch angle in rad
	float yaw; ///< Yaw angle in rad
	float local_z; ///< Local frame Z coordinate (height over ground)
	float lat; ///< GPS X coordinate
	float lon; ///< GPS Y coordinate
	float alt; ///< Global frame altitude
	float ground_x; ///< Ground truth X
	float ground_y; ///< Ground truth Y
	float ground_z; ///< Ground truth Z

} mavlink_image_triggered_t;



/**
 * @brief Pack a image_triggered message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp
 * @param seq IMU seq
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @param local_z Local frame Z coordinate (height over ground)
 * @param lat GPS X coordinate
 * @param lon GPS Y coordinate
 * @param alt Global frame altitude
 * @param ground_x Ground truth X
 * @param ground_y Ground truth Y
 * @param ground_z Ground truth Z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_triggered_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t timestamp, uint32_t seq, float roll, float pitch, float yaw, float local_z, float lat, float lon, float alt, float ground_x, float ground_y, float ground_z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGERED;

	i += put_uint64_t_by_index(timestamp, i, msg->payload); // Timestamp
	i += put_uint32_t_by_index(seq, i, msg->payload); // IMU seq
	i += put_float_by_index(roll, i, msg->payload); // Roll angle in rad
	i += put_float_by_index(pitch, i, msg->payload); // Pitch angle in rad
	i += put_float_by_index(yaw, i, msg->payload); // Yaw angle in rad
	i += put_float_by_index(local_z, i, msg->payload); // Local frame Z coordinate (height over ground)
	i += put_float_by_index(lat, i, msg->payload); // GPS X coordinate
	i += put_float_by_index(lon, i, msg->payload); // GPS Y coordinate
	i += put_float_by_index(alt, i, msg->payload); // Global frame altitude
	i += put_float_by_index(ground_x, i, msg->payload); // Ground truth X
	i += put_float_by_index(ground_y, i, msg->payload); // Ground truth Y
	i += put_float_by_index(ground_z, i, msg->payload); // Ground truth Z

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a image_triggered message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp
 * @param seq IMU seq
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @param local_z Local frame Z coordinate (height over ground)
 * @param lat GPS X coordinate
 * @param lon GPS Y coordinate
 * @param alt Global frame altitude
 * @param ground_x Ground truth X
 * @param ground_y Ground truth Y
 * @param ground_z Ground truth Z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_triggered_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t timestamp, uint32_t seq, float roll, float pitch, float yaw, float local_z, float lat, float lon, float alt, float ground_x, float ground_y, float ground_z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGERED;

	i += put_uint64_t_by_index(timestamp, i, msg->payload); // Timestamp
	i += put_uint32_t_by_index(seq, i, msg->payload); // IMU seq
	i += put_float_by_index(roll, i, msg->payload); // Roll angle in rad
	i += put_float_by_index(pitch, i, msg->payload); // Pitch angle in rad
	i += put_float_by_index(yaw, i, msg->payload); // Yaw angle in rad
	i += put_float_by_index(local_z, i, msg->payload); // Local frame Z coordinate (height over ground)
	i += put_float_by_index(lat, i, msg->payload); // GPS X coordinate
	i += put_float_by_index(lon, i, msg->payload); // GPS Y coordinate
	i += put_float_by_index(alt, i, msg->payload); // Global frame altitude
	i += put_float_by_index(ground_x, i, msg->payload); // Ground truth X
	i += put_float_by_index(ground_y, i, msg->payload); // Ground truth Y
	i += put_float_by_index(ground_z, i, msg->payload); // Ground truth Z

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a image_triggered struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param image_triggered C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_image_triggered_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_triggered_t* image_triggered)
{
	return mavlink_msg_image_triggered_pack(system_id, component_id, msg, image_triggered->timestamp, image_triggered->seq, image_triggered->roll, image_triggered->pitch, image_triggered->yaw, image_triggered->local_z, image_triggered->lat, image_triggered->lon, image_triggered->alt, image_triggered->ground_x, image_triggered->ground_y, image_triggered->ground_z);
}

/**
 * @brief Send a image_triggered message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp
 * @param seq IMU seq
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @param local_z Local frame Z coordinate (height over ground)
 * @param lat GPS X coordinate
 * @param lon GPS Y coordinate
 * @param alt Global frame altitude
 * @param ground_x Ground truth X
 * @param ground_y Ground truth Y
 * @param ground_z Ground truth Z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_image_triggered_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t seq, float roll, float pitch, float yaw, float local_z, float lat, float lon, float alt, float ground_x, float ground_y, float ground_z)
{
	mavlink_message_t msg;
	mavlink_msg_image_triggered_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, timestamp, seq, roll, pitch, yaw, local_z, lat, lon, alt, ground_x, ground_y, ground_z);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE IMAGE_TRIGGERED UNPACKING

/**
 * @brief Get field timestamp from image_triggered message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_image_triggered_get_timestamp(const mavlink_message_t* msg)
{
	generic_64bit r;
	r.b[7] = (msg->payload)[0];
	r.b[6] = (msg->payload)[1];
	r.b[5] = (msg->payload)[2];
	r.b[4] = (msg->payload)[3];
	r.b[3] = (msg->payload)[4];
	r.b[2] = (msg->payload)[5];
	r.b[1] = (msg->payload)[6];
	r.b[0] = (msg->payload)[7];
	return (uint64_t)r.ll;
}

/**
 * @brief Get field seq from image_triggered message
 *
 * @return IMU seq
 */
static inline uint32_t mavlink_msg_image_triggered_get_seq(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field roll from image_triggered message
 *
 * @return Roll angle in rad
 */
static inline float mavlink_msg_image_triggered_get_roll(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitch from image_triggered message
 *
 * @return Pitch angle in rad
 */
static inline float mavlink_msg_image_triggered_get_pitch(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from image_triggered message
 *
 * @return Yaw angle in rad
 */
static inline float mavlink_msg_image_triggered_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field local_z from image_triggered message
 *
 * @return Local frame Z coordinate (height over ground)
 */
static inline float mavlink_msg_image_triggered_get_local_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field lat from image_triggered message
 *
 * @return GPS X coordinate
 */
static inline float mavlink_msg_image_triggered_get_lat(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field lon from image_triggered message
 *
 * @return GPS Y coordinate
 */
static inline float mavlink_msg_image_triggered_get_lon(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field alt from image_triggered message
 *
 * @return Global frame altitude
 */
static inline float mavlink_msg_image_triggered_get_alt(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field ground_x from image_triggered message
 *
 * @return Ground truth X
 */
static inline float mavlink_msg_image_triggered_get_ground_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field ground_y from image_triggered message
 *
 * @return Ground truth Y
 */
static inline float mavlink_msg_image_triggered_get_ground_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field ground_z from image_triggered message
 *
 * @return Ground truth Z
 */
static inline float mavlink_msg_image_triggered_get_ground_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a image_triggered message into a struct
 *
 * @param msg The message to decode
 * @param image_triggered C-struct to decode the message contents into
 */
static inline void mavlink_msg_image_triggered_decode(const mavlink_message_t* msg, mavlink_image_triggered_t* image_triggered)
{
	image_triggered->timestamp = mavlink_msg_image_triggered_get_timestamp(msg);
	image_triggered->seq = mavlink_msg_image_triggered_get_seq(msg);
	image_triggered->roll = mavlink_msg_image_triggered_get_roll(msg);
	image_triggered->pitch = mavlink_msg_image_triggered_get_pitch(msg);
	image_triggered->yaw = mavlink_msg_image_triggered_get_yaw(msg);
	image_triggered->local_z = mavlink_msg_image_triggered_get_local_z(msg);
	image_triggered->lat = mavlink_msg_image_triggered_get_lat(msg);
	image_triggered->lon = mavlink_msg_image_triggered_get_lon(msg);
	image_triggered->alt = mavlink_msg_image_triggered_get_alt(msg);
	image_triggered->ground_x = mavlink_msg_image_triggered_get_ground_x(msg);
	image_triggered->ground_y = mavlink_msg_image_triggered_get_ground_y(msg);
	image_triggered->ground_z = mavlink_msg_image_triggered_get_ground_z(msg);
}
