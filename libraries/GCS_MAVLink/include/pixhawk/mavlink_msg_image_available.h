// MESSAGE IMAGE_AVAILABLE PACKING

#define MAVLINK_MSG_ID_IMAGE_AVAILABLE 103

typedef struct __mavlink_image_available_t 
{
	uint64_t cam_id; ///< Camera id
	uint8_t cam_no; ///< Camera # (starts with 0)
	uint64_t timestamp; ///< Timestamp
	uint64_t valid_until; ///< Until which timestamp this buffer will stay valid
	uint32_t img_seq; ///< The image sequence number
	uint32_t img_buf_index; ///< Position of the image in the buffer, starts with 0
	uint16_t width; ///< Image width
	uint16_t height; ///< Image height
	uint16_t depth; ///< Image depth
	uint8_t channels; ///< Image channels
	uint32_t key; ///< Shared memory area key
	uint32_t exposure; ///< Exposure time, in microseconds
	float gain; ///< Camera gain
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

} mavlink_image_available_t;



/**
 * @brief Pack a image_available message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cam_id Camera id
 * @param cam_no Camera # (starts with 0)
 * @param timestamp Timestamp
 * @param valid_until Until which timestamp this buffer will stay valid
 * @param img_seq The image sequence number
 * @param img_buf_index Position of the image in the buffer, starts with 0
 * @param width Image width
 * @param height Image height
 * @param depth Image depth
 * @param channels Image channels
 * @param key Shared memory area key
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
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
static inline uint16_t mavlink_msg_image_available_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t cam_id, uint8_t cam_no, uint64_t timestamp, uint64_t valid_until, uint32_t img_seq, uint32_t img_buf_index, uint16_t width, uint16_t height, uint16_t depth, uint8_t channels, uint32_t key, uint32_t exposure, float gain, float roll, float pitch, float yaw, float local_z, float lat, float lon, float alt, float ground_x, float ground_y, float ground_z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_IMAGE_AVAILABLE;

	i += put_uint64_t_by_index(cam_id, i, msg->payload); // Camera id
	i += put_uint8_t_by_index(cam_no, i, msg->payload); // Camera # (starts with 0)
	i += put_uint64_t_by_index(timestamp, i, msg->payload); // Timestamp
	i += put_uint64_t_by_index(valid_until, i, msg->payload); // Until which timestamp this buffer will stay valid
	i += put_uint32_t_by_index(img_seq, i, msg->payload); // The image sequence number
	i += put_uint32_t_by_index(img_buf_index, i, msg->payload); // Position of the image in the buffer, starts with 0
	i += put_uint16_t_by_index(width, i, msg->payload); // Image width
	i += put_uint16_t_by_index(height, i, msg->payload); // Image height
	i += put_uint16_t_by_index(depth, i, msg->payload); // Image depth
	i += put_uint8_t_by_index(channels, i, msg->payload); // Image channels
	i += put_uint32_t_by_index(key, i, msg->payload); // Shared memory area key
	i += put_uint32_t_by_index(exposure, i, msg->payload); // Exposure time, in microseconds
	i += put_float_by_index(gain, i, msg->payload); // Camera gain
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
 * @brief Pack a image_available message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param cam_id Camera id
 * @param cam_no Camera # (starts with 0)
 * @param timestamp Timestamp
 * @param valid_until Until which timestamp this buffer will stay valid
 * @param img_seq The image sequence number
 * @param img_buf_index Position of the image in the buffer, starts with 0
 * @param width Image width
 * @param height Image height
 * @param depth Image depth
 * @param channels Image channels
 * @param key Shared memory area key
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
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
static inline uint16_t mavlink_msg_image_available_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t cam_id, uint8_t cam_no, uint64_t timestamp, uint64_t valid_until, uint32_t img_seq, uint32_t img_buf_index, uint16_t width, uint16_t height, uint16_t depth, uint8_t channels, uint32_t key, uint32_t exposure, float gain, float roll, float pitch, float yaw, float local_z, float lat, float lon, float alt, float ground_x, float ground_y, float ground_z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_IMAGE_AVAILABLE;

	i += put_uint64_t_by_index(cam_id, i, msg->payload); // Camera id
	i += put_uint8_t_by_index(cam_no, i, msg->payload); // Camera # (starts with 0)
	i += put_uint64_t_by_index(timestamp, i, msg->payload); // Timestamp
	i += put_uint64_t_by_index(valid_until, i, msg->payload); // Until which timestamp this buffer will stay valid
	i += put_uint32_t_by_index(img_seq, i, msg->payload); // The image sequence number
	i += put_uint32_t_by_index(img_buf_index, i, msg->payload); // Position of the image in the buffer, starts with 0
	i += put_uint16_t_by_index(width, i, msg->payload); // Image width
	i += put_uint16_t_by_index(height, i, msg->payload); // Image height
	i += put_uint16_t_by_index(depth, i, msg->payload); // Image depth
	i += put_uint8_t_by_index(channels, i, msg->payload); // Image channels
	i += put_uint32_t_by_index(key, i, msg->payload); // Shared memory area key
	i += put_uint32_t_by_index(exposure, i, msg->payload); // Exposure time, in microseconds
	i += put_float_by_index(gain, i, msg->payload); // Camera gain
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
 * @brief Encode a image_available struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param image_available C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_image_available_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_available_t* image_available)
{
	return mavlink_msg_image_available_pack(system_id, component_id, msg, image_available->cam_id, image_available->cam_no, image_available->timestamp, image_available->valid_until, image_available->img_seq, image_available->img_buf_index, image_available->width, image_available->height, image_available->depth, image_available->channels, image_available->key, image_available->exposure, image_available->gain, image_available->roll, image_available->pitch, image_available->yaw, image_available->local_z, image_available->lat, image_available->lon, image_available->alt, image_available->ground_x, image_available->ground_y, image_available->ground_z);
}

/**
 * @brief Send a image_available message
 * @param chan MAVLink channel to send the message
 *
 * @param cam_id Camera id
 * @param cam_no Camera # (starts with 0)
 * @param timestamp Timestamp
 * @param valid_until Until which timestamp this buffer will stay valid
 * @param img_seq The image sequence number
 * @param img_buf_index Position of the image in the buffer, starts with 0
 * @param width Image width
 * @param height Image height
 * @param depth Image depth
 * @param channels Image channels
 * @param key Shared memory area key
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
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

static inline void mavlink_msg_image_available_send(mavlink_channel_t chan, uint64_t cam_id, uint8_t cam_no, uint64_t timestamp, uint64_t valid_until, uint32_t img_seq, uint32_t img_buf_index, uint16_t width, uint16_t height, uint16_t depth, uint8_t channels, uint32_t key, uint32_t exposure, float gain, float roll, float pitch, float yaw, float local_z, float lat, float lon, float alt, float ground_x, float ground_y, float ground_z)
{
	mavlink_message_t msg;
	mavlink_msg_image_available_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, cam_id, cam_no, timestamp, valid_until, img_seq, img_buf_index, width, height, depth, channels, key, exposure, gain, roll, pitch, yaw, local_z, lat, lon, alt, ground_x, ground_y, ground_z);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE IMAGE_AVAILABLE UNPACKING

/**
 * @brief Get field cam_id from image_available message
 *
 * @return Camera id
 */
static inline uint64_t mavlink_msg_image_available_get_cam_id(const mavlink_message_t* msg)
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
 * @brief Get field cam_no from image_available message
 *
 * @return Camera # (starts with 0)
 */
static inline uint8_t mavlink_msg_image_available_get_cam_no(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint64_t))[0];
}

/**
 * @brief Get field timestamp from image_available message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_image_available_get_timestamp(const mavlink_message_t* msg)
{
	generic_64bit r;
	r.b[7] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[0];
	r.b[6] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[1];
	r.b[5] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[2];
	r.b[4] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[3];
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[4];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[5];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[6];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t))[7];
	return (uint64_t)r.ll;
}

/**
 * @brief Get field valid_until from image_available message
 *
 * @return Until which timestamp this buffer will stay valid
 */
static inline uint64_t mavlink_msg_image_available_get_valid_until(const mavlink_message_t* msg)
{
	generic_64bit r;
	r.b[7] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t))[0];
	r.b[6] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t))[1];
	r.b[5] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t))[2];
	r.b[4] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t))[3];
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t))[4];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t))[5];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t))[6];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t))[7];
	return (uint64_t)r.ll;
}

/**
 * @brief Get field img_seq from image_available message
 *
 * @return The image sequence number
 */
static inline uint32_t mavlink_msg_image_available_get_img_seq(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field img_buf_index from image_available message
 *
 * @return Position of the image in the buffer, starts with 0
 */
static inline uint32_t mavlink_msg_image_available_get_img_buf_index(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field width from image_available message
 *
 * @return Image width
 */
static inline uint16_t mavlink_msg_image_available_get_width(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field height from image_available message
 *
 * @return Image height
 */
static inline uint16_t mavlink_msg_image_available_get_height(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field depth from image_available message
 *
 * @return Image depth
 */
static inline uint16_t mavlink_msg_image_available_get_depth(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field channels from image_available message
 *
 * @return Image channels
 */
static inline uint8_t mavlink_msg_image_available_get_channels(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
}

/**
 * @brief Get field key from image_available message
 *
 * @return Shared memory area key
 */
static inline uint32_t mavlink_msg_image_available_get_key(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field exposure from image_available message
 *
 * @return Exposure time, in microseconds
 */
static inline uint32_t mavlink_msg_image_available_get_exposure(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Get field gain from image_available message
 *
 * @return Camera gain
 */
static inline float mavlink_msg_image_available_get_gain(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field roll from image_available message
 *
 * @return Roll angle in rad
 */
static inline float mavlink_msg_image_available_get_roll(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitch from image_available message
 *
 * @return Pitch angle in rad
 */
static inline float mavlink_msg_image_available_get_pitch(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from image_available message
 *
 * @return Yaw angle in rad
 */
static inline float mavlink_msg_image_available_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field local_z from image_available message
 *
 * @return Local frame Z coordinate (height over ground)
 */
static inline float mavlink_msg_image_available_get_local_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field lat from image_available message
 *
 * @return GPS X coordinate
 */
static inline float mavlink_msg_image_available_get_lat(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field lon from image_available message
 *
 * @return GPS Y coordinate
 */
static inline float mavlink_msg_image_available_get_lon(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field alt from image_available message
 *
 * @return Global frame altitude
 */
static inline float mavlink_msg_image_available_get_alt(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field ground_x from image_available message
 *
 * @return Ground truth X
 */
static inline float mavlink_msg_image_available_get_ground_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field ground_y from image_available message
 *
 * @return Ground truth Y
 */
static inline float mavlink_msg_image_available_get_ground_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field ground_z from image_available message
 *
 * @return Ground truth Z
 */
static inline float mavlink_msg_image_available_get_ground_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(uint8_t)+sizeof(uint64_t)+sizeof(uint64_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)+sizeof(uint32_t)+sizeof(uint32_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a image_available message into a struct
 *
 * @param msg The message to decode
 * @param image_available C-struct to decode the message contents into
 */
static inline void mavlink_msg_image_available_decode(const mavlink_message_t* msg, mavlink_image_available_t* image_available)
{
	image_available->cam_id = mavlink_msg_image_available_get_cam_id(msg);
	image_available->cam_no = mavlink_msg_image_available_get_cam_no(msg);
	image_available->timestamp = mavlink_msg_image_available_get_timestamp(msg);
	image_available->valid_until = mavlink_msg_image_available_get_valid_until(msg);
	image_available->img_seq = mavlink_msg_image_available_get_img_seq(msg);
	image_available->img_buf_index = mavlink_msg_image_available_get_img_buf_index(msg);
	image_available->width = mavlink_msg_image_available_get_width(msg);
	image_available->height = mavlink_msg_image_available_get_height(msg);
	image_available->depth = mavlink_msg_image_available_get_depth(msg);
	image_available->channels = mavlink_msg_image_available_get_channels(msg);
	image_available->key = mavlink_msg_image_available_get_key(msg);
	image_available->exposure = mavlink_msg_image_available_get_exposure(msg);
	image_available->gain = mavlink_msg_image_available_get_gain(msg);
	image_available->roll = mavlink_msg_image_available_get_roll(msg);
	image_available->pitch = mavlink_msg_image_available_get_pitch(msg);
	image_available->yaw = mavlink_msg_image_available_get_yaw(msg);
	image_available->local_z = mavlink_msg_image_available_get_local_z(msg);
	image_available->lat = mavlink_msg_image_available_get_lat(msg);
	image_available->lon = mavlink_msg_image_available_get_lon(msg);
	image_available->alt = mavlink_msg_image_available_get_alt(msg);
	image_available->ground_x = mavlink_msg_image_available_get_ground_x(msg);
	image_available->ground_y = mavlink_msg_image_available_get_ground_y(msg);
	image_available->ground_z = mavlink_msg_image_available_get_ground_z(msg);
}
