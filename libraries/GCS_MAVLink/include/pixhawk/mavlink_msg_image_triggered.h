// MESSAGE IMAGE_TRIGGERED PACKING

#define MAVLINK_MSG_ID_IMAGE_TRIGGERED 101

typedef struct __mavlink_image_triggered_t 
{
	uint64_t timestamp; ///< Timestamp
	uint32_t seq; ///< IMU seq
	float roll; ///< Roll angle in rad
	float pitch; ///< Pitch angle in rad

} mavlink_image_triggered_t;



/**
 * @brief Send a image_triggered message
 *
 * @param timestamp Timestamp
 * @param seq IMU seq
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_triggered_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t timestamp, uint32_t seq, float roll, float pitch)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGERED;

	i += put_uint64_t_by_index(timestamp, i, msg->payload); //Timestamp
	i += put_uint32_t_by_index(seq, i, msg->payload); //IMU seq
	i += put_float_by_index(roll, i, msg->payload); //Roll angle in rad
	i += put_float_by_index(pitch, i, msg->payload); //Pitch angle in rad

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_image_triggered_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_triggered_t* image_triggered)
{
	return mavlink_msg_image_triggered_pack(system_id, component_id, msg, image_triggered->timestamp, image_triggered->seq, image_triggered->roll, image_triggered->pitch);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_image_triggered_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t seq, float roll, float pitch)
{
	mavlink_message_t msg;
	mavlink_msg_image_triggered_pack(mavlink_system.sysid, mavlink_system.compid, &msg, timestamp, seq, roll, pitch);
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

static inline void mavlink_msg_image_triggered_decode(const mavlink_message_t* msg, mavlink_image_triggered_t* image_triggered)
{
	image_triggered->timestamp = mavlink_msg_image_triggered_get_timestamp(msg);
	image_triggered->seq = mavlink_msg_image_triggered_get_seq(msg);
	image_triggered->roll = mavlink_msg_image_triggered_get_roll(msg);
	image_triggered->pitch = mavlink_msg_image_triggered_get_pitch(msg);
}
