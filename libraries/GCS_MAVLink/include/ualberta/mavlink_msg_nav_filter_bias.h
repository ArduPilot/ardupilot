// MESSAGE NAV_FILTER_BIAS PACKING

#define MAVLINK_MSG_ID_NAV_FILTER_BIAS 220

typedef struct __mavlink_nav_filter_bias_t 
{
	uint64_t usec; ///< Timestamp (microseconds)
	float accel_0; ///< b_f[0]
	float accel_1; ///< b_f[1]
	float accel_2; ///< b_f[2]
	float gyro_0; ///< b_f[0]
	float gyro_1; ///< b_f[1]
	float gyro_2; ///< b_f[2]

} mavlink_nav_filter_bias_t;



/**
 * @brief Send a nav_filter_bias message
 *
 * @param usec Timestamp (microseconds)
 * @param accel_0 b_f[0]
 * @param accel_1 b_f[1]
 * @param accel_2 b_f[2]
 * @param gyro_0 b_f[0]
 * @param gyro_1 b_f[1]
 * @param gyro_2 b_f[2]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nav_filter_bias_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_NAV_FILTER_BIAS;

	i += put_uint64_t_by_index(usec, i, msg->payload); //Timestamp (microseconds)
	i += put_float_by_index(accel_0, i, msg->payload); //b_f[0]
	i += put_float_by_index(accel_1, i, msg->payload); //b_f[1]
	i += put_float_by_index(accel_2, i, msg->payload); //b_f[2]
	i += put_float_by_index(gyro_0, i, msg->payload); //b_f[0]
	i += put_float_by_index(gyro_1, i, msg->payload); //b_f[1]
	i += put_float_by_index(gyro_2, i, msg->payload); //b_f[2]

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_nav_filter_bias_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nav_filter_bias_t* nav_filter_bias)
{
	return mavlink_msg_nav_filter_bias_pack(system_id, component_id, msg, nav_filter_bias->usec, nav_filter_bias->accel_0, nav_filter_bias->accel_1, nav_filter_bias->accel_2, nav_filter_bias->gyro_0, nav_filter_bias->gyro_1, nav_filter_bias->gyro_2);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nav_filter_bias_send(mavlink_channel_t chan, uint64_t usec, float accel_0, float accel_1, float accel_2, float gyro_0, float gyro_1, float gyro_2)
{
	mavlink_message_t msg;
	mavlink_msg_nav_filter_bias_pack(mavlink_system.sysid, mavlink_system.compid, &msg, usec, accel_0, accel_1, accel_2, gyro_0, gyro_1, gyro_2);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE NAV_FILTER_BIAS UNPACKING

/**
 * @brief Get field usec from nav_filter_bias message
 *
 * @return Timestamp (microseconds)
 */
static inline uint64_t mavlink_msg_nav_filter_bias_get_usec(const mavlink_message_t* msg)
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
 * @brief Get field accel_0 from nav_filter_bias message
 *
 * @return b_f[0]
 */
static inline float mavlink_msg_nav_filter_bias_get_accel_0(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field accel_1 from nav_filter_bias message
 *
 * @return b_f[1]
 */
static inline float mavlink_msg_nav_filter_bias_get_accel_1(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field accel_2 from nav_filter_bias message
 *
 * @return b_f[2]
 */
static inline float mavlink_msg_nav_filter_bias_get_accel_2(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field gyro_0 from nav_filter_bias message
 *
 * @return b_f[0]
 */
static inline float mavlink_msg_nav_filter_bias_get_gyro_0(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field gyro_1 from nav_filter_bias message
 *
 * @return b_f[1]
 */
static inline float mavlink_msg_nav_filter_bias_get_gyro_1(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field gyro_2 from nav_filter_bias message
 *
 * @return b_f[2]
 */
static inline float mavlink_msg_nav_filter_bias_get_gyro_2(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

static inline void mavlink_msg_nav_filter_bias_decode(const mavlink_message_t* msg, mavlink_nav_filter_bias_t* nav_filter_bias)
{
	nav_filter_bias->usec = mavlink_msg_nav_filter_bias_get_usec(msg);
	nav_filter_bias->accel_0 = mavlink_msg_nav_filter_bias_get_accel_0(msg);
	nav_filter_bias->accel_1 = mavlink_msg_nav_filter_bias_get_accel_1(msg);
	nav_filter_bias->accel_2 = mavlink_msg_nav_filter_bias_get_accel_2(msg);
	nav_filter_bias->gyro_0 = mavlink_msg_nav_filter_bias_get_gyro_0(msg);
	nav_filter_bias->gyro_1 = mavlink_msg_nav_filter_bias_get_gyro_1(msg);
	nav_filter_bias->gyro_2 = mavlink_msg_nav_filter_bias_get_gyro_2(msg);
}
