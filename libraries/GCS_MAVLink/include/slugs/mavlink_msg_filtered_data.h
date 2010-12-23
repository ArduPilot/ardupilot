// MESSAGE FILTERED_DATA PACKING

#define MAVLINK_MSG_ID_FILTERED_DATA 178

typedef struct __mavlink_filtered_data_t 
{
	float aX; ///< Accelerometer X value (m/s^2) 
	float aY; ///< Accelerometer Y value (m/s^2)
	float aZ; ///< Accelerometer Z value (m/s^2)
	float gX; ///< Gyro X value (rad/s) 
	float gY; ///< Gyro Y value (rad/s)
	float gZ; ///< Gyro Z value (rad/s)
	float mX; ///< Magnetometer X (normalized to 1) 
	float mY; ///< Magnetometer Y (normalized to 1) 
	float mZ; ///< Magnetometer Z (normalized to 1) 

} mavlink_filtered_data_t;



/**
 * @brief Send a filtered_data message
 *
 * @param aX Accelerometer X value (m/s^2) 
 * @param aY Accelerometer Y value (m/s^2)
 * @param aZ Accelerometer Z value (m/s^2)
 * @param gX Gyro X value (rad/s) 
 * @param gY Gyro Y value (rad/s)
 * @param gZ Gyro Z value (rad/s)
 * @param mX Magnetometer X (normalized to 1) 
 * @param mY Magnetometer Y (normalized to 1) 
 * @param mZ Magnetometer Z (normalized to 1) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_filtered_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float aX, float aY, float aZ, float gX, float gY, float gZ, float mX, float mY, float mZ)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_FILTERED_DATA;

	i += put_float_by_index(aX, i, msg->payload); //Accelerometer X value (m/s^2) 
	i += put_float_by_index(aY, i, msg->payload); //Accelerometer Y value (m/s^2)
	i += put_float_by_index(aZ, i, msg->payload); //Accelerometer Z value (m/s^2)
	i += put_float_by_index(gX, i, msg->payload); //Gyro X value (rad/s) 
	i += put_float_by_index(gY, i, msg->payload); //Gyro Y value (rad/s)
	i += put_float_by_index(gZ, i, msg->payload); //Gyro Z value (rad/s)
	i += put_float_by_index(mX, i, msg->payload); //Magnetometer X (normalized to 1) 
	i += put_float_by_index(mY, i, msg->payload); //Magnetometer Y (normalized to 1) 
	i += put_float_by_index(mZ, i, msg->payload); //Magnetometer Z (normalized to 1) 

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_filtered_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_filtered_data_t* filtered_data)
{
	return mavlink_msg_filtered_data_pack(system_id, component_id, msg, filtered_data->aX, filtered_data->aY, filtered_data->aZ, filtered_data->gX, filtered_data->gY, filtered_data->gZ, filtered_data->mX, filtered_data->mY, filtered_data->mZ);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_filtered_data_send(mavlink_channel_t chan, float aX, float aY, float aZ, float gX, float gY, float gZ, float mX, float mY, float mZ)
{
	mavlink_message_t msg;
	mavlink_msg_filtered_data_pack(mavlink_system.sysid, mavlink_system.compid, &msg, aX, aY, aZ, gX, gY, gZ, mX, mY, mZ);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE FILTERED_DATA UNPACKING

/**
 * @brief Get field aX from filtered_data message
 *
 * @return Accelerometer X value (m/s^2) 
 */
static inline float mavlink_msg_filtered_data_get_aX(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (float)r.f;
}

/**
 * @brief Get field aY from filtered_data message
 *
 * @return Accelerometer Y value (m/s^2)
 */
static inline float mavlink_msg_filtered_data_get_aY(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field aZ from filtered_data message
 *
 * @return Accelerometer Z value (m/s^2)
 */
static inline float mavlink_msg_filtered_data_get_aZ(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field gX from filtered_data message
 *
 * @return Gyro X value (rad/s) 
 */
static inline float mavlink_msg_filtered_data_get_gX(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field gY from filtered_data message
 *
 * @return Gyro Y value (rad/s)
 */
static inline float mavlink_msg_filtered_data_get_gY(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field gZ from filtered_data message
 *
 * @return Gyro Z value (rad/s)
 */
static inline float mavlink_msg_filtered_data_get_gZ(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field mX from filtered_data message
 *
 * @return Magnetometer X (normalized to 1) 
 */
static inline float mavlink_msg_filtered_data_get_mX(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field mY from filtered_data message
 *
 * @return Magnetometer Y (normalized to 1) 
 */
static inline float mavlink_msg_filtered_data_get_mY(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field mZ from filtered_data message
 *
 * @return Magnetometer Z (normalized to 1) 
 */
static inline float mavlink_msg_filtered_data_get_mZ(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

static inline void mavlink_msg_filtered_data_decode(const mavlink_message_t* msg, mavlink_filtered_data_t* filtered_data)
{
	filtered_data->aX = mavlink_msg_filtered_data_get_aX(msg);
	filtered_data->aY = mavlink_msg_filtered_data_get_aY(msg);
	filtered_data->aZ = mavlink_msg_filtered_data_get_aZ(msg);
	filtered_data->gX = mavlink_msg_filtered_data_get_gX(msg);
	filtered_data->gY = mavlink_msg_filtered_data_get_gY(msg);
	filtered_data->gZ = mavlink_msg_filtered_data_get_gZ(msg);
	filtered_data->mX = mavlink_msg_filtered_data_get_mX(msg);
	filtered_data->mY = mavlink_msg_filtered_data_get_mY(msg);
	filtered_data->mZ = mavlink_msg_filtered_data_get_mZ(msg);
}
