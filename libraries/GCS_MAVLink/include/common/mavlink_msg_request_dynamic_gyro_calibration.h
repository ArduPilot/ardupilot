// MESSAGE REQUEST_DYNAMIC_GYRO_CALIBRATION PACKING

#define MAVLINK_MSG_ID_REQUEST_DYNAMIC_GYRO_CALIBRATION 67

typedef struct __mavlink_request_dynamic_gyro_calibration_t 
{
	uint8_t target_system; ///< The system which should auto-calibrate
	uint8_t target_component; ///< The system component which should auto-calibrate
	float mode; ///< The current ground-truth rpm
	uint8_t axis; ///< The axis to calibrate: 0 roll, 1 pitch, 2 yaw
	uint16_t time; ///< The time to average over in ms

} mavlink_request_dynamic_gyro_calibration_t;



/**
 * @brief Pack a request_dynamic_gyro_calibration message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system The system which should auto-calibrate
 * @param target_component The system component which should auto-calibrate
 * @param mode The current ground-truth rpm
 * @param axis The axis to calibrate: 0 roll, 1 pitch, 2 yaw
 * @param time The time to average over in ms
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_dynamic_gyro_calibration_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, float mode, uint8_t axis, uint16_t time)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_REQUEST_DYNAMIC_GYRO_CALIBRATION;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // The system which should auto-calibrate
	i += put_uint8_t_by_index(target_component, i, msg->payload); // The system component which should auto-calibrate
	i += put_float_by_index(mode, i, msg->payload); // The current ground-truth rpm
	i += put_uint8_t_by_index(axis, i, msg->payload); // The axis to calibrate: 0 roll, 1 pitch, 2 yaw
	i += put_uint16_t_by_index(time, i, msg->payload); // The time to average over in ms

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a request_dynamic_gyro_calibration message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system The system which should auto-calibrate
 * @param target_component The system component which should auto-calibrate
 * @param mode The current ground-truth rpm
 * @param axis The axis to calibrate: 0 roll, 1 pitch, 2 yaw
 * @param time The time to average over in ms
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_dynamic_gyro_calibration_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, float mode, uint8_t axis, uint16_t time)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_REQUEST_DYNAMIC_GYRO_CALIBRATION;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // The system which should auto-calibrate
	i += put_uint8_t_by_index(target_component, i, msg->payload); // The system component which should auto-calibrate
	i += put_float_by_index(mode, i, msg->payload); // The current ground-truth rpm
	i += put_uint8_t_by_index(axis, i, msg->payload); // The axis to calibrate: 0 roll, 1 pitch, 2 yaw
	i += put_uint16_t_by_index(time, i, msg->payload); // The time to average over in ms

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a request_dynamic_gyro_calibration struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param request_dynamic_gyro_calibration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_dynamic_gyro_calibration_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_dynamic_gyro_calibration_t* request_dynamic_gyro_calibration)
{
	return mavlink_msg_request_dynamic_gyro_calibration_pack(system_id, component_id, msg, request_dynamic_gyro_calibration->target_system, request_dynamic_gyro_calibration->target_component, request_dynamic_gyro_calibration->mode, request_dynamic_gyro_calibration->axis, request_dynamic_gyro_calibration->time);
}

/**
 * @brief Send a request_dynamic_gyro_calibration message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system The system which should auto-calibrate
 * @param target_component The system component which should auto-calibrate
 * @param mode The current ground-truth rpm
 * @param axis The axis to calibrate: 0 roll, 1 pitch, 2 yaw
 * @param time The time to average over in ms
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_dynamic_gyro_calibration_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float mode, uint8_t axis, uint16_t time)
{
	mavlink_message_t msg;
	mavlink_msg_request_dynamic_gyro_calibration_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, mode, axis, time);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE REQUEST_DYNAMIC_GYRO_CALIBRATION UNPACKING

/**
 * @brief Get field target_system from request_dynamic_gyro_calibration message
 *
 * @return The system which should auto-calibrate
 */
static inline uint8_t mavlink_msg_request_dynamic_gyro_calibration_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from request_dynamic_gyro_calibration message
 *
 * @return The system component which should auto-calibrate
 */
static inline uint8_t mavlink_msg_request_dynamic_gyro_calibration_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field mode from request_dynamic_gyro_calibration message
 *
 * @return The current ground-truth rpm
 */
static inline float mavlink_msg_request_dynamic_gyro_calibration_get_mode(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field axis from request_dynamic_gyro_calibration message
 *
 * @return The axis to calibrate: 0 roll, 1 pitch, 2 yaw
 */
static inline uint8_t mavlink_msg_request_dynamic_gyro_calibration_get_axis(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[0];
}

/**
 * @brief Get field time from request_dynamic_gyro_calibration message
 *
 * @return The time to average over in ms
 */
static inline uint16_t mavlink_msg_request_dynamic_gyro_calibration_get_time(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Decode a request_dynamic_gyro_calibration message into a struct
 *
 * @param msg The message to decode
 * @param request_dynamic_gyro_calibration C-struct to decode the message contents into
 */
static inline void mavlink_msg_request_dynamic_gyro_calibration_decode(const mavlink_message_t* msg, mavlink_request_dynamic_gyro_calibration_t* request_dynamic_gyro_calibration)
{
	request_dynamic_gyro_calibration->target_system = mavlink_msg_request_dynamic_gyro_calibration_get_target_system(msg);
	request_dynamic_gyro_calibration->target_component = mavlink_msg_request_dynamic_gyro_calibration_get_target_component(msg);
	request_dynamic_gyro_calibration->mode = mavlink_msg_request_dynamic_gyro_calibration_get_mode(msg);
	request_dynamic_gyro_calibration->axis = mavlink_msg_request_dynamic_gyro_calibration_get_axis(msg);
	request_dynamic_gyro_calibration->time = mavlink_msg_request_dynamic_gyro_calibration_get_time(msg);
}
