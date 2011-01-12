// MESSAGE REQUEST_STATIC_CALIBRATION PACKING

#define MAVLINK_MSG_ID_REQUEST_STATIC_CALIBRATION 68

typedef struct __mavlink_request_static_calibration_t 
{
	uint8_t target_system; ///< The system which should auto-calibrate
	uint8_t target_component; ///< The system component which should auto-calibrate
	uint16_t time; ///< The time to average over in ms

} mavlink_request_static_calibration_t;



/**
 * @brief Pack a request_static_calibration message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system The system which should auto-calibrate
 * @param target_component The system component which should auto-calibrate
 * @param time The time to average over in ms
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_static_calibration_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t time)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_REQUEST_STATIC_CALIBRATION;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // The system which should auto-calibrate
	i += put_uint8_t_by_index(target_component, i, msg->payload); // The system component which should auto-calibrate
	i += put_uint16_t_by_index(time, i, msg->payload); // The time to average over in ms

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a request_static_calibration message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system The system which should auto-calibrate
 * @param target_component The system component which should auto-calibrate
 * @param time The time to average over in ms
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_static_calibration_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t time)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_REQUEST_STATIC_CALIBRATION;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // The system which should auto-calibrate
	i += put_uint8_t_by_index(target_component, i, msg->payload); // The system component which should auto-calibrate
	i += put_uint16_t_by_index(time, i, msg->payload); // The time to average over in ms

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a request_static_calibration struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param request_static_calibration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_static_calibration_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_static_calibration_t* request_static_calibration)
{
	return mavlink_msg_request_static_calibration_pack(system_id, component_id, msg, request_static_calibration->target_system, request_static_calibration->target_component, request_static_calibration->time);
}

/**
 * @brief Send a request_static_calibration message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system The system which should auto-calibrate
 * @param target_component The system component which should auto-calibrate
 * @param time The time to average over in ms
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_static_calibration_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t time)
{
	mavlink_message_t msg;
	mavlink_msg_request_static_calibration_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, time);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE REQUEST_STATIC_CALIBRATION UNPACKING

/**
 * @brief Get field target_system from request_static_calibration message
 *
 * @return The system which should auto-calibrate
 */
static inline uint8_t mavlink_msg_request_static_calibration_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from request_static_calibration message
 *
 * @return The system component which should auto-calibrate
 */
static inline uint8_t mavlink_msg_request_static_calibration_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field time from request_static_calibration message
 *
 * @return The time to average over in ms
 */
static inline uint16_t mavlink_msg_request_static_calibration_get_time(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Decode a request_static_calibration message into a struct
 *
 * @param msg The message to decode
 * @param request_static_calibration C-struct to decode the message contents into
 */
static inline void mavlink_msg_request_static_calibration_decode(const mavlink_message_t* msg, mavlink_request_static_calibration_t* request_static_calibration)
{
	request_static_calibration->target_system = mavlink_msg_request_static_calibration_get_target_system(msg);
	request_static_calibration->target_component = mavlink_msg_request_static_calibration_get_target_component(msg);
	request_static_calibration->time = mavlink_msg_request_static_calibration_get_time(msg);
}
