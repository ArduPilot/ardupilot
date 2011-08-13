// MESSAGE HIL_CONTROLS PACKING

#define MAVLINK_MSG_ID_HIL_CONTROLS 68

typedef struct __mavlink_hil_controls_t 
{
	uint64_t time_us; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	float roll_ailerons; ///< Control output -3 .. 1
	float pitch_elevator; ///< Control output -1 .. 1
	float yaw_rudder; ///< Control output -1 .. 1
	float throttle; ///< Throttle 0 .. 1
	uint8_t mode; ///< System mode (MAV_MODE)
	uint8_t nav_mode; ///< Navigation mode (MAV_NAV_MODE)

} mavlink_hil_controls_t;



/**
 * @brief Pack a hil_controls message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll_ailerons Control output -3 .. 1
 * @param pitch_elevator Control output -1 .. 1
 * @param yaw_rudder Control output -1 .. 1
 * @param throttle Throttle 0 .. 1
 * @param mode System mode (MAV_MODE)
 * @param nav_mode Navigation mode (MAV_NAV_MODE)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_controls_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t time_us, float roll_ailerons, float pitch_elevator, float yaw_rudder, float throttle, uint8_t mode, uint8_t nav_mode)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_HIL_CONTROLS;

	i += put_uint64_t_by_index(time_us, i, msg->payload); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	i += put_float_by_index(roll_ailerons, i, msg->payload); // Control output -3 .. 1
	i += put_float_by_index(pitch_elevator, i, msg->payload); // Control output -1 .. 1
	i += put_float_by_index(yaw_rudder, i, msg->payload); // Control output -1 .. 1
	i += put_float_by_index(throttle, i, msg->payload); // Throttle 0 .. 1
	i += put_uint8_t_by_index(mode, i, msg->payload); // System mode (MAV_MODE)
	i += put_uint8_t_by_index(nav_mode, i, msg->payload); // Navigation mode (MAV_NAV_MODE)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a hil_controls message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_us Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll_ailerons Control output -3 .. 1
 * @param pitch_elevator Control output -1 .. 1
 * @param yaw_rudder Control output -1 .. 1
 * @param throttle Throttle 0 .. 1
 * @param mode System mode (MAV_MODE)
 * @param nav_mode Navigation mode (MAV_NAV_MODE)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_controls_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t time_us, float roll_ailerons, float pitch_elevator, float yaw_rudder, float throttle, uint8_t mode, uint8_t nav_mode)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_HIL_CONTROLS;

	i += put_uint64_t_by_index(time_us, i, msg->payload); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	i += put_float_by_index(roll_ailerons, i, msg->payload); // Control output -3 .. 1
	i += put_float_by_index(pitch_elevator, i, msg->payload); // Control output -1 .. 1
	i += put_float_by_index(yaw_rudder, i, msg->payload); // Control output -1 .. 1
	i += put_float_by_index(throttle, i, msg->payload); // Throttle 0 .. 1
	i += put_uint8_t_by_index(mode, i, msg->payload); // System mode (MAV_MODE)
	i += put_uint8_t_by_index(nav_mode, i, msg->payload); // Navigation mode (MAV_NAV_MODE)

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a hil_controls struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_controls C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_controls_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_controls_t* hil_controls)
{
	return mavlink_msg_hil_controls_pack(system_id, component_id, msg, hil_controls->time_us, hil_controls->roll_ailerons, hil_controls->pitch_elevator, hil_controls->yaw_rudder, hil_controls->throttle, hil_controls->mode, hil_controls->nav_mode);
}

/**
 * @brief Send a hil_controls message
 * @param chan MAVLink channel to send the message
 *
 * @param time_us Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll_ailerons Control output -3 .. 1
 * @param pitch_elevator Control output -1 .. 1
 * @param yaw_rudder Control output -1 .. 1
 * @param throttle Throttle 0 .. 1
 * @param mode System mode (MAV_MODE)
 * @param nav_mode Navigation mode (MAV_NAV_MODE)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_controls_send(mavlink_channel_t chan, uint64_t time_us, float roll_ailerons, float pitch_elevator, float yaw_rudder, float throttle, uint8_t mode, uint8_t nav_mode)
{
	mavlink_message_t msg;
	mavlink_msg_hil_controls_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, time_us, roll_ailerons, pitch_elevator, yaw_rudder, throttle, mode, nav_mode);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE HIL_CONTROLS UNPACKING

/**
 * @brief Get field time_us from hil_controls message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_hil_controls_get_time_us(const mavlink_message_t* msg)
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
 * @brief Get field roll_ailerons from hil_controls message
 *
 * @return Control output -3 .. 1
 */
static inline float mavlink_msg_hil_controls_get_roll_ailerons(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitch_elevator from hil_controls message
 *
 * @return Control output -1 .. 1
 */
static inline float mavlink_msg_hil_controls_get_pitch_elevator(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw_rudder from hil_controls message
 *
 * @return Control output -1 .. 1
 */
static inline float mavlink_msg_hil_controls_get_yaw_rudder(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field throttle from hil_controls message
 *
 * @return Throttle 0 .. 1
 */
static inline float mavlink_msg_hil_controls_get_throttle(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field mode from hil_controls message
 *
 * @return System mode (MAV_MODE)
 */
static inline uint8_t mavlink_msg_hil_controls_get_mode(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
}

/**
 * @brief Get field nav_mode from hil_controls message
 *
 * @return Navigation mode (MAV_NAV_MODE)
 */
static inline uint8_t mavlink_msg_hil_controls_get_nav_mode(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t))[0];
}

/**
 * @brief Decode a hil_controls message into a struct
 *
 * @param msg The message to decode
 * @param hil_controls C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_controls_decode(const mavlink_message_t* msg, mavlink_hil_controls_t* hil_controls)
{
	hil_controls->time_us = mavlink_msg_hil_controls_get_time_us(msg);
	hil_controls->roll_ailerons = mavlink_msg_hil_controls_get_roll_ailerons(msg);
	hil_controls->pitch_elevator = mavlink_msg_hil_controls_get_pitch_elevator(msg);
	hil_controls->yaw_rudder = mavlink_msg_hil_controls_get_yaw_rudder(msg);
	hil_controls->throttle = mavlink_msg_hil_controls_get_throttle(msg);
	hil_controls->mode = mavlink_msg_hil_controls_get_mode(msg);
	hil_controls->nav_mode = mavlink_msg_hil_controls_get_nav_mode(msg);
}
