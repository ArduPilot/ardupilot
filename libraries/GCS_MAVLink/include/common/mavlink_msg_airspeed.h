// MESSAGE AIRSPEED PACKING

#define MAVLINK_MSG_ID_AIRSPEED 72

typedef struct __mavlink_airspeed_t 
{
	float airspeed; ///< meters/second

} mavlink_airspeed_t;



/**
 * @brief Pack a airspeed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param airspeed meters/second
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeed_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float airspeed)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_AIRSPEED;

	i += put_float_by_index(airspeed, i, msg->payload); // meters/second

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a airspeed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param airspeed meters/second
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeed_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, float airspeed)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_AIRSPEED;

	i += put_float_by_index(airspeed, i, msg->payload); // meters/second

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a airspeed struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airspeed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airspeed_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airspeed_t* airspeed)
{
	return mavlink_msg_airspeed_pack(system_id, component_id, msg, airspeed->airspeed);
}

/**
 * @brief Send a airspeed message
 * @param chan MAVLink channel to send the message
 *
 * @param airspeed meters/second
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airspeed_send(mavlink_channel_t chan, float airspeed)
{
	mavlink_message_t msg;
	mavlink_msg_airspeed_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, airspeed);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE AIRSPEED UNPACKING

/**
 * @brief Get field airspeed from airspeed message
 *
 * @return meters/second
 */
static inline float mavlink_msg_airspeed_get_airspeed(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (float)r.f;
}

/**
 * @brief Decode a airspeed message into a struct
 *
 * @param msg The message to decode
 * @param airspeed C-struct to decode the message contents into
 */
static inline void mavlink_msg_airspeed_decode(const mavlink_message_t* msg, mavlink_airspeed_t* airspeed)
{
	airspeed->airspeed = mavlink_msg_airspeed_get_airspeed(msg);
}
