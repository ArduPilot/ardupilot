// MESSAGE NAV_CONTROLLER_OUTPUT PACKING

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT 62

typedef struct __mavlink_nav_controller_output_t
{
 float nav_roll; ///< Current desired roll in degrees
 float nav_pitch; ///< Current desired pitch in degrees
 int16_t nav_bearing; ///< Current desired heading in degrees
 int16_t target_bearing; ///< Bearing to current waypoint/target in degrees
 uint16_t wp_dist; ///< Distance to active waypoint in meters
 float alt_error; ///< Current altitude error in meters
 float aspd_error; ///< Current airspeed error in meters/second
 float xtrack_error; ///< Current crosstrack error on x-y plane in meters
} mavlink_nav_controller_output_t;

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN 26
#define MAVLINK_MSG_ID_62_LEN 26



#define MAVLINK_MESSAGE_INFO_NAV_CONTROLLER_OUTPUT { \
	"NAV_CONTROLLER_OUTPUT", \
	8, \
	{  { "nav_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_nav_controller_output_t, nav_roll) }, \
         { "nav_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_nav_controller_output_t, nav_pitch) }, \
         { "nav_bearing", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_nav_controller_output_t, nav_bearing) }, \
         { "target_bearing", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_nav_controller_output_t, target_bearing) }, \
         { "wp_dist", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_nav_controller_output_t, wp_dist) }, \
         { "alt_error", NULL, MAVLINK_TYPE_FLOAT, 0, 14, offsetof(mavlink_nav_controller_output_t, alt_error) }, \
         { "aspd_error", NULL, MAVLINK_TYPE_FLOAT, 0, 18, offsetof(mavlink_nav_controller_output_t, aspd_error) }, \
         { "xtrack_error", NULL, MAVLINK_TYPE_FLOAT, 0, 22, offsetof(mavlink_nav_controller_output_t, xtrack_error) }, \
         } \
}


/**
 * @brief Pack a nav_controller_output message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nav_roll Current desired roll in degrees
 * @param nav_pitch Current desired pitch in degrees
 * @param nav_bearing Current desired heading in degrees
 * @param target_bearing Bearing to current waypoint/target in degrees
 * @param wp_dist Distance to active waypoint in meters
 * @param alt_error Current altitude error in meters
 * @param aspd_error Current airspeed error in meters/second
 * @param xtrack_error Current crosstrack error on x-y plane in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nav_controller_output_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[26];
	_mav_put_float(buf, 0, nav_roll);
	_mav_put_float(buf, 4, nav_pitch);
	_mav_put_int16_t(buf, 8, nav_bearing);
	_mav_put_int16_t(buf, 10, target_bearing);
	_mav_put_uint16_t(buf, 12, wp_dist);
	_mav_put_float(buf, 14, alt_error);
	_mav_put_float(buf, 18, aspd_error);
	_mav_put_float(buf, 22, xtrack_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 26);
#else
	mavlink_nav_controller_output_t packet;
	packet.nav_roll = nav_roll;
	packet.nav_pitch = nav_pitch;
	packet.nav_bearing = nav_bearing;
	packet.target_bearing = target_bearing;
	packet.wp_dist = wp_dist;
	packet.alt_error = alt_error;
	packet.aspd_error = aspd_error;
	packet.xtrack_error = xtrack_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 26);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
	return mavlink_finalize_message(msg, system_id, component_id, 26);
}

/**
 * @brief Pack a nav_controller_output message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param nav_roll Current desired roll in degrees
 * @param nav_pitch Current desired pitch in degrees
 * @param nav_bearing Current desired heading in degrees
 * @param target_bearing Bearing to current waypoint/target in degrees
 * @param wp_dist Distance to active waypoint in meters
 * @param alt_error Current altitude error in meters
 * @param aspd_error Current airspeed error in meters/second
 * @param xtrack_error Current crosstrack error on x-y plane in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nav_controller_output_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float nav_roll,float nav_pitch,int16_t nav_bearing,int16_t target_bearing,uint16_t wp_dist,float alt_error,float aspd_error,float xtrack_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[26];
	_mav_put_float(buf, 0, nav_roll);
	_mav_put_float(buf, 4, nav_pitch);
	_mav_put_int16_t(buf, 8, nav_bearing);
	_mav_put_int16_t(buf, 10, target_bearing);
	_mav_put_uint16_t(buf, 12, wp_dist);
	_mav_put_float(buf, 14, alt_error);
	_mav_put_float(buf, 18, aspd_error);
	_mav_put_float(buf, 22, xtrack_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 26);
#else
	mavlink_nav_controller_output_t packet;
	packet.nav_roll = nav_roll;
	packet.nav_pitch = nav_pitch;
	packet.nav_bearing = nav_bearing;
	packet.target_bearing = target_bearing;
	packet.wp_dist = wp_dist;
	packet.alt_error = alt_error;
	packet.aspd_error = aspd_error;
	packet.xtrack_error = xtrack_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 26);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 26);
}

/**
 * @brief Encode a nav_controller_output struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param nav_controller_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nav_controller_output_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nav_controller_output_t* nav_controller_output)
{
	return mavlink_msg_nav_controller_output_pack(system_id, component_id, msg, nav_controller_output->nav_roll, nav_controller_output->nav_pitch, nav_controller_output->nav_bearing, nav_controller_output->target_bearing, nav_controller_output->wp_dist, nav_controller_output->alt_error, nav_controller_output->aspd_error, nav_controller_output->xtrack_error);
}

/**
 * @brief Send a nav_controller_output message
 * @param chan MAVLink channel to send the message
 *
 * @param nav_roll Current desired roll in degrees
 * @param nav_pitch Current desired pitch in degrees
 * @param nav_bearing Current desired heading in degrees
 * @param target_bearing Bearing to current waypoint/target in degrees
 * @param wp_dist Distance to active waypoint in meters
 * @param alt_error Current altitude error in meters
 * @param aspd_error Current airspeed error in meters/second
 * @param xtrack_error Current crosstrack error on x-y plane in meters
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nav_controller_output_send(mavlink_channel_t chan, float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[26];
	_mav_put_float(buf, 0, nav_roll);
	_mav_put_float(buf, 4, nav_pitch);
	_mav_put_int16_t(buf, 8, nav_bearing);
	_mav_put_int16_t(buf, 10, target_bearing);
	_mav_put_uint16_t(buf, 12, wp_dist);
	_mav_put_float(buf, 14, alt_error);
	_mav_put_float(buf, 18, aspd_error);
	_mav_put_float(buf, 22, xtrack_error);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, buf, 26);
#else
	mavlink_nav_controller_output_t packet;
	packet.nav_roll = nav_roll;
	packet.nav_pitch = nav_pitch;
	packet.nav_bearing = nav_bearing;
	packet.target_bearing = target_bearing;
	packet.wp_dist = wp_dist;
	packet.alt_error = alt_error;
	packet.aspd_error = aspd_error;
	packet.xtrack_error = xtrack_error;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, (const char *)&packet, 26);
#endif
}

#endif

// MESSAGE NAV_CONTROLLER_OUTPUT UNPACKING


/**
 * @brief Get field nav_roll from nav_controller_output message
 *
 * @return Current desired roll in degrees
 */
static inline float mavlink_msg_nav_controller_output_get_nav_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field nav_pitch from nav_controller_output message
 *
 * @return Current desired pitch in degrees
 */
static inline float mavlink_msg_nav_controller_output_get_nav_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field nav_bearing from nav_controller_output message
 *
 * @return Current desired heading in degrees
 */
static inline int16_t mavlink_msg_nav_controller_output_get_nav_bearing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field target_bearing from nav_controller_output message
 *
 * @return Bearing to current waypoint/target in degrees
 */
static inline int16_t mavlink_msg_nav_controller_output_get_target_bearing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field wp_dist from nav_controller_output message
 *
 * @return Distance to active waypoint in meters
 */
static inline uint16_t mavlink_msg_nav_controller_output_get_wp_dist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field alt_error from nav_controller_output message
 *
 * @return Current altitude error in meters
 */
static inline float mavlink_msg_nav_controller_output_get_alt_error(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  14);
}

/**
 * @brief Get field aspd_error from nav_controller_output message
 *
 * @return Current airspeed error in meters/second
 */
static inline float mavlink_msg_nav_controller_output_get_aspd_error(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  18);
}

/**
 * @brief Get field xtrack_error from nav_controller_output message
 *
 * @return Current crosstrack error on x-y plane in meters
 */
static inline float mavlink_msg_nav_controller_output_get_xtrack_error(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  22);
}

/**
 * @brief Decode a nav_controller_output message into a struct
 *
 * @param msg The message to decode
 * @param nav_controller_output C-struct to decode the message contents into
 */
static inline void mavlink_msg_nav_controller_output_decode(const mavlink_message_t* msg, mavlink_nav_controller_output_t* nav_controller_output)
{
#if MAVLINK_NEED_BYTE_SWAP
	nav_controller_output->nav_roll = mavlink_msg_nav_controller_output_get_nav_roll(msg);
	nav_controller_output->nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(msg);
	nav_controller_output->nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(msg);
	nav_controller_output->target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(msg);
	nav_controller_output->wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(msg);
	nav_controller_output->alt_error = mavlink_msg_nav_controller_output_get_alt_error(msg);
	nav_controller_output->aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(msg);
	nav_controller_output->xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(msg);
#else
	memcpy(nav_controller_output, _MAV_PAYLOAD(msg), 26);
#endif
}
