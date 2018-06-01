// MESSAGE NAV_CONTROLLER_OUTPUT PACKING

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT 62

typedef struct __mavlink_nav_controller_output_t
{
 float nav_roll; /*< Current desired roll in degrees*/
 float nav_pitch; /*< Current desired pitch in degrees*/
 float alt_error; /*< Current altitude error in meters*/
 float aspd_error; /*< Current airspeed error in meters/second*/
 float xtrack_error; /*< Current crosstrack error on x-y plane in meters*/
 int16_t nav_bearing; /*< Current desired heading in degrees*/
 int16_t target_bearing; /*< Bearing to current MISSION/target in degrees*/
 uint16_t wp_dist; /*< Distance to active MISSION in meters*/
} mavlink_nav_controller_output_t;

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN 26
#define MAVLINK_MSG_ID_62_LEN 26

#define MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC 183
#define MAVLINK_MSG_ID_62_CRC 183



#define MAVLINK_MESSAGE_INFO_NAV_CONTROLLER_OUTPUT { \
	"NAV_CONTROLLER_OUTPUT", \
	8, \
	{  { "nav_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_nav_controller_output_t, nav_roll) }, \
         { "nav_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_nav_controller_output_t, nav_pitch) }, \
         { "alt_error", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_nav_controller_output_t, alt_error) }, \
         { "aspd_error", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_nav_controller_output_t, aspd_error) }, \
         { "xtrack_error", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_nav_controller_output_t, xtrack_error) }, \
         { "nav_bearing", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_nav_controller_output_t, nav_bearing) }, \
         { "target_bearing", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_nav_controller_output_t, target_bearing) }, \
         { "wp_dist", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_nav_controller_output_t, wp_dist) }, \
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
 * @param target_bearing Bearing to current MISSION/target in degrees
 * @param wp_dist Distance to active MISSION in meters
 * @param alt_error Current altitude error in meters
 * @param aspd_error Current airspeed error in meters/second
 * @param xtrack_error Current crosstrack error on x-y plane in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nav_controller_output_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN];
	_mav_put_float(buf, 0, nav_roll);
	_mav_put_float(buf, 4, nav_pitch);
	_mav_put_float(buf, 8, alt_error);
	_mav_put_float(buf, 12, aspd_error);
	_mav_put_float(buf, 16, xtrack_error);
	_mav_put_int16_t(buf, 20, nav_bearing);
	_mav_put_int16_t(buf, 22, target_bearing);
	_mav_put_uint16_t(buf, 24, wp_dist);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#else
	mavlink_nav_controller_output_t packet;
	packet.nav_roll = nav_roll;
	packet.nav_pitch = nav_pitch;
	packet.alt_error = alt_error;
	packet.aspd_error = aspd_error;
	packet.xtrack_error = xtrack_error;
	packet.nav_bearing = nav_bearing;
	packet.target_bearing = target_bearing;
	packet.wp_dist = wp_dist;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#endif
}

/**
 * @brief Pack a nav_controller_output message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nav_roll Current desired roll in degrees
 * @param nav_pitch Current desired pitch in degrees
 * @param nav_bearing Current desired heading in degrees
 * @param target_bearing Bearing to current MISSION/target in degrees
 * @param wp_dist Distance to active MISSION in meters
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
	char buf[MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN];
	_mav_put_float(buf, 0, nav_roll);
	_mav_put_float(buf, 4, nav_pitch);
	_mav_put_float(buf, 8, alt_error);
	_mav_put_float(buf, 12, aspd_error);
	_mav_put_float(buf, 16, xtrack_error);
	_mav_put_int16_t(buf, 20, nav_bearing);
	_mav_put_int16_t(buf, 22, target_bearing);
	_mav_put_uint16_t(buf, 24, wp_dist);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#else
	mavlink_nav_controller_output_t packet;
	packet.nav_roll = nav_roll;
	packet.nav_pitch = nav_pitch;
	packet.alt_error = alt_error;
	packet.aspd_error = aspd_error;
	packet.xtrack_error = xtrack_error;
	packet.nav_bearing = nav_bearing;
	packet.target_bearing = target_bearing;
	packet.wp_dist = wp_dist;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#endif
}

/**
 * @brief Encode a nav_controller_output struct
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
 * @brief Encode a nav_controller_output struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nav_controller_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nav_controller_output_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_nav_controller_output_t* nav_controller_output)
{
	return mavlink_msg_nav_controller_output_pack_chan(system_id, component_id, chan, msg, nav_controller_output->nav_roll, nav_controller_output->nav_pitch, nav_controller_output->nav_bearing, nav_controller_output->target_bearing, nav_controller_output->wp_dist, nav_controller_output->alt_error, nav_controller_output->aspd_error, nav_controller_output->xtrack_error);
}

/**
 * @brief Send a nav_controller_output message
 * @param chan MAVLink channel to send the message
 *
 * @param nav_roll Current desired roll in degrees
 * @param nav_pitch Current desired pitch in degrees
 * @param nav_bearing Current desired heading in degrees
 * @param target_bearing Bearing to current MISSION/target in degrees
 * @param wp_dist Distance to active MISSION in meters
 * @param alt_error Current altitude error in meters
 * @param aspd_error Current airspeed error in meters/second
 * @param xtrack_error Current crosstrack error on x-y plane in meters
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nav_controller_output_send(mavlink_channel_t chan, float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN];
	_mav_put_float(buf, 0, nav_roll);
	_mav_put_float(buf, 4, nav_pitch);
	_mav_put_float(buf, 8, alt_error);
	_mav_put_float(buf, 12, aspd_error);
	_mav_put_float(buf, 16, xtrack_error);
	_mav_put_int16_t(buf, 20, nav_bearing);
	_mav_put_int16_t(buf, 22, target_bearing);
	_mav_put_uint16_t(buf, 24, wp_dist);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, buf, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, buf, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#endif
#else
	mavlink_nav_controller_output_t packet;
	packet.nav_roll = nav_roll;
	packet.nav_pitch = nav_pitch;
	packet.alt_error = alt_error;
	packet.aspd_error = aspd_error;
	packet.xtrack_error = xtrack_error;
	packet.nav_bearing = nav_bearing;
	packet.target_bearing = target_bearing;
	packet.wp_dist = wp_dist;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, (const char *)&packet, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, (const char *)&packet, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_nav_controller_output_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float nav_roll, float nav_pitch, int16_t nav_bearing, int16_t target_bearing, uint16_t wp_dist, float alt_error, float aspd_error, float xtrack_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, nav_roll);
	_mav_put_float(buf, 4, nav_pitch);
	_mav_put_float(buf, 8, alt_error);
	_mav_put_float(buf, 12, aspd_error);
	_mav_put_float(buf, 16, xtrack_error);
	_mav_put_int16_t(buf, 20, nav_bearing);
	_mav_put_int16_t(buf, 22, target_bearing);
	_mav_put_uint16_t(buf, 24, wp_dist);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, buf, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, buf, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#endif
#else
	mavlink_nav_controller_output_t *packet = (mavlink_nav_controller_output_t *)msgbuf;
	packet->nav_roll = nav_roll;
	packet->nav_pitch = nav_pitch;
	packet->alt_error = alt_error;
	packet->aspd_error = aspd_error;
	packet->xtrack_error = xtrack_error;
	packet->nav_bearing = nav_bearing;
	packet->target_bearing = target_bearing;
	packet->wp_dist = wp_dist;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, (const char *)packet, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, (const char *)packet, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#endif
#endif
}
#endif

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
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field target_bearing from nav_controller_output message
 *
 * @return Bearing to current MISSION/target in degrees
 */
static inline int16_t mavlink_msg_nav_controller_output_get_target_bearing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field wp_dist from nav_controller_output message
 *
 * @return Distance to active MISSION in meters
 */
static inline uint16_t mavlink_msg_nav_controller_output_get_wp_dist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field alt_error from nav_controller_output message
 *
 * @return Current altitude error in meters
 */
static inline float mavlink_msg_nav_controller_output_get_alt_error(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field aspd_error from nav_controller_output message
 *
 * @return Current airspeed error in meters/second
 */
static inline float mavlink_msg_nav_controller_output_get_aspd_error(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field xtrack_error from nav_controller_output message
 *
 * @return Current crosstrack error on x-y plane in meters
 */
static inline float mavlink_msg_nav_controller_output_get_xtrack_error(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
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
	nav_controller_output->alt_error = mavlink_msg_nav_controller_output_get_alt_error(msg);
	nav_controller_output->aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(msg);
	nav_controller_output->xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(msg);
	nav_controller_output->nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(msg);
	nav_controller_output->target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(msg);
	nav_controller_output->wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(msg);
#else
	memcpy(nav_controller_output, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN);
#endif
}
