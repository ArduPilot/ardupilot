// MESSAGE EKF_STATUS_REPORT PACKING

#define MAVLINK_MSG_ID_EKF_STATUS_REPORT 193

typedef struct __mavlink_ekf_status_report_t
{
 float velocity_variance; /*< Velocity variance*/
 float pos_horiz_variance; /*< Horizontal Position variance*/
 float pos_vert_variance; /*< Vertical Position variance*/
 float compass_variance; /*< Compass variance*/
 float terrain_alt_variance; /*< Terrain Altitude variance*/
 uint16_t flags; /*< Flags*/
} mavlink_ekf_status_report_t;

#define MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN 22
#define MAVLINK_MSG_ID_193_LEN 22

#define MAVLINK_MSG_ID_EKF_STATUS_REPORT_CRC 71
#define MAVLINK_MSG_ID_193_CRC 71



#define MAVLINK_MESSAGE_INFO_EKF_STATUS_REPORT { \
	"EKF_STATUS_REPORT", \
	6, \
	{  { "velocity_variance", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ekf_status_report_t, velocity_variance) }, \
         { "pos_horiz_variance", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ekf_status_report_t, pos_horiz_variance) }, \
         { "pos_vert_variance", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ekf_status_report_t, pos_vert_variance) }, \
         { "compass_variance", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ekf_status_report_t, compass_variance) }, \
         { "terrain_alt_variance", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ekf_status_report_t, terrain_alt_variance) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_ekf_status_report_t, flags) }, \
         } \
}


/**
 * @brief Pack a ekf_status_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param flags Flags
 * @param velocity_variance Velocity variance
 * @param pos_horiz_variance Horizontal Position variance
 * @param pos_vert_variance Vertical Position variance
 * @param compass_variance Compass variance
 * @param terrain_alt_variance Terrain Altitude variance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ekf_status_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN];
	_mav_put_float(buf, 0, velocity_variance);
	_mav_put_float(buf, 4, pos_horiz_variance);
	_mav_put_float(buf, 8, pos_vert_variance);
	_mav_put_float(buf, 12, compass_variance);
	_mav_put_float(buf, 16, terrain_alt_variance);
	_mav_put_uint16_t(buf, 20, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#else
	mavlink_ekf_status_report_t packet;
	packet.velocity_variance = velocity_variance;
	packet.pos_horiz_variance = pos_horiz_variance;
	packet.pos_vert_variance = pos_vert_variance;
	packet.compass_variance = compass_variance;
	packet.terrain_alt_variance = terrain_alt_variance;
	packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EKF_STATUS_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN, MAVLINK_MSG_ID_EKF_STATUS_REPORT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#endif
}

/**
 * @brief Pack a ekf_status_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flags Flags
 * @param velocity_variance Velocity variance
 * @param pos_horiz_variance Horizontal Position variance
 * @param pos_vert_variance Vertical Position variance
 * @param compass_variance Compass variance
 * @param terrain_alt_variance Terrain Altitude variance
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ekf_status_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t flags,float velocity_variance,float pos_horiz_variance,float pos_vert_variance,float compass_variance,float terrain_alt_variance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN];
	_mav_put_float(buf, 0, velocity_variance);
	_mav_put_float(buf, 4, pos_horiz_variance);
	_mav_put_float(buf, 8, pos_vert_variance);
	_mav_put_float(buf, 12, compass_variance);
	_mav_put_float(buf, 16, terrain_alt_variance);
	_mav_put_uint16_t(buf, 20, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#else
	mavlink_ekf_status_report_t packet;
	packet.velocity_variance = velocity_variance;
	packet.pos_horiz_variance = pos_horiz_variance;
	packet.pos_vert_variance = pos_vert_variance;
	packet.compass_variance = compass_variance;
	packet.terrain_alt_variance = terrain_alt_variance;
	packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EKF_STATUS_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN, MAVLINK_MSG_ID_EKF_STATUS_REPORT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#endif
}

/**
 * @brief Encode a ekf_status_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ekf_status_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ekf_status_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ekf_status_report_t* ekf_status_report)
{
	return mavlink_msg_ekf_status_report_pack(system_id, component_id, msg, ekf_status_report->flags, ekf_status_report->velocity_variance, ekf_status_report->pos_horiz_variance, ekf_status_report->pos_vert_variance, ekf_status_report->compass_variance, ekf_status_report->terrain_alt_variance);
}

/**
 * @brief Encode a ekf_status_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ekf_status_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ekf_status_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ekf_status_report_t* ekf_status_report)
{
	return mavlink_msg_ekf_status_report_pack_chan(system_id, component_id, chan, msg, ekf_status_report->flags, ekf_status_report->velocity_variance, ekf_status_report->pos_horiz_variance, ekf_status_report->pos_vert_variance, ekf_status_report->compass_variance, ekf_status_report->terrain_alt_variance);
}

/**
 * @brief Send a ekf_status_report message
 * @param chan MAVLink channel to send the message
 *
 * @param flags Flags
 * @param velocity_variance Velocity variance
 * @param pos_horiz_variance Horizontal Position variance
 * @param pos_vert_variance Vertical Position variance
 * @param compass_variance Compass variance
 * @param terrain_alt_variance Terrain Altitude variance
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ekf_status_report_send(mavlink_channel_t chan, uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN];
	_mav_put_float(buf, 0, velocity_variance);
	_mav_put_float(buf, 4, pos_horiz_variance);
	_mav_put_float(buf, 8, pos_vert_variance);
	_mav_put_float(buf, 12, compass_variance);
	_mav_put_float(buf, 16, terrain_alt_variance);
	_mav_put_uint16_t(buf, 20, flags);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATUS_REPORT, buf, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN, MAVLINK_MSG_ID_EKF_STATUS_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATUS_REPORT, buf, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#endif
#else
	mavlink_ekf_status_report_t packet;
	packet.velocity_variance = velocity_variance;
	packet.pos_horiz_variance = pos_horiz_variance;
	packet.pos_vert_variance = pos_vert_variance;
	packet.compass_variance = compass_variance;
	packet.terrain_alt_variance = terrain_alt_variance;
	packet.flags = flags;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATUS_REPORT, (const char *)&packet, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN, MAVLINK_MSG_ID_EKF_STATUS_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATUS_REPORT, (const char *)&packet, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ekf_status_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t flags, float velocity_variance, float pos_horiz_variance, float pos_vert_variance, float compass_variance, float terrain_alt_variance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, velocity_variance);
	_mav_put_float(buf, 4, pos_horiz_variance);
	_mav_put_float(buf, 8, pos_vert_variance);
	_mav_put_float(buf, 12, compass_variance);
	_mav_put_float(buf, 16, terrain_alt_variance);
	_mav_put_uint16_t(buf, 20, flags);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATUS_REPORT, buf, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN, MAVLINK_MSG_ID_EKF_STATUS_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATUS_REPORT, buf, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#endif
#else
	mavlink_ekf_status_report_t *packet = (mavlink_ekf_status_report_t *)msgbuf;
	packet->velocity_variance = velocity_variance;
	packet->pos_horiz_variance = pos_horiz_variance;
	packet->pos_vert_variance = pos_vert_variance;
	packet->compass_variance = compass_variance;
	packet->terrain_alt_variance = terrain_alt_variance;
	packet->flags = flags;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATUS_REPORT, (const char *)packet, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN, MAVLINK_MSG_ID_EKF_STATUS_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EKF_STATUS_REPORT, (const char *)packet, MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE EKF_STATUS_REPORT UNPACKING


/**
 * @brief Get field flags from ekf_status_report message
 *
 * @return Flags
 */
static inline uint16_t mavlink_msg_ekf_status_report_get_flags(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field velocity_variance from ekf_status_report message
 *
 * @return Velocity variance
 */
static inline float mavlink_msg_ekf_status_report_get_velocity_variance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pos_horiz_variance from ekf_status_report message
 *
 * @return Horizontal Position variance
 */
static inline float mavlink_msg_ekf_status_report_get_pos_horiz_variance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pos_vert_variance from ekf_status_report message
 *
 * @return Vertical Position variance
 */
static inline float mavlink_msg_ekf_status_report_get_pos_vert_variance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field compass_variance from ekf_status_report message
 *
 * @return Compass variance
 */
static inline float mavlink_msg_ekf_status_report_get_compass_variance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field terrain_alt_variance from ekf_status_report message
 *
 * @return Terrain Altitude variance
 */
static inline float mavlink_msg_ekf_status_report_get_terrain_alt_variance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a ekf_status_report message into a struct
 *
 * @param msg The message to decode
 * @param ekf_status_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_ekf_status_report_decode(const mavlink_message_t* msg, mavlink_ekf_status_report_t* ekf_status_report)
{
#if MAVLINK_NEED_BYTE_SWAP
	ekf_status_report->velocity_variance = mavlink_msg_ekf_status_report_get_velocity_variance(msg);
	ekf_status_report->pos_horiz_variance = mavlink_msg_ekf_status_report_get_pos_horiz_variance(msg);
	ekf_status_report->pos_vert_variance = mavlink_msg_ekf_status_report_get_pos_vert_variance(msg);
	ekf_status_report->compass_variance = mavlink_msg_ekf_status_report_get_compass_variance(msg);
	ekf_status_report->terrain_alt_variance = mavlink_msg_ekf_status_report_get_terrain_alt_variance(msg);
	ekf_status_report->flags = mavlink_msg_ekf_status_report_get_flags(msg);
#else
	memcpy(ekf_status_report, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN);
#endif
}
