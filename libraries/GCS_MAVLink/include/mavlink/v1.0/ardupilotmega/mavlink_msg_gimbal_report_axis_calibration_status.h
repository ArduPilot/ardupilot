// MESSAGE GIMBAL_REPORT_AXIS_CALIBRATION_STATUS PACKING

#define MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS 212

typedef struct __mavlink_gimbal_report_axis_calibration_status_t
{
 uint8_t yaw_requires_calibration; /*< Whether or not the yaw axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration*/
 uint8_t pitch_requires_calibration; /*< Whether or not the pitch axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration*/
 uint8_t roll_requires_calibration; /*< Whether or not the roll axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration*/
} mavlink_gimbal_report_axis_calibration_status_t;

#define MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN 3
#define MAVLINK_MSG_ID_212_LEN 3

#define MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_CRC 235
#define MAVLINK_MSG_ID_212_CRC 235



#define MAVLINK_MESSAGE_INFO_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS { \
	"GIMBAL_REPORT_AXIS_CALIBRATION_STATUS", \
	3, \
	{  { "yaw_requires_calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gimbal_report_axis_calibration_status_t, yaw_requires_calibration) }, \
         { "pitch_requires_calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gimbal_report_axis_calibration_status_t, pitch_requires_calibration) }, \
         { "roll_requires_calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gimbal_report_axis_calibration_status_t, roll_requires_calibration) }, \
         } \
}


/**
 * @brief Pack a gimbal_report_axis_calibration_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param yaw_requires_calibration Whether or not the yaw axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 * @param pitch_requires_calibration Whether or not the pitch axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 * @param roll_requires_calibration Whether or not the roll axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_report_axis_calibration_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t yaw_requires_calibration, uint8_t pitch_requires_calibration, uint8_t roll_requires_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN];
	_mav_put_uint8_t(buf, 0, yaw_requires_calibration);
	_mav_put_uint8_t(buf, 1, pitch_requires_calibration);
	_mav_put_uint8_t(buf, 2, roll_requires_calibration);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#else
	mavlink_gimbal_report_axis_calibration_status_t packet;
	packet.yaw_requires_calibration = yaw_requires_calibration;
	packet.pitch_requires_calibration = pitch_requires_calibration;
	packet.roll_requires_calibration = roll_requires_calibration;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#endif
}

/**
 * @brief Pack a gimbal_report_axis_calibration_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param yaw_requires_calibration Whether or not the yaw axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 * @param pitch_requires_calibration Whether or not the pitch axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 * @param roll_requires_calibration Whether or not the roll axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_report_axis_calibration_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t yaw_requires_calibration,uint8_t pitch_requires_calibration,uint8_t roll_requires_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN];
	_mav_put_uint8_t(buf, 0, yaw_requires_calibration);
	_mav_put_uint8_t(buf, 1, pitch_requires_calibration);
	_mav_put_uint8_t(buf, 2, roll_requires_calibration);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#else
	mavlink_gimbal_report_axis_calibration_status_t packet;
	packet.yaw_requires_calibration = yaw_requires_calibration;
	packet.pitch_requires_calibration = pitch_requires_calibration;
	packet.roll_requires_calibration = roll_requires_calibration;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#endif
}

/**
 * @brief Encode a gimbal_report_axis_calibration_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_report_axis_calibration_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_report_axis_calibration_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_report_axis_calibration_status_t* gimbal_report_axis_calibration_status)
{
	return mavlink_msg_gimbal_report_axis_calibration_status_pack(system_id, component_id, msg, gimbal_report_axis_calibration_status->yaw_requires_calibration, gimbal_report_axis_calibration_status->pitch_requires_calibration, gimbal_report_axis_calibration_status->roll_requires_calibration);
}

/**
 * @brief Encode a gimbal_report_axis_calibration_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_report_axis_calibration_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_report_axis_calibration_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_report_axis_calibration_status_t* gimbal_report_axis_calibration_status)
{
	return mavlink_msg_gimbal_report_axis_calibration_status_pack_chan(system_id, component_id, chan, msg, gimbal_report_axis_calibration_status->yaw_requires_calibration, gimbal_report_axis_calibration_status->pitch_requires_calibration, gimbal_report_axis_calibration_status->roll_requires_calibration);
}

/**
 * @brief Send a gimbal_report_axis_calibration_status message
 * @param chan MAVLink channel to send the message
 *
 * @param yaw_requires_calibration Whether or not the yaw axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 * @param pitch_requires_calibration Whether or not the pitch axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 * @param roll_requires_calibration Whether or not the roll axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_report_axis_calibration_status_send(mavlink_channel_t chan, uint8_t yaw_requires_calibration, uint8_t pitch_requires_calibration, uint8_t roll_requires_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN];
	_mav_put_uint8_t(buf, 0, yaw_requires_calibration);
	_mav_put_uint8_t(buf, 1, pitch_requires_calibration);
	_mav_put_uint8_t(buf, 2, roll_requires_calibration);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS, buf, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS, buf, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#endif
#else
	mavlink_gimbal_report_axis_calibration_status_t packet;
	packet.yaw_requires_calibration = yaw_requires_calibration;
	packet.pitch_requires_calibration = pitch_requires_calibration;
	packet.roll_requires_calibration = roll_requires_calibration;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_report_axis_calibration_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t yaw_requires_calibration, uint8_t pitch_requires_calibration, uint8_t roll_requires_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, yaw_requires_calibration);
	_mav_put_uint8_t(buf, 1, pitch_requires_calibration);
	_mav_put_uint8_t(buf, 2, roll_requires_calibration);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS, buf, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS, buf, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#endif
#else
	mavlink_gimbal_report_axis_calibration_status_t *packet = (mavlink_gimbal_report_axis_calibration_status_t *)msgbuf;
	packet->yaw_requires_calibration = yaw_requires_calibration;
	packet->pitch_requires_calibration = pitch_requires_calibration;
	packet->roll_requires_calibration = roll_requires_calibration;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GIMBAL_REPORT_AXIS_CALIBRATION_STATUS UNPACKING


/**
 * @brief Get field yaw_requires_calibration from gimbal_report_axis_calibration_status message
 *
 * @return Whether or not the yaw axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 */
static inline uint8_t mavlink_msg_gimbal_report_axis_calibration_status_get_yaw_requires_calibration(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field pitch_requires_calibration from gimbal_report_axis_calibration_status message
 *
 * @return Whether or not the pitch axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 */
static inline uint8_t mavlink_msg_gimbal_report_axis_calibration_status_get_pitch_requires_calibration(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field roll_requires_calibration from gimbal_report_axis_calibration_status message
 *
 * @return Whether or not the roll axis requires calibration, see GIMBAL_AXIS_CALIBRATION_REQUIRED enumeration
 */
static inline uint8_t mavlink_msg_gimbal_report_axis_calibration_status_get_roll_requires_calibration(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a gimbal_report_axis_calibration_status message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_report_axis_calibration_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_report_axis_calibration_status_decode(const mavlink_message_t* msg, mavlink_gimbal_report_axis_calibration_status_t* gimbal_report_axis_calibration_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	gimbal_report_axis_calibration_status->yaw_requires_calibration = mavlink_msg_gimbal_report_axis_calibration_status_get_yaw_requires_calibration(msg);
	gimbal_report_axis_calibration_status->pitch_requires_calibration = mavlink_msg_gimbal_report_axis_calibration_status_get_pitch_requires_calibration(msg);
	gimbal_report_axis_calibration_status->roll_requires_calibration = mavlink_msg_gimbal_report_axis_calibration_status_get_roll_requires_calibration(msg);
#else
	memcpy(gimbal_report_axis_calibration_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GIMBAL_REPORT_AXIS_CALIBRATION_STATUS_LEN);
#endif
}
