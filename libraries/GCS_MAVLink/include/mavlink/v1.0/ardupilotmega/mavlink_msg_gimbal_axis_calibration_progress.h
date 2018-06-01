// MESSAGE GIMBAL_AXIS_CALIBRATION_PROGRESS PACKING

#define MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS 203

typedef struct __mavlink_gimbal_axis_calibration_progress_t
{
 uint8_t calibration_axis; /*< Which gimbal axis we're reporting calibration progress for*/
 uint8_t calibration_progress; /*< The current calibration progress for this axis, 0x64=100%*/
 uint8_t calibration_status; /*< The status of the running calibration*/
} mavlink_gimbal_axis_calibration_progress_t;

#define MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN 3
#define MAVLINK_MSG_ID_203_LEN 3

#define MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_CRC 128
#define MAVLINK_MSG_ID_203_CRC 128



#define MAVLINK_MESSAGE_INFO_GIMBAL_AXIS_CALIBRATION_PROGRESS { \
	"GIMBAL_AXIS_CALIBRATION_PROGRESS", \
	3, \
	{  { "calibration_axis", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gimbal_axis_calibration_progress_t, calibration_axis) }, \
         { "calibration_progress", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gimbal_axis_calibration_progress_t, calibration_progress) }, \
         { "calibration_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gimbal_axis_calibration_progress_t, calibration_status) }, \
         } \
}


/**
 * @brief Pack a gimbal_axis_calibration_progress message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param calibration_axis Which gimbal axis we're reporting calibration progress for
 * @param calibration_progress The current calibration progress for this axis, 0x64=100%
 * @param calibration_status The status of the running calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_axis_calibration_progress_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t calibration_axis, uint8_t calibration_progress, uint8_t calibration_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN];
	_mav_put_uint8_t(buf, 0, calibration_axis);
	_mav_put_uint8_t(buf, 1, calibration_progress);
	_mav_put_uint8_t(buf, 2, calibration_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#else
	mavlink_gimbal_axis_calibration_progress_t packet;
	packet.calibration_axis = calibration_axis;
	packet.calibration_progress = calibration_progress;
	packet.calibration_status = calibration_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#endif
}

/**
 * @brief Pack a gimbal_axis_calibration_progress message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param calibration_axis Which gimbal axis we're reporting calibration progress for
 * @param calibration_progress The current calibration progress for this axis, 0x64=100%
 * @param calibration_status The status of the running calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_axis_calibration_progress_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t calibration_axis,uint8_t calibration_progress,uint8_t calibration_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN];
	_mav_put_uint8_t(buf, 0, calibration_axis);
	_mav_put_uint8_t(buf, 1, calibration_progress);
	_mav_put_uint8_t(buf, 2, calibration_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#else
	mavlink_gimbal_axis_calibration_progress_t packet;
	packet.calibration_axis = calibration_axis;
	packet.calibration_progress = calibration_progress;
	packet.calibration_status = calibration_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#endif
}

/**
 * @brief Encode a gimbal_axis_calibration_progress struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_axis_calibration_progress C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_axis_calibration_progress_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_axis_calibration_progress_t* gimbal_axis_calibration_progress)
{
	return mavlink_msg_gimbal_axis_calibration_progress_pack(system_id, component_id, msg, gimbal_axis_calibration_progress->calibration_axis, gimbal_axis_calibration_progress->calibration_progress, gimbal_axis_calibration_progress->calibration_status);
}

/**
 * @brief Encode a gimbal_axis_calibration_progress struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_axis_calibration_progress C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_axis_calibration_progress_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_axis_calibration_progress_t* gimbal_axis_calibration_progress)
{
	return mavlink_msg_gimbal_axis_calibration_progress_pack_chan(system_id, component_id, chan, msg, gimbal_axis_calibration_progress->calibration_axis, gimbal_axis_calibration_progress->calibration_progress, gimbal_axis_calibration_progress->calibration_status);
}

/**
 * @brief Send a gimbal_axis_calibration_progress message
 * @param chan MAVLink channel to send the message
 *
 * @param calibration_axis Which gimbal axis we're reporting calibration progress for
 * @param calibration_progress The current calibration progress for this axis, 0x64=100%
 * @param calibration_status The status of the running calibration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_axis_calibration_progress_send(mavlink_channel_t chan, uint8_t calibration_axis, uint8_t calibration_progress, uint8_t calibration_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN];
	_mav_put_uint8_t(buf, 0, calibration_axis);
	_mav_put_uint8_t(buf, 1, calibration_progress);
	_mav_put_uint8_t(buf, 2, calibration_status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS, buf, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS, buf, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#endif
#else
	mavlink_gimbal_axis_calibration_progress_t packet;
	packet.calibration_axis = calibration_axis;
	packet.calibration_progress = calibration_progress;
	packet.calibration_status = calibration_status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_axis_calibration_progress_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t calibration_axis, uint8_t calibration_progress, uint8_t calibration_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, calibration_axis);
	_mav_put_uint8_t(buf, 1, calibration_progress);
	_mav_put_uint8_t(buf, 2, calibration_status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS, buf, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS, buf, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#endif
#else
	mavlink_gimbal_axis_calibration_progress_t *packet = (mavlink_gimbal_axis_calibration_progress_t *)msgbuf;
	packet->calibration_axis = calibration_axis;
	packet->calibration_progress = calibration_progress;
	packet->calibration_status = calibration_status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GIMBAL_AXIS_CALIBRATION_PROGRESS UNPACKING


/**
 * @brief Get field calibration_axis from gimbal_axis_calibration_progress message
 *
 * @return Which gimbal axis we're reporting calibration progress for
 */
static inline uint8_t mavlink_msg_gimbal_axis_calibration_progress_get_calibration_axis(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field calibration_progress from gimbal_axis_calibration_progress message
 *
 * @return The current calibration progress for this axis, 0x64=100%
 */
static inline uint8_t mavlink_msg_gimbal_axis_calibration_progress_get_calibration_progress(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field calibration_status from gimbal_axis_calibration_progress message
 *
 * @return The status of the running calibration
 */
static inline uint8_t mavlink_msg_gimbal_axis_calibration_progress_get_calibration_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a gimbal_axis_calibration_progress message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_axis_calibration_progress C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_axis_calibration_progress_decode(const mavlink_message_t* msg, mavlink_gimbal_axis_calibration_progress_t* gimbal_axis_calibration_progress)
{
#if MAVLINK_NEED_BYTE_SWAP
	gimbal_axis_calibration_progress->calibration_axis = mavlink_msg_gimbal_axis_calibration_progress_get_calibration_axis(msg);
	gimbal_axis_calibration_progress->calibration_progress = mavlink_msg_gimbal_axis_calibration_progress_get_calibration_progress(msg);
	gimbal_axis_calibration_progress->calibration_status = mavlink_msg_gimbal_axis_calibration_progress_get_calibration_status(msg);
#else
	memcpy(gimbal_axis_calibration_progress, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS_LEN);
#endif
}
