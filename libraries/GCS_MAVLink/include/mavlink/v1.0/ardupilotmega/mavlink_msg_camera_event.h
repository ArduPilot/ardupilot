// MESSAGE CAMERA_EVENT PACKING

#define MAVLINK_MSG_ID_CAMERA_EVENT 179

typedef struct __mavlink_camera_event_t
{
 uint64_t time_usec; ///< Image timestamp (microseconds since UNIX epoch, according to camera clock)
 float p1; ///< Parameter 1 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 float p2; ///< Parameter 2 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 float p3; ///< Parameter 3 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 float p4; ///< Parameter 4 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 uint16_t img_idx; ///< Image index
 uint8_t target_system; ///< System ID
 uint8_t cam_idx; ///< Camera ID
 uint8_t event_id; ///< See CAMERA_EVENT_TYPES enum for definition of the bitmask
} mavlink_camera_event_t;

#define MAVLINK_MSG_ID_CAMERA_EVENT_LEN 29
#define MAVLINK_MSG_ID_179_LEN 29

#define MAVLINK_MSG_ID_CAMERA_EVENT_CRC 206
#define MAVLINK_MSG_ID_179_CRC 206



#define MAVLINK_MESSAGE_INFO_CAMERA_EVENT { \
	"CAMERA_EVENT", \
	9, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_camera_event_t, time_usec) }, \
         { "p1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_camera_event_t, p1) }, \
         { "p2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_event_t, p2) }, \
         { "p3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_event_t, p3) }, \
         { "p4", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_event_t, p4) }, \
         { "img_idx", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_camera_event_t, img_idx) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_camera_event_t, target_system) }, \
         { "cam_idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_camera_event_t, cam_idx) }, \
         { "event_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_camera_event_t, event_id) }, \
         } \
}


/**
 * @brief Pack a camera_event message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Image timestamp (microseconds since UNIX epoch, according to camera clock)
 * @param target_system System ID
 * @param cam_idx Camera ID
 * @param img_idx Image index
 * @param event_id See CAMERA_EVENT_TYPES enum for definition of the bitmask
 * @param p1 Parameter 1 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @param p2 Parameter 2 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @param p3 Parameter 3 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @param p4 Parameter 4 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_event_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, uint8_t event_id, float p1, float p2, float p3, float p4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_EVENT_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, p1);
	_mav_put_float(buf, 12, p2);
	_mav_put_float(buf, 16, p3);
	_mav_put_float(buf, 20, p4);
	_mav_put_uint16_t(buf, 24, img_idx);
	_mav_put_uint8_t(buf, 26, target_system);
	_mav_put_uint8_t(buf, 27, cam_idx);
	_mav_put_uint8_t(buf, 28, event_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#else
	mavlink_camera_event_t packet;
	packet.time_usec = time_usec;
	packet.p1 = p1;
	packet.p2 = p2;
	packet.p3 = p3;
	packet.p4 = p4;
	packet.img_idx = img_idx;
	packet.target_system = target_system;
	packet.cam_idx = cam_idx;
	packet.event_id = event_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CAMERA_EVENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_EVENT_LEN, MAVLINK_MSG_ID_CAMERA_EVENT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#endif
}

/**
 * @brief Pack a camera_event message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Image timestamp (microseconds since UNIX epoch, according to camera clock)
 * @param target_system System ID
 * @param cam_idx Camera ID
 * @param img_idx Image index
 * @param event_id See CAMERA_EVENT_TYPES enum for definition of the bitmask
 * @param p1 Parameter 1 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @param p2 Parameter 2 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @param p3 Parameter 3 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @param p4 Parameter 4 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_event_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t target_system,uint8_t cam_idx,uint16_t img_idx,uint8_t event_id,float p1,float p2,float p3,float p4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_EVENT_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, p1);
	_mav_put_float(buf, 12, p2);
	_mav_put_float(buf, 16, p3);
	_mav_put_float(buf, 20, p4);
	_mav_put_uint16_t(buf, 24, img_idx);
	_mav_put_uint8_t(buf, 26, target_system);
	_mav_put_uint8_t(buf, 27, cam_idx);
	_mav_put_uint8_t(buf, 28, event_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#else
	mavlink_camera_event_t packet;
	packet.time_usec = time_usec;
	packet.p1 = p1;
	packet.p2 = p2;
	packet.p3 = p3;
	packet.p4 = p4;
	packet.img_idx = img_idx;
	packet.target_system = target_system;
	packet.cam_idx = cam_idx;
	packet.event_id = event_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CAMERA_EVENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_EVENT_LEN, MAVLINK_MSG_ID_CAMERA_EVENT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#endif
}

/**
 * @brief Encode a camera_event struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_event_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_event_t* camera_event)
{
	return mavlink_msg_camera_event_pack(system_id, component_id, msg, camera_event->time_usec, camera_event->target_system, camera_event->cam_idx, camera_event->img_idx, camera_event->event_id, camera_event->p1, camera_event->p2, camera_event->p3, camera_event->p4);
}

/**
 * @brief Encode a camera_event struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_event_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_event_t* camera_event)
{
	return mavlink_msg_camera_event_pack_chan(system_id, component_id, chan, msg, camera_event->time_usec, camera_event->target_system, camera_event->cam_idx, camera_event->img_idx, camera_event->event_id, camera_event->p1, camera_event->p2, camera_event->p3, camera_event->p4);
}

/**
 * @brief Send a camera_event message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Image timestamp (microseconds since UNIX epoch, according to camera clock)
 * @param target_system System ID
 * @param cam_idx Camera ID
 * @param img_idx Image index
 * @param event_id See CAMERA_EVENT_TYPES enum for definition of the bitmask
 * @param p1 Parameter 1 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @param p2 Parameter 2 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @param p3 Parameter 3 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 * @param p4 Parameter 4 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_event_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, uint8_t event_id, float p1, float p2, float p3, float p4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_EVENT_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, p1);
	_mav_put_float(buf, 12, p2);
	_mav_put_float(buf, 16, p3);
	_mav_put_float(buf, 20, p4);
	_mav_put_uint16_t(buf, 24, img_idx);
	_mav_put_uint8_t(buf, 26, target_system);
	_mav_put_uint8_t(buf, 27, cam_idx);
	_mav_put_uint8_t(buf, 28, event_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_EVENT, buf, MAVLINK_MSG_ID_CAMERA_EVENT_LEN, MAVLINK_MSG_ID_CAMERA_EVENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_EVENT, buf, MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#endif
#else
	mavlink_camera_event_t packet;
	packet.time_usec = time_usec;
	packet.p1 = p1;
	packet.p2 = p2;
	packet.p3 = p3;
	packet.p4 = p4;
	packet.img_idx = img_idx;
	packet.target_system = target_system;
	packet.cam_idx = cam_idx;
	packet.event_id = event_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_EVENT, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_EVENT_LEN, MAVLINK_MSG_ID_CAMERA_EVENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_EVENT, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_CAMERA_EVENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_camera_event_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t target_system, uint8_t cam_idx, uint16_t img_idx, uint8_t event_id, float p1, float p2, float p3, float p4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, p1);
	_mav_put_float(buf, 12, p2);
	_mav_put_float(buf, 16, p3);
	_mav_put_float(buf, 20, p4);
	_mav_put_uint16_t(buf, 24, img_idx);
	_mav_put_uint8_t(buf, 26, target_system);
	_mav_put_uint8_t(buf, 27, cam_idx);
	_mav_put_uint8_t(buf, 28, event_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_EVENT, buf, MAVLINK_MSG_ID_CAMERA_EVENT_LEN, MAVLINK_MSG_ID_CAMERA_EVENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_EVENT, buf, MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#endif
#else
	mavlink_camera_event_t *packet = (mavlink_camera_event_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->p1 = p1;
	packet->p2 = p2;
	packet->p3 = p3;
	packet->p4 = p4;
	packet->img_idx = img_idx;
	packet->target_system = target_system;
	packet->cam_idx = cam_idx;
	packet->event_id = event_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_EVENT, (const char *)packet, MAVLINK_MSG_ID_CAMERA_EVENT_LEN, MAVLINK_MSG_ID_CAMERA_EVENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_EVENT, (const char *)packet, MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE CAMERA_EVENT UNPACKING


/**
 * @brief Get field time_usec from camera_event message
 *
 * @return Image timestamp (microseconds since UNIX epoch, according to camera clock)
 */
static inline uint64_t mavlink_msg_camera_event_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field target_system from camera_event message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_camera_event_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field cam_idx from camera_event message
 *
 * @return Camera ID
 */
static inline uint8_t mavlink_msg_camera_event_get_cam_idx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field img_idx from camera_event message
 *
 * @return Image index
 */
static inline uint16_t mavlink_msg_camera_event_get_img_idx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field event_id from camera_event message
 *
 * @return See CAMERA_EVENT_TYPES enum for definition of the bitmask
 */
static inline uint8_t mavlink_msg_camera_event_get_event_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field p1 from camera_event message
 *
 * @return Parameter 1 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 */
static inline float mavlink_msg_camera_event_get_p1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field p2 from camera_event message
 *
 * @return Parameter 2 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 */
static inline float mavlink_msg_camera_event_get_p2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field p3 from camera_event message
 *
 * @return Parameter 3 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 */
static inline float mavlink_msg_camera_event_get_p3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field p4 from camera_event message
 *
 * @return Parameter 4 (meaning depends on event, see CAMERA_EVENT_TYPES enum)
 */
static inline float mavlink_msg_camera_event_get_p4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a camera_event message into a struct
 *
 * @param msg The message to decode
 * @param camera_event C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_event_decode(const mavlink_message_t* msg, mavlink_camera_event_t* camera_event)
{
#if MAVLINK_NEED_BYTE_SWAP
	camera_event->time_usec = mavlink_msg_camera_event_get_time_usec(msg);
	camera_event->p1 = mavlink_msg_camera_event_get_p1(msg);
	camera_event->p2 = mavlink_msg_camera_event_get_p2(msg);
	camera_event->p3 = mavlink_msg_camera_event_get_p3(msg);
	camera_event->p4 = mavlink_msg_camera_event_get_p4(msg);
	camera_event->img_idx = mavlink_msg_camera_event_get_img_idx(msg);
	camera_event->target_system = mavlink_msg_camera_event_get_target_system(msg);
	camera_event->cam_idx = mavlink_msg_camera_event_get_cam_idx(msg);
	camera_event->event_id = mavlink_msg_camera_event_get_event_id(msg);
#else
	memcpy(camera_event, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CAMERA_EVENT_LEN);
#endif
}
