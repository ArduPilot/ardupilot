// MESSAGE GIMBAL_TORQUE_CMD_REPORT PACKING

#define MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT 214

typedef struct __mavlink_gimbal_torque_cmd_report_t
{
 int16_t rl_torque_cmd; /*< Roll Torque Command*/
 int16_t el_torque_cmd; /*< Elevation Torque Command*/
 int16_t az_torque_cmd; /*< Azimuth Torque Command*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
} mavlink_gimbal_torque_cmd_report_t;

#define MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN 8
#define MAVLINK_MSG_ID_214_LEN 8

#define MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_CRC 69
#define MAVLINK_MSG_ID_214_CRC 69



#define MAVLINK_MESSAGE_INFO_GIMBAL_TORQUE_CMD_REPORT { \
	"GIMBAL_TORQUE_CMD_REPORT", \
	5, \
	{  { "rl_torque_cmd", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_gimbal_torque_cmd_report_t, rl_torque_cmd) }, \
         { "el_torque_cmd", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_gimbal_torque_cmd_report_t, el_torque_cmd) }, \
         { "az_torque_cmd", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_gimbal_torque_cmd_report_t, az_torque_cmd) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_gimbal_torque_cmd_report_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_gimbal_torque_cmd_report_t, target_component) }, \
         } \
}


/**
 * @brief Pack a gimbal_torque_cmd_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param rl_torque_cmd Roll Torque Command
 * @param el_torque_cmd Elevation Torque Command
 * @param az_torque_cmd Azimuth Torque Command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_torque_cmd_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, int16_t rl_torque_cmd, int16_t el_torque_cmd, int16_t az_torque_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN];
	_mav_put_int16_t(buf, 0, rl_torque_cmd);
	_mav_put_int16_t(buf, 2, el_torque_cmd);
	_mav_put_int16_t(buf, 4, az_torque_cmd);
	_mav_put_uint8_t(buf, 6, target_system);
	_mav_put_uint8_t(buf, 7, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#else
	mavlink_gimbal_torque_cmd_report_t packet;
	packet.rl_torque_cmd = rl_torque_cmd;
	packet.el_torque_cmd = el_torque_cmd;
	packet.az_torque_cmd = az_torque_cmd;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#endif
}

/**
 * @brief Pack a gimbal_torque_cmd_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param rl_torque_cmd Roll Torque Command
 * @param el_torque_cmd Elevation Torque Command
 * @param az_torque_cmd Azimuth Torque Command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_torque_cmd_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,int16_t rl_torque_cmd,int16_t el_torque_cmd,int16_t az_torque_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN];
	_mav_put_int16_t(buf, 0, rl_torque_cmd);
	_mav_put_int16_t(buf, 2, el_torque_cmd);
	_mav_put_int16_t(buf, 4, az_torque_cmd);
	_mav_put_uint8_t(buf, 6, target_system);
	_mav_put_uint8_t(buf, 7, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#else
	mavlink_gimbal_torque_cmd_report_t packet;
	packet.rl_torque_cmd = rl_torque_cmd;
	packet.el_torque_cmd = el_torque_cmd;
	packet.az_torque_cmd = az_torque_cmd;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#endif
}

/**
 * @brief Encode a gimbal_torque_cmd_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_torque_cmd_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_torque_cmd_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_torque_cmd_report_t* gimbal_torque_cmd_report)
{
	return mavlink_msg_gimbal_torque_cmd_report_pack(system_id, component_id, msg, gimbal_torque_cmd_report->target_system, gimbal_torque_cmd_report->target_component, gimbal_torque_cmd_report->rl_torque_cmd, gimbal_torque_cmd_report->el_torque_cmd, gimbal_torque_cmd_report->az_torque_cmd);
}

/**
 * @brief Encode a gimbal_torque_cmd_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_torque_cmd_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_torque_cmd_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_torque_cmd_report_t* gimbal_torque_cmd_report)
{
	return mavlink_msg_gimbal_torque_cmd_report_pack_chan(system_id, component_id, chan, msg, gimbal_torque_cmd_report->target_system, gimbal_torque_cmd_report->target_component, gimbal_torque_cmd_report->rl_torque_cmd, gimbal_torque_cmd_report->el_torque_cmd, gimbal_torque_cmd_report->az_torque_cmd);
}

/**
 * @brief Send a gimbal_torque_cmd_report message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param rl_torque_cmd Roll Torque Command
 * @param el_torque_cmd Elevation Torque Command
 * @param az_torque_cmd Azimuth Torque Command
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_torque_cmd_report_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int16_t rl_torque_cmd, int16_t el_torque_cmd, int16_t az_torque_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN];
	_mav_put_int16_t(buf, 0, rl_torque_cmd);
	_mav_put_int16_t(buf, 2, el_torque_cmd);
	_mav_put_int16_t(buf, 4, az_torque_cmd);
	_mav_put_uint8_t(buf, 6, target_system);
	_mav_put_uint8_t(buf, 7, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT, buf, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT, buf, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#endif
#else
	mavlink_gimbal_torque_cmd_report_t packet;
	packet.rl_torque_cmd = rl_torque_cmd;
	packet.el_torque_cmd = el_torque_cmd;
	packet.az_torque_cmd = az_torque_cmd;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_torque_cmd_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, int16_t rl_torque_cmd, int16_t el_torque_cmd, int16_t az_torque_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int16_t(buf, 0, rl_torque_cmd);
	_mav_put_int16_t(buf, 2, el_torque_cmd);
	_mav_put_int16_t(buf, 4, az_torque_cmd);
	_mav_put_uint8_t(buf, 6, target_system);
	_mav_put_uint8_t(buf, 7, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT, buf, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT, buf, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#endif
#else
	mavlink_gimbal_torque_cmd_report_t *packet = (mavlink_gimbal_torque_cmd_report_t *)msgbuf;
	packet->rl_torque_cmd = rl_torque_cmd;
	packet->el_torque_cmd = el_torque_cmd;
	packet->az_torque_cmd = az_torque_cmd;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GIMBAL_TORQUE_CMD_REPORT UNPACKING


/**
 * @brief Get field target_system from gimbal_torque_cmd_report message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gimbal_torque_cmd_report_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field target_component from gimbal_torque_cmd_report message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gimbal_torque_cmd_report_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field rl_torque_cmd from gimbal_torque_cmd_report message
 *
 * @return Roll Torque Command
 */
static inline int16_t mavlink_msg_gimbal_torque_cmd_report_get_rl_torque_cmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field el_torque_cmd from gimbal_torque_cmd_report message
 *
 * @return Elevation Torque Command
 */
static inline int16_t mavlink_msg_gimbal_torque_cmd_report_get_el_torque_cmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field az_torque_cmd from gimbal_torque_cmd_report message
 *
 * @return Azimuth Torque Command
 */
static inline int16_t mavlink_msg_gimbal_torque_cmd_report_get_az_torque_cmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Decode a gimbal_torque_cmd_report message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_torque_cmd_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_torque_cmd_report_decode(const mavlink_message_t* msg, mavlink_gimbal_torque_cmd_report_t* gimbal_torque_cmd_report)
{
#if MAVLINK_NEED_BYTE_SWAP
	gimbal_torque_cmd_report->rl_torque_cmd = mavlink_msg_gimbal_torque_cmd_report_get_rl_torque_cmd(msg);
	gimbal_torque_cmd_report->el_torque_cmd = mavlink_msg_gimbal_torque_cmd_report_get_el_torque_cmd(msg);
	gimbal_torque_cmd_report->az_torque_cmd = mavlink_msg_gimbal_torque_cmd_report_get_az_torque_cmd(msg);
	gimbal_torque_cmd_report->target_system = mavlink_msg_gimbal_torque_cmd_report_get_target_system(msg);
	gimbal_torque_cmd_report->target_component = mavlink_msg_gimbal_torque_cmd_report_get_target_component(msg);
#else
	memcpy(gimbal_torque_cmd_report, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_LEN);
#endif
}
