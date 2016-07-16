// MESSAGE PREARM_CHECK_REPORT PACKING

#define MAVLINK_MSG_ID_PREARM_CHECK_REPORT 246

typedef struct __mavlink_prearm_check_report_t
{
 uint64_t enabled_checks; /*< bitmask of enabled checks (0=disabled, 1=enabled)*/
 uint64_t passed_checks; /*< bitmask of passed checks (0=failed, 1=passed).  Bitmask of passed checks = (enabled_checks AND passed_checks), Bitmask of failed checks = (enabled_checks AND ~passed_checks)*/
} mavlink_prearm_check_report_t;

#define MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN 16
#define MAVLINK_MSG_ID_246_LEN 16

#define MAVLINK_MSG_ID_PREARM_CHECK_REPORT_CRC 118
#define MAVLINK_MSG_ID_246_CRC 118



#define MAVLINK_MESSAGE_INFO_PREARM_CHECK_REPORT { \
	"PREARM_CHECK_REPORT", \
	2, \
	{  { "enabled_checks", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_prearm_check_report_t, enabled_checks) }, \
         { "passed_checks", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_prearm_check_report_t, passed_checks) }, \
         } \
}


/**
 * @brief Pack a prearm_check_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param enabled_checks bitmask of enabled checks (0=disabled, 1=enabled)
 * @param passed_checks bitmask of passed checks (0=failed, 1=passed).  Bitmask of passed checks = (enabled_checks AND passed_checks), Bitmask of failed checks = (enabled_checks AND ~passed_checks)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prearm_check_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t enabled_checks, uint64_t passed_checks)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN];
	_mav_put_uint64_t(buf, 0, enabled_checks);
	_mav_put_uint64_t(buf, 8, passed_checks);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#else
	mavlink_prearm_check_report_t packet;
	packet.enabled_checks = enabled_checks;
	packet.passed_checks = passed_checks;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PREARM_CHECK_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#endif
}

/**
 * @brief Pack a prearm_check_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param enabled_checks bitmask of enabled checks (0=disabled, 1=enabled)
 * @param passed_checks bitmask of passed checks (0=failed, 1=passed).  Bitmask of passed checks = (enabled_checks AND passed_checks), Bitmask of failed checks = (enabled_checks AND ~passed_checks)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prearm_check_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t enabled_checks,uint64_t passed_checks)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN];
	_mav_put_uint64_t(buf, 0, enabled_checks);
	_mav_put_uint64_t(buf, 8, passed_checks);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#else
	mavlink_prearm_check_report_t packet;
	packet.enabled_checks = enabled_checks;
	packet.passed_checks = passed_checks;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PREARM_CHECK_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#endif
}

/**
 * @brief Encode a prearm_check_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param prearm_check_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prearm_check_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_prearm_check_report_t* prearm_check_report)
{
	return mavlink_msg_prearm_check_report_pack(system_id, component_id, msg, prearm_check_report->enabled_checks, prearm_check_report->passed_checks);
}

/**
 * @brief Encode a prearm_check_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param prearm_check_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prearm_check_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_prearm_check_report_t* prearm_check_report)
{
	return mavlink_msg_prearm_check_report_pack_chan(system_id, component_id, chan, msg, prearm_check_report->enabled_checks, prearm_check_report->passed_checks);
}

/**
 * @brief Send a prearm_check_report message
 * @param chan MAVLink channel to send the message
 *
 * @param enabled_checks bitmask of enabled checks (0=disabled, 1=enabled)
 * @param passed_checks bitmask of passed checks (0=failed, 1=passed).  Bitmask of passed checks = (enabled_checks AND passed_checks), Bitmask of failed checks = (enabled_checks AND ~passed_checks)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_prearm_check_report_send(mavlink_channel_t chan, uint64_t enabled_checks, uint64_t passed_checks)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN];
	_mav_put_uint64_t(buf, 0, enabled_checks);
	_mav_put_uint64_t(buf, 8, passed_checks);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_REPORT, buf, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_REPORT, buf, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#endif
#else
	mavlink_prearm_check_report_t packet;
	packet.enabled_checks = enabled_checks;
	packet.passed_checks = passed_checks;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_REPORT, (const char *)&packet, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_REPORT, (const char *)&packet, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_prearm_check_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t enabled_checks, uint64_t passed_checks)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, enabled_checks);
	_mav_put_uint64_t(buf, 8, passed_checks);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_REPORT, buf, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_REPORT, buf, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#endif
#else
	mavlink_prearm_check_report_t *packet = (mavlink_prearm_check_report_t *)msgbuf;
	packet->enabled_checks = enabled_checks;
	packet->passed_checks = passed_checks;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_REPORT, (const char *)packet, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PREARM_CHECK_REPORT, (const char *)packet, MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PREARM_CHECK_REPORT UNPACKING


/**
 * @brief Get field enabled_checks from prearm_check_report message
 *
 * @return bitmask of enabled checks (0=disabled, 1=enabled)
 */
static inline uint64_t mavlink_msg_prearm_check_report_get_enabled_checks(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field passed_checks from prearm_check_report message
 *
 * @return bitmask of passed checks (0=failed, 1=passed).  Bitmask of passed checks = (enabled_checks AND passed_checks), Bitmask of failed checks = (enabled_checks AND ~passed_checks)
 */
static inline uint64_t mavlink_msg_prearm_check_report_get_passed_checks(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Decode a prearm_check_report message into a struct
 *
 * @param msg The message to decode
 * @param prearm_check_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_prearm_check_report_decode(const mavlink_message_t* msg, mavlink_prearm_check_report_t* prearm_check_report)
{
#if MAVLINK_NEED_BYTE_SWAP
	prearm_check_report->enabled_checks = mavlink_msg_prearm_check_report_get_enabled_checks(msg);
	prearm_check_report->passed_checks = mavlink_msg_prearm_check_report_get_passed_checks(msg);
#else
	memcpy(prearm_check_report, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PREARM_CHECK_REPORT_LEN);
#endif
}
