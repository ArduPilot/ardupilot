// MESSAGE BATTERY2 PACKING

#define MAVLINK_MSG_ID_BATTERY2 181

typedef struct __mavlink_battery2_t
{
 uint16_t voltage; ///< voltage in millivolts
 int16_t current_battery; ///< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
} mavlink_battery2_t;

#define MAVLINK_MSG_ID_BATTERY2_LEN 4
#define MAVLINK_MSG_ID_181_LEN 4

#define MAVLINK_MSG_ID_BATTERY2_CRC 174
#define MAVLINK_MSG_ID_181_CRC 174



#define MAVLINK_MESSAGE_INFO_BATTERY2 { \
	"BATTERY2", \
	2, \
	{  { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_battery2_t, voltage) }, \
         { "current_battery", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_battery2_t, current_battery) }, \
         } \
}


/**
 * @brief Pack a battery2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param voltage voltage in millivolts
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t voltage, int16_t current_battery)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BATTERY2_LEN];
	_mav_put_uint16_t(buf, 0, voltage);
	_mav_put_int16_t(buf, 2, current_battery);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY2_LEN);
#else
	mavlink_battery2_t packet;
	packet.voltage = voltage;
	packet.current_battery = current_battery;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY2_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BATTERY2;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BATTERY2_LEN, MAVLINK_MSG_ID_BATTERY2_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BATTERY2_LEN);
#endif
}

/**
 * @brief Pack a battery2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param voltage voltage in millivolts
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t voltage,int16_t current_battery)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BATTERY2_LEN];
	_mav_put_uint16_t(buf, 0, voltage);
	_mav_put_int16_t(buf, 2, current_battery);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY2_LEN);
#else
	mavlink_battery2_t packet;
	packet.voltage = voltage;
	packet.current_battery = current_battery;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY2_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BATTERY2;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BATTERY2_LEN, MAVLINK_MSG_ID_BATTERY2_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BATTERY2_LEN);
#endif
}

/**
 * @brief Encode a battery2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param battery2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery2_t* battery2)
{
	return mavlink_msg_battery2_pack(system_id, component_id, msg, battery2->voltage, battery2->current_battery);
}

/**
 * @brief Encode a battery2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_battery2_t* battery2)
{
	return mavlink_msg_battery2_pack_chan(system_id, component_id, chan, msg, battery2->voltage, battery2->current_battery);
}

/**
 * @brief Send a battery2 message
 * @param chan MAVLink channel to send the message
 *
 * @param voltage voltage in millivolts
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_battery2_send(mavlink_channel_t chan, uint16_t voltage, int16_t current_battery)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BATTERY2_LEN];
	_mav_put_uint16_t(buf, 0, voltage);
	_mav_put_int16_t(buf, 2, current_battery);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY2, buf, MAVLINK_MSG_ID_BATTERY2_LEN, MAVLINK_MSG_ID_BATTERY2_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY2, buf, MAVLINK_MSG_ID_BATTERY2_LEN);
#endif
#else
	mavlink_battery2_t packet;
	packet.voltage = voltage;
	packet.current_battery = current_battery;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY2, (const char *)&packet, MAVLINK_MSG_ID_BATTERY2_LEN, MAVLINK_MSG_ID_BATTERY2_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY2, (const char *)&packet, MAVLINK_MSG_ID_BATTERY2_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_BATTERY2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_battery2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t voltage, int16_t current_battery)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, voltage);
	_mav_put_int16_t(buf, 2, current_battery);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY2, buf, MAVLINK_MSG_ID_BATTERY2_LEN, MAVLINK_MSG_ID_BATTERY2_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY2, buf, MAVLINK_MSG_ID_BATTERY2_LEN);
#endif
#else
	mavlink_battery2_t *packet = (mavlink_battery2_t *)msgbuf;
	packet->voltage = voltage;
	packet->current_battery = current_battery;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY2, (const char *)packet, MAVLINK_MSG_ID_BATTERY2_LEN, MAVLINK_MSG_ID_BATTERY2_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY2, (const char *)packet, MAVLINK_MSG_ID_BATTERY2_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE BATTERY2 UNPACKING


/**
 * @brief Get field voltage from battery2 message
 *
 * @return voltage in millivolts
 */
static inline uint16_t mavlink_msg_battery2_get_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field current_battery from battery2 message
 *
 * @return Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 */
static inline int16_t mavlink_msg_battery2_get_current_battery(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a battery2 message into a struct
 *
 * @param msg The message to decode
 * @param battery2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_battery2_decode(const mavlink_message_t* msg, mavlink_battery2_t* battery2)
{
#if MAVLINK_NEED_BYTE_SWAP
	battery2->voltage = mavlink_msg_battery2_get_voltage(msg);
	battery2->current_battery = mavlink_msg_battery2_get_current_battery(msg);
#else
	memcpy(battery2, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_BATTERY2_LEN);
#endif
}
