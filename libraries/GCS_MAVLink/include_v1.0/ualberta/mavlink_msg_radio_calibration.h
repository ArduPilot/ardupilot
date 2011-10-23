// MESSAGE RADIO_CALIBRATION PACKING

#define MAVLINK_MSG_ID_RADIO_CALIBRATION 221

typedef struct __mavlink_radio_calibration_t
{
 uint16_t aileron[3]; ///< Aileron setpoints: left, center, right
 uint16_t elevator[3]; ///< Elevator setpoints: nose down, center, nose up
 uint16_t rudder[3]; ///< Rudder setpoints: nose left, center, nose right
 uint16_t gyro[2]; ///< Tail gyro mode/gain setpoints: heading hold, rate mode
 uint16_t pitch[5]; ///< Pitch curve setpoints (every 25%)
 uint16_t throttle[5]; ///< Throttle curve setpoints (every 25%)
} mavlink_radio_calibration_t;

#define MAVLINK_MSG_ID_RADIO_CALIBRATION_LEN 42
#define MAVLINK_MSG_ID_221_LEN 42

#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_AILERON_LEN 3
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_ELEVATOR_LEN 3
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_RUDDER_LEN 3
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_GYRO_LEN 2
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_PITCH_LEN 5
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_THROTTLE_LEN 5

#define MAVLINK_MESSAGE_INFO_RADIO_CALIBRATION { \
	"RADIO_CALIBRATION", \
	6, \
	{  { "aileron", NULL, MAVLINK_TYPE_UINT16_T, 3, 0, offsetof(mavlink_radio_calibration_t, aileron) }, \
         { "elevator", NULL, MAVLINK_TYPE_UINT16_T, 3, 6, offsetof(mavlink_radio_calibration_t, elevator) }, \
         { "rudder", NULL, MAVLINK_TYPE_UINT16_T, 3, 12, offsetof(mavlink_radio_calibration_t, rudder) }, \
         { "gyro", NULL, MAVLINK_TYPE_UINT16_T, 2, 18, offsetof(mavlink_radio_calibration_t, gyro) }, \
         { "pitch", NULL, MAVLINK_TYPE_UINT16_T, 5, 22, offsetof(mavlink_radio_calibration_t, pitch) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 5, 32, offsetof(mavlink_radio_calibration_t, throttle) }, \
         } \
}


/**
 * @brief Pack a radio_calibration message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param aileron Aileron setpoints: left, center, right
 * @param elevator Elevator setpoints: nose down, center, nose up
 * @param rudder Rudder setpoints: nose left, center, nose right
 * @param gyro Tail gyro mode/gain setpoints: heading hold, rate mode
 * @param pitch Pitch curve setpoints (every 25%)
 * @param throttle Throttle curve setpoints (every 25%)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_calibration_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const uint16_t *aileron, const uint16_t *elevator, const uint16_t *rudder, const uint16_t *gyro, const uint16_t *pitch, const uint16_t *throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[42];

	_mav_put_uint16_t_array(buf, 0, aileron, 3);
	_mav_put_uint16_t_array(buf, 6, elevator, 3);
	_mav_put_uint16_t_array(buf, 12, rudder, 3);
	_mav_put_uint16_t_array(buf, 18, gyro, 2);
	_mav_put_uint16_t_array(buf, 22, pitch, 5);
	_mav_put_uint16_t_array(buf, 32, throttle, 5);
        memcpy(_MAV_PAYLOAD(msg), buf, 42);
#else
	mavlink_radio_calibration_t packet;

	memcpy(packet.aileron, aileron, sizeof(uint16_t)*3);
	memcpy(packet.elevator, elevator, sizeof(uint16_t)*3);
	memcpy(packet.rudder, rudder, sizeof(uint16_t)*3);
	memcpy(packet.gyro, gyro, sizeof(uint16_t)*2);
	memcpy(packet.pitch, pitch, sizeof(uint16_t)*5);
	memcpy(packet.throttle, throttle, sizeof(uint16_t)*5);
        memcpy(_MAV_PAYLOAD(msg), &packet, 42);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADIO_CALIBRATION;
	return mavlink_finalize_message(msg, system_id, component_id, 42, 71);
}

/**
 * @brief Pack a radio_calibration message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param aileron Aileron setpoints: left, center, right
 * @param elevator Elevator setpoints: nose down, center, nose up
 * @param rudder Rudder setpoints: nose left, center, nose right
 * @param gyro Tail gyro mode/gain setpoints: heading hold, rate mode
 * @param pitch Pitch curve setpoints (every 25%)
 * @param throttle Throttle curve setpoints (every 25%)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_calibration_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const uint16_t *aileron,const uint16_t *elevator,const uint16_t *rudder,const uint16_t *gyro,const uint16_t *pitch,const uint16_t *throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[42];

	_mav_put_uint16_t_array(buf, 0, aileron, 3);
	_mav_put_uint16_t_array(buf, 6, elevator, 3);
	_mav_put_uint16_t_array(buf, 12, rudder, 3);
	_mav_put_uint16_t_array(buf, 18, gyro, 2);
	_mav_put_uint16_t_array(buf, 22, pitch, 5);
	_mav_put_uint16_t_array(buf, 32, throttle, 5);
        memcpy(_MAV_PAYLOAD(msg), buf, 42);
#else
	mavlink_radio_calibration_t packet;

	memcpy(packet.aileron, aileron, sizeof(uint16_t)*3);
	memcpy(packet.elevator, elevator, sizeof(uint16_t)*3);
	memcpy(packet.rudder, rudder, sizeof(uint16_t)*3);
	memcpy(packet.gyro, gyro, sizeof(uint16_t)*2);
	memcpy(packet.pitch, pitch, sizeof(uint16_t)*5);
	memcpy(packet.throttle, throttle, sizeof(uint16_t)*5);
        memcpy(_MAV_PAYLOAD(msg), &packet, 42);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADIO_CALIBRATION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 42, 71);
}

/**
 * @brief Encode a radio_calibration struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radio_calibration C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_calibration_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radio_calibration_t* radio_calibration)
{
	return mavlink_msg_radio_calibration_pack(system_id, component_id, msg, radio_calibration->aileron, radio_calibration->elevator, radio_calibration->rudder, radio_calibration->gyro, radio_calibration->pitch, radio_calibration->throttle);
}

/**
 * @brief Send a radio_calibration message
 * @param chan MAVLink channel to send the message
 *
 * @param aileron Aileron setpoints: left, center, right
 * @param elevator Elevator setpoints: nose down, center, nose up
 * @param rudder Rudder setpoints: nose left, center, nose right
 * @param gyro Tail gyro mode/gain setpoints: heading hold, rate mode
 * @param pitch Pitch curve setpoints (every 25%)
 * @param throttle Throttle curve setpoints (every 25%)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radio_calibration_send(mavlink_channel_t chan, const uint16_t *aileron, const uint16_t *elevator, const uint16_t *rudder, const uint16_t *gyro, const uint16_t *pitch, const uint16_t *throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[42];

	_mav_put_uint16_t_array(buf, 0, aileron, 3);
	_mav_put_uint16_t_array(buf, 6, elevator, 3);
	_mav_put_uint16_t_array(buf, 12, rudder, 3);
	_mav_put_uint16_t_array(buf, 18, gyro, 2);
	_mav_put_uint16_t_array(buf, 22, pitch, 5);
	_mav_put_uint16_t_array(buf, 32, throttle, 5);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_CALIBRATION, buf, 42, 71);
#else
	mavlink_radio_calibration_t packet;

	memcpy(packet.aileron, aileron, sizeof(uint16_t)*3);
	memcpy(packet.elevator, elevator, sizeof(uint16_t)*3);
	memcpy(packet.rudder, rudder, sizeof(uint16_t)*3);
	memcpy(packet.gyro, gyro, sizeof(uint16_t)*2);
	memcpy(packet.pitch, pitch, sizeof(uint16_t)*5);
	memcpy(packet.throttle, throttle, sizeof(uint16_t)*5);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_CALIBRATION, (const char *)&packet, 42, 71);
#endif
}

#endif

// MESSAGE RADIO_CALIBRATION UNPACKING


/**
 * @brief Get field aileron from radio_calibration message
 *
 * @return Aileron setpoints: left, center, right
 */
static inline uint16_t mavlink_msg_radio_calibration_get_aileron(const mavlink_message_t* msg, uint16_t *aileron)
{
	return _MAV_RETURN_uint16_t_array(msg, aileron, 3,  0);
}

/**
 * @brief Get field elevator from radio_calibration message
 *
 * @return Elevator setpoints: nose down, center, nose up
 */
static inline uint16_t mavlink_msg_radio_calibration_get_elevator(const mavlink_message_t* msg, uint16_t *elevator)
{
	return _MAV_RETURN_uint16_t_array(msg, elevator, 3,  6);
}

/**
 * @brief Get field rudder from radio_calibration message
 *
 * @return Rudder setpoints: nose left, center, nose right
 */
static inline uint16_t mavlink_msg_radio_calibration_get_rudder(const mavlink_message_t* msg, uint16_t *rudder)
{
	return _MAV_RETURN_uint16_t_array(msg, rudder, 3,  12);
}

/**
 * @brief Get field gyro from radio_calibration message
 *
 * @return Tail gyro mode/gain setpoints: heading hold, rate mode
 */
static inline uint16_t mavlink_msg_radio_calibration_get_gyro(const mavlink_message_t* msg, uint16_t *gyro)
{
	return _MAV_RETURN_uint16_t_array(msg, gyro, 2,  18);
}

/**
 * @brief Get field pitch from radio_calibration message
 *
 * @return Pitch curve setpoints (every 25%)
 */
static inline uint16_t mavlink_msg_radio_calibration_get_pitch(const mavlink_message_t* msg, uint16_t *pitch)
{
	return _MAV_RETURN_uint16_t_array(msg, pitch, 5,  22);
}

/**
 * @brief Get field throttle from radio_calibration message
 *
 * @return Throttle curve setpoints (every 25%)
 */
static inline uint16_t mavlink_msg_radio_calibration_get_throttle(const mavlink_message_t* msg, uint16_t *throttle)
{
	return _MAV_RETURN_uint16_t_array(msg, throttle, 5,  32);
}

/**
 * @brief Decode a radio_calibration message into a struct
 *
 * @param msg The message to decode
 * @param radio_calibration C-struct to decode the message contents into
 */
static inline void mavlink_msg_radio_calibration_decode(const mavlink_message_t* msg, mavlink_radio_calibration_t* radio_calibration)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_radio_calibration_get_aileron(msg, radio_calibration->aileron);
	mavlink_msg_radio_calibration_get_elevator(msg, radio_calibration->elevator);
	mavlink_msg_radio_calibration_get_rudder(msg, radio_calibration->rudder);
	mavlink_msg_radio_calibration_get_gyro(msg, radio_calibration->gyro);
	mavlink_msg_radio_calibration_get_pitch(msg, radio_calibration->pitch);
	mavlink_msg_radio_calibration_get_throttle(msg, radio_calibration->throttle);
#else
	memcpy(radio_calibration, _MAV_PAYLOAD(msg), 42);
#endif
}
