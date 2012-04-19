// MESSAGE LLC_OUT PACKING

#define MAVLINK_MSG_ID_LLC_OUT 186

typedef struct __mavlink_llc_out_t
{
 int16_t servoOut[4]; ///< Servo signal
 int16_t MotorOut[2]; ///< motor signal
} mavlink_llc_out_t;

#define MAVLINK_MSG_ID_LLC_OUT_LEN 12
#define MAVLINK_MSG_ID_186_LEN 12

#define MAVLINK_MSG_LLC_OUT_FIELD_SERVOOUT_LEN 4
#define MAVLINK_MSG_LLC_OUT_FIELD_MOTOROUT_LEN 2

#define MAVLINK_MESSAGE_INFO_LLC_OUT { \
	"LLC_OUT", \
	2, \
	{  { "servoOut", NULL, MAVLINK_TYPE_INT16_T, 4, 0, offsetof(mavlink_llc_out_t, servoOut) }, \
         { "MotorOut", NULL, MAVLINK_TYPE_INT16_T, 2, 8, offsetof(mavlink_llc_out_t, MotorOut) }, \
         } \
}


/**
 * @brief Pack a llc_out message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param servoOut Servo signal
 * @param MotorOut motor signal
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_llc_out_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const int16_t *servoOut, const int16_t *MotorOut)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];

	_mav_put_int16_t_array(buf, 0, servoOut, 4);
	_mav_put_int16_t_array(buf, 8, MotorOut, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_llc_out_t packet;

	mav_array_memcpy(packet.servoOut, servoOut, sizeof(int16_t)*4);
	mav_array_memcpy(packet.MotorOut, MotorOut, sizeof(int16_t)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_LLC_OUT;
	return mavlink_finalize_message(msg, system_id, component_id, 12, 5);
}

/**
 * @brief Pack a llc_out message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param servoOut Servo signal
 * @param MotorOut motor signal
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_llc_out_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const int16_t *servoOut,const int16_t *MotorOut)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];

	_mav_put_int16_t_array(buf, 0, servoOut, 4);
	_mav_put_int16_t_array(buf, 8, MotorOut, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 12);
#else
	mavlink_llc_out_t packet;

	mav_array_memcpy(packet.servoOut, servoOut, sizeof(int16_t)*4);
	mav_array_memcpy(packet.MotorOut, MotorOut, sizeof(int16_t)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 12);
#endif

	msg->msgid = MAVLINK_MSG_ID_LLC_OUT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 5);
}

/**
 * @brief Encode a llc_out struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param llc_out C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_llc_out_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_llc_out_t* llc_out)
{
	return mavlink_msg_llc_out_pack(system_id, component_id, msg, llc_out->servoOut, llc_out->MotorOut);
}

/**
 * @brief Send a llc_out message
 * @param chan MAVLink channel to send the message
 *
 * @param servoOut Servo signal
 * @param MotorOut motor signal
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_llc_out_send(mavlink_channel_t chan, const int16_t *servoOut, const int16_t *MotorOut)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[12];

	_mav_put_int16_t_array(buf, 0, servoOut, 4);
	_mav_put_int16_t_array(buf, 8, MotorOut, 2);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LLC_OUT, buf, 12, 5);
#else
	mavlink_llc_out_t packet;

	mav_array_memcpy(packet.servoOut, servoOut, sizeof(int16_t)*4);
	mav_array_memcpy(packet.MotorOut, MotorOut, sizeof(int16_t)*2);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LLC_OUT, (const char *)&packet, 12, 5);
#endif
}

#endif

// MESSAGE LLC_OUT UNPACKING


/**
 * @brief Get field servoOut from llc_out message
 *
 * @return Servo signal
 */
static inline uint16_t mavlink_msg_llc_out_get_servoOut(const mavlink_message_t* msg, int16_t *servoOut)
{
	return _MAV_RETURN_int16_t_array(msg, servoOut, 4,  0);
}

/**
 * @brief Get field MotorOut from llc_out message
 *
 * @return motor signal
 */
static inline uint16_t mavlink_msg_llc_out_get_MotorOut(const mavlink_message_t* msg, int16_t *MotorOut)
{
	return _MAV_RETURN_int16_t_array(msg, MotorOut, 2,  8);
}

/**
 * @brief Decode a llc_out message into a struct
 *
 * @param msg The message to decode
 * @param llc_out C-struct to decode the message contents into
 */
static inline void mavlink_msg_llc_out_decode(const mavlink_message_t* msg, mavlink_llc_out_t* llc_out)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_llc_out_get_servoOut(msg, llc_out->servoOut);
	mavlink_msg_llc_out_get_MotorOut(msg, llc_out->MotorOut);
#else
	memcpy(llc_out, _MAV_PAYLOAD(msg), 12);
#endif
}
