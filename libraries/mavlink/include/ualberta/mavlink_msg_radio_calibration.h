// MESSAGE RADIO_CALIBRATION PACKING

#define MAVLINK_MSG_ID_RADIO_CALIBRATION 82

typedef struct __mavlink_radio_calibration_t 
{
	float aileron[3]; ///< Aileron setpoints: high, center, low
	float elevator[3]; ///< Elevator setpoints: high, center, low
	float rudder[3]; ///< Rudder setpoints: high, center, low
	float gyro[2]; ///< Tail gyro mode/gain setpoints: heading hold, rate mode
	float pitch[5]; ///< Pitch curve setpoints (every 25%)
	float throttle[5]; ///< Throttle curve setpoints (every 25%)

} mavlink_radio_calibration_t;

#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_AILERON_LEN 3
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_ELEVATOR_LEN 3
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_RUDDER_LEN 3
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_GYRO_LEN 2
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_PITCH_LEN 5
#define MAVLINK_MSG_RADIO_CALIBRATION_FIELD_THROTTLE_LEN 5


/**
 * @brief Send a radio_calibration message
 *
 * @param aileron Aileron setpoints: high, center, low
 * @param elevator Elevator setpoints: high, center, low
 * @param rudder Rudder setpoints: high, center, low
 * @param gyro Tail gyro mode/gain setpoints: heading hold, rate mode
 * @param pitch Pitch curve setpoints (every 25%)
 * @param throttle Throttle curve setpoints (every 25%)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_calibration_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const float* aileron, const float* elevator, const float* rudder, const float* gyro, const float* pitch, const float* throttle)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_RADIO_CALIBRATION;

	i += put_array_by_index((int8_t*)aileron, sizeof(float)*3, i, msg->payload); //Aileron setpoints: high, center, low
	i += put_array_by_index((int8_t*)elevator, sizeof(float)*3, i, msg->payload); //Elevator setpoints: high, center, low
	i += put_array_by_index((int8_t*)rudder, sizeof(float)*3, i, msg->payload); //Rudder setpoints: high, center, low
	i += put_array_by_index((int8_t*)gyro, sizeof(float)*2, i, msg->payload); //Tail gyro mode/gain setpoints: heading hold, rate mode
	i += put_array_by_index((int8_t*)pitch, sizeof(float)*5, i, msg->payload); //Pitch curve setpoints (every 25%)
	i += put_array_by_index((int8_t*)throttle, sizeof(float)*5, i, msg->payload); //Throttle curve setpoints (every 25%)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_radio_calibration_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radio_calibration_t* radio_calibration)
{
	return mavlink_msg_radio_calibration_pack(system_id, component_id, msg, radio_calibration->aileron, radio_calibration->elevator, radio_calibration->rudder, radio_calibration->gyro, radio_calibration->pitch, radio_calibration->throttle);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radio_calibration_send(mavlink_channel_t chan, const float* aileron, const float* elevator, const float* rudder, const float* gyro, const float* pitch, const float* throttle)
{
	mavlink_message_t msg;
	mavlink_msg_radio_calibration_pack(mavlink_system.sysid, mavlink_system.compid, &msg, aileron, elevator, rudder, gyro, pitch, throttle);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE RADIO_CALIBRATION UNPACKING

/**
 * @brief Get field aileron from radio_calibration message
 *
 * @return Aileron setpoints: high, center, low
 */
static inline uint16_t mavlink_msg_radio_calibration_get_aileron(const mavlink_message_t* msg, float* r_data)
{

	memcpy(r_data, msg->payload, sizeof(float)*3);
	return sizeof(float)*3;
}

/**
 * @brief Get field elevator from radio_calibration message
 *
 * @return Elevator setpoints: high, center, low
 */
static inline uint16_t mavlink_msg_radio_calibration_get_elevator(const mavlink_message_t* msg, float* r_data)
{

	memcpy(r_data, msg->payload+sizeof(float)*3, sizeof(float)*3);
	return sizeof(float)*3;
}

/**
 * @brief Get field rudder from radio_calibration message
 *
 * @return Rudder setpoints: high, center, low
 */
static inline uint16_t mavlink_msg_radio_calibration_get_rudder(const mavlink_message_t* msg, float* r_data)
{

	memcpy(r_data, msg->payload+sizeof(float)*3+sizeof(float)*3, sizeof(float)*3);
	return sizeof(float)*3;
}

/**
 * @brief Get field gyro from radio_calibration message
 *
 * @return Tail gyro mode/gain setpoints: heading hold, rate mode
 */
static inline uint16_t mavlink_msg_radio_calibration_get_gyro(const mavlink_message_t* msg, float* r_data)
{

	memcpy(r_data, msg->payload+sizeof(float)*3+sizeof(float)*3+sizeof(float)*3, sizeof(float)*2);
	return sizeof(float)*2;
}

/**
 * @brief Get field pitch from radio_calibration message
 *
 * @return Pitch curve setpoints (every 25%)
 */
static inline uint16_t mavlink_msg_radio_calibration_get_pitch(const mavlink_message_t* msg, float* r_data)
{

	memcpy(r_data, msg->payload+sizeof(float)*3+sizeof(float)*3+sizeof(float)*3+sizeof(float)*2, sizeof(float)*5);
	return sizeof(float)*5;
}

/**
 * @brief Get field throttle from radio_calibration message
 *
 * @return Throttle curve setpoints (every 25%)
 */
static inline uint16_t mavlink_msg_radio_calibration_get_throttle(const mavlink_message_t* msg, float* r_data)
{

	memcpy(r_data, msg->payload+sizeof(float)*3+sizeof(float)*3+sizeof(float)*3+sizeof(float)*2+sizeof(float)*5, sizeof(float)*5);
	return sizeof(float)*5;
}

static inline void mavlink_msg_radio_calibration_decode(const mavlink_message_t* msg, mavlink_radio_calibration_t* radio_calibration)
{
	mavlink_msg_radio_calibration_get_aileron(msg, radio_calibration->aileron);
	mavlink_msg_radio_calibration_get_elevator(msg, radio_calibration->elevator);
	mavlink_msg_radio_calibration_get_rudder(msg, radio_calibration->rudder);
	mavlink_msg_radio_calibration_get_gyro(msg, radio_calibration->gyro);
	mavlink_msg_radio_calibration_get_pitch(msg, radio_calibration->pitch);
	mavlink_msg_radio_calibration_get_throttle(msg, radio_calibration->throttle);
}
