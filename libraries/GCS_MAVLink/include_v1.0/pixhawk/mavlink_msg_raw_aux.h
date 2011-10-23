// MESSAGE RAW_AUX PACKING

#define MAVLINK_MSG_ID_RAW_AUX 172

typedef struct __mavlink_raw_aux_t
{
 int32_t baro; ///< Barometric pressure (hecto Pascal)
 uint16_t adc1; ///< ADC1 (J405 ADC3, LPC2148 AD0.6)
 uint16_t adc2; ///< ADC2 (J405 ADC5, LPC2148 AD0.2)
 uint16_t adc3; ///< ADC3 (J405 ADC6, LPC2148 AD0.1)
 uint16_t adc4; ///< ADC4 (J405 ADC7, LPC2148 AD1.3)
 uint16_t vbat; ///< Battery voltage
 int16_t temp; ///< Temperature (degrees celcius)
} mavlink_raw_aux_t;

#define MAVLINK_MSG_ID_RAW_AUX_LEN 16
#define MAVLINK_MSG_ID_172_LEN 16



#define MAVLINK_MESSAGE_INFO_RAW_AUX { \
	"RAW_AUX", \
	7, \
	{  { "baro", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_raw_aux_t, baro) }, \
         { "adc1", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_raw_aux_t, adc1) }, \
         { "adc2", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_raw_aux_t, adc2) }, \
         { "adc3", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_raw_aux_t, adc3) }, \
         { "adc4", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_raw_aux_t, adc4) }, \
         { "vbat", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_raw_aux_t, vbat) }, \
         { "temp", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_raw_aux_t, temp) }, \
         } \
}


/**
 * @brief Pack a raw_aux message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param adc1 ADC1 (J405 ADC3, LPC2148 AD0.6)
 * @param adc2 ADC2 (J405 ADC5, LPC2148 AD0.2)
 * @param adc3 ADC3 (J405 ADC6, LPC2148 AD0.1)
 * @param adc4 ADC4 (J405 ADC7, LPC2148 AD1.3)
 * @param vbat Battery voltage
 * @param temp Temperature (degrees celcius)
 * @param baro Barometric pressure (hecto Pascal)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_aux_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t vbat, int16_t temp, int32_t baro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_int32_t(buf, 0, baro);
	_mav_put_uint16_t(buf, 4, adc1);
	_mav_put_uint16_t(buf, 6, adc2);
	_mav_put_uint16_t(buf, 8, adc3);
	_mav_put_uint16_t(buf, 10, adc4);
	_mav_put_uint16_t(buf, 12, vbat);
	_mav_put_int16_t(buf, 14, temp);

        memcpy(_MAV_PAYLOAD(msg), buf, 16);
#else
	mavlink_raw_aux_t packet;
	packet.baro = baro;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.vbat = vbat;
	packet.temp = temp;

        memcpy(_MAV_PAYLOAD(msg), &packet, 16);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_AUX;
	return mavlink_finalize_message(msg, system_id, component_id, 16, 182);
}

/**
 * @brief Pack a raw_aux message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param adc1 ADC1 (J405 ADC3, LPC2148 AD0.6)
 * @param adc2 ADC2 (J405 ADC5, LPC2148 AD0.2)
 * @param adc3 ADC3 (J405 ADC6, LPC2148 AD0.1)
 * @param adc4 ADC4 (J405 ADC7, LPC2148 AD1.3)
 * @param vbat Battery voltage
 * @param temp Temperature (degrees celcius)
 * @param baro Barometric pressure (hecto Pascal)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_aux_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t adc1,uint16_t adc2,uint16_t adc3,uint16_t adc4,uint16_t vbat,int16_t temp,int32_t baro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_int32_t(buf, 0, baro);
	_mav_put_uint16_t(buf, 4, adc1);
	_mav_put_uint16_t(buf, 6, adc2);
	_mav_put_uint16_t(buf, 8, adc3);
	_mav_put_uint16_t(buf, 10, adc4);
	_mav_put_uint16_t(buf, 12, vbat);
	_mav_put_int16_t(buf, 14, temp);

        memcpy(_MAV_PAYLOAD(msg), buf, 16);
#else
	mavlink_raw_aux_t packet;
	packet.baro = baro;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.vbat = vbat;
	packet.temp = temp;

        memcpy(_MAV_PAYLOAD(msg), &packet, 16);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_AUX;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 16, 182);
}

/**
 * @brief Encode a raw_aux struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_aux C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_aux_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_aux_t* raw_aux)
{
	return mavlink_msg_raw_aux_pack(system_id, component_id, msg, raw_aux->adc1, raw_aux->adc2, raw_aux->adc3, raw_aux->adc4, raw_aux->vbat, raw_aux->temp, raw_aux->baro);
}

/**
 * @brief Send a raw_aux message
 * @param chan MAVLink channel to send the message
 *
 * @param adc1 ADC1 (J405 ADC3, LPC2148 AD0.6)
 * @param adc2 ADC2 (J405 ADC5, LPC2148 AD0.2)
 * @param adc3 ADC3 (J405 ADC6, LPC2148 AD0.1)
 * @param adc4 ADC4 (J405 ADC7, LPC2148 AD1.3)
 * @param vbat Battery voltage
 * @param temp Temperature (degrees celcius)
 * @param baro Barometric pressure (hecto Pascal)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_aux_send(mavlink_channel_t chan, uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t vbat, int16_t temp, int32_t baro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_int32_t(buf, 0, baro);
	_mav_put_uint16_t(buf, 4, adc1);
	_mav_put_uint16_t(buf, 6, adc2);
	_mav_put_uint16_t(buf, 8, adc3);
	_mav_put_uint16_t(buf, 10, adc4);
	_mav_put_uint16_t(buf, 12, vbat);
	_mav_put_int16_t(buf, 14, temp);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_AUX, buf, 16, 182);
#else
	mavlink_raw_aux_t packet;
	packet.baro = baro;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.vbat = vbat;
	packet.temp = temp;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_AUX, (const char *)&packet, 16, 182);
#endif
}

#endif

// MESSAGE RAW_AUX UNPACKING


/**
 * @brief Get field adc1 from raw_aux message
 *
 * @return ADC1 (J405 ADC3, LPC2148 AD0.6)
 */
static inline uint16_t mavlink_msg_raw_aux_get_adc1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field adc2 from raw_aux message
 *
 * @return ADC2 (J405 ADC5, LPC2148 AD0.2)
 */
static inline uint16_t mavlink_msg_raw_aux_get_adc2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field adc3 from raw_aux message
 *
 * @return ADC3 (J405 ADC6, LPC2148 AD0.1)
 */
static inline uint16_t mavlink_msg_raw_aux_get_adc3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field adc4 from raw_aux message
 *
 * @return ADC4 (J405 ADC7, LPC2148 AD1.3)
 */
static inline uint16_t mavlink_msg_raw_aux_get_adc4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field vbat from raw_aux message
 *
 * @return Battery voltage
 */
static inline uint16_t mavlink_msg_raw_aux_get_vbat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field temp from raw_aux message
 *
 * @return Temperature (degrees celcius)
 */
static inline int16_t mavlink_msg_raw_aux_get_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field baro from raw_aux message
 *
 * @return Barometric pressure (hecto Pascal)
 */
static inline int32_t mavlink_msg_raw_aux_get_baro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Decode a raw_aux message into a struct
 *
 * @param msg The message to decode
 * @param raw_aux C-struct to decode the message contents into
 */
static inline void mavlink_msg_raw_aux_decode(const mavlink_message_t* msg, mavlink_raw_aux_t* raw_aux)
{
#if MAVLINK_NEED_BYTE_SWAP
	raw_aux->baro = mavlink_msg_raw_aux_get_baro(msg);
	raw_aux->adc1 = mavlink_msg_raw_aux_get_adc1(msg);
	raw_aux->adc2 = mavlink_msg_raw_aux_get_adc2(msg);
	raw_aux->adc3 = mavlink_msg_raw_aux_get_adc3(msg);
	raw_aux->adc4 = mavlink_msg_raw_aux_get_adc4(msg);
	raw_aux->vbat = mavlink_msg_raw_aux_get_vbat(msg);
	raw_aux->temp = mavlink_msg_raw_aux_get_temp(msg);
#else
	memcpy(raw_aux, _MAV_PAYLOAD(msg), 16);
#endif
}
