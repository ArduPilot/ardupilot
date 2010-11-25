// MESSAGE RAW_AUX PACKING

#define MAVLINK_MSG_ID_RAW_AUX 141

typedef struct __mavlink_raw_aux_t 
{
	uint16_t adc1; ///< ADC1 (J405 ADC3, LPC2148 AD0.6)
	uint16_t adc2; ///< ADC2 (J405 ADC5, LPC2148 AD0.2)
	uint16_t adc3; ///< ADC3 (J405 ADC6, LPC2148 AD0.1)
	uint16_t adc4; ///< ADC4 (J405 ADC7, LPC2148 AD1.3)
	uint16_t vbat; ///< Battery voltage
	int16_t temp; ///< Temperature (degrees celcius)
	int32_t baro; ///< Barometric pressure (hecto Pascal)

} mavlink_raw_aux_t;



/**
 * @brief Send a raw_aux message
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
static inline uint16_t mavlink_msg_raw_aux_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t vbat, int16_t temp, int32_t baro)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_RAW_AUX;

	i += put_uint16_t_by_index(adc1, i, msg->payload); //ADC1 (J405 ADC3, LPC2148 AD0.6)
	i += put_uint16_t_by_index(adc2, i, msg->payload); //ADC2 (J405 ADC5, LPC2148 AD0.2)
	i += put_uint16_t_by_index(adc3, i, msg->payload); //ADC3 (J405 ADC6, LPC2148 AD0.1)
	i += put_uint16_t_by_index(adc4, i, msg->payload); //ADC4 (J405 ADC7, LPC2148 AD1.3)
	i += put_uint16_t_by_index(vbat, i, msg->payload); //Battery voltage
	i += put_int16_t_by_index(temp, i, msg->payload); //Temperature (degrees celcius)
	i += put_int32_t_by_index(baro, i, msg->payload); //Barometric pressure (hecto Pascal)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_raw_aux_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_aux_t* raw_aux)
{
	return mavlink_msg_raw_aux_pack(system_id, component_id, msg, raw_aux->adc1, raw_aux->adc2, raw_aux->adc3, raw_aux->adc4, raw_aux->vbat, raw_aux->temp, raw_aux->baro);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_aux_send(mavlink_channel_t chan, uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t vbat, int16_t temp, int32_t baro)
{
	mavlink_message_t msg;
	mavlink_msg_raw_aux_pack(mavlink_system.sysid, mavlink_system.compid, &msg, adc1, adc2, adc3, adc4, vbat, temp, baro);
	mavlink_send_uart(chan, &msg);
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
	generic_16bit r;
	r.b[1] = (msg->payload)[0];
	r.b[0] = (msg->payload)[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field adc2 from raw_aux message
 *
 * @return ADC2 (J405 ADC5, LPC2148 AD0.2)
 */
static inline uint16_t mavlink_msg_raw_aux_get_adc2(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field adc3 from raw_aux message
 *
 * @return ADC3 (J405 ADC6, LPC2148 AD0.1)
 */
static inline uint16_t mavlink_msg_raw_aux_get_adc3(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field adc4 from raw_aux message
 *
 * @return ADC4 (J405 ADC7, LPC2148 AD1.3)
 */
static inline uint16_t mavlink_msg_raw_aux_get_adc4(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field vbat from raw_aux message
 *
 * @return Battery voltage
 */
static inline uint16_t mavlink_msg_raw_aux_get_vbat(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field temp from raw_aux message
 *
 * @return Temperature (degrees celcius)
 */
static inline int16_t mavlink_msg_raw_aux_get_temp(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field baro from raw_aux message
 *
 * @return Barometric pressure (hecto Pascal)
 */
static inline int32_t mavlink_msg_raw_aux_get_baro(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(int16_t))[0];
	r.b[2] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(int16_t))[1];
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(int16_t))[2];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(int16_t))[3];
	return (int32_t)r.i;
}

static inline void mavlink_msg_raw_aux_decode(const mavlink_message_t* msg, mavlink_raw_aux_t* raw_aux)
{
	raw_aux->adc1 = mavlink_msg_raw_aux_get_adc1(msg);
	raw_aux->adc2 = mavlink_msg_raw_aux_get_adc2(msg);
	raw_aux->adc3 = mavlink_msg_raw_aux_get_adc3(msg);
	raw_aux->adc4 = mavlink_msg_raw_aux_get_adc4(msg);
	raw_aux->vbat = mavlink_msg_raw_aux_get_vbat(msg);
	raw_aux->temp = mavlink_msg_raw_aux_get_temp(msg);
	raw_aux->baro = mavlink_msg_raw_aux_get_baro(msg);
}
