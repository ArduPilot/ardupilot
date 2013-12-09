// MESSAGE AP_ADC PACKING

#define MAVLINK_MSG_ID_AP_ADC 153

typedef struct __mavlink_ap_adc_t
{
 uint16_t adc1; ///< ADC output 1
 uint16_t adc2; ///< ADC output 2
 uint16_t adc3; ///< ADC output 3
 uint16_t adc4; ///< ADC output 4
 uint16_t adc5; ///< ADC output 5
 uint16_t adc6; ///< ADC output 6
} mavlink_ap_adc_t;

#define MAVLINK_MSG_ID_AP_ADC_LEN 12
#define MAVLINK_MSG_ID_153_LEN 12

#define MAVLINK_MSG_ID_AP_ADC_CRC 188
#define MAVLINK_MSG_ID_153_CRC 188



#define MAVLINK_MESSAGE_INFO_AP_ADC { \
	"AP_ADC", \
	6, \
	{  { "adc1", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_ap_adc_t, adc1) }, \
         { "adc2", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_ap_adc_t, adc2) }, \
         { "adc3", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_ap_adc_t, adc3) }, \
         { "adc4", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_ap_adc_t, adc4) }, \
         { "adc5", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_ap_adc_t, adc5) }, \
         { "adc6", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_ap_adc_t, adc6) }, \
         } \
}


/**
 * @brief Pack a ap_adc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param adc1 ADC output 1
 * @param adc2 ADC output 2
 * @param adc3 ADC output 3
 * @param adc4 ADC output 4
 * @param adc5 ADC output 5
 * @param adc6 ADC output 6
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ap_adc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AP_ADC_LEN];
	_mav_put_uint16_t(buf, 0, adc1);
	_mav_put_uint16_t(buf, 2, adc2);
	_mav_put_uint16_t(buf, 4, adc3);
	_mav_put_uint16_t(buf, 6, adc4);
	_mav_put_uint16_t(buf, 8, adc5);
	_mav_put_uint16_t(buf, 10, adc6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AP_ADC_LEN);
#else
	mavlink_ap_adc_t packet;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.adc5 = adc5;
	packet.adc6 = adc6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AP_ADC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AP_ADC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AP_ADC_LEN, MAVLINK_MSG_ID_AP_ADC_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AP_ADC_LEN);
#endif
}

/**
 * @brief Pack a ap_adc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adc1 ADC output 1
 * @param adc2 ADC output 2
 * @param adc3 ADC output 3
 * @param adc4 ADC output 4
 * @param adc5 ADC output 5
 * @param adc6 ADC output 6
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ap_adc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t adc1,uint16_t adc2,uint16_t adc3,uint16_t adc4,uint16_t adc5,uint16_t adc6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AP_ADC_LEN];
	_mav_put_uint16_t(buf, 0, adc1);
	_mav_put_uint16_t(buf, 2, adc2);
	_mav_put_uint16_t(buf, 4, adc3);
	_mav_put_uint16_t(buf, 6, adc4);
	_mav_put_uint16_t(buf, 8, adc5);
	_mav_put_uint16_t(buf, 10, adc6);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AP_ADC_LEN);
#else
	mavlink_ap_adc_t packet;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.adc5 = adc5;
	packet.adc6 = adc6;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AP_ADC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_AP_ADC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AP_ADC_LEN, MAVLINK_MSG_ID_AP_ADC_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AP_ADC_LEN);
#endif
}

/**
 * @brief Encode a ap_adc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ap_adc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ap_adc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ap_adc_t* ap_adc)
{
	return mavlink_msg_ap_adc_pack(system_id, component_id, msg, ap_adc->adc1, ap_adc->adc2, ap_adc->adc3, ap_adc->adc4, ap_adc->adc5, ap_adc->adc6);
}

/**
 * @brief Encode a ap_adc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ap_adc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ap_adc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ap_adc_t* ap_adc)
{
	return mavlink_msg_ap_adc_pack_chan(system_id, component_id, chan, msg, ap_adc->adc1, ap_adc->adc2, ap_adc->adc3, ap_adc->adc4, ap_adc->adc5, ap_adc->adc6);
}

/**
 * @brief Send a ap_adc message
 * @param chan MAVLink channel to send the message
 *
 * @param adc1 ADC output 1
 * @param adc2 ADC output 2
 * @param adc3 ADC output 3
 * @param adc4 ADC output 4
 * @param adc5 ADC output 5
 * @param adc6 ADC output 6
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ap_adc_send(mavlink_channel_t chan, uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4, uint16_t adc5, uint16_t adc6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_AP_ADC_LEN];
	_mav_put_uint16_t(buf, 0, adc1);
	_mav_put_uint16_t(buf, 2, adc2);
	_mav_put_uint16_t(buf, 4, adc3);
	_mav_put_uint16_t(buf, 6, adc4);
	_mav_put_uint16_t(buf, 8, adc5);
	_mav_put_uint16_t(buf, 10, adc6);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ADC, buf, MAVLINK_MSG_ID_AP_ADC_LEN, MAVLINK_MSG_ID_AP_ADC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ADC, buf, MAVLINK_MSG_ID_AP_ADC_LEN);
#endif
#else
	mavlink_ap_adc_t packet;
	packet.adc1 = adc1;
	packet.adc2 = adc2;
	packet.adc3 = adc3;
	packet.adc4 = adc4;
	packet.adc5 = adc5;
	packet.adc6 = adc6;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ADC, (const char *)&packet, MAVLINK_MSG_ID_AP_ADC_LEN, MAVLINK_MSG_ID_AP_ADC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AP_ADC, (const char *)&packet, MAVLINK_MSG_ID_AP_ADC_LEN);
#endif
#endif
}

#endif

// MESSAGE AP_ADC UNPACKING


/**
 * @brief Get field adc1 from ap_adc message
 *
 * @return ADC output 1
 */
static inline uint16_t mavlink_msg_ap_adc_get_adc1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field adc2 from ap_adc message
 *
 * @return ADC output 2
 */
static inline uint16_t mavlink_msg_ap_adc_get_adc2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field adc3 from ap_adc message
 *
 * @return ADC output 3
 */
static inline uint16_t mavlink_msg_ap_adc_get_adc3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field adc4 from ap_adc message
 *
 * @return ADC output 4
 */
static inline uint16_t mavlink_msg_ap_adc_get_adc4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field adc5 from ap_adc message
 *
 * @return ADC output 5
 */
static inline uint16_t mavlink_msg_ap_adc_get_adc5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field adc6 from ap_adc message
 *
 * @return ADC output 6
 */
static inline uint16_t mavlink_msg_ap_adc_get_adc6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Decode a ap_adc message into a struct
 *
 * @param msg The message to decode
 * @param ap_adc C-struct to decode the message contents into
 */
static inline void mavlink_msg_ap_adc_decode(const mavlink_message_t* msg, mavlink_ap_adc_t* ap_adc)
{
#if MAVLINK_NEED_BYTE_SWAP
	ap_adc->adc1 = mavlink_msg_ap_adc_get_adc1(msg);
	ap_adc->adc2 = mavlink_msg_ap_adc_get_adc2(msg);
	ap_adc->adc3 = mavlink_msg_ap_adc_get_adc3(msg);
	ap_adc->adc4 = mavlink_msg_ap_adc_get_adc4(msg);
	ap_adc->adc5 = mavlink_msg_ap_adc_get_adc5(msg);
	ap_adc->adc6 = mavlink_msg_ap_adc_get_adc6(msg);
#else
	memcpy(ap_adc, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_AP_ADC_LEN);
#endif
}
