// MESSAGE CPU_LOAD PACKING

#define MAVLINK_MSG_ID_CPU_LOAD 170

typedef struct __mavlink_cpu_load_t
{
 uint16_t batVolt; ///< Battery Voltage in millivolts
 uint8_t sensLoad; ///< Sensor DSC Load
 uint8_t ctrlLoad; ///< Control DSC Load
} mavlink_cpu_load_t;

#define MAVLINK_MSG_ID_CPU_LOAD_LEN 4
#define MAVLINK_MSG_ID_170_LEN 4



#define MAVLINK_MESSAGE_INFO_CPU_LOAD { \
	"CPU_LOAD", \
	3, \
	{  { "batVolt", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_cpu_load_t, batVolt) }, \
         { "sensLoad", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_cpu_load_t, sensLoad) }, \
         { "ctrlLoad", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_cpu_load_t, ctrlLoad) }, \
         } \
}


/**
 * @brief Pack a cpu_load message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sensLoad Sensor DSC Load
 * @param ctrlLoad Control DSC Load
 * @param batVolt Battery Voltage in millivolts
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cpu_load_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t sensLoad, uint8_t ctrlLoad, uint16_t batVolt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, batVolt);
	_mav_put_uint8_t(buf, 2, sensLoad);
	_mav_put_uint8_t(buf, 3, ctrlLoad);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_cpu_load_t packet;
	packet.batVolt = batVolt;
	packet.sensLoad = sensLoad;
	packet.ctrlLoad = ctrlLoad;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_CPU_LOAD;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 75);
}

/**
 * @brief Pack a cpu_load message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensLoad Sensor DSC Load
 * @param ctrlLoad Control DSC Load
 * @param batVolt Battery Voltage in millivolts
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cpu_load_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t sensLoad,uint8_t ctrlLoad,uint16_t batVolt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, batVolt);
	_mav_put_uint8_t(buf, 2, sensLoad);
	_mav_put_uint8_t(buf, 3, ctrlLoad);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_cpu_load_t packet;
	packet.batVolt = batVolt;
	packet.sensLoad = sensLoad;
	packet.ctrlLoad = ctrlLoad;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_CPU_LOAD;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 75);
}

/**
 * @brief Encode a cpu_load struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cpu_load C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cpu_load_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cpu_load_t* cpu_load)
{
	return mavlink_msg_cpu_load_pack(system_id, component_id, msg, cpu_load->sensLoad, cpu_load->ctrlLoad, cpu_load->batVolt);
}

/**
 * @brief Send a cpu_load message
 * @param chan MAVLink channel to send the message
 *
 * @param sensLoad Sensor DSC Load
 * @param ctrlLoad Control DSC Load
 * @param batVolt Battery Voltage in millivolts
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cpu_load_send(mavlink_channel_t chan, uint8_t sensLoad, uint8_t ctrlLoad, uint16_t batVolt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, batVolt);
	_mav_put_uint8_t(buf, 2, sensLoad);
	_mav_put_uint8_t(buf, 3, ctrlLoad);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_LOAD, buf, 4, 75);
#else
	mavlink_cpu_load_t packet;
	packet.batVolt = batVolt;
	packet.sensLoad = sensLoad;
	packet.ctrlLoad = ctrlLoad;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_LOAD, (const char *)&packet, 4, 75);
#endif
}

#endif

// MESSAGE CPU_LOAD UNPACKING


/**
 * @brief Get field sensLoad from cpu_load message
 *
 * @return Sensor DSC Load
 */
static inline uint8_t mavlink_msg_cpu_load_get_sensLoad(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field ctrlLoad from cpu_load message
 *
 * @return Control DSC Load
 */
static inline uint8_t mavlink_msg_cpu_load_get_ctrlLoad(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field batVolt from cpu_load message
 *
 * @return Battery Voltage in millivolts
 */
static inline uint16_t mavlink_msg_cpu_load_get_batVolt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a cpu_load message into a struct
 *
 * @param msg The message to decode
 * @param cpu_load C-struct to decode the message contents into
 */
static inline void mavlink_msg_cpu_load_decode(const mavlink_message_t* msg, mavlink_cpu_load_t* cpu_load)
{
#if MAVLINK_NEED_BYTE_SWAP
	cpu_load->batVolt = mavlink_msg_cpu_load_get_batVolt(msg);
	cpu_load->sensLoad = mavlink_msg_cpu_load_get_sensLoad(msg);
	cpu_load->ctrlLoad = mavlink_msg_cpu_load_get_ctrlLoad(msg);
#else
	memcpy(cpu_load, _MAV_PAYLOAD(msg), 4);
#endif
}
