// MESSAGE PM_ELEC PACKING

#define MAVLINK_MSG_ID_PM_ELEC 188

typedef struct __mavlink_pm_elec_t
{
 float PwCons; ///< current power consumption
 float BatStat; ///< battery status
 float PwGen[3]; ///< Power generation from each module
} mavlink_pm_elec_t;

#define MAVLINK_MSG_ID_PM_ELEC_LEN 20
#define MAVLINK_MSG_ID_188_LEN 20

#define MAVLINK_MSG_PM_ELEC_FIELD_PWGEN_LEN 3

#define MAVLINK_MESSAGE_INFO_PM_ELEC { \
	"PM_ELEC", \
	3, \
	{  { "PwCons", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pm_elec_t, PwCons) }, \
         { "BatStat", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pm_elec_t, BatStat) }, \
         { "PwGen", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_pm_elec_t, PwGen) }, \
         } \
}


/**
 * @brief Pack a pm_elec message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param PwCons current power consumption
 * @param BatStat battery status
 * @param PwGen Power generation from each module
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pm_elec_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float PwCons, float BatStat, const float *PwGen)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_float(buf, 0, PwCons);
	_mav_put_float(buf, 4, BatStat);
	_mav_put_float_array(buf, 8, PwGen, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 20);
#else
	mavlink_pm_elec_t packet;
	packet.PwCons = PwCons;
	packet.BatStat = BatStat;
	mav_array_memcpy(packet.PwGen, PwGen, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_PM_ELEC;
	return mavlink_finalize_message(msg, system_id, component_id, 20, 170);
}

/**
 * @brief Pack a pm_elec message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param PwCons current power consumption
 * @param BatStat battery status
 * @param PwGen Power generation from each module
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pm_elec_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float PwCons,float BatStat,const float *PwGen)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_float(buf, 0, PwCons);
	_mav_put_float(buf, 4, BatStat);
	_mav_put_float_array(buf, 8, PwGen, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 20);
#else
	mavlink_pm_elec_t packet;
	packet.PwCons = PwCons;
	packet.BatStat = BatStat;
	mav_array_memcpy(packet.PwGen, PwGen, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_PM_ELEC;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 170);
}

/**
 * @brief Encode a pm_elec struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pm_elec C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pm_elec_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pm_elec_t* pm_elec)
{
	return mavlink_msg_pm_elec_pack(system_id, component_id, msg, pm_elec->PwCons, pm_elec->BatStat, pm_elec->PwGen);
}

/**
 * @brief Send a pm_elec message
 * @param chan MAVLink channel to send the message
 *
 * @param PwCons current power consumption
 * @param BatStat battery status
 * @param PwGen Power generation from each module
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pm_elec_send(mavlink_channel_t chan, float PwCons, float BatStat, const float *PwGen)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_float(buf, 0, PwCons);
	_mav_put_float(buf, 4, BatStat);
	_mav_put_float_array(buf, 8, PwGen, 3);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PM_ELEC, buf, 20, 170);
#else
	mavlink_pm_elec_t packet;
	packet.PwCons = PwCons;
	packet.BatStat = BatStat;
	mav_array_memcpy(packet.PwGen, PwGen, sizeof(float)*3);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PM_ELEC, (const char *)&packet, 20, 170);
#endif
}

#endif

// MESSAGE PM_ELEC UNPACKING


/**
 * @brief Get field PwCons from pm_elec message
 *
 * @return current power consumption
 */
static inline float mavlink_msg_pm_elec_get_PwCons(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field BatStat from pm_elec message
 *
 * @return battery status
 */
static inline float mavlink_msg_pm_elec_get_BatStat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field PwGen from pm_elec message
 *
 * @return Power generation from each module
 */
static inline uint16_t mavlink_msg_pm_elec_get_PwGen(const mavlink_message_t* msg, float *PwGen)
{
	return _MAV_RETURN_float_array(msg, PwGen, 3,  8);
}

/**
 * @brief Decode a pm_elec message into a struct
 *
 * @param msg The message to decode
 * @param pm_elec C-struct to decode the message contents into
 */
static inline void mavlink_msg_pm_elec_decode(const mavlink_message_t* msg, mavlink_pm_elec_t* pm_elec)
{
#if MAVLINK_NEED_BYTE_SWAP
	pm_elec->PwCons = mavlink_msg_pm_elec_get_PwCons(msg);
	pm_elec->BatStat = mavlink_msg_pm_elec_get_BatStat(msg);
	mavlink_msg_pm_elec_get_PwGen(msg, pm_elec->PwGen);
#else
	memcpy(pm_elec, _MAV_PAYLOAD(msg), 20);
#endif
}
