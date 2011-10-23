// MESSAGE GPS_DATE_TIME PACKING

#define MAVLINK_MSG_ID_GPS_DATE_TIME 179

typedef struct __mavlink_gps_date_time_t
{
 uint8_t year; ///< Year reported by Gps 
 uint8_t month; ///< Month reported by Gps 
 uint8_t day; ///< Day reported by Gps 
 uint8_t hour; ///< Hour reported by Gps 
 uint8_t min; ///< Min reported by Gps 
 uint8_t sec; ///< Sec reported by Gps  
 uint8_t visSat; ///< Visible sattelites reported by Gps  
} mavlink_gps_date_time_t;

#define MAVLINK_MSG_ID_GPS_DATE_TIME_LEN 7
#define MAVLINK_MSG_ID_179_LEN 7



#define MAVLINK_MESSAGE_INFO_GPS_DATE_TIME { \
	"GPS_DATE_TIME", \
	7, \
	{  { "year", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_date_time_t, year) }, \
         { "month", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gps_date_time_t, month) }, \
         { "day", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gps_date_time_t, day) }, \
         { "hour", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_gps_date_time_t, hour) }, \
         { "min", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_gps_date_time_t, min) }, \
         { "sec", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_gps_date_time_t, sec) }, \
         { "visSat", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_gps_date_time_t, visSat) }, \
         } \
}


/**
 * @brief Pack a gps_date_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param year Year reported by Gps 
 * @param month Month reported by Gps 
 * @param day Day reported by Gps 
 * @param hour Hour reported by Gps 
 * @param min Min reported by Gps 
 * @param sec Sec reported by Gps  
 * @param visSat Visible sattelites reported by Gps  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_date_time_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t visSat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_uint8_t(buf, 0, year);
	_mav_put_uint8_t(buf, 1, month);
	_mav_put_uint8_t(buf, 2, day);
	_mav_put_uint8_t(buf, 3, hour);
	_mav_put_uint8_t(buf, 4, min);
	_mav_put_uint8_t(buf, 5, sec);
	_mav_put_uint8_t(buf, 6, visSat);

        memcpy(_MAV_PAYLOAD(msg), buf, 7);
#else
	mavlink_gps_date_time_t packet;
	packet.year = year;
	packet.month = month;
	packet.day = day;
	packet.hour = hour;
	packet.min = min;
	packet.sec = sec;
	packet.visSat = visSat;

        memcpy(_MAV_PAYLOAD(msg), &packet, 7);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_DATE_TIME;
	return mavlink_finalize_message(msg, system_id, component_id, 7, 16);
}

/**
 * @brief Pack a gps_date_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param year Year reported by Gps 
 * @param month Month reported by Gps 
 * @param day Day reported by Gps 
 * @param hour Hour reported by Gps 
 * @param min Min reported by Gps 
 * @param sec Sec reported by Gps  
 * @param visSat Visible sattelites reported by Gps  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_date_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t min,uint8_t sec,uint8_t visSat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_uint8_t(buf, 0, year);
	_mav_put_uint8_t(buf, 1, month);
	_mav_put_uint8_t(buf, 2, day);
	_mav_put_uint8_t(buf, 3, hour);
	_mav_put_uint8_t(buf, 4, min);
	_mav_put_uint8_t(buf, 5, sec);
	_mav_put_uint8_t(buf, 6, visSat);

        memcpy(_MAV_PAYLOAD(msg), buf, 7);
#else
	mavlink_gps_date_time_t packet;
	packet.year = year;
	packet.month = month;
	packet.day = day;
	packet.hour = hour;
	packet.min = min;
	packet.sec = sec;
	packet.visSat = visSat;

        memcpy(_MAV_PAYLOAD(msg), &packet, 7);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_DATE_TIME;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 7, 16);
}

/**
 * @brief Encode a gps_date_time struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_date_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_date_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_date_time_t* gps_date_time)
{
	return mavlink_msg_gps_date_time_pack(system_id, component_id, msg, gps_date_time->year, gps_date_time->month, gps_date_time->day, gps_date_time->hour, gps_date_time->min, gps_date_time->sec, gps_date_time->visSat);
}

/**
 * @brief Send a gps_date_time message
 * @param chan MAVLink channel to send the message
 *
 * @param year Year reported by Gps 
 * @param month Month reported by Gps 
 * @param day Day reported by Gps 
 * @param hour Hour reported by Gps 
 * @param min Min reported by Gps 
 * @param sec Sec reported by Gps  
 * @param visSat Visible sattelites reported by Gps  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_date_time_send(mavlink_channel_t chan, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t visSat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_uint8_t(buf, 0, year);
	_mav_put_uint8_t(buf, 1, month);
	_mav_put_uint8_t(buf, 2, day);
	_mav_put_uint8_t(buf, 3, hour);
	_mav_put_uint8_t(buf, 4, min);
	_mav_put_uint8_t(buf, 5, sec);
	_mav_put_uint8_t(buf, 6, visSat);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATE_TIME, buf, 7, 16);
#else
	mavlink_gps_date_time_t packet;
	packet.year = year;
	packet.month = month;
	packet.day = day;
	packet.hour = hour;
	packet.min = min;
	packet.sec = sec;
	packet.visSat = visSat;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATE_TIME, (const char *)&packet, 7, 16);
#endif
}

#endif

// MESSAGE GPS_DATE_TIME UNPACKING


/**
 * @brief Get field year from gps_date_time message
 *
 * @return Year reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_year(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field month from gps_date_time message
 *
 * @return Month reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_month(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field day from gps_date_time message
 *
 * @return Day reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_day(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field hour from gps_date_time message
 *
 * @return Hour reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_hour(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field min from gps_date_time message
 *
 * @return Min reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_min(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field sec from gps_date_time message
 *
 * @return Sec reported by Gps  
 */
static inline uint8_t mavlink_msg_gps_date_time_get_sec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field visSat from gps_date_time message
 *
 * @return Visible sattelites reported by Gps  
 */
static inline uint8_t mavlink_msg_gps_date_time_get_visSat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a gps_date_time message into a struct
 *
 * @param msg The message to decode
 * @param gps_date_time C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_date_time_decode(const mavlink_message_t* msg, mavlink_gps_date_time_t* gps_date_time)
{
#if MAVLINK_NEED_BYTE_SWAP
	gps_date_time->year = mavlink_msg_gps_date_time_get_year(msg);
	gps_date_time->month = mavlink_msg_gps_date_time_get_month(msg);
	gps_date_time->day = mavlink_msg_gps_date_time_get_day(msg);
	gps_date_time->hour = mavlink_msg_gps_date_time_get_hour(msg);
	gps_date_time->min = mavlink_msg_gps_date_time_get_min(msg);
	gps_date_time->sec = mavlink_msg_gps_date_time_get_sec(msg);
	gps_date_time->visSat = mavlink_msg_gps_date_time_get_visSat(msg);
#else
	memcpy(gps_date_time, _MAV_PAYLOAD(msg), 7);
#endif
}
