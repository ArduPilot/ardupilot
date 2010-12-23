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



/**
 * @brief Send a gps_date_time message
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
static inline uint16_t mavlink_msg_gps_date_time_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t visSat)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GPS_DATE_TIME;

	i += put_uint8_t_by_index(year, i, msg->payload); //Year reported by Gps 
	i += put_uint8_t_by_index(month, i, msg->payload); //Month reported by Gps 
	i += put_uint8_t_by_index(day, i, msg->payload); //Day reported by Gps 
	i += put_uint8_t_by_index(hour, i, msg->payload); //Hour reported by Gps 
	i += put_uint8_t_by_index(min, i, msg->payload); //Min reported by Gps 
	i += put_uint8_t_by_index(sec, i, msg->payload); //Sec reported by Gps  
	i += put_uint8_t_by_index(visSat, i, msg->payload); //Visible sattelites reported by Gps  

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_gps_date_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_date_time_t* gps_date_time)
{
	return mavlink_msg_gps_date_time_pack(system_id, component_id, msg, gps_date_time->year, gps_date_time->month, gps_date_time->day, gps_date_time->hour, gps_date_time->min, gps_date_time->sec, gps_date_time->visSat);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_date_time_send(mavlink_channel_t chan, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t visSat)
{
	mavlink_message_t msg;
	mavlink_msg_gps_date_time_pack(mavlink_system.sysid, mavlink_system.compid, &msg, year, month, day, hour, min, sec, visSat);
	mavlink_send_uart(chan, &msg);
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
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field month from gps_date_time message
 *
 * @return Month reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_month(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field day from gps_date_time message
 *
 * @return Day reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_day(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field hour from gps_date_time message
 *
 * @return Hour reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_hour(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field min from gps_date_time message
 *
 * @return Min reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_min(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field sec from gps_date_time message
 *
 * @return Sec reported by Gps  
 */
static inline uint8_t mavlink_msg_gps_date_time_get_sec(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field visSat from gps_date_time message
 *
 * @return Visible sattelites reported by Gps  
 */
static inline uint8_t mavlink_msg_gps_date_time_get_visSat(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

static inline void mavlink_msg_gps_date_time_decode(const mavlink_message_t* msg, mavlink_gps_date_time_t* gps_date_time)
{
	gps_date_time->year = mavlink_msg_gps_date_time_get_year(msg);
	gps_date_time->month = mavlink_msg_gps_date_time_get_month(msg);
	gps_date_time->day = mavlink_msg_gps_date_time_get_day(msg);
	gps_date_time->hour = mavlink_msg_gps_date_time_get_hour(msg);
	gps_date_time->min = mavlink_msg_gps_date_time_get_min(msg);
	gps_date_time->sec = mavlink_msg_gps_date_time_get_sec(msg);
	gps_date_time->visSat = mavlink_msg_gps_date_time_get_visSat(msg);
}
