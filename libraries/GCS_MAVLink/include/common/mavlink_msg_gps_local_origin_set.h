// MESSAGE GPS_LOCAL_ORIGIN_SET PACKING

#define MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET 71

typedef struct __mavlink_gps_local_origin_set_t 
{
	float latitude; ///< Latitude (WGS84)
	float longitude; ///< Longitude (WGS84)
	float altitude; ///< Altitude(WGS84)
	float x; ///< Local X coordinate in meters
	float y; ///< Local Y coordinate in meters
	float z; ///< Local Z coordinate in meters

} mavlink_gps_local_origin_set_t;



/**
 * @brief Send a gps_local_origin_set message
 *
 * @param latitude Latitude (WGS84)
 * @param longitude Longitude (WGS84)
 * @param altitude Altitude(WGS84)
 * @param x Local X coordinate in meters
 * @param y Local Y coordinate in meters
 * @param z Local Z coordinate in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_local_origin_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float latitude, float longitude, float altitude, float x, float y, float z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET;

	i += put_float_by_index(latitude, i, msg->payload); //Latitude (WGS84)
	i += put_float_by_index(longitude, i, msg->payload); //Longitude (WGS84)
	i += put_float_by_index(altitude, i, msg->payload); //Altitude(WGS84)
	i += put_float_by_index(x, i, msg->payload); //Local X coordinate in meters
	i += put_float_by_index(y, i, msg->payload); //Local Y coordinate in meters
	i += put_float_by_index(z, i, msg->payload); //Local Z coordinate in meters

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_gps_local_origin_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, float latitude, float longitude, float altitude, float x, float y, float z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET;

	i += put_float_by_index(latitude, i, msg->payload); //Latitude (WGS84)
	i += put_float_by_index(longitude, i, msg->payload); //Longitude (WGS84)
	i += put_float_by_index(altitude, i, msg->payload); //Altitude(WGS84)
	i += put_float_by_index(x, i, msg->payload); //Local X coordinate in meters
	i += put_float_by_index(y, i, msg->payload); //Local Y coordinate in meters
	i += put_float_by_index(z, i, msg->payload); //Local Z coordinate in meters

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

static inline uint16_t mavlink_msg_gps_local_origin_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_local_origin_set_t* gps_local_origin_set)
{
	return mavlink_msg_gps_local_origin_set_pack(system_id, component_id, msg, gps_local_origin_set->latitude, gps_local_origin_set->longitude, gps_local_origin_set->altitude, gps_local_origin_set->x, gps_local_origin_set->y, gps_local_origin_set->z);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_local_origin_set_send(mavlink_channel_t chan, float latitude, float longitude, float altitude, float x, float y, float z)
{
	mavlink_message_t msg;
	mavlink_msg_gps_local_origin_set_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, latitude, longitude, altitude, x, y, z);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE GPS_LOCAL_ORIGIN_SET UNPACKING

/**
 * @brief Get field latitude from gps_local_origin_set message
 *
 * @return Latitude (WGS84)
 */
static inline float mavlink_msg_gps_local_origin_set_get_latitude(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (float)r.f;
}

/**
 * @brief Get field longitude from gps_local_origin_set message
 *
 * @return Longitude (WGS84)
 */
static inline float mavlink_msg_gps_local_origin_set_get_longitude(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field altitude from gps_local_origin_set message
 *
 * @return Altitude(WGS84)
 */
static inline float mavlink_msg_gps_local_origin_set_get_altitude(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field x from gps_local_origin_set message
 *
 * @return Local X coordinate in meters
 */
static inline float mavlink_msg_gps_local_origin_set_get_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field y from gps_local_origin_set message
 *
 * @return Local Y coordinate in meters
 */
static inline float mavlink_msg_gps_local_origin_set_get_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z from gps_local_origin_set message
 *
 * @return Local Z coordinate in meters
 */
static inline float mavlink_msg_gps_local_origin_set_get_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

static inline void mavlink_msg_gps_local_origin_set_decode(const mavlink_message_t* msg, mavlink_gps_local_origin_set_t* gps_local_origin_set)
{
	gps_local_origin_set->latitude = mavlink_msg_gps_local_origin_set_get_latitude(msg);
	gps_local_origin_set->longitude = mavlink_msg_gps_local_origin_set_get_longitude(msg);
	gps_local_origin_set->altitude = mavlink_msg_gps_local_origin_set_get_altitude(msg);
	gps_local_origin_set->x = mavlink_msg_gps_local_origin_set_get_x(msg);
	gps_local_origin_set->y = mavlink_msg_gps_local_origin_set_get_y(msg);
	gps_local_origin_set->z = mavlink_msg_gps_local_origin_set_get_z(msg);
}
