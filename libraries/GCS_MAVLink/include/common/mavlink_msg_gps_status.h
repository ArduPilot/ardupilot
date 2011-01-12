// MESSAGE GPS_STATUS PACKING

#define MAVLINK_MSG_ID_GPS_STATUS 27

typedef struct __mavlink_gps_status_t 
{
	uint8_t satellites_visible; ///< Number of satellites visible
	int8_t satellite_prn[20]; ///< Global satellite ID
	int8_t satellite_used[20]; ///< 0: Satellite not used, 1: used for localization
	int8_t satellite_elevation[20]; ///< Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	int8_t satellite_azimuth[20]; ///< Direction of satellite, 0: 0 deg, 255: 360 deg.
	int8_t satellite_snr[20]; ///< Signal to noise ratio of satellite

} mavlink_gps_status_t;

#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_PRN_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_USED_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_ELEVATION_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_AZIMUTH_LEN 20
#define MAVLINK_MSG_GPS_STATUS_FIELD_SATELLITE_SNR_LEN 20


/**
 * @brief Pack a gps_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t satellites_visible, const int8_t* satellite_prn, const int8_t* satellite_used, const int8_t* satellite_elevation, const int8_t* satellite_azimuth, const int8_t* satellite_snr)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GPS_STATUS;

	i += put_uint8_t_by_index(satellites_visible, i, msg->payload); // Number of satellites visible
	i += put_array_by_index(satellite_prn, 20, i, msg->payload); // Global satellite ID
	i += put_array_by_index(satellite_used, 20, i, msg->payload); // 0: Satellite not used, 1: used for localization
	i += put_array_by_index(satellite_elevation, 20, i, msg->payload); // Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	i += put_array_by_index(satellite_azimuth, 20, i, msg->payload); // Direction of satellite, 0: 0 deg, 255: 360 deg.
	i += put_array_by_index(satellite_snr, 20, i, msg->payload); // Signal to noise ratio of satellite

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a gps_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t satellites_visible, const int8_t* satellite_prn, const int8_t* satellite_used, const int8_t* satellite_elevation, const int8_t* satellite_azimuth, const int8_t* satellite_snr)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GPS_STATUS;

	i += put_uint8_t_by_index(satellites_visible, i, msg->payload); // Number of satellites visible
	i += put_array_by_index(satellite_prn, 20, i, msg->payload); // Global satellite ID
	i += put_array_by_index(satellite_used, 20, i, msg->payload); // 0: Satellite not used, 1: used for localization
	i += put_array_by_index(satellite_elevation, 20, i, msg->payload); // Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	i += put_array_by_index(satellite_azimuth, 20, i, msg->payload); // Direction of satellite, 0: 0 deg, 255: 360 deg.
	i += put_array_by_index(satellite_snr, 20, i, msg->payload); // Signal to noise ratio of satellite

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a gps_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_status_t* gps_status)
{
	return mavlink_msg_gps_status_pack(system_id, component_id, msg, gps_status->satellites_visible, gps_status->satellite_prn, gps_status->satellite_used, gps_status->satellite_elevation, gps_status->satellite_azimuth, gps_status->satellite_snr);
}

/**
 * @brief Send a gps_status message
 * @param chan MAVLink channel to send the message
 *
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_status_send(mavlink_channel_t chan, uint8_t satellites_visible, const int8_t* satellite_prn, const int8_t* satellite_used, const int8_t* satellite_elevation, const int8_t* satellite_azimuth, const int8_t* satellite_snr)
{
	mavlink_message_t msg;
	mavlink_msg_gps_status_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, satellites_visible, satellite_prn, satellite_used, satellite_elevation, satellite_azimuth, satellite_snr);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE GPS_STATUS UNPACKING

/**
 * @brief Get field satellites_visible from gps_status message
 *
 * @return Number of satellites visible
 */
static inline uint8_t mavlink_msg_gps_status_get_satellites_visible(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field satellite_prn from gps_status message
 *
 * @return Global satellite ID
 */
static inline uint16_t mavlink_msg_gps_status_get_satellite_prn(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint8_t), 20);
	return 20;
}

/**
 * @brief Get field satellite_used from gps_status message
 *
 * @return 0: Satellite not used, 1: used for localization
 */
static inline uint16_t mavlink_msg_gps_status_get_satellite_used(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint8_t)+20, 20);
	return 20;
}

/**
 * @brief Get field satellite_elevation from gps_status message
 *
 * @return Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 */
static inline uint16_t mavlink_msg_gps_status_get_satellite_elevation(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint8_t)+20+20, 20);
	return 20;
}

/**
 * @brief Get field satellite_azimuth from gps_status message
 *
 * @return Direction of satellite, 0: 0 deg, 255: 360 deg.
 */
static inline uint16_t mavlink_msg_gps_status_get_satellite_azimuth(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint8_t)+20+20+20, 20);
	return 20;
}

/**
 * @brief Get field satellite_snr from gps_status message
 *
 * @return Signal to noise ratio of satellite
 */
static inline uint16_t mavlink_msg_gps_status_get_satellite_snr(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint8_t)+20+20+20+20, 20);
	return 20;
}

/**
 * @brief Decode a gps_status message into a struct
 *
 * @param msg The message to decode
 * @param gps_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_status_decode(const mavlink_message_t* msg, mavlink_gps_status_t* gps_status)
{
	gps_status->satellites_visible = mavlink_msg_gps_status_get_satellites_visible(msg);
	mavlink_msg_gps_status_get_satellite_prn(msg, gps_status->satellite_prn);
	mavlink_msg_gps_status_get_satellite_used(msg, gps_status->satellite_used);
	mavlink_msg_gps_status_get_satellite_elevation(msg, gps_status->satellite_elevation);
	mavlink_msg_gps_status_get_satellite_azimuth(msg, gps_status->satellite_azimuth);
	mavlink_msg_gps_status_get_satellite_snr(msg, gps_status->satellite_snr);
}
