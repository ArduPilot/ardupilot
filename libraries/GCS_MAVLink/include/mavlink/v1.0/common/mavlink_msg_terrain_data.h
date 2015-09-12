// MESSAGE TERRAIN_DATA PACKING

#define MAVLINK_MSG_ID_TERRAIN_DATA 134

typedef struct __mavlink_terrain_data_t
{
 int32_t lat; /*< Latitude of SW corner of first grid (degrees *10^7)*/
 int32_t lon; /*< Longitude of SW corner of first grid (in degrees *10^7)*/
 uint16_t grid_spacing; /*< Grid spacing in meters*/
 int16_t data[16]; /*< Terrain data in meters AMSL*/
 uint8_t gridbit; /*< bit within the terrain request mask*/
} mavlink_terrain_data_t;

#define MAVLINK_MSG_ID_TERRAIN_DATA_LEN 43
#define MAVLINK_MSG_ID_134_LEN 43

#define MAVLINK_MSG_ID_TERRAIN_DATA_CRC 229
#define MAVLINK_MSG_ID_134_CRC 229

#define MAVLINK_MSG_TERRAIN_DATA_FIELD_DATA_LEN 16

#define MAVLINK_MESSAGE_INFO_TERRAIN_DATA { \
	"TERRAIN_DATA", \
	5, \
	{  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_terrain_data_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_terrain_data_t, lon) }, \
         { "grid_spacing", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_terrain_data_t, grid_spacing) }, \
         { "data", NULL, MAVLINK_TYPE_INT16_T, 16, 10, offsetof(mavlink_terrain_data_t, data) }, \
         { "gridbit", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_terrain_data_t, gridbit) }, \
         } \
}


/**
 * @brief Pack a terrain_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat Latitude of SW corner of first grid (degrees *10^7)
 * @param lon Longitude of SW corner of first grid (in degrees *10^7)
 * @param grid_spacing Grid spacing in meters
 * @param gridbit bit within the terrain request mask
 * @param data Terrain data in meters AMSL
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_terrain_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t lat, int32_t lon, uint16_t grid_spacing, uint8_t gridbit, const int16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TERRAIN_DATA_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_uint16_t(buf, 8, grid_spacing);
	_mav_put_uint8_t(buf, 42, gridbit);
	_mav_put_int16_t_array(buf, 10, data, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#else
	mavlink_terrain_data_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.grid_spacing = grid_spacing;
	packet.gridbit = gridbit;
	mav_array_memcpy(packet.data, data, sizeof(int16_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TERRAIN_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TERRAIN_DATA_LEN, MAVLINK_MSG_ID_TERRAIN_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#endif
}

/**
 * @brief Pack a terrain_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat Latitude of SW corner of first grid (degrees *10^7)
 * @param lon Longitude of SW corner of first grid (in degrees *10^7)
 * @param grid_spacing Grid spacing in meters
 * @param gridbit bit within the terrain request mask
 * @param data Terrain data in meters AMSL
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_terrain_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t lat,int32_t lon,uint16_t grid_spacing,uint8_t gridbit,const int16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TERRAIN_DATA_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_uint16_t(buf, 8, grid_spacing);
	_mav_put_uint8_t(buf, 42, gridbit);
	_mav_put_int16_t_array(buf, 10, data, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#else
	mavlink_terrain_data_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.grid_spacing = grid_spacing;
	packet.gridbit = gridbit;
	mav_array_memcpy(packet.data, data, sizeof(int16_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TERRAIN_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TERRAIN_DATA_LEN, MAVLINK_MSG_ID_TERRAIN_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#endif
}

/**
 * @brief Encode a terrain_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param terrain_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_terrain_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_terrain_data_t* terrain_data)
{
	return mavlink_msg_terrain_data_pack(system_id, component_id, msg, terrain_data->lat, terrain_data->lon, terrain_data->grid_spacing, terrain_data->gridbit, terrain_data->data);
}

/**
 * @brief Encode a terrain_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param terrain_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_terrain_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_terrain_data_t* terrain_data)
{
	return mavlink_msg_terrain_data_pack_chan(system_id, component_id, chan, msg, terrain_data->lat, terrain_data->lon, terrain_data->grid_spacing, terrain_data->gridbit, terrain_data->data);
}

/**
 * @brief Send a terrain_data message
 * @param chan MAVLink channel to send the message
 *
 * @param lat Latitude of SW corner of first grid (degrees *10^7)
 * @param lon Longitude of SW corner of first grid (in degrees *10^7)
 * @param grid_spacing Grid spacing in meters
 * @param gridbit bit within the terrain request mask
 * @param data Terrain data in meters AMSL
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_terrain_data_send(mavlink_channel_t chan, int32_t lat, int32_t lon, uint16_t grid_spacing, uint8_t gridbit, const int16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TERRAIN_DATA_LEN];
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_uint16_t(buf, 8, grid_spacing);
	_mav_put_uint8_t(buf, 42, gridbit);
	_mav_put_int16_t_array(buf, 10, data, 16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_DATA, buf, MAVLINK_MSG_ID_TERRAIN_DATA_LEN, MAVLINK_MSG_ID_TERRAIN_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_DATA, buf, MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#endif
#else
	mavlink_terrain_data_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.grid_spacing = grid_spacing;
	packet.gridbit = gridbit;
	mav_array_memcpy(packet.data, data, sizeof(int16_t)*16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_DATA, (const char *)&packet, MAVLINK_MSG_ID_TERRAIN_DATA_LEN, MAVLINK_MSG_ID_TERRAIN_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_DATA, (const char *)&packet, MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_TERRAIN_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_terrain_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon, uint16_t grid_spacing, uint8_t gridbit, const int16_t *data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, lat);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_uint16_t(buf, 8, grid_spacing);
	_mav_put_uint8_t(buf, 42, gridbit);
	_mav_put_int16_t_array(buf, 10, data, 16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_DATA, buf, MAVLINK_MSG_ID_TERRAIN_DATA_LEN, MAVLINK_MSG_ID_TERRAIN_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_DATA, buf, MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#endif
#else
	mavlink_terrain_data_t *packet = (mavlink_terrain_data_t *)msgbuf;
	packet->lat = lat;
	packet->lon = lon;
	packet->grid_spacing = grid_spacing;
	packet->gridbit = gridbit;
	mav_array_memcpy(packet->data, data, sizeof(int16_t)*16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_DATA, (const char *)packet, MAVLINK_MSG_ID_TERRAIN_DATA_LEN, MAVLINK_MSG_ID_TERRAIN_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TERRAIN_DATA, (const char *)packet, MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE TERRAIN_DATA UNPACKING


/**
 * @brief Get field lat from terrain_data message
 *
 * @return Latitude of SW corner of first grid (degrees *10^7)
 */
static inline int32_t mavlink_msg_terrain_data_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from terrain_data message
 *
 * @return Longitude of SW corner of first grid (in degrees *10^7)
 */
static inline int32_t mavlink_msg_terrain_data_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field grid_spacing from terrain_data message
 *
 * @return Grid spacing in meters
 */
static inline uint16_t mavlink_msg_terrain_data_get_grid_spacing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field gridbit from terrain_data message
 *
 * @return bit within the terrain request mask
 */
static inline uint8_t mavlink_msg_terrain_data_get_gridbit(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field data from terrain_data message
 *
 * @return Terrain data in meters AMSL
 */
static inline uint16_t mavlink_msg_terrain_data_get_data(const mavlink_message_t* msg, int16_t *data)
{
	return _MAV_RETURN_int16_t_array(msg, data, 16,  10);
}

/**
 * @brief Decode a terrain_data message into a struct
 *
 * @param msg The message to decode
 * @param terrain_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_terrain_data_decode(const mavlink_message_t* msg, mavlink_terrain_data_t* terrain_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	terrain_data->lat = mavlink_msg_terrain_data_get_lat(msg);
	terrain_data->lon = mavlink_msg_terrain_data_get_lon(msg);
	terrain_data->grid_spacing = mavlink_msg_terrain_data_get_grid_spacing(msg);
	mavlink_msg_terrain_data_get_data(msg, terrain_data->data);
	terrain_data->gridbit = mavlink_msg_terrain_data_get_gridbit(msg);
#else
	memcpy(terrain_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TERRAIN_DATA_LEN);
#endif
}
