// MESSAGE HIL_STATE PACKING

#define MAVLINK_MSG_ID_HIL_STATE 67

typedef struct __mavlink_hil_state_t 
{
	uint64_t usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	float roll; ///< Roll angle (rad)
	float pitch; ///< Pitch angle (rad)
	float yaw; ///< Yaw angle (rad)
	float rollspeed; ///< Roll angular speed (rad/s)
	float pitchspeed; ///< Pitch angular speed (rad/s)
	float yawspeed; ///< Yaw angular speed (rad/s)
	int32_t lat; ///< Latitude, expressed as * 1E7
	int32_t lon; ///< Longitude, expressed as * 1E7
	int32_t alt; ///< Altitude in meters, expressed as * 1000 (millimeters)
	int16_t vx; ///< Ground X Speed (Latitude), expressed as m/s * 100
	int16_t vy; ///< Ground Y Speed (Longitude), expressed as m/s * 100
	int16_t vz; ///< Ground Z Speed (Altitude), expressed as m/s * 100
	int16_t xacc; ///< X acceleration (mg)
	int16_t yacc; ///< Y acceleration (mg)
	int16_t zacc; ///< Z acceleration (mg)

} mavlink_hil_state_t;



/**
 * @brief Pack a hil_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_HIL_STATE;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	i += put_float_by_index(roll, i, msg->payload); // Roll angle (rad)
	i += put_float_by_index(pitch, i, msg->payload); // Pitch angle (rad)
	i += put_float_by_index(yaw, i, msg->payload); // Yaw angle (rad)
	i += put_float_by_index(rollspeed, i, msg->payload); // Roll angular speed (rad/s)
	i += put_float_by_index(pitchspeed, i, msg->payload); // Pitch angular speed (rad/s)
	i += put_float_by_index(yawspeed, i, msg->payload); // Yaw angular speed (rad/s)
	i += put_int32_t_by_index(lat, i, msg->payload); // Latitude, expressed as * 1E7
	i += put_int32_t_by_index(lon, i, msg->payload); // Longitude, expressed as * 1E7
	i += put_int32_t_by_index(alt, i, msg->payload); // Altitude in meters, expressed as * 1000 (millimeters)
	i += put_int16_t_by_index(vx, i, msg->payload); // Ground X Speed (Latitude), expressed as m/s * 100
	i += put_int16_t_by_index(vy, i, msg->payload); // Ground Y Speed (Longitude), expressed as m/s * 100
	i += put_int16_t_by_index(vz, i, msg->payload); // Ground Z Speed (Altitude), expressed as m/s * 100
	i += put_int16_t_by_index(xacc, i, msg->payload); // X acceleration (mg)
	i += put_int16_t_by_index(yacc, i, msg->payload); // Y acceleration (mg)
	i += put_int16_t_by_index(zacc, i, msg->payload); // Z acceleration (mg)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a hil_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_HIL_STATE;

	i += put_uint64_t_by_index(usec, i, msg->payload); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	i += put_float_by_index(roll, i, msg->payload); // Roll angle (rad)
	i += put_float_by_index(pitch, i, msg->payload); // Pitch angle (rad)
	i += put_float_by_index(yaw, i, msg->payload); // Yaw angle (rad)
	i += put_float_by_index(rollspeed, i, msg->payload); // Roll angular speed (rad/s)
	i += put_float_by_index(pitchspeed, i, msg->payload); // Pitch angular speed (rad/s)
	i += put_float_by_index(yawspeed, i, msg->payload); // Yaw angular speed (rad/s)
	i += put_int32_t_by_index(lat, i, msg->payload); // Latitude, expressed as * 1E7
	i += put_int32_t_by_index(lon, i, msg->payload); // Longitude, expressed as * 1E7
	i += put_int32_t_by_index(alt, i, msg->payload); // Altitude in meters, expressed as * 1000 (millimeters)
	i += put_int16_t_by_index(vx, i, msg->payload); // Ground X Speed (Latitude), expressed as m/s * 100
	i += put_int16_t_by_index(vy, i, msg->payload); // Ground Y Speed (Longitude), expressed as m/s * 100
	i += put_int16_t_by_index(vz, i, msg->payload); // Ground Z Speed (Altitude), expressed as m/s * 100
	i += put_int16_t_by_index(xacc, i, msg->payload); // X acceleration (mg)
	i += put_int16_t_by_index(yacc, i, msg->payload); // Y acceleration (mg)
	i += put_int16_t_by_index(zacc, i, msg->payload); // Z acceleration (mg)

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a hil_state struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_state_t* hil_state)
{
	return mavlink_msg_hil_state_pack(system_id, component_id, msg, hil_state->usec, hil_state->roll, hil_state->pitch, hil_state->yaw, hil_state->rollspeed, hil_state->pitchspeed, hil_state->yawspeed, hil_state->lat, hil_state->lon, hil_state->alt, hil_state->vx, hil_state->vy, hil_state->vz, hil_state->xacc, hil_state->yacc, hil_state->zacc);
}

/**
 * @brief Send a hil_state message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_state_send(mavlink_channel_t chan, uint64_t usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, int16_t xacc, int16_t yacc, int16_t zacc)
{
	mavlink_message_t msg;
	mavlink_msg_hil_state_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, usec, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, xacc, yacc, zacc);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE HIL_STATE UNPACKING

/**
 * @brief Get field usec from hil_state message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_hil_state_get_usec(const mavlink_message_t* msg)
{
	generic_64bit r;
	r.b[7] = (msg->payload)[0];
	r.b[6] = (msg->payload)[1];
	r.b[5] = (msg->payload)[2];
	r.b[4] = (msg->payload)[3];
	r.b[3] = (msg->payload)[4];
	r.b[2] = (msg->payload)[5];
	r.b[1] = (msg->payload)[6];
	r.b[0] = (msg->payload)[7];
	return (uint64_t)r.ll;
}

/**
 * @brief Get field roll from hil_state message
 *
 * @return Roll angle (rad)
 */
static inline float mavlink_msg_hil_state_get_roll(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitch from hil_state message
 *
 * @return Pitch angle (rad)
 */
static inline float mavlink_msg_hil_state_get_pitch(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yaw from hil_state message
 *
 * @return Yaw angle (rad)
 */
static inline float mavlink_msg_hil_state_get_yaw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field rollspeed from hil_state message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_hil_state_get_rollspeed(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field pitchspeed from hil_state message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_hil_state_get_pitchspeed(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field yawspeed from hil_state message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_hil_state_get_yawspeed(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field lat from hil_state message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_hil_state_get_lat(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field lon from hil_state message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_hil_state_get_lon(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field alt from hil_state message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_hil_state_get_alt(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field vx from hil_state message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_hil_state_get_vx(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field vy from hil_state message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_hil_state_get_vy(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field vz from hil_state message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
static inline int16_t mavlink_msg_hil_state_get_vz(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field xacc from hil_state message
 *
 * @return X acceleration (mg)
 */
static inline int16_t mavlink_msg_hil_state_get_xacc(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field yacc from hil_state message
 *
 * @return Y acceleration (mg)
 */
static inline int16_t mavlink_msg_hil_state_get_yacc(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field zacc from hil_state message
 *
 * @return Z acceleration (mg)
 */
static inline int16_t mavlink_msg_hil_state_get_zacc(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Decode a hil_state message into a struct
 *
 * @param msg The message to decode
 * @param hil_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_state_decode(const mavlink_message_t* msg, mavlink_hil_state_t* hil_state)
{
	hil_state->usec = mavlink_msg_hil_state_get_usec(msg);
	hil_state->roll = mavlink_msg_hil_state_get_roll(msg);
	hil_state->pitch = mavlink_msg_hil_state_get_pitch(msg);
	hil_state->yaw = mavlink_msg_hil_state_get_yaw(msg);
	hil_state->rollspeed = mavlink_msg_hil_state_get_rollspeed(msg);
	hil_state->pitchspeed = mavlink_msg_hil_state_get_pitchspeed(msg);
	hil_state->yawspeed = mavlink_msg_hil_state_get_yawspeed(msg);
	hil_state->lat = mavlink_msg_hil_state_get_lat(msg);
	hil_state->lon = mavlink_msg_hil_state_get_lon(msg);
	hil_state->alt = mavlink_msg_hil_state_get_alt(msg);
	hil_state->vx = mavlink_msg_hil_state_get_vx(msg);
	hil_state->vy = mavlink_msg_hil_state_get_vy(msg);
	hil_state->vz = mavlink_msg_hil_state_get_vz(msg);
	hil_state->xacc = mavlink_msg_hil_state_get_xacc(msg);
	hil_state->yacc = mavlink_msg_hil_state_get_yacc(msg);
	hil_state->zacc = mavlink_msg_hil_state_get_zacc(msg);
}
