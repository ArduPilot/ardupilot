// MESSAGE RAW_PRESSURE PACKING

#define MAVLINK_MSG_ID_RAW_PRESSURE 29

typedef struct __mavlink_raw_pressure_t 
{
	uint64_t usec; ///< Timestamp (microseconds since UNIX epoch)
	int32_t press_abs; ///< Absolute pressure (hectopascal)
	int32_t press_diff1; ///< Differential pressure 1 (hectopascal)
	int32_t press_diff2; ///< Differential pressure 2 (hectopascal)

} mavlink_raw_pressure_t;



/**
 * @brief Send a raw_pressure message
 *
 * @param usec Timestamp (microseconds since UNIX epoch)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff1 Differential pressure 1 (hectopascal)
 * @param press_diff2 Differential pressure 2 (hectopascal)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_pressure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, int32_t press_abs, int32_t press_diff1, int32_t press_diff2)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_RAW_PRESSURE;

	i += put_uint64_t_by_index(usec, i, msg->payload); //Timestamp (microseconds since UNIX epoch)
	i += put_int32_t_by_index(press_abs, i, msg->payload); //Absolute pressure (hectopascal)
	i += put_int32_t_by_index(press_diff1, i, msg->payload); //Differential pressure 1 (hectopascal)
	i += put_int32_t_by_index(press_diff2, i, msg->payload); //Differential pressure 2 (hectopascal)

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_raw_pressure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_pressure_t* raw_pressure)
{
	return mavlink_msg_raw_pressure_pack(system_id, component_id, msg, raw_pressure->usec, raw_pressure->press_abs, raw_pressure->press_diff1, raw_pressure->press_diff2);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_pressure_send(mavlink_channel_t chan, uint64_t usec, int32_t press_abs, int32_t press_diff1, int32_t press_diff2)
{
	mavlink_message_t msg;
	mavlink_msg_raw_pressure_pack(mavlink_system.sysid, mavlink_system.compid, &msg, usec, press_abs, press_diff1, press_diff2);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE RAW_PRESSURE UNPACKING

/**
 * @brief Get field usec from raw_pressure message
 *
 * @return Timestamp (microseconds since UNIX epoch)
 */
static inline uint64_t mavlink_msg_raw_pressure_get_usec(const mavlink_message_t* msg)
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
 * @brief Get field press_abs from raw_pressure message
 *
 * @return Absolute pressure (hectopascal)
 */
static inline int32_t mavlink_msg_raw_pressure_get_press_abs(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field press_diff1 from raw_pressure message
 *
 * @return Differential pressure 1 (hectopascal)
 */
static inline int32_t mavlink_msg_raw_pressure_get_press_diff1(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(int32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(int32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(int32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(int32_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field press_diff2 from raw_pressure message
 *
 * @return Differential pressure 2 (hectopascal)
 */
static inline int32_t mavlink_msg_raw_pressure_get_press_diff2(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint64_t)+sizeof(int32_t)+sizeof(int32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint64_t)+sizeof(int32_t)+sizeof(int32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint64_t)+sizeof(int32_t)+sizeof(int32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint64_t)+sizeof(int32_t)+sizeof(int32_t))[3];
	return (int32_t)r.i;
}

static inline void mavlink_msg_raw_pressure_decode(const mavlink_message_t* msg, mavlink_raw_pressure_t* raw_pressure)
{
	raw_pressure->usec = mavlink_msg_raw_pressure_get_usec(msg);
	raw_pressure->press_abs = mavlink_msg_raw_pressure_get_press_abs(msg);
	raw_pressure->press_diff1 = mavlink_msg_raw_pressure_get_press_diff1(msg);
	raw_pressure->press_diff2 = mavlink_msg_raw_pressure_get_press_diff2(msg);
}
