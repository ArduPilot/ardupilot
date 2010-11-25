// MESSAGE PILOT_CONSOLE PACKING

#define MAVLINK_MSG_ID_PILOT_CONSOLE 194

typedef struct __mavlink_pilot_console_t 
{
	uint8_t target; ///< The system reporting the diagnostic
	uint16_t dt; ///< Pilot's console throttle command 
	uint16_t dla; ///< Pilot's console left aileron command 
	uint16_t dra; ///< Pilot's console right aileron command 
	uint16_t dr; ///< Pilot's console rudder command 
	uint16_t de; ///< Pilot's console elevator command 

} mavlink_pilot_console_t;



/**
 * @brief Send a pilot_console message
 *
 * @param target The system reporting the diagnostic
 * @param dt Pilot's console throttle command 
 * @param dla Pilot's console left aileron command 
 * @param dra Pilot's console right aileron command 
 * @param dr Pilot's console rudder command 
 * @param de Pilot's console elevator command 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pilot_console_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, uint16_t dt, uint16_t dla, uint16_t dra, uint16_t dr, uint16_t de)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_PILOT_CONSOLE;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system reporting the diagnostic
	i += put_uint16_t_by_index(dt, i, msg->payload); //Pilot's console throttle command 
	i += put_uint16_t_by_index(dla, i, msg->payload); //Pilot's console left aileron command 
	i += put_uint16_t_by_index(dra, i, msg->payload); //Pilot's console right aileron command 
	i += put_uint16_t_by_index(dr, i, msg->payload); //Pilot's console rudder command 
	i += put_uint16_t_by_index(de, i, msg->payload); //Pilot's console elevator command 

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_pilot_console_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pilot_console_t* pilot_console)
{
	return mavlink_msg_pilot_console_pack(system_id, component_id, msg, pilot_console->target, pilot_console->dt, pilot_console->dla, pilot_console->dra, pilot_console->dr, pilot_console->de);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pilot_console_send(mavlink_channel_t chan, uint8_t target, uint16_t dt, uint16_t dla, uint16_t dra, uint16_t dr, uint16_t de)
{
	mavlink_message_t msg;
	mavlink_msg_pilot_console_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, dt, dla, dra, dr, de);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE PILOT_CONSOLE UNPACKING

/**
 * @brief Get field target from pilot_console message
 *
 * @return The system reporting the diagnostic
 */
static inline uint8_t mavlink_msg_pilot_console_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field dt from pilot_console message
 *
 * @return Pilot's console throttle command 
 */
static inline uint16_t mavlink_msg_pilot_console_get_dt(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field dla from pilot_console message
 *
 * @return Pilot's console left aileron command 
 */
static inline uint16_t mavlink_msg_pilot_console_get_dla(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field dra from pilot_console message
 *
 * @return Pilot's console right aileron command 
 */
static inline uint16_t mavlink_msg_pilot_console_get_dra(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field dr from pilot_console message
 *
 * @return Pilot's console rudder command 
 */
static inline uint16_t mavlink_msg_pilot_console_get_dr(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field de from pilot_console message
 *
 * @return Pilot's console elevator command 
 */
static inline uint16_t mavlink_msg_pilot_console_get_de(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

static inline void mavlink_msg_pilot_console_decode(const mavlink_message_t* msg, mavlink_pilot_console_t* pilot_console)
{
	pilot_console->target = mavlink_msg_pilot_console_get_target(msg);
	pilot_console->dt = mavlink_msg_pilot_console_get_dt(msg);
	pilot_console->dla = mavlink_msg_pilot_console_get_dla(msg);
	pilot_console->dra = mavlink_msg_pilot_console_get_dra(msg);
	pilot_console->dr = mavlink_msg_pilot_console_get_dr(msg);
	pilot_console->de = mavlink_msg_pilot_console_get_de(msg);
}
