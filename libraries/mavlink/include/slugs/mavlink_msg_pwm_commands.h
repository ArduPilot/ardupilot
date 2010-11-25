// MESSAGE PWM_COMMANDS PACKING

#define MAVLINK_MSG_ID_PWM_COMMANDS 195

typedef struct __mavlink_pwm_commands_t 
{
	uint8_t target; ///< The system reporting the diagnostic
	uint16_t dt_c; ///< AutoPilot's throttle command 
	uint16_t dla_c; ///< AutoPilot's left aileron command 
	uint16_t dra_c; ///< AutoPilot's right aileron command 
	uint16_t dr_c; ///< AutoPilot's rudder command 
	uint16_t dle_c; ///< AutoPilot's left elevator command 
	uint16_t dre_c; ///< AutoPilot's right elevator command 
	uint16_t dlf_c; ///< AutoPilot's left  flap command 
	uint16_t drf_c; ///< AutoPilot's right flap command 
	uint16_t aux1; ///< AutoPilot's aux1 command 
	uint16_t aux2; ///< AutoPilot's aux2 command 

} mavlink_pwm_commands_t;



/**
 * @brief Send a pwm_commands message
 *
 * @param target The system reporting the diagnostic
 * @param dt_c AutoPilot's throttle command 
 * @param dla_c AutoPilot's left aileron command 
 * @param dra_c AutoPilot's right aileron command 
 * @param dr_c AutoPilot's rudder command 
 * @param dle_c AutoPilot's left elevator command 
 * @param dre_c AutoPilot's right elevator command 
 * @param dlf_c AutoPilot's left  flap command 
 * @param drf_c AutoPilot's right flap command 
 * @param aux1 AutoPilot's aux1 command 
 * @param aux2 AutoPilot's aux2 command 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pwm_commands_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, uint16_t dt_c, uint16_t dla_c, uint16_t dra_c, uint16_t dr_c, uint16_t dle_c, uint16_t dre_c, uint16_t dlf_c, uint16_t drf_c, uint16_t aux1, uint16_t aux2)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_PWM_COMMANDS;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system reporting the diagnostic
	i += put_uint16_t_by_index(dt_c, i, msg->payload); //AutoPilot's throttle command 
	i += put_uint16_t_by_index(dla_c, i, msg->payload); //AutoPilot's left aileron command 
	i += put_uint16_t_by_index(dra_c, i, msg->payload); //AutoPilot's right aileron command 
	i += put_uint16_t_by_index(dr_c, i, msg->payload); //AutoPilot's rudder command 
	i += put_uint16_t_by_index(dle_c, i, msg->payload); //AutoPilot's left elevator command 
	i += put_uint16_t_by_index(dre_c, i, msg->payload); //AutoPilot's right elevator command 
	i += put_uint16_t_by_index(dlf_c, i, msg->payload); //AutoPilot's left  flap command 
	i += put_uint16_t_by_index(drf_c, i, msg->payload); //AutoPilot's right flap command 
	i += put_uint16_t_by_index(aux1, i, msg->payload); //AutoPilot's aux1 command 
	i += put_uint16_t_by_index(aux2, i, msg->payload); //AutoPilot's aux2 command 

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_pwm_commands_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pwm_commands_t* pwm_commands)
{
	return mavlink_msg_pwm_commands_pack(system_id, component_id, msg, pwm_commands->target, pwm_commands->dt_c, pwm_commands->dla_c, pwm_commands->dra_c, pwm_commands->dr_c, pwm_commands->dle_c, pwm_commands->dre_c, pwm_commands->dlf_c, pwm_commands->drf_c, pwm_commands->aux1, pwm_commands->aux2);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pwm_commands_send(mavlink_channel_t chan, uint8_t target, uint16_t dt_c, uint16_t dla_c, uint16_t dra_c, uint16_t dr_c, uint16_t dle_c, uint16_t dre_c, uint16_t dlf_c, uint16_t drf_c, uint16_t aux1, uint16_t aux2)
{
	mavlink_message_t msg;
	mavlink_msg_pwm_commands_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, dt_c, dla_c, dra_c, dr_c, dle_c, dre_c, dlf_c, drf_c, aux1, aux2);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE PWM_COMMANDS UNPACKING

/**
 * @brief Get field target from pwm_commands message
 *
 * @return The system reporting the diagnostic
 */
static inline uint8_t mavlink_msg_pwm_commands_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field dt_c from pwm_commands message
 *
 * @return AutoPilot's throttle command 
 */
static inline uint16_t mavlink_msg_pwm_commands_get_dt_c(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field dla_c from pwm_commands message
 *
 * @return AutoPilot's left aileron command 
 */
static inline uint16_t mavlink_msg_pwm_commands_get_dla_c(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field dra_c from pwm_commands message
 *
 * @return AutoPilot's right aileron command 
 */
static inline uint16_t mavlink_msg_pwm_commands_get_dra_c(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field dr_c from pwm_commands message
 *
 * @return AutoPilot's rudder command 
 */
static inline uint16_t mavlink_msg_pwm_commands_get_dr_c(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field dle_c from pwm_commands message
 *
 * @return AutoPilot's left elevator command 
 */
static inline uint16_t mavlink_msg_pwm_commands_get_dle_c(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field dre_c from pwm_commands message
 *
 * @return AutoPilot's right elevator command 
 */
static inline uint16_t mavlink_msg_pwm_commands_get_dre_c(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field dlf_c from pwm_commands message
 *
 * @return AutoPilot's left  flap command 
 */
static inline uint16_t mavlink_msg_pwm_commands_get_dlf_c(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field drf_c from pwm_commands message
 *
 * @return AutoPilot's right flap command 
 */
static inline uint16_t mavlink_msg_pwm_commands_get_drf_c(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field aux1 from pwm_commands message
 *
 * @return AutoPilot's aux1 command 
 */
static inline uint16_t mavlink_msg_pwm_commands_get_aux1(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field aux2 from pwm_commands message
 *
 * @return AutoPilot's aux2 command 
 */
static inline uint16_t mavlink_msg_pwm_commands_get_aux2(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

static inline void mavlink_msg_pwm_commands_decode(const mavlink_message_t* msg, mavlink_pwm_commands_t* pwm_commands)
{
	pwm_commands->target = mavlink_msg_pwm_commands_get_target(msg);
	pwm_commands->dt_c = mavlink_msg_pwm_commands_get_dt_c(msg);
	pwm_commands->dla_c = mavlink_msg_pwm_commands_get_dla_c(msg);
	pwm_commands->dra_c = mavlink_msg_pwm_commands_get_dra_c(msg);
	pwm_commands->dr_c = mavlink_msg_pwm_commands_get_dr_c(msg);
	pwm_commands->dle_c = mavlink_msg_pwm_commands_get_dle_c(msg);
	pwm_commands->dre_c = mavlink_msg_pwm_commands_get_dre_c(msg);
	pwm_commands->dlf_c = mavlink_msg_pwm_commands_get_dlf_c(msg);
	pwm_commands->drf_c = mavlink_msg_pwm_commands_get_drf_c(msg);
	pwm_commands->aux1 = mavlink_msg_pwm_commands_get_aux1(msg);
	pwm_commands->aux2 = mavlink_msg_pwm_commands_get_aux2(msg);
}
