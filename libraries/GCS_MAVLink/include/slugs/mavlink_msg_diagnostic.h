// MESSAGE DIAGNOSTIC PACKING

#define MAVLINK_MSG_ID_DIAGNOSTIC 193

typedef struct __mavlink_diagnostic_t 
{
	uint8_t target; ///< The system reporting the diagnostic
	float diagFl1; ///< Diagnostic float 1
	float diagFl2; ///< Diagnostic float 2
	float diagFl3; ///< Diagnostic float 3
	int16_t diagSh1; ///< Diagnostic short 1
	int16_t diagSh2; ///< Diagnostic short 2
	int16_t diagSh3; ///< Diagnostic short 3

} mavlink_diagnostic_t;



/**
 * @brief Send a diagnostic message
 *
 * @param target The system reporting the diagnostic
 * @param diagFl1 Diagnostic float 1
 * @param diagFl2 Diagnostic float 2
 * @param diagFl3 Diagnostic float 3
 * @param diagSh1 Diagnostic short 1
 * @param diagSh2 Diagnostic short 2
 * @param diagSh3 Diagnostic short 3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_diagnostic_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, float diagFl1, float diagFl2, float diagFl3, int16_t diagSh1, int16_t diagSh2, int16_t diagSh3)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_DIAGNOSTIC;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system reporting the diagnostic
	i += put_float_by_index(diagFl1, i, msg->payload); //Diagnostic float 1
	i += put_float_by_index(diagFl2, i, msg->payload); //Diagnostic float 2
	i += put_float_by_index(diagFl3, i, msg->payload); //Diagnostic float 3
	i += put_int16_t_by_index(diagSh1, i, msg->payload); //Diagnostic short 1
	i += put_int16_t_by_index(diagSh2, i, msg->payload); //Diagnostic short 2
	i += put_int16_t_by_index(diagSh3, i, msg->payload); //Diagnostic short 3

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_diagnostic_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_diagnostic_t* diagnostic)
{
	return mavlink_msg_diagnostic_pack(system_id, component_id, msg, diagnostic->target, diagnostic->diagFl1, diagnostic->diagFl2, diagnostic->diagFl3, diagnostic->diagSh1, diagnostic->diagSh2, diagnostic->diagSh3);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_diagnostic_send(mavlink_channel_t chan, uint8_t target, float diagFl1, float diagFl2, float diagFl3, int16_t diagSh1, int16_t diagSh2, int16_t diagSh3)
{
	mavlink_message_t msg;
	mavlink_msg_diagnostic_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, diagFl1, diagFl2, diagFl3, diagSh1, diagSh2, diagSh3);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE DIAGNOSTIC UNPACKING

/**
 * @brief Get field target from diagnostic message
 *
 * @return The system reporting the diagnostic
 */
static inline uint8_t mavlink_msg_diagnostic_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field diagFl1 from diagnostic message
 *
 * @return Diagnostic float 1
 */
static inline float mavlink_msg_diagnostic_get_diagFl1(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field diagFl2 from diagnostic message
 *
 * @return Diagnostic float 2
 */
static inline float mavlink_msg_diagnostic_get_diagFl2(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field diagFl3 from diagnostic message
 *
 * @return Diagnostic float 3
 */
static inline float mavlink_msg_diagnostic_get_diagFl3(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field diagSh1 from diagnostic message
 *
 * @return Diagnostic short 1
 */
static inline int16_t mavlink_msg_diagnostic_get_diagSh1(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field diagSh2 from diagnostic message
 *
 * @return Diagnostic short 2
 */
static inline int16_t mavlink_msg_diagnostic_get_diagSh2(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field diagSh3 from diagnostic message
 *
 * @return Diagnostic short 3
 */
static inline int16_t mavlink_msg_diagnostic_get_diagSh3(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

static inline void mavlink_msg_diagnostic_decode(const mavlink_message_t* msg, mavlink_diagnostic_t* diagnostic)
{
	diagnostic->target = mavlink_msg_diagnostic_get_target(msg);
	diagnostic->diagFl1 = mavlink_msg_diagnostic_get_diagFl1(msg);
	diagnostic->diagFl2 = mavlink_msg_diagnostic_get_diagFl2(msg);
	diagnostic->diagFl3 = mavlink_msg_diagnostic_get_diagFl3(msg);
	diagnostic->diagSh1 = mavlink_msg_diagnostic_get_diagSh1(msg);
	diagnostic->diagSh2 = mavlink_msg_diagnostic_get_diagSh2(msg);
	diagnostic->diagSh3 = mavlink_msg_diagnostic_get_diagSh3(msg);
}
