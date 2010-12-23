// MESSAGE SLUGS_NAVIGATION PACKING

#define MAVLINK_MSG_ID_SLUGS_NAVIGATION 176

typedef struct __mavlink_slugs_navigation_t 
{
	float u_m; ///< Measured Airspeed prior to the Nav Filter
	float phi_c; ///< Commanded Roll
	float theta_c; ///< Commanded Pitch
	float psiDot_c; ///< Commanded Turn rate
	float ay_body; ///< Y component of the body acceleration
	float totalDist; ///< Total Distance to Run on this leg of Navigation
	float dist2Go; ///< Remaining distance to Run on this leg of Navigation
	uint8_t fromWP; ///< Origin WP
	uint8_t toWP; ///< Destination WP

} mavlink_slugs_navigation_t;



/**
 * @brief Send a slugs_navigation message
 *
 * @param u_m Measured Airspeed prior to the Nav Filter
 * @param phi_c Commanded Roll
 * @param theta_c Commanded Pitch
 * @param psiDot_c Commanded Turn rate
 * @param ay_body Y component of the body acceleration
 * @param totalDist Total Distance to Run on this leg of Navigation
 * @param dist2Go Remaining distance to Run on this leg of Navigation
 * @param fromWP Origin WP
 * @param toWP Destination WP
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_navigation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float u_m, float phi_c, float theta_c, float psiDot_c, float ay_body, float totalDist, float dist2Go, uint8_t fromWP, uint8_t toWP)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SLUGS_NAVIGATION;

	i += put_float_by_index(u_m, i, msg->payload); //Measured Airspeed prior to the Nav Filter
	i += put_float_by_index(phi_c, i, msg->payload); //Commanded Roll
	i += put_float_by_index(theta_c, i, msg->payload); //Commanded Pitch
	i += put_float_by_index(psiDot_c, i, msg->payload); //Commanded Turn rate
	i += put_float_by_index(ay_body, i, msg->payload); //Y component of the body acceleration
	i += put_float_by_index(totalDist, i, msg->payload); //Total Distance to Run on this leg of Navigation
	i += put_float_by_index(dist2Go, i, msg->payload); //Remaining distance to Run on this leg of Navigation
	i += put_uint8_t_by_index(fromWP, i, msg->payload); //Origin WP
	i += put_uint8_t_by_index(toWP, i, msg->payload); //Destination WP

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_slugs_navigation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slugs_navigation_t* slugs_navigation)
{
	return mavlink_msg_slugs_navigation_pack(system_id, component_id, msg, slugs_navigation->u_m, slugs_navigation->phi_c, slugs_navigation->theta_c, slugs_navigation->psiDot_c, slugs_navigation->ay_body, slugs_navigation->totalDist, slugs_navigation->dist2Go, slugs_navigation->fromWP, slugs_navigation->toWP);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slugs_navigation_send(mavlink_channel_t chan, float u_m, float phi_c, float theta_c, float psiDot_c, float ay_body, float totalDist, float dist2Go, uint8_t fromWP, uint8_t toWP)
{
	mavlink_message_t msg;
	mavlink_msg_slugs_navigation_pack(mavlink_system.sysid, mavlink_system.compid, &msg, u_m, phi_c, theta_c, psiDot_c, ay_body, totalDist, dist2Go, fromWP, toWP);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SLUGS_NAVIGATION UNPACKING

/**
 * @brief Get field u_m from slugs_navigation message
 *
 * @return Measured Airspeed prior to the Nav Filter
 */
static inline float mavlink_msg_slugs_navigation_get_u_m(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (float)r.f;
}

/**
 * @brief Get field phi_c from slugs_navigation message
 *
 * @return Commanded Roll
 */
static inline float mavlink_msg_slugs_navigation_get_phi_c(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field theta_c from slugs_navigation message
 *
 * @return Commanded Pitch
 */
static inline float mavlink_msg_slugs_navigation_get_theta_c(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field psiDot_c from slugs_navigation message
 *
 * @return Commanded Turn rate
 */
static inline float mavlink_msg_slugs_navigation_get_psiDot_c(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field ay_body from slugs_navigation message
 *
 * @return Y component of the body acceleration
 */
static inline float mavlink_msg_slugs_navigation_get_ay_body(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field totalDist from slugs_navigation message
 *
 * @return Total Distance to Run on this leg of Navigation
 */
static inline float mavlink_msg_slugs_navigation_get_totalDist(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field dist2Go from slugs_navigation message
 *
 * @return Remaining distance to Run on this leg of Navigation
 */
static inline float mavlink_msg_slugs_navigation_get_dist2Go(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field fromWP from slugs_navigation message
 *
 * @return Origin WP
 */
static inline uint8_t mavlink_msg_slugs_navigation_get_fromWP(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
}

/**
 * @brief Get field toWP from slugs_navigation message
 *
 * @return Destination WP
 */
static inline uint8_t mavlink_msg_slugs_navigation_get_toWP(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t))[0];
}

static inline void mavlink_msg_slugs_navigation_decode(const mavlink_message_t* msg, mavlink_slugs_navigation_t* slugs_navigation)
{
	slugs_navigation->u_m = mavlink_msg_slugs_navigation_get_u_m(msg);
	slugs_navigation->phi_c = mavlink_msg_slugs_navigation_get_phi_c(msg);
	slugs_navigation->theta_c = mavlink_msg_slugs_navigation_get_theta_c(msg);
	slugs_navigation->psiDot_c = mavlink_msg_slugs_navigation_get_psiDot_c(msg);
	slugs_navigation->ay_body = mavlink_msg_slugs_navigation_get_ay_body(msg);
	slugs_navigation->totalDist = mavlink_msg_slugs_navigation_get_totalDist(msg);
	slugs_navigation->dist2Go = mavlink_msg_slugs_navigation_get_dist2Go(msg);
	slugs_navigation->fromWP = mavlink_msg_slugs_navigation_get_fromWP(msg);
	slugs_navigation->toWP = mavlink_msg_slugs_navigation_get_toWP(msg);
}
