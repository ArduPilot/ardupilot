// MESSAGE CTRL_SRFC_PT PACKING

#define MAVLINK_MSG_ID_CTRL_SRFC_PT 181

typedef struct __mavlink_ctrl_srfc_pt_t 
{
	uint8_t target; ///< The system setting the commands
	uint16_t bitfieldPt; ///< Bitfield containing the PT configuration

} mavlink_ctrl_srfc_pt_t;



/**
 * @brief Send a ctrl_srfc_pt message
 *
 * @param target The system setting the commands
 * @param bitfieldPt Bitfield containing the PT configuration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ctrl_srfc_pt_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, uint16_t bitfieldPt)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_CTRL_SRFC_PT;

	i += put_uint8_t_by_index(target, i, msg->payload); //The system setting the commands
	i += put_uint16_t_by_index(bitfieldPt, i, msg->payload); //Bitfield containing the PT configuration

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_ctrl_srfc_pt_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ctrl_srfc_pt_t* ctrl_srfc_pt)
{
	return mavlink_msg_ctrl_srfc_pt_pack(system_id, component_id, msg, ctrl_srfc_pt->target, ctrl_srfc_pt->bitfieldPt);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ctrl_srfc_pt_send(mavlink_channel_t chan, uint8_t target, uint16_t bitfieldPt)
{
	mavlink_message_t msg;
	mavlink_msg_ctrl_srfc_pt_pack(mavlink_system.sysid, mavlink_system.compid, &msg, target, bitfieldPt);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE CTRL_SRFC_PT UNPACKING

/**
 * @brief Get field target from ctrl_srfc_pt message
 *
 * @return The system setting the commands
 */
static inline uint8_t mavlink_msg_ctrl_srfc_pt_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field bitfieldPt from ctrl_srfc_pt message
 *
 * @return Bitfield containing the PT configuration
 */
static inline uint16_t mavlink_msg_ctrl_srfc_pt_get_bitfieldPt(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

static inline void mavlink_msg_ctrl_srfc_pt_decode(const mavlink_message_t* msg, mavlink_ctrl_srfc_pt_t* ctrl_srfc_pt)
{
	ctrl_srfc_pt->target = mavlink_msg_ctrl_srfc_pt_get_target(msg);
	ctrl_srfc_pt->bitfieldPt = mavlink_msg_ctrl_srfc_pt_get_bitfieldPt(msg);
}
