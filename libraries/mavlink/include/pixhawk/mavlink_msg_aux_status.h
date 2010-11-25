// MESSAGE AUX_STATUS PACKING

#define MAVLINK_MSG_ID_AUX_STATUS 142

typedef struct __mavlink_aux_status_t 
{
	uint16_t load; ///< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
	uint16_t i2c0_err_count; ///< Number of I2C errors since startup
	uint16_t i2c1_err_count; ///< Number of I2C errors since startup
	uint16_t spi0_err_count; ///< Number of I2C errors since startup
	uint16_t spi1_err_count; ///< Number of I2C errors since startup
	uint16_t uart_total_err_count; ///< Number of I2C errors since startup

} mavlink_aux_status_t;



/**
 * @brief Send a aux_status message
 *
 * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 * @param i2c0_err_count Number of I2C errors since startup
 * @param i2c1_err_count Number of I2C errors since startup
 * @param spi0_err_count Number of I2C errors since startup
 * @param spi1_err_count Number of I2C errors since startup
 * @param uart_total_err_count Number of I2C errors since startup
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aux_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint16_t load, uint16_t i2c0_err_count, uint16_t i2c1_err_count, uint16_t spi0_err_count, uint16_t spi1_err_count, uint16_t uart_total_err_count)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_AUX_STATUS;

	i += put_uint16_t_by_index(load, i, msg->payload); //Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
	i += put_uint16_t_by_index(i2c0_err_count, i, msg->payload); //Number of I2C errors since startup
	i += put_uint16_t_by_index(i2c1_err_count, i, msg->payload); //Number of I2C errors since startup
	i += put_uint16_t_by_index(spi0_err_count, i, msg->payload); //Number of I2C errors since startup
	i += put_uint16_t_by_index(spi1_err_count, i, msg->payload); //Number of I2C errors since startup
	i += put_uint16_t_by_index(uart_total_err_count, i, msg->payload); //Number of I2C errors since startup

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_aux_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_aux_status_t* aux_status)
{
	return mavlink_msg_aux_status_pack(system_id, component_id, msg, aux_status->load, aux_status->i2c0_err_count, aux_status->i2c1_err_count, aux_status->spi0_err_count, aux_status->spi1_err_count, aux_status->uart_total_err_count);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_aux_status_send(mavlink_channel_t chan, uint16_t load, uint16_t i2c0_err_count, uint16_t i2c1_err_count, uint16_t spi0_err_count, uint16_t spi1_err_count, uint16_t uart_total_err_count)
{
	mavlink_message_t msg;
	mavlink_msg_aux_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, load, i2c0_err_count, i2c1_err_count, spi0_err_count, spi1_err_count, uart_total_err_count);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE AUX_STATUS UNPACKING

/**
 * @brief Get field load from aux_status message
 *
 * @return Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 */
static inline uint16_t mavlink_msg_aux_status_get_load(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload)[0];
	r.b[0] = (msg->payload)[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field i2c0_err_count from aux_status message
 *
 * @return Number of I2C errors since startup
 */
static inline uint16_t mavlink_msg_aux_status_get_i2c0_err_count(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field i2c1_err_count from aux_status message
 *
 * @return Number of I2C errors since startup
 */
static inline uint16_t mavlink_msg_aux_status_get_i2c1_err_count(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field spi0_err_count from aux_status message
 *
 * @return Number of I2C errors since startup
 */
static inline uint16_t mavlink_msg_aux_status_get_spi0_err_count(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field spi1_err_count from aux_status message
 *
 * @return Number of I2C errors since startup
 */
static inline uint16_t mavlink_msg_aux_status_get_spi1_err_count(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field uart_total_err_count from aux_status message
 *
 * @return Number of I2C errors since startup
 */
static inline uint16_t mavlink_msg_aux_status_get_uart_total_err_count(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

static inline void mavlink_msg_aux_status_decode(const mavlink_message_t* msg, mavlink_aux_status_t* aux_status)
{
	aux_status->load = mavlink_msg_aux_status_get_load(msg);
	aux_status->i2c0_err_count = mavlink_msg_aux_status_get_i2c0_err_count(msg);
	aux_status->i2c1_err_count = mavlink_msg_aux_status_get_i2c1_err_count(msg);
	aux_status->spi0_err_count = mavlink_msg_aux_status_get_spi0_err_count(msg);
	aux_status->spi1_err_count = mavlink_msg_aux_status_get_spi1_err_count(msg);
	aux_status->uart_total_err_count = mavlink_msg_aux_status_get_uart_total_err_count(msg);
}
