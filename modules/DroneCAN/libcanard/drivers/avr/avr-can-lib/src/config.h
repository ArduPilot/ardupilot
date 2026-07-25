#ifndef	CONFIG_H
#define	CONFIG_H

#warning "Default config.h used!"
#define CAN_CONFIG_LOADED

// -----------------------------------------------------------------------------
/* Global settings for building the can-lib and application program.
 *
 * The following two #defines must be set identically for the can-lib and
 * your application program. They control the underlying CAN struct. If the
 * settings disagree, the underlying CAN struct will be broken, with
 * unpredictable results.
 * If can.h detects that any of the #defines is not defined, it will set them
 * to the default values shown here, so it is in your own interest to have a
 * consistent setting. Ommiting the #defines in both can-lib and application
 * program will apply the defaults in a consistent way too.
 *
 * Select if you want to use 29 bit identifiers.
 */
#define	SUPPORT_EXTENDED_CANID	1

/* Select if you want to use timestamps.
 * Timestamps are sourced from a register internal to the AT90CAN.
 * Selecting them on any other controller will have no effect, they will
 * be 0 all the time.
 */
#define	SUPPORT_TIMESTAMPS		0


// -----------------------------------------------------------------------------
/* Global settings for building the can-lib.
 *
 * Select ONE CAN controller for which you are building the can-lib. 
 */
#define	SUPPORT_MCP2515			1
#define	SUPPORT_AT90CAN			0
#define	SUPPORT_SJA1000			0


// -----------------------------------------------------------------------------
/* Setting for MCP2515
 *
 * Declare which pins you are using for communication.
 * Remember NOT to use them in your application!
 * It is a good idea to use bits from the port that carries MOSI, MISO, SCK.
 */
#define	MCP2515_CS				B,4
#define	MCP2515_INT				B,2

// -----------------------------------------------------------------------------
// Setting for SJA1000

#define	SJA1000_INT				E,0
#define	SJA1000_MEMORY_MAPPED	1

// memory-mapped interface
#define	SJA1000_BASE_ADDR		0x8000		// for ATMega162

/*
// port-interface
#define	SJA1000_WR				D,6
#define	SJA1000_RD				D,7

#define	SJA1000_ALE				E,1
#define	SJA1000_CS				C,0
#define	SJA1000_DATA			A
*/

// -----------------------------------------------------------------------------
// Setting for AT90CAN

// Number of CAN messages which are buffered in RAM additinally to the MObs
#define CAN_RX_BUFFER_SIZE		16
#define CAN_TX_BUFFER_SIZE		8

// only available if CAN_TX_BUFFER_SIZE > 0
#define CAN_FORCE_TX_ORDER		1

#endif	// CONFIG_H
