/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <inttypes.h>

/**
 * @file protocol.h
 *
 * PX4IO interface protocol.
 *
 * Communication is performed via writes to and reads from 16-bit virtual
 * registers organised into pages of 255 registers each.
 *
 * The first two bytes of each write select a page and offset address
 * respectively. Subsequent reads and writes increment the offset within
 * the page. 
 *
 * Some pages are read- or write-only.
 *
 * Note that some pages may permit offset values greater than 255, which
 * can only be achieved by long writes. The offset does not wrap.
 *
 * Writes to unimplemented registers are ignored. Reads from unimplemented
 * registers return undefined values.
 *
 * As convention, values that would be floating point in other parts of 
 * the PX4 system are expressed as signed integer values scaled by 10000,
 * e.g. control values range from -10000..10000.  Use the REG_TO_SIGNED and
 * SIGNED_TO_REG macros to convert between register representation and
 * the signed version, and REG_TO_FLOAT/FLOAT_TO_REG to convert to float.
 *
 * Note that the implementation of readable pages prefers registers within
 * readable pages to be densely packed. Page numbers do not need to be
 * packed.
 *
 * Definitions marked [1] are only valid on PX4IOv1 boards. Likewise, 
 * [2] denotes definitions specific to the PX4IOv2 board.
 */

/* Per C, this is safe for all 2's complement systems */
#define REG_TO_SIGNED(_reg)	((int16_t)(_reg))
#define SIGNED_TO_REG(_signed)	((uint16_t)(_signed))

#define REG_TO_FLOAT(_reg)	((float)REG_TO_SIGNED(_reg) / 10000.0f)
#define FLOAT_TO_REG(_float)	SIGNED_TO_REG((int16_t)((_float) * 10000.0f))

#define PX4IO_PROTOCOL_VERSION		4

/* maximum allowable sizes on this protocol version */
#define PX4IO_PROTOCOL_MAX_CONTROL_COUNT	8	/**< The protocol does not support more than set here, individual units might support less - see PX4IO_P_CONFIG_CONTROL_COUNT */

/* static configuration page */
#define PX4IO_PAGE_CONFIG		0
#define PX4IO_P_CONFIG_PROTOCOL_VERSION		0	/* PX4IO_PROTOCOL_VERSION */
#define PX4IO_P_CONFIG_HARDWARE_VERSION		1	/* magic numbers TBD */
#define PX4IO_P_CONFIG_BOOTLOADER_VERSION	2	/* get this how? */
#define PX4IO_P_CONFIG_MAX_TRANSFER		3	/* maximum I2C transfer size */
#define PX4IO_P_CONFIG_CONTROL_COUNT		4	/* hardcoded max control count supported */
#define PX4IO_P_CONFIG_ACTUATOR_COUNT		5	/* hardcoded max actuator output count */
#define PX4IO_P_CONFIG_RC_INPUT_COUNT		6	/* hardcoded max R/C input count supported */
#define PX4IO_P_CONFIG_ADC_INPUT_COUNT		7	/* hardcoded max ADC inputs */
#define PX4IO_P_CONFIG_RELAY_COUNT		8	/* hardcoded # of relay outputs */
#define PX4IO_P_CONFIG_CONTROL_GROUP_COUNT	8	/**< hardcoded # of control groups*/

/* dynamic status page */
#define PX4IO_PAGE_STATUS		1
#define PX4IO_P_STATUS_FREEMEM			0
#define PX4IO_P_STATUS_CPULOAD			1

#define PX4IO_P_STATUS_FLAGS			2	 /* monitoring flags */
#define PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED	(1 << 0) /* arm-ok and locally armed */
#define PX4IO_P_STATUS_FLAGS_OVERRIDE		(1 << 1) /* in manual override */
#define PX4IO_P_STATUS_FLAGS_RC_OK		(1 << 2) /* RC input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_PPM		(1 << 3) /* PPM input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_DSM		(1 << 4) /* DSM input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_SBUS		(1 << 5) /* SBUS input is valid */
#define PX4IO_P_STATUS_FLAGS_FMU_OK		(1 << 6) /* controls from FMU are valid */
#define PX4IO_P_STATUS_FLAGS_RAW_PWM		(1 << 7) /* raw PWM from FMU is bypassing the mixer */
#define PX4IO_P_STATUS_FLAGS_MIXER_OK		(1 << 8) /* mixer is OK */
#define PX4IO_P_STATUS_FLAGS_ARM_SYNC		(1 << 9) /* the arming state between IO and FMU is in sync */
#define PX4IO_P_STATUS_FLAGS_INIT_OK		(1 << 10) /* initialisation of the IO completed without error */
#define PX4IO_P_STATUS_FLAGS_FAILSAFE		(1 << 11) /* failsafe is active */
#define PX4IO_P_STATUS_FLAGS_SAFETY_OFF		(1 << 12) /* safety is off */
#define PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED	(1 << 13) /* FMU was initialized and OK once */
#define PX4IO_P_STATUS_FLAGS_RC_ST24		(1 << 14) /* ST24 input is valid */
#define PX4IO_P_STATUS_FLAGS_RC_SUMD		(1 << 15) /* SUMD input is valid */

#define PX4IO_P_STATUS_ALARMS			3	 /* alarm flags - alarms latch, write 1 to a bit to clear it */
#define PX4IO_P_STATUS_ALARMS_VBATT_LOW		(1 << 0) /* [1] VBatt is very close to regulator dropout */
#define PX4IO_P_STATUS_ALARMS_TEMPERATURE	(1 << 1) /* board temperature is high */
#define PX4IO_P_STATUS_ALARMS_SERVO_CURRENT	(1 << 2) /* [1] servo current limit was exceeded */
#define PX4IO_P_STATUS_ALARMS_ACC_CURRENT	(1 << 3) /* [1] accessory current limit was exceeded */
#define PX4IO_P_STATUS_ALARMS_FMU_LOST		(1 << 4) /* timed out waiting for controls from FMU */
#define PX4IO_P_STATUS_ALARMS_RC_LOST		(1 << 5) /* timed out waiting for RC input */
#define PX4IO_P_STATUS_ALARMS_PWM_ERROR		(1 << 6) /* PWM configuration or output was bad */
#define PX4IO_P_STATUS_ALARMS_VSERVO_FAULT	(1 << 7) /* [2] VServo was out of the valid range (2.5 - 5.5 V) */

#define PX4IO_P_STATUS_VBATT			4	/* [1] battery voltage in mV */
#define PX4IO_P_STATUS_IBATT			5	/* [1] battery current (raw ADC) */
#define PX4IO_P_STATUS_VSERVO			6	/* [2] servo rail voltage in mV */
#define PX4IO_P_STATUS_VRSSI			7	/* [2] RSSI voltage */
#define PX4IO_P_STATUS_PRSSI			8	/* [2] RSSI PWM value */

/* array of post-mix actuator outputs, -10000..10000 */
#define PX4IO_PAGE_ACTUATORS		2		/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* array of PWM servo output values, microseconds */
#define PX4IO_PAGE_SERVOS		3		/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* array of raw RC input values, microseconds */
#define PX4IO_PAGE_RAW_RC_INPUT		4
#define PX4IO_P_RAW_RC_COUNT			0	/* number of valid channels */
#define PX4IO_P_RAW_RC_FLAGS			1	/* RC detail status flags */
#define PX4IO_P_RAW_RC_FLAGS_FRAME_DROP		(1 << 0) /* single frame drop */
#define PX4IO_P_RAW_RC_FLAGS_FAILSAFE		(1 << 1) /* receiver is in failsafe mode */
#define PX4IO_P_RAW_RC_FLAGS_RC_DSM11		(1 << 2) /* DSM decoding is 11 bit mode */
#define PX4IO_P_RAW_RC_FLAGS_MAPPING_OK		(1 << 3) /* Channel mapping is ok */
#define PX4IO_P_RAW_RC_FLAGS_RC_OK		(1 << 4) /* RC reception ok */

#define PX4IO_P_RAW_RC_NRSSI			2	/* [2] Normalized RSSI value, 0: no reception, 255: perfect reception */
#define PX4IO_P_RAW_RC_DATA			3	/* [1] + [2] Details about the RC source (PPM frame length, Spektrum protocol type) */
#define PX4IO_P_RAW_FRAME_COUNT			4	/* Number of total received frames (wrapping counter) */
#define PX4IO_P_RAW_LOST_FRAME_COUNT		5	/* Number of total dropped frames (wrapping counter) */
#define PX4IO_P_RAW_RC_BASE			6	/* CONFIG_RC_INPUT_COUNT channels from here */

/* array of scaled RC input values, -10000..10000 */
#define PX4IO_PAGE_RC_INPUT		5
#define PX4IO_P_RC_VALID			0	/* bitmask of valid controls */
#define PX4IO_P_RC_BASE				1	/* CONFIG_RC_INPUT_COUNT controls from here */

/* array of raw ADC values */
#define PX4IO_PAGE_RAW_ADC_INPUT	6		/* 0..CONFIG_ADC_INPUT_COUNT-1 */

/* PWM servo information */
#define PX4IO_PAGE_PWM_INFO		7
#define PX4IO_RATE_MAP_BASE			0	/* 0..CONFIG_ACTUATOR_COUNT bitmaps of PWM rate groups */

/* setup page */
#define PX4IO_PAGE_SETUP		50
#define PX4IO_P_SETUP_FEATURES			0
#define PX4IO_P_SETUP_FEATURES_SBUS1_OUT	(1 << 0) /**< enable S.Bus v1 output */
#define PX4IO_P_SETUP_FEATURES_SBUS2_OUT	(1 << 1) /**< enable S.Bus v2 output */
#define PX4IO_P_SETUP_FEATURES_PWM_RSSI		(1 << 2) /**< enable PWM RSSI parsing */
#define PX4IO_P_SETUP_FEATURES_ADC_RSSI		(1 << 3) /**< enable ADC RSSI parsing */

#define PX4IO_P_SETUP_ARMING			1	 /* arming controls */
#define PX4IO_P_SETUP_ARMING_IO_ARM_OK		(1 << 0) /* OK to arm the IO side */
#define PX4IO_P_SETUP_ARMING_FMU_ARMED		(1 << 1) /* FMU is already armed */
#define PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK	(1 << 2) /* OK to switch to manual override via override RC channel */
#define PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM	(1 << 3) /* use custom failsafe values, not 0 values of mixer */
#define PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK	(1 << 4) /* OK to try in-air restart */
#define PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE	(1 << 5) /* Output of PWM right after startup enabled to help ESCs initialize and prevent them from beeping */
#define PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED	(1 << 6) /* Disable the IO-internal evaluation of the RC */
#define PX4IO_P_SETUP_ARMING_LOCKDOWN		(1 << 7) /* If set, the system operates normally, but won't actuate any servos */
#define PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE	(1 << 8) /* If set, the system will always output the failsafe values */
#define PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE	(1 << 9) /* If set, the system will never return from a failsafe, but remain in failsafe once triggered. */
#define PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE	(1 << 10) /* If set then on FMU failure override is immediate. Othewise it waits for the mode switch to go past the override thrshold */

#define PX4IO_P_SETUP_PWM_RATES			2	/* bitmask, 0 = low rate, 1 = high rate */
#define PX4IO_P_SETUP_PWM_DEFAULTRATE		3	/* 'low' PWM frame output rate in Hz */
#define PX4IO_P_SETUP_PWM_ALTRATE		4	/* 'high' PWM frame output rate in Hz */

#if defined(CONFIG_ARCH_BOARD_PX4IO_V1) || defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
#define PX4IO_P_SETUP_RELAYS			5	/* bitmask of relay/switch outputs, 0 = off, 1 = on */
#define PX4IO_P_SETUP_RELAYS_POWER1		(1<<0)	/* hardware rev [1] power relay 1 */
#define PX4IO_P_SETUP_RELAYS_POWER2		(1<<1)	/* hardware rev [1] power relay 2 */
#define PX4IO_P_SETUP_RELAYS_ACC1		(1<<2)	/* hardware rev [1] accessory power 1 */
#define PX4IO_P_SETUP_RELAYS_ACC2		(1<<3)	/* hardware rev [1] accessory power 2 */
#else
#define PX4IO_P_SETUP_RELAYS_PAD		5
#endif

#define PX4IO_P_SETUP_VBATT_SCALE		6	/* hardware rev [1] battery voltage correction factor (float) */
#define PX4IO_P_SETUP_VSERVO_SCALE		6	/* hardware rev [2] servo voltage correction factor (float) */
#define PX4IO_P_SETUP_DSM			7	/* DSM bind state */
enum {							/* DSM bind states */
	dsm_bind_power_down = 0,
	dsm_bind_power_up,
	dsm_bind_set_rx_out,
	dsm_bind_send_pulses,
	dsm_bind_reinit_uart
};
						/* 8 */
#define PX4IO_P_SETUP_SET_DEBUG			9	/* debug level for IO board */

#define PX4IO_P_SETUP_REBOOT_BL			10	/* reboot IO into bootloader */
#define PX4IO_REBOOT_BL_MAGIC			14662	/* required argument for reboot (random) */

#define PX4IO_P_SETUP_CRC			11	/* get CRC of IO firmware */
						/* storage space of 12 occupied by CRC */
#define PX4IO_P_SETUP_FORCE_SAFETY_OFF		12	/* force safety switch into
                                                           'armed' (PWM enabled) state - this is a non-data write and
                                                           hence index 12 can safely be used. */
#define PX4IO_P_SETUP_RC_THR_FAILSAFE_US	13	/**< the throttle failsafe pulse length in microseconds */

#define PX4IO_P_SETUP_FORCE_SAFETY_ON		14	/* force safety switch into 'disarmed' (PWM disabled state) */
#define PX4IO_FORCE_SAFETY_MAGIC		22027	/* required argument for force safety (random) */

/* autopilot control values, -10000..10000 */
#define PX4IO_PAGE_CONTROLS			51	/**< actuator control groups, one after the other, 8 wide */
#define PX4IO_P_CONTROLS_GROUP_0		(PX4IO_PROTOCOL_MAX_CONTROL_COUNT * 0)	/**< 0..PX4IO_PROTOCOL_MAX_CONTROL_COUNT - 1 */
#define PX4IO_P_CONTROLS_GROUP_1		(PX4IO_PROTOCOL_MAX_CONTROL_COUNT * 1)	/**< 0..PX4IO_PROTOCOL_MAX_CONTROL_COUNT - 1 */
#define PX4IO_P_CONTROLS_GROUP_2		(PX4IO_PROTOCOL_MAX_CONTROL_COUNT * 2)	/**< 0..PX4IO_PROTOCOL_MAX_CONTROL_COUNT - 1 */
#define PX4IO_P_CONTROLS_GROUP_3		(PX4IO_PROTOCOL_MAX_CONTROL_COUNT * 3)	/**< 0..PX4IO_PROTOCOL_MAX_CONTROL_COUNT - 1 */

#define PX4IO_P_CONTROLS_GROUP_VALID		64
#define PX4IO_P_CONTROLS_GROUP_VALID_GROUP0	(1 << 0) /**< group 0 is valid / received */
#define PX4IO_P_CONTROLS_GROUP_VALID_GROUP1	(1 << 1) /**< group 1 is valid / received */
#define PX4IO_P_CONTROLS_GROUP_VALID_GROUP2	(1 << 2) /**< group 2 is valid / received */
#define PX4IO_P_CONTROLS_GROUP_VALID_GROUP3	(1 << 3) /**< group 3 is valid / received */

/* raw text load to the mixer parser - ignores offset */
#define PX4IO_PAGE_MIXERLOAD			52

/* R/C channel config */
#define PX4IO_PAGE_RC_CONFIG			53		/**< R/C input configuration */
#define PX4IO_P_RC_CONFIG_MIN			0		/**< lowest input value */
#define PX4IO_P_RC_CONFIG_CENTER		1		/**< center input value */
#define PX4IO_P_RC_CONFIG_MAX			2		/**< highest input value */
#define PX4IO_P_RC_CONFIG_DEADZONE		3		/**< band around center that is ignored */
#define PX4IO_P_RC_CONFIG_ASSIGNMENT		4		/**< mapped input value */
#define PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH	100		/**< magic value for mode switch */
#define PX4IO_P_RC_CONFIG_OPTIONS		5		/**< channel options bitmask */
#define PX4IO_P_RC_CONFIG_OPTIONS_ENABLED	(1 << 0)
#define PX4IO_P_RC_CONFIG_OPTIONS_REVERSE	(1 << 1)
#define PX4IO_P_RC_CONFIG_STRIDE		6		/**< spacing between channel config data */

/* PWM output - overrides mixer */
#define PX4IO_PAGE_DIRECT_PWM			54		/**< 0..CONFIG_ACTUATOR_COUNT-1 */

/* PWM failsafe values - zero disables the output */
#define PX4IO_PAGE_FAILSAFE_PWM			55		/**< 0..CONFIG_ACTUATOR_COUNT-1 */

/* PWM failsafe values - zero disables the output */
#define PX4IO_PAGE_SENSORS			56		/**< Sensors connected to PX4IO */
#define PX4IO_P_SENSORS_ALTITUDE		0		/**< Altitude of an external sensor (HoTT or S.BUS2) */

/* Debug and test page - not used in normal operation */
#define PX4IO_PAGE_TEST				127
#define PX4IO_P_TEST_LED			0		/**< set the amber LED on/off */

/* PWM minimum values for certain ESCs */
#define PX4IO_PAGE_CONTROL_MIN_PWM		106		/**< 0..CONFIG_ACTUATOR_COUNT-1 */

/* PWM maximum values for certain ESCs */
#define PX4IO_PAGE_CONTROL_MAX_PWM		107		/**< 0..CONFIG_ACTUATOR_COUNT-1 */

/* PWM disarmed values that are active, even when SAFETY_SAFE */
#define PX4IO_PAGE_DISARMED_PWM		108			/* 0..CONFIG_ACTUATOR_COUNT-1 */

/* SPI <-> UART */
#define PX4IO_PAGE_UART_BUFFER		120

/**
 * As-needed mixer data upload.
 *
 * This message adds text to the mixer text buffer; the text
 * buffer is drained as the definitions are consumed.
 */
#pragma pack(push, 1)
struct px4io_mixdata {
	uint16_t	f2i_mixer_magic;
#define F2I_MIXER_MAGIC		0x6d74

	uint8_t		action;
#define F2I_MIXER_ACTION_RESET			0
#define F2I_MIXER_ACTION_APPEND			1

	char		text[0];	/* actual text size may vary */
};
#pragma pack(pop)

/**
 * Serial protocol encapsulation.
 */

#define PKT_MAX_REGS	32 // by agreement w/FMU

#pragma pack(push, 1)
struct IOPacket {
	uint8_t 	count_code;
	uint8_t 	crc;
	uint8_t 	page;
	uint8_t 	offset;
	uint16_t	regs[PKT_MAX_REGS];
};
#pragma pack(pop)

#define PKT_CODE_READ		0x00	/* FMU->IO read transaction */
#define PKT_CODE_WRITE		0x40	/* FMU->IO write transaction */
#define PKT_CODE_SPIUART	0xC0	/* FMU<->IO spi-uart transaction */
#define PKT_CODE_SUCCESS	0x00	/* IO->FMU success reply */
#define PKT_CODE_CORRUPT	0x40	/* IO->FMU bad packet reply */
#define PKT_CODE_ERROR		0x80	/* IO->FMU register op error reply */

#define PKT_CODE_MASK		0xc0
#define PKT_COUNT_MASK		0x3f

#define PKT_COUNT(_p)	((_p).count_code & PKT_COUNT_MASK)
#define PKT_CODE(_p)	((_p).count_code & PKT_CODE_MASK)
#define PKT_SIZE(_p)	((size_t)((uint8_t *)&((_p).regs[PKT_COUNT(_p)]) - ((uint8_t *)&(_p))))

static const uint8_t crc8_tab[256] __attribute__((unused)) =
{
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
	0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
	0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
	0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
	0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
	0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
	0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
	0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
	0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
	0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
	0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
	0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
	0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
	0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
	0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
	0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
	0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
	0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
	0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
	0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
	0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
	0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
	0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
	0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
	0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
	0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
	0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

static uint8_t crc_packet(struct IOPacket *pkt) __attribute__((unused));
static uint8_t
crc_packet(struct IOPacket *pkt)
{
	uint8_t *end = (uint8_t *)(&pkt->regs[PKT_COUNT(*pkt)]);
	uint8_t *p = (uint8_t *)pkt;
	uint8_t c = 0;

	while (p < end)
		c = crc8_tab[c ^ *(p++)];

	return c;
}
