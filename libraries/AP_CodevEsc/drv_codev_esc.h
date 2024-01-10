#pragma once

#include <stdint.h>

/* At the moment the only known use is with a current sensor */
#define ESC_HAVE_CURRENT_SENSOR

#define TAP_ESC_MAX_PACKET_LEN 20
#define TAP_ESC_MAX_MOTOR_NUM 8

#define PACKET_HEAD 0xfe
#define PACKET_ID_MASK	0x55

/* ESC_POS maps the values stored in the channelMapTable to reorder the ESC's
 * id so that that match the mux setting, so that the ressonder's data
 * will be read.
 * The index on channelMapTable[p] p is the physical ESC
 * The value it is set to is the logical value from ESC_POS[p]
 *  Phy Log
 *  0   0
 *  1   1
 *  2   4
 *  3   3
 *  4   2
 *  5   5
 *   ....
 *
 */

// Circular from back right in CCW direction
#define ESC_POS {2, 1, 0, 3, 4, 5, 6, 7}
// 0 is CW, 1 is CCW
#define ESC_DIR {0, 1, 1, 0, 1 ,1, 1, 1}

#define RPMMAX 1900
#define RPMMIN 1200
#define RPMSTOPPED (RPMMIN - 10)
#define ESC_HAVE_TEMPERATURE_SENSOR 1


#define MAX_BOOT_TIME_MS		 (550) // Minimum time to wait after Power on before sending commands

#pragma pack(push,1)

/****** Run ***********/

#define RUN_CHANNEL_VALUE_MASK   (uint16_t)0x07ff
#define RUN_RED_LED_ON_MASK      (uint16_t)0x0800
#define RUN_GREEN_LED_ON_MASK    (uint16_t)0x1000
#define RUN_BLUE_LED_ON_MASK     (uint16_t)0x2000

#define RUN_RED_LED_OFF_MASK      (uint16_t)0x0000
#define RUN_GREEN_LED_OFF_MASK    (uint16_t)0x0000
#define RUN_BLUE_LED_OFF_MASK     (uint16_t)0x0000

#define RUN_LED_ON_MASK          (uint16_t)0x3800
#define RUN_FEEDBACK_ENABLE_MASK (uint16_t)0x4000
#define RUN_REVERSE_MASK         (uint16_t)0x8000

typedef struct {

	uint16_t rpm_flags[TAP_ESC_MAX_MOTOR_NUM];
} RunReq;

typedef struct {
	uint8_t channelID;
	uint8_t ESCStatus;
	int16_t speed; // -32767 - 32768
#if defined(ESC_HAVE_VOLTAGE_SENSOR)
	uint16_t voltage; // 0.00 - 100.00 V
#endif
#if defined(ESC_HAVE_CURRENT_SENSOR)
	uint16_t current; // 0.0 - 200.0 A
#endif
#if defined(ESC_HAVE_TEMPERATURE_SENSOR)
	uint8_t temperature; // 0 - 256 degree celsius
#endif
} RunInfoRepsonse;
/****** Run ***********/

/****** ConFigInfoBasic ***********/
typedef  struct {
	uint8_t  maxChannelInUse;
	uint8_t  channelMapTable[TAP_ESC_MAX_MOTOR_NUM];
	uint8_t  monitorMsgType;
	uint8_t  controlMode;
	uint16_t minChannelValue;
	uint16_t maxChannelValue;
} ConfigInfoBasicRequest;

typedef  struct {
	uint8_t  channelID;
	ConfigInfoBasicRequest resp;
} ConfigInfoBasicResponse;

#define ESC_MASK_MAP_CHANNEL           0x0f
#define ESC_MASK_MAP_RUNNING_DIRECTION 0xf0
/****** ConFigInfoBasicResponse ***********/

/****** InfoRequest  ***********/
typedef enum {
	REQUEST_INFO_BASIC = 0,
	REQUEST_INFO_FUll,
	REQUEST_INFO_RUN,
	REQUEST_INFO_STUDY,
	REQUEST_INFO_COMM,
	REQUEST_INFO_DEVICE,
} InfoTypes;

typedef  struct {
	uint8_t  channelID;
	uint8_t  requestInfoType;
} InfoRequest;

typedef struct {
	uint16_t frequency; // 0 - 20kHz
	uint16_t duration_ms;
	uint8_t strength;
} EscbusTunePacket;

/****** InfoRequest ***********/

/****** IdDoCmd ***********/
// the real packet definition for ESCBUS_MSG_ID_DO_CMD
// command definition
typedef enum {
	DO_RESET = 0,
	DO_STUDY,
	DO_ID_ASSIGNMENT,
	DO_POWER_TEST,
} ESCBUS_ENUM_COMMAND;

typedef struct {
	uint8_t channelIDMask;
	uint8_t command;
	uint8_t escID;
} EscbusDoCmdPacket;

typedef struct {
	uint8_t id_mask;
	uint8_t child_cmd;
	uint8_t id;
} EscbusConfigidPacket;

typedef  struct {
	uint8_t  escID;
} AssignedIdResponse;

/****** InfoRequest ***********/

typedef  struct {
	uint8_t head;
	uint8_t len;
	uint8_t msg_id;
	union {
		InfoRequest 		reqInfo;
		ConfigInfoBasicRequest 	reqConfigInfoBasic;
		RunReq			reqRun;
		EscbusTunePacket	tunePacket;
		EscbusConfigidPacket    configidPacket;
		ConfigInfoBasicResponse rspConfigInfoBasic;
		RunInfoRepsonse		rspRunInfo;
		AssignedIdResponse      rspAssignedId;
		uint8_t bytes[100];
	} d;
	uint8_t crc_data;

} EscPacket;

typedef  struct  {
    uint8_t id;
	uint8_t state;
    uint8_t temperature;
    uint16_t current;
	int16_t rpm;
	uint16_t esc_set;
} Esc_Status;

#define UART_BUFFER_SIZE 128
typedef  struct {
	uint8_t head;
	uint8_t tail;
	uint8_t dat_cnt;
	uint8_t esc_feedback_buf[UART_BUFFER_SIZE];
} ESC_UART_BUF;

const uint8_t crc_table[256] = {
	0x00, 0xE7, 0x29, 0xCE, 0x52, 0xB5, 0x7B, 0x9C, 0xA4, 0x43, 0x8D, 0x6A,
	0xF6, 0x11, 0xDF, 0x38, 0xAF, 0x48, 0x86, 0x61, 0xFD, 0x1A, 0xD4, 0x33,
	0x0B, 0xEC, 0x22, 0xC5, 0x59, 0xBE, 0x70, 0x97, 0xB9, 0x5E, 0x90, 0x77,
	0xEB, 0x0C, 0xC2, 0x25, 0x1D, 0xFA, 0x34, 0xD3, 0x4F, 0xA8, 0x66, 0x81,
	0x16, 0xF1, 0x3F, 0xD8, 0x44, 0xA3, 0x6D, 0x8A, 0xB2, 0x55, 0x9B, 0x7C,
	0xE0, 0x07, 0xC9, 0x2E, 0x95, 0x72, 0xBC, 0x5B, 0xC7, 0x20, 0xEE, 0x09,
	0x31, 0xD6, 0x18, 0xFF, 0x63, 0x84, 0x4A, 0xAD, 0x3A, 0xDD, 0x13, 0xF4,
	0x68, 0x8F, 0x41, 0xA6, 0x9E, 0x79, 0xB7, 0x50, 0xCC, 0x2B, 0xE5, 0x02,
	0x2C, 0xCB, 0x05, 0xE2, 0x7E, 0x99, 0x57, 0xB0, 0x88, 0x6F, 0xA1, 0x46,
	0xDA, 0x3D, 0xF3, 0x14, 0x83, 0x64, 0xAA, 0x4D, 0xD1, 0x36, 0xF8, 0x1F,
	0x27, 0xC0, 0x0E, 0xE9, 0x75, 0x92, 0x5C, 0xBB, 0xCD, 0x2A, 0xE4, 0x03,
	0x9F, 0x78, 0xB6, 0x51, 0x69, 0x8E, 0x40, 0xA7, 0x3B, 0xDC, 0x12, 0xF5,
	0x62, 0x85, 0x4B, 0xAC, 0x30, 0xD7, 0x19, 0xFE, 0xC6, 0x21, 0xEF, 0x08,
	0x94, 0x73, 0xBD, 0x5A, 0x74, 0x93, 0x5D, 0xBA, 0x26, 0xC1, 0x0F, 0xE8,
	0xD0, 0x37, 0xF9, 0x1E, 0x82, 0x65, 0xAB, 0x4C, 0xDB, 0x3C, 0xF2, 0x15,
	0x89, 0x6E, 0xA0, 0x47, 0x7F, 0x98, 0x56, 0xB1, 0x2D, 0xCA, 0x04, 0xE3,
	0x58, 0xBF, 0x71, 0x96, 0x0A, 0xED, 0x23, 0xC4, 0xFC, 0x1B, 0xD5, 0x32,
	0xAE, 0x49, 0x87, 0x60, 0xF7, 0x10, 0xDE, 0x39, 0xA5, 0x42, 0x8C, 0x6B,
	0x53, 0xB4, 0x7A, 0x9D, 0x01, 0xE6, 0x28, 0xCF, 0xE1, 0x06, 0xC8, 0x2F,
	0xB3, 0x54, 0x9A, 0x7D, 0x45, 0xA2, 0x6C, 0x8B, 0x17, 0xF0, 0x3E, 0xD9,
	0x4E, 0xA9, 0x67, 0x80, 0x1C, 0xFB, 0x35, 0xD2, 0xEA, 0x0D, 0xC3, 0x24,
	0xB8, 0x5F, 0x91, 0x76
};

#pragma pack(pop)
/******************************************************************************************
 * ESCBUS_MSG_ID_RUN_INFO packet
 *
 * Monitor message of ESCs while motor is running
 *
 * channelID: assigned channel number
 *
 * ESCStatus: status of ESC
 * 	Num		Health status
 * 	0		HEALTHY
 * 	1		WARNING_LOW_VOLTAGE
 * 	2		WARNING_OVER_CURRENT
 * 	3		WARNING_OVER_HEAT
 *	4		ERROR_MOTOR_LOW_SPEED_LOSE_STEP
 *  5		ERROR_MOTOR_STALL
 *  6		ERROR_HARDWARE
 *  7		ERROR_LOSE_PROPELLER
 *  8		ERROR_OVER_CURRENT
 *
 * speed: -32767 - 32767 rpm
 *
 * temperature: 0 - 256 celsius degree (if available)
 * voltage: 0.00 - 100.00 V (if available)
 * current: 0.0 - 200.0 A (if available)
 */

typedef enum {
	ESC_STATUS_HEALTHY,
	ESC_STATUS_WARNING_LOW_VOLTAGE,
	ESC_STATUS_WARNING_OVER_HEAT,
	ESC_STATUS_ERROR_MOTOR_LOW_SPEED_LOSE_STEP,
	ESC_STATUS_ERROR_MOTOR_STALL,
	ESC_STATUS_ERROR_HARDWARE,
	ESC_STATUS_ERROR_LOSE_PROPELLER,
	ESC_STATUS_ERROR_OVER_CURRENT,
	ESC_STATUS_ERROR_MOTOR_HIGH_SPEED_LOSE_STEP,
	ESC_STATUS_ERROR_LOSE_CMD,
} ESCBUS_ENUM_ESC_STATUS;


typedef enum {
// messages or command to ESC
	ESCBUS_MSG_ID_CONFIG_BASIC = 0,
	ESCBUS_MSG_ID_CONFIG_FULL,
	ESCBUS_MSG_ID_RUN,
	ESCBUS_MSG_ID_TUNE,
	ESCBUS_MSG_ID_DO_CMD,
// messages from ESC
	ESCBUS_MSG_ID_REQUEST_INFO,
	ESCBUS_MSG_ID_CONFIG_INFO_BASIC,	// simple configuration info for request from flight controller
	ESCBUS_MSG_ID_CONFIG_INFO_FULL,// full configuration info for request from host such as computer
	ESCBUS_MSG_ID_RUN_INFO,// feedback message in RUN mode
	ESCBUS_MSG_ID_STUDY_INFO,	// studied parameters in STUDY mode
	ESCBUS_MSG_ID_COMM_INFO,	// communication method info
	ESCBUS_MSG_ID_DEVICE_INFO,// ESC device info
	ESCBUS_MSG_ID_ASSIGNED_ID,	// never touch ESCBUS_MSG_ID_MAX_NUM
	//boot loader used
	PROTO_OK = 0x10, // INSYNC/OK - 'ok' response
	PROTO_FAILED = 0x11, // INSYNC/FAILED - 'fail' response

	ESCBUS_MSG_ID_BOOT_SYNC = 0x21, // boot loader used
	PROTO_GET_DEVICE = 0x22, // get device ID bytes
	PROTO_CHIP_ERASE = 0x23, // erase program area and reset program address
	PROTO_PROG_MULTI = 0x27, // write bytes at program address and increment
	PROTO_GET_CRC = 0x29, // compute & return a CRC
	PROTO_BOOT = 0x30, // boot the application
	PROTO_GET_SOFTWARE_VERSION = 0x40,
	ESCBUS_MSG_ID_MAX_NUM,
}
ESCBUS_ENUM_MESSAGE_ID;

typedef enum {
	HEAD =0,
	LEN,
	ID,
	DATA,
	CRC1 = 4,

} PARSR_ESC_STATE;
