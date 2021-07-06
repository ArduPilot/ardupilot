/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Initial protocol implementation was provided by FETtec */

// includes
#include "config.h"
#include "serial.h"
#ifdef STM32F3
	#include "driversF3/GEN_SERIAL_Driver_F3.h"
#endif
#ifdef STM32L4
	#include "driversL4/GEN_SERIAL_Driver_L4.h"
#endif
#ifdef STM32G4
	#include "driversG4/GEN_SERIAL_Driver_G4.h"
#endif
#ifdef STM32F7
	#include "driversF7/GEN_SERIAL_Driver_F7.h"
#endif

//UART uasage

/*
	inits the UART with 2000000 baud 8n1 half duplex
*/
#define FETOW_UART_INIT \
		GEN_SERIAL_swapRXTXpins(FETTEC_ONE_WIRE_UART_NUM, FETTEC_ONE_WIRE_UART_SWAPP_PINS); \
		GEN_SERIAL_initUART(FETTEC_ONE_WIRE_UART_NUM, FETTEC_ONE_WIRE_UART_PINSET, 2000000, 1, 0, 1, 0); 

/*
	de inits the UART with 2000000 baud 8n1 half duplex
*/
#define FETOW_UART_DEINIT GEN_SERIAL_DeInitUART(FETTEC_ONE_WIRE_UART_NUM); 	

/*
	should return the available bytes in RX buffer
*/
#define FETOW_UART_BYTES_AVAILABLE GEN_SERIAL_bytesAvailable(FETTEC_ONE_WIRE_UART_NUM)

/*
	should return one byte of the RX buffer
*/
#define FETOW_UART_READ_BYTE GEN_SERIAL_readByte(FETTEC_ONE_WIRE_UART_NUM)

/*
	to transmit a array of bytes. where buf will be a uint8_t byte array and len is the byte count that should be sent
*/
#define FETOW_UART_SEND_BYTES(buf, len) GEN_SERIAL_sendBytes(FETTEC_ONE_WIRE_UART_NUM, buf, len)

/*
	emtys the RX buffer
*/
#define FETOW_UART_EMPTY_RX while(FETOW_UART_BYTES_AVAILABLE) FETOW_UART_READ_BYTE;

		
		

		
		
#define ALL_ID 0x1F

enum {
	OW_RETURN_RESPONSE,
	OW_RETURN_FULL_FRAME
};

enum {
	OW_OK,
	OW_BL_PAGE_CORRECT, // BL only
	OW_NOT_OK,
	OW_BL_START_FW, // BL only
	OW_BL_PAGES_TO_FLASH, // BL only
	OW_REQ_TYPE,
	OW_REQ_SN,
	OW_REQ_SW_VER
};

enum {
	OW_RESET_TO_BL = 10,
	OW_THROTTLE = 11,
	OW_REQ_TLM = 12,
	OW_BEEP = 13,

	OW_SET_FAST_COM_LENGTH = 26,
	
	OW_GET_ROTATION_DIRECTION = 28,
	OW_SET_ROTATION_DIRECTION = 29,
	
	OW_GET_USE_SIN_START = 30,
	OW_SET_USE_SIN_START = 31,
	
	OW_GET_3D_MODE = 32,
	OW_SET_3D_MODE = 33,
	
	OW_GET_ID = 34,
	OW_SET_ID = 35,

/*	
	OW_GET_LINEAR_THRUST = 36,
	OW_SET_LINEAR_THRUST = 37,
*/

	OW_GET_EEVER = 38,
	
	OW_GET_PWM_MIN = 39,
	OW_SET_PWM_MIN = 40,
	
	OW_GET_PWM_MAX = 41,
	OW_SET_PWM_MAX = 42,
	
	OW_GET_ESC_BEEP = 43,
	OW_SET_ESC_BEEP = 44,
	
	OW_GET_CURRENT_CALIB = 45,
	OW_SET_CURRENT_CALIB = 46,
	
	OW_SET_LED_TMP_COLOR = 51,
	OW_GET_LED_COLOR = 52,
	OW_SET_LED_COLOR = 53

};

enum {
	OW_TLM_TEMP,
	OW_TLM_VOLT,
	OW_TLM_CURRENT,
	OW_TLM_ERPM,
	OW_TLM_CONSUMPTION,
	OW_TLM_DEBUG1,
	OW_TLM_DEBUG2,
	OW_TLM_DEBUG3
};

extern uint8_t FETtecOneWire_ResponseLength[54];
extern uint8_t FETtecOneWire_RequestLength[54];
		

/*
	initialize FETtecOneWire protocol
*/
void FETtecOneWire_Init(void);

/*
	deinitialize FETtecOneWire protocol
*/
void FETtecOneWire_DeInit(void);

/*
	generates used 8 bit CRC
	crc = byte to be added to CRC
	crc_seed = CRC where it gets added too
	returns 8 bit CRC
*/
uint8_t FETtecOneWire_Update_crc8(uint8_t crc, uint8_t crc_seed);

/*
	generates used 8 bit CRC for arrays
	Buf = 8 bit byte array
	BufLen = count of bytes that should be used for CRC calculation
	returns 8 bit CRC
*/
uint8_t FETtecOneWire_Get_crc8(uint8_t *Buf, uint16_t BufLen);

/*
	transmitts a FETtecOneWire frame to a ESC
	ESC_id = id of the ESC
	Bytes = 8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
	Length = length of the Bytes array
	returns nothing
*/
void FETtecOneWire_Transmit(uint8_t ESC_id, uint8_t *Bytes, uint8_t Length);

/*
	reads the answer frame of a ESC
	Bytes = 8 bit byte array, where the received answer gets stored in
	Length = the expected answer length
	returnFullFrame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
	returns 1 if the expected answer frame was there, 0 if dont
*/
uint8_t FETtecOneWire_Receive(uint8_t *Bytes, uint8_t Length, uint8_t returnFullFrame);

/*
	makes all connected ESC's beep
	beepFreqency = a 8 bit value from 0-255. higher make a higher beep
*/
void FETtecOneWire_Beep(uint8_t beepFreqency);

/*
	sets the racewire color for all ESC's
	R, G, B = 8bit colors
*/
void FETtecOneWire_RW_LEDcolor(uint8_t R, uint8_t G, uint8_t B);

/*
	Resets a pending pull request
	returns nothing
*/
void FETtecOneWire_PullReset(void);

/*
	Pulls a complete request between for ESC
	ESC_id = id of the ESC
	command = 8bit array containing the command that thould be send including the possible payload
	response = 8bit array where the response will be stored in
	returnFullFrame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
	returns 1 if the request is completed, 0 if dont
*/
uint8_t FETtecOneWire_PullCommand(uint8_t ESC_id, uint8_t *command, uint8_t *response, uint8_t returnFullFrame);

/*
	scans for ESC's in bus. should be called intill FETtecOneWire_ScanActive >= 25
	returns the current scanned ID
*/
uint8_t FETtecOneWire_ScanESCs(void);

/*
	starts all ESC's in bus and prepares them for receiving teh fast throttle command should be called untill FETtecOneWire_SetupActive >= 25
	returns the current used ID
*/
uint8_t FETtecOneWire_InitESCs(void);

/*
	checks if the requested telemetry is available. 
	Telemetry = 16bit array where the read Telemetry will be stored in. 
	returns the telemetry request number or -1 if unavailable
*/
int8_t FETtecOneWire_CheckForTLM(uint16_t *Telemetry);

/*
	does almost all of the job.
	scans for ESC's if not already done.
	initializes the ESC's if not already done.
	sends fast throttle signals if init is complete.
	motorValues = a 16bit array containing the throttle signals that should be sent to the motors. 0-2000 where 1001-2000 is positive rotation and 999-0 reversed rotation
	Telemetry = 16bit array where the read telemetry will be stored in. 
	motorCount = the count of motors that should get values send
	tlmRequest = the requested telemetry type (OW_TLM_XXXXX)
	returns the telemetry request if telemetry was available, -1 if dont
*/
int8_t FETtecOneWire_ESCsSetValues(uint16_t *motorValues, uint16_t *Telemetry, uint8_t motorCount, uint8_t tlmRequest);