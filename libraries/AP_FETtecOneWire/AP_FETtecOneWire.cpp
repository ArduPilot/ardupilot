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

#include "FETtecOneWire.h"

typedef struct FETtecOneWireESC {
    uint8_t inBootLoader;
	uint8_t firmWareVersion;
	uint8_t firmWareSubVersion;
	uint8_t ESCtype;
	uint8_t serialNumber[12];
} FETtecOneWireESC_t;

static uint8_t FETtecOneWire_activeESC_IDs[25] = {0};
static FETtecOneWireESC_t FETtecOneWire_foundESCs[25];
static uint8_t FETtecOneWire_FoundESCs = 0;
static uint8_t FETtecOneWire_ScanActive = 0;
static uint8_t FETtecOneWire_SetupActive  = 0;
static uint8_t FETtecOneWire_IgnoreOwnBytes = 0;
static int8_t FETtecOneWire_minID = 25;
static int8_t FETtecOneWire_maxID = 0;
static uint8_t FETtecOneWire_IDcount = 0;
static uint8_t FETtecOneWire_FastThrottleByteCount = 0;
static uint8_t FETtecOneWire_PullSuccess = 0;
static uint8_t FETtecOneWire_PullBusy = 0;
static uint8_t FETtecOneWire_TLM_request = 0;
static uint8_t FETtecOneWire_lastCRC = 0;
static uint8_t FETtecOneWire_firstInitDone = 0;

uint8_t FETtecOneWire_ResponseLength[54] = {
	[OW_OK] = 1,
	[OW_BL_PAGE_CORRECT] = 1, // BL only
	[OW_NOT_OK] = 1,
	[OW_BL_START_FW] = 0, // BL only
	[OW_BL_PAGES_TO_FLASH] = 1, // BL only
	[OW_REQ_TYPE] = 1,
	[OW_REQ_SN] = 12,
	[OW_REQ_SW_VER] = 2,
	[OW_RESET_TO_BL] = 0,
	[OW_THROTTLE] = 1,
	[OW_REQ_TLM] = 2,
	[OW_BEEP] = 0,

	[OW_SET_FAST_COM_LENGTH] = 1,
	
	[OW_GET_ROTATION_DIRECTION] = 1,
	[OW_SET_ROTATION_DIRECTION] = 1,
	
	[OW_GET_USE_SIN_START] = 1,
	[OW_SET_USE_SIN_START] = 1,
	
	[OW_GET_3D_MODE] = 1,
	[OW_SET_3D_MODE] = 1,
	
	[OW_GET_ID] = 1,
	[OW_SET_ID] = 1,

/*	
	[OW_GET_LINEAR_THRUST] = 1,
	[OW_SET_LINEAR_THRUST] = 1,
*/

	[OW_GET_EEVER] = 1,
	
	[OW_GET_PWM_MIN] = 2,
	[OW_SET_PWM_MIN] = 1,
	
	[OW_GET_PWM_MAX] = 2,
	[OW_SET_PWM_MAX] = 1,
	
	[OW_GET_ESC_BEEP] = 1,
	[OW_SET_ESC_BEEP] = 1,
	
	[OW_GET_CURRENT_CALIB] = 1,
	[OW_SET_CURRENT_CALIB] = 1,

	[OW_SET_LED_TMP_COLOR] = 0,
	[OW_GET_LED_COLOR] = 5,
	[OW_SET_LED_COLOR] = 1

};
uint8_t FETtecOneWire_RequestLength[54] = {
	[OW_OK] = 1,
	[OW_BL_PAGE_CORRECT] = 1, // BL only
	[OW_NOT_OK] = 1,
	[OW_BL_START_FW] = 1, // BL only
	[OW_BL_PAGES_TO_FLASH] = 1, // BL only
	[OW_REQ_TYPE] = 1,
	[OW_REQ_SN] = 1,
	[OW_REQ_SW_VER] = 1,
	[OW_RESET_TO_BL] = 1,
	[OW_THROTTLE] = 1,
	[OW_REQ_TLM] = 1,
	[OW_BEEP] = 2,

	[OW_SET_FAST_COM_LENGTH] = 4,
	
	[OW_GET_ROTATION_DIRECTION] = 1,
	[OW_SET_ROTATION_DIRECTION] = 1,
	
	[OW_GET_USE_SIN_START] = 1,
	[OW_SET_USE_SIN_START] = 1,
	
	[OW_GET_3D_MODE] = 1,
	[OW_SET_3D_MODE] = 1,
	
	[OW_GET_ID] = 1,
	[OW_SET_ID] = 1,

/*	
	[OW_GET_LINEAR_THRUST] = 1,
	[OW_SET_LINEAR_THRUST] = 1,
*/

	[OW_GET_EEVER] = 1,
	
	[OW_GET_PWM_MIN] = 1,
	[OW_SET_PWM_MIN] = 2,
	
	[OW_GET_PWM_MAX] = 1,
	[OW_SET_PWM_MAX] = 2,
	
	[OW_GET_ESC_BEEP] = 1,
	[OW_SET_ESC_BEEP] = 1,
	
	[OW_GET_CURRENT_CALIB] = 1,
	[OW_SET_CURRENT_CALIB] = 1,

	[OW_SET_LED_TMP_COLOR] = 4,
	[OW_GET_LED_COLOR] = 1,
	[OW_SET_LED_COLOR] = 5
};


/*
	initialize FETtecOneWire protocol
*/
void FETtecOneWire_Init(){
	if(FETtecOneWire_firstInitDone == 0){
		FETtecOneWire_FoundESCs = 0;
		FETtecOneWire_ScanActive = 0;
		FETtecOneWire_SetupActive  = 0;
		FETtecOneWire_minID = 25;
		FETtecOneWire_maxID = 0;
		FETtecOneWire_IDcount = 0;
		FETtecOneWire_FastThrottleByteCount = 0;
		for(uint8_t i = 0; i < 25; i++) FETtecOneWire_activeESC_IDs[i] = 0;
	}
	FETtecOneWire_IgnoreOwnBytes = 0;	
	FETtecOneWire_PullSuccess = 0;
	FETtecOneWire_PullBusy = 0;
	FETOW_UART_DEINIT;
	FETOW_UART_INIT;
	FETtecOneWire_firstInitDone = 1;
}

/*
	deinitialize FETtecOneWire protocol
*/
void FETtecOneWire_DeInit(){
	FETOW_UART_DEINIT;
	
}

/*
	generates used 8 bit CRC
	crc = byte to be added to CRC
	crc_seed = CRC where it gets added too
	returns 8 bit CRC
*/
uint8_t FETtecOneWire_UpdateCrc8(uint8_t crc, uint8_t crc_seed){
	uint8_t crc_u, i;
	crc_u = crc;
	crc_u ^= crc_seed;
	for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
	return (crc_u);
}

/*
	generates used 8 bit CRC for arrays
	Buf = 8 bit byte array
	BufLen = count of bytes that should be used for CRC calculation
	returns 8 bit CRC
*/
uint8_t FETtecOneWire_GetCrc8(uint8_t *Buf, uint16_t BufLen){
	uint8_t crc = 0;
	for(uint16_t i=0; i<BufLen; i++) crc = FETtecOneWire_UpdateCrc8(Buf[i], crc);
	return (crc);
}

/*
	transmitts a FETtecOneWire frame to a ESC
	ESC_id = id of the ESC
	Bytes = 8 bit array of bytes. Where byte 1 contains the command, and all following bytes can be the payload
	Length = length of the Bytes array
	returns nothing
*/
void FETtecOneWire_Transmit(uint8_t ESC_id, uint8_t *Bytes, uint8_t Length){
	/*
	a frame lookes like:
	byte 1 = fremae header (master is always 0x01)
	byte 2 = target ID (5bit)
	byte 3 & 4 = frame type (always 0x00, 0x00 used for bootloader. here just for compatibility)
	byte 5 = frame length over all bytes
	byte 6 - X = request type, followed by the payload
	byte X+1 = 8bit CRC
	*/
	uint8_t transmitArr[256] = {0x01,ESC_id,0x00,0x00};
	transmitArr[4] = Length+6; 
	for(uint8_t i = 0; i < Length; i++) transmitArr[i+5] = Bytes[i];
	transmitArr[Length+5] = FETtecOneWire_GetCrc8(transmitArr, Length+5); // crc
	FETOW_UART_SEND_BYTES(transmitArr, Length+6);
	FETtecOneWire_IgnoreOwnBytes += Length+6;
}

/*
	reads the answer frame of a ESC
	Bytes = 8 bit byte array, where the received answer gets stored in
	Length = the expected answer length
	returnFullFrame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
	returns 1 if the expected answer frame was there, 0 if dont
*/
uint8_t FETtecOneWire_Receive(uint8_t *Bytes, uint8_t Length, uint8_t returnFullFrame){
	/*
	a frame lookes like:
	byte 1 = fremae header (0x02 = bootloader, 0x03 = ESC firmware)
	byte 2 = sender ID (5bit)
	byte 3 & 4 = frame type (always 0x00, 0x00 used for bootloader. here just for compatibility)
	byte 5 = frame length over all bytes
	byte 6 - X = answer type, followed by the payload
	byte X+1 = 8bit CRC
	*/
	
	//ignore own bytes
	while(FETtecOneWire_IgnoreOwnBytes > 0 && FETOW_UART_BYTES_AVAILABLE){
		FETtecOneWire_IgnoreOwnBytes--;
		FETOW_UART_READ_BYTE;
	}
	// look for the real answer
	if(FETOW_UART_BYTES_AVAILABLE >= Length+6){
		// sync to frame starte byte
		uint8_t testFrameStart = 0;
		do{
			testFrameStart = FETOW_UART_READ_BYTE;
		}while(testFrameStart != 0x02 && testFrameStart != 0x03 && FETOW_UART_BYTES_AVAILABLE);
		// copy message
		if(FETOW_UART_BYTES_AVAILABLE >= Length+5){
			uint8_t ReceiveBuf[20] = {0};
			ReceiveBuf[0] = testFrameStart;
			for(uint8_t i=1; i<Length+6;i++) ReceiveBuf[i] = FETOW_UART_READ_BYTE;
			// check CRC
			if(FETtecOneWire_GetCrc8(ReceiveBuf, Length+5) == ReceiveBuf[Length+5]){
				if(!returnFullFrame) for(uint8_t i=0; i<Length; i++) Bytes[i] = ReceiveBuf[5+i];
				else for(uint8_t i=0; i<Length+6; i++) Bytes[i] = ReceiveBuf[i];
				return 1;
			}else return 0; // crc missmatch
		}else return 0; // no answer yet
	}else return 0; // no answer yet
}

/*
	makes all connected ESC's beep
	beepFreqency = a 8 bit value from 0-255. higher make a higher beep
*/
void FETtecOneWire_Beep(uint8_t beepFreqency){
	if(FETtecOneWire_IDcount > 0){
		uint8_t request[2] = {OW_BEEP, beepFreqency};
		uint8_t spacer[2] = {0,0};
		for(uint8_t i=FETtecOneWire_minID; i<FETtecOneWire_maxID+1; i++){
			FETtecOneWire_Transmit(i, request, FETtecOneWire_RequestLength[request[0]]);
			// add two zeros to make sure all ESC's can catch their command as we dont wait for a response here
			FETOW_UART_SEND_BYTES(spacer, 2);
			FETtecOneWire_IgnoreOwnBytes += 2;
		}
	}
}

/*
	sets the racewire color for all ESC's
	R, G, B = 8bit colors
*/
void FETtecOneWire_RW_LEDcolor(uint8_t R, uint8_t G, uint8_t B){
	if(FETtecOneWire_IDcount > 0){
		uint8_t request[4] = {OW_SET_LED_TMP_COLOR, R, G, B};
		uint8_t spacer[2] = {0,0};
		for(uint8_t i=FETtecOneWire_minID; i<FETtecOneWire_maxID+1; i++){
			FETtecOneWire_Transmit(i, request, FETtecOneWire_RequestLength[request[0]]);
			// add two zeros to make sure all ESC's can catch their command as we dont wait for a response here
			FETOW_UART_SEND_BYTES(spacer, 2);
			FETtecOneWire_IgnoreOwnBytes += 2;
		}
	}
}

/*
	Resets a pending pull request
	returns nothing
*/
void FETtecOneWire_PullReset(){
	FETtecOneWire_PullSuccess = 0;
	FETtecOneWire_PullBusy = 0;	
}

/*
	Pulls a complete request between for ESC
	ESC_id = id of the ESC
	command = 8bit array containing the command that thould be send including the possible payload
	response = 8bit array where the response will be stored in
	returnFullFrame can be OW_RETURN_RESPONSE or OW_RETURN_FULL_FRAME
	returns 1 if the request is completed, 0 if dont
*/
uint8_t FETtecOneWire_PullCommand(uint8_t ESC_id, uint8_t *command, uint8_t *response, uint8_t returnFullFrame){
	if(!FETtecOneWire_PullBusy){
		FETtecOneWire_PullBusy = 1;
		FETtecOneWire_PullSuccess = 0;
		FETtecOneWire_Transmit(ESC_id, command, FETtecOneWire_RequestLength[command[0]]);
	}else{
		if(FETtecOneWire_Receive(response, FETtecOneWire_ResponseLength[command[0]], returnFullFrame)){
			FETtecOneWire_PullSuccess = 1;
			FETtecOneWire_PullBusy = 0;
		}
	}
	return FETtecOneWire_PullSuccess;
}


/*
	scans for ESC's in bus. should be called intill FETtecOneWire_ScanActive >= 25
	returns the currend scanned ID
*/
uint8_t FETtecOneWire_ScanESCs(){
	static uint16_t delayLoops = 500;
	static uint8_t scanID = 0;
	static uint8_t scanState = 0;
	static uint8_t scanTimeOut = 0;
	uint8_t response[18] = {0};
	uint8_t request[1] = {0};
	if(FETtecOneWire_ScanActive == 0){
		delayLoops = 500;
		scanID = 0;
		scanState = 0;
		scanTimeOut = 0;	
		return FETtecOneWire_ScanActive+1;
	}
	if(delayLoops > 0){
		delayLoops--;
		return FETtecOneWire_ScanActive;
	}
	if(scanID < FETtecOneWire_ScanActive){
		scanID = FETtecOneWire_ScanActive;
		scanState = 0;
		scanTimeOut = 0;
	}
	if(scanTimeOut == 3 || scanTimeOut == 6 || scanTimeOut == 9 || scanTimeOut == 12) FETtecOneWire_PullReset();
	if(scanTimeOut < 15){
		switch(scanState){
			case 0:
				request[0] = OW_OK;
				if(FETtecOneWire_PullCommand(scanID, request, response, OW_RETURN_FULL_FRAME)){
					scanTimeOut = 0;
					FETtecOneWire_activeESC_IDs[scanID] = 1;
					FETtecOneWire_FoundESCs++;
					if(response[0] == 0x02) FETtecOneWire_foundESCs[scanID].inBootLoader = 1;
					else FETtecOneWire_foundESCs[scanID].inBootLoader = 0;
					delayLoops = 1;
					scanState++;
				}else scanTimeOut++;
			break;
			case 1:
				request[0] = OW_REQ_TYPE;
				if(FETtecOneWire_PullCommand(scanID, request, response, OW_RETURN_RESPONSE)){
					scanTimeOut = 0;
					FETtecOneWire_foundESCs[scanID].ESCtype = response[0];
					delayLoops = 1;
					scanState++;
				}else scanTimeOut++;
			break;
			case 2:
				request[0] = OW_REQ_SW_VER;
				if(FETtecOneWire_PullCommand(scanID, request, response, OW_RETURN_RESPONSE)){
					scanTimeOut = 0;
					FETtecOneWire_foundESCs[scanID].firmWareVersion = response[0];
					FETtecOneWire_foundESCs[scanID].firmWareSubVersion = response[1];
					delayLoops = 1;
					scanState++;
				}else scanTimeOut++;
			break;
			case 3:
				request[0] = OW_REQ_SN;
				if(FETtecOneWire_PullCommand(scanID, request, response, OW_RETURN_RESPONSE)){
					scanTimeOut = 0;
					for(uint8_t i=0; i<12; i++) FETtecOneWire_foundESCs[scanID].serialNumber[i] = response[i];
					delayLoops = 1;
					return scanID+1;
				}else scanTimeOut++;
			break;
		}
	}else{
		FETtecOneWire_PullReset();
		return scanID+1;
	}
	return scanID;
}

/*
	starts all ESC's in bus and prepares them for receiving teh fast throttle command should be called untill FETtecOneWire_SetupActive >= 25
	returns the current used ID
*/
uint8_t FETtecOneWire_InitESCs(){
	static uint8_t delayLoops = 0;
	static uint8_t activeID = 1;
	static uint8_t State = 0;
	static uint8_t TimeOut = 0;
	static uint8_t wakeFromBL = 1;
	static uint8_t setFastCommand[4] = {OW_SET_FAST_COM_LENGTH,0,0,0};
	uint8_t response[18] = {0};
	uint8_t request[1] = {0};
	if(FETtecOneWire_SetupActive == 0){
		delayLoops = 0;
		activeID = 1;
		State = 0;
		TimeOut = 0;
		wakeFromBL = 1;
		return FETtecOneWire_SetupActive+1;
	}
	while(FETtecOneWire_activeESC_IDs[FETtecOneWire_SetupActive] == 0 && FETtecOneWire_SetupActive < 25) FETtecOneWire_SetupActive++;
	
	if(FETtecOneWire_SetupActive == 25 && wakeFromBL == 0) return FETtecOneWire_SetupActive;
	else if(FETtecOneWire_SetupActive == 25 && wakeFromBL){
		wakeFromBL = 0;
		activeID = 1;
		FETtecOneWire_SetupActive = 1;
		State = 0;
		TimeOut = 0;		
		
		FETtecOneWire_minID = 25;
		FETtecOneWire_maxID = 0;
		FETtecOneWire_IDcount = 0;
		for(uint8_t i=0; i<25; i++){
			if(FETtecOneWire_activeESC_IDs[i] != 0){
				FETtecOneWire_IDcount++;
				if(i < FETtecOneWire_minID) FETtecOneWire_minID = i;
				if(i > FETtecOneWire_maxID) FETtecOneWire_maxID = i;
			}
		}
		
		if(FETtecOneWire_IDcount == 0 || FETtecOneWire_maxID-FETtecOneWire_minID > FETtecOneWire_IDcount-1){ // loop forever
			wakeFromBL = 1;
			return activeID;	
		}
		FETtecOneWire_FastThrottleByteCount = 1; 
		int8_t bitCount = 12+(FETtecOneWire_IDcount*11);
		while(bitCount > 0){
			FETtecOneWire_FastThrottleByteCount++;
			bitCount -= 8;
		}
		setFastCommand[1] = FETtecOneWire_FastThrottleByteCount; // just for older ESC FW versions since 1.0 001 this byte is ignored as the ESC calculates it itself
		setFastCommand[2] = FETtecOneWire_minID; // min ESC id
		setFastCommand[3] = FETtecOneWire_IDcount; // count of ESC's that will get signals
		
	}
	
	if(delayLoops > 0){
		delayLoops--;
		return FETtecOneWire_SetupActive;
	}
	
	if(activeID < FETtecOneWire_SetupActive){
		activeID = FETtecOneWire_SetupActive;
		State = 0;
		TimeOut = 0;
	}
	
	if(TimeOut == 3 || TimeOut == 6 || TimeOut == 9 || TimeOut == 12) FETtecOneWire_PullReset();
	
	if(TimeOut < 15){
		if(wakeFromBL){
			switch(State){
				case 0:
					request[0] = OW_BL_START_FW;
					if(FETtecOneWire_foundESCs[activeID].inBootLoader == 1){
						FETtecOneWire_Transmit(activeID, request, FETtecOneWire_RequestLength[request[0]]);
						delayLoops = 5;
					}else return activeID+1;
					State = 1;
				break;
				case 1:
					request[0] = OW_OK;
					if(FETtecOneWire_PullCommand(activeID, request, response, OW_RETURN_FULL_FRAME)){
						TimeOut = 0;
						if(response[0] == 0x02){
							FETtecOneWire_foundESCs[activeID].inBootLoader = 1;
							State = 0;
						}else{
							FETtecOneWire_foundESCs[activeID].inBootLoader = 0;
							delayLoops = 1;
							return activeID+1;
						}
					}else TimeOut++;
				break;
			}
		}else{
			if(FETtecOneWire_PullCommand(activeID, setFastCommand, response, OW_RETURN_RESPONSE)){
				TimeOut = 0;
				delayLoops = 1;
				return activeID+1;
			}else TimeOut++;	
		}
	}else{
		FETtecOneWire_PullReset();
		return activeID+1;
	}
	return activeID;	
	
}

/*
	checks if the requested telemetry is available. 
	Telemetry = 16bit array where the read Telemetry will be stored in. 
	returns the telemetry request number or -1 if unavailable
*/
int8_t FETtecOneWire_CheckForTLM(uint16_t *Telemetry){
	int8_t return_TLM_request = 0;
	if(FETtecOneWire_IDcount > 0){
		// empty buffer
		while(FETtecOneWire_IgnoreOwnBytes > 0 && FETOW_UART_BYTES_AVAILABLE){
			FETOW_UART_READ_BYTE;
			FETtecOneWire_IgnoreOwnBytes--;
		}
		
		// first two byte are the ESC Telemetry of the first ESC. next two byte of the second....
		if(FETOW_UART_BYTES_AVAILABLE == (FETtecOneWire_IDcount*2)+1){
			// look if first byte in buffer is equal to last byte of throttle command (crc)
			if(FETOW_UART_READ_BYTE == FETtecOneWire_lastCRC){
				for(uint8_t i=0; i<FETtecOneWire_IDcount; i++){
					Telemetry[i] = FETOW_UART_READ_BYTE<<8;
					Telemetry[i] |= FETOW_UART_READ_BYTE;
				}
				return_TLM_request = FETtecOneWire_TLM_request;
			}else return_TLM_request = -1;
		}else return_TLM_request = -1;
	}else return_TLM_request = -1;
	return return_TLM_request;
}

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
int8_t FETtecOneWire_ESCsSetValues(uint16_t *motorValues, uint16_t *Telemetry, uint8_t motorCount, uint8_t tlmRequest){
	static uint32_t lastTime = 0;
	int8_t return_TLM_request = -2;
	
	// init should not be done too fast. as at last the bootloader has some timing requirements with messages. so loop delays must fit more or less
	if(FETtecOneWire_ScanActive < 25 || FETtecOneWire_SetupActive < 25){
		if((uint32_t)micros() -(uint32_t)lastTime < 700) return 0;
		else lastTime = micros();
		
		if(FETtecOneWire_ScanActive < 25){
			// scan for all ESC's in onewire bus
			FETtecOneWire_ScanActive = FETtecOneWire_ScanESCs();
		}else if(FETtecOneWire_SetupActive < 25){
			if(FETtecOneWire_FoundESCs == 0){
				FETtecOneWire_ScanActive = 0;
			}else{
				// check if in bootloader, start ESC's FW if they are and prepare fast throttle command
				FETtecOneWire_SetupActive = FETtecOneWire_InitESCs();
			}
		}
	}else{
		//send fast throttle signals
		if(FETtecOneWire_IDcount > 0){
			
			// check for telemetry
			return_TLM_request = FETtecOneWire_CheckForTLM(Telemetry);
			FETtecOneWire_TLM_request = tlmRequest;
			
			//prepare fast throttle signals
			uint16_t useSignals[24] = {0};
			uint8_t OneWireFastThrottleCommand[36] = {0};
			if(motorCount > FETtecOneWire_IDcount) motorCount = FETtecOneWire_IDcount;
			for(uint8_t i=0; i < motorCount; i++) useSignals[i] = constrain(motorValues[i], 0, 2000);
			
			uint8_t actThrottleCommand = 0;
			
			// byte 1:
			// bit 0 = TLMrequest, bit 1,2,3 = TLM type, bit 4 = first bit of first ESC (11bit)signal, bit 5,6,7 = frame header
			// so ABBBCDDD
			// A = TLM request yes or no
			// B = TLM request type (temp, volt, current, erpm, consumption, debug1, debug2, debug3)
			// C = first bit from first throttle signal
			// D = frame header
			OneWireFastThrottleCommand[0] = 128 | (FETtecOneWire_TLM_request << 4);
			OneWireFastThrottleCommand[0] |= ((useSignals[actThrottleCommand] >> 10) & 0x01) << 3;
			OneWireFastThrottleCommand[0] |= 0x01;
			
			// byte 2:
			// AAABBBBB
			// A = next 3 bits from (11bit)throttle signal
			// B = 5bit target ID 
			OneWireFastThrottleCommand[1] = (((useSignals[actThrottleCommand] >> 7) & 0x07)) << 5;
			OneWireFastThrottleCommand[1] |= ALL_ID;
			
			// following bytes are the rest 7 bit of the first (11bit)throttle signal, and all bit from all other signals, followed by the CRC byte
			uint8_t BitsLeftFromCommand = 7;
			uint8_t actByte = 2;
			uint8_t bitsFromByteLeft = 8;
			uint8_t bitsToAddLeft = (12 + (((FETtecOneWire_maxID - FETtecOneWire_minID) + 1) * 11)) - 16;
			while (bitsToAddLeft > 0) {
				if (bitsFromByteLeft >= BitsLeftFromCommand) {
					OneWireFastThrottleCommand[actByte] |= (useSignals[actThrottleCommand] & ((1 << BitsLeftFromCommand) - 1)) << (bitsFromByteLeft - BitsLeftFromCommand);
					bitsToAddLeft -= BitsLeftFromCommand;
					bitsFromByteLeft -= BitsLeftFromCommand;
					actThrottleCommand++;
					BitsLeftFromCommand = 11;
					if (bitsToAddLeft == 0) {
						actByte++;
						bitsFromByteLeft = 8;
					}
				} else {
					OneWireFastThrottleCommand[actByte] |= (useSignals[actThrottleCommand] >> (BitsLeftFromCommand - bitsFromByteLeft)) & ((1 << bitsFromByteLeft) - 1);
					bitsToAddLeft -= bitsFromByteLeft;
					BitsLeftFromCommand -= bitsFromByteLeft;
					actByte++;
					bitsFromByteLeft = 8;
					if (BitsLeftFromCommand == 0) {
						actThrottleCommand++;
						BitsLeftFromCommand = 11;
					}
				}
			}
			// empty buffer
			while(FETOW_UART_BYTES_AVAILABLE) FETOW_UART_READ_BYTE;
			
			// send throttle signal
			OneWireFastThrottleCommand[FETtecOneWire_FastThrottleByteCount-1] = FETtecOneWire_GetCrc8(OneWireFastThrottleCommand, FETtecOneWire_FastThrottleByteCount-1);
			FETOW_UART_SEND_BYTES(OneWireFastThrottleCommand, FETtecOneWire_FastThrottleByteCount);
			// last byte of signal can be used to make sure the first TLM byte is correct, in case of spike corruption
			FETtecOneWire_IgnoreOwnBytes = FETtecOneWire_FastThrottleByteCount-1;
			FETtecOneWire_lastCRC = OneWireFastThrottleCommand[FETtecOneWire_FastThrottleByteCount-1];
			// the ESC's will answer the TLM as 16bit each ESC, so 2byte each ESC. 
		}
	}
	return return_TLM_request; // returns the readed tlm as it is 1 loop delayed
}













