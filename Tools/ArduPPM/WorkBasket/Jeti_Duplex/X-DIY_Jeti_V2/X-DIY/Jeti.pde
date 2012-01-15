// menu structure
//
// [status] - [data1] - [data2] - [data3] - [data4] - [dataXY]
//     |_________|  _______|_________|_________|_________|
//     |   
// [SETUP1] - [SETUP2] - [SETUPXY]
//     |         |          |
// [para1.1] [PARAM2.1] [PARAMX.Y]
//     |         |          |
// [para1.2] [PARAM2.2] [PARAMX.Y]
//     |         |          |
// [para1.3] [PARAM2.3] [PARAMX.Y]

// Statusmenu
const uint8_t mnuSTATUS		= 0;
const uint8_t mnuGPS1 		= 1;
const uint8_t mnuGPS2 		= 2;
const uint8_t mnuIMU 		= 3;
const uint8_t mnuACC 		= 4;
const uint8_t mnuGYRO		= 5;
const uint8_t mnuCOMPASS	= 6;
const uint8_t mnuSERVO1		= 7;
const uint8_t mnuSERVO2		= 8;
const uint8_t mnuDATA9		= 9;

// Setup 1
const uint8_t mnuSETUP1 	= 10;
const uint8_t mnuPARAM11	= 11;
const uint8_t mnuPARAM12	= 12;
const uint8_t mnuPARAM13	= 13;
const uint8_t mnuPARAM14	= 14;

// Setup 2
const uint8_t mnuSETUP2 	= 20;
const uint8_t mnuPARAM21	= 21;
const uint8_t mnuPARAM22	= 22;
const uint8_t mnuPARAM23	= 23;
const uint8_t mnuPARAM24	= 24;

// Setup 3
const uint8_t mnuSETUP3 	= 30;
const uint8_t mnuPARAM31	= 31;
const uint8_t mnuPARAM32	= 32;
const uint8_t mnuPARAM33	= 33;
const uint8_t mnuPARAM34	= 34;

// Setup 4
const uint8_t mnuSETUP4 	= 40;
const uint8_t mnuPARAM41	= 41;
const uint8_t mnuPARAM42	= 42;
const uint8_t mnuPARAM43	= 43;
const uint8_t mnuPARAM44	= 44;

#define LCDMaxPos 32

static uint8_t MnuPtr = 0;
static uint8_t statustxt[LCDMaxPos];

void jeti_status(const char *str)
{
	byte i;
	byte length = strlen(str) + 1;
	if (length > LCDMaxPos) length = LCDMaxPos;
	for (i = 0; i<LCDMaxPos; i++) {
		if (i<length) {
			statustxt[i] = str[i];
		} else {
			statustxt[i] = 32; // if text < 32 chars -> fill up with SPACE
		}
	}
}

void jeti_menuctrl(uint8_t btns) {
	// Algorithm for menubuttons
	// if MnuPtr == 0 then Status
	// if MnuPtr <10 then Data
	// if (MnuPtr mod 10) = 0 then Setupp
	// if (MnuPtr mod 10) <> 0 then Parameter
	
	if (MnuPtr == mnuSTATUS) {				// MnuPtr = Status
		switch (btns) {
			case JB_key_right:
				MnuPtr += 1;
				break;

			case JB_key_left:
				MnuPtr = mnuDATA9;
				break;

			case JB_key_down:
				MnuPtr = mnuSETUP1;
				break;
		}
	}
	else if (MnuPtr < 10) {			// MnuPtr = Data
		switch (btns) {
			case JB_key_right:
				if (MnuPtr < mnuDATA9) MnuPtr += 1; else MnuPtr = mnuSTATUS;
				break;

			case JB_key_left:
				if (MnuPtr > mnuSTATUS) MnuPtr -= 1; else MnuPtr = mnuDATA9;
				break;
		}
	}
	else if (MnuPtr % 10 == 0) {	// MnuPtr = Setup
		switch (btns) {
			case JB_key_right:
				if (MnuPtr < mnuSETUP4) MnuPtr += 10; else MnuPtr = mnuSETUP1;
				break;

			case JB_key_left:
				if (MnuPtr > mnuSETUP1) MnuPtr -= 10; else MnuPtr = mnuSETUP4;
				break;
				
			case JB_key_up:
				MnuPtr = mnuSTATUS;
				break;

			case JB_key_down:
				MnuPtr += 1;
				break;
		}				
	}
	else {							// MnuPtr = Parameter
		switch (btns) {
			case JB_key_down:
				if ((MnuPtr % 10) < 4) MnuPtr += 1;
				break;

			case JB_key_up:
				if ((MnuPtr % 10) > 0) MnuPtr -= 1;
				break;
		}		
	}
}

void jeti_init() {
	// Init Jeti Serial Port
	JB.begin();
}

void jeti_update() {
	JB.clear(0);
	uint8_t Buttons = JB.readbuttons();
	jeti_menuctrl(Buttons);
	switch (MnuPtr) {
		case mnuGPS1:
			JB.print("GPS> ");	
			if (GPS.fix == 1) {
				JB.print("FIX");
				JB.print("  SAT>");
				JB.print((int)GPS.num_sats);
				JB.setcursor(LCDLine2);
				JB.print("Alt: ");
				JB.print((int)GPS.altitude/100);
			} else {
				JB.print("no FIX");
			}
			break;
			
		case mnuGPS2: 
			if (GPS.fix ==1 ) {
				JB.print("Lat ");
				JB.print((float)GPS.latitude/10000000, 9);
				JB.setcursor(LCDLine2);
				JB.print("Lon  ");
				JB.print((float)GPS.longitude/10000000, 9);
			} else {
				JB.print("no Data avail.");
			}
			break;
			
		case mnuIMU:
			//read_AHRS();
			JB.print("IMU>  R:");
			JB.print(roll_sensor/100,DEC);
			JB.print(" P:");
			JB.print(pitch_sensor/100,DEC);
			JB.setcursor(LCDLine2);
			JB.print("Y:");
			JB.print(yaw_sensor/100,DEC);
			break;
			
		case mnuACC:
			for (int i = 0; i < 6; i++) {
				AN[i] = APM_ADC.Ch(sensors[i]);
			}
			JB.print("ACC>  X:");
			JB.print((int)AN[3]);
			JB.setcursor(LCDLine2);
			JB.print(" Y:");
			JB.print((int)AN[4]);
			JB.print(" Z:");
			JB.print((int)AN[5]);
			break;

		case mnuGYRO:
			for (int i = 0; i < 6; i++) {
				AN[i] = APM_ADC.Ch(sensors[i]);
			}
			JB.print("GYRO>  Y:");
			JB.print((int)AN[0]);
			JB.setcursor(LCDLine2);
			JB.print(" R:");
			JB.print((int)AN[1]);
			JB.print(" P:");
			JB.print((int)AN[2]);
			break;
			
		case mnuCOMPASS:
			JB.print("MAGN>  Head:");
			JB.print((int)ToDeg(APM_Compass.Heading));
			JB.setcursor(LCDLine2);
			JB.println("[");			
			JB.print((int)APM_Compass.Mag_X);
			JB.print(comma);
			JB.print((int)APM_Compass.Mag_Y);
			JB.print(comma);
			JB.print((int)APM_Compass.Mag_Z);
			JB.println("]");			
			break;

		case mnuSERVO1:
			JB.print("#1:");
			JB.print((int)radio_in[CH_1]);
			JB.print(" #2:");
			JB.print((int)radio_in[CH_2]);
			JB.setcursor(LCDLine2);
			JB.print("#3:");
			JB.print((int)radio_in[CH_3]);
			JB.print(" #4:");
			JB.print((int)radio_in[CH_4]);
			break;

		case mnuSERVO2:
			JB.print("#Q:");
			JB.print((int)servo_out[CH_ROLL]/100);
			JB.print(" #H:");
			JB.print((int)servo_out[CH_PITCH]/100);
			JB.setcursor(LCDLine2);
			JB.print("#G:");
			JB.print((int)servo_out[CH_THROTTLE]);
			JB.print(" #S:");
			JB.print((int)servo_out[CH_RUDDER]/100);
			break;
			
		case mnuSETUP1:
			JB.print("SETUP 1");
			break;
		
		case mnuSTATUS: 
			for (byte i = 0; i<32; i++) {
				JB.print(statustxt[i]);
			}
			//JB.print("STATUS*");
			break;
		
		default:
			JB.print("Menu: n/a #");
			JB.print((int) MnuPtr);
			break;
	}
}
