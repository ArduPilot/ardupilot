//////////////////////////////////////////////////////////////////////////////
//
//	Copyright 2013 by Horizon Hobby, Inc.
//	All Rights Reserved Worldwide.
//
//	Released to Public Domain
//
//	This header file may be incorporated into non-Horizon
//	products.
//
//////////////////////////////////////////////////////////////////////////////
//
//	Author:		AK
//	Date:		2017-02-24
//	Mods:		Sync to Spektrum internal version by matching sequence of
//				structs, formatting, etc.  Also changed some structs from
//				having "id" to "identifier."  Also redefined "spare" in Rx MAH
//				to provide more bits for "chargeUsed" fields.
//
#ifndef TELEMETRY_H
#define	TELEMETRY_H

//////////////////////////////////////////////////////////////////////////////
//
//				TELEMETRY SENSOR I2C ADDRESSES & DEVICE TYPES
//
//////////////////////////////////////////////////////////////////////////////

#define	TELE_DEVICE_NODATA			(0x00)										// No data in packet, but telemetry is alive
#define	TELE_DEVICE_VOLTAGE			(0x01)										// High-Voltage sensor (INTERNAL)
#define	TELE_DEVICE_TEMPERATURE		(0x02)										// Temperature Sensor (INTERNAL)
#define	TELE_DEVICE_AMPS			(0x03)										// Amps (INTERNAL)
#define	TELE_DEVICE_RSV_04			(0x04)										// Reserved
#define	TELE_DEVICE_FLITECTRL		(0x05)										// Flight Controller Status Report
#define	TELE_DEVICE_RSV_06			(0x06)										// Reserved
#define	TELE_DEVICE_RSV_07			(0x07)										// Reserved
#define	TELE_DEVICE_RSV_08			(0x08)										// Reserved
//#define	DO_NOT_USE				(0x09)										// DO NOT USE!
#define	TELE_DEVICE_PBOX			(0x0A)										// PowerBox
#define	TELE_DEVICE_LAPTIMER		(0x0B)										// Lap Timer
#define	TELE_DEVICE_TEXTGEN			(0x0C)										// Text Generator
#define	TELE_DEVICE_VTX				(0x0D)										// Video Transmitter Feedback
#define	TELE_DEVICE_RSV_0E			(0x0E)										// Reserved
#define	TELE_DEVICE_RSV_0F			(0x0F)										// Reserved
#define	TELE_DEVICE_RSV_10			(0x10)										// Reserved
#define	TELE_DEVICE_AIRSPEED		(0x11)										// Air Speed (Eagle Tree Sensor)
#define	TELE_DEVICE_ALTITUDE		(0x12)										// Altitude (Eagle Tree Sensor)
#define	TELE_DEVICE_RSV_13			(0x13)										// Reserved
#define	TELE_DEVICE_GMETER			(0x14)										// G-Force (Eagle Tree Sensor)
#define	TELE_DEVICE_JETCAT			(0x15)										// Turbine interface (Eagle Tree)
#define	TELE_DEVICE_GPS_LOC			(0x16)										// GPS Location Data (Eagle Tree)
#define	TELE_DEVICE_GPS_STATS		(0x17)										// GPS Status (Eagle Tree)
#define	TELE_DEVICE_RX_MAH			(0x18)										// Receiver Pack Capacity (Dual)
#define	TELE_DEVICE_JETCAT_2		(0x19)										// Turbine interface, message 2 format (Eagle Tree)
#define	TELE_DEVICE_GYRO			(0x1A)										// 3-axis gyro
#define	TELE_DEVICE_ATTMAG			(0x1B)										// Attitude and Magnetic Compass
#define	TELE_DEVICE_TILT			(0x1C)										// Surface Tilt Sensor
#define	TELE_DEVICE_RSV_1D			(0x1D)										// Reserved
#define	TELE_DEVICE_AS6X_GAIN		(0x1E)										// Active AS6X Gains (new mode)
#define	TELE_DEVICE_AS3X_LEGACYGAIN	(0x1F)										// Active AS3X Gains for legacy mode
#define	TELE_DEVICE_ESC				(0x20)										// Electronic Speed Control
#define	TELE_DEVICE_RSV_21			(0x21)										// Reserved
#define	TELE_DEVICE_FUEL			(0x22)										// Fuel Flow Meter
#define	TELE_DEVICE_RSV_23			(0x23)										// Reserved
#define	TELE_DEVICE_ALPHA6			(0x24)										// Alpha6 Stabilizer
#define	TELE_DEVICE_RSV_25			(0x25)										// Reserved
#define	TELE_DEVICE_GPS_BINARY		(0x26)										// GPS, binary format
#define	TELE_DEVICE_RSV_27			(0x27)										// Reserved
#define	TELE_DEVICE_RSV_28			(0x28)										// Reserved
#define	TELE_DEVICE_RSV_29			(0x29)										// Reserved
#define	TELE_DEVICE_RSV_2A			(0x2A)										// Reserved
#define	TELE_DEVICE_RSV_2B			(0x2B)										// Reserved
#define	TELE_DEVICE_RSV_2C			(0x2C)										// Reserved
#define	TELE_DEVICE_RSV_2D			(0x2D)										// Reserved
#define	TELE_DEVICE_RSV_2E			(0x2E)										// Reserved
#define	TELE_DEVICE_RSV_2F			(0x2F)										// Reserved
//#define	DO_NOT_USE				(0x30)										// Internal ST sensor
//#define	DO_NOT_USE				(0x32)										// Internal ST sensor
#define	TELE_DEVICE_RSV_33			(0x33)										// Reserved
#define	TELE_DEVICE_FP_MAH			(0x34)										// Flight Battery Capacity (Dual)
#define	TELE_DEVICE_RSV_35			(0x35)										// Reserved
#define	TELE_DEVICE_DIGITAL_AIR		(0x36)										// Digital Inputs & Tank Pressure
#define	TELE_DEVICE_RSV_37			(0x37)										// Reserved
#define	TELE_DEVICE_STRAIN			(0x38)										// Thrust/Strain Gauge
#define	TELE_DEVICE_RSV_39			(0x39)										// Reserved
#define	TELE_DEVICE_LIPOMON			(0x3A)										// 6S Cell Monitor (LiPo taps)
#define	TELE_DEVICE_RSV_3B			(0x3B)										// Reserved
#define	TELE_DEVICE_RSV_3C			(0x3C)										// Reserved
#define	TELE_DEVICE_RSV_3D			(0x3D)										// Reserved
#define	TELE_DEVICE_RSV_3E			(0x3E)										// Reserved
#define	TELE_DEVICE_LIPOMON_14		(0x3F)										// 14S Cell Monitor (LiPo taps)
#define	TELE_DEVICE_VARIO_S			(0x40)										// Vario
#define	TELE_DEVICE_RSV_41			(0x41)										// Reserved
#define	TELE_DEVICE_SMARTBATT		(0x42)										// Spektrum SMART Battery (multiple structs)
#define	TELE_DEVICE_RSV_43			(0x43)										// Reserved
#define	TELE_DEVICE_RSV_44			(0x44)										// Reserved
#define	TELE_DEVICE_RSV_45			(0x45)										// Reserved
#define	TELE_DEVICE_RSV_46			(0x46)										// Reserved
#define	TELE_DEVICE_RSV_47			(0x47)										// Reserved
#define	TELE_DEVICE_RSV_48			(0x48)										// Reserved
#define	TELE_DEVICE_RSV_49			(0x49)										// Reserved
#define	TELE_DEVICE_RSV_4A			(0x4A)										// Reserved
#define	TELE_DEVICE_RSV_4B			(0x4B)										// Reserved
#define	TELE_DEVICE_RSV_4C			(0x4C)										// Reserved
#define	TELE_DEVICE_RSV_4D			(0x4D)										// Reserved
#define	TELE_DEVICE_RSV_4E			(0x4E)										// Reserved
#define	TELE_DEVICE_RSV_4F			(0x4F)										// Reserved
#define	TELE_DEVICE_USER_16SU		(0x50)										// User-Defined, STRU_TELE_USER_16SU
#define	TELE_DEVICE_RSV_51			(0x51)										// Reserved
#define	TELE_DEVICE_USER_16SU32U	(0x52)										// User-Defined, STRU_TELE_USER_16SU32U
#define	TELE_DEVICE_RSV_53			(0x53)										// Reserved
#define	TELE_DEVICE_USER_16SU32S	(0x54)										// User-Defined, STRU_TELE_USER_16SU32S
#define	TELE_DEVICE_RSV_55			(0x55)										// Reserved
#define	TELE_DEVICE_USER_16U32SU	(0x56)										// User-Defined, STRU_TELE_USER_16U32SU
#define	TELE_DEVICE_RSV_57			(0x57)										// Reserved
#define	TELE_DEVICE_RSV_58			(0x58)										// Reserved
#define	TELE_DEVICE_MULTICYLINDER	(0x59)										// Multi-cylinder temp sensor
#define	TELE_DEVICE_RSV_5A			(0x5A)										// Reserved
#define	TELE_DEVICE_RSV_5B			(0x5B)										// Reserved
#define	TELE_DEVICE_RSV_5C			(0x5C)										// Reserved
#define	TELE_DEVICE_RSV_5D			(0x5D)										// Reserved
#define	TELE_DEVICE_RSV_5E			(0x5E)										// Reserved
#define	TELE_DEVICE_RSV_5F			(0x5F)										// Reserved
#define	TELE_DEVICE_VSPEAK			(0x60)										// Reserved for V-Speak
#define	TELE_DEVICE_SMOKE_EL		(0x61)										// Reserved for Smoke-EL.de
#define	TELE_DEVICE_CROSSFIRE		(0x62)										// Reserved for Crossfire devices
#define	TELE_DEVICE_RSV_63			(0x63)										// Reserved
#define	TELE_DEVICE_RSV_64			(0x64)										// Reserved
#define	TELE_DEVICE_RSV_65			(0x65)										// Reserved
#define	TELE_DEVICE_EXTRF			(0x66)										// Reserved for Generic External RF sources
#define	TELE_DEVICE_RSV_67			(0x67)										// Reserved
#define	TELE_DEVICE_RSV_68			(0x68)										// Reserved
#define	TELE_DEVICE_RSV_69			(0x69)										// Reserved
#define	TELE_DEVICE_RSV_6A			(0x6A)										// Reserved
//#define	DO_NOT_USE				(0x6B)										// DO NOT USE!
#define	TELE_DEVICE_RSV_6C			(0x6C)										// Reserved
#define	TELE_DEVICE_RSV_6D			(0x6D)										// Reserved
#define	TELE_DEVICE_RSV_6E			(0x6E)										// Reserved
#define	TELE_DEVICE_RSV_6F			(0x6F)										// Reserved
#define	TELE_DEVICE_RSV_70			(0x70)										// Reserved
#define	TELE_XRF_LINKSTATUS			(0x71)										// External RF Link Status
#define	TELE_DEVICE_RSV_72			(0x72)										// Reserved
#define	TELE_DEVICE_RSV_73			(0x73)										// Reserved
#define	TELE_DEVICE_RSV_74			(0x74)										// Reserved
#define	TELE_DEVICE_RSV_75			(0x75)										// Reserved
#define	TELE_DEVICE_RSV_76			(0x76)										// Reserved
#define	TELE_DEVICE_RSV_77			(0x77)										// Reserved
#define	TELE_DEVICE_RSV_78			(0x78)										// Reserved
#define	TELE_DEVICE_RSV_79			(0x79)										// Reserved
#define	TELE_DEVICE_RSV_7A			(0x7A)										// Reserved
#define	TELE_DEVICE_ALT_ZERO		(0x7B)										// Pseudo-device setting Altitude "zero"
#define	TELE_DEVICE_RTC				(0x7C)										// Pseudo-device giving timestamp
#define	TELE_DEVICE_RPM				(0x7E)										// RPM sensor
#define	TELE_DEVICE_QOS				(0x7F)										// RxV + flight log data
#define	TELE_DEVICE_MAX				(0x7F)										// Last address available

#define	TELE_DEVICE_SHORTRANGE		(0x80)										// OR this bit to indicate data is from a short-range telemetry device (e.g. TM1100)

#define	TELE_DEVICE_MAX_PROGRAM		(0x70)										// Last programmable address when using sID

//////////////////////////////////////////////////////////////////////////////
//
//							TELEMETRY
//					DEVICE-SPECIFIC STRUCTURES
//
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT DATA SIGNED/UNSIGNED
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;															// Source device = 0x50
	UINT8	sID;																// Secondary ID
	INT16	sField1,															// Signed 16-bit data fields
			sField2,
			sField3;
	UINT16	uField1, 															// Unsigned 16-bit data fields
			uField2,
			uField3,
			uField4;
} STRU_TELE_USER_16SU;

//////////////////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT SIGNED/UNSIGNED AND 32-BIT UNSIGNED
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;															// Source device = 0x52
	UINT8	sID;																// Secondary ID
	INT16	sField1,															// Signed 16-bit data fields
			sField2;
	UINT16	uField1, 															// Unsigned 16-bit data fields
			uField2,
			uField3;
	UINT32	u32Field; 															// Unsigned 32-bit data field
} STRU_TELE_USER_16SU32U;

//////////////////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT SIGNED/UNSIGNED AND 32-BIT SIGNED
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;															// Source device = 0x54
	UINT8	sID;																// Secondary ID
	INT16	sField1,															// Signed 16-bit data fields
			sField2;
	UINT16	uField1, 															// Unsigned 16-bit data fields
			uField2,
			uField3;
	INT32	s32Field; 															// Signed 32-bit data field
} STRU_TELE_USER_16SU32S;

//////////////////////////////////////////////////////////////////////////////
//
//		THIRD-PARTY 16-BIT UNSIGNED AND 32-BIT SIGNED/UNSIGNED
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;															// Source device = 0x56
	UINT8	sID;																// Secondary ID
	UINT16	uField1; 															// Unsigned 16-bit data field
	INT32	s32Field; 															// Signed 32-bit data field
	UINT32	u32Field1, 															// Unsigned 32-bit data fields
			u32Field2;
} STRU_TELE_USER_16U32SU;

//////////////////////////////////////////////////////////////////////////////
//
//							POWERBOX
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8	identifier;															// Source device = 0x0A
	UINT8	sID;																// Secondary ID
	UINT16	volt1;																// Volts, 0.01v
	UINT16	volt2;																// Volts, 0.01v
	UINT16	capacity1;															// mAh, 1mAh
	UINT16	capacity2;															// mAh, 1mAh
	UINT16	spare16_1;
	UINT16	spare16_2;
	UINT8	spare;
	UINT8	alarms;																// Alarm bitmask (see below)
} STRU_TELE_POWERBOX;

#define	TELE_PBOX_ALARM_VOLTAGE_1			(0x01)
#define	TELE_PBOX_ALARM_VOLTAGE_2			(0x02)
#define	TELE_PBOX_ALARM_CAPACITY_1			(0x04)
#define	TELE_PBOX_ALARM_CAPACITY_2			(0x08)
//#define	TELE_PBOX_ALARM_RPM					(0x10)
//#define	TELE_PBOX_ALARM_TEMPERATURE			(0x20)
#define	TELE_PBOX_ALARM_RESERVED_1			(0x40)
#define	TELE_PBOX_ALARM_RESERVED_2			(0x80)

//////////////////////////////////////////////////////////////////////////////
//
//							VOLTAGE
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x01
	UINT8		sID;															// Secondary ID
	UINT16		volts;															// 0.01V increments
} STRU_TELE_HV;

//////////////////////////////////////////////////////////////////////////////
//
//							TEMPERATURE
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x02
	UINT8		sID;															// Secondary ID
	INT16		temperature;													// Temperature in degrees Fahrenheit
} STRU_TELE_TEMP;

//////////////////////////////////////////////////////////////////////////////
//
//						RX CAPACITY METER
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x18
	UINT8		sID;															// Secondary ID
	INT16		current_A;														// Instantaneous current, 0.01A (0-328.7A)		7FFF-> no data
	UINT16		chargeUsed_A;													// Integrated mAh used, 0.1mAh (0-3276.6mAh)
	UINT16		volts_A;														// Volts, 0.01V increments (0-16.00V)
	INT16		current_B;														// Instantaneous current, 0.01A (0-328.7A)		7FFF-> no data/sensor B
	UINT16		chargeUsed_B;													// Integrated mAh used, 0.1mAh (0-3276.6mAh)
	UINT16		volts_B;														// Volts, 0.01V increments (0-16.00V)
	UINT8		alerts,															// Bit mapped alert conditions (see below)
				highCharge;														// High nybble is extra bits for chargeUsed_B, Low is for chargeUsed_A
} STRU_TELE_RX_MAH;

#define	RXMAH_PS_ALERT_NONE			(0)											// No alarms
#define	RXMAH_PS_ALERT_RF_INT		(1 << 0)									// A or internal Remote failure
#define	RXMAH_PS_ALERT_RF_ANT1		(1 << 1)									// B remote power fault
#define	RXMAH_PS_ALERT_RF_ANT2		(1 << 2)									// L remote power fault
#define	RXMAH_PS_ALERT_RF_ANT3		(1 << 3)									// R remote power fault
#define	RXMAH_PS_ALERT_OVERVOLT_A	(1 << 4) 									// Battery A is over voltage
#define	RXMAH_PS_ALERT_OVERVOLT_B	(1 << 5) 									// Battery A is over voltage
#define	RXMAH_PS_ALERT_RFU1			(1 << 6)
#define	RXMAH_PS_ALERT_RFU2			(1 << 7)

//////////////////////////////////////////////////////////////////////////////
//
//							HIGH-CURRENT
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x03
	UINT8		sID;															// Secondary ID
	INT16		current,														// Range: +/- 150A     Resolution: 300A / 2048 = 0.196791 A/count
				dummy;															// TBD
} STRU_TELE_IHIGH;

#define	IHIGH_RESOLUTION_FACTOR						((FP32)(0.196791))

//////////////////////////////////////////////////////////////////////////////
//
//							SIMPLE VARIO
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x40
	UINT8		sID;															// Secondary ID
	INT16		altitude;														// .1m increments
	INT16		delta_0250ms,													// change in altitude last 250ms, 0.1m/s increments
				delta_0500ms,													// change in altitude last 500ms, 0.1m/s increments
				delta_1000ms,													// change in altitude last 1.0 seconds, 0.1m/s increments
				delta_1500ms,													// change in altitude last 1.5 seconds, 0.1m/s increments
				delta_2000ms,													// change in altitude last 2.0 seconds, 0.1m/s increments
				delta_3000ms;													// change in altitude last 3.0 seconds, 0.1m/s increments
} STRU_TELE_VARIO_S;

//////////////////////////////////////////////////////////////////////////////
//
//							ALTIMETER
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;
	UINT8		sID;															// Secondary ID
	INT16		altitude;														// .1m increments
	INT16		maxAltitude;													// .1m increments
} STRU_TELE_ALT;																// Eagle Tree Sensor

//////////////////////////////////////////////////////////////////////////////
//
//							AIRSPEED
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;
	UINT8		sID;															// Secondary ID
	UINT16		airspeed;														// 1 km/h increments
	UINT16		maxAirspeed;													// 1 km/h increments
} STRU_TELE_SPEED;																// Eagle Tree Sensor

//////////////////////////////////////////////////////////////////////////////
//
//							LAP TIMER
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;
	UINT8		sID;															// Secondary ID
	UINT8		lapNumber;														// Lap last finished
	UINT8		gateNumber;														// Last gate passed
	UINT32		lastLapTime;													// Time of lap in 1ms increments (NOT duration)
	UINT32		gateTime;														// Duration between last 2 gates
	UINT8		unused[4];
} STRU_TELE_LAPTIMER;

//////////////////////////////////////////////////////////////////////////////
//
//							TEXT GENERATOR
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;
	UINT8		sID;															// Secondary ID
	UINT8		lineNumber;														// Line number to display (0 = title, 1-8 for general, 254 = Refresh backlight, 255 = Erase all text on screen)
	char		text[13];														// 0-terminated text when < 13 chars
} STRU_TELE_TEXTGEN;

//////////////////////////////////////////////////////////////////////////////
//
//							VIDEO TRANSMITTER (VTX)
//
//////////////////////////////////////////////////////////////////////////////
//
//	VTX spec subject to change. Refer to Spektrum VTX Interfacing document for latest info
//
typedef struct
{
	UINT8		identifier;
	UINT8		sID;															// Secondary ID
	UINT8		band;															// VTX Band (0 = Fatshark, 1 = Raceband, 2 = E, 3 = B, 4 = A, 5-7 = Reserved)
	UINT8		channel;														// VTX Channel (0-7)
	UINT8		pit;															// Pit/Race mode (0 = Race, 1 = Pit). Race = (normal operating) mode. Pit = (reduced power) mode. When PIT is set, it overrides all other power settings.
	UINT8		power;															// VTX Power (0 = Off, 1 = 1mw to 14mW, 2 = 15mW to 25mW, 3 = 26mW to 99mW, 4 = 100mW to 299mW, 5 = 300mW to 600mW, 6 = 601mW+, 7 = manual control)
	UINT16		powerDec;														// VTX Power as a decimal 1mw/unit
	UINT8		region;															// Region (0 = USA, 1 = EU, 0xFF = Not Provided)
	UINT8		unused[7];														// reserved
} STRU_TELE_VTX;

//////////////////////////////////////////////////////////////////////////////
//
//							ESC
//
//////////////////////////////////////////////////////////////////////////////
//
//	Uses big-endian byte order
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x20
	UINT8		sID;															// Secondary ID
	UINT16		RPM;															// Electrical RPM, 10RPM (0-655340 RPM)  0xFFFF --> "No data"
	UINT16		voltsInput;														// Volts, 0.01v (0-655.34V)       0xFFFF --> "No data"
	UINT16		tempFET;														// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	UINT16		currentMotor;													// Current, 10mA (0-655.34A)      0xFFFF --> "No data"
	UINT16		tempBEC;														// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	UINT8		currentBEC;														// BEC Current, 100mA (0-25.4A)   0xFF ----> "No data"
	UINT8		voltsBEC;														// BEC Volts, 0.05V (0-12.70V)    0xFF ----> "No data"
	UINT8		throttle;														// 0.5% (0-100%)                  0xFF ----> "No data"
	UINT8		powerOut;														// Power Output, 0.5% (0-127%)    0xFF ----> "No data"
} STRU_TELE_ESC;

//////////////////////////////////////////////////////////////////////////////
//
//		(Liquid) Fuel Flow/Capacity (Two Tanks/Engines)
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x22
	UINT8		sID;															// Secondary ID
	UINT16		fuelConsumed_A;													// Integrated fuel consumption, 0.1mL
	UINT16		flowRate_A;														// Instantaneous consumption, 0.01mL/min
	UINT16		temp_A;															// Temperature, 0.1C (0-655.34C)
	UINT16		fuelConsumed_B;													// Integrated fuel consumption, 0.1mL
	UINT16		flowRate_B;														// Instantaneous consumption, 0.01mL/min
	UINT16		temp_B;															// Temperature, 0.1C (0-655.34C)
	UINT16		spare;															// Not used
} STRU_TELE_FUEL;

//////////////////////////////////////////////////////////////////////////////
//
//		Battery Current/Capacity (Flight Pack Capacity)
//
//////////////////////////////////////////////////////////////////////////////
//
// AK 2013-11-19 make struct align with 0x03 device
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x34
	UINT8		sID;															// Secondary ID
	INT16		current_A;														// Instantaneous current, 0.1A (0-3276.6A)
	INT16		chargeUsed_A;													// Integrated mAh used, 1mAh (0-32.766Ah)
	UINT16		temp_A;															// Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
	INT16		current_B;														// Instantaneous current, 0.1A (0-3276.6A)
	INT16		chargeUsed_B;													// Integrated mAh used, 1mAh (0-32.766Ah)
	UINT16		temp_B;															// Temperature, 0.1C (0-150C, 0x7FFF indicates not populated)
	UINT16		spare;															// Not used
} STRU_TELE_FP_MAH;

//////////////////////////////////////////////////////////////////////////////
//
//		Digital Input Status (Retract Status) and Tank Pressure
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = TELE_DEVICE_DIGITAL_AIR
	UINT8		sID;															// Secondary ID
	UINT16		digital;														// Digital inputs (bit per input)
	UINT16		spare1;
	UINT16		pressure[4];													// Tank pressure, 0.1PSI (0-6553.4PSI), 0xFFFF = Not Installed
	UINT16		spare2;
} STRU_TELE_DIGITAL_AIR;

//////////////////////////////////////////////////////////////////////////////
//
//		Thrust/Strain Gauge
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x38
	UINT8		sID;															// Secondary ID
	UINT16		strain_A,														// Strain sensor A
				strain_B,														// Strain sensor B
				strain_C,														// Strain sensor D
				strain_D;														// Strain sensor C
} STRU_TELE_STRAIN;

//////////////////////////////////////////////////////////////////////////////
//
//						6S LiPo Cell Monitor
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x3A
	UINT8		sID;															// Secondary ID
	UINT16		cell[6];														// Voltage across cell 1, .01V steps
																				// 0x7FFF --> cell not present
	UINT16		temp;															// Temperature, 0.1C (0-655.34C)
} STRU_TELE_LIPOMON;

//////////////////////////////////////////////////////////////////////////////
//
//						14S LiPo Cell Monitor
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x3F
	UINT8		sID;															// Secondary ID
	UINT8		cell[14];														// Voltage across cell 1, .01V steps, excess of 2.56V
																				// (ie, 3.00V would report 300-256 = 44)
																				// 0xFF --> cell not present
} STRU_TELE_LIPOMON_14;

//////////////////////////////////////////////////////////////////////////////
//
//								Smart Battery
//
//////////////////////////////////////////////////////////////////////////////
//
//			Uses little-endian byte order for all multi-byte fields
//
typedef struct
{
	UINT8	identifier;															// Source device = 0x42
	UINT8	sID;																// Secondary ID
	UINT8	typeChannel;														// Upper nybble = Message type; Lower nybble = Battery number (0 or 1)
	UINT8	msgData[13];														// Message-specific data, determined by upper nybble of typeChannel (see defs below)
} STRU_SMARTBATT_HEADER;

#define	SMARTBATT_MSG_TYPE_MASK_BATTNUMBER		(0x0F)
#define	SMARTBATT_MSG_TYPE_MASK_MSGTYPE			(0xF0)

#define SMARTBATT_MSG_TYPE_REALTIME				(0x00)
#define SMARTBATT_MSG_TYPE_CELLS_1_6			(0x10)
#define SMARTBATT_MSG_TYPE_CELLS_7_12			(0x20)
#define SMARTBATT_MSG_TYPE_CELLS_13_18			(0x30)
#define SMARTBATT_MSG_TYPE_ID					(0x80)
#define SMARTBATT_MSG_TYPE_LIMITS				(0x90)

//...........................................................................
// Real-time battery data when current sense is available
typedef struct
{
	UINT8		identifier;														// Source device = 0x42
	UINT8		sID;															// Secondary ID
	UINT8		typeChannel;													// Msg type = SMARTBATT_MSG_TYPE_REALTIME | Battery number (0 or 1)
	INT8		temperature_C;													// Temperature in degrees C, 1 degree increments (-128 = unavailable)
	UINT32		dischargeCurrent_mA;											// Amount of current being drawn from battery, in mA steps (0xFFFFFFFF = unavailable)
	UINT16		batteryCapacityUsage_mAh;										// Approximate battery capacity usage, in mAh (0xFFFF = unavailable)
	UINT16		minCellVoltage_mV;												// Minimum cell voltage of pack, in mV
	UINT16		maxCellVoltage_mV;												// Maximum cell voltage of pack, in mV
	UINT8		rfu[2];
} STRU_SMARTBATT_REALTIME;

//...........................................................................
// Real-time cell voltage
typedef struct
{
	UINT8		identifier;														// Source device = 0x42
	UINT8		sID;															// Secondary ID
	UINT8		typeChannel;													// Msg type = SMARTBATT_MSG_TYPE_CELLS_X_Y | Battery number (0 or 1)
	INT8		temperature_C;													// Temperature in degrees C, 1 degree increments (-128 = unavailable)
	UINT16		cellVoltage_mV[6];												// Cell voltage of first 6 cells, in mV (0xFFFF = unavailable)
} STRU_SMARTBATT_CELLS;

//...........................................................................
// Smart Battery ID and general info
typedef struct
{
	UINT8		identifier;														// Source device = 0x42
	UINT8		sID;															// Secondary ID
	UINT8		typeChannel;													// Msg type = SMARTBATT_MSG_TYPE_ID | Battery number (0 or 1)
	UINT8		chemistry;														// 0:LiHv, 1:LiPo, 2:LiIon, 3:LiFe, 4:Pb, 5:Ni-MH/Cd
	UINT8		numOfCells;														// Number of cells in the battery
	UINT8		manufacturer;													// 0:BattGo
	UINT16		cycles;															// Number of charge/discharge cycles recorded (0 = unavailable)
	UINT8		uniqueID[8];													// Unique battery ID, manufacturer-specific
																				// 0: [0]   = lower (first) byte of "Customer ID"
																				//    [1-3] = lower 3 bytes of "Special Mark of Battery"
																				//    [4-7] = 4-byte "Manufacturing Date"
} STRU_SMARTBATT_ID;

//...........................................................................
// Smart Battery Limits
typedef struct
{
	UINT8		identifier;														// Source device = 0x42
	UINT8		sID;															// Secondary ID
	UINT8		typeChannel;													// Msg type = SMARTBATT_MSG_TYPE_LIMITS | Battery number (0 or 1)
	UINT8		rfu;
	UINT16		fullCapacity_mAh;												// Fully charged battery capacity, in mAh
	UINT16		dischargeCurrentRating;											// Rated discharge current, in 0.1C
	UINT16		overDischarge_mV;												// Limit below which battery is likely damaged, in mV
	UINT16		zeroCapacity_mV;												// Voltage at which LVC protection should activate, in mV
	UINT16		fullyCharged_mV;												// Voltage reading expected when fully charged, in mV
	INT8		minWorkingTemp;													// Minimum working temperature in degrees C, 1 degree steps
	INT8		maxWorkingTemp;													// Maximum working temperature in degrees C, 1 degree steps
} STRU_SMARTBATT_LIMITS;

//////////////////////////////////////////////////////////////////////////////
//
//							ACCELEROMETER
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x14
	UINT8		sID;															// Secondary ID
	INT16		GForceX;														// force is reported as .01G increments
	INT16		GForceY;														// 		Range = +/-4000 (+/- 40G) in Pro model
	INT16		GForceZ;														// 		Range = +/-800 (+/- 8G) in Standard model
	INT16		maxGForceX;														// abs(max G X-axis)   FORE/AFT
	INT16		maxGForceY;														// abs (max G Y-axis)  LEFT/RIGHT
	INT16		maxGForceZ;														// max G Z-axis        WING SPAR LOAD
	INT16		minGForceZ;														// min G Z-axis        WING SPAR LOAD
} STRU_TELE_G_METER;

//////////////////////////////////////////////////////////////////////////////
//
//						SURFACE TILT (ATTITUDE) SENSOR
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x1C TELE_DEVICE_TILT
	UINT8		sID;															// Secondary ID
	INT16		attQuatX;														// Quaternion representing attitude using RHR. X component in Q14.
	INT16		attQuatY;														// Y component in Q14.
	INT16		attQuatZ;														// Z component in Q14.
	INT16		attQuatW;														// W component in Q14.
	UINT16		spare[3];
} STRU_TELE_TILT;

//////////////////////////////////////////////////////////////////////////////
//
//								TURBINE
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x15
	UINT8		sID;															// Secondary ID
	UINT8		status;															// Table below
	UINT8		throttle;														// (BCD) xx Percent
	UINT16		packVoltage;													// (BCD) xx.yy
	UINT16		pumpVoltage;													// (BCD) xx.yy
	UINT32		RPM;															// (BCD)
	UINT16		EGT;															// (BCD) Temperature, Celsius
	UINT8		offCondition;													// Table below
	UINT8		spare;
} STRU_TELE_JETCAT;

enum JETCAT_ECU_TURBINE_STATE {							// ECU Status definitions
		JETCAT_ECU_STATE_OFF = 0x00,
		JETCAT_ECU_STATE_WAIT_for_RPM = 0x01, // (Stby/Start)
		JETCAT_ECU_STATE_Ignite = 0x02,
		JETCAT_ECU_STATE_Accelerate = 0x03,
		JETCAT_ECU_STATE_Stabilise = 0x04,
		JETCAT_ECU_STATE_Learn_HI = 0x05,
		JETCAT_ECU_STATE_Learn_LO = 0x06,
		JETCAT_ECU_STATE_UNDEFINED = 0x07,
		JETCAT_ECU_STATE_Slow_Down = 0x08,
		JETCAT_ECU_STATE_Manual = 0x09,
		JETCAT_ECU_STATE_AutoOff = 0x10,
		JETCAT_ECU_STATE_Run = 0x11, // (reg.)
		JETCAT_ECU_STATE_Accleleration_delay = 0x12,
		JETCAT_ECU_STATE_SpeedReg = 0x13, // (Speed Ctrl)
		JETCAT_ECU_STATE_Two_Shaft_Regulate = 0x14, // (only for secondary shaft)
		JETCAT_ECU_STATE_PreHeat1 = 0x15,
		JETCAT_ECU_STATE_PreHeat2 = 0x16,
		JETCAT_ECU_STATE_MainFStart = 0x17,
		JETCAT_ECU_STATE_NotUsed = 0x18,
		JETCAT_ECU_STATE_KeroFullOn = 0x19,
		// undefined states 0x1A-0x1F
		EVOJET_ECU_STATE_off = 0x20,
		EVOJET_ECU_STATE_ignt = 0x21,
		EVOJET_ECU_STATE_acce = 0x22,
		EVOJET_ECU_STATE_run = 0x23,
		EVOJET_ECU_STATE_cal = 0x24,
		EVOJET_ECU_STATE_cool = 0x25,
		EVOJET_ECU_STATE_fire = 0x26,
		EVOJET_ECU_STATE_glow = 0x27,
		EVOJET_ECU_STATE_heat = 0x28,
		EVOJET_ECU_STATE_idle = 0x29,
		EVOJET_ECU_STATE_lock = 0x2A,
		EVOJET_ECU_STATE_rel = 0x2B,
		EVOJET_ECU_STATE_spin = 0x2C,
		EVOJET_ECU_STATE_stop = 0x2D,
		// undefined states 0x2E-0x2F
		HORNET_ECU_STATE_OFF = 0x30,
		HORNET_ECU_STATE_SLOWDOWN = 0x31,
		HORNET_ECU_STATE_COOL_DOWN = 0x32,
		HORNET_ECU_STATE_AUTO = 0x33,
		HORNET_ECU_STATE_AUTO_HC = 0x34,
		HORNET_ECU_STATE_BURNER_ON = 0x35,
		HORNET_ECU_STATE_CAL_IDLE = 0x36,
		HORNET_ECU_STATE_CALIBRATE = 0x37,
		HORNET_ECU_STATE_DEV_DELAY = 0x38,
		HORNET_ECU_STATE_EMERGENCY = 0x39,
		HORNET_ECU_STATE_FUEL_HEAT = 0x3A,
		HORNET_ECU_STATE_FUEL_IGNITE = 0x3B,
		HORNET_ECU_STATE_GO_IDLE = 0x3C,
		HORNET_ECU_STATE_PROP_IGNITE = 0x3D,
		HORNET_ECU_STATE_RAMP_DELAY = 0x3E,
		HORNET_ECU_STATE_RAMP_UP = 0x3F,
		HORNET_ECU_STATE_STANDBY = 0x40,
		HORNET_ECU_STATE_STEADY = 0x41,
		HORNET_ECU_STATE_WAIT_ACC = 0x42,
		HORNET_ECU_STATE_ERROR = 0x43,
		// undefined states 0x44-0x4F
		XICOY_ECU_STATE_Temp_High = 0x50,
		XICOY_ECU_STATE_Trim_Low = 0x51,
		XICOY_ECU_STATE_Set_Idle = 0x52,
		XICOY_ECU_STATE_Ready = 0x53,
		XICOY_ECU_STATE_Ignition = 0x54,
		XICOY_ECU_STATE_Fuel_Ramp = 0x55,
		XICOY_ECU_STATE_Glow_Test = 0x56,
		XICOY_ECU_STATE_Running = 0x57,
		XICOY_ECU_STATE_Stop = 0x58,
		XICOY_ECU_STATE_Flameout = 0x59,
		XICOY_ECU_STATE_Speed_Low = 0x5A,
		XICOY_ECU_STATE_Cooling = 0x5B,
		XICOY_ECU_STATE_Igniter_Bad = 0x5C,
		XICOY_ECU_STATE_Starter_F = 0x5D,
		XICOY_ECU_STATE_Weak_Fuel = 0x5E,
		XICOY_ECU_STATE_Start_On = 0x5F,
		XICOY_ECU_STATE_Pre_Heat = 0x60,
		XICOY_ECU_STATE_Battery = 0x61,
		XICOY_ECU_STATE_Time_Out = 0x62,
		XICOY_ECU_STATE_Overload = 0x63,
		XICOY_ECU_STATE_Igniter_Fail = 0x64,
		XICOY_ECU_STATE_Burner_On = 0x65,
		XICOY_ECU_STATE_Starting = 0x66,
		XICOY_ECU_STATE_SwitchOver = 0x67,
		XICOY_ECU_STATE_Cal_Pump = 0x68,
		XICOY_ECU_STATE_Pump_Limit = 0x69,
		XICOY_ECU_STATE_No_Engine = 0x6A,
		XICOY_ECU_STATE_Pwr_Boost = 0x6B,
		XICOY_ECU_STATE_Run_Idle = 0x6C,
		XICOY_ECU_STATE_Run_Max = 0x6D,
		// undefined states 0x6e-0x73
		JETCENT_ECU_STATE_STOP = 0x74,
		JETCENT_ECU_STATE_GLOW_TEST = 0x75,
		JETCENT_ECU_STATE_STARTER_TEST = 0x76,
		JETCENT_ECU_STATE_PRIME_FUEL = 0x77,
		JETCENT_ECU_STATE_PRIME_BURNER = 0x78,
		JETCENT_ECU_STATE_MAN_COOL = 0x79,
		JETCENT_ECU_STATE_AUTO_COOL = 0x7A,
		JETCENT_ECU_STATE_IGN_HEAT = 0x7B,
		JETCENT_ECU_STATE_IGNITION = 0x7C,
		JETCENT_ECU_STATE_PREHEAT = 0x7D,
		JETCENT_ECU_STATE_SWITCHOVER = 0x7E,
		JETCENT_ECU_STATE_TO_IDLE = 0x7F,
		JETCENT_ECU_STATE_RUNNING = 0x80,
		JETCENT_ECU_STATE_STOP_ERROR = 0x81,
		// undefined states 0x82-0x8F
		SWIWIN_ECU_STATE_STOP = 0x90,
		SWIWIN_ECU_STATE_READY = 0x91,
		SWIWIN_ECU_STATE_IGNITION = 0x92,
		SWIWIN_ECU_STATE_PREHEAT = 0x93,
		SWIWIN_ECU_STATE_FUEL_RAMP = 0x94,
		SWIWIN_ECU_STATE_RUNNING = 0x95,
		SWIWIN_ECU_STATE_COOLING = 0x96,
		SWIWIN_ECU_STATE_RESTART_SWOVER = 0x97,
		SWIWIN_ECU_STATE_NOTUSED = 0x98,
		// undefined states 0x99-0x9F

		TURBINE_ECU_MAX_STATE = 0x9F
};

enum JETCAT_ECU_OFF_CONDITIONS {					// ECU off conditions. Valid only when the ECUStatus = JETCAT_ECU_STATE_OFF
		JETCAT_ECU_OFF_No_Off_Condition_defined = 0,
		JETCAT_ECU_OFF_Shut_down_via_RC,
		JETCAT_ECU_OFF_Overtemperature,
		JETCAT_ECU_OFF_Ignition_timeout,
		JETCAT_ECU_OFF_Acceleration_time_out,
		JETCAT_ECU_OFF_Acceleration_too_slow,
		JETCAT_ECU_OFF_Over_RPM,
		JETCAT_ECU_OFF_Low_Rpm_Off,
		JETCAT_ECU_OFF_Low_Battery,
		JETCAT_ECU_OFF_Auto_Off,
		JETCAT_ECU_OFF_Low_temperature_Off,
		JETCAT_ECU_OFF_Hi_Temp_Off,
		JETCAT_ECU_OFF_Glow_Plug_defective,
		JETCAT_ECU_OFF_Watch_Dog_Timer,
		JETCAT_ECU_OFF_Fail_Safe_Off,
		JETCAT_ECU_OFF_Manual_Off, // (via GSU)
		JETCAT_ECU_OFF_Power_fail, // (Battery fail)
		JETCAT_ECU_OFF_Temp_Sensor_fail, // (only during startup)
		JETCAT_ECU_OFF_Fuel_fail,
		JETCAT_ECU_OFF_Prop_fail,
		JETCAT_ECU_OFF_2nd_Engine_fail,
		JETCAT_ECU_OFF_2nd_Engine_Diff_Too_High,
		JETCAT_ECU_OFF_2nd_Engine_No_Comm,
		JETCAT_ECU_MAX_OFF_COND,
		// Jet Central
		JETCENT_ECU_OFF_No_Off_Condition_defined = 24,		// ECU off conditions. Valid only when the ECUStatus = JETCENT_ECU_STATE_STOP or JETCENT_ECU_STATE_STOP_ERROR or JETCENT_ECU_STATE_RUNNING
		JETCENT_ECU_OFF_IGNITION_ERROR,
		JETCENT_ECU_OFF_PREHEAT_ERROR,
		JETCENT_ECU_OFF_SWITCHOVER_ERROR,
		JETCENT_ECU_OFF_STARTER_MOTOR_ERROR,
		JETCENT_ECU_OFF_TO_IDLE_ERROR,
		JETCENT_ECU_OFF_ACCELERATION_ERROR,
		JETCENT_ECU_OFF_IGNITER_BAD,
		JETCENT_ECU_OFF_MIN_PUMP_OK,
		JETCENT_ECU_OFF_MAX_PUMP_OK,
		JETCENT_ECU_OFF_LOW_RX_BATTERY,
		JETCENT_ECU_OFF_LOW_ECU_BATTERY,
		JETCENT_ECU_OFF_NO_RX,
		JETCENT_ECU_OFF_TRIM_DOWN,
		JETCENT_ECU_OFF_TRIM_UP,
		JETCENT_ECU_OFF_FAILSAFE,
		JETCENT_ECU_OFF_FULL,
		JETCENT_ECU_OFF_RX_SETUP_ERROR,
		JETCENT_ECU_OFF_TEMP_SENSOR_ERROR,
		JETCENT_ECU_OFF_COM_TURBINE_ERROR,
		JETCENT_ECU_OFF_MAX_TEMP,
		JETCENT_ECU_OFF_MAX_AMPS,
		JETCENT_ECU_OFF_LOW_RPM,
		JETCENT_ECU_OFF_ERROR_RPM_SENSOR,
		JETCENT_ECU_OFF_MAX_PUMP,
		JETCENT_ECU_MAX_OFF_COND
};

typedef struct
{
	UINT8		identifier;														// Source device = 0x19
	UINT8		sID;															// Secondary ID
	UINT16		FuelFlowRateMLMin;												// (BCD) mL per Minute
	UINT32		RestFuelVolumeInTankML;											// (BCD) mL remaining in tank
	UINT8		ECUbatteryPercent;												// (BCD) % battery pack capacity remaining
	// 7 bytes left
} STRU_TELE_JETCAT2;

//////////////////////////////////////////////////////////////////////////////
//
//								GPS
//						  Packed-BCD Type
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x16
	UINT8		sID;															// Secondary ID
	UINT16		altitudeLow;													// BCD, meters, format 3.1 (Low order of altitude)
	UINT32		latitude;														// BCD, format 4.4, Degrees * 100 + minutes, less than 100 degrees
	UINT32		longitude;														// BCD, format 4.4 , Degrees * 100 + minutes, flag indicates > 99 degrees
	UINT16		course;															// BCD, 3.1
	UINT8		HDOP;															// BCD, format 1.1
	UINT8		GPSflags;														// see definitions below
} STRU_TELE_GPS_LOC;

typedef struct
{
	UINT8		identifier;														// Source device = 0x17
	UINT8		sID;															// Secondary ID
	UINT16		speed;															// BCD, knots, format 3.1
	UINT32		UTC;															// BCD, format HH:MM:SS.S, format 6.1
	UINT8		numSats;														// BCD, 0-99
	UINT8		altitudeHigh;													// BCD, meters, format 2.0 (High order of altitude)
} STRU_TELE_GPS_STAT;

// GPS flags definitions:
#define	GPS_INFO_FLAGS_IS_NORTH_BIT					(0)
#define	GPS_INFO_FLAGS_IS_NORTH						(1 << GPS_INFO_FLAGS_IS_NORTH_BIT)
#define	GPS_INFO_FLAGS_IS_EAST_BIT					(1)
#define	GPS_INFO_FLAGS_IS_EAST						(1 << GPS_INFO_FLAGS_IS_EAST_BIT)
#define	GPS_INFO_FLAGS_LONGITUDE_GREATER_99_BIT		(2)
#define	GPS_INFO_FLAGS_LONGITUDE_GREATER_99			(1 << GPS_INFO_FLAGS_LONGITUDE_GREATER_99_BIT)
#define	GPS_INFO_FLAGS_GPS_FIX_VALID_BIT			(3)
#define	GPS_INFO_FLAGS_GPS_FIX_VALID				(1 << GPS_INFO_FLAGS_GPS_FIX_VALID_BIT)
#define	GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT		(4)
#define	GPS_INFO_FLAGS_GPS_DATA_RECEIVED			(1 << GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT)
#define	GPS_INFO_FLAGS_3D_FIX_BIT					(5)
#define	GPS_INFO_FLAGS_3D_FIX						(1 << GPS_INFO_FLAGS_3D_FIX_BIT)
#define GPS_INFO_FLAGS_NEGATIVE_ALT_BIT				(7)
#define GPS_INFO_FLAGS_NEGATIVE_ALT					(1 << GPS_INFO_FLAGS_NEGATIVE_ALT_BIT)

//////////////////////////////////////////////////////////////////////////////
//
//								GPS
//							Binary Type
//
//		NOTE:	Data resolution for all fields matches Crossfire EXCEPT speed.
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x26
	UINT8		sID;															// Secondary ID
	UINT16		altitude;														// m, 1000m offset
	INT32		latitude;														// degree / 10,000,000
	INT32		longitude;														// degree / 10,000,000
	UINT16		heading;														// degree / 10
	UINT8		groundSpeed;													// km/h
	UINT8		numSats;														// count
} STRU_TELE_GPS_BINARY;

//////////////////////////////////////////////////////////////////////////////
//
//					AS3X Legacy Gain Report
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = TELE_DEVICE_AS3X_LEGACYGAIN
	UINT8		sID;															// Secondary ID
	UINT8		gainRoll;														// Configured normal gains per axis
	UINT8		gainPitch;
	UINT8		gainYaw;
	UINT8		headRoll;														// Configured heading hold gains per axis
	UINT8		headPitch;
	UINT8		headYaw;
	UINT8		activeRoll;														// Active gains per axis (as affected by FM channel)
	UINT8		activePitch;
	UINT8		activeYaw;
	UINT8		flightMode;														// bit 7 1 --> FM present in bits 0,1 except 0xFF --> not present
	UINT8		unused[4];
} STRU_TELE_AS3X_LEGACY;

//////////////////////////////////////////////////////////////////////////////
//
//					AS6X Gain Report (AS3X Legacy + more fields)
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = TELE_DEVICE_AS6X_GAIN
	UINT8		sID;															// Secondary ID
	UINT8		gainRoll;														// Configured normal gains per axis
	UINT8		gainPitch;
	UINT8		gainYaw;
	UINT8		headRoll;														// Configured heading hold gains per axis
	UINT8		headPitch;
	UINT8		headYaw;
	UINT8		activeRoll;														// Active gains per axis (as affected by FM channel)
	UINT8		activePitch;
	UINT8		activeYaw;
	UINT8		flightMode;														// bit 7 1 --> FM present in bits 0,1 except 0xFF --> not present
	// new fields go here:
	UINT8		unused[4];
} STRU_TELE_AS6X_GAIN;

//////////////////////////////////////////////////////////////////////////////
//
//							GYRO
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x1A
	UINT8		sID;															// Secondary ID
	INT16		gyroX;															// Rotation rates of the body - Rate is about the X Axis which is defined out the nose of the vehicle.
	INT16		gyroY;															// Units are 0.1 deg/sec  - Rate is about the Y Axis which is define out the right wing of the vehicle.
	INT16		gyroZ;															// Rate is about the Z axis which is defined down from the vehicle.
	INT16		maxGyroX;														// Max rates (absolute value)
	INT16		maxGyroY;
	INT16		maxGyroZ;
} STRU_TELE_GYRO;

//////////////////////////////////////////////////////////////////////////////
//
//						Alpha6 Stabilizer
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x24
	UINT8		sID;															// Secondary ID
	UINT16		volts;															// 0.01V increments
	UINT8		state_FM;														// Flight Mode and System State (see below)
	UINT8		gainRoll,														// Roll Gain,  high bit --> Heading Hold
				gainPitch,														// Pitch Gain
				gainYaw;														// Yaw Gain
	INT16		attRoll,														// Roll Attitude, 0.1degree, RHR
				attPitch,														// Pitch Attitude
				attYaw;															// Yaw Attitude
	UINT16		spare;
} STRU_TELE_ALPHA6;

#define	GBOX_STATE_BOOT							(0x00)							// Alpha6 State - Boot
#define	GBOX_STATE_INIT							(0x01)							// Init
#define	GBOX_STATE_READY						(0x02)							// Ready
#define	GBOX_STATE_SENSORFAULT					(0x03)							// Sensor Fault
#define	GBOX_STATE_POWERFAULT					(0x04)							// Power Fault
#define	GBOX_STATE_MASK							(0x0F)

#define	GBOX_FMODE_FM0							(0x00)							// FM0 through FM4
#define	GBOX_FMODE_FM1							(0x10)
#define	GBOX_FMODE_FM2							(0x20)
#define	GBOX_FMODE_FM3							(0x30)
#define	GBOX_FMODE_FM4							(0x40)
#define	GBOX_FMODE_PANIC						(0x50)
#define	GBOX_FMODE_MASK							(0xF0)

//////////////////////////////////////////////////////////////////////////////
//
//						ATTITUDE & MAG COMPASS
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x1B
	UINT8		sID;															// Secondary ID
	INT16		attRoll;														// Attitude, 3 axes.  Roll is a rotation about the X Axis of the vehicle using the RHR.
	INT16		attPitch;														// Units are 0.1 deg - Pitch is a rotation about the Y Axis of the vehicle using the RHR.
	INT16		attYaw;															// Yaw is a rotation about the Z Axis of the vehicle using the RHR.
	INT16		magX;															// Magnetic Compass, 3 axes
	INT16		magY;															// Units are 0.1mG
	INT16		magZ;															//
	UINT16		heading;														// Heading, 0.1deg
} STRU_TELE_ATTMAG;

//////////////////////////////////////////////////////////////////////////////
//
//						Altitude "Zero" Message
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x7B
	UINT8		sID;															// Secondary ID
	UINT8		spare[2];
	UINT32		altOffset;														// Altitude "zero" log
} STRU_TELE_ALT_ZERO;

//////////////////////////////////////////////////////////////////////////////
//
//						Real-Time Clock
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x7C
	UINT8		sID;															// Secondary ID
	UINT8		spare[6];
	UINT64		UTC64;															// Linux 64-bit time_t for post-2038 date compatibility
} STRU_TELE_RTC;

//////////////////////////////////////////////////////////////////////////////
//
//						V-Speak (Placeholder)
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x61
	UINT8		sID;															// Secondary ID
	UINT8		spare[14];														// Format TBD by V-Speak
} STRU_TELE_V_SPEAK;

//////////////////////////////////////////////////////////////////////////////
//
//						www.Smoke-EL.de
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x61
	UINT8		sID;															// Secondary ID
	UINT16		batteryV;														// 0.01V, Range 0.00-70.00V
	UINT16		countdown;														// 0.01s, Range 0.00-30.00s
	INT16		GForce;															// 0.01g, Range = +/-8.00g
	UINT8		cutoff;															// 1 count, Range 0-9
	UINT8		connected;														// 0=not connected, 1=connected, x = TBD
	UINT16		spare[3];
} STRU_TELE_SMOKE_EL;

//////////////////////////////////////////////////////////////////////////////
//
//							MULTI-CYLINDER SENSOR
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = TELE_DEVICE_MULTICYLINDER
	UINT8		sID;															// Secondary ID
	UINT8		temperature[9];													// Temperature, 1C increments, Offset = 30C, 0xFF = NO DATA
				// 0x00 = 30C		(86F)
				// 0x01 = 31C ...	(88F)
				// 0xFE = 284C		(543F)
				// 0xFF = NO SENSOR ATTACHED.  Note that sensors must be installed cylinder 1-9 in sequence!
	UINT8		throttlePct;													// Throttle percent (0-100% typical, 0xFF = NO DATA)
	UINT16		RPM;															// 4 RPM increments, Offset = 400RPM, range 404-16776.
				// 0x000 = 0 RPM
				// 0x001 = 404 RPM
				// 0x002 = 408 RPM
				// 0xFFE = 16776 RPM
				// 0xFFF = NO SENSOR ATTACHED
				// NOTE:  HI NYBBLE RESERVED, set to 0xF to mark "NO DATA" for now
	UINT8		batteryV;														// Voltage, 0.1V increments, Offset = 3.5V, 0xFF = NO DATA
				// 0x00 = 3.5V
				// 0x01 = 3.6V
				// 0xFE = 28.9V
				// 0xFF = NO SENSOR ATTACHED
	UINT8		spare;															// 0xFF --> no data
} STRU_TELE_MULTI_TEMP;

//////////////////////////////////////////////////////////////////////////////
//
//						Transmitter Frame Data
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x7D
	UINT8		sID;															// Secondary ID
	UINT16		chanData[7];													// Channel Data array
} STRU_TELE_FRAMEDATA;

//////////////////////////////////////////////////////////////////////////////
//
//							AHRS Monitor
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = TELE_DEVICE_AHRS
	UINT8		sID;															// Secondary ID
	INT16		attRoll;														// Attitude, 3 axes.  Roll is a rotation about the X Axis of the vehicle using the RHR.
	INT16		attPitch;														// Units are 0.1 deg - Pitch is a rotation about the Y Axis of the vehicle using the RHR.
	INT16		attYaw;															// Roll is a rotation about the Z Axis of the vehicle using the RHR.
	INT16		altitude;														// .1m increments
	UINT8		waypoint;														// Waypoint number
	UINT8		spare8;
	UINT16		spare16[2];
} STRU_TELE_AHRS;																// AHRS data from rx

//////////////////////////////////////////////////////////////////////////////
//
//							FLIGHT MODE
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x05 TELE_DEVICE_FLITECTRL
	UINT8		sID;															// Secondary ID
	UINT8		fMode,															// Current flight mode (low nybble)
				spare8;
	UINT16		spare[6];														// Growth
	// Ideas -
	//		arming status in a bitmap
	//		time in state
} STRU_TELE_FLITECTRL;


//////////////////////////////////////////////////////////////////////////////
//
//							Crossfire QOS
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;														// Source device = TELE_XRF_LINKSTATUS
	UINT8		sID;															// Secondary ID
	UINT8		ant1,					// dBm * -1
				ant2,
				quality;				// %
	INT8		SNR;					// dB
	UINT8		activeAnt,				// ant1=0, ant2=1
				RFmode,					// 4fps=0, 50fps, 150Hz
				upPower,				// 0mW=0, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW
				downlink,				// dBm * -1
				qualityDown;			// %
	INT8		SNRdown;				// dB
} STRU_TELE_XF_QOS;

//////////////////////////////////////////////////////////////////////////////
//
//						RPM/Volts/Temperature
//
//////////////////////////////////////////////////////////////////////////////
//
//	Uses big-endian byte order
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x7E
	UINT8		sID;															// Secondary ID
	UINT16		microseconds;													// microseconds between pulse leading edges
	UINT16		volts;															// 0.01V increments (typically flight pack voltage)
	INT16		temperature;													// Temperature in degrees F.  0x7FFF = "No Data"
	INT8		dBm_A,															// Avg RSSI in dBm (<-1 = dBm, 0 = no data, >0 = % range) -- (legacy)antenna A in dBm
				dBm_B;															// Avg RSSI in % (<-1 = dBm, 0 = no data, >0 = % range)   -- (legacy)antenna B in dBm
																				// Note: Legacy use as antenna A/B dBm values is still supported. If only 1 antenna, set B = A.
																				//       The "no data" value is 0, but -1 (0xFF) is treated the same for backwards compatibility
	UINT16		spare[2];
} STRU_TELE_RPM;

//////////////////////////////////////////////////////////////////////////////
//
//							QoS DATA
//
//////////////////////////////////////////////////////////////////////////////
//
//	NOTE:  AR6410-series send:
//			id = 7F
//			sID = 0
//			A = 0
//			B = 0
//			L = 0
//			R = 0
//			F = fades
//			H = holds
//			rxV = 0xFFFF
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x7F
	UINT8		sID;															// Secondary ID
	UINT16		A;																// Internal/base receiver fades. 0xFFFF = "No data"
	UINT16		B;																// Remote receiver fades. 0xFFFF = "No data"
	UINT16		L;																// Third receiver fades. 0xFFFF = "No data"
	UINT16		R;																// Fourth receiver fades. 0xFFFF = "No data"
	UINT16		F;																// Frame losses. 0xFFFF = "No data"
	UINT16		H;																// Holds. 0xFFFF = "No data"
	UINT16		rxVoltage;														// Volts, .01V increment. 0xFFFF = "No data"
} STRU_TELE_QOS;

//////////////////////////////////////////////////////////////////////////////
//
//					UNION OF ALL DEVICE MESSAGES
//
//////////////////////////////////////////////////////////////////////////////
//
typedef union
{
	UINT16					raw[8];
	STRU_TELE_QOS			qos;
	STRU_TELE_RPM			rpm;
	STRU_TELE_HV			hv;
	STRU_TELE_TEMP			temp;
	STRU_TELE_IHIGH			amps;
	STRU_TELE_ALT			alt;
	STRU_TELE_SPEED			speed;
	STRU_TELE_ESC			escSPM;
	STRU_TELE_VARIO_S		varioSimple;
	STRU_TELE_G_METER		accel;
	STRU_TELE_JETCAT		jetcat;
	STRU_TELE_JETCAT2		jetcat2;
	STRU_TELE_GPS_LOC		gpsloc;
	STRU_TELE_GPS_STAT		gpsstat;
	STRU_TELE_GPS_BINARY	gpsbin;
	STRU_TELE_AS3X_LEGACY	as3x;
	STRU_TELE_AS6X_GAIN		as6x;
	STRU_TELE_GYRO			gyro;
	STRU_TELE_ALPHA6		alpha6;
	STRU_TELE_ATTMAG		attMag;
	STRU_TELE_POWERBOX		powerBox;
	STRU_TELE_RX_MAH		rxMAH;
	STRU_TELE_FP_MAH		fpMAH;
	STRU_TELE_ESC			esc;
	STRU_TELE_FUEL			fuel;
	STRU_TELE_DIGITAL_AIR	digAir;
	STRU_TELE_STRAIN		strain;
	STRU_TELE_LIPOMON		lipomon;
	STRU_TELE_LIPOMON_14	lipomon14;
	STRU_SMARTBATT_HEADER	smartBatt_header;
	STRU_SMARTBATT_REALTIME	smartBatt_realtime;
	STRU_SMARTBATT_CELLS	smartBatt_cells;
	STRU_SMARTBATT_ID		smartBatt_ID;
	STRU_SMARTBATT_LIMITS	smartBatt_limits;
	STRU_TELE_USER_16SU		user_16SU;
	STRU_TELE_USER_16SU32U	user_16SU32U;
	STRU_TELE_USER_16SU32S	user_16SU32S;
	STRU_TELE_USER_16U32SU	user_16U32SU;
	STRU_TELE_TEXTGEN		textgen;
	STRU_TELE_VTX			vtx;
	STRU_TELE_V_SPEAK		vSpeak;
	STRU_TELE_SMOKE_EL		smoke_el;
	STRU_TELE_MULTI_TEMP	multiCylinder;
	STRU_TELE_FLITECTRL		fControl;
	STRU_TELE_TILT			tilt;
	STRU_TELE_XF_QOS		xfire;
} UN_TELEMETRY;																	// All telemetry messages

//////////////////////////////////////////////////////////////////
//
//					sID Field Functionality
//
//////////////////////////////////////////////////////////////////
//
//		if .sID == 0x00 then .identifier = device type (TELE_DEVICE_xxx) and address I2C bus
//		if .sID != 0x00 then .sID = device type and .identifer = address on I2C bus

#endif
