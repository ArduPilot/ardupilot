/*
	APM_MS5611.cpp - Arduino Library for MS5611-01BA01 absolute pressure sensor
	Code by Jose Julio, Pat Hickey and Jordi Muñoz. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Sensor is conected to standard SPI port
	Chip Select pin: Analog2 (provisional until Jordi defines the pin)!!

	Variables:
		Temp : Calculated temperature (in Celsius degrees * 100)
		Press : Calculated pressure   (in mbar units * 100)


	Methods:
		init() : Initialization and sensor reset
		read() : Read sensor data and calculate Temperature, Pressure and Altitude
		         This function is optimized so the main host don´t need to wait
				 You can call this function in your main loop
				 Maximun data output frequency 100Hz
				 It returns a 1 if there are new data.
		get_pressure() : return pressure in mbar*100 units
		get_temperature() : return temperature in celsius degrees*100 units
		get_altitude() : return altitude in meters

	Internal functions:
		calculate() : Calculate Temperature and Pressure (temperature compensated) in real units


*/

#include <SPI.h>
#include "AP_Baro_MS5611.h"


/* on APM v.24 MS5661_CS is PG1 (Arduino pin 40) */
#define MS5611_CS 40

#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximun resolution
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximun resolution


uint8_t MS5611_SPI_read(byte reg)
{
  byte dump;
  uint8_t return_value;
  byte addr = reg; // | 0x80; // Set most significant bit
  digitalWrite(MS5611_CS, LOW);
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);
  digitalWrite(MS5611_CS, HIGH);
  return(return_value);
}

uint16_t MS5611_SPI_read_16bits(byte reg)
{
  byte dump,byteH,byteL;
  uint16_t return_value;
  byte addr = reg; // | 0x80; // Set most significant bit
  digitalWrite(MS5611_CS, LOW);
  dump = SPI.transfer(addr);
  byteH = SPI.transfer(0);
  byteL = SPI.transfer(0);
  digitalWrite(MS5611_CS, HIGH);
  return_value = ((uint16_t)byteH<<8) | (byteL);
  return(return_value);
}

uint32_t MS5611_SPI_read_ADC()
{
  byte dump,byteH,byteM,byteL;
  uint32_t return_value;
  byte addr = 0x00;
  digitalWrite(MS5611_CS, LOW);
  dump = SPI.transfer(addr);
  byteH = SPI.transfer(0);
  byteM = SPI.transfer(0);
  byteL = SPI.transfer(0);
  digitalWrite(MS5611_CS, HIGH);
  return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
  return(return_value);
}


void MS5611_SPI_write(byte reg)
{
  byte dump;
  digitalWrite(MS5611_CS, LOW);
  dump = SPI.transfer(reg);
  digitalWrite(MS5611_CS, HIGH);
}

// The conversion proccess takes 8.2ms since the command
uint8_t AP_Baro_MS5611::MS5611_Ready()
{
	if ((millis()-MS5611_timer)>10)   // wait for more than 10ms
		return(1);
	else
		return(0);
}

// Public Methods //////////////////////////////////////////////////////////////
// SPI should be initialized externally
void AP_Baro_MS5611::init()
{

	pinMode(MS5611_CS, OUTPUT);	 // Chip select Pin

	MS5611_SPI_write(CMD_MS5611_RESET);
	delay(4);

	// We read the factory calibration
	C1 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C1);
	C2 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C2);
	C3 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C3);
	C4 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C4);
	C5 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C5);
	C6 = MS5611_SPI_read_16bits(CMD_MS5611_PROM_C6);


  //Send a command to read Temp first
	MS5611_SPI_write(CMD_CONVERT_D2_OSR4096);
	MS5611_timer = millis();
	MS5611_State = 1;
    Temp=0;
    Press=0;
}


// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
// temperature does not change so quickly...
uint8_t AP_Baro_MS5611::read()
{
	uint8_t result = 0;

	if (MS5611_State == 1){
		if (MS5611_Ready()){
			D2=MS5611_SPI_read_ADC();  				 // On state 1 we read temp
            _raw_temp = D2;
			MS5611_State++;
			MS5611_SPI_write(CMD_CONVERT_D1_OSR4096);  // Command to read pressure
			MS5611_timer = millis();
		}
	}else{
		if (MS5611_State == 5){
			if (MS5611_Ready()){
				D1=MS5611_SPI_read_ADC();
                _raw_press = D1;
				calculate();
				MS5611_State = 1;			                // Start again from state = 1
				MS5611_SPI_write(CMD_CONVERT_D2_OSR4096);	// Command to read temperature
				MS5611_timer = millis();
				result = 1;					                // New pressure reading
			}
		}else{
			if (MS5611_Ready()){
				D1=MS5611_SPI_read_ADC();
                _raw_press = D1;
				calculate();
				MS5611_State++;
				MS5611_SPI_write(CMD_CONVERT_D1_OSR4096);  // Command to read pressure
				MS5611_timer = millis();
				result = 1;					               // New pressure reading
			}
		}
	}
	return(result);
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS5611::calculate()
{
	int32_t dT;
	long long TEMP;  // 64 bits
	long long OFF;
	long long SENS;
	long long P;

	// Formulas from manufacturer datasheet
	// TODO: optimization with shift operations... (shift operations works well on 64 bits variables?)
	// We define parameters as 64 bits to prevent overflow on operations
	dT = D2-((long)C5*256);
	TEMP = 2000 + ((long long)dT * C6)/8388608;
	OFF = (long long)C2 * 65536 + ((long long)C4 * dT ) / 128; 
	SENS = (long long)C1 * 32768 + ((long long)C3 * dT) / 256;

	/*
	if (TEMP < 2000){   // second order temperature compensation
		long long T2 = (long long)dT*dT / 2147483648;
		long long Aux_64 = (TEMP-2000)*(TEMP-2000);
		long long OFF2 = 5*Aux_64/2;
		long long SENS2 = 5*Aux_64/4;
		TEMP = TEMP - T2;
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
	}
	*/
	P = (D1*SENS/2097152 - OFF)/32768;
	Temp = TEMP;
	Press = P;
}

int32_t AP_Baro_MS5611::get_pressure()
{
	return(Press);
}

int16_t AP_Baro_MS5611::get_temperature()
{
	return(Temp);
}

// Return altitude using the standard 1013.25 mbar at sea level reference
float AP_Baro_MS5611::get_altitude()
{
	float tmp_float;
	float Altitude;

	tmp_float = (Press / 101325.0);
	tmp_float = pow(tmp_float, 0.190295);
	Altitude = 44330 * (1.0 - tmp_float);

	return (Altitude);
}

int32_t AP_Baro_MS5611::get_raw_pressure() {
	return _raw_press;
}

int32_t AP_Baro_MS5611::get_raw_temp() {
	return _raw_temp;
}


