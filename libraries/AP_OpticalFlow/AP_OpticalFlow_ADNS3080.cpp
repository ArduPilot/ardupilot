/*
	AP_OpticalFlow_ADNS3080.cpp - ADNS3080 OpticalFlow Library for Ardupilot Mega
	Code by Randy Mackay. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	External ADNS3080 OpticalFlow is connected via Serial port 2 (in SPI mode)
	TXD2 = MOSI = pin PH1
	RXD2 = MISO = pin PH0
	XCK2 = SCK = pin PH2
	Chip Select pin is PC4 (33)   [PH6 (9)]
	We are using the 16 clocks per conversion timming to increase efficiency (fast)
	The sampling frequency is 400Hz (Timer2 overflow interrupt)
	So if our loop is at 50Hz, our needed sampling freq should be 100Hz, so
	we have an 4x oversampling and averaging.

	Methods:
		Init() : Initialization of interrupts an Timers (Timer2 overflow interrupt)
		Read() : Read latest values from OpticalFlow and store to x,y, surface_quality parameters

*/

#include "AP_OpticalFlow_ADNS3080.h"
#include "WProgram.h"
#include "SPI.h"

#define AP_SPI_TIMEOUT 1000

union NumericIntType
{
   int         intValue;
   unsigned int uintValue;
   byte        byteValue[2];
};

	// Constructors ////////////////////////////////////////////////////////////////
AP_OpticalFlow_ADNS3080::AP_OpticalFlow_ADNS3080(int cs_pin, int reset_pin) : _cs_pin(cs_pin), _reset_pin(reset_pin)
{
	num_pixels = ADNS3080_PIXELS_X;
	field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV;
	scaler = AP_OPTICALFLOW_ADNS3080_SCALER;
}


// Public Methods //////////////////////////////////////////////////////////////
// init - initialise sensor
// initCommAPI parameter controls whether SPI interface is initialised (set to false if other devices are on the SPI bus and have already initialised the interface)
bool
AP_OpticalFlow_ADNS3080::init(bool initCommAPI)
{
    int retry = 0;

	pinMode(AP_SPI_DATAOUT,OUTPUT);
	pinMode(AP_SPI_DATAIN,INPUT);
	pinMode(AP_SPI_CLOCK,OUTPUT);
	pinMode(_cs_pin,OUTPUT);
	if( _reset_pin != 0)
	    pinMode(ADNS3080_RESET,OUTPUT);

	digitalWrite(_cs_pin,HIGH);                 // disable device (Chip select is active low)

	// reset the device
	reset();

	// start the SPI library:
	if( initCommAPI ) {
        SPI.begin();
	}

	// check the sensor is functioning
	if( retry < 3 ) {
	    if( read_register(ADNS3080_PRODUCT_ID) == 0x17 )
	        return true;
	    retry++;
	}

	return false;
}

//
// backup_spi_settings - checks current SPI settings (clock speed, etc), sets values to what we need
//
byte
AP_OpticalFlow_ADNS3080::backup_spi_settings()
{
	// store current spi values
	orig_spi_settings_spcr = SPCR & (DORD | CPOL | CPHA);
	orig_spi_settings_spsr = SPSR & SPI2X;

	// set the values that we need
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE3);
	SPI.setClockDivider(SPI_CLOCK_DIV8);  // sensor running at 2Mhz.  this is it's maximum speed

	return orig_spi_settings_spcr;
}

// restore_spi_settings - restores SPI settings (clock speed, etc) to what their values were before the sensor used the bus
byte
AP_OpticalFlow_ADNS3080::restore_spi_settings()
{
    byte temp;

	// restore SPSR
	temp = SPSR;
	temp &= ~SPI2X;
	temp |= orig_spi_settings_spsr;
	SPSR = temp;

	// restore SPCR
    temp = SPCR;
	temp &= ~(DORD | CPOL | CPHA);   // zero out the important bits
	temp |= orig_spi_settings_spcr;  // restore important bits
    SPCR = temp;

	return temp;
}

// Read a register from the sensor
byte
AP_OpticalFlow_ADNS3080::read_register(byte address)
{
    byte result = 0, junk = 0;

    backup_spi_settings();

	// take the chip select low to select the device
    digitalWrite(_cs_pin, LOW);

    // send the device the register you want to read:
    junk = SPI.transfer(address);

	// small delay
	delayMicroseconds(50);

	// send a value of 0 to read the first byte returned:
    result = SPI.transfer(0x00);

    // take the chip select high to de-select:
    digitalWrite(_cs_pin, HIGH);

	restore_spi_settings();

	return result;
}

// write a value to one of the sensor's registers
void
AP_OpticalFlow_ADNS3080::write_register(byte address, byte value)
{
    byte junk = 0;

    backup_spi_settings();

	// take the chip select low to select the device
    digitalWrite(_cs_pin, LOW);

	// send register address
    junk = SPI.transfer(address | 0x80 );

	// small delay
	delayMicroseconds(50);

	// send data
	junk = SPI.transfer(value);

    // take the chip select high to de-select:
    digitalWrite(_cs_pin, HIGH);

	restore_spi_settings();
}

// reset sensor by holding a pin high (or is it low?) for 10us.
void
AP_OpticalFlow_ADNS3080::reset()
{
    // return immediately if the reset pin is not defined
    if( _reset_pin == 0)
	    return;

    digitalWrite(_reset_pin,HIGH);                 // reset sensor
	delayMicroseconds(10);
	digitalWrite(_reset_pin,LOW);                  // return sensor to normal
}

// read latest values from sensor and fill in x,y and totals
bool
AP_OpticalFlow_ADNS3080::update()
{
    surface_quality = (unsigned int)read_register(ADNS3080_SQUAL);
	delayMicroseconds(50);  // small delay

    // check for movement, update x,y values
	if( (read_register(ADNS3080_MOTION) & 0x80) != 0 ) {
		raw_dx = ((char)read_register(ADNS3080_DELTA_X));
		delayMicroseconds(50);  // small delay
		raw_dy = ((char)read_register(ADNS3080_DELTA_Y));
		_motion = true;
	}else{
	    raw_dx = 0;
		raw_dy = 0;
	}

	last_update = millis();

	apply_orientation_matrix();
	
	return true;
}

void
AP_OpticalFlow_ADNS3080::disable_serial_pullup()
{
    byte regVal = read_register(ADNS3080_EXTENDED_CONFIG);
	regVal = (regVal | ADNS3080_SERIALNPU_OFF);
	delayMicroseconds(50);  // small delay
	write_register(ADNS3080_EXTENDED_CONFIG, regVal);
}

// get_led_always_on - returns true if LED is always on, false if only on when required
bool
AP_OpticalFlow_ADNS3080::get_led_always_on()
{
	return ( (read_register(ADNS3080_CONFIGURATION_BITS) & 0x40) > 0 );
}

// set_led_always_on - set parameter to true if you want LED always on, otherwise false for only when required
void
AP_OpticalFlow_ADNS3080::set_led_always_on( bool alwaysOn )
{
    byte regVal = read_register(ADNS3080_CONFIGURATION_BITS);
    regVal = (regVal & 0xbf) | (alwaysOn << 6);
	delayMicroseconds(50);  // small delay
	write_register(ADNS3080_CONFIGURATION_BITS, regVal);
}

// returns resolution (either 400 or 1200 counts per inch)
int
AP_OpticalFlow_ADNS3080::get_resolution()
{
    if( (read_register(ADNS3080_CONFIGURATION_BITS) & 0x10) == 0 )
	    return 400;
	else
	    return 1200;
}

// set parameter to 400 or 1200 counts per inch
void
AP_OpticalFlow_ADNS3080::set_resolution(int resolution)
{
    byte regVal = read_register(ADNS3080_CONFIGURATION_BITS);

    if( resolution == ADNS3080_RESOLUTION_400 )	{
	    regVal &= ~0x10;
	}else if( resolution == ADNS3080_RESOLUTION_1200) {
	    regVal |= 0x10;
	}

	delayMicroseconds(50);  // small delay
	write_register(ADNS3080_CONFIGURATION_BITS, regVal);
}

// get_frame_rate_auto - return whether frame rate is set to "auto" or manual
bool
AP_OpticalFlow_ADNS3080::get_frame_rate_auto()
{
    byte regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    if( (regVal & 0x01) != 0 ) {
	    return false;
	}else{
	    return true;
	}
}

// set_frame_rate_auto - set frame rate to auto (true) or manual (false)
void
AP_OpticalFlow_ADNS3080::set_frame_rate_auto(bool auto_frame_rate)
{
    byte regVal = read_register(ADNS3080_EXTENDED_CONFIG);
	delayMicroseconds(50);  // small delay
	if( auto_frame_rate == true ) {
		// set specific frame period
		write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0xE0);
		delayMicroseconds(50);  // small delay
		write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x1A);
		delayMicroseconds(50);  // small delay

		// decide what value to update in extended config
        regVal = (regVal & ~0x01);
	}else{
	    // decide what value to update in extended config
	    regVal = (regVal & ~0x01) | 0x01;
	}
	write_register(ADNS3080_EXTENDED_CONFIG, regVal);
}

// get frame period
unsigned int
AP_OpticalFlow_ADNS3080::get_frame_period()
{
    NumericIntType aNum;
	aNum.byteValue[1] = read_register(ADNS3080_FRAME_PERIOD_UPPER);
	delayMicroseconds(50);  // small delay
    aNum.byteValue[0] = read_register(ADNS3080_FRAME_PERIOD_LOWER);
	return aNum.uintValue;
}

// set frame period
void
AP_OpticalFlow_ADNS3080::set_frame_period(unsigned int period)
{
    NumericIntType aNum;
	aNum.uintValue = period;

	// set frame rate to manual
	set_frame_rate_auto(false);
	delayMicroseconds(50);  // small delay

	// set specific frame period
	write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,aNum.byteValue[0]);
	delayMicroseconds(50);  // small delay
	write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,aNum.byteValue[1]);

}

unsigned int
AP_OpticalFlow_ADNS3080::get_frame_rate()
{
    unsigned long clockSpeed = ADNS3080_CLOCK_SPEED;
	unsigned int rate = clockSpeed / get_frame_period();
	return rate;
}

void
AP_OpticalFlow_ADNS3080::set_frame_rate(unsigned int rate)
{
    unsigned long clockSpeed = ADNS3080_CLOCK_SPEED;
    unsigned int period = (unsigned int)(clockSpeed / (unsigned long)rate);

	set_frame_period(period);
}

// get_shutter_speed_auto - returns true if shutter speed is adjusted automatically, false if manual
bool
AP_OpticalFlow_ADNS3080::get_shutter_speed_auto()
{
    byte regVal = read_register(ADNS3080_EXTENDED_CONFIG);
	if( (regVal & 0x02) > 0 ) {
	    return false;
	}else{
	    return true;
	}
}

// set_shutter_speed_auto - set shutter speed to auto (true), or manual (false)
void
AP_OpticalFlow_ADNS3080::set_shutter_speed_auto(bool auto_shutter_speed)
{
    byte regVal = read_register(ADNS3080_EXTENDED_CONFIG);
	delayMicroseconds(50);  // small delay
	if( auto_shutter_speed ) {
		// return shutter speed max to default
		write_register(ADNS3080_SHUTTER_MAX_BOUND_LOWER,0x8c);
		delayMicroseconds(50);  // small delay
		write_register(ADNS3080_SHUTTER_MAX_BOUND_UPPER,0x20);
		delayMicroseconds(50);  // small delay

		// determine value to put into extended config
        regVal &= ~0x02;
	}else{
	    // determine value to put into extended config
	    regVal |= 0x02;
	}
	write_register(ADNS3080_EXTENDED_CONFIG, regVal);
	delayMicroseconds(50);  // small delay
}

// get_shutter_speed_auto - returns true if shutter speed is adjusted automatically, false if manual
unsigned int
AP_OpticalFlow_ADNS3080::get_shutter_speed()
{
    NumericIntType aNum;
	aNum.byteValue[1] = read_register(ADNS3080_SHUTTER_UPPER);
	delayMicroseconds(50);  // small delay
    aNum.byteValue[0] = read_register(ADNS3080_SHUTTER_LOWER);
	return aNum.uintValue;
}


// set_shutter_speed_auto - set shutter speed to auto (true), or manual (false)
void
AP_OpticalFlow_ADNS3080::set_shutter_speed(unsigned int shutter_speed)
{
    NumericIntType aNum;
	aNum.uintValue = shutter_speed;

	// set shutter speed to manual
    set_shutter_speed_auto(false);
	delayMicroseconds(50);  // small delay

	// set specific shutter speed
	write_register(ADNS3080_SHUTTER_MAX_BOUND_LOWER,aNum.byteValue[0]);
	delayMicroseconds(50);  // small delay
	write_register(ADNS3080_SHUTTER_MAX_BOUND_UPPER,aNum.byteValue[1]);
	delayMicroseconds(50);  // small delay

	// larger delay
	delay(50);

	// need to update frame period to cause shutter value to take effect
	aNum.byteValue[1] = read_register(ADNS3080_FRAME_PERIOD_UPPER);
	delayMicroseconds(50);  // small delay
    aNum.byteValue[0] = read_register(ADNS3080_FRAME_PERIOD_LOWER);
	delayMicroseconds(50);  // small delay
	write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,aNum.byteValue[0]);
	delayMicroseconds(50);  // small delay
	write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,aNum.byteValue[1]);
	delayMicroseconds(50);  // small delay
}

// clear_motion - will cause the Delta_X, Delta_Y, and internal motion registers to be cleared
void
AP_OpticalFlow_ADNS3080::clear_motion()
{
    write_register(ADNS3080_MOTION_CLEAR,0xFF);  // writing anything to this register will clear the sensor's motion registers
	x = 0;
	y = 0;
	dx = 0;
	dy = 0;
	_motion = false;
}

// get_pixel_data - captures an image from the sensor and stores it to the pixe_data array
void
AP_OpticalFlow_ADNS3080::print_pixel_data(Stream *serPort)
{
    int i,j;
	bool isFirstPixel = true;
	byte regValue;
	byte pixelValue;

    // write to frame capture register to force capture of frame
	write_register(ADNS3080_FRAME_CAPTURE,0x83);

	// wait 3 frame periods + 10 nanoseconds for frame to be captured
	delayMicroseconds(1510);  // min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.  so 500 x 3 + 10 = 1510

    // display the pixel data
	for( i=0; i<ADNS3080_PIXELS_Y; i++ ) {
	    for( j=0; j<ADNS3080_PIXELS_X; j++ ) {
		    regValue = read_register(ADNS3080_FRAME_CAPTURE);
		    if( isFirstPixel && (regValue & 0x40) == 0 ) {
			    serPort->println("failed to find first pixel");
			}
			isFirstPixel = false;
			pixelValue = ( regValue << 2);
			serPort->print(pixelValue,DEC);
			if( j!= ADNS3080_PIXELS_X-1 )
				serPort->print(",");
			delayMicroseconds(50);
		}
		serPort->println();
	}

	// hardware reset to restore sensor to normal operation
	reset();
}
