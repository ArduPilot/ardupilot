/*
 *       AP_OpticalFlow_ADNS3080.cpp - ADNS3080 OpticalFlow Library for Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       External ADNS3080 OpticalFlow is connected via Serial port 2 (in SPI mode)
 *       TXD2 = MOSI = pin PH1
 *       RXD2 = MISO = pin PH0
 *       XCK2 = SCK = pin PH2
 *       Chip Select pin is PC4 (33)   [PH6 (9)]
 *       We are using the 16 clocks per conversion timming to increase efficiency (fast)
 *       The sampling frequency is 400Hz (Timer2 overflow interrupt)
 *       So if our loop is at 50Hz, our needed sampling freq should be 100Hz, so
 *       we have an 4x oversampling and averaging.
 *
 *       Methods:
 *               Init() : Initialization of interrupts an Timers (Timer2 overflow interrupt)
 *               Read() : Read latest values from OpticalFlow and store to x,y, surface_quality parameters
 *
 */

#include "AP_OpticalFlow_ADNS3080.h"
#include "SPI.h"
#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define AP_SPI_TIMEOUT 1000

// We use Serial Port 2 in SPI Mode
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
 #define AP_SPI_DATAIN      15           // MISO
 #define AP_SPI_DATAOUT     14           // MOSI
 #define AP_SPI_CLOCK       PJ2          // SCK
#else  // normal arduino SPI pins...these need to be checked
 # error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#endif

// mask for saving bit order and data mode to avoid interference with other users of the bus
#define UCSR3C_MASK 0x07

// SPI3 setting for UCSR3C
#define SPI3_MODE_SPI 0xC0      // UMSEL31 = 1, UMSEL30 = 1

// settings for phase and polarity bits of UCSR3C
#define SPI3_MODE_MASK 0x03
#define SPI3_MODE0 0x00
#define SPI3_MODE1 0x01
#define SPI3_MODE2 0x02
#define SPI3_MODE3 0x03
#define SPI3_MODE SPI3_MODE3

// settings for phase and polarity bits of UCSR3C
#define SPI3_ORDER_MASK 0x04
#define SPI3_MSBFIRST 0x00
#define SPI3_LSBFIRST 0x04

#define SPI3_SPEED 0x04 // 2 megahertz?

#define SPI3_DELAY 20   // delay in microseconds after sending data


union NumericIntType
{
    int intValue;
    unsigned int uintValue;
    byte byteValue[2];
};

// Constructors ////////////////////////////////////////////////////////////////
AP_OpticalFlow_ADNS3080_APM2::AP_OpticalFlow_ADNS3080_APM2(int cs_pin, int reset_pin) : _cs_pin(cs_pin), _reset_pin(reset_pin)
{
    num_pixels = ADNS3080_PIXELS_X;
    field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV;
    scaler = AP_OPTICALFLOW_ADNS3080_SCALER;
}

// SPI Methods
// *** INTERNAL FUNCTIONS ***
unsigned char AP_OpticalFlow_ADNS3080_APM2::SPI_transfer(uint8_t data)
{

    /* Wait for empty transmit buffer */
    while ( !( UCSR3A & (1<<UDRE3)) ) ;

    /* Put data into buffer, sends the data */
    UDR3 = data;

    /* Wait for data to be received */
    while ( !(UCSR3A & (1<<RXC3)) ) ;

    /* Get and return received data from buffer */
    return UDR3;
}

// Public Methods //////////////////////////////////////////////////////////////
// init - initialise sensor
// initCommAPI parameter controls whether SPI interface is initialised (set to false if other devices are on the SPI bus and have already initialised the interface)
bool
AP_OpticalFlow_ADNS3080_APM2::init(bool initCommAPI)
{
    int retry = 0;

    pinMode(AP_SPI_DATAOUT,OUTPUT);
    pinMode(AP_SPI_DATAIN,INPUT);
    pinMode(AP_SPI_CLOCK,OUTPUT);
    pinMode(_cs_pin,OUTPUT);
    if( _reset_pin != 0)
        pinMode(ADNS3080_RESET,OUTPUT);

    digitalWrite(_cs_pin,HIGH);                     // disable device (Chip select is active low)

    // reset the device
    reset();

    // start the SPI library:
    if( initCommAPI ) {
        // Setup Serial Port3 in SPI mode (MSPI), Mode 0, Clock: 8Mhz
        UBRR3 = 0;
        DDRJ |= (1<<PJ2);                                           // SPI clock XCK3 (PJ2) as output. This enable SPI Master mode
        // put UART3 into SPI master mode
        UCSR3C = SPI3_MODE_SPI | SPI3_MODE;
        // Enable receiver and transmitter.
        UCSR3B = (1<<RXEN3)|(1<<TXEN3);
        // Set Baud rate
        UBRR3 = SPI3_SPEED;                 // SPI running at 8Mhz
    }

    delay(10);

    // check the sensor is functioning
    while( retry < 3 ) {
        if( read_register(ADNS3080_PRODUCT_ID) == 0x17 ) {
            return true;
        }
        retry++;
    }

    return false;
}

//
// backup_spi_settings - checks current SPI settings (clock speed, etc), sets values to what we need
//
void AP_OpticalFlow_ADNS3080_APM2::backup_spi_settings()
{

    uint8_t temp;

    /* Wait for empty transmit buffer */
    while ( !( UCSR3A & (1<<UDRE3)) ) ;

    // store current spi values
    orig_spi_settings_ucsr3c = UCSR3C;
    orig_spi_settings_ubrr3 = UBRR3;

    // decide new value for UCSR3C
    temp = (orig_spi_settings_ucsr3c & ~UCSR3C_MASK) | SPI3_MODE | SPI3_MSBFIRST;
    UCSR3C = temp;
    UBRR3 = SPI3_SPEED;             // SPI running at 1Mhz
}

// restore_spi_settings - restores SPI settings (clock speed, etc) to what their values were before the sensor used the bus
void AP_OpticalFlow_ADNS3080_APM2::restore_spi_settings()
{
    /* Wait for empty transmit buffer */
    while ( !( UCSR3A & (1<<UDRE3)) ) ;

    // restore UCSRC3C and UBRR3
    UCSR3C = orig_spi_settings_ucsr3c;
    UBRR3 = orig_spi_settings_ubrr3;
}

// Read a register from the sensor
byte
AP_OpticalFlow_ADNS3080_APM2::read_register(byte address)
{
    byte result = 0, junk = 0;

    backup_spi_settings();

    // take the chip select low to select the device
    digitalWrite(_cs_pin, LOW);

    // send the device the register you want to read:
    junk = SPI_transfer(address);

    // small delay
    delayMicroseconds(SPI3_DELAY);

    // send a value of 0 to read the first byte returned:
    result = SPI_transfer(0x00);

    // take the chip select high to de-select:
    digitalWrite(_cs_pin, HIGH);

    restore_spi_settings();

    return result;
}

// write a value to one of the sensor's registers
void
AP_OpticalFlow_ADNS3080_APM2::write_register(byte address, byte value)
{
    byte junk = 0;

    backup_spi_settings();

    // take the chip select low to select the device
    digitalWrite(_cs_pin, LOW);

    // send register address
    junk = SPI_transfer(address | 0x80 );

    // small delay
    delayMicroseconds(SPI3_DELAY);

    // send data
    junk = SPI_transfer(value);

    // take the chip select high to de-select:
    digitalWrite(_cs_pin, HIGH);

    restore_spi_settings();
}

// reset sensor by holding a pin high (or is it low?) for 10us.
void
AP_OpticalFlow_ADNS3080_APM2::reset()
{
    // return immediately if the reset pin is not defined
    if( _reset_pin == 0)
        return;

    digitalWrite(_reset_pin,HIGH);                 // reset sensor
    delayMicroseconds(10);
    digitalWrite(_reset_pin,LOW);                      // return sensor to normal
}

// read latest values from sensor and fill in x,y and totals
bool
AP_OpticalFlow_ADNS3080_APM2::update()
{
    byte motion_reg;
    surface_quality = (unsigned int)read_register(ADNS3080_SQUAL);
    delayMicroseconds(SPI3_DELAY);      // small delay

    // check for movement, update x,y values
    motion_reg = read_register(ADNS3080_MOTION);
    _overflow = ((motion_reg & 0x10) != 0);      // check if we've had an overflow
    if( (motion_reg & 0x80) != 0 ) {
        raw_dx = ((char)read_register(ADNS3080_DELTA_X));
        delayMicroseconds(SPI3_DELAY);          // small delay
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
AP_OpticalFlow_ADNS3080_APM2::disable_serial_pullup()
{
    byte regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    regVal = (regVal | ADNS3080_SERIALNPU_OFF);
    delayMicroseconds(SPI3_DELAY);      // small delay
    write_register(ADNS3080_EXTENDED_CONFIG, regVal);
}

// get_led_always_on - returns true if LED is always on, false if only on when required
bool
AP_OpticalFlow_ADNS3080_APM2::get_led_always_on()
{
    return ( (read_register(ADNS3080_CONFIGURATION_BITS) & 0x40) > 0 );
}

// set_led_always_on - set parameter to true if you want LED always on, otherwise false for only when required
void
AP_OpticalFlow_ADNS3080_APM2::set_led_always_on( bool alwaysOn )
{
    byte regVal = read_register(ADNS3080_CONFIGURATION_BITS);
    regVal = (regVal & 0xbf) | (alwaysOn << 6);
    delayMicroseconds(SPI3_DELAY);      // small delay
    write_register(ADNS3080_CONFIGURATION_BITS, regVal);
}

// returns resolution (either 400 or 1600 counts per inch)
int
AP_OpticalFlow_ADNS3080_APM2::get_resolution()
{
    if( (read_register(ADNS3080_CONFIGURATION_BITS) & 0x10) == 0 )
        return 400;
    else
        return 1600;
}

// set parameter to 400 or 1600 counts per inch
void
AP_OpticalFlow_ADNS3080_APM2::set_resolution(int resolution)
{
    byte regVal = read_register(ADNS3080_CONFIGURATION_BITS);

    if( resolution == ADNS3080_RESOLUTION_400 ) {
        regVal &= ~0x10;
        scaler = AP_OPTICALFLOW_ADNS3080_SCALER;
    }else if( resolution == ADNS3080_RESOLUTION_1600) {
        regVal |= 0x10;
        scaler = AP_OPTICALFLOW_ADNS3080_SCALER * 4;
    }

    delayMicroseconds(SPI3_DELAY);      // small delay
    write_register(ADNS3080_CONFIGURATION_BITS, regVal);

    // this will affect conversion factors so update them
    update_conversion_factors();
}

// get_frame_rate_auto - return whether frame rate is set to "auto" or manual
bool
AP_OpticalFlow_ADNS3080_APM2::get_frame_rate_auto()
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
AP_OpticalFlow_ADNS3080_APM2::set_frame_rate_auto(bool auto_frame_rate)
{
    byte regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    delayMicroseconds(SPI3_DELAY);      // small delay
    if( auto_frame_rate == true ) {
        // set specific frame period
        write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0xE0);
        delayMicroseconds(SPI3_DELAY);          // small delay
        write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x1A);
        delayMicroseconds(SPI3_DELAY);          // small delay

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
AP_OpticalFlow_ADNS3080_APM2::get_frame_period()
{
    NumericIntType aNum;
    aNum.byteValue[1] = read_register(ADNS3080_FRAME_PERIOD_UPPER);
    delayMicroseconds(SPI3_DELAY);      // small delay
    aNum.byteValue[0] = read_register(ADNS3080_FRAME_PERIOD_LOWER);
    return aNum.uintValue;
}

// set frame period
void
AP_OpticalFlow_ADNS3080_APM2::set_frame_period(unsigned int period)
{
    NumericIntType aNum;
    aNum.uintValue = period;

    // set frame rate to manual
    set_frame_rate_auto(false);
    delayMicroseconds(SPI3_DELAY);      // small delay

    // set specific frame period
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,aNum.byteValue[0]);
    delayMicroseconds(SPI3_DELAY);      // small delay
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,aNum.byteValue[1]);

}

unsigned int
AP_OpticalFlow_ADNS3080_APM2::get_frame_rate()
{
    unsigned long clockSpeed = ADNS3080_CLOCK_SPEED;
    unsigned int rate = clockSpeed / get_frame_period();
    return rate;
}

void
AP_OpticalFlow_ADNS3080_APM2::set_frame_rate(unsigned int rate)
{
    unsigned long clockSpeed = ADNS3080_CLOCK_SPEED;
    unsigned int period = (unsigned int)(clockSpeed / (unsigned long)rate);

    set_frame_period(period);
}

// get_shutter_speed_auto - returns true if shutter speed is adjusted automatically, false if manual
bool
AP_OpticalFlow_ADNS3080_APM2::get_shutter_speed_auto()
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
AP_OpticalFlow_ADNS3080_APM2::set_shutter_speed_auto(bool auto_shutter_speed)
{
    byte regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    delayMicroseconds(SPI3_DELAY);      // small delay
    if( auto_shutter_speed ) {
        // return shutter speed max to default
        write_register(ADNS3080_SHUTTER_MAX_BOUND_LOWER,0x8c);
        delayMicroseconds(SPI3_DELAY);          // small delay
        write_register(ADNS3080_SHUTTER_MAX_BOUND_UPPER,0x20);
        delayMicroseconds(SPI3_DELAY);          // small delay

        // determine value to put into extended config
        regVal &= ~0x02;
    }else{
        // determine value to put into extended config
        regVal |= 0x02;
    }
    write_register(ADNS3080_EXTENDED_CONFIG, regVal);
    delayMicroseconds(SPI3_DELAY);      // small delay
}

// get_shutter_speed_auto - returns true if shutter speed is adjusted automatically, false if manual
unsigned int
AP_OpticalFlow_ADNS3080_APM2::get_shutter_speed()
{
    NumericIntType aNum;
    aNum.byteValue[1] = read_register(ADNS3080_SHUTTER_UPPER);
    delayMicroseconds(SPI3_DELAY);      // small delay
    aNum.byteValue[0] = read_register(ADNS3080_SHUTTER_LOWER);
    return aNum.uintValue;
}


// set_shutter_speed_auto - set shutter speed to auto (true), or manual (false)
void
AP_OpticalFlow_ADNS3080_APM2::set_shutter_speed(unsigned int shutter_speed)
{
    NumericIntType aNum;
    aNum.uintValue = shutter_speed;

    // set shutter speed to manual
    set_shutter_speed_auto(false);
    delayMicroseconds(SPI3_DELAY);      // small delay

    // set specific shutter speed
    write_register(ADNS3080_SHUTTER_MAX_BOUND_LOWER,aNum.byteValue[0]);
    delayMicroseconds(SPI3_DELAY);      // small delay
    write_register(ADNS3080_SHUTTER_MAX_BOUND_UPPER,aNum.byteValue[1]);
    delayMicroseconds(SPI3_DELAY);      // small delay

    // larger delay
    delay(50);

    // need to update frame period to cause shutter value to take effect
    aNum.byteValue[1] = read_register(ADNS3080_FRAME_PERIOD_UPPER);
    delayMicroseconds(SPI3_DELAY);      // small delay
    aNum.byteValue[0] = read_register(ADNS3080_FRAME_PERIOD_LOWER);
    delayMicroseconds(SPI3_DELAY);      // small delay
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,aNum.byteValue[0]);
    delayMicroseconds(SPI3_DELAY);      // small delay
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,aNum.byteValue[1]);
    delayMicroseconds(SPI3_DELAY);      // small delay
}

// clear_motion - will cause the Delta_X, Delta_Y, and internal motion registers to be cleared
void
AP_OpticalFlow_ADNS3080_APM2::clear_motion()
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
AP_OpticalFlow_ADNS3080_APM2::print_pixel_data(Stream *serPort)
{
    int i,j;
    bool isFirstPixel = true;
    byte regValue;
    byte pixelValue;

    // write to frame capture register to force capture of frame
    write_register(ADNS3080_FRAME_CAPTURE,0x83);

    // wait 3 frame periods + 10 nanoseconds for frame to be captured
    delayMicroseconds(1510);      // min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.  so 500 x 3 + 10 = 1510

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
            delayMicroseconds(SPI3_DELAY);
        }
        serPort->println();
    }

    // hardware reset to restore sensor to normal operation
    reset();
}
