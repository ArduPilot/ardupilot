/*
 *       AP_OpticalFlow_ADNS3080.cpp - ADNS3080 OpticalFlow Library for Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 */

#include "AP_OpticalFlow_ADNS3080.h"
#include "SPI.h"
#include "SPI3.h"
#include "AP_Semaphore.h"

#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define AP_SPI_TIMEOUT 1000

// We use Serial Port 2 in SPI Mode
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
 #define ADNS3080_SPI_MISO        50      // PB3
 #define ADNS3080_SPI_MOSI        51      // PB2
 #define ADNS3080_SPI_SCK         52      // PB1
#else  // normal arduino SPI pins...these need to be checked
 #define ADNS3080_SPI_MISO        12      // MISO
 #define ADNS3080_SPI_MOSI        11      // MOSI
 #define ADNS3080_SPI_SCK         13      // SCK
#endif

union NumericIntType
{
    int16_t intValue;
    uint16_t uintValue;
    uint8_t byteValue[2];
};

// Constructors ////////////////////////////////////////////////////////////////
AP_OpticalFlow_ADNS3080::AP_OpticalFlow_ADNS3080(AP_Semaphore* semaphore, int16_t cs_pin, int16_t reset_pin) :
    _semaphore(semaphore),
    _cs_pin(cs_pin),
    _reset_pin(reset_pin),
    _spi_bus(ADNS3080_SPI_UNKNOWN)
{
    num_pixels = ADNS3080_PIXELS_X;
    field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV;
    scaler = AP_OPTICALFLOW_ADNS3080_SCALER;
}

// Public Methods //////////////////////////////////////////////////////////////
// init - initialise sensor
// assumes SPI bus has been initialised but will attempt to initialise nonstandard SPI3 bus if required
bool
AP_OpticalFlow_ADNS3080::init(bool initCommAPI, AP_PeriodicProcess *scheduler)
{
    int8_t retry = 0;
    bool retvalue = false;

    // suspend timer while we set-up SPI communication
    scheduler->suspend_timer();

    pinMode(_cs_pin,OUTPUT);
    if( _reset_pin != 0)
        pinMode(ADNS3080_RESET,OUTPUT);

    digitalWrite(_cs_pin,HIGH);                     // disable device (Chip select is active low)

    // reset the device
    reset();

    // start the SPI library:
    if( initCommAPI ) {
        pinMode(ADNS3080_SPI_MOSI,OUTPUT);
        pinMode(ADNS3080_SPI_MISO,INPUT);
        pinMode(ADNS3080_SPI_SCK,OUTPUT);
        SPI.begin();
        SPI.setClockDivider(SPI_CLOCK_DIV8); // 2MHZ SPI rate
    }

    // check 3 times for the sensor on standard SPI bus
    _spi_bus = ADNS3080_SPIBUS_1;
    while( retvalue == false && retry < 3 ) {
        if( read_register(ADNS3080_PRODUCT_ID) == 0x17 ) {
            retvalue = true;
        }
        retry++;
    }

    // if not found, check 3 times on SPI3
    if( !retvalue ) {

        // start the SPI3 library:
        if( initCommAPI ) {
            SPI3.begin();
            SPI3.setDataMode(SPI3_MODE3); // Mode3
            SPI3.setSpeed(SPI3_SPEED_2MHZ); // 2MHZ SPI rate
        }

        _spi_bus = ADNS3080_SPIBUS_3;
        retry = 0;
        while( retvalue == false && retry < 3 ) {
            if( read_register(ADNS3080_PRODUCT_ID) == 0x17 ) {
                retvalue = true;
            }
            retry++;
        }
    }

    // resume timer
    scheduler->resume_timer();

    // if device is working register the global static read function to be called at 1khz
    if( retvalue ) {
        scheduler->register_process( AP_OpticalFlow_ADNS3080::read );
    }else{
        _spi_bus = ADNS3080_SPI_UNKNOWN;
    }

    return retvalue;
}

//
// backup_spi_settings - checks current SPI settings (clock speed, etc), sets values to what we need
//
void
AP_OpticalFlow_ADNS3080::backup_spi_settings()
{
    if( _spi_bus == ADNS3080_SPIBUS_1 ) {
        // store current spi mode and data rate
        orig_spi_settings_spcr = SPCR & (CPOL | CPHA | SPR1 | SPR0);

        // set to our required values
        SPI.setDataMode(SPI_MODE3);
        SPI.setClockDivider(SPI_CLOCK_DIV8); // 2MHZ SPI rate

    }else if( _spi_bus == ADNS3080_SPIBUS_3 ) {
        /* Wait for empty transmit buffer */
        while ( !( UCSR3A & (1<<UDRE3)) ) ;

        // store current spi values
        orig_spi3_settings_ucsr3c = UCSR3C;
        orig_spi3_settings_ubrr3 = UBRR3;

        // set to our required values
        SPI3.setDataMode(SPI3_MODE3);
        SPI3.setSpeed(SPI3_SPEED_2MHZ); // 2MHZ SPI rate
    }
}

// restore_spi_settings - restores SPI settings (clock speed, etc) to what their values were before the sensor used the bus
void
AP_OpticalFlow_ADNS3080::restore_spi_settings()
{
    byte temp;

    if( _spi_bus == ADNS3080_SPIBUS_1 ) {
        // split off the two bits we need to write
        temp = SPCR & ~(CPOL | CPHA | SPR1 | SPR0);
        temp |= orig_spi_settings_spcr;

        // write back the bits
        SPCR = temp;
    }else if( _spi_bus == ADNS3080_SPIBUS_3 ) {
        /* Wait for empty transmit buffer */
        while ( !( UCSR3A & (1<<UDRE3)) ) ;

        // restore UCSRC3C (spi mode) and UBBR3 (speed)
        UCSR3C = orig_spi3_settings_ucsr3c;
        UBRR3 = orig_spi3_settings_ubrr3;
    }
}

// Read a register from the sensor
byte
AP_OpticalFlow_ADNS3080::read_register(byte address)
{
    uint8_t result = 0;
    uint8_t junk = 0;

    // get spi semaphore if required
    if( _semaphore != NULL) {
        // if failed to get semaphore then just quietly fail
        if( !_semaphore->get(this) ) {
            return 0;
        }
    }

    backup_spi_settings();

    // take the chip select low to select the device
    digitalWrite(_cs_pin, LOW);

    if( _spi_bus == ADNS3080_SPIBUS_1 ) {
        junk = SPI.transfer(address);   // send the device the register you want to read:
        delayMicroseconds(50);          // small delay
        result = SPI.transfer(0x00);    // send a value of 0 to read the first byte returned:
    }else if( _spi_bus == ADNS3080_SPIBUS_3 ) {
        junk = SPI3.transfer(address);   // send the device the register you want to read:
        delayMicroseconds(50);          // small delay
        result = SPI3.transfer(0x00);    // send a value of 0 to read the first byte returned:
    }

    // take the chip select high to de-select:
    digitalWrite(_cs_pin, HIGH);

    restore_spi_settings();

    // get spi semaphore if required
    if( _semaphore != NULL) {
        _semaphore->release(this);
    }

    return result;
}

// write a value to one of the sensor's registers
void
AP_OpticalFlow_ADNS3080::write_register(byte address, byte value)
{
    byte junk = 0;

    // get spi semaphore if required
    if( _semaphore != NULL) {
        // if failed to get semaphore then just quietly fail
        if( !_semaphore->get(this) ) {
            Serial.println("Optflow: failed to get spi3 semaphore!");
            return;
        }
    }

    backup_spi_settings();

    // take the chip select low to select the device
    digitalWrite(_cs_pin, LOW);

    if( _spi_bus == ADNS3080_SPIBUS_1 ) {
        junk = SPI.transfer(address | 0x80 );   // send register address
        delayMicroseconds(50);                  // small delay
        junk = SPI.transfer(value);             // send data
    }else if( _spi_bus == ADNS3080_SPIBUS_3 ) {
        junk = SPI3.transfer(address | 0x80 );   // send register address
        delayMicroseconds(50);                  // small delay
        junk = SPI3.transfer(value);             // send data
    }

    // take the chip select high to de-select:
    digitalWrite(_cs_pin, HIGH);

    restore_spi_settings();

    // get spi3 semaphore if required
    if( _semaphore != NULL) {
        _semaphore->release(this);
    }
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
    digitalWrite(_reset_pin,LOW);                      // return sensor to normal
}

// read latest values from sensor and fill in x,y and totals
void
AP_OpticalFlow_ADNS3080::update(uint32_t now)
{
    byte motion_reg;
    surface_quality = (uint16_t)read_register(ADNS3080_SQUAL);
    delayMicroseconds(50);      // small delay

    // check for movement, update x,y values
    motion_reg = read_register(ADNS3080_MOTION);
    _overflow = ((motion_reg & 0x10) != 0);      // check if we've had an overflow
    if( (motion_reg & 0x80) != 0 ) {
        raw_dx = ((int8_t)read_register(ADNS3080_DELTA_X));
        delayMicroseconds(50);          // small delay
        raw_dy = ((int8_t)read_register(ADNS3080_DELTA_Y));
        _motion = true;
    }else{
        raw_dx = 0;
        raw_dy = 0;
    }

    last_update = millis();

    apply_orientation_matrix();
}

void
AP_OpticalFlow_ADNS3080::disable_serial_pullup()
{
    byte regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    regVal = (regVal | ADNS3080_SERIALNPU_OFF);
    delayMicroseconds(50);      // small delay
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
    delayMicroseconds(50);      // small delay
    write_register(ADNS3080_CONFIGURATION_BITS, regVal);
}

// returns resolution (either 400 or 1600 counts per inch)
int16_t
AP_OpticalFlow_ADNS3080::get_resolution()
{
    if( (read_register(ADNS3080_CONFIGURATION_BITS) & 0x10) == 0 )
        return 400;
    else
        return 1600;
}

// set parameter to 400 or 1600 counts per inch
void
AP_OpticalFlow_ADNS3080::set_resolution(uint16_t resolution)
{
    byte regVal = read_register(ADNS3080_CONFIGURATION_BITS);

    if( resolution == ADNS3080_RESOLUTION_400 ) {
        regVal &= ~0x10;
        scaler = AP_OPTICALFLOW_ADNS3080_SCALER;
    }else if( resolution == ADNS3080_RESOLUTION_1600) {
        regVal |= 0x10;
        scaler = AP_OPTICALFLOW_ADNS3080_SCALER * 4;
    }

    delayMicroseconds(50);      // small delay
    write_register(ADNS3080_CONFIGURATION_BITS, regVal);

    // this will affect conversion factors so update them
    update_conversion_factors();
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
    delayMicroseconds(50);      // small delay
    if( auto_frame_rate == true ) {
        // set specific frame period
        write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0xE0);
        delayMicroseconds(50);          // small delay
        write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x1A);
        delayMicroseconds(50);          // small delay

        // decide what value to update in extended config
        regVal = (regVal & ~0x01);
    }else{
        // decide what value to update in extended config
        regVal = (regVal & ~0x01) | 0x01;
    }
    write_register(ADNS3080_EXTENDED_CONFIG, regVal);
}

// get frame period
uint16_t
AP_OpticalFlow_ADNS3080::get_frame_period()
{
    NumericIntType aNum;
    aNum.byteValue[1] = read_register(ADNS3080_FRAME_PERIOD_UPPER);
    delayMicroseconds(50);      // small delay
    aNum.byteValue[0] = read_register(ADNS3080_FRAME_PERIOD_LOWER);
    return aNum.uintValue;
}

// set frame period
void
AP_OpticalFlow_ADNS3080::set_frame_period(uint16_t period)
{
    NumericIntType aNum;
    aNum.uintValue = period;

    // set frame rate to manual
    set_frame_rate_auto(false);
    delayMicroseconds(50);      // small delay

    // set specific frame period
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,aNum.byteValue[0]);
    delayMicroseconds(50);      // small delay
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,aNum.byteValue[1]);

}

uint16_t
AP_OpticalFlow_ADNS3080::get_frame_rate()
{
    uint32_t clockSpeed = ADNS3080_CLOCK_SPEED;
    uint16_t rate = clockSpeed / get_frame_period();
    return rate;
}

void
AP_OpticalFlow_ADNS3080::set_frame_rate(uint16_t rate)
{
    uint32_t clockSpeed = ADNS3080_CLOCK_SPEED;
    uint16_t period = (uint16_t)(clockSpeed / (uint32_t)rate);

    set_frame_period(period);
}

// get_shutter_speed_auto - returns true if shutter speed is adjusted automatically, false if manual
bool
AP_OpticalFlow_ADNS3080::get_shutter_speed_auto()
{
    uint8_t regVal = read_register(ADNS3080_EXTENDED_CONFIG);
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
    uint8_t regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    delayMicroseconds(50);      // small delay
    if( auto_shutter_speed ) {
        // return shutter speed max to default
        write_register(ADNS3080_SHUTTER_MAX_BOUND_LOWER,0x8c);
        delayMicroseconds(50);          // small delay
        write_register(ADNS3080_SHUTTER_MAX_BOUND_UPPER,0x20);
        delayMicroseconds(50);          // small delay

        // determine value to put into extended config
        regVal &= ~0x02;
    }else{
        // determine value to put into extended config
        regVal |= 0x02;
    }
    write_register(ADNS3080_EXTENDED_CONFIG, regVal);
    delayMicroseconds(50);      // small delay
}

// get_shutter_speed_auto - returns true if shutter speed is adjusted automatically, false if manual
uint16_t
AP_OpticalFlow_ADNS3080::get_shutter_speed()
{
    NumericIntType aNum;
    aNum.byteValue[1] = read_register(ADNS3080_SHUTTER_UPPER);
    delayMicroseconds(50);      // small delay
    aNum.byteValue[0] = read_register(ADNS3080_SHUTTER_LOWER);
    return aNum.uintValue;
}


// set_shutter_speed_auto - set shutter speed to auto (true), or manual (false)
void
AP_OpticalFlow_ADNS3080::set_shutter_speed(uint16_t shutter_speed)
{
    NumericIntType aNum;
    aNum.uintValue = shutter_speed;

    // set shutter speed to manual
    set_shutter_speed_auto(false);
    delayMicroseconds(50);      // small delay

    // set specific shutter speed
    write_register(ADNS3080_SHUTTER_MAX_BOUND_LOWER,aNum.byteValue[0]);
    delayMicroseconds(50);      // small delay
    write_register(ADNS3080_SHUTTER_MAX_BOUND_UPPER,aNum.byteValue[1]);
    delayMicroseconds(50);      // small delay

    // larger delay
    delay(50);

    // need to update frame period to cause shutter value to take effect
    aNum.byteValue[1] = read_register(ADNS3080_FRAME_PERIOD_UPPER);
    delayMicroseconds(50);      // small delay
    aNum.byteValue[0] = read_register(ADNS3080_FRAME_PERIOD_LOWER);
    delayMicroseconds(50);      // small delay
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,aNum.byteValue[0]);
    delayMicroseconds(50);      // small delay
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,aNum.byteValue[1]);
    delayMicroseconds(50);      // small delay
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
    int16_t i,j;
    bool isFirstPixel = true;
    uint8_t regValue;
    uint8_t pixelValue;

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
            delayMicroseconds(50);
        }
        serPort->println();
    }

    // hardware reset to restore sensor to normal operation
    reset();
}
