/*
 *       AP_OpticalFlow_ADNS3080.cpp - ADNS3080 OpticalFlow Library for
 *       Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 */

#include <AP_HAL.h>
#include "AP_OpticalFlow_ADNS3080.h"

extern const AP_HAL::HAL& hal;

#define AP_SPI_TIMEOUT 1000

union NumericIntType
{
    int16_t intValue;
    uint16_t uintValue;
    uint8_t byteValue[2];
};

// Constructors ////////////////////////////////////////////////////////////////
AP_OpticalFlow_ADNS3080::AP_OpticalFlow_ADNS3080(uint8_t reset_pin) :
    _reset_pin(reset_pin)
{
    num_pixels = ADNS3080_PIXELS_X;
    field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV;
    scaler = AP_OPTICALFLOW_ADNS3080_SCALER;
}

// Public Methods //////////////////////////////////////////////////////////////
// init - initialise sensor
// assumes SPI bus has been initialised but will attempt to initialise 
// nonstandard SPI3 bus if required
bool
AP_OpticalFlow_ADNS3080::init()
{
    int8_t retry = 0;
    bool retvalue = false;

    // suspend timer while we set-up SPI communication
    hal.scheduler->suspend_timer_procs();

    if( _reset_pin != 0)
        hal.gpio->pinMode(_reset_pin, GPIO_OUTPUT);
    // reset the device
    reset();

    // check 3 times for the sensor on standard SPI bus
    _spi = hal.spi->device(AP_HAL::SPIDevice_ADNS3080_SPI0);
    if (_spi == NULL) {
        retvalue = false; goto finish;
    }

    while( retvalue == false && retry < 3 ) {
        if( read_register(ADNS3080_PRODUCT_ID) == 0x17 ) {
            retvalue = true;
            goto finish;
        }
        retry++;
    }

    // if not found, check 3 times on SPI3
    _spi = hal.spi->device(AP_HAL::SPIDevice_ADNS3080_SPI3);
    if (_spi == NULL) {
        retvalue = false; goto finish;
    }
    retry = 0;
    while( retvalue == false && retry < 3 ) {
        if( read_register(ADNS3080_PRODUCT_ID) == 0x17 ) {
            retvalue = true;
        }
        retry++;
    }
    // If we fail to find on SPI3, no connection available.
    retvalue = false;
    _spi = NULL;

finish:
    // resume timer
    hal.scheduler->resume_timer_procs();

    // if device is working register the global static read function to
    // be called at 1khz
    if( retvalue ) {
        hal.scheduler->register_timer_process( AP_OpticalFlow_ADNS3080::read );
    }

    return retvalue;
}


// Read a register from the sensor
uint8_t AP_OpticalFlow_ADNS3080::read_register(uint8_t address)
{
    if (_spi == NULL) return 0;

    // get spi semaphore if required
    AP_HAL::Semaphore* sem = _spi->get_semaphore();
    if( sem != NULL ) {
        // if failed to get semaphore then just quietly fail
        if( !sem->get(this) ) {
            return 0;
        }
    }

    _spi->cs_assert();
    // send the device the register you want to read:
    _spi->transfer(address);
    hal.scheduler->delay_microseconds(50);
    // send a value of 0 to read the first byte returned:
    uint8_t result = _spi->transfer(0x00);

    _spi->cs_release();

    // get spi semaphore if required
    if( sem != NULL ) {
        sem->release(this);
    }

    return result;
}

// write a value to one of the sensor's registers
void AP_OpticalFlow_ADNS3080::write_register(uint8_t address, uint8_t value)
{
    if (_spi == NULL) return;

    AP_HAL::Semaphore* sem = _spi->get_semaphore();
    // get spi semaphore if required
    if( sem != NULL ) {
        // if failed to get semaphore then just quietly fail
        if( !sem->get(this) ) {
            return;
        }
    }

    _spi->cs_assert();

    // send register address
    _spi->transfer(address | 0x80 );
    hal.scheduler->delay_microseconds(50);
    // send data
    _spi->transfer(value);

    _spi->cs_release();
    
    // get spi3 semaphore if required
    if( sem != NULL ) {
        sem->release(this);
    }
}

// reset sensor by holding a pin high (or is it low?) for 10us.
void
AP_OpticalFlow_ADNS3080::reset()
{
    // return immediately if the reset pin is not defined
    if( _reset_pin == 0)
        return;

    // reset sensor
    hal.gpio->write(_reset_pin, 1);
    hal.scheduler->delay_microseconds(10);
    // return sensor to normal
    hal.gpio->write(_reset_pin, 0);
}

// read latest values from sensor and fill in x,y and totals
void
AP_OpticalFlow_ADNS3080::update(uint32_t now)
{
    uint8_t motion_reg;
    surface_quality = (uint16_t)read_register(ADNS3080_SQUAL);
    hal.scheduler->delay_microseconds(50);

    // check for movement, update x,y values
    motion_reg = read_register(ADNS3080_MOTION);
    // check if we've had an overflow
    _overflow = ((motion_reg & 0x10) != 0);
    if( (motion_reg & 0x80) != 0 ) {
        raw_dx = ((int8_t)read_register(ADNS3080_DELTA_X));
        hal.scheduler->delay_microseconds(50);
        raw_dy = ((int8_t)read_register(ADNS3080_DELTA_Y));
        _motion = true;
    }else{
        raw_dx = 0;
        raw_dy = 0;
    }

    last_update = hal.scheduler->millis();

    apply_orientation_matrix();
}

void
AP_OpticalFlow_ADNS3080::disable_serial_pullup()
{
    uint8_t regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    regVal = (regVal | ADNS3080_SERIALNPU_OFF);
    hal.scheduler->delay_microseconds(50);
    write_register(ADNS3080_EXTENDED_CONFIG, regVal);
}

// get_led_always_on - returns true if LED is always on, false if only on
// when required
bool AP_OpticalFlow_ADNS3080::get_led_always_on()
{
    return ( (read_register(ADNS3080_CONFIGURATION_BITS) & 0x40) > 0 );
}

// set_led_always_on - set parameter to true if you want LED always on,
// otherwise false for only when required
void AP_OpticalFlow_ADNS3080::set_led_always_on( bool alwaysOn )
{
    uint8_t regVal = read_register(ADNS3080_CONFIGURATION_BITS);
    regVal = (regVal & 0xbf) | (alwaysOn << 6);
    hal.scheduler->delay_microseconds(50);
    write_register(ADNS3080_CONFIGURATION_BITS, regVal);
}

// returns resolution (either 400 or 1600 counts per inch)
int16_t AP_OpticalFlow_ADNS3080::get_resolution()
{
    if( (read_register(ADNS3080_CONFIGURATION_BITS) & 0x10) == 0 )
        return 400;
    else
        return 1600;
}

// set parameter to 400 or 1600 counts per inch
void AP_OpticalFlow_ADNS3080::set_resolution(uint16_t resolution)
{
    uint8_t regVal = read_register(ADNS3080_CONFIGURATION_BITS);

    if( resolution == ADNS3080_RESOLUTION_400 ) {
        regVal &= ~0x10;
        scaler = AP_OPTICALFLOW_ADNS3080_SCALER;
    }else if( resolution == ADNS3080_RESOLUTION_1600) {
        regVal |= 0x10;
        scaler = AP_OPTICALFLOW_ADNS3080_SCALER * 4;
    }

    hal.scheduler->delay_microseconds(50);
    write_register(ADNS3080_CONFIGURATION_BITS, regVal);

    // this will affect conversion factors so update them
    update_conversion_factors();
}

// get_frame_rate_auto - return whether frame rate is set to "auto" or manual
bool AP_OpticalFlow_ADNS3080::get_frame_rate_auto()
{
    uint8_t regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    if( (regVal & 0x01) != 0 ) {
        return false;
    }else{
        return true;
    }
}

// set_frame_rate_auto - set frame rate to auto (true) or manual (false)
void AP_OpticalFlow_ADNS3080::set_frame_rate_auto(bool auto_frame_rate)
{
    uint8_t regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    hal.scheduler->delay_microseconds(50);
    if( auto_frame_rate == true ) {
        // set specific frame period
        write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0xE0);
        hal.scheduler->delay_microseconds(50);
        write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x1A);
        hal.scheduler->delay_microseconds(50);

        // decide what value to update in extended config
        regVal = (regVal & ~0x01);
    }else{
        // decide what value to update in extended config
        regVal = (regVal & ~0x01) | 0x01;
    }
    write_register(ADNS3080_EXTENDED_CONFIG, regVal);
}

// get frame period
uint16_t AP_OpticalFlow_ADNS3080::get_frame_period()
{
    NumericIntType aNum;
    aNum.byteValue[1] = read_register(ADNS3080_FRAME_PERIOD_UPPER);
    hal.scheduler->delay_microseconds(50);
    aNum.byteValue[0] = read_register(ADNS3080_FRAME_PERIOD_LOWER);
    return aNum.uintValue;
}

// set frame period
void AP_OpticalFlow_ADNS3080::set_frame_period(uint16_t period)
{
    NumericIntType aNum;
    aNum.uintValue = period;

    // set frame rate to manual
    set_frame_rate_auto(false);
    hal.scheduler->delay_microseconds(50);

    // set specific frame period
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,aNum.byteValue[0]);
    hal.scheduler->delay_microseconds(50);
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,aNum.byteValue[1]);

}

uint16_t AP_OpticalFlow_ADNS3080::get_frame_rate()
{
    uint32_t clockSpeed = ADNS3080_CLOCK_SPEED;
    uint16_t rate = clockSpeed / get_frame_period();
    return rate;
}

void AP_OpticalFlow_ADNS3080::set_frame_rate(uint16_t rate)
{
    uint32_t clockSpeed = ADNS3080_CLOCK_SPEED;
    uint16_t period = (uint16_t)(clockSpeed / (uint32_t)rate);

    set_frame_period(period);
}

// get_shutter_speed_auto - returns true if shutter speed is adjusted
// automatically, false if manual
bool AP_OpticalFlow_ADNS3080::get_shutter_speed_auto()
{
    uint8_t regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    if( (regVal & 0x02) > 0 ) {
        return false;
    }else{
        return true;
    }
}

// set_shutter_speed_auto - set shutter speed to auto (true), or manual (false)
void AP_OpticalFlow_ADNS3080::set_shutter_speed_auto(bool auto_shutter_speed)
{
    uint8_t regVal = read_register(ADNS3080_EXTENDED_CONFIG);
    hal.scheduler->delay_microseconds(50);
    if( auto_shutter_speed ) {
        // return shutter speed max to default
        write_register(ADNS3080_SHUTTER_MAX_BOUND_LOWER,0x8c);
        hal.scheduler->delay_microseconds(50);
        write_register(ADNS3080_SHUTTER_MAX_BOUND_UPPER,0x20);
        hal.scheduler->delay_microseconds(50);

        // determine value to put into extended config
        regVal &= ~0x02;
    }else{
        // determine value to put into extended config
        regVal |= 0x02;
    }
    write_register(ADNS3080_EXTENDED_CONFIG, regVal);
    hal.scheduler->delay_microseconds(50);
}

// get_shutter_speed_auto - returns true if shutter speed is adjusted
// automatically, false if manual
uint16_t AP_OpticalFlow_ADNS3080::get_shutter_speed()
{
    NumericIntType aNum;
    aNum.byteValue[1] = read_register(ADNS3080_SHUTTER_UPPER);
    hal.scheduler->delay_microseconds(50);
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
    hal.scheduler->delay_microseconds(50);

    // set specific shutter speed
    write_register(ADNS3080_SHUTTER_MAX_BOUND_LOWER,aNum.byteValue[0]);
    hal.scheduler->delay_microseconds(50);
    write_register(ADNS3080_SHUTTER_MAX_BOUND_UPPER,aNum.byteValue[1]);
    hal.scheduler->delay_microseconds(50);

    // larger delay
    hal.scheduler->delay(50);

    // need to update frame period to cause shutter value to take effect
    aNum.byteValue[1] = read_register(ADNS3080_FRAME_PERIOD_UPPER);
    hal.scheduler->delay_microseconds(50);
    aNum.byteValue[0] = read_register(ADNS3080_FRAME_PERIOD_LOWER);
    hal.scheduler->delay_microseconds(50);
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,aNum.byteValue[0]);
    hal.scheduler->delay_microseconds(50);
    write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,aNum.byteValue[1]);
    hal.scheduler->delay_microseconds(50);
}

// clear_motion - will cause the Delta_X, Delta_Y, and internal motion
// registers to be cleared
void AP_OpticalFlow_ADNS3080::clear_motion()
{
    // writing anything to this register will clear the sensor's motion
    // registers
    write_register(ADNS3080_MOTION_CLEAR,0xFF); 
    x = 0;
    y = 0;
    dx = 0;
    dy = 0;
    _motion = false;
}

// get_pixel_data - captures an image from the sensor and stores it to the
// pixe_data array
void AP_OpticalFlow_ADNS3080::print_pixel_data()
{
    int16_t i,j;
    bool isFirstPixel = true;
    uint8_t regValue;
    uint8_t pixelValue;

    // write to frame capture register to force capture of frame
    write_register(ADNS3080_FRAME_CAPTURE,0x83);

    // wait 3 frame periods + 10 nanoseconds for frame to be captured
    // min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.
    // so 500 x 3 + 10 = 1510
    hal.scheduler->delay_microseconds(1510);

    // display the pixel data
    for( i=0; i<ADNS3080_PIXELS_Y; i++ ) {
        for( j=0; j<ADNS3080_PIXELS_X; j++ ) {
            regValue = read_register(ADNS3080_FRAME_CAPTURE);
            if( isFirstPixel && (regValue & 0x40) == 0 ) {
                hal.console->println_P(
                        PSTR("Optflow: failed to find first pixel"));
            }
            isFirstPixel = false;
            pixelValue = ( regValue << 2 );
            hal.console->print(pixelValue,DEC);
            if( j!= ADNS3080_PIXELS_X-1 )
                hal.console->print_P(PSTR(","));
            hal.scheduler->delay_microseconds(50);
        }
        hal.console->println();
    }

    // hardware reset to restore sensor to normal operation
    reset();
}
