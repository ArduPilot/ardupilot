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

/*
 *       AP_OpticalFlow_ADNS3080.cpp - ADNS3080 OpticalFlow Library for
 *       Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 */

#include <AP_HAL.h>
#include "AP_OpticalFlow_ADNS3080.h"

extern const AP_HAL::HAL& hal;

// Constructors ////////////////////////////////////////////////////////////////
AP_OpticalFlow_ADNS3080::AP_OpticalFlow_ADNS3080()
{
    field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV;
    scaler = AP_OPTICALFLOW_ADNS3080_SCALER_1600;
}

// Public Methods //////////////////////////////////////////////////////////////
// init - initialise sensor
// assumes SPI bus has been initialised but will attempt to initialise 
// nonstandard SPI3 bus if required
void AP_OpticalFlow_ADNS3080::init()
{
    int8_t retry = 0;
    _flags.healthy = false;

    // suspend timer while we set-up SPI communication
    hal.scheduler->suspend_timer_procs();

    // get pointer to the spi bus
    _spi = hal.spi->device(AP_HAL::SPIDevice_ADNS3080_SPI0);
    if (_spi != NULL) {
        // check 3 times for the sensor on standard SPI bus
        while (!_flags.healthy && retry < 3) {
            if (read_register(ADNS3080_PRODUCT_ID) == 0x17) {
                _flags.healthy = true;
            }
            retry++;
        }
    }

    // if not yet found, get pointer to the SPI3 bus
    if (!_flags.healthy) {
        _spi = hal.spi->device(AP_HAL::SPIDevice_ADNS3080_SPI3);
        if (_spi != NULL) {
            // check 3 times on SPI3
            retry = 0;
            while (!_flags.healthy && retry < 3) {
                if (read_register(ADNS3080_PRODUCT_ID) == 0x17) {
                    _flags.healthy = true;
                }
                retry++;
            }
        }
    }

    // configure the sensor
    if (_flags.healthy) {
        // set frame rate to manual
        uint8_t regVal = read_register(ADNS3080_EXTENDED_CONFIG);
        hal.scheduler->delay_microseconds(50);
        regVal = (regVal & ~0x01) | 0x01;
        write_register(ADNS3080_EXTENDED_CONFIG, regVal);
        hal.scheduler->delay_microseconds(50);

        // set frame period to 12000 (0x2EE0)
        write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0xE0);
        hal.scheduler->delay_microseconds(50);
        write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x2E);
        hal.scheduler->delay_microseconds(50);

        // set 1600 resolution bit
        regVal = read_register(ADNS3080_CONFIGURATION_BITS);
        hal.scheduler->delay_microseconds(50);
        regVal |= 0x10;
        write_register(ADNS3080_CONFIGURATION_BITS, regVal);
        hal.scheduler->delay_microseconds(50);

        // update scalers
        update_conversion_factors();

        // register the global static read function to be called at 1khz
        hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_OpticalFlow_ADNS3080::read));
    }else{
        // no connection available.
        _spi = NULL;
    }

    // resume timer
    hal.scheduler->resume_timer_procs();
}

// Read a register from the sensor
uint8_t AP_OpticalFlow_ADNS3080::read_register(uint8_t address)
{
    AP_HAL::Semaphore *spi_sem;

    // check that we have an spi bus
    if (_spi == NULL) {
        return 0;
    }

    // get spi bus semaphore
    spi_sem = _spi->get_semaphore();

    // try to get control of the spi bus
    if (spi_sem == NULL || !spi_sem->take_nonblocking()) {
        return 0;
    }

    _spi->cs_assert();
    // send the device the register you want to read:
    _spi->transfer(address);
    hal.scheduler->delay_microseconds(50);
    // send a value of 0 to read the first byte returned:
    uint8_t result = _spi->transfer(0x00);

    _spi->cs_release();

    // release the spi bus
    spi_sem->give();

    return result;
}

// write a value to one of the sensor's registers
void AP_OpticalFlow_ADNS3080::write_register(uint8_t address, uint8_t value)
{
    AP_HAL::Semaphore *spi_sem;

    // check that we have an spi bus
    if (_spi == NULL) {
        return;
    }

    // get spi bus semaphore
    spi_sem = _spi->get_semaphore();

    // try to get control of the spi bus
    if (spi_sem == NULL || !spi_sem->take_nonblocking()) {
        return;
    }

    _spi->cs_assert();

    // send register address
    _spi->transfer(address | 0x80 );
    hal.scheduler->delay_microseconds(50);
    // send data
    _spi->transfer(value);

    _spi->cs_release();

    // release the spi bus
    spi_sem->give();
}

// read latest values from sensor and fill in x,y and totals
void AP_OpticalFlow_ADNS3080::update(void)
{
    uint8_t motion_reg;
    surface_quality = read_register(ADNS3080_SQUAL);
    hal.scheduler->delay_microseconds(50);

    // check for movement, update x,y values
    motion_reg = read_register(ADNS3080_MOTION);
    if ((motion_reg & 0x80) != 0) {
        raw_dx = ((int8_t)read_register(ADNS3080_DELTA_X));
        hal.scheduler->delay_microseconds(50);
        raw_dy = ((int8_t)read_register(ADNS3080_DELTA_Y));
    }else{
        raw_dx = 0;
        raw_dy = 0;
    }

    last_update = hal.scheduler->millis();

    Vector3f rot_vector(raw_dx, raw_dy, 0);

    // rotate dx and dy
    rot_vector.rotate(_orientation);
    dx = rot_vector.x;
    dy = rot_vector.y;
}

// parent method called at 1khz by periodic process
// this is slowed down to 20hz and each instance's update function is called
// (only one instance is supported at the moment)
void AP_OpticalFlow_ADNS3080::read(void)
{
    _num_calls++;

    if (_num_calls >= AP_OPTICALFLOW_ADNS3080_NUM_CALLS_FOR_20HZ) {
        _num_calls = 0;
        update();
    }
};

// clear_motion - will cause the Delta_X, Delta_Y, and internal motion
// registers to be cleared
void AP_OpticalFlow_ADNS3080::clear_motion()
{
    // writing anything to this register will clear the sensor's motion
    // registers
    write_register(ADNS3080_MOTION_CLEAR,0xFF); 
    x_cm = 0;
    y_cm = 0;
    dx = 0;
    dy = 0;
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
    for (i=0; i<ADNS3080_PIXELS_Y; i++) {
        for (j=0; j<ADNS3080_PIXELS_X; j++) {
            regValue = read_register(ADNS3080_FRAME_CAPTURE);
            if (isFirstPixel && (regValue & 0x40) == 0) {
                hal.console->println_P(
                        PSTR("Optflow: failed to find first pixel"));
            }
            isFirstPixel = false;
            pixelValue = ( regValue << 2 );
            hal.console->print(pixelValue,BASE_DEC);
            if (j!= ADNS3080_PIXELS_X-1)
                hal.console->print_P(PSTR(","));
            hal.scheduler->delay_microseconds(50);
        }
        hal.console->println();
    }
}

// updates conversion factors that are dependent upon field_of_view
void AP_OpticalFlow_ADNS3080::update_conversion_factors()
{
    // multiply this number by altitude and pixel change to get horizontal
    // move (in same units as altitude)
    conv_factor = ((1.0f / (float)(ADNS3080_PIXELS_X * scaler))
                   * 2.0f * tanf(field_of_view / 2.0f));
    // 0.00615
    radians_to_pixels = (ADNS3080_PIXELS_X * scaler) / field_of_view;
    // 162.99
}
