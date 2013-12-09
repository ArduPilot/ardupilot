/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_ADC_ADS7844.cpp - ADC ADS7844 Library for Ardupilot Mega
 *       Code by Jordi Mu�oz and Jose Julio. DIYDrones.com
 *
 *       Modified by John Ihlein 6 / 19 / 2010 to:
 *       1)Prevent overflow of adc_counter when more than 8 samples collected between reads.	Probably
 *               only an issue on initial read of ADC at program start.
 *       2)Reorder analog read order as follows:
 *               p, q, r, ax, ay, az
 *       External ADC ADS7844 is connected via Serial port 2 (in SPI mode)
 *       TXD2 = MOSI = pin PH1
 *       RXD2 = MISO = pin PH0
 *       XCK2 = SCK = pin PH2
 *       Chip Select pin is PC4 (33)	 [PH6 (9)]
 *       We are using the 16 clocks per conversion timming to increase efficiency (fast)
 *
 *       The sampling frequency is 1kHz (Timer2 overflow interrupt)
 *
 *       So if our loop is at 50Hz, our needed sampling freq should be 100Hz, so
 *       we have an 10x oversampling and averaging.
 *
 *       Methods:
 *               Init() : Initialization of interrupts an Timers (Timer2 overflow interrupt)
 *               Ch(ch_num) : Return the ADC channel value
 *
 *       // HJI - Input definitions.  USB connector assumed to be on the left, Rx and servo
 *       // connector pins to the rear.  IMU shield components facing up.  These are board
 *       // referenced sensor inputs, not device referenced.
 *       On Ardupilot Mega Hardware, oriented as described above:
 *       Chennel 0 : yaw rate, r
 *       Channel 1 : roll rate, p
 *       Channel 2 : pitch rate, q
 *       Channel 3 : x / y gyro temperature
 *       Channel 4 : x acceleration, aX
 *       Channel 5 : y acceleration, aY
 *       Channel 6 : z acceleration, aZ
 *       Channel 7 : Differential pressure sensor port
 *
 */

#include <AP_Progmem.h>
#include <AP_Common.h>
#include <AP_HAL.h>

#include "AP_ADC_ADS7844.h"

extern const AP_HAL::HAL& hal;

// DO NOT CHANGE FROM 8!!
#define ADC_ACCEL_FILTER_SIZE 8
// Commands for reading ADC channels on ADS7844
static const unsigned char adc_cmd[17] =
    { 0x87, 0, 0xC7, 0, 0x97, 0, 0xD7, 0, 0xA7, 0, 0xE7, 0, 0xB7, 0, 0xF7};

// the sum of the values since last read
static volatile uint32_t _sum[8];

// how many values we've accumulated since last read
static volatile uint16_t _count[8];

// variables to calculate time period over which a group of samples were
// collected
// time we start collecting sample (reset on update)
static volatile uint32_t _ch6_delta_time_start_micros = 0;
// time latest sample was collected
static volatile uint32_t _ch6_last_sample_time_micros = 0;

void AP_ADC_ADS7844::read(void)
{
    static int semfail_ctr = 0;
    uint8_t ch;

    /** Take nonblocking: ::read happens from the TimerProcess context! */
    bool got = _spi_sem->take_nonblocking();
    if (!got) { 
        semfail_ctr++;
        if (semfail_ctr > 100) {
            hal.scheduler->panic(PSTR("PANIC: failed to take _spi_sem "
                        "100 times in AP_ADC_ADS7844::read"));
        }
        return;
    } else {
        semfail_ctr = 0;
    }   

    uint8_t rx[17];
    _spi->transaction(adc_cmd, rx, 17);

    for (ch = 0; ch < 8; ch++) {
        uint16_t v = (rx[2*ch+1] << 8) | rx[2*ch+2];
        if (v & 0x8007) {
            // this is a 12-bit ADC, shifted by 3 bits.
            // if we get other bits set then the value is
            // bogus and should be ignored
            continue;
        }
        if (++_count[ch] == 0) {
            // overflow ... shouldn't happen too often
            // unless we're just not using the
            // channel. Notice that we overflow the count
            // to 1 here, not zero, as otherwise the
            // reader below could get a division by zero
            _sum[ch] = 0;
            _count[ch] = 1;
        }
        _sum[ch] += (v >> 3);
    }

    _spi_sem->give();

    // record time of this sample
    _ch6_last_sample_time_micros = hal.scheduler->micros();
}


// Constructors ////////////////////////////////////////////////////////////////
AP_ADC_ADS7844::AP_ADC_ADS7844() { }

// Public Methods //////////////////////////////////////////////////////////////
void AP_ADC_ADS7844::Init()
{
    hal.scheduler->suspend_timer_procs();
    _spi = hal.spi->device(AP_HAL::SPIDevice_ADS7844);
    if (_spi == NULL) {
        hal.scheduler->panic(PSTR("PANIC: AP_ADC_ADS7844 missing SPI device "
                    "driver\n"));
    }

    _spi_sem = _spi->get_semaphore();

    if (_spi_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: AP_ADC_ADS7844 missing SPI device "
                    "semaphore"));
    }
   
    if (!_spi_sem->take(0)) {
        hal.scheduler->panic(PSTR("PANIC: failed to take _spi_sem in"
                    "AP_ADC_ADS7844::Init"));
    }
    
    _spi->cs_assert();
    // get an initial value for each channel. This ensures
    // _count[] is never zero
    for (uint8_t i=0; i<8; i++) {
        uint16_t adc_tmp;
        adc_tmp  = _spi->transfer(0) << 8;
        adc_tmp |= _spi->transfer(adc_cmd[i + 1]);
        _count[i] = 1;
        _sum[i]   = adc_tmp;
    }
    
    _spi->cs_release();

    _spi_sem->give();

    _ch6_last_sample_time_micros = hal.scheduler->micros();

    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_ADC_ADS7844::read));
    hal.scheduler->resume_timer_procs();

}

// Read one channel value
float AP_ADC_ADS7844::Ch(uint8_t ch_num)
{
    uint16_t count;
    uint32_t sum;

    // ensure we have at least one value
    while (_count[ch_num] == 0) /* noop */;

    // grab the value with timer procs disabled, and clear the count
    hal.scheduler->suspend_timer_procs();
    count = _count[ch_num];
    sum   = _sum[ch_num];
    _count[ch_num] = 0;
    _sum[ch_num]   = 0;
    hal.scheduler->resume_timer_procs();

    return ((float)sum)/count;
}

// see if Ch6() can return new data
bool AP_ADC_ADS7844::new_data_available(const uint8_t *channel_numbers)
{
    uint8_t i;

    for (i=0; i<6; i++) {
        if (_count[channel_numbers[i]] == 0) {
            return false;
        }
    }
    return true;
}


// Read 6 channel values
// this assumes that the counts for all of the 6 channels are
// equal. This will only be true if we always consistently access a
// sensor by either Ch6() or Ch() and never mix them. If you mix them
// then you will get very strange results
uint32_t AP_ADC_ADS7844::Ch6(const uint8_t *channel_numbers, float *result)
{
    uint16_t count[6];
    uint32_t sum[6];
    uint8_t i;

    // ensure we have at least one value
    for (i=0; i<6; i++) {
        while (_count[channel_numbers[i]] == 0) /* noop */;
    }

    // grab the values with timer procs disabled, and clear the counts
    hal.scheduler->suspend_timer_procs();
    for (i=0; i<6; i++) {
        count[i] = _count[channel_numbers[i]];
        sum[i]   = _sum[channel_numbers[i]];
        _count[channel_numbers[i]] = 0;
        _sum[channel_numbers[i]]   = 0;
    }

    // calculate the delta time.
    // we do this before re-enabling interrupts because another sensor read could fire immediately and change the _last_sensor_time value
    uint32_t ret = _ch6_last_sample_time_micros - _ch6_delta_time_start_micros;
    _ch6_delta_time_start_micros = _ch6_last_sample_time_micros;

    hal.scheduler->resume_timer_procs();

    // calculate averages. We keep this out of the cli region
    // to prevent us stalling the ISR while doing the
    // division. That costs us 36 bytes of stack, but I think its
    // worth it.
    for (i = 0; i < 6; i++) {
        result[i] = sum[i] / (float)count[i];
    }

    // return number of microseconds since last call
    return ret;
}

/// Get minimum number of samples read from the sensors
uint16_t AP_ADC_ADS7844::num_samples_available(const uint8_t *channel_numbers)
{
    // get count of first channel as a base
    uint16_t min_count = _count[channel_numbers[0]];

    // reduce to minimum count of all other channels
    for (uint8_t i=1; i<6; i++) {
        if (_count[channel_numbers[i]] < min_count) {
            min_count = _count[channel_numbers[i]];
        }
    }
    return min_count;
}
