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
  Flymaple port by Mike McCauley
 */

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
#include <stdlib.h>

#include "UARTDriver.h"
#include "FlymapleWirish.h"
#include <HardwareSerial.h>
#include <usart.h>

using namespace AP_HAL_FLYMAPLE_NS;

FLYMAPLEUARTDriver::FLYMAPLEUARTDriver(HardwareSerial* hws):
    _hws(hws),
    _txBuf(NULL),
    _txBufSize(63) // libmaple usart default driver buffer of 63
 {}

void FLYMAPLEUARTDriver::begin(uint32_t b) 
{
    _hws->begin(b);
}

void FLYMAPLEUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
    // Our private buffer can only grow, never shrink
    if (txS > _txBufSize)
    {
	if (_txBuf)
	    free(_txBuf); // CAUTION: old contents lost
	_txBuf = (uint8_t*)malloc(txS);
	_txBufSize = txS;
    }
    begin(b); // libmaple internal ring buffer reinititalised to defaults here
    if (_txBuf)
	rb_init(_hws->c_dev()->rb, _txBufSize, _txBuf); // Get the ring buffer size we want
}

void FLYMAPLEUARTDriver::end() 
{
    _hws->end();
}

void FLYMAPLEUARTDriver::flush() 
{
    _hws->flush();
}

bool FLYMAPLEUARTDriver::is_initialized() 
{ 
    return true; 
}

void FLYMAPLEUARTDriver::set_blocking_writes(bool blocking) {}

bool FLYMAPLEUARTDriver::tx_pending() { return false; }

/* FLYMAPLE implementations of Stream virtual methods */
int16_t FLYMAPLEUARTDriver::available() 
{ 
    return _hws->available(); 
}

int16_t FLYMAPLEUARTDriver::txspace() 
{ 
    // Get available space from guts of HardwareSerial
    // CAUTION: dependent on implmentation of HardwareSerial
    return _hws->c_dev()->rb->size - rb_full_count(_hws->c_dev()->rb);
}

int16_t FLYMAPLEUARTDriver::read() 
{ 
    return _hws->read(); 
}

/* FLYMAPLE implementations of Print virtual methods */
size_t FLYMAPLEUARTDriver::write(uint8_t c) 
{
    _hws->write(c);
    return 1; 
}

size_t FLYMAPLEUARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

#endif
