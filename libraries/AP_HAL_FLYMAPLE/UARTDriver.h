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

#ifndef __AP_HAL_FLYMAPLE_UARTDRIVER_H__
#define __AP_HAL_FLYMAPLE_UARTDRIVER_H__

#include <AP_HAL_FLYMAPLE.h>

class HardwareSerial; // A libmaple classs

class AP_HAL_FLYMAPLE_NS::FLYMAPLEUARTDriver : public AP_HAL::UARTDriver {
public:
    FLYMAPLEUARTDriver(HardwareSerial* hws);

    /* FLYMAPLE implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* FLYMAPLE implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();

    /* FLYMAPLE implementations of Print virtual methods */
    size_t write(uint8_t c);
private:
    HardwareSerial*    _hws;
};

#endif // __AP_HAL_FLYMAPLE_UARTDRIVER_H__
