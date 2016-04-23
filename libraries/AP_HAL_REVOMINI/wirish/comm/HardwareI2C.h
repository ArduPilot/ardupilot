/*
  I2C.h - I2C library
  Copyright (c) 2011 Wayne Truchsess.  All right reserved.
  Rev 2.0 - September 19th, 2011
          - Added support for timeout function to prevent 
            and recover from bus lockup (thanks to PaulS
            and CrossRoads on the Arduino forum)
          - Changed return type for stop() from void to
            uint8_t to handle timeOut function 
  Rev 1.0 - August 8th, 2011
  
  This is a modified version of the Arduino Wire/TWI 
  library.  Functions were rewritten to provide more functionality
  and also the use of Repeated Start.  Some I2C devices will not
  function correctly without the use of a Repeated Start.  The 
  initial version of this library only supports the Master.


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdint.h>
#include "wirish.h"
#include "i2c.h"

//#include "boards.h"

#ifndef _HARDWAREI2C_H_
#define _HARDWAREI2C_H_

class HardwareI2C
{
  public:
	HardwareI2C(uint32_t i2c_num);
    void begin();
    void end();
    void setSpeed(boolean); 
    ///////////////////////////////////////////
    int8_t write(uint8_t, uint8_t, uint8_t*);
    int8_t write(uint8_t, uint8_t, uint8_t);
    int8_t write(uint8_t, uint16_t, uint8_t);
    int8_t read(uint8_t, uint8_t, uint8_t, uint8_t*);
    int8_t read(uint8_t, uint16_t, uint8_t, uint8_t*);

  private:
    i2c_dev *i2c_d;

};

#endif
