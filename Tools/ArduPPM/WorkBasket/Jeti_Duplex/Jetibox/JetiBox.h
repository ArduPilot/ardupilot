/*
  JetiBox.h, Version 1.0 beta
  July 2010, by Uwe Gartmann

  Library acts as a Sensor when connected to a Jeti Receiver
  written for Arduino Mega / ArduPilot Mega

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

#ifndef JetiBox_h
#define JetiBox_h

#include <inttypes.h>
#include <AP_HAL/utility/Print.h>

#define jbox_key_up 0b0010
#define jbox_key_right 0b0001
#define jbox_key_down 0b0100
#define jbox_key_left 0b1000

struct jeti_box;

class JetiBox : public Print
{
  public:
	JetiBox();
    void begin();
    void refresh();
    uint8_t keys(void);
    virtual void write(uint8_t);
    using Print::write; // pull in write(str) and write(buf, size) from Print
    void line1();
    void line2();
};

extern JetiBox JBox;

#endif
