/*
  JETI_Box.h, Version 1.0 beta
  July 2010, by Uwe Gartmann
  Library acts as a Sensor when connected to a Jeti Receiver
*/

#ifndef JETI_Box_h
#define JETI_Box_h

//#include <inttypes.h>
//#include "Print.h"

#define LCDLine1 1
#define LCDLine2 17

#define JB_key_up 		0b0010
#define JB_key_right	0b0001
#define JB_key_down		0b0100
#define JB_key_left		0b1000

class JETI_Box_class : public Print {
public:
	uint8_t readbuttons(void);
	//long checkvalue(long v);
	virtual void write(uint8_t c);
	using Print::write; // pull in write(str) and write(buf, size) from Print
	JETI_Box_class();
	void begin();
	void setcursor(uint8_t p);
	void clear();
	void clear(uint8_t l);
};

extern JETI_Box_class JB;

#endif
