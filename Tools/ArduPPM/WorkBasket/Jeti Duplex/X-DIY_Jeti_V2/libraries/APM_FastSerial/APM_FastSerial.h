#ifndef APM_FastSerial_h
#define APM_FastSerial_h

#include <inttypes.h>

#include "Print.h"

class APM_FastSerial_Class : public Print   // Inherit from Print
{
  private:    
    uint8_t SerialPortNumber;

  public:
    APM_FastSerial_Class(uint8_t SerialPort); // Constructor	
	// we overwrite the write methods
	void write(uint8_t b);                    // basic funtion : send a byte   
	void write(const uint8_t *buffer, int size);
};

extern APM_FastSerial_Class APM_FastSerial;
#if defined(__AVR_ATmega1280__)
	extern APM_FastSerial_Class APM_FastSerial3;
#endif
#endif

