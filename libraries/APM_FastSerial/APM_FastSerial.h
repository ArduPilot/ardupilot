#ifndef APM_FastSerial_h
#define APM_FastSerial_h

#include <inttypes.h>

class APM_FastSerial_Class
{
  private:    
    void printNumber(unsigned long, uint8_t);
	void printFloat(double number, uint8_t digits);
    uint8_t SerialPortNumber;
  public:
    APM_FastSerial_Class(uint8_t SerialPort); // Constructor	
	void write(uint8_t b);  // basic funtion : send a byte   
	void write(const uint8_t *buffer, int size);
    void print(char);
    void print(const char[]);
    void print(uint8_t);
    void print(int);
    void print(unsigned int);
    void print(long);
    void print(unsigned long);
    void print(long, int);
	void print(double, int=2);
    void println(void);
    void println(char);
    void println(const char[]);
    void println(uint8_t);
    void println(int);
    void println(long);
    void println(unsigned long);
    void println(long, int);
	void println(double, int=2);
};

extern APM_FastSerial_Class APM_FastSerial;
extern APM_FastSerial_Class APM_FastSerial3;
#endif

