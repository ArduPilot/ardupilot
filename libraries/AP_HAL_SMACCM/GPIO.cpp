
#include "GPIO.h"

using namespace SMACCM;

SMACCMGPIO::SMACCMGPIO()
{
}

void SMACCMGPIO::init()
{
}

void SMACCMGPIO::pinMode(uint8_t pin, uint8_t output)
{
}

int8_t SMACCMGPIO::analogPinToDigitalPin(uint8_t pin)
{
	return -1;
}

uint8_t SMACCMGPIO::read(uint8_t pin)
{
  return 0;
}

void SMACCMGPIO::write(uint8_t pin, uint8_t value)
{
}

void SMACCMGPIO::toggle(uint8_t pin)
{
}

bool SMACCMGPIO::usb_connected(void)
{
    return false;
}

/* Alternative interface: */
AP_HAL::DigitalSource* SMACCMGPIO::channel(uint16_t n)
{
  return NULL;
}

/* Interrupt interface: */
bool SMACCMGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
                                  uint8_t mode)
{
  return true;
}

SMACCMDigitalSource::SMACCMDigitalSource(uint8_t v) :
  _v(v)
{
}

void SMACCMDigitalSource::mode(uint8_t output)
{
}

uint8_t SMACCMDigitalSource::read()
{
  return _v;
}

void SMACCMDigitalSource::write(uint8_t value)
{
  _v = value;
}

void SMACCMDigitalSource::toggle()
{
  _v = !_v;
}
