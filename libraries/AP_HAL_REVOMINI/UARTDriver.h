
#ifndef __AP_HAL_REVOMINI_UARTDRIVER_H__
#define __AP_HAL_REVOMINI_UARTDRIVER_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>

#include <usart.h>
#include <usb.h>
#include <gpio_hal.h>

#define DEFAULT_TX_TIMEOUT 10000

class REVOMINI::REVOMINIUARTDriver : public AP_HAL::UARTDriver  {
public:
    REVOMINIUARTDriver(struct usart_dev *usart, uint8_t use_usb);
  /* REVOMINI implementations of UARTDriver virtual methods */
  void begin(uint32_t b);
  void begin(uint32_t b, uint16_t rxS, uint16_t txS);
  void end();
  void flush();
  bool is_initialized(){ return _initialized; }

  void set_blocking_writes(bool blocking);

  bool tx_pending();


  /* REVOMINI implementations of Stream virtual methods */
  int16_t available();
  int16_t txspace();
  int16_t read();

  /* Empty implementations of Print virtual methods */
  size_t write(uint8_t c);
  size_t write(const uint8_t *buffer, size_t size);

private:

    struct usart_dev *_usart_device;
    uint8_t _usb;
    uint8_t _usb_present;
    bool _initialized;
};

#endif // __AP_HAL_EMPTY_UARTDRIVER_H__
