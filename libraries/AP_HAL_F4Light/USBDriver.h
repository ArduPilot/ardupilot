
#pragma once

#include <AP_HAL_F4Light/AP_HAL_F4Light.h>

#include <gpio_hal.h>
//#include <usb.h> can't include here because defines there conflicts with AP_Math
#include <usart.h>

#define DEFAULT_TX_TIMEOUT 10000

extern "C" {
 extern int usb_open(void);
 extern int usb_close(void);
 uint32_t usb_data_available(void);
 void usb_reset_rx();
}

namespace F4Light {

class USBDriver : public AP_HAL::UARTDriver  {
public:
    USBDriver(bool usb);

    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) {    begin(b); }

    inline void end()   {  usb_close(); } 
    inline bool is_initialized(){ return _initialized; }
    inline void set_blocking_writes(bool blocking) { _blocking=blocking; }
    inline bool tx_pending() {   return false; } // not used

    void flush() { return; };

    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;

    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

private:
    bool _initialized;
    bool _blocking;
};

} // namespace


