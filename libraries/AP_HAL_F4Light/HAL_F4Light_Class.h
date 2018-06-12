
#pragma once


#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT

#include <AP_HAL_F4Light/AP_HAL_F4Light.h>
#include "handler.h"

#include <AP_Param/AP_Param.h>

#include <hal.h>
#include "USBDriver.h"
#include <pwm_in.h>
#include <usart.h>
#include <i2c.h>
#include <spi.h>


#if defined(USB_MASSSTORAGE)
#include "massstorage/mass_storage.h"
#endif

/*
Backup SRAM 4KByte - 0x4002 4000 - 0x4002 3FFF can be used as EEPROM buffer?

ideally it should be used for state storage for recovery from in-flight reboots but we never get this from upstream...

*/


class HAL_state{ 
public:
    HAL_state()
     : sd_busy(0)
     , disconnect(0)
    {}

    uint8_t sd_busy;
    uint8_t disconnect;
};

class HAL_F4Light : public AP_HAL::HAL {
public:
    HAL_F4Light();
    void run(int argc, char * const argv[], Callbacks* callbacks) const override;

    void lateInit();

    static HAL_state state; // hal is const so we should move out its internal state
    
private:
    AP_HAL::UARTDriver** uarts[6];

    void connect_uart(AP_HAL::UARTDriver* uartL,AP_HAL::UARTDriver* uartR, AP_HAL::Proc proc);
    // parameters in hal_param_helper

#if defined(USB_MASSSTORAGE)
    F4Light::MassStorage massstorage;
#endif

};


#endif // __AP_HAL_F4Light_CLASS_H__
