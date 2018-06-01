/*
    (c) 2017 night_ghost@ykoctpa.ru
 

 * USBDriver.cpp --- AP_HAL_F4Light USB-UART driver.
 *
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT
#include "USBDriver.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <usb.h>
#include <gpio_hal.h>
#include "Scheduler.h"

using namespace F4Light;

extern const AP_HAL::HAL& hal;


// usb *can* be used  in air, eg. to connect companion computer



USBDriver::USBDriver(bool usb):
    _initialized(false),
    _blocking(false)
{
}

void USBDriver::begin(uint32_t baud) {

/*    _usb_present = gpio_read_bit(PIN_MAP[BOARD_USB_SENSE].gpio_device,PIN_MAP[BOARD_USB_SENSE].gpio_bit); 
   using of this bit prevents USB hotplug
*/

    _initialized = true; // real USB initialization was much earlier
}


uint32_t USBDriver::available() { 
    uint32_t v = usb_data_available();
    return v; 
}

uint32_t USBDriver::txspace() {   return usb_tx_space(); }


int16_t USBDriver::read() {
    if(is_usb_opened() ){
	if (available() == 0)
	    return -1;
	return usb_getc();
    }
    return 0;
}

size_t USBDriver::write(uint8_t c) {
    return write(&c,1);
}


size_t USBDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    uint32_t t = Scheduler::_micros();

    if(is_usb_opened()){
        while(true) {
            uint8_t k=usb_write((uint8_t *)buffer, size);
            size-=k;
            n+=k;
            buffer+=k;
            if(size == 0) break; //done
            
            uint32_t now = Scheduler::_micros();
            if(k==0) {
                if(!_blocking && now - t > 5000 ){        // the waiting time exceeded 5ms - something went wrong ...
                    reset_usb_opened();
                    return n;
                }
                if(!is_usb_opened()) break;
                hal_yield(0);
            } else {
                t = now;
            }
        }
        return n;
    }
    return size;
}

#endif // CONFIG_HAL_BOARD
