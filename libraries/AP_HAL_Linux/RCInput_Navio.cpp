#include <AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "RCInput_Navio.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

// This is a prototype code for RC input decoding on Raspberry Pi.
// It uses pigpio daemon to sample GPIOs over DMA with 1 microsecond resolution.
// This code should be rewritten to configure DMA GPIO sampling without pigpio.

void LinuxRCInput_Navio::init(void*)
{
    curtick = 0;
    prevtick = 0;
    width_s0 = 0;
    width_s1 = 0;
    
    int error = -1;
    
    // Kills pigpio daemon in case it was run with wrong parameters
    error = unlink("/var/run/pigpio.pid");
    // no error means the process was running and could be stopped
    if(error == 0) {
        error = system("killall pigpiod -q");
        if(error != 0) {
            hal.scheduler->panic("Killing pigpiod failed for RCInput (check permissions).");
        }
    }
    
    // Starts pigpio daemon with 1 microsecond sampling resolution
    error = system("pigpiod -s 1");
    if(error == -1) {
        hal.scheduler->panic("Starting pigpiod failed for RCInput");
    }
    
    // Configures pigpiod to send GPIO change notifications to /dev/pigpio0
    error = system("pigs NO NB 0 0x10");
    if(error == -1) {
        hal.scheduler->panic("Change pigs notification configuration failed for RCInput");
    }
    
    // Configures /dev/pigpio0 for non-blocking read
    pigpio = open("/dev/pigpio0", O_RDONLY);
    if(pigpio == -1) {
        hal.scheduler->panic("No pigpio interface for RCInput");
    }
}

void LinuxRCInput_Navio::_timer_tick()
{
    while (true) {
        int bytesAvailable;
        ioctl(pigpio, FIONREAD, &bytesAvailable);
        
        if (bytesAvailable >= 12)
            ::read(pigpio, reinterpret_cast<uint8_t*>(&gpioReport), 12);
        else
            break;
            
        prevtick = curtick;
        curtick = gpioReport.tick;
        
        if (((gpioReport.level >> 4) & 0x01) == 0) {
            width_s0 = curtick - prevtick;  
        }          
        else {
            width_s1 = curtick - prevtick;
            _process_rc_pulse(width_s0, width_s1);
        }
    } 
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
