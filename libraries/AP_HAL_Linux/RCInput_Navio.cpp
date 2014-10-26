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



void LinuxRCInput_Navio::init(void*)
{
    // To be added
}

void LinuxRCInput_Navio::_timer_tick()
{
    // To be added
}

#endif // CONFIG_HAL_BOARD_SUBTYPE
