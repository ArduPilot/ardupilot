#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

#include "Util.h"
using namespace Linux;

/**
   return commandline arguments, if available
*/
void LinuxUtil::commandline_arguments(uint8_t &argc, char * const *&argv)
{
    argc = saved_argc;
    argv = saved_argv;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX
