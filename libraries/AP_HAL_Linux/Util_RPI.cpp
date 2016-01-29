#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI

#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#include "Util_RPI.h"
#include "Util.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

UtilRPI::UtilRPI()
{
    _check_rpi_version();
}

int UtilRPI::_check_rpi_version()
{
    int hw;
    hw = Util::from(hal.util)->get_hw_arm32();

    if (hw == UTIL_HARDWARE_RPI2) {
        printf("Raspberry Pi 2 with BCM2709!\n");
        _rpi_version = 2;
    } else if (hw == UTIL_HARDWARE_RPI1) {
        printf("Raspberry Pi 1 with BCM2708!\n");
        _rpi_version = 1;
    } else {
        /* defaults to 1 */
        fprintf(stderr, "Could not detect RPi version, defaulting to 1\n");
        _rpi_version = 1;
    }
    return _rpi_version;
}

int UtilRPI::get_rpi_version() const
{
    return _rpi_version;
}

#endif
