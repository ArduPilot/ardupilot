#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OBAL_V1 

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "Util.h"
#include "Util_RPI.h"

extern const AP_HAL::HAL &hal;

using namespace Linux;

UtilRPI::UtilRPI()
{
    _check_rpi_version();
}

int UtilRPI::_check_rpi_version()
{
    const unsigned int MAX_SIZE_LINE = 50;
    char buffer[MAX_SIZE_LINE];
    int hw;

    memset(buffer, 0, MAX_SIZE_LINE);
    FILE *f = fopen("/sys/firmware/devicetree/base/model", "r");
    if (f != nullptr && fgets(buffer, MAX_SIZE_LINE, f) != nullptr) {
        fclose(f);
        
        int ret = strncmp(buffer, "Raspberry Pi Compute Module 4", 28);
        if (ret == 0) {
             _rpi_version = 4; // compute module 4 e.g. Raspberry Pi Compute Module 4 Rev 1.0.
             printf("%s. (intern: %d)\n", buffer, _rpi_version);
             return _rpi_version;
        }
        
        ret = strncmp(buffer, "Raspberry Pi Zero 2", 19);
        if (ret == 0) {
             _rpi_version = 2; // Raspberry PI Zero 2 W e.g. Raspberry Pi Zero 2 Rev 1.0.
             printf("%s. (intern: %d)\n", buffer, _rpi_version);
             return _rpi_version;
        }
        
        ret = sscanf(buffer + 12, "%d", &_rpi_version);
        if (ret != EOF) {
            if (_rpi_version > 3)  {
                _rpi_version = 4;
            } else if (_rpi_version > 2) {
                // Preserving old behavior.
                _rpi_version = 2;
            } else if (_rpi_version == 0) {
                // RPi 1 doesn't have a number there, so sscanf() won't have read anything.
                _rpi_version = 1;
            }

            printf("%s. (intern: %d)\n", buffer, _rpi_version);

            return _rpi_version;
        }
    }

    // Attempting old method if the version couldn't be read with the new one.
    hw = Util::from(hal.util)->get_hw_arm32();

    if (hw == UTIL_HARDWARE_RPI4) {
        printf("Raspberry Pi 4 with BCM2711!\n");
        _rpi_version = 4;
    } else if (hw == UTIL_HARDWARE_RPI2) {
        printf("Raspberry Pi 2/3 with BCM2709!\n");
        _rpi_version = 2;
    } else if (hw == UTIL_HARDWARE_RPI1) {
        printf("Raspberry Pi 1 with BCM2708!\n");
        _rpi_version = 1;
    } else {
        /* defaults to RPi version 2/3 */
        fprintf(stderr, "Could not detect RPi version, defaulting to 2/3\n");
        _rpi_version = 2;
    }
    return _rpi_version;
}

int UtilRPI::get_rpi_version() const
{
    return _rpi_version;
}

#endif
