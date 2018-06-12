#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI

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

    FILE *f = fopen("/sys/firmware/devicetree/base/model", "r");
    if (f != nullptr && fgets(buffer, MAX_SIZE_LINE, f) != nullptr) {
        int ret = sscanf(buffer + 12, "%d", &_rpi_version);
        fclose(f);
        if (ret != EOF) {

            if (_rpi_version > 2) {
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

    if (hw == UTIL_HARDWARE_RPI2) {
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
