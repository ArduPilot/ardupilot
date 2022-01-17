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
    _check_rpi_version_by_rev();
}

int UtilRPI::_check_rpi_version_by_rev()
{
    // assume 2 if unknown
    _rpi_version = 2;
    const char *SOC[]= {"Broadcom BCM2835","Broadcom BCM2836","Broadcom BCM2837","Broadcom BCM2711"};
    const char *revision_file_ = "/proc/device-tree/system/linux,revision";
    uint8_t revision[4] = { 0 };
    uint32_t cpu = 0;
    FILE *fd;

    if ((fd = fopen(revision_file_, "rb")) == NULL) {
        printf("Can't open '%s'\n", revision_file_);
    }
    else {
        if (fread(revision, 1, sizeof(revision), fd) == 4) {
            cpu = (revision[2] >> 4) & 0xf;
            
            _rpi_version = cpu;
            
            if (_rpi_version==0) {
                _rpi_version=1; //RPI-Zero
            }

            printf("SOC: %s    use intern: %d \r\n",SOC[cpu], _rpi_version);
        }
        else {
            printf("Revision data too short\n");
        }
        fclose(fd);
    }

    return _rpi_version;
}


int UtilRPI::get_rpi_version() const
{
    return _rpi_version;
}

#endif
