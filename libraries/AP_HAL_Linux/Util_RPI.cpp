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
    _get_board_type_using_peripheral_base ();
}


// 
// previous appraoch was using /proc/device-tree/system/linux,revision
// now we use /proc/device-tree/soc/ranges see: https://forums.raspberrypi.com//viewtopic.php?t=244031
void UtilRPI::_get_board_type_using_peripheral_base() 
{
    FILE *fp;
    uint32_t base=0x00;
    unsigned char buf[32];
    _linux_board_version = LINUX_BOARD_TYPE::UNKNOWN_BOARD;
    fp = fopen("/proc/device-tree/soc/ranges" , "rb");
    if (fp) {
        const uint16_t len = fread(buf, 1, sizeof(buf), fp);
        if (len >= 8) {
            base = buf[4]<<24 | buf[5]<<16 | buf[6]<<8 | buf[7];
            if (!base)
                base = buf[8]<<24 | buf[9]<<16 | buf[10]<<8 | buf[11];
            if (!base)
                base = buf[8]<<24 | buf[9]<<16 | buf[10]<<8 | buf[11];
            if (!base && (len>15))
                base = buf[12]<<24 | buf[13]<<16 | buf[14]<<8 | buf[15];
            
        }
        fclose(fp);
    }

    switch (base) {
        case 0x0:
            _linux_board_version = LINUX_BOARD_TYPE::UNKNOWN_BOARD;
            printf("Cannot detect board-type \r\n");
        break;
        case 0x20000000:
            _linux_board_version = LINUX_BOARD_TYPE::RPI_ZERO_1;
            printf("RPI Zero / 1 \r\n");
            printf("Peripheral base address is %x\n", base);
        break;
        case 0x3f000000:
            _linux_board_version = LINUX_BOARD_TYPE::RPI_2_3_ZERO2;
            printf("RPI 2, 3 or Zero-2 \r\n");
            printf("Peripheral base address is %x\n", base);
        break;
        case 0xfe000000:
            _linux_board_version = LINUX_BOARD_TYPE::RPI_4;
            printf("RPI 4 \r\n");
            printf("Peripheral base address is %x\n", base);
        break;
        case 0x40000000:
            _linux_board_version = LINUX_BOARD_TYPE::ALLWINNWER_H616;
            printf("AllWinner-H616 \r\n");
        break;
    }

    return ;
}

LINUX_BOARD_TYPE UtilRPI::detect_linux_board_type() const
{
    return _linux_board_version;
}

#endif
