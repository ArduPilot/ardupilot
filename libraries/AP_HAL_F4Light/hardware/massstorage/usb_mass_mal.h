#ifndef __USB_MASS_MAL_H
#define __USB_MASS_MAL_H

#include <stdint.h>

#define STORAGE_LUN_NBR 1

#ifdef __cplusplus
extern "C" {
#endif

    extern uint32_t MAL_massBlockCount[STORAGE_LUN_NBR];
    extern uint32_t MAL_massBlockSize[STORAGE_LUN_NBR];

    uint16_t usb_mass_mal_get_status(uint8_t lun);
    int8_t usb_mass_mal_read_memory(uint8_t lun, uint8_t *readbuff, uint32_t memoryOffset, uint16_t transferLength);
    int8_t usb_mass_mal_write_memory(uint8_t lun, uint8_t *writebuff, uint32_t memoryOffset, uint16_t transferLength);

    void usb_mass_mal_USBdisconnect();

#ifdef __cplusplus
}
#endif

#endif
