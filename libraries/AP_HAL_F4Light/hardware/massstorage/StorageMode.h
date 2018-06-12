#ifndef STORAGE_MODE_H
#define STORAGE_MODE_H

#include "msc/usbd_msc_mem.h"
#include <usb_dcd_int.h>

#include "usb_mass_mal.h"

int8_t STORAGE_Init(uint8_t lun);
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint32_t *block_size);
int8_t STORAGE_IsReady(uint8_t lun);
int8_t STORAGE_IsWriteProtected(uint8_t lun);
int8_t STORAGE_GetMaxLun(void);
void OTG_FS_WKUP_IRQHandler(void);

int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t STORAGE_Write(uint8_t lun,uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);


#endif
