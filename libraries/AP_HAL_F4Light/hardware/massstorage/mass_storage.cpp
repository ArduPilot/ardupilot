/*
(c) 2017 night_ghost@ykoctpa.ru
 

*/

#include <AP_HAL_F4Light/AP_HAL_F4Light_Namespace.h>

#include "mass_storage.h"
#include "../sd/SD.h"
#include "../sd/Sd2Card.h"
#include <usb.h>

#include "usbMassStorage.h"

#ifdef USB_MASSSTORAGE

/*
    also see https://habrahabr.ru/post/335018/ and https://habrahabr.ru/post/336968/

*/


uint32_t MAL_massBlockCount[STORAGE_LUN_NBR];
uint32_t MAL_massBlockSize[STORAGE_LUN_NBR];


extern USB_OTG_CORE_HANDLE       USB_OTG_dev;


void MassStorage::setup() const {
    usb_attr_t usb_attr;
    
    usb_open(); 

    usb_default_attr(&usb_attr);
    usb_attr.preempt_prio = USB_INT_PRIORITY;
    usb_attr.sub_prio = 0;
    usb_attr.use_present_pin = 1;
    usb_attr.present_port = PIN_MAP[BOARD_USB_SENSE].gpio_device;
    usb_attr.present_pin =  PIN_MAP[BOARD_USB_SENSE].gpio_bit;

    usb_setParams(&usb_attr);


    SdFatFs *fs = SD.getVolume();

    MAL_massBlockSize[0] = fs->sectorSize();
    MAL_massBlockCount[0] = fs->sectorCount();   // in blocks
    
    SD.sync(); // consistent state

    SCSI_Init(); // start USB IO task

    USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_MSC_desc, &USBD_MSC_cb, &USR_cb);
}


// The following methods are used by the USBMassStorage driver to read and write to the SDCard.  
 
extern "C" {

uint16_t usb_mass_mal_get_status(uint8_t lun){
    return Sd2Card::errorCode();
}


int8_t usb_mass_mal_write_memory(uint8_t lun,uint8_t *writebuff,  uint32_t lba, uint16_t transferLength) {  
  if (lun != 0) {
    return USB_MASS_MAL_FAIL;
  }
  if (Sd2Card::writeBlock(writebuff, lba, transferLength)) {
    return USB_MASS_MAL_SUCCESS;
  }
  return USB_MASS_MAL_FAIL;
}

int8_t usb_mass_mal_read_memory(uint8_t lun, uint8_t *readbuff, uint32_t lba, uint16_t transferLength) {
  if (lun != 0) {
    return USB_MASS_MAL_FAIL;
  }
  if (Sd2Card::readBlock(readbuff, lba, transferLength)) {
    return USB_MASS_MAL_SUCCESS;
  }
  return USB_MASS_MAL_FAIL;
}


} // extern C

#endif
