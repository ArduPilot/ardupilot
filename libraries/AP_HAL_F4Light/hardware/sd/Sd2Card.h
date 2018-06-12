/*

    a low-level interface for SD card driver
    
 */
#ifndef Sd2Card_h
#define Sd2Card_h

#include "FatFs/drivers/sd.h"
#include "FatFs/diskio.h"

#include <AP_HAL/AP_HAL.h>


#if defined(BOARD_SDCARD_CS_PIN) || defined(BOARD_DATAFLASH_FATFS)

#include <AP_HAL_F4Light/AP_HAL_F4Light.h>
#include <AP_HAL_F4Light/SPIDevice.h>
#include <AP_HAL_F4Light/Scheduler.h>
#include <AP_HAL_F4Light/handler.h>

#include "FatFs/diskio.h"

using namespace F4Light;

#define   FALSE      ((uint8_t)0x00)
#define   TRUE       ((uint8_t)0x01)

// card types to match Arduino definition
/** Standard capacity V1 SD card */
#define SD_CARD_TYPE_SD1	CT_SD1
/** Standard capacity V2 SD card */
#define SD_CARD_TYPE_SD2	CT_SD2
/** High Capacity SD card */
#define SD_CARD_TYPE_SDHC	CT_BLOCK




extern "C" {
    void    spi_spiSend(uint8_t b);
    uint8_t spi_spiRecv(void);
    uint8_t spi_spiXchg(uint8_t b);
    void spi_spiTransfer(const uint8_t *send, uint32_t send_len,  uint8_t *recv, uint32_t recv_len);
    void spi_chipSelectHigh(void);
    bool spi_chipSelectLow(bool take_sem);
    void spi_yield();
    uint8_t spi_waitFor(uint8_t out, spi_WaitFunc cb, uint32_t dly);
    uint8_t spi_detect();
    uint32_t get_fattime();
};


class Sd2Card {
public:

    uint8_t init(AP_HAL::OwnPtr<F4Light::SPIDevice> spi);

    /** Return the card type: SD V1, SD V2 or SDHC */
    static uint8_t type(void) { return sd_get_type(); }

    static uint16_t errorCode() { return sd_status(); }
    static uint8_t writeBlock(uint8_t *buff, uint32_t block) {               return sd_write(buff, block, 1)==RES_OK; }
    static uint8_t readBlock(uint8_t *buff, uint32_t block){                 return sd_read( buff, block, 1)==RES_OK; }

    static uint8_t writeBlock(uint8_t *buff, uint32_t block, uint16_t len) { return sd_write(buff, block, len)==RES_OK; }
    static uint8_t readBlock(uint8_t *buff, uint32_t block, uint16_t len){   return sd_read( buff, block, len)==RES_OK; }
    
    static uint8_t ioctl(uint32_t cmd, uint32_t *buff){   return sd_ioctl(cmd, buff) == RES_OK; }

    static uint32_t sectorCount() { // full number of sectors
        uint32_t sz;
        if(sd_ioctl(GET_SECTOR_COUNT, &sz) == RES_OK)
            return sz;
        return 0;

    }

    static uint32_t blockSize() { // sectors in erase block
        uint32_t sz;
        if(sd_ioctl(GET_BLOCK_SIZE, &sz) == RES_OK)
            return sz; 
#ifdef BOARD_DATAFLASH_ERASE_SIZE
        return BOARD_DATAFLASH_ERASE_SIZE/512;
#else
        return 8;
#endif

    }

    static uint32_t sectorSize() { // sector size in bytes
        uint32_t sz;
        if(sd_ioctl(GET_SECTOR_SIZE, &sz) == RES_OK)
            return sz;
            
        return 512;
    }

private:
    void _timer(void) { sd_timerproc(); }

};

#endif // revomini

#endif  // sd2Card_h
