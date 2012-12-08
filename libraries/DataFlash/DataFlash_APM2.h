/* ************************************************************ */
/* DataFlash_APM2 Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_APM2_H__
#define __DATAFLASH_APM2_H__

#include <AP_Semaphore.h>
#include "DataFlash.h"

class DataFlash_APM2 : public DataFlash_Class
{
private:
    //Methods
    uint8_t           BufferRead (uint8_t BufferNum, uint16_t IntPageAdr);
    void                    BufferWrite (uint8_t BufferNum, uint16_t IntPageAdr, uint8_t Data);
    void                    BufferToPage (uint8_t BufferNum, uint16_t PageAdr, uint8_t wait);
    void                    PageToBuffer(uint8_t BufferNum, uint16_t PageAdr);
    void                    WaitReady();
    uint8_t           ReadStatusReg();
    uint8_t           ReadStatus();
    uint16_t                PageSize();

    uint8_t           SPI_transfer(uint8_t data);
    void                    CS_inactive();
    void                    CS_active();
    void                    PageErase (uint16_t PageAdr);
    void                    BlockErase (uint16_t BlockAdr);
    void                    ChipErase(void (*delay_cb)(unsigned long));

    AP_Semaphore*           _spi3_semaphore;
public:
    DataFlash_APM2(AP_Semaphore* spi3_semaphore = NULL) : _spi3_semaphore(spi3_semaphore) {}

    void        Init();
    void        ReadManufacturerID();
    bool        CardInserted();
};

#endif
