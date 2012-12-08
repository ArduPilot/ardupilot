/* ************************************************************ */
/* DataFlash_APM1 Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_APM1_H__
#define __DATAFLASH_APM1_H__

#include <AP_Semaphore.h>
#include "DataFlash.h"

class DataFlash_APM1 : public DataFlash_Class
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
    void                    CS_inactive();
    void                    CS_active();
    void                    PageErase (uint16_t PageAdr);
    void                    BlockErase (uint16_t BlockAdr);
    void                    ChipErase(void (*delay_cb)(unsigned long));

    uint8_t           SPI_transfer(uint8_t data);
    AP_Semaphore*           _spi_semaphore;
public:

    DataFlash_APM1(AP_Semaphore* spi_semaphore = NULL) : _spi_semaphore(spi_semaphore) {}
    void        Init();
    void        ReadManufacturerID();
    bool        CardInserted();
};

#endif
