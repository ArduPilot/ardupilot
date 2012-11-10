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
    unsigned char           BufferRead (unsigned char BufferNum, uint16_t IntPageAdr);
    void                    BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data);
    void                    BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait);
    void                    PageToBuffer(unsigned char BufferNum, uint16_t PageAdr);
    void                    WaitReady();
    unsigned char           ReadStatusReg();
    unsigned char           ReadStatus();
    uint16_t                PageSize();
    void                    CS_inactive();
    void                    CS_active();
    void                    PageErase (uint16_t PageAdr);
    void                    BlockErase (uint16_t BlockAdr);
    void                    ChipErase(void (*delay_cb)(unsigned long));

    unsigned char           SPI_transfer(unsigned char data);
    AP_Semaphore*           _spi_semaphore;
public:

    DataFlash_APM1(AP_Semaphore* spi_semaphore = NULL) : _spi_semaphore(spi_semaphore) {}
    void        Init();
    void        ReadManufacturerID();
    bool        CardInserted();
};

#endif
