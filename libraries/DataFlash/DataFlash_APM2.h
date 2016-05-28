/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* DataFlash_APM2 Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_APM2_H__
#define __DATAFLASH_APM2_H__

#include <AP_HAL.h>
#include "DataFlash.h"

class DataFlash_APM2 : public DataFlash_Block
{
private:
    //Methods
    void                    BufferToPage (uint8_t BufferNum, uint16_t PageAdr, uint8_t wait);
    void                    PageToBuffer(uint8_t BufferNum, uint16_t PageAdr);
    void                    WaitReady();
    uint8_t	            ReadStatusReg();
    uint8_t     	    ReadStatus();
    uint16_t                PageSize();

    // write size bytes of data to a page. The caller must ensure that
    // the data fits within the page, otherwise it will wrap to the
    // start of the page
    // If pHeader is not NULL then write the header bytes before the data
    void		    BlockWrite(uint8_t BufferNum, uint16_t IntPageAdr, 
				       const void *pHeader, uint8_t hdr_size,
				       const void *pBuffer, uint16_t size);

    // read size bytes of data to a page. The caller must ensure that
    // the data fits within the page, otherwise it will wrap to the
    // start of the page
    bool 		    BlockRead(uint8_t BufferNum, uint16_t IntPageAdr, void *pBuffer, uint16_t size);

    void                    PageErase (uint16_t PageAdr);
    void                    BlockErase (uint16_t BlockAdr);
    void                    ChipErase();

    // take a semaphore safely
    bool		            _sem_take(uint8_t timeout);

    AP_HAL::SPIDeviceDriver* _spi;
    AP_HAL::Semaphore* _spi_sem;

public:
    void        Init(const struct LogStructure *structure, uint8_t num_types);
    void        ReadManufacturerID();
    bool        CardInserted();
};

#endif
