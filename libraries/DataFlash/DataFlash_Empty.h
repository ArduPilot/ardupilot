/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* DataFlash_EMPTY Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_EMPTY_H__
#define __DATAFLASH_EMPTY_H__

#include <AP_HAL.h>
#include "DataFlash.h"

class DataFlash_Empty : public DataFlash_Block
{
private:
    //Methods
    uint8_t           BufferRead (uint8_t BufferNum, uint16_t IntPageAdr);
    void              BufferWrite (uint8_t BufferNum, uint16_t IntPageAdr, uint8_t Data);
    void              BufferToPage (uint8_t BufferNum, uint16_t PageAdr, uint8_t wait);
    void              PageToBuffer(uint8_t BufferNum, uint16_t PageAdr);
    void              WaitReady();
    uint8_t           ReadStatusReg();
    uint8_t           ReadStatus();
    uint16_t          PageSize();
    void              PageErase (uint16_t PageAdr);
    void              BlockErase (uint16_t BlockAdr);
    void              ChipErase();
    
    void BlockWrite(uint8_t BufferNum, uint16_t IntPageAdr, 
                    const void *pHeader, uint8_t hdr_size,
                    const void *pBuffer, uint16_t size);
    bool BlockRead(uint8_t BufferNum, uint16_t IntPageAdr, void *pBuffer, uint16_t size);

public:
    void        Init();
    void        ReadManufacturerID();
    bool        CardInserted();
};

#endif // __DATAFLASH_EMPTY_H__
