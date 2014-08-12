/* ************************************************************ */
/* DataFlash_SITL Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_SITL_H__
#define __DATAFLASH_SITL_H__

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL.h>
#include "DataFlash.h"

class DataFlash_SITL : public DataFlash_Block
{
private:
    //Methods
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
    
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
public:

    DataFlash_SITL() {}
    void        Init(const struct LogStructure *structure, uint8_t num_types);
    void        ReadManufacturerID();
    bool        CardInserted();
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#endif // __DATAFLASH_SITL_H__
