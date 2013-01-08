/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       DataFlash_APM1.cpp - DataFlash log library for AT45DB161
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *       This code works only on ATMega2560. It uses Serial port 3 in SPI MSPI mdoe.
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       Dataflash library for AT45DB161D flash memory
 *       Memory organization : 4096 pages of 512 bytes or 528 bytes
 *
 *       Maximun write bandwidth : 512 bytes in 14ms
 *       This code is written so the master never has to wait to write the data on the eeprom
 *
 *       Methods:
 *               Init() : Library initialization (SPI initialization)
 *               StartWrite(page) : Start a write session. page=start page.
 *               WriteByte(data) : Write a byte
 *               WriteInt(data) :  Write an integer (2 bytes)
 *               WriteLong(data) : Write a long (4 bytes)
 *               StartRead(page) : Start a read on (page)
 *               GetWritePage() : Returns the last page written to
 *               GetPage() : Returns the last page read
 *               ReadByte()
 *               ReadInt()
 *               ReadLong()
 *
 *       Properties:
 *
 */
#include <AP_HAL.h>
#include "DataFlash_APM2.h"

extern const AP_HAL::HAL& hal;

//#define ENABLE_FASTSERIAL_DEBUG
#ifdef ENABLE_FASTSERIAL_DEBUG
 #define serialDebug(fmt, args...)  do {hal.console->printf_P(PSTR( __FUNCTION__ ":%d:" fmt "\n"), __LINE__, ##args); } while(0)
#else
 # define serialDebug(fmt, args...)
#endif



// flash size
#define DF_LAST_PAGE 4096

#define DF_RESET 31             // RESET  (PC6)

// AT45DB161D Commands (from Datasheet)
#define DF_TRANSFER_PAGE_TO_BUFFER_1   0x53
#define DF_TRANSFER_PAGE_TO_BUFFER_2   0x55
#define DF_STATUS_REGISTER_READ   0xD7
#define DF_READ_MANUFACTURER_AND_DEVICE_ID   0x9F
#define DF_PAGE_READ   0xD2
#define DF_BUFFER_1_READ   0xD4
#define DF_BUFFER_2_READ   0xD6
#define DF_BUFFER_1_WRITE   0x84
#define DF_BUFFER_2_WRITE   0x87
#define DF_BUFFER_1_TO_PAGE_WITH_ERASE   0x83
#define DF_BUFFER_2_TO_PAGE_WITH_ERASE   0x86
#define DF_PAGE_ERASE   0x81
#define DF_BLOCK_ERASE   0x50
#define DF_SECTOR_ERASE   0x7C
#define DF_CHIP_ERASE_0   0xC7
#define DF_CHIP_ERASE_1   0x94
#define DF_CHIP_ERASE_2   0x80
#define DF_CHIP_ERASE_3   0x9A



// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_APM1::Init(void)
{
    // init to zero
    df_NumPages = 0;

    hal.gpio->pinMode(DF_RESET,GPIO_OUTPUT);
    // Reset the chip
    hal.gpio->write(DF_RESET,0);
    hal.scheduler->delay(1);
    hal.gpio->write(DF_RESET,1);

    _spi = hal.spi->device(AP_HAL::SPIDevice_Dataflash);
    if (_spi == NULL) {
        hal.scheduler->panic(
                PSTR("PANIC: DataFlash SPIDeviceDriver not found"));
        return; /* never reached */
    }

    _spi_sem = _spi->get_semaphore();
    if (_spi_sem == NULL) {
        hal.scheduler->panic(
                PSTR("PANIC: DataFlash SPIDeviceDriver semaphore is null"));
        return; /* never reached */
    }

    // get page size: 512 or 528  (by default: 528)
    df_PageSize = PageSize();

    // the last page is reserved for config information
    df_NumPages = DF_LAST_PAGE - 1;
}

// This function is mainly to test the device
void DataFlash_APM1::ReadManufacturerID()
{
    if (!_spi_sem->take(5))
        return;
    // activate dataflash command decoder
    _spi->cs_assert();

    // Read manufacturer and ID command...
    _spi->transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);

    df_manufacturer = _spi->transfer(0xff);
    df_device = _spi->transfer(0xff);
    df_device = (df_device << 8) | _spi->transfer(0xff);
    _spi->transfer(0xff);

    // release SPI bus for use by other sensors
    _spi->cs_release();

    _spi_sem->give();
}

// This function return 1 if Card is inserted on SD slot
bool DataFlash_APM1::CardInserted()
{
    return true;
}

// Read the status register
// Assumes _spi_sem handled by caller
uint8_t DataFlash_APM1::ReadStatusReg()
{
    uint8_t tmp;

    // activate dataflash command decoder
    _spi->cs_assert();

    // Read status command
    _spi->transfer(DF_STATUS_REGISTER_READ);
    tmp = _spi->transfer(0x00); // We only want to extract the READY/BUSY bit

    // release SPI bus for use by other sensors
    _spi->cs_release();

    return tmp;
}

// Read the status of the DataFlash
// Assumes _spi_sem handled by caller.
inline
uint8_t DataFlash_APM1::ReadStatus()
{
    return(ReadStatusReg()&0x80); // We only want to extract the READY/BUSY bit
}

inline
uint16_t DataFlash_APM1::PageSize()
{
    if (!_spi_sem->take(5))
        return 0;
    
    uint16_t ret = 528-((ReadStatusReg()&0x01) << 4); // if first bit 1 trhen 512 else 528 bytes

    _spi_sem->give();
    return ret;
}

// Wait until DataFlash is in ready state...
// Assumes _spi_sem handled by caller.
void DataFlash_APM1::WaitReady()
{
    while(!ReadStatus()) ;
}

void DataFlash_APM1::PageToBuffer(uint8_t BufferNum, uint16_t PageAdr)
{
    if (!_spi_sem->take(1))
        return;

    // activate dataflash command decoder
    _spi->cs_assert();

    if (BufferNum==1)
        _spi->transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
    else
        _spi->transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);

    if(df_PageSize==512) {
        _spi->transfer((uint8_t)(PageAdr >> 7));
        _spi->transfer((uint8_t)(PageAdr << 1));
    }else{
        _spi->transfer((uint8_t)(PageAdr >> 6));
        _spi->transfer((uint8_t)(PageAdr << 2));
    }
    _spi->transfer(0x00); // don´t care bytes

    //initiate the transfer
    _spi->cs_release();

    while(!ReadStatus()) ;  //monitor the status register, wait until busy-flag is high
    _spi_sem->give();
}

void DataFlash_APM1::BufferToPage (uint8_t BufferNum, uint16_t PageAdr, uint8_t wait)
{
    if (!_spi_sem->take(1))
        return;

    // activate dataflash command decoder
    _spi->cs_assert();

    if (BufferNum==1)
        _spi->transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
    else
        _spi->transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);

    if(df_PageSize==512) {
        _spi->transfer((uint8_t)(PageAdr >> 7));
        _spi->transfer((uint8_t)(PageAdr << 1));
    }else{
        _spi->transfer((uint8_t)(PageAdr >> 6));
        _spi->transfer((uint8_t)(PageAdr << 2));
    }
    _spi->transfer(0x00); // don´t care bytes

    //initiate the transfer
    _spi->cs_release();

    // Check if we need to wait to write the buffer to memory or we can continue...
    if (wait)
        while(!ReadStatus()) ;  //monitor the status register, wait until busy-flag is high
    _spi_sem->give();

}

void DataFlash_APM1::BufferWrite (uint8_t BufferNum, uint16_t IntPageAdr, uint8_t Data)
{
    if (!_spi_sem->take(1))
        return;
    
    // activate dataflash command decoder
    _spi->cs_assert();

    if (BufferNum==1)
        _spi->transfer(DF_BUFFER_1_WRITE);
    else
        _spi->transfer(DF_BUFFER_2_WRITE);

    _spi->transfer(0x00);									// don't care
    _spi->transfer((uint8_t)(IntPageAdr>>8));       // upper part of internal buffer address
    _spi->transfer((uint8_t)(IntPageAdr));          // lower part of internal buffer address
    _spi->transfer(Data);                                 // write data byte

    // release SPI bus for use by other sensors
    _spi->cs_release();
    _spi_sem->give();
}

uint8_t DataFlash_APM1::BufferRead (uint8_t BufferNum, uint16_t IntPageAdr)
{
    uint8_t tmp;

    if (!_spi_sem->take(1))
        return 0;

    // activate dataflash command decoder
    _spi->cs_assert();

    if (BufferNum==1)
        _spi->transfer(DF_BUFFER_1_READ);
    else
        _spi->transfer(DF_BUFFER_2_READ);

    _spi->transfer(0x00);
    _spi->transfer((uint8_t)(IntPageAdr>>8)); 		// upper part of internal buffer address
    _spi->transfer((uint8_t)(IntPageAdr));   		// lower part of internal buffer address
    _spi->transfer(0x00);                          		// don't cares
    tmp = _spi->transfer(0x00);                    		// read data byte

    // release SPI bus for use by other sensors
    _spi->cs_release();

    _spi_sem->give();
    return (tmp);
}
// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_APM1::PageErase (uint16_t PageAdr)
{
    if (!_spi_sem->take(1))
        return;

    // activate dataflash command decoder
    _spi->cs_assert();

    // Send page erase command
    _spi->transfer(DF_PAGE_ERASE);

    if(df_PageSize==512) {
        _spi->transfer((uint8_t)(PageAdr >> 7));
        _spi->transfer((uint8_t)(PageAdr << 1));
    }else{
        _spi->transfer((uint8_t)(PageAdr >> 6));
        _spi->transfer((uint8_t)(PageAdr << 2));
    }

    _spi->transfer(0x00);

    //initiate flash page erase
    _spi->cs_release();

    _spi_sem->give();
    while(!ReadStatus()) ;
}

void DataFlash_APM1::BlockErase (uint16_t BlockAdr)
{
    if (!_spi_sem->take(1))
        return;

    // activate dataflash command decoder
    _spi->cs_assert();

    // Send block erase command
    _spi->transfer(DF_BLOCK_ERASE);

	/*
    if (df_PageSize==512) {
        _spi->transfer((uint8_t)(BlockAdr >> 3));
        _spi->transfer((uint8_t)(BlockAdr << 5));
    } else {
        _spi->transfer((uint8_t)(BlockAdr >> 4));
        _spi->transfer((uint8_t)(BlockAdr << 4));
    }*/

    if (df_PageSize==512) {
        _spi->transfer((uint8_t)(BlockAdr >> 4));
        _spi->transfer((uint8_t)(BlockAdr << 4));
    } else {
        _spi->transfer((uint8_t)(BlockAdr >> 3));
        _spi->transfer((uint8_t)(BlockAdr << 5));
    }

    _spi->transfer(0x00);
	//serialDebug("BL Erase, %d\n", BlockAdr);

    //initiate flash page erase
    _spi->cs_release();
    while(!ReadStatus()) ;
    _spi_sem->give();
}


void DataFlash_APM1::ChipErase()
{
    if (!_spi_sem->take(5))
        return;

    // activate dataflash command decoder
    _spi->cs_assert();

    // opcodes for chip erase
    _spi->transfer(DF_CHIP_ERASE_0);
    _spi->transfer(DF_CHIP_ERASE_1);
    _spi->transfer(DF_CHIP_ERASE_2);
    _spi->transfer(DF_CHIP_ERASE_3);

    //initiate flash page erase
    _spi->cs_release();

    while(!ReadStatus()) {
        hal.scheduler->delay(6);
    }
    
    _spi_sem->give();
    
}
