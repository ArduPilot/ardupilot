/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       DataFlash_APM2.cpp - DataFlash log library for AT45DB321D
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *       This code works only on ATMega2560. It uses Serial port 3 in SPI MSPI mdoe.
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       Dataflash library for AT45DB321D flash memory
 *       Memory organization : 8192 pages of 512 bytes or 528 bytes
 *
 *       Maximun write bandwidth : 512 bytes in 14ms
 *       This code is written so the master never has to wait to write the data on the eeprom
 *
 *       Methods:
 *               Init() : Library initialization (SPI initialization)
 *               StartWrite(page) : Start a write session. page=start page.
 *               StartRead(page) : Start a read on (page)
 *               GetWritePage() : Returns the last page written to
 *               GetPage() : Returns the last page read
 *
 *       Properties:
 *
 */

#include <AP_HAL.h>               // for removing conflict with optical flow sensor on SPI3 bus
#include "DataFlash_APM2.h"

extern const AP_HAL::HAL& hal;
/*
 * #define ENABLE_FASTSERIAL_DEBUG
 *
 * #ifdef ENABLE_FASTSERIAL_DEBUG
 # define serialDebug(fmt, args...)  if (FastSerial::getInitialized(0)) do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(0); } while(0)
 ##else
 # define serialDebug(fmt, args...)
 ##endif
 #  //*/

#define DF_RESET 41             // RESET  (PG0)
#define DF_CARDDETECT 33        // PC4

// AT45DB321D Commands (from Datasheet)
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


/*
  try to take a semaphore safely from both in a timer and outside
 */
bool DataFlash_APM2::_sem_take(uint8_t timeout)
{
    if (hal.scheduler->in_timerprocess()) {
        return _spi_sem->take_nonblocking();
    }
    return _spi_sem->take(timeout);
}


// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_APM2::Init(void)
{
    // init to zero
    df_NumPages = 0;

    hal.gpio->pinMode(DF_RESET, GPIO_OUTPUT);
    hal.gpio->pinMode(DF_CARDDETECT, GPIO_INPUT);

    // Reset the chip
    hal.gpio->write(DF_RESET,0);
    hal.scheduler->delay(1);
    hal.gpio->write(DF_RESET,1);
   
    _spi = hal.spi->device(AP_HAL::SPIDevice_Dataflash);
    if (_spi == NULL) {
        hal.scheduler->panic(
                PSTR("PANIC: DataFlash SPIDeviceDriver not found"));
        return;
    }
    _spi_sem = _spi->get_semaphore();
    if (_spi_sem == NULL) {
        hal.scheduler->panic(
                PSTR("PANIC: DataFlash SPIDeviceDriver semaphore is null"));
        return; /* never reached */
    }

    if (!_sem_take(5))
        return;
    // get page size: 512 or 528  (by default: 528)
    df_PageSize=PageSize();

    ReadManufacturerID();
    
    _spi_sem->give();

    // see page 22 of the spec for the density code
    uint8_t density_code = (df_device >> 8) & 0x1F;

    // note that we set df_NumPages to one lower than the highest, as
    // the last page is reserved for a config page
    if (density_code == 0x7) {
        // 32 Mbit
        df_NumPages = 8191;
    } else if (density_code == 0x6) {
        // 16 Mbit
        df_NumPages = 4095;
    }

    //serialDebug("density_code %d pages %d, size %d\n", density_code, df_NumPages, df_PageSize);
}

// This function is mainly to test the device
void DataFlash_APM2::ReadManufacturerID()
{
    // activate dataflash command decoder
    _spi->cs_assert();

    // Read manufacturer and ID command...
    _spi->transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);

    df_manufacturer = _spi->transfer(0xff);
    df_device = _spi->transfer(0xff);
    df_device = (df_device<<8) | _spi->transfer(0xff);
    _spi->transfer(0xff);

    // release SPI bus for use by other sensors
    _spi->cs_release();
}

// This function return 1 if Card is inserted on SD slot
bool DataFlash_APM2::CardInserted()
{
    //serialDebug("df_NumPages %d, detect:%d\n", df_NumPages, tmp);
    //return (df_NumPages >= 4095 && digitalRead(DF_CARDDETECT) == 0);
    return (df_NumPages >= 4095);
}

// Read the status register
uint8_t DataFlash_APM2::ReadStatusReg()
{
    uint8_t tmp;

    // activate dataflash command decoder
    _spi->cs_assert();

    // Read status command
    _spi->transfer(DF_STATUS_REGISTER_READ);
    tmp = _spi->transfer(0x00);      // We only want to extract the READY/BUSY bit

    // release SPI bus for use by other sensors
    _spi->cs_release();

    return tmp;
}

// Read the status of the DataFlash
inline
uint8_t DataFlash_APM2::ReadStatus()
{
    return(ReadStatusReg()&0x80);      // We only want to extract the READY/BUSY bit
}

inline
uint16_t DataFlash_APM2::PageSize()
{
    return(528-((ReadStatusReg()&0x01)<<4));      // if first bit 1 trhen 512 else 528 bytes
}

// Wait until DataFlash is in ready state...
void DataFlash_APM2::WaitReady()
{
    while(!ReadStatus()) ;
}

void DataFlash_APM2::PageToBuffer(uint8_t BufferNum, uint16_t PageAdr)
{
    if (!_sem_take(1))
        return;

    // activate dataflash command decoder
    _spi->cs_assert();

    uint8_t cmd[4];
    cmd[0] = BufferNum?DF_TRANSFER_PAGE_TO_BUFFER_2:DF_TRANSFER_PAGE_TO_BUFFER_1;
    if(df_PageSize==512) {
        cmd[1] = (uint8_t)(PageAdr >> 7);
        cmd[2] = (uint8_t)(PageAdr << 1);
    }else{
        cmd[1] = (uint8_t)(PageAdr >> 6);
        cmd[2] = (uint8_t)(PageAdr << 2);
    }
    cmd[3] = 0;
    _spi->transfer(cmd, sizeof(cmd));

    //initiate the transfer
    _spi->cs_release();
    _spi->cs_assert();

    while(!ReadStatus()) ;     //monitor the status register, wait until busy-flag is high

    // release SPI bus for use by other sensors
    _spi->cs_release();
    _spi_sem->give();
}

void DataFlash_APM2::BufferToPage (uint8_t BufferNum, uint16_t PageAdr, uint8_t wait)
{
    if (!_sem_take(1))
        return;
    // activate dataflash command decoder
    _spi->cs_assert();

    uint8_t cmd[4];
    cmd[0] = BufferNum?DF_BUFFER_2_TO_PAGE_WITH_ERASE:DF_BUFFER_1_TO_PAGE_WITH_ERASE;
    if(df_PageSize==512) {
        cmd[1] = (uint8_t)(PageAdr >> 7);
        cmd[2] = (uint8_t)(PageAdr << 1);
    }else{
        cmd[1] = (uint8_t)(PageAdr >> 6);
        cmd[2] = (uint8_t)(PageAdr << 2);
    }
    cmd[3] = 0;
    _spi->transfer(cmd, sizeof(cmd));

    //initiate the transfer
    _spi->cs_release();

    // Check if we need to wait to write the buffer to memory or we can continue...
    if (wait)
        while(!ReadStatus()) ;  //monitor the status register, wait until busy-flag is high

    // release SPI bus for use by other sensors
    _spi_sem->give();
}

void DataFlash_APM2::BlockWrite (uint8_t BufferNum, uint16_t IntPageAdr, 
                                 const void *pHeader, uint8_t hdr_size,
                                 const void *pBuffer, uint16_t size)
{
    if (!_sem_take(1))
        return;
    // activate dataflash command decoder
    _spi->cs_assert();

    uint8_t cmd[] = { 
        (uint8_t)(BufferNum?DF_BUFFER_2_WRITE:DF_BUFFER_1_WRITE), 
        0, 
        (uint8_t)(IntPageAdr>>8), 
        (uint8_t)(IntPageAdr)
    };
    _spi->transfer(cmd, sizeof(cmd));

    // transfer header, if any
    if (hdr_size != 0) {
        _spi->transfer((const uint8_t *)pHeader, hdr_size);
    }

    // transfer data
    _spi->transfer((const uint8_t *)pBuffer, size);

    // release SPI bus for use by other sensors
    _spi->cs_release();
    _spi_sem->give();
}

bool DataFlash_APM2::BlockRead(uint8_t BufferNum, uint16_t IntPageAdr, void *pBuffer, uint16_t size)
{
    if (!_sem_take(1))
        return false;

    // activate dataflash command decoder
    _spi->cs_assert();

    if (BufferNum==0)
        _spi->transfer(DF_BUFFER_1_READ);
    else
        _spi->transfer(DF_BUFFER_2_READ);

    _spi->transfer(0x00);
    _spi->transfer((uint8_t)(IntPageAdr>>8));       //upper part of internal buffer address
    _spi->transfer((uint8_t)(IntPageAdr));                  //lower part of internal buffer address
    _spi->transfer(0x00);                                                                 //don't cares

    uint8_t *pData = (uint8_t *)pBuffer;
    while (size--) {
        *pData++ = _spi->transfer(0x00);
    }

    // release SPI bus for use by other sensors
    _spi->cs_release();

    _spi_sem->give();
    return true;
}

uint8_t DataFlash_APM2::BufferRead (uint8_t BufferNum, uint16_t IntPageAdr)
{
    uint8_t tmp;
    if (!BlockRead(BufferNum, IntPageAdr, &tmp, 1)) {
        return 0;
    }
    return tmp;
}


// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_APM2::PageErase (uint16_t PageAdr)
{
    if (!_sem_take(1))
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
    while(!ReadStatus()) ;

    // release SPI bus for use by other sensors
    _spi_sem->give();
}

// erase a block of 8 pages.
void DataFlash_APM2::BlockErase(uint16_t BlockAdr)
{
    if (!_sem_take(1))
        return;
    // activate dataflash command decoder
    _spi->cs_assert();

    // Send block erase command
    _spi->transfer(DF_BLOCK_ERASE);

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

    // release SPI bus for use by other sensors
    _spi_sem->give();
}


void DataFlash_APM2::ChipErase()
{
    if (!_sem_take(1))
        return;
    //serialDebug("Chip Erase\n");

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
        hal.scheduler->delay(1);
    }

    // release SPI bus for use by other sensors
    _spi_sem->give();
}
