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

extern "C" {
// AVR LibC Includes
#include <inttypes.h>
#include <avr/interrupt.h>
}
#include <FastSerial.h>
#include <SPI.h>

#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WConstants.h"
#endif

#include <AP_Semaphore.h>               // for removing conflict with optical flow sensor on SPI3 bus
#include "DataFlash_APM2.h"

///*
#define ENABLE_FASTSERIAL_DEBUG
#ifdef ENABLE_FASTSERIAL_DEBUG
 # define serialDebug(fmt, args...)  if (FastSerial::getInitialized(0)) do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(0); } while(0)
#else
 # define serialDebug(fmt, args...)
#endif
//*/



// flash size
#define DF_LAST_PAGE 4096

// arduino mega SPI pins
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(DESKTOP_BUILD)
 #define DF_DATAOUT 51              // MOSI
 #define DF_DATAIN  50              // MISO
 #define DF_SPICLOCK  52            // SCK
 #define DF_SLAVESELECT 53          // SS     (PB0)
 #define DF_RESET 31             // RESET  (PC6)
#else  // normal arduino SPI pins...
 #define DF_DATAOUT 11           //MOSI
 #define DF_DATAIN  12           //MISO
 #define DF_SPICLOCK  13         //SCK
 #define DF_SLAVESELECT 10       //SS
#endif

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


// *** INTERNAL FUNCTIONS ***

unsigned char DataFlash_APM1::SPI_transfer(unsigned char data)
{
    unsigned char retval;

    // get spi semaphore if required.  if failed to get semaphore then
    // just quietly fail
    if ( _spi_semaphore != NULL) {
        if( !_spi_semaphore->get(this) ) {
            return 0;
        }
    }

    // send the data
    retval = SPI.transfer(data);

    // release spi3 semaphore
    if ( _spi_semaphore != NULL) {
        _spi_semaphore->release(this);
    }

    return retval;
}

// disable device
void DataFlash_APM1::CS_inactive()
{
    digitalWrite(DF_SLAVESELECT,HIGH);
}

// enable device
void DataFlash_APM1::CS_active()
{
    digitalWrite(DF_SLAVESELECT,LOW);
}

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_APM1::Init(void)
{
    // init to zero
    df_NumPages = 0;

    pinMode(DF_DATAOUT, OUTPUT);
    pinMode(DF_DATAIN, INPUT);
    pinMode(DF_SPICLOCK,OUTPUT);
    pinMode(DF_SLAVESELECT,OUTPUT);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(DESKTOP_BUILD)
    pinMode(DF_RESET,OUTPUT);
    // Reset the chip
    digitalWrite(DF_RESET,LOW);
    delay(1);
    digitalWrite(DF_RESET,HIGH);
#endif

    // disable device
    CS_inactive();

    // Setup SPI  Master, Mode 3, fosc/4 = 4MHz
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3);
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    // get page size: 512 or 528  (by default: 528)
    df_PageSize = PageSize();

    // the last page is reserved for config information
    df_NumPages = DF_LAST_PAGE - 1;
}

// This function is mainly to test the device
void DataFlash_APM1::ReadManufacturerID()
{
    // activate dataflash command decoder
    CS_active();

    // Read manufacturer and ID command...
    SPI_transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);

    df_manufacturer = SPI_transfer(0xff);
    df_device = SPI_transfer(0xff);
    df_device = (df_device << 8) | SPI_transfer(0xff);
    SPI_transfer(0xff);

    // release SPI bus for use by other sensors
    CS_inactive();
}

// This function return 1 if Card is inserted on SD slot
bool DataFlash_APM1::CardInserted()
{
    return true;
}

// Read the status register
byte DataFlash_APM1::ReadStatusReg()
{
    byte tmp;

    // activate dataflash command decoder
    CS_active();

    // Read status command
    SPI_transfer(DF_STATUS_REGISTER_READ);
    tmp = SPI_transfer(0x00); // We only want to extract the READY/BUSY bit

    // release SPI bus for use by other sensors
    CS_inactive();

    return tmp;
}

// Read the status of the DataFlash
inline
byte DataFlash_APM1::ReadStatus()
{
    return(ReadStatusReg()&0x80); // We only want to extract the READY/BUSY bit
}

inline
uint16_t DataFlash_APM1::PageSize()
{
    return(528-((ReadStatusReg()&0x01) << 4)); // if first bit 1 trhen 512 else 528 bytes
}

// Wait until DataFlash is in ready state...
void DataFlash_APM1::WaitReady()
{
    while(!ReadStatus()) ;
}

void DataFlash_APM1::PageToBuffer(unsigned char BufferNum, uint16_t PageAdr)
{
    // activate dataflash command decoder
    CS_active();

    if (BufferNum==1)
        SPI_transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
    else
        SPI_transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);

    if(df_PageSize==512) {
        SPI_transfer((unsigned char)(PageAdr >> 7));
        SPI_transfer((unsigned char)(PageAdr << 1));
    }else{
        SPI_transfer((unsigned char)(PageAdr >> 6));
        SPI_transfer((unsigned char)(PageAdr << 2));
    }
    SPI_transfer(0x00); // don´t care bytes

    //initiate the transfer
    CS_inactive();
    CS_active();

    while(!ReadStatus()) ;  //monitor the status register, wait until busy-flag is high

    // release SPI bus for use by other sensors
    CS_inactive();
}

void DataFlash_APM1::BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait)
{
    // activate dataflash command decoder
    CS_active();

    if (BufferNum==1)
        SPI_transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
    else
        SPI_transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);

    if(df_PageSize==512) {
        SPI_transfer((unsigned char)(PageAdr >> 7));
        SPI_transfer((unsigned char)(PageAdr << 1));
    }else{
        SPI_transfer((unsigned char)(PageAdr >> 6));
        SPI_transfer((unsigned char)(PageAdr << 2));
    }
    SPI_transfer(0x00); // don´t care bytes

    //initiate the transfer
    CS_inactive();
    CS_active();

    // Check if we need to wait to write the buffer to memory or we can continue...
    if (wait)
        while(!ReadStatus()) ;  //monitor the status register, wait until busy-flag is high

    // release SPI bus for use by other sensors
    CS_inactive();
}

void DataFlash_APM1::BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data)
{
    // activate dataflash command decoder
    CS_active();

    if (BufferNum==1)
        SPI_transfer(DF_BUFFER_1_WRITE);
    else
        SPI_transfer(DF_BUFFER_2_WRITE);

    SPI_transfer(0x00);									// don't care
    SPI_transfer((unsigned char)(IntPageAdr>>8));       // upper part of internal buffer address
    SPI_transfer((unsigned char)(IntPageAdr));          // lower part of internal buffer address
    SPI_transfer(Data);                                 // write data byte

    // release SPI bus for use by other sensors
    CS_inactive();
}

unsigned char DataFlash_APM1::BufferRead (unsigned char BufferNum, uint16_t IntPageAdr)
{
    byte tmp;

    // activate dataflash command decoder
    CS_active();

    if (BufferNum==1)
        SPI_transfer(DF_BUFFER_1_READ);
    else
        SPI_transfer(DF_BUFFER_2_READ);

    SPI_transfer(0x00);
    SPI_transfer((unsigned char)(IntPageAdr>>8)); 		// upper part of internal buffer address
    SPI_transfer((unsigned char)(IntPageAdr));   		// lower part of internal buffer address
    SPI_transfer(0x00);                          		// don't cares
    tmp = SPI_transfer(0x00);                    		// read data byte

    // release SPI bus for use by other sensors
    CS_inactive();

    return (tmp);
}
// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_APM1::PageErase (uint16_t PageAdr)
{
    // activate dataflash command decoder
    CS_active();

    // Send page erase command
    SPI_transfer(DF_PAGE_ERASE);

    if(df_PageSize==512) {
        SPI_transfer((unsigned char)(PageAdr >> 7));
        SPI_transfer((unsigned char)(PageAdr << 1));
    }else{
        SPI_transfer((unsigned char)(PageAdr >> 6));
        SPI_transfer((unsigned char)(PageAdr << 2));
    }

    SPI_transfer(0x00);

    //initiate flash page erase
    CS_inactive();
    CS_active();
    while(!ReadStatus()) ;

    // release SPI bus for use by other sensors
    CS_inactive();
}

void DataFlash_APM1::BlockErase (uint16_t BlockAdr)
{
    // activate dataflash command decoder
    CS_active();

    // Send block erase command
    SPI_transfer(DF_BLOCK_ERASE);

	/*
    if (df_PageSize==512) {
        SPI_transfer((unsigned char)(BlockAdr >> 3));
        SPI_transfer((unsigned char)(BlockAdr << 5));
    } else {
        SPI_transfer((unsigned char)(BlockAdr >> 4));
        SPI_transfer((unsigned char)(BlockAdr << 4));
    }*/

    if (df_PageSize==512) {
        SPI_transfer((unsigned char)(BlockAdr >> 4));
        SPI_transfer((unsigned char)(BlockAdr << 4));
    } else {
        SPI_transfer((unsigned char)(BlockAdr >> 3));
        SPI_transfer((unsigned char)(BlockAdr << 5));
    }

    SPI_transfer(0x00);
	serialDebug("BL Erase, %d\n", BlockAdr);

    //initiate flash page erase
    CS_inactive();
    CS_active();
    while(!ReadStatus()) ;

    // release SPI bus for use by other sensors
    CS_inactive();
}


void DataFlash_APM1::ChipErase(void (*delay_cb)(unsigned long))
{
    //serialDebug("Chip Erase\n");

    // activate dataflash command decoder
    CS_active();

    // opcodes for chip erase
    SPI_transfer(DF_CHIP_ERASE_0);
    SPI_transfer(DF_CHIP_ERASE_1);
    SPI_transfer(DF_CHIP_ERASE_2);
    SPI_transfer(DF_CHIP_ERASE_3);

    //initiate flash page erase
    CS_inactive();
    CS_active();

    while(!ReadStatus()) {
        delay_cb(1);
    }

    // release SPI bus for use by other sensors
    CS_inactive();
}
