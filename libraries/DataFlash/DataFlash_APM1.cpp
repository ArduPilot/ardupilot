/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       DataFlash_APM1.cpp - DataFlash log library for AT45DB161
 *       Code by Jordi Munoz and Jose Julio. DIYDrones.com
 *       This code works with boards based on ATMega168/328 and ATMega1280/2560 using SPI port
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

#include <stdint.h>
#include "DataFlash.h"
#include <SPI.h>

// flash size
#define DF_LAST_PAGE 4096

// arduino mega SPI pins
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
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

    // get spi semaphore if required.  if failed to get semaphore then just quietly fail
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


void dataflash_CS_inactive()
{
    digitalWrite(DF_SLAVESELECT,HIGH); //disable device
}

void dataflash_CS_active()
{
    digitalWrite(DF_SLAVESELECT,LOW); //enable device
}

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_APM1::Init(void)
{
    pinMode(DF_DATAOUT, OUTPUT);
    pinMode(DF_DATAIN, INPUT);
    pinMode(DF_SPICLOCK,OUTPUT);
    pinMode(DF_SLAVESELECT,OUTPUT);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    pinMode(DF_RESET,OUTPUT);
    // Reset the chip
    digitalWrite(DF_RESET,LOW);
    delay(1);
    digitalWrite(DF_RESET,HIGH);
#endif

    dataflash_CS_inactive();   //disable device

    // Setup SPI  Master, Mode 3, fosc/4 = 4MHz
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3);
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    // get page size: 512 or 528
    df_PageSize=PageSize();

    // the last page is reserved for config information
    df_NumPages = DF_LAST_PAGE - 1;
}

// This function is mainly to test the device
void DataFlash_APM1::ReadManufacturerID()
{
    dataflash_CS_active();   // activate dataflash command decoder

    // Read manufacturer and ID command...
    SPI_transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);

    df_manufacturer = SPI_transfer(0xff);
    df_device = SPI_transfer(0xff);
    df_device = (df_device<<8) | SPI_transfer(0xff);
    SPI_transfer(0xff);

    dataflash_CS_inactive();  // Reset dataflash command decoder
}


bool DataFlash_APM1::CardInserted(void)
{
    return true;
}

// Read the status register
byte DataFlash_APM1::ReadStatusReg()
{
    byte tmp;

    dataflash_CS_active();   // activate dataflash command decoder

    // Read status command
    SPI_transfer(DF_STATUS_REGISTER_READ);
    tmp = SPI_transfer(0x00); // We only want to extract the READY/BUSY bit

    dataflash_CS_inactive();  // Reset dataflash command decoder

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
    return(528-((ReadStatusReg()&0x01)<<4)); // if first bit 1 trhen 512 else 528 bytes
}


// Wait until DataFlash is in ready state...
void DataFlash_APM1::WaitReady()
{
    while(!ReadStatus()) ;
}

void DataFlash_APM1::PageToBuffer(unsigned char BufferNum, uint16_t PageAdr)
{
    dataflash_CS_active();   // activate dataflash command decoder

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

    dataflash_CS_inactive();    //initiate the transfer
    dataflash_CS_active();

    while(!ReadStatus()) ;  //monitor the status register, wait until busy-flag is high

    dataflash_CS_inactive();

}

void DataFlash_APM1::BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait)
{
    dataflash_CS_active();   // activate dataflash command decoder

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

    dataflash_CS_inactive();    //initiate the transfer
    dataflash_CS_active();

    // Check if we need to wait to write the buffer to memory or we can continue...
    if (wait)
        while(!ReadStatus()) ;  //monitor the status register, wait until busy-flag is high

    dataflash_CS_inactive();    //deactivate dataflash command decoder
}

void DataFlash_APM1::BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data)
{
    dataflash_CS_active();   // activate dataflash command decoder

    if (BufferNum==1)
        SPI_transfer(DF_BUFFER_1_WRITE);
    else
        SPI_transfer(DF_BUFFER_2_WRITE);
    SPI_transfer(0x00);                          //don't cares
    SPI_transfer((unsigned char)(IntPageAdr>>8)); //upper part of internal buffer address
    SPI_transfer((unsigned char)(IntPageAdr));   //lower part of internal buffer address
    SPI_transfer(Data);                          //write data byte

    dataflash_CS_inactive(); // disable dataflash command decoder
}

unsigned char DataFlash_APM1::BufferRead (unsigned char BufferNum, uint16_t IntPageAdr)
{
    byte tmp;

    dataflash_CS_active();   // activate dataflash command decoder

    if (BufferNum==1)
        SPI_transfer(DF_BUFFER_1_READ);
    else
        SPI_transfer(DF_BUFFER_2_READ);
    SPI_transfer(0x00);                          //don't cares
    SPI_transfer((unsigned char)(IntPageAdr>>8)); //upper part of internal buffer address
    SPI_transfer((unsigned char)(IntPageAdr));   //lower part of internal buffer address
    SPI_transfer(0x00);                          //don't cares
    tmp = SPI_transfer(0x00);                    //read data byte

    dataflash_CS_inactive(); // deactivate dataflash command decoder

    return (tmp);
}
// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_APM1::PageErase (uint16_t PageAdr)
{
    dataflash_CS_active();   // activate dataflash command decoder
    SPI_transfer(DF_PAGE_ERASE); // Command

    if(df_PageSize==512) {
        SPI_transfer((unsigned char)(PageAdr >> 7));
        SPI_transfer((unsigned char)(PageAdr << 1));
    }else{
        SPI_transfer((unsigned char)(PageAdr >> 6));
        SPI_transfer((unsigned char)(PageAdr << 2));
    }

    SPI_transfer(0x00);            // "dont cares"
    dataflash_CS_inactive();             //initiate flash page erase
    dataflash_CS_active();
    while(!ReadStatus()) ;

    dataflash_CS_inactive(); // deactivate dataflash command decoder
}


void DataFlash_APM1::BlockErase (uint16_t BlockAdr)
{
    dataflash_CS_active();   // activate dataflash command decoder
    SPI_transfer(DF_BLOCK_ERASE); // Command

    if (df_PageSize==512) {
        SPI_transfer((unsigned char)(BlockAdr >> 3));
        SPI_transfer((unsigned char)(BlockAdr << 5));
    } else {
        SPI_transfer((unsigned char)(BlockAdr >> 4));
        SPI_transfer((unsigned char)(BlockAdr << 4));
    }

    SPI_transfer(0x00);            // "dont cares"
    dataflash_CS_inactive();             //initiate flash page erase
    dataflash_CS_active();
    while(!ReadStatus()) ;

    dataflash_CS_inactive(); // deactivate dataflash command decoder
}



void DataFlash_APM1::ChipErase(void (*delay_cb)(unsigned long))
{

    dataflash_CS_active();   // activate dataflash command decoder
    // opcodes for chip erase
    SPI_transfer(DF_CHIP_ERASE_0);
    SPI_transfer(DF_CHIP_ERASE_1);
    SPI_transfer(DF_CHIP_ERASE_2);
    SPI_transfer(DF_CHIP_ERASE_3);

    dataflash_CS_inactive();             //initiate flash page erase
    dataflash_CS_active();
    while (!ReadStatus()) {
        delay_cb(1);
    }

    dataflash_CS_inactive(); // deactivate dataflash command decoder
}

