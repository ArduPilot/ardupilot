/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	DataFlash_APM2.cpp - DataFlash log library for AT45DB321D
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com
	This code works only on ATMega2560. It uses Serial port 3 in SPI MSPI mdoe.

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Dataflash library for AT45DB321D flash memory
	Memory organization : 8192 pages of 512 bytes or 528 bytes

	Maximun write bandwidth : 512 bytes in 14ms
	This code is written so the master never has to wait to write the data on the eeprom

	Methods:
		Init() : Library initialization (SPI initialization)
		StartWrite(page) : Start a write session. page=start page.
		WriteByte(data) : Write a byte
		WriteInt(data) :  Write an integer (2 bytes)
		WriteLong(data) : Write a long (4 bytes)
		StartRead(page) : Start a read on (page)
		GetWritePage() : Returns the last page written to
		GetPage() : Returns the last page read
		ReadByte()
		ReadInt()
		ReadLong()

	Properties:

*/

extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <avr/interrupt.h>
  #include "WConstants.h"
}

#include "DataFlash_APM2.h"

// DataFlash is connected to Serial Port 3 (we will use SPI mode)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	#define DF_DATAOUT 14        // MOSI
	#define DF_DATAIN  15        // MISO
	#define DF_SPICLOCK  PJ2     // SCK
	#define DF_SLAVESELECT 28    // SS     (PA6)
    #define DF_RESET 41          // RESET  (PG0)
    #define DF_CARDDETECT 33     // PC4
#else
	# error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#endif

#define DF_MAX_PAGE 8192

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


#define OVERWRITE_DATA 0 // 0: When reach the end page stop, 1: Start overwritten from page 1

// *** INTERNAL FUNCTIONS ***
unsigned char DataFlash_APM2::SPI_transfer(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR3A & (1<<UDRE3)) );
	/* Put data into buffer, sends the data */
	UDR3 = data;
	/* Wait for data to be received */
	while ( !(UCSR3A & (1<<RXC3)) );
	/* Get and return received data from buffer */
	return UDR3;
}

void DataFlash_APM2::CS_inactive()
{
  digitalWrite(DF_SLAVESELECT,HIGH); //disable device
}

void DataFlash_APM2::CS_active()
{
  digitalWrite(DF_SLAVESELECT,LOW); //enable device
}

// Constructors ////////////////////////////////////////////////////////////////
DataFlash_APM2::DataFlash_APM2()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_APM2::Init(void)
{
  pinMode(DF_DATAOUT, OUTPUT);
  pinMode(DF_DATAIN, INPUT);
  pinMode(DF_SLAVESELECT,OUTPUT);
  pinMode(DF_RESET,OUTPUT);
  pinMode(DF_CARDDETECT, INPUT);

  // Reset the chip
  digitalWrite(DF_RESET,LOW);
  delay(1);
  digitalWrite(DF_RESET,HIGH);

  df_Read_END=false;

  CS_inactive();     //disable device

  // Setup Serial Port3 in SPI mode (MSPI), Mode 0, Clock: 8Mhz
  UBRR3 = 0;
  DDRJ |= (1<<PJ2);                                   // SPI clock XCK3 (PJ2) as output. This enable SPI Master mode
  // Set MSPI mode of operation and SPI data mode 0.
  UCSR3C = (1<<UMSEL31)|(1<<UMSEL30);       //|(1<<1)|(1<<UCPOL3);
  // Enable receiver and transmitter.
  UCSR3B = (1<<RXEN3)|(1<<TXEN3);
  // Set Baud rate
  UBRR3 = 0;         // SPI running at 8Mhz

  // get page size: 512 or 528  (by default: 528)
  df_PageSize=PageSize();
}

// This function is mainly to test the device
void DataFlash_APM2::ReadManufacturerID()
{
  CS_inactive();    // Reset dataflash command decoder
  CS_active();

  // Read manufacturer and ID command...
  SPI_transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);

  df_manufacturer = SPI_transfer(0xff);
  df_device_0 = SPI_transfer(0xff);
  df_device_1 = SPI_transfer(0xff);
  SPI_transfer(0xff);
}

// This function return 1 if Card is inserted on SD slot
bool DataFlash_APM2::CardInserted()
{
    return (digitalRead(DF_CARDDETECT) != 0);
}

// Read the status register
byte DataFlash_APM2::ReadStatusReg()
{
  CS_inactive();    // Reset dataflash command decoder
  CS_active();

  // Read status command
  SPI_transfer(DF_STATUS_REGISTER_READ);
  return SPI_transfer(0x00);  // We only want to extract the READY/BUSY bit
}

// Read the status of the DataFlash
inline
byte DataFlash_APM2::ReadStatus()
{
  return(ReadStatusReg()&0x80);  // We only want to extract the READY/BUSY bit
}


inline
uint16_t DataFlash_APM2::PageSize()
{
  return(528-((ReadStatusReg()&0x01)<<4));  // if first bit 1 trhen 512 else 528 bytes
}


// Wait until DataFlash is in ready state...
void DataFlash_APM2::WaitReady()
{
  while(!ReadStatus());
}

void DataFlash_APM2::PageToBuffer(unsigned char BufferNum, uint16_t PageAdr)
{
  CS_inactive();
  CS_active();
  if (BufferNum==1)
    SPI_transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
  else
    SPI_transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);

  if(df_PageSize==512){
    SPI_transfer((unsigned char)(PageAdr >> 7));
    SPI_transfer((unsigned char)(PageAdr << 1));
  }else{
    SPI_transfer((unsigned char)(PageAdr >> 6));
    SPI_transfer((unsigned char)(PageAdr << 2));
  }
  SPI_transfer(0x00);	// don´t care bytes

  CS_inactive();	//initiate the transfer
  CS_active();

  while(!ReadStatus());  //monitor the status register, wait until busy-flag is high
}

void DataFlash_APM2::BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait)
{
  CS_inactive();     // Reset dataflash command decoder
  CS_active();

  if (BufferNum==1)
    SPI_transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
  else
    SPI_transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);

  if(df_PageSize==512){
    SPI_transfer((unsigned char)(PageAdr >> 7));
    SPI_transfer((unsigned char)(PageAdr << 1));
  }else{
    SPI_transfer((unsigned char)(PageAdr >> 6));
    SPI_transfer((unsigned char)(PageAdr << 2));
  }
  SPI_transfer(0x00);	// don´t care bytes

  CS_inactive();	//initiate the transfer
  CS_active();

  // Check if we need to wait to write the buffer to memory or we can continue...
  if (wait)
	while(!ReadStatus());  //monitor the status register, wait until busy-flag is high
}

void DataFlash_APM2::BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data)
{
  CS_inactive();   // Reset dataflash command decoder
  CS_active();

  if (BufferNum==1)
    SPI_transfer(DF_BUFFER_1_WRITE);
  else
    SPI_transfer(DF_BUFFER_2_WRITE);
  SPI_transfer(0x00);				 //don't cares
  SPI_transfer((unsigned char)(IntPageAdr>>8));  //upper part of internal buffer address
  SPI_transfer((unsigned char)(IntPageAdr));	 //lower part of internal buffer address
  SPI_transfer(Data);				 //write data byte
}

unsigned char DataFlash_APM2::BufferRead (unsigned char BufferNum, uint16_t IntPageAdr)
{
  byte tmp;

  CS_inactive();   // Reset dataflash command decoder
  CS_active();

  if (BufferNum==1)
    SPI_transfer(DF_BUFFER_1_READ);
  else
    SPI_transfer(DF_BUFFER_2_READ);
  SPI_transfer(0x00);				 //don't cares
  SPI_transfer((unsigned char)(IntPageAdr>>8));  //upper part of internal buffer address
  SPI_transfer((unsigned char)(IntPageAdr));	 //lower part of internal buffer address
  SPI_transfer(0x00);                            //don't cares
  tmp = SPI_transfer(0x00);		         //read data byte

  return (tmp);
}
// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_APM2::PageErase (uint16_t PageAdr)
{
  CS_inactive();																//make sure to toggle CS signal in order
  CS_active();																//to reset Dataflash command decoder
  SPI_transfer(DF_PAGE_ERASE);   // Command

  if(df_PageSize==512){
    SPI_transfer((unsigned char)(PageAdr >> 7));
    SPI_transfer((unsigned char)(PageAdr << 1));
  }else{
    SPI_transfer((unsigned char)(PageAdr >> 6));
    SPI_transfer((unsigned char)(PageAdr << 2));
  }

  SPI_transfer(0x00);	           // "dont cares"
  CS_inactive();               //initiate flash page erase
  CS_active();
  while(!ReadStatus());
}


void DataFlash_APM2::ChipErase ()
{
  CS_inactive();																//make sure to toggle CS signal in order
  CS_active();																//to reset Dataflash command decoder
  // opcodes for chip erase
  SPI_transfer(DF_CHIP_ERASE_0);
  SPI_transfer(DF_CHIP_ERASE_1);
  SPI_transfer(DF_CHIP_ERASE_2);
  SPI_transfer(DF_CHIP_ERASE_3);

  CS_inactive();               //initiate flash page erase
  CS_active();
  while(!ReadStatus());
}

// *** DATAFLASH PUBLIC FUNCTIONS ***
void DataFlash_APM2::StartWrite(int16_t PageAdr)
{
  df_BufferNum=1;
  df_BufferIdx=4;
  df_PageAdr=PageAdr;
  df_Stop_Write=0;
  WaitReady();

  // We are starting a new page - write FileNumber and FilePage
  BufferWrite(df_BufferNum,0,df_FileNumber>>8);   // High byte
  BufferWrite(df_BufferNum,1,df_FileNumber&0xFF); // Low byte
  BufferWrite(df_BufferNum,2,df_FilePage>>8);   // High byte
  BufferWrite(df_BufferNum,3,df_FilePage&0xFF); // Low byte
}

void DataFlash_APM2::FinishWrite(void)
{
	df_BufferIdx=0;
	BufferToPage(df_BufferNum,df_PageAdr,0);  // Write Buffer to memory, NO WAIT
	df_PageAdr++;
	if (OVERWRITE_DATA==1)
	    {
        if (df_PageAdr>=DF_MAX_PAGE)  // If we reach the end of the memory, start from the begining
		  df_PageAdr = 1;
	    }
	else
	    {
        if (df_PageAdr>=DF_MAX_PAGE)  // If we reach the end of the memory, stop here
		  df_Stop_Write=1;
	    }

	if (df_BufferNum==1)  // Change buffer to continue writing...
        df_BufferNum=2;
	else
        df_BufferNum=1;
}


void DataFlash_APM2::WriteByte(byte data)
{
  if (!df_Stop_Write)
    {
    BufferWrite(df_BufferNum,df_BufferIdx,data);
    df_BufferIdx++;
    if (df_BufferIdx >= df_PageSize)  // End of buffer?
      {
	  df_BufferIdx=4;		//(4 bytes for FileNumber, FilePage)
	  BufferToPage(df_BufferNum,df_PageAdr,0);  // Write Buffer to memory, NO WAIT
      df_PageAdr++;
	  if (OVERWRITE_DATA==1)
	    {
        if (df_PageAdr>=DF_MAX_PAGE)  // If we reach the end of the memory, start from the begining
		  df_PageAdr = 1;
	    }
      else
	    {
        if (df_PageAdr>=DF_MAX_PAGE)  // If we reach the end of the memory, stop here
		  df_Stop_Write=1;
	    }

      if (df_BufferNum==1)  // Change buffer to continue writing...
        df_BufferNum=2;
      else
        df_BufferNum=1;
      // We are starting a new page - write FileNumber and FilePage
      BufferWrite(df_BufferNum,0,df_FileNumber>>8);   // High byte
      BufferWrite(df_BufferNum,1,df_FileNumber&0xFF); // Low byte
      df_FilePage++;
      BufferWrite(df_BufferNum,2,df_FilePage>>8);   // High byte
      BufferWrite(df_BufferNum,3,df_FilePage&0xFF); // Low byte
      }
    }
}

void DataFlash_APM2::WriteInt(int16_t data)
{
  WriteByte(data>>8);   // High byte
  WriteByte(data&0xFF); // Low byte
}

void DataFlash_APM2::WriteLong(int32_t data)
{
  WriteByte(data>>24);   // First byte
  WriteByte(data>>16);
  WriteByte(data>>8);
  WriteByte(data&0xFF);  // Last byte
}

// Get the last page written to
int16_t DataFlash_APM2::GetWritePage()
{
  return(df_PageAdr);
}

// Get the last page read
int16_t DataFlash_APM2::GetPage()
{
  return(df_Read_PageAdr-1);
}

void DataFlash_APM2::StartRead(int16_t PageAdr)
{
  df_Read_BufferNum=1;
  df_Read_BufferIdx=4;
  df_Read_PageAdr=PageAdr;
  WaitReady();
  PageToBuffer(df_Read_BufferNum,df_Read_PageAdr);  // Write Memory page to buffer
  df_Read_PageAdr++;

  // We are starting a new page - read FileNumber and FilePage
  df_FileNumber = BufferRead(df_Read_BufferNum,0);   // High byte
  df_FileNumber = (df_FileNumber<<8) | BufferRead(df_Read_BufferNum,1); // Low byte
  df_FilePage = BufferRead(df_Read_BufferNum,2);   // High byte
  df_FilePage = (df_FilePage<<8) | BufferRead(df_Read_BufferNum,3); // Low byte
}

byte DataFlash_APM2::ReadByte()
{
  byte result;

  WaitReady();
  result = BufferRead(df_Read_BufferNum,df_Read_BufferIdx);
  df_Read_BufferIdx++;
  if (df_Read_BufferIdx >= df_PageSize)  // End of buffer?
    {
    df_Read_BufferIdx=4;		//(4 bytes for FileNumber, FilePage)
    PageToBuffer(df_Read_BufferNum,df_Read_PageAdr);  // Write memory page to Buffer
    df_Read_PageAdr++;
    if (df_Read_PageAdr>=DF_MAX_PAGE)  // If we reach the end of the memory, start from the begining
      {
      df_Read_PageAdr = 0;
      df_Read_END = true;
      }

    // We are starting a new page - read FileNumber and FilePage
    df_FileNumber = BufferRead(df_Read_BufferNum,0);   // High byte
    df_FileNumber = (df_FileNumber<<8) | BufferRead(df_Read_BufferNum,1); // Low byte
    df_FilePage = BufferRead(df_Read_BufferNum,2);   // High byte
    df_FilePage = (df_FilePage<<8) | BufferRead(df_Read_BufferNum,3); // Low byte
    }
  return result;
}

int16_t DataFlash_APM2::ReadInt()
{
  uint16_t result;

  result = ReadByte();               // High byte
  result = (result<<8) | ReadByte(); // Low byte
  return (int16_t)result;
}

int32_t DataFlash_APM2::ReadLong()
{
  uint32_t result;

  result = ReadByte();               // First byte
  result = (result<<8) | ReadByte();
  result = (result<<8) | ReadByte();
  result = (result<<8) | ReadByte(); // Last byte
  return (int32_t)result;
}

void DataFlash_APM2::SetFileNumber(uint16_t FileNumber)
{
	df_FileNumber = FileNumber;
	df_FilePage = 1;
}

uint16_t DataFlash_APM2::GetFileNumber()
{
	return df_FileNumber;
}

uint16_t DataFlash_APM2::GetFilePage()
{
	return df_FilePage;
}
