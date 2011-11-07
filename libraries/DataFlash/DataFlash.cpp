/*
	DataFlash.cpp - DataFlash log library for AT45DB161
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com
	This code works with boards based on ATMega168/328 and ATMega1280/2560 using SPI port

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Dataflash library for AT45DB161D flash memory
	Memory organization : 4096 pages of 512 bytes or 528 bytes

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

#include <stdint.h>
#include "DataFlash.h"
#include <SPI.h>

#define OVERWRITE_DATA 0 // 0: When reach the end page stop, 1: Start overwritten from page 1

// *** INTERNAL FUNCTIONS ***

void dataflash_CS_inactive()
{
  digitalWrite(DF_SLAVESELECT,HIGH); //disable device
}

void dataflash_CS_active()
{
  digitalWrite(DF_SLAVESELECT,LOW); //enable device
}

// Constructors ////////////////////////////////////////////////////////////////
DataFlash_Class::DataFlash_Class()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_Class::Init(void)
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

  df_Read_END=false;
  
  dataflash_CS_inactive();     //disable device
  
  // Setup SPI  Master, Mode 3, fosc/4 = 4MHz
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV2);  

  // get page size: 512 or 528
  df_PageSize=PageSize();
}

// This function is mainly to test the device
void DataFlash_Class::ReadManufacturerID()
{
  dataflash_CS_active();     // activate dataflash command decoder
 
  // Read manufacturer and ID command...
  SPI.transfer(DF_READ_MANUFACTURER_AND_DEVICE_ID);

  df_manufacturer = SPI.transfer(0xff);
  df_device_0 = SPI.transfer(0xff);
  df_device_1 = SPI.transfer(0xff);
  SPI.transfer(0xff);
  
  dataflash_CS_inactive();    // Reset dataflash command decoder  
}

// Read the status register
byte DataFlash_Class::ReadStatusReg()
{ 
  byte tmp;

  dataflash_CS_active();     // activate dataflash command decoder
 
  // Read status command
  SPI.transfer(DF_STATUS_REGISTER_READ);
  tmp = SPI.transfer(0x00);  // We only want to extract the READY/BUSY bit
  
  dataflash_CS_inactive();    // Reset dataflash command decoder
  
  return tmp;
}

// Read the status of the DataFlash
inline
byte DataFlash_Class::ReadStatus()
{ 
  return(ReadStatusReg()&0x80);  // We only want to extract the READY/BUSY bit
}


inline
unsigned int DataFlash_Class::PageSize()
{ 
  return(528-((ReadStatusReg()&0x01)<<4));  // if first bit 1 trhen 512 else 528 bytes
}


// Wait until DataFlash is in ready state...
void DataFlash_Class::WaitReady()
{
  while(!ReadStatus());
}

void DataFlash_Class::PageToBuffer(unsigned char BufferNum, unsigned int PageAdr)
{
  dataflash_CS_active();     // activate dataflash command decoder
  
  if (BufferNum==1)				
    SPI.transfer(DF_TRANSFER_PAGE_TO_BUFFER_1);
  else
    SPI.transfer(DF_TRANSFER_PAGE_TO_BUFFER_2);
  
  if(df_PageSize==512){
    SPI.transfer((unsigned char)(PageAdr >> 7));	
    SPI.transfer((unsigned char)(PageAdr << 1));	
  }else{
    SPI.transfer((unsigned char)(PageAdr >> 6));	
    SPI.transfer((unsigned char)(PageAdr << 2));
  }
  SPI.transfer(0x00);	// don´t care bytes			

  dataflash_CS_inactive();	//initiate the transfer
  dataflash_CS_active();
  
  while(!ReadStatus());  //monitor the status register, wait until busy-flag is high
  
  dataflash_CS_inactive();
}

void DataFlash_Class::BufferToPage (unsigned char BufferNum, unsigned int PageAdr, unsigned char wait)
{
  dataflash_CS_active();     // activate dataflash command decoder
  
  if (BufferNum==1)	
    SPI.transfer(DF_BUFFER_1_TO_PAGE_WITH_ERASE);
  else
    SPI.transfer(DF_BUFFER_2_TO_PAGE_WITH_ERASE);

  if(df_PageSize==512){
    SPI.transfer((unsigned char)(PageAdr >> 7));	
    SPI.transfer((unsigned char)(PageAdr << 1));	
  }else{
    SPI.transfer((unsigned char)(PageAdr >> 6));	
    SPI.transfer((unsigned char)(PageAdr << 2));
  }
  SPI.transfer(0x00);	// don´t care bytes			

  dataflash_CS_inactive();	//initiate the transfer
  dataflash_CS_active();
  
  // Check if we need to wait to write the buffer to memory or we can continue...
  if (wait)
	while(!ReadStatus());  //monitor the status register, wait until busy-flag is high
	
  dataflash_CS_inactive();	//deactivate dataflash command decoder	
}

void DataFlash_Class::BufferWrite (unsigned char BufferNum, unsigned int IntPageAdr, unsigned char Data)
{
  dataflash_CS_active();     // activate dataflash command decoder
  
  if (BufferNum==1)	
    SPI.transfer(DF_BUFFER_1_WRITE);
  else
    SPI.transfer(DF_BUFFER_2_WRITE);
  SPI.transfer(0x00);				 //don't cares
  SPI.transfer((unsigned char)(IntPageAdr>>8));  //upper part of internal buffer address
  SPI.transfer((unsigned char)(IntPageAdr));	 //lower part of internal buffer address
  SPI.transfer(Data);				 //write data byte
  
  dataflash_CS_inactive();   // disable dataflash command decoder
}
  
unsigned char DataFlash_Class::BufferRead (unsigned char BufferNum, unsigned int IntPageAdr)
{
  byte tmp;

  dataflash_CS_active();     // activate dataflash command decoder
  
  if (BufferNum==1)	
    SPI.transfer(DF_BUFFER_1_READ);
  else
    SPI.transfer(DF_BUFFER_2_READ);
  SPI.transfer(0x00);				 //don't cares
  SPI.transfer((unsigned char)(IntPageAdr>>8));  //upper part of internal buffer address
  SPI.transfer((unsigned char)(IntPageAdr));	 //lower part of internal buffer address
  SPI.transfer(0x00);                            //don't cares
  tmp = SPI.transfer(0x00);		         //read data byte
  
  dataflash_CS_inactive();   // deactivate dataflash command decoder
  
  return (tmp);
}
// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_Class::PageErase (unsigned int PageAdr)
{
  dataflash_CS_active();     // activate dataflash command decoder
  SPI.transfer(DF_PAGE_ERASE);   // Command

  if(df_PageSize==512){
    SPI.transfer((unsigned char)(PageAdr >> 7));	
    SPI.transfer((unsigned char)(PageAdr << 1));	
  }else{
    SPI.transfer((unsigned char)(PageAdr >> 6));	
    SPI.transfer((unsigned char)(PageAdr << 2));
  }

  SPI.transfer(0x00);	           // "dont cares"
  dataflash_CS_inactive();               //initiate flash page erase
  dataflash_CS_active();
  while(!ReadStatus());
  
  dataflash_CS_inactive();   // deactivate dataflash command decoder
}


void DataFlash_Class::ChipErase ()
{

  dataflash_CS_active();     // activate dataflash command decoder
  // opcodes for chip erase
  SPI.transfer(DF_CHIP_ERASE_0);
  SPI.transfer(DF_CHIP_ERASE_1);
  SPI.transfer(DF_CHIP_ERASE_2);
  SPI.transfer(DF_CHIP_ERASE_3);
  
  dataflash_CS_inactive();               //initiate flash page erase
  dataflash_CS_active();
  while(!ReadStatus());
  
  dataflash_CS_inactive();   // deactivate dataflash command decoder
}

// *** DATAFLASH PUBLIC FUNCTIONS ***
void DataFlash_Class::StartWrite(int PageAdr)
{
  df_BufferNum=1;
  df_BufferIdx=0;
  df_PageAdr=PageAdr;
  df_Stop_Write=0;
  WaitReady();
}

void DataFlash_Class::FinishWrite(void)
{
	df_BufferIdx=0;
	BufferToPage(df_BufferNum,df_PageAdr,0);  // Write Buffer to memory, NO WAIT
	df_PageAdr++;
	if (OVERWRITE_DATA==1)
	    {
        if (df_PageAdr>=4096)  // If we reach the end of the memory, start from the begining
		  df_PageAdr = 1;
	    }
	else
	    {
        if (df_PageAdr>=4096)  // If we reach the end of the memory, stop here
		  df_Stop_Write=1;
	    }

	if (df_BufferNum==1)  // Change buffer to continue writing...
        df_BufferNum=2;
	else
        df_BufferNum=1;
}
	

void DataFlash_Class::WriteByte(byte data)
{
  if (!df_Stop_Write)
    {
    BufferWrite(df_BufferNum,df_BufferIdx,data);
    df_BufferIdx++;
    if (df_BufferIdx >= df_PageSize)  // End of buffer?
      {
      df_BufferIdx=0;
	  BufferToPage(df_BufferNum,df_PageAdr,0);  // Write Buffer to memory, NO WAIT
      df_PageAdr++;
	  if (OVERWRITE_DATA==1)
	    {
        if (df_PageAdr>=4096)  // If we reach the end of the memory, start from the begining
		  df_PageAdr = 1;
	    }
      else
	    {
        if (df_PageAdr>=4096)  // If we reach the end of the memory, stop here
		  df_Stop_Write=1;
	    }

      if (df_BufferNum==1)  // Change buffer to continue writing...
        df_BufferNum=2;
      else
        df_BufferNum=1;
      }
    }
}

void DataFlash_Class::WriteInt(int data)
{
  WriteByte(data>>8);   // High byte
  WriteByte(data&0xFF); // Low byte
}

void DataFlash_Class::WriteLong(long data)
{
  WriteByte(data>>24);   // First byte
  WriteByte(data>>16);
  WriteByte(data>>8);
  WriteByte(data&0xFF);  // Last byte
}

// Get the last page written to
int DataFlash_Class::GetWritePage() 
{
  return(df_PageAdr);
}

// Get the last page read
int DataFlash_Class::GetPage() 
{
  return(df_Read_PageAdr-1);
}

void DataFlash_Class::StartRead(int PageAdr)
{
  df_Read_BufferNum=1;
  df_Read_BufferIdx=0;
  df_Read_PageAdr=PageAdr;
  WaitReady();
  PageToBuffer(df_Read_BufferNum,df_Read_PageAdr);  // Write Memory page to buffer
  df_Read_PageAdr++;
}

byte DataFlash_Class::ReadByte()
{
  byte result;
  
  WaitReady();
  result = BufferRead(df_Read_BufferNum,df_Read_BufferIdx);
  df_Read_BufferIdx++;
  if (df_Read_BufferIdx >= df_PageSize)  // End of buffer?
    {
    df_Read_BufferIdx=0;
    PageToBuffer(df_Read_BufferNum,df_Read_PageAdr);  // Write memory page to Buffer
    df_Read_PageAdr++;
    if (df_Read_PageAdr>=4096)  // If we reach the end of the memory, start from the begining
      {
      df_Read_PageAdr = 0;
      df_Read_END = true;
      }
    }
  return result;
}

int DataFlash_Class::ReadInt()
{
  int result;
  
  result = ReadByte();               // High byte
  result = (result<<8) | ReadByte(); // Low byte
  return result;
}

long DataFlash_Class::ReadLong()
{
  long result;
  
  result = ReadByte();               // First byte
  result = (result<<8) | ReadByte(); 
  result = (result<<8) | ReadByte();
  result = (result<<8) | ReadByte(); // Last byte
  return result;
}

// make one instance for the user to use
DataFlash_Class DataFlash;
