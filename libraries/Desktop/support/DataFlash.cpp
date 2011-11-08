/*
  hacked up DataFlash library for Desktop support
*/

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include "DataFlash.h"
#include <SPI.h>

#define DF_PAGE_SIZE 512
#define DF_NUM_PAGES 4096

static int flash_fd;
static uint8_t buffer[2][DF_PAGE_SIZE];

#define OVERWRITE_DATA 0 // 0: When reach the end page stop, 1: Start overwritten from page 1

// Constructors ////////////////////////////////////////////////////////////////
DataFlash_Class::DataFlash_Class()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_Class::Init(void)
{
	if (flash_fd == 0) {
		flash_fd = open("dataflash.bin", O_RDWR, 0777);
		if (flash_fd == -1) {
			uint8_t *fill;
			fill = (uint8_t *)malloc(DF_PAGE_SIZE*DF_NUM_PAGES);
			flash_fd = open("dataflash.bin", O_RDWR | O_CREAT, 0777);
			memset(fill, 0xFF, DF_PAGE_SIZE*DF_NUM_PAGES);
			write(flash_fd, fill, DF_PAGE_SIZE*DF_NUM_PAGES);
			free(fill);
		}
	}
	df_PageSize = DF_PAGE_SIZE;
	df_BufferNum  = 1;
	df_BufferIdx  = 0;
	df_PageAdr    = 0;
	df_Stop_Write = 0;
}

// This function is mainly to test the device
void DataFlash_Class::ReadManufacturerID()
{
	df_manufacturer = 1;
	df_device_0 = 2;
	df_device_1 = 3;
}

// Read the status register
byte DataFlash_Class::ReadStatusReg()
{
	return 0;
}

// Read the status of the DataFlash
inline
byte DataFlash_Class::ReadStatus()
{
	return 1;
}


inline
uint16_t DataFlash_Class::PageSize()
{
	return df_PageSize;
}


// Wait until DataFlash is in ready state...
void DataFlash_Class::WaitReady()
{
	while(!ReadStatus());
}

void DataFlash_Class::PageToBuffer(unsigned char BufferNum, uint16_t PageAdr)
{
	pread(flash_fd, buffer[BufferNum-1], DF_PAGE_SIZE, PageAdr*DF_PAGE_SIZE);
}

void DataFlash_Class::BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait)
{
	pwrite(flash_fd, buffer[BufferNum-1], DF_PAGE_SIZE, PageAdr*DF_PAGE_SIZE);
}

void DataFlash_Class::BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data)
{
	buffer[BufferNum-1][IntPageAdr] = (uint8_t)Data;
}

unsigned char DataFlash_Class::BufferRead (unsigned char BufferNum, uint16_t IntPageAdr)
{
	return (unsigned char)buffer[BufferNum-1][IntPageAdr];
}

// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_Class::PageErase (uint16_t PageAdr)
{
	uint8_t fill[DF_PAGE_SIZE];
	memset(fill, 0xFF, sizeof(fill));
	pwrite(flash_fd, fill, DF_PAGE_SIZE, PageAdr*DF_PAGE_SIZE);
}


void DataFlash_Class::ChipErase ()
{
	for (int i=0; DF_NUM_PAGES; i++) {
		PageErase(i);
	}
}

// *** DATAFLASH PUBLIC FUNCTIONS ***
void DataFlash_Class::StartWrite(int16_t PageAdr)
{
	df_BufferNum  = 1;
	df_BufferIdx  = 0;
	df_PageAdr    = PageAdr;
	df_Stop_Write = 0;
}

void DataFlash_Class::FinishWrite(void)
{
	df_BufferIdx = 0;
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

void DataFlash_Class::WriteInt(int16_t data)
{
  WriteByte(data>>8);   // High byte
  WriteByte(data&0xFF); // Low byte
}

void DataFlash_Class::WriteLong(int32_t data)
{
  WriteByte(data>>24);   // First byte
  WriteByte(data>>16);
  WriteByte(data>>8);
  WriteByte(data&0xFF);  // Last byte
}

// Get the last page written to
int16_t DataFlash_Class::GetWritePage()
{
  return(df_PageAdr);
}

// Get the last page read
int16_t DataFlash_Class::GetPage()
{
  return(df_Read_PageAdr-1);
}

void DataFlash_Class::StartRead(int16_t PageAdr)
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

int16_t DataFlash_Class::ReadInt()
{
  uint16_t result;

  result = ReadByte();               // High byte
  result = (result<<8) | ReadByte(); // Low byte
  return (int16_t)result;
}

int32_t DataFlash_Class::ReadLong()
{
  uint32_t result;

  result = ReadByte();               // First byte
  result = (result<<8) | ReadByte();
  result = (result<<8) | ReadByte();
  result = (result<<8) | ReadByte(); // Last byte
  return (int32_t)result;
}

// make one instance for the user to use
DataFlash_Class DataFlash;
