/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_APM1::Init(void)
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
    df_NumPages   = DF_NUM_PAGES;
}

// This function is mainly to test the device
void DataFlash_APM1::ReadManufacturerID()
{
	df_manufacturer = 1;
	df_device_0 = 2;
	df_device_1 = 3;
}

// Read the status register
byte DataFlash_APM1::ReadStatusReg()
{
	return 0;
}

// Read the status of the DataFlash
inline
byte DataFlash_APM1::ReadStatus()
{
	return 1;
}


inline
uint16_t DataFlash_APM1::PageSize()
{
	return df_PageSize;
}


// Wait until DataFlash is in ready state...
void DataFlash_APM1::WaitReady()
{
	while(!ReadStatus());
}

void DataFlash_APM1::PageToBuffer(unsigned char BufferNum, uint16_t PageAdr)
{
	pread(flash_fd, buffer[BufferNum-1], DF_PAGE_SIZE, PageAdr*DF_PAGE_SIZE);
}

void DataFlash_APM1::BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait)
{
	pwrite(flash_fd, buffer[BufferNum-1], DF_PAGE_SIZE, PageAdr*DF_PAGE_SIZE);
}

void DataFlash_APM1::BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data)
{
	buffer[BufferNum-1][IntPageAdr] = (uint8_t)Data;
}

unsigned char DataFlash_APM1::BufferRead (unsigned char BufferNum, uint16_t IntPageAdr)
{
	return (unsigned char)buffer[BufferNum-1][IntPageAdr];
}

// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_APM1::PageErase (uint16_t PageAdr)
{
	uint8_t fill[DF_PAGE_SIZE];
	memset(fill, 0xFF, sizeof(fill));
	pwrite(flash_fd, fill, DF_PAGE_SIZE, PageAdr*DF_PAGE_SIZE);
}


void DataFlash_APM1::ChipErase ()
{
	for (int i=0; DF_NUM_PAGES; i++) {
		PageErase(i);
	}
}

// make one instance for the user to use
DataFlash_APM1::DataFlash_APM1() {}
