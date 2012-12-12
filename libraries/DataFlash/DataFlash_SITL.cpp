/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  hacked up DataFlash library for Desktop support
*/

#if 0
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include "DataFlash.h"

#define DF_PAGE_SIZE 512
#define DF_NUM_PAGES 4096

extern const AP_HAL::HAL& hal;

static int flash_fd;
static uint8_t buffer[2][DF_PAGE_SIZE];

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_SITL::Init(void)
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

    // reserve last page for config information
    df_NumPages   = DF_NUM_PAGES - 1;
}

// This function is mainly to test the device
void DataFlash_SITL::ReadManufacturerID()
{
	df_manufacturer = 1;
	df_device = 0x0203;
}

bool DataFlash_SITL::CardInserted(void)
{
    return true;
}

// Read the status register
uint8_t DataFlash_SITL::ReadStatusReg()
{
	return 0;
}

// Read the status of the DataFlash
inline
uint8_t DataFlash_SITL::ReadStatus()
{
	return 1;
}


inline
uint16_t DataFlash_SITL::PageSize()
{
	return df_PageSize;
}


// Wait until DataFlash is in ready state...
void DataFlash_SITL::WaitReady()
{
	while(!ReadStatus());
}

void DataFlash_SITL::PageToBuffer(unsigned char BufferNum, uint16_t PageAdr)
{
	pread(flash_fd, buffer[BufferNum-1], DF_PAGE_SIZE, PageAdr*DF_PAGE_SIZE);
}

void DataFlash_SITL::BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait)
{
	pwrite(flash_fd, buffer[BufferNum-1], DF_PAGE_SIZE, PageAdr*DF_PAGE_SIZE);
}

void DataFlash_SITL::BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data)
{
	buffer[BufferNum-1][IntPageAdr] = (uint8_t)Data;
}

unsigned char DataFlash_SITL::BufferRead (unsigned char BufferNum, uint16_t IntPageAdr)
{
	return (unsigned char)buffer[BufferNum-1][IntPageAdr];
}

// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_SITL::PageErase (uint16_t PageAdr)
{
	uint8_t fill[DF_PAGE_SIZE];
	memset(fill, 0xFF, sizeof(fill));
	pwrite(flash_fd, fill, DF_PAGE_SIZE, PageAdr*DF_PAGE_SIZE);
}

void DataFlash_SITL::BlockErase (uint16_t BlockAdr)
{
	uint8_t fill[DF_PAGE_SIZE*8];
	memset(fill, 0xFF, sizeof(fill));
	pwrite(flash_fd, fill, DF_PAGE_SIZE*8, BlockAdr*DF_PAGE_SIZE*8);
}


void DataFlash_SITL::ChipErase()
{
	for (int i=0; i<DF_NUM_PAGES; i++) {
		PageErase(i);
        hal.scheduler->delay(1);
	}
}


#endif
#endif

