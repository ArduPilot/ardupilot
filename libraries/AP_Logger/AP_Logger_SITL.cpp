/*
  backend for SITL emulation of block logging
*/

#include "AP_Logger_SITL.h"

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdio.h>

#define DF_PAGE_SIZE 256UL
#define DF_PAGE_PER_SECTOR 16 // 4k sectors
#define DF_NUM_PAGES 65536UL

#define ERASE_TIME_MS 10000

extern const AP_HAL::HAL& hal;

void AP_Logger_SITL::Init()
{
    if (flash_fd == 0) {
        flash_fd = open(filename, O_RDWR|O_CLOEXEC, 0777);
        if (flash_fd == -1) {
            flash_fd = open(filename, O_RDWR | O_CREAT | O_CLOEXEC, 0777);
            StartErase();
            erase_started_ms = 0;
        }
        if (ftruncate(flash_fd, DF_PAGE_SIZE*uint32_t(DF_NUM_PAGES)) != 0) {
            AP_HAL::panic("Failed to create %s", filename);
        }
    }

    df_PageSize = DF_PAGE_SIZE;
    df_PagePerSector = DF_PAGE_PER_SECTOR;
    df_PagePerBlock = DF_PAGE_PER_SECTOR;
    df_NumPages = DF_NUM_PAGES;

    AP_Logger_Block::Init();
}

bool AP_Logger_SITL::CardInserted(void) const
{
    return df_NumPages > 0;
}

void AP_Logger_SITL::PageToBuffer(uint32_t PageAdr)
{
    assert(PageAdr>0 && PageAdr <= df_NumPages+1);
    if (pread(flash_fd, buffer, DF_PAGE_SIZE, (PageAdr-1)*DF_PAGE_SIZE) != DF_PAGE_SIZE) {
        printf("Failed flash read");
    }
}

void AP_Logger_SITL::BufferToPage(uint32_t PageAdr)
{
    assert(PageAdr>0 && PageAdr <= df_NumPages+1);
    if (pwrite(flash_fd, buffer, DF_PAGE_SIZE, (PageAdr-1)*DF_PAGE_SIZE) != DF_PAGE_SIZE) {
        printf("Failed flash write");
    }
}

void AP_Logger_SITL::SectorErase(uint32_t SectorAdr)
{
    uint8_t fill[DF_PAGE_SIZE*DF_PAGE_PER_SECTOR];
    memset(fill, 0xFF, sizeof(fill));
    if (pwrite(flash_fd, fill, sizeof(fill), SectorAdr*DF_PAGE_PER_SECTOR*DF_PAGE_SIZE) != sizeof(fill)) {
        printf("Failed sector erase");
    }
}

void AP_Logger_SITL::Sector4kErase(uint32_t SectorAdr)
{
    SectorErase(SectorAdr);
}

void AP_Logger_SITL::StartErase()
{
    for (uint32_t i=0; i<DF_NUM_PAGES/DF_PAGE_PER_SECTOR; i++) {
        SectorErase(i);
    }
    erase_started_ms = AP_HAL::millis();
}

bool AP_Logger_SITL::InErase()
{
    if (erase_started_ms == 0) {
        return false;
    }
    uint32_t now = AP_HAL::millis();
    if (now - erase_started_ms < ERASE_TIME_MS) {
        return true;
    }
    erase_started_ms = 0;
    return false;
}

#endif // HAL_BOARD_SITL

