/*
  logging to a DataFlash block based storage device on SPI
*/


#include <AP_HAL/AP_HAL.h>

#include "AP_Logger_DataFlash.h"

#if HAL_LOGGING_DATAFLASH_ENABLED

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_WRITE_DISABLE          0x04
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_FAST_READ              0x0b
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02

#define JEDEC_BULK_ERASE             0xC7
#define JEDEC_SECTOR4_ERASE          0x20 // 4k erase
#define JEDEC_BLOCK32_ERASE          0x52 // 32K erase
#define JEDEC_BLOCK64_ERASE          0xD8 // 64K erase

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02
#define JEDEC_STATUS_BP0             0x04
#define JEDEC_STATUS_BP1             0x08
#define JEDEC_STATUS_BP2             0x10
#define JEDEC_STATUS_TP              0x20
#define JEDEC_STATUS_SEC             0x40
#define JEDEC_STATUS_SRP0            0x80

/*
  flash device IDs taken from betaflight flash_m25p16.c

  Format is manufacturer, memory type, then capacity
*/
#define JEDEC_ID_MACRONIX_MX25L3206E   0xC22016
#define JEDEC_ID_MACRONIX_MX25L6406E   0xC22017
#define JEDEC_ID_MACRONIX_MX25L25635E  0xC22019
#define JEDEC_ID_MICRON_M25P16         0x202015
#define JEDEC_ID_MICRON_N25Q064        0x20BA17
#define JEDEC_ID_MICRON_N25Q128        0x20ba18
#define JEDEC_ID_WINBOND_W25Q16        0xEF4015
#define JEDEC_ID_WINBOND_W25Q32        0xEF4016
#define JEDEC_ID_WINBOND_W25X32        0xEF3016
#define JEDEC_ID_WINBOND_W25Q64        0xEF4017
#define JEDEC_ID_WINBOND_W25Q128       0xEF4018
#define JEDEC_ID_WINBOND_W25Q256       0xEF4019
#define JEDEC_ID_CYPRESS_S25FL128L     0x016018

void AP_Logger_DataFlash::Init()
{
    dev = hal.spi->get_device("dataflash");
    if (!dev) {
        AP_HAL::panic("PANIC: AP_Logger SPIDeviceDriver not found");
        return;
    }

    dev_sem = dev->get_semaphore();

    if (!getSectorCount()) {
        flash_died = true;
        return;
    }

    if (use_32bit_address) {
        Enter4ByteAddressMode();
    }

    flash_died = false;

    AP_Logger_Block::Init();

    //flash_test();
}

/*
  wait for busy flag to be cleared
 */
void AP_Logger_DataFlash::WaitReady()
{
    if (flash_died) {
        return;
    }

    uint32_t t = AP_HAL::millis();
    while (Busy()) {
        hal.scheduler->delay_microseconds(100);
        if (AP_HAL::millis() - t > 5000) {
            printf("DataFlash: flash_died\n");
            flash_died = true;
            break;
        }
    }
}

bool AP_Logger_DataFlash::getSectorCount(void)
{
    WaitReady();

    WITH_SEMAPHORE(dev_sem);

    // Read manufacturer ID
    uint8_t cmd = JEDEC_DEVICE_ID;
    uint8_t buf[4]; // buffer not yet allocated
    dev->transfer(&cmd, 1, buf, 4);

    uint32_t id = buf[0] << 16 | buf[1] << 8 | buf[2];

    uint32_t blocks = 0;

    switch (id) {
    case JEDEC_ID_WINBOND_W25Q16:
    case JEDEC_ID_MICRON_M25P16:
        blocks = 32;
        df_PagePerBlock = 256;
        df_PagePerSector = 16;
        break;
    case JEDEC_ID_WINBOND_W25Q32:
    case JEDEC_ID_WINBOND_W25X32:
    case JEDEC_ID_MACRONIX_MX25L3206E:
        blocks = 64;
        df_PagePerBlock = 256;
        df_PagePerSector = 16;
        break;
    case JEDEC_ID_MICRON_N25Q064:
    case JEDEC_ID_WINBOND_W25Q64:
    case JEDEC_ID_MACRONIX_MX25L6406E:
        blocks = 128;
        df_PagePerBlock = 256;
        df_PagePerSector = 16;
        break;
    case JEDEC_ID_MICRON_N25Q128:
    case JEDEC_ID_WINBOND_W25Q128:
    case JEDEC_ID_CYPRESS_S25FL128L:
        blocks = 256;
        df_PagePerBlock = 256;
        df_PagePerSector = 16;
        break;
    case JEDEC_ID_WINBOND_W25Q256:
    case JEDEC_ID_MACRONIX_MX25L25635E:
        blocks = 512;
        df_PagePerBlock = 256;
        df_PagePerSector = 16;
        use_32bit_address = true;
        break;
    default:
        hal.scheduler->delay(2000);
        printf("Unknown SPI Flash 0x%08x\n", id);
        return false;
    }

    df_PageSize = 256;
    df_NumPages = blocks * df_PagePerBlock;
    erase_cmd = JEDEC_BLOCK64_ERASE;

    printf("SPI Flash 0x%08x found pages=%u erase=%uk\n",
           id, df_NumPages, (df_PagePerBlock * (uint32_t)df_PageSize)/1024);
    return true;

}

// Read the status register
uint8_t AP_Logger_DataFlash::ReadStatusReg()
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd = JEDEC_READ_STATUS;
    uint8_t status;
    dev->transfer(&cmd, 1, &status, 1);
    return status;
}

bool AP_Logger_DataFlash::Busy()
{
    return (ReadStatusReg() & (JEDEC_STATUS_BUSY | JEDEC_STATUS_SRP0)) != 0;
}

void AP_Logger_DataFlash::Enter4ByteAddressMode(void)
{
    WITH_SEMAPHORE(dev_sem);

    const uint8_t cmd = 0xB7;
    dev->transfer(&cmd, 1, nullptr, 0);
}

/*
  send a command with an address
*/
void AP_Logger_DataFlash::send_command_addr(uint8_t command, uint32_t PageAdr)
{
    uint8_t cmd[5];
    cmd[0] = command;
    if (use_32bit_address) {
        cmd[1] = (PageAdr >> 24) & 0xff;
        cmd[2] = (PageAdr >> 16) & 0xff;
        cmd[3] = (PageAdr >>  8) & 0xff;
        cmd[4] = (PageAdr >>  0) & 0xff;
    } else {
        cmd[1] = (PageAdr >> 16) & 0xff;
        cmd[2] = (PageAdr >>  8) & 0xff;
        cmd[3] = (PageAdr >>  0) & 0xff;
    }

    dev->transfer(cmd, use_32bit_address?5:4, nullptr, 0);
}


void AP_Logger_DataFlash::PageToBuffer(uint32_t pageNum)
{
    if (pageNum == 0 || pageNum > df_NumPages+1) {
        printf("Invalid page read %u\n", pageNum);
        memset(buffer, 0xFF, df_PageSize);
        return;
    }
    WaitReady();

    uint32_t PageAdr = (pageNum-1) * df_PageSize;

    WITH_SEMAPHORE(dev_sem);
    dev->set_chip_select(true);
    send_command_addr(JEDEC_READ_DATA, PageAdr);
    dev->transfer(nullptr, 0, buffer, df_PageSize);
    dev->set_chip_select(false);
}

void AP_Logger_DataFlash::BufferToPage(uint32_t pageNum)
{
    if (pageNum == 0 || pageNum > df_NumPages+1) {
        printf("Invalid page write %u\n", pageNum);
        return;
    }
    WriteEnable();

    WITH_SEMAPHORE(dev_sem);

    uint32_t PageAdr = (pageNum-1) * df_PageSize;

    dev->set_chip_select(true);
    send_command_addr(JEDEC_PAGE_WRITE, PageAdr);
    dev->transfer(buffer, df_PageSize, nullptr, 0);
    dev->set_chip_select(false);
}

/*
  erase one sector (sizes varies with hw)
*/
void AP_Logger_DataFlash::SectorErase(uint32_t blockNum)
{
    WriteEnable();

    WITH_SEMAPHORE(dev_sem);

    uint32_t PageAdr = blockNum * df_PageSize * df_PagePerBlock;
    send_command_addr(erase_cmd, PageAdr);
}

/*
  erase one 4k sector
*/
void AP_Logger_DataFlash::Sector4kErase(uint32_t sectorNum)
{
    WriteEnable();

    WITH_SEMAPHORE(dev_sem);
    uint32_t SectorAddr = sectorNum * df_PageSize * df_PagePerSector;
    send_command_addr(JEDEC_SECTOR4_ERASE, SectorAddr);
}

void AP_Logger_DataFlash::StartErase()
{
    WriteEnable();

    WITH_SEMAPHORE(dev_sem);

    uint8_t cmd = JEDEC_BULK_ERASE;
    dev->transfer(&cmd, 1, nullptr, 0);

    erase_start_ms = AP_HAL::millis();
    printf("Dataflash: erase started\n");
}

bool AP_Logger_DataFlash::InErase()
{
    if (erase_start_ms && !Busy()) {
        printf("Dataflash: erase done (%u ms)\n", AP_HAL::millis() - erase_start_ms);
        erase_start_ms = 0;
    }
    return erase_start_ms != 0;
}

void AP_Logger_DataFlash::WriteEnable(void)
{
    WaitReady();
    WITH_SEMAPHORE(dev_sem);
    uint8_t b = JEDEC_WRITE_ENABLE;
    dev->transfer(&b, 1, nullptr, 0);
}

#endif // HAL_LOGGING_DATAFLASH_ENABLED
