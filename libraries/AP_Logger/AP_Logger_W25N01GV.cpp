/*
  logging to a DataFlash block based storage device on SPI
*/


#include <AP_HAL/AP_HAL.h>

#include "AP_Logger_W25N01GV.h"

#if HAL_LOGGING_DATAFLASH_ENABLED

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_WRITE_DISABLE          0x04
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_PAGE_DATA_READ         0x13
#define JEDEC_FAST_READ              0x0b
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02
#define JEDEC_PROGRAM_EXECUTE        0x10

#define JEDEC_DEVICE_RESET           0xFF
#define JEDEC_BLOCK_ERASE            0xD8 // 128K erase

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02

#define W25N01G_STATUS_REG           0xC0
#define W25N01G_PROT_REG             0xA0
#define W25N01G_CONF_REG             0xB0
#define W25N01G_STATUS_EFAIL         0x04
#define W25N01G_STATUS_PFAIL         0x08

#define W25N01G_PROT_SRP1_ENABLE          (1 << 0)
#define W25N01G_PROT_WP_E_ENABLE          (1 << 1)
#define W25N01G_PROT_TB_ENABLE            (1 << 2)
#define W25N01G_PROT_PB0_ENABLE           (1 << 3)
#define W25N01G_PROT_PB1_ENABLE           (1 << 4)
#define W25N01G_PROT_PB2_ENABLE           (1 << 5)
#define W25N01G_PROT_PB3_ENABLE           (1 << 6)
#define W25N01G_PROT_SRP2_ENABLE          (1 << 7)

#define W25N01G_CONFIG_ECC_ENABLE         (1 << 4)
#define W25N01G_CONFIG_BUFFER_READ_MODE   (1 << 3)

#define W25N01G_TIMEOUT_PAGE_READ_US        60   // tREmax = 60us (ECC enabled)
#define W25N01G_TIMEOUT_PAGE_PROGRAM_US     700  // tPPmax = 700us
#define W25N01G_TIMEOUT_BLOCK_ERASE_MS      10   // tBEmax = 10ms
#define W25N01G_TIMEOUT_RESET_MS            500  // tRSTmax = 500ms
#define W25N01G_NUM_BLOCKS                  1024

#define JEDEC_ID_WINBOND_W25N01GV      0xEFAA21

void AP_Logger_W25N01GV::Init()
{
    dev = hal.spi->get_device("dataflash");
    if (!dev) {
        AP_HAL::panic("PANIC: AP_Logger W25N01GV device not found");
        return;
    }

    dev_sem = dev->get_semaphore();

    if (!getSectorCount()) {
        flash_died = true;
        return;
    }

    flash_died = false;

    // reset the device
    WaitReady();
    {
        WITH_SEMAPHORE(dev_sem);
        uint8_t b = JEDEC_DEVICE_RESET;
        dev->transfer(&b, 1, nullptr, 0);
    }
    hal.scheduler->delay(W25N01G_TIMEOUT_RESET_MS);

    // disable write protection
    WriteStatusReg(W25N01G_PROT_REG, 0);
    // enable ECC and buffer mode
    WriteStatusReg(W25N01G_CONF_REG, W25N01G_CONFIG_ECC_ENABLE|W25N01G_CONFIG_BUFFER_READ_MODE);

    printf("W25N01GV status: SR-1=0x%x, SR-2=0x%x, SR-3=0x%x\n",
        ReadStatusRegBits(W25N01G_PROT_REG),
        ReadStatusRegBits(W25N01G_CONF_REG),
        ReadStatusRegBits(W25N01G_STATUS_REG));

    AP_Logger_Block::Init();
}

/*
  wait for busy flag to be cleared
 */
void AP_Logger_W25N01GV::WaitReady()
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

bool AP_Logger_W25N01GV::getSectorCount(void)
{
    WaitReady();

    WITH_SEMAPHORE(dev_sem);

    // Read manufacturer ID
    uint8_t cmd = JEDEC_DEVICE_ID;
    uint8_t buf[4]; // buffer not yet allocated
    dev->transfer(&cmd, 1, buf, 4);

    uint32_t id = buf[1] << 16 | buf[2] << 8 | buf[3];

    switch (id) {
    case JEDEC_ID_WINBOND_W25N01GV:
        df_PageSize = 2048;
        df_PagePerBlock = 64;
        df_PagePerSector = 64; // make sectors equivalent to block
        break;

    default:
        hal.scheduler->delay(2000);
        printf("Unknown SPI Flash 0x%08x\n", id);
        return false;
    }

    df_NumPages = W25N01G_NUM_BLOCKS * df_PagePerBlock;

    printf("SPI Flash 0x%08x found pages=%u\n", id, df_NumPages);
    return true;
}

// Read the status register bits
uint8_t AP_Logger_W25N01GV::ReadStatusRegBits(uint8_t bits)
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd[2] { JEDEC_READ_STATUS, bits };
    uint8_t status;
    dev->transfer(cmd, 2, &status, 1);
    return status;
}

void AP_Logger_W25N01GV::WriteStatusReg(uint8_t reg, uint8_t bits)
{
    WaitReady();
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd[3] = {JEDEC_WRITE_STATUS, reg, bits};
    dev->transfer(cmd, 3, nullptr, 0);
}

bool AP_Logger_W25N01GV::Busy()
{
    uint8_t status = ReadStatusRegBits(W25N01G_STATUS_REG);

    if ((status & W25N01G_STATUS_PFAIL) != 0) {
        printf("Program failure!\n");
    }
    if ((status & W25N01G_STATUS_EFAIL) != 0) {
        printf("Erase failure!\n");
    }

    return (status & JEDEC_STATUS_BUSY) != 0;
}

/*
  send a command with an address
*/
void AP_Logger_W25N01GV::send_command_addr(uint8_t command, uint32_t PageAdr)
{
    uint8_t cmd[4];
    cmd[0] = command;
    cmd[1] = 0;    // dummy
    cmd[2] = (PageAdr >>  8) & 0xff;
    cmd[3] = (PageAdr >>  0) & 0xff;

    dev->transfer(cmd, 4, nullptr, 0);
}

void AP_Logger_W25N01GV::PageToBuffer(uint32_t pageNum)
{
    if (pageNum == 0 || pageNum > df_NumPages+1) {
        printf("Invalid page read %u\n", pageNum);
        memset(buffer, 0xFF, df_PageSize);
        df_Read_PageAdr = pageNum;
        return;
    }

    // we already just read this page
    if (pageNum == df_Read_PageAdr && read_cache_valid) {
        return;
    }

    df_Read_PageAdr = pageNum;

    WaitReady();

    uint32_t PageAdr = (pageNum-1);

    {
        WITH_SEMAPHORE(dev_sem);
        // read page into internal buffer
        send_command_addr(JEDEC_PAGE_DATA_READ, PageAdr);
    }

    // read from internal buffer into our buffer
    WaitReady();
    {
        WITH_SEMAPHORE(dev_sem);
        dev->set_chip_select(true);
        uint8_t cmd[4];
        cmd[0] = JEDEC_READ_DATA;
        cmd[1] = (0 >>  8) & 0xff; // column address zero
        cmd[2] = (0 >>  0) & 0xff; // column address zero
        cmd[3] = 0; // dummy
        dev->transfer(cmd, 4, nullptr, 0);
        dev->transfer(nullptr, 0, buffer, df_PageSize);
        dev->set_chip_select(false);

        read_cache_valid = true;
    }
}

void AP_Logger_W25N01GV::BufferToPage(uint32_t pageNum)
{
    if (pageNum == 0 || pageNum > df_NumPages+1) {
        printf("Invalid page write %u\n", pageNum);
        return;
    }

    // just wrote the cached page
    if (pageNum != df_Read_PageAdr) {
        read_cache_valid = false;
    }

    WriteEnable();

    uint32_t PageAdr = (pageNum-1);
    {
        WITH_SEMAPHORE(dev_sem);

        // write our buffer into internal buffer
        dev->set_chip_select(true);

        uint8_t cmd[3];
        cmd[0] = JEDEC_PAGE_WRITE;
        cmd[1] = (0 >>  8) & 0xff; // column address zero
        cmd[2] = (0 >>  0) & 0xff; // column address zero

        dev->transfer(cmd, 3, nullptr, 0);
        dev->transfer(buffer, df_PageSize, nullptr, 0);
        dev->set_chip_select(false);
    }

    // write from internal buffer into page
    {
        WITH_SEMAPHORE(dev_sem);
        send_command_addr(JEDEC_PROGRAM_EXECUTE, PageAdr);
    }
}

/*
  erase one sector (sizes varies with hw)
*/
void AP_Logger_W25N01GV::SectorErase(uint32_t blockNum)
{
    WriteEnable();
    WITH_SEMAPHORE(dev_sem);

    uint32_t PageAdr = blockNum  * df_PagePerBlock;
    send_command_addr(JEDEC_BLOCK_ERASE, PageAdr);
}

/*
  erase one 4k sector
*/
void AP_Logger_W25N01GV::Sector4kErase(uint32_t sectorNum)
{
    SectorErase(sectorNum);
}

void AP_Logger_W25N01GV::StartErase()
{
    WriteEnable();

    WITH_SEMAPHORE(dev_sem);

    // just erase the first block, others will follow in InErase
    send_command_addr(JEDEC_BLOCK_ERASE, 0);

    erase_block = 1;
    erase_start_ms = AP_HAL::millis();
    printf("Dataflash: erase started\n");
}

bool AP_Logger_W25N01GV::InErase()
{
    if (erase_start_ms && !Busy()) {
        if (erase_block < W25N01G_NUM_BLOCKS) {
            SectorErase(erase_block++);
        } else {
            printf("Dataflash: erase done (%u ms)\n", AP_HAL::millis() - erase_start_ms);
            erase_start_ms = 0;
            erase_block = 0;
        }
    }
    return erase_start_ms != 0;
}

void AP_Logger_W25N01GV::WriteEnable(void)
{
    WaitReady();
    WITH_SEMAPHORE(dev_sem);
    uint8_t b = JEDEC_WRITE_ENABLE;
    dev->transfer(&b, 1, nullptr, 0);
}

#endif // HAL_LOGGING_DATAFLASH_ENABLED
