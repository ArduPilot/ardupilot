/*
  logging for block based dataflash devices on SPI
 */
#pragma once

#include "AP_Logger_config.h"

#if HAL_LOGGING_FLASH_W25NXX_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "AP_Logger_Block.h"

class AP_Logger_W25NXX : public AP_Logger_Block {
public:
    AP_Logger_W25NXX(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
        AP_Logger_Block(front, writer) {}
    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_W25NXX(front, ls);
    }
    void              Init(void) override;
    bool              CardInserted() const override { return !flash_died && df_NumPages > 0; }

private:
    void              BufferToPage(uint32_t PageAdr) override;
    void              PageToBuffer(uint32_t PageAdr) override;
    void              SectorErase(uint32_t SectorAdr) override;
    void              Sector4kErase(uint32_t SectorAdr) override;
    void              StartErase() override;
    bool              InErase() override;
    void              send_command_addr(uint8_t cmd, uint32_t address);
    void              WaitReady();
    bool              Busy();
    uint8_t           ReadStatusRegBits(uint8_t bits);
    void              WriteStatusReg(uint8_t reg, uint8_t bits);

    void              WriteEnable();
    bool              getSectorCount(void);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    AP_HAL::Semaphore *dev_sem;

    uint32_t flash_blockNum;

    bool flash_died;
    uint32_t erase_start_ms;
    uint16_t erase_block;
    bool read_cache_valid;
};

#endif // HAL_LOGGING_FLASH_W25NXX_ENABLED
