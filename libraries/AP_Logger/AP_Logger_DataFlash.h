/*
  logging for block based dataflash devices on SPI
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifdef HAL_LOGGING_DATAFLASH
#include "AP_Logger_Backend.h"
#include "AP_Logger_Block.h"


class AP_Logger_DataFlash : public AP_Logger_Block {
public:
    AP_Logger_DataFlash(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
        AP_Logger_Block(front, writer) {}
    void              Init(void) override;
    bool              CardInserted() const override { return !flash_died && df_NumPages > 0; }

private:
    void              BufferToPage(uint32_t PageAdr) override;
    void              PageToBuffer(uint32_t PageAdr) override;
    void              SectorErase(uint32_t SectorAdr) override;
    void              StartErase() override;
    bool              InErase() override;
    void              send_command_addr(uint8_t cmd, uint32_t address);
    void              WaitReady();
    bool              Busy();
    uint8_t           ReadStatusReg();
    
    void              WriteEnable();
    bool              getSectorCount(void);
    void              flash_test(void);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    AP_HAL::Semaphore *dev_sem;

    bool flash_died;
    uint32_t erase_start_ms;
    uint8_t erase_cmd;
    bool use_32bit_address;
};

#endif // HAL_LOGGING_DATAFLASH
