/*
  block based logging backend for SITL, simulating a flash storage
  chip
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Logger_Block.h"

class AP_Logger_SITL : public AP_Logger_Block {
public:
    AP_Logger_SITL(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
        AP_Logger_Block(front, writer) {}
    void        Init() override;
    bool        CardInserted() const override;

private:
    void  BufferToPage(uint32_t PageAdr) override;
    void  PageToBuffer(uint32_t PageAdr) override;
    void  SectorErase(uint32_t SectorAdr) override;
    void  StartErase() override;
    bool  InErase() override;

    int flash_fd;
    uint32_t erase_started_ms;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
