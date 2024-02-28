#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SIM_RAMTRON_ENABLED
#define AP_SIM_RAMTRON_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if AP_SIM_RAMTRON_ENABLED

#include "SIM_SPIDevice.h"

namespace SITL {

class RAMTRON : public SPIDevice
{
public:

    int rdwr(uint8_t count, SPI::spi_ioc_transfer *&data) override;

protected:

    virtual void fill_rdid(uint8_t *buf, uint8_t len) = 0;
    virtual uint32_t storage_size() const = 0;  // in bytes
    virtual const char *filename() const = 0;

private:

    enum class State {
        WAITING,
        READING_RDID,
        READING,
        WRITING,
    } state = State::WAITING;

    bool write_enabled;
    uint32_t xfr_addr;

    void open_storage_fd();
    int storage_fd = -1;
};

}

#endif  // AP_SIM_RAMTRON_ENABLED
