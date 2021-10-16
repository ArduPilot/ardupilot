#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SIM_JEDEC_ENABLED
#define AP_SIM_JEDEC_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if AP_SIM_JEDEC_ENABLED

#include "SIM_SPIDevice.h"

namespace SITL {

class JEDEC : public SPIDevice
{
public:

    int rdwr(uint8_t count, SPI::spi_ioc_transfer *&data) override;

protected:

    virtual void fill_rdid(uint8_t *buf, uint8_t len) = 0;
    virtual void fill_rdsr(uint8_t *buf, uint8_t len) = 0;
    virtual const char *filename() const = 0;
    virtual uint8_t get_num_blocks() const = 0;
    virtual uint16_t get_page_per_block() const = 0;
    virtual uint8_t get_page_per_sector() const = 0;
    virtual uint16_t get_page_size() const = 0;
    uint32_t get_storage_size() const { return get_num_pages()*get_page_size(); } // in bytes
    uint32_t get_num_pages() const { return get_num_blocks()*get_page_per_block(); }

private:

    enum class State {
        WAITING,
        READING_RDID,
        READING,
        WRITING,
        READING_RDSR
    } state = State::WAITING;

    bool write_enabled;
    uint32_t xfr_addr;

    void sector4k_erase(uint32_t addr);
    void block64k_erase(uint32_t addr);
    void page_erase(uint32_t addr);
    void bulk_erase();
    uint32_t parse_addr(uint8_t *buffer, uint32_t len);
    void assert_writes_enabled();
    void open_storage_fd();
    int storage_fd = -1;
};

}

#endif  // AP_SIM_JEDEC_ENABLED
