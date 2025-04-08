#pragma once

#include "SIM_JEDEC.h"

#ifndef AP_SIM_JEDEC_MX25L3206E_ENABLED
#define AP_SIM_JEDEC_MX25L3206E_ENABLED AP_SIM_JEDEC_ENABLED
#endif

#if AP_SIM_JEDEC_MX25L3206E_ENABLED

namespace SITL {

class JEDEC_MX25L3206E : public JEDEC
{
protected:

    void fill_rdid(uint8_t *buf, uint8_t len) override;
    void fill_rdsr(uint8_t *buf, uint8_t len) override;

    const char *filename() const override { return "blackbox.bin"; }
    uint8_t get_num_blocks() const override { return 64; }
    uint16_t get_page_per_block() const override { return 256; }
    uint8_t get_page_per_sector() const override { return 16; }
    uint16_t get_page_size() const override { return 256; }

private:

    static const uint8_t type = 0x20;
    static const uint8_t density = 0x16;
};

}

#endif  // AP_SIM_JEDEC_MX25L3206E_ENABLED
