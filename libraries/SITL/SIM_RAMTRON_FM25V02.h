#pragma once

#include "SIM_config.h"

#if AP_SIM_RAMTRON_FM25V02_ENABLED

#include "SIM_RAMTRON.h"

namespace SITL {

class RAMTRON_FM25V02 : public RAMTRON
{
protected:

    void fill_rdid(uint8_t *buf, uint8_t len) override;

    uint8_t id1() { return family << 5 | density; }
    uint8_t id2() { return sub << 5 | rev << 2; }

    const char *filename() const override { return "RAMTRON-FM25V02.dat"; }

    uint32_t storage_size() const override {
        return 32768;
    }

private:

    const uint8_t manufacturer[7] { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2 };
    static const uint8_t family = 1;
    static const uint8_t density = 2;
    static const uint8_t sub = 0;
    static const uint8_t rev = 0;
    static const uint8_t rsvd = 0;

};

}

#endif // AP_SIM_RAMTRON_FM25V02_ENABLED
