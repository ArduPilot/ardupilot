#pragma once

#include "AP_HAL_Linux.h"

#include "UARTDriver.h"

namespace Linux {

class SPIUARTDriver : public UARTDriver {
public:
    SPIUARTDriver();
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void _timer_tick(void) override;
    uint32_t get_baud_rate() const override {
        return high_speed_set ? 4000000U : 1000000U;
    }

protected:
    int _write_fd(const uint8_t *buf, uint16_t n) override;
    int _read_fd(uint8_t *buf, uint16_t n) override;

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    uint8_t *_buffer;

    uint32_t _last_update_timestamp;

    bool _external;

    bool high_speed_set;
};

}
