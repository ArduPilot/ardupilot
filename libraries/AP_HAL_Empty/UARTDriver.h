#pragma once

#include "AP_HAL_Empty.h"

class Empty::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver();
    /* Empty implementations of UARTDriver virtual methods */
    bool is_initialized() override;
    bool tx_pending() override;

    /* Empty implementations of Stream virtual methods */
    uint32_t txspace() override;


#if HAL_UART_STATS_ENABLED
    // request information on uart I/O for one uart
    void uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms) override;
#endif

protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t size) override WARN_IF_UNUSED;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;
};
