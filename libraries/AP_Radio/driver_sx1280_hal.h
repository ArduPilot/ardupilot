/*
  Low-level SX1280 SPI/GPIO transaction driver.

  Ported from the ExpressLRS project (lib/SX1280Driver/SX1280_hal.cpp/.h,
  https://github.com/ExpressLRS/ExpressLRS, GPLv3), which itself derives
  from Semtech's reference driver (Revised BSD License). This only covers
  the generic SPI command/register/buffer framing and reset/busy handling
  for a single radio - it does not implement LoRa configuration, the ELRS
  OTA packet format, FHSS hopping, or binding. The latter are implemented
  by AP_ELRS and the AP_Periph receiver adapter.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "driver_sx1280_config.h"
#include "driver_sx1280_regs.h"

#if AP_RADIO_SX1280_ENABLED

class AP_SX1280_HAL
{
public:
    AP_SX1280_HAL();

    CLASS_NO_COPY(AP_SX1280_HAL);

    // initialise the SPI device and GPIOs, returns false if the SPI device isn't found
    bool init(void);

    // hardware reset via the RESET pin, then wait for BUSY to clear
    void reset(void);

    void write_command(SX1280_RadioCommands_t command, uint8_t val, uint32_t busy_delay_us = 15);
    void write_command(SX1280_RadioCommands_t command, const uint8_t *buffer, uint8_t size, uint32_t busy_delay_us = 15);
    uint8_t read_command(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size);

    void write_register(uint16_t address, uint8_t value);
    void write_register(uint16_t address, const uint8_t *buffer, uint8_t size);
    uint8_t read_register(uint16_t address);
    void read_register(uint16_t address, uint8_t *buffer, uint8_t size);

    void write_buffer(uint8_t offset, const uint8_t *buffer, uint8_t size);
    void read_buffer(uint8_t offset, uint8_t *buffer, uint8_t size);

    // wait for the BUSY pin to go low, returns false on timeout
    bool wait_on_busy(void);

    bool irq_pending(void) const;
    bool healthy(void) const
    {
        return io_ok;
    }

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    bool io_ok = false;
};

#endif  // AP_RADIO_SX1280_ENABLED
