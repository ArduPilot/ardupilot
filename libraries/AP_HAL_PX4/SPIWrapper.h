#include <AP_HAL/AP_HAL.h>

#include <arch/board/board.h>
#include "board_config.h"
#include <drivers/device/spi.h>
#include "AP_HAL_PX4.h"

extern const AP_HAL::HAL& hal;
/*
  wrapper class for SPI to expose protected functions from PX4Firmware
 */
class PX4::PX4_SPI : public device::SPI {
public:
    PX4_SPI(uint8_t bus, const char *name, const char *devname, enum spi_dev_e device, enum spi_mode_e mode, uint32_t frequency) :
        SPI(name, devname, bus, device, mode, frequency) {}
    
    bool do_transfer(uint8_t *send, uint8_t *recv, uint32_t len) {
        if (!init_done) {
            if (init() != 0) {
                return false;
            }
            init_done = true;
        }
        return transfer(send, recv, len) == 0;
    }
    void set_speed(uint32_t speed_hz) {
        set_frequency(speed_hz);
    }
private:
    bool init_done;
};

