#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <arch/board/board.h>
#include "board_config.h"
#include <drivers/device/i2c.h>
#include "AP_HAL_PX4.h"

extern const AP_HAL::HAL& hal;
/*
  wrapper class for I2C to expose protected functions from PX4Firmware
 */
class PX4::PX4_I2C : public device::I2C {
public:
    PX4_I2C(uint8_t bus) : I2C(devname, devpath, bus, 0, 400000UL) { }
    bool do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len);

private:
    static uint8_t instance;
    bool init_done;
    bool init_ok;
    char devname[10];
    char devpath[14];
};


#endif // CONFIG_HAL_BOARD
