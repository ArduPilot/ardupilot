#pragma once

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
    PX4_I2C(uint8_t bus);
    bool do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len, bool split_transfers);

    void set_retries(uint8_t retries) {
        _retries = retries;
    }

    uint8_t map_bus_number(uint8_t bus) const;

    // setup instance_lock
    static void init_lock(void) {
        pthread_mutex_init(&instance_lock, nullptr);
    }
    
private:
    static uint8_t instance;
    static pthread_mutex_t instance_lock;
    bool init_done;
    bool init_ok;
    char devname[11];
    char devpath[15];
};


#endif // CONFIG_HAL_BOARD
