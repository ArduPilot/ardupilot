/*
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>
#include "Semaphores.h"
#include "Scheduler.h"
#include "shared_dma.h"

namespace ChibiOS {

class DeviceBus {
public:
    DeviceBus(uint8_t _thread_priority = APM_I2C_PRIORITY) :
        thread_priority(_thread_priority) {}

    struct DeviceBus *next;
    Semaphore semaphore;
    Shared_DMA *dma_handle;

    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb, AP_HAL::Device *hal_device);
    bool adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec);
    static void bus_thread(void *arg);

    void bouncebuffer_setup(const uint8_t *&buf_tx, uint16_t tx_len,
                            uint8_t *&buf_rx, uint16_t rx_len);
    void bouncebuffer_rx_copy(uint8_t *buf_rx, uint16_t rx_len);
    
private:
    struct callback_info {
        struct callback_info *next;
        AP_HAL::Device::PeriodicCb cb;
        uint32_t period_usec;
        uint64_t next_usec;
    } *callbacks;
    uint8_t thread_priority;
    thread_t* thread_ctx;
    bool thread_started;
    AP_HAL::Device *hal_device;

    // support for bounce buffers for DMA-safe transfers
    uint8_t *bounce_buffer_tx;
    uint8_t *bounce_buffer_rx;
    uint16_t bounce_buffer_tx_size;
    uint16_t bounce_buffer_rx_size;
};

}
