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
 *
 * Code by @davidbuzz and Claude
 *
 * Zephyr port of AP_HAL_ChibiOS/Device.h DeviceBus — a per-bus thread
 * that runs sensor drivers' registered periodic callbacks with the bus
 * semaphore held. No DMA bounce buffers: the Zephyr SPI/I2C drivers
 * manage their own DMA-safe buffering internally.
 */
#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>

#include "Semaphores.h"

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#endif

/* bouncebuffer.h declares this at GLOBAL scope. Forward-declaring it here
   rather than inside namespace Zephyr matters: a bare "struct bouncebuffer_t *"
   member inside the namespace declares a NEW Zephyr::bouncebuffer_t, and
   bouncebuffer_init() then fails to convert Zephyr::bouncebuffer_t** to
   bouncebuffer_t**. */
struct bouncebuffer_t;

namespace Zephyr {

class DeviceBus {
public:
    DeviceBus(uint8_t bus_num, uint8_t bus_type);

    struct DeviceBus *next;
    Semaphore semaphore;

    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb, AP_HAL::Device *hal_device);
    bool adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec);
    static void bus_thread(void *arg1, void *arg2, void *arg3);

    bool bouncebuffer_setup(const uint8_t *&buf_tx, uint16_t tx_len,
                            uint8_t *&buf_rx, uint16_t rx_len) WARN_IF_UNUSED;
    void bouncebuffer_finish(const uint8_t *buf_tx, uint8_t *buf_rx, uint16_t rx_len);

    // grow-on-demand DMA-safe scratch pair for combining a send-then-recv
    // register transaction into ONE full-duplex chunk (SPIDevice::transfer).
    // Returned buffers live in the MEM_DMA_SAFE (__nocache) pool and stay
    // owned by this bus; only the bus semaphore holder may use them.
    bool xfer_scratch(uint16_t size, uint8_t *&tx, uint8_t *&rx) WARN_IF_UNUSED;

    // return (creating if needed) the shared DeviceBus for a bus number.
    // bus_type (AP_HAL::Device::BusType) keys the lookup so SPI bus N and
    // I2C bus N get separate threads and semaphores.
    static DeviceBus *get_bus(uint8_t bus_num, uint8_t bus_type);

private:
    struct callback_info {
        struct callback_info *next;
        AP_HAL::Device::PeriodicCb cb;
        uint32_t period_usec;
        uint64_t next_usec;
    } *callbacks;
    uint8_t bus_num;
    uint8_t bus_type;
    bool thread_started;
    AP_HAL::Device *hal_device;
#ifdef __ZEPHYR__
    struct k_thread thread_data;
#endif

    // support for bounce buffers for DMA-safe transfers
    ::bouncebuffer_t *bounce_buffer_tx;
    ::bouncebuffer_t *bounce_buffer_rx;
    // xfer_scratch() backing store (MEM_DMA_SAFE pool; zeroed by AP's
    // zeroing operator new)
    uint8_t *scratch_tx;
    uint8_t *scratch_rx;
    uint16_t scratch_size;
};

}
