/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by @davidbuzz and Claude
 *
 * Zephyr port of AP_HAL_ChibiOS/Device.cpp DeviceBus.
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "DeviceBus.h"
#include "bouncebuffer.h"
#include "chain_profile.h"
#include "Scheduler.h"   // APM_SPI_PRIORITY / APM_I2C_PRIORITY

/* Bus threads take the priority of the BUS TYPE, exactly as AP_HAL_ChibiOS does. */

#include <new>
#include <stdio.h>

using namespace Zephyr;

extern const AP_HAL::HAL& hal;

#ifdef __ZEPHYR__

#define HAL_ZEPHYR_DEVICE_STACK_SIZE 4096  // bytes, per bus thread
#define HAL_ZEPHYR_MAX_DEVICE_BUSES  4

/* Bus-thread stacks in DTCM - see the stack definitions in Scheduler.h. */
#ifdef CONFIG_ARM
#define AP_DEVICE_BUS_STACK_SECTION __dtcm_noinit_section
#else
#define AP_DEVICE_BUS_STACK_SECTION __noinit
#endif
// _device_bus_stacks     0x20023200  DTCM ✅
Z_KERNEL_STACK_ARRAY_DEFINE_IN(_device_bus_stacks, HAL_ZEPHYR_MAX_DEVICE_BUSES,
                               HAL_ZEPHYR_DEVICE_STACK_SIZE,
                               AP_DEVICE_BUS_STACK_SECTION);
static uint8_t _device_bus_count;

// shared bus list, one DeviceBus per hardware bus number
static DeviceBus *_buses;

#endif  // __ZEPHYR__

DeviceBus::DeviceBus(uint8_t _bus_num, uint8_t _bus_type) :
    bus_num(_bus_num),
    bus_type(_bus_type)
{
    /* 10-byte prealloc matches AP_HAL_ChibiOS/Device.cpp:40-41; the buffers
       grow on demand. ChibiOS passes a third "axi_sram" argument to pick the
       memory region - this HAL has no such choice, malloc_type(MEM_DMA_SAFE)
       always serves the __nocache pool, so the parameter does not exist. */
    bouncebuffer_init(&bounce_buffer_tx, 10);
    bouncebuffer_init(&bounce_buffer_rx, 10);
}

bool DeviceBus::xfer_scratch(uint16_t size, uint8_t *&tx, uint8_t *&rx)
{
    if (size > scratch_size) {
        if (scratch_size > 0) {
            hal.util->free_type(scratch_tx, scratch_size, AP_HAL::Util::MEM_DMA_SAFE);
            hal.util->free_type(scratch_rx, scratch_size, AP_HAL::Util::MEM_DMA_SAFE);
            scratch_size = 0;
        }
        scratch_tx = (uint8_t *)hal.util->malloc_type(size, AP_HAL::Util::MEM_DMA_SAFE);
        scratch_rx = (uint8_t *)hal.util->malloc_type(size, AP_HAL::Util::MEM_DMA_SAFE);
        if (scratch_tx == nullptr || scratch_rx == nullptr) {
            hal.util->free_type(scratch_tx, size, AP_HAL::Util::MEM_DMA_SAFE);
            hal.util->free_type(scratch_rx, size, AP_HAL::Util::MEM_DMA_SAFE);
            scratch_tx = scratch_rx = nullptr;
            return false;
        }
        scratch_size = size;
    }
    tx = scratch_tx;
    rx = scratch_rx;
    return true;
}

/*
  setup to use DMA-safe bouncebuffers for device transfers
 */
bool DeviceBus::bouncebuffer_setup(const uint8_t *&buf_tx, uint16_t tx_len,
                                   uint8_t *&buf_rx, uint16_t rx_len)
{
#ifndef CONFIG_SPI_NXP_LPSPI_DMA
    /* NO DMA IN THIS BUILD -> NO BOUNCE BUFFER. Skip entirely rather than staging
     * through a buffer nothing will DMA from. */
    (void)tx_len; (void)rx_len;
    return true;
#else
    if (buf_rx) {
        if (!bouncebuffer_setup_read(bounce_buffer_rx, &buf_rx, rx_len)) {
            return false;
        }
    }
    if (buf_tx) {
        if (!bouncebuffer_setup_write(bounce_buffer_tx, &buf_tx, tx_len)) {
            if (buf_rx) {
                bouncebuffer_abort(bounce_buffer_rx);
            }
            return false;
        }
    }
    return true;
#endif
}

/*
  complete a transfer using DMA bounce buffer
 */
void DeviceBus::bouncebuffer_finish(const uint8_t *buf_tx, uint8_t *buf_rx, uint16_t rx_len)
{
#ifndef CONFIG_SPI_NXP_LPSPI_DMA
    (void)buf_tx; (void)buf_rx; (void)rx_len;
    return;
#else
    if (buf_rx) {
        bouncebuffer_finish_read(bounce_buffer_rx, buf_rx, rx_len);
    }
    if (buf_tx) {
        bouncebuffer_finish_write(bounce_buffer_tx, buf_tx);
    }
#endif
}

DeviceBus *DeviceBus::get_bus(uint8_t bus_num, uint8_t bus_type)
{
#ifdef __ZEPHYR__
    for (DeviceBus *b = _buses; b; b = b->next) {
        if (b->bus_num == bus_num && b->bus_type == bus_type) {
            return b;
        }
    }
    DeviceBus *b = NEW_NOTHROW DeviceBus(bus_num, bus_type);
    if (b == nullptr) {
        return nullptr;
    }
    b->next = _buses;
    _buses = b;
    return b;
#else
    (void)bus_type;
    return nullptr;
#endif
}

/*
  per-bus callback thread
*/
void DeviceBus::bus_thread(void *arg1, void *arg2, void *arg3)
{
    struct DeviceBus *binfo = (struct DeviceBus *)arg1;
    (void)arg2;
    (void)arg3;

    while (true) {
        uint64_t now = AP_HAL::micros64();
        DeviceBus::callback_info *callback;

        // find a callback to run
        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (now >= callback->next_usec) {
                while (now >= callback->next_usec) {
                    callback->next_usec += callback->period_usec;
                }
                // call it with semaphore held
                WITH_SEMAPHORE(binfo->semaphore);
                {
                    /* Slot 0-2 = SPI bus 1-3, slot 3-5 = I2C bus 1-3.
                       Previously indexed on bus_num ALONE, so SPI bus 2 and I2C
                       bus 2 incremented the SAME counter - which made a measured
                       "bus 2 = 655 Hz" unattributable between an IMU and a baro,
                       and cost real diagnostic time on 2026-08-05. */
                    const uint8_t prof_slot =
                        ((binfo->bus_type == AP_HAL::Device::BUS_TYPE_I2C) ? 3U : 0U)
                        + ((binfo->bus_num - 1U) % 3U);
                    AP_PHASE_BUSN(prof_slot, AP_PHASE_BUS_CB);
                    AP_PROF_TICK(AP_PROF_BUSCB_COUNT);
                    AP_PROF_TICK(AP_PROF_BUSCNT0 + (prof_slot % AP_PROF_MAX_BUSES));
                    callback->cb();
                }
            }
        }

        // work out when next loop is needed
        uint64_t next_needed = 0;
        now = AP_HAL::micros64();

        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (next_needed == 0 ||
                callback->next_usec < next_needed) {
                next_needed = callback->next_usec;
                if (next_needed < now) {
                    next_needed = now;
                }
            }
        }

        // delay for at most 50ms, to handle newly added callbacks
        uint32_t delay = 50000;
        if (next_needed >= now && next_needed - now < delay) {
            delay = next_needed - now;
        }
        // don't delay for less than 100usec, so one thread doesn't
        // completely dominate the CPU
        if (delay < 100) {
            delay = 100;
        }
#ifdef __ZEPHYR__
        /* Must genuinely sleep — busy-waiting at this priority starves all
         * lower-priority threads (main/io/storage). k_usleep rounds up to
         * the tick granularity; CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000 keeps
         * that rounding to ≤100us for 1kHz sensor callbacks. */
        k_usleep(delay);
#else
        hal.scheduler->delay_microseconds(delay);
#endif
    }
}

AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb, AP_HAL::Device *_hal_device)
{
#ifdef CONFIG_AP_CHAIN_PROFILE
    /* One line per registration: the SUM of 1e6/period over these lines is the
       callback rate the bus threads should produce. Compare against the
       measured AP_PROF_BUSCB_COUNT rate - a large excess means the bus loop is
       firing faster than programmed. */
    printk("BUSREG bus=%u type=%u period_us=%lu -> %lu Hz\n",
           (unsigned)bus_num, (unsigned)bus_type,
           (unsigned long)period_usec,
           (unsigned long)(period_usec ? 1000000UL/period_usec : 0));
#endif
#ifdef __ZEPHYR__
    if (!thread_started) {
        if (_device_bus_count >= HAL_ZEPHYR_MAX_DEVICE_BUSES) {
            return nullptr;
        }
        thread_started = true;

        hal_device = _hal_device;
        // setup a name for the thread
        const uint8_t name_len = 7;
        char *name = (char *)malloc(name_len);
        if (name == nullptr) {
            return nullptr;
        }
        switch (hal_device->bus_type()) {
        case AP_HAL::Device::BUS_TYPE_I2C:
            snprintf(name, name_len, "I2C%u",
                     hal_device->bus_num());
            break;

        case AP_HAL::Device::BUS_TYPE_SPI:
            snprintf(name, name_len, "SPI%u",
                     hal_device->bus_num());
            break;
        default:
            break;
        }

        const uint8_t stack_idx = _device_bus_count++;
        k_tid_t tid = k_thread_create(&thread_data,
                                      _device_bus_stacks[stack_idx],
                                      K_THREAD_STACK_SIZEOF(_device_bus_stacks[stack_idx]),
                                      DeviceBus::bus_thread,
                                      this, nullptr, nullptr,
                                      (bus_type == AP_HAL::Device::BUS_TYPE_SPI)
                                          ? APM_SPI_PRIORITY : APM_I2C_PRIORITY,
                                      0, K_NO_WAIT);
        if (tid == nullptr) {
            AP_HAL::panic("Failed to create bus thread %s", name);
        }
        k_thread_name_set(tid, name);
    }
    DeviceBus::callback_info *callback = NEW_NOTHROW DeviceBus::callback_info;
    if (callback == nullptr) {
        return nullptr;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    // add to linked list of callbacks on thread
    callback->next = callbacks;
    callbacks = callback;

    return callback;
#else
    (void)period_usec;
    (void)cb;
    (void)_hal_device;
    return nullptr;
#endif
}

/*
 * Adjust the timer for the next call: it needs to be called from the bus
 * thread, otherwise it will race with it
 */
bool DeviceBus::adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
#ifdef __ZEPHYR__
    if (k_current_get() != &thread_data) {
        return false;
    }

    DeviceBus::callback_info *callback = static_cast<DeviceBus::callback_info *>(h);

    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    return true;
#else
    (void)h;
    (void)period_usec;
    return false;
#endif
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
