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
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include <AP_HAL/CANIface.h>

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#if HAL_NUM_CAN_IFACES

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#endif

#ifndef HAL_CAN_RX_QUEUE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

namespace Zephyr {

class CANIface : public AP_HAL::CANIface
{
public:
    explicit CANIface(uint8_t index);

    // AP_HAL::CANIface pure-virtual overrides
    bool     init(const uint32_t bitrate) override;
    bool     is_initialized() const override { return _initialized; }

    int16_t  send(const AP_HAL::CANFrame &frame,
                  uint64_t tx_deadline,
                  CanIOFlags flags) override;

    int16_t  receive(AP_HAL::CANFrame &out_frame,
                     uint64_t &out_ts_monotonic,
                     CanIOFlags &out_flags) override;

    bool     select(bool &read_select, bool &write_select,
                    const AP_HAL::CANFrame *const pending_tx,
                    uint64_t timeout_us) override;

    bool     set_event_handle(AP_HAL::BinarySemaphore *handle) override;

    uint32_t getErrorCount() const override { return _error_count; }

    /* Backs @SYS/can0_stats.txt / can1_stats.txt. */
    void get_stats(ExpandingString &str) override;

protected:
    int8_t get_iface_num() const override { return static_cast<int8_t>(_index); }
    bool add_to_rx_queue(const CanRxItem &rx_item) override;

private:
    uint8_t _index;
    bool    _initialized = false;
    uint32_t _error_count = 0;

    // stats counters, reported by get_stats()
    uint32_t _rx_received = 0;   // frames accepted into the RX ring
    uint32_t _rx_overflow = 0;   // frames dropped: ring full
    uint32_t _tx_sent = 0;       // frames handed to the driver OK
    uint32_t _tx_full = 0;       // send() found driver queue full

#ifdef __ZEPHYR__
    const struct device *_dev = nullptr;

    // Rx ring buffer
    struct RxEntry {
        AP_HAL::CANFrame frame;
        uint64_t         timestamp_us;
        CanIOFlags       flags;
    };
    RxEntry  _rx_buf[HAL_CAN_RX_QUEUE_SIZE];
    uint16_t _rx_head = 0;
    uint16_t _rx_tail = 0;
    /* Spinlock, NOT k_mutex: rx_callback() runs in ISR context, where a mutex is
     * illegal. */
    struct k_spinlock _rx_lock;
    struct k_sem   _rx_sem;

    AP_HAL::BinarySemaphore *_event_handle = nullptr;

    // Zephyr RX callback (static, dispatched to instance via user_data)
    static void rx_callback(const struct device *dev,
                            struct can_frame *frame,
                            void *user_data);

    // Convert between AP_HAL and Zephyr CAN frame types
    static bool to_zephyr_frame(const AP_HAL::CANFrame &in,
                                 struct can_frame &out);
    static bool from_zephyr_frame(const struct can_frame &in,
                                   AP_HAL::CANFrame &out);
#endif  // __ZEPHYR__
};

}  // namespace Zephyr

#endif  // HAL_NUM_CAN_IFACES
#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
