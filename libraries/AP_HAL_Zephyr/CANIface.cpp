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
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "CANIface.h"
#include <AP_Common/ExpandingString.h>   // get_stats() writes into one

#if HAL_NUM_CAN_IFACES

using namespace Zephyr;

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>

// Device node labels for the two FlexCAN controllers
static const struct device *_can_devs[2];
#endif  // __ZEPHYR__

CANIface::CANIface(uint8_t index) :
    _index(index)
{
#ifdef __ZEPHYR__
    /* _rx_lock (k_spinlock) needs no init; see its declaration for why it
       replaced the k_mutex that used to be initialised here. */
    k_sem_init(&_rx_sem, 0, HAL_CAN_RX_QUEUE_SIZE);
#endif
}

bool CANIface::init(const uint32_t bitrate)
{
#ifdef __ZEPHYR__
    if (_index >= HAL_NUM_CAN_IFACES) {
        return false;
    }

    /* Resolve device at runtime to avoid static-init ordering issues.

       Prefer the can0/can1 devicetree ALIASES. This used to name flexcan1 and
       flexcan2 directly, which are NXP node labels: on an STM32 board the
       controllers are fdcan1/fdcan2, both lookups returned null, and CAN was
       silently dead however the board was wired or configured. An alias lets
       each board point at its own controllers and keeps this file free of
       per-SoC names. The flexcan labels stay as a fallback so boards whose
       devicetree predates the aliases are unaffected. */
    if (_index == 0) {
#if DT_NODE_EXISTS(DT_ALIAS(can0))
        _dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(can0));
#else
        _dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(flexcan1));
#endif
    } else {
#if DT_NODE_EXISTS(DT_ALIAS(can1))
        _dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(can1));
#else
        _dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(flexcan2));
#endif
    }

    if (_dev == nullptr || !device_is_ready(_dev)) {
        return false;
    }

    if (can_set_bitrate(_dev, bitrate) != 0) {
        return false;
    }

    if (can_start(_dev) != 0) {
        return false;
    }

    /* Two wildcard (pass-all) filters, one per ID format. A filter with
       CAN_FILTER_IDE set matches EXTENDED frames only, and one without it
       matches STANDARD frames only - they are not alternatives.
       Installing the standard one only as a FALLBACK for a failed extended
       one meant that on any controller which accepted the extended filter -
       i.e. the normal case - no standard-ID frame was ever received.
       Succeed if at least one lands, so a controller with a single filter
       slot still works. */
    struct can_filter filter_ext = {
        .id    = 0U,
        .mask  = 0U,
        .flags = CAN_FILTER_IDE,
    };
    struct can_filter filter_std = {
        .id    = 0U,
        .mask  = 0U,
        .flags = 0,
    };
    const int filter_ext_id = can_add_rx_filter(_dev, rx_callback, this, &filter_ext);
    const int filter_std_id = can_add_rx_filter(_dev, rx_callback, this, &filter_std);
    if (filter_ext_id < 0 && filter_std_id < 0) {
        can_stop(_dev);
        return false;
    }

    bitrate_ = bitrate;
    _initialized = true;
    return true;
#else
    (void)bitrate;
    return false;
#endif
}

// ─── TX ──────────────────────────────────────────────────────────────────

int16_t CANIface::send(const AP_HAL::CANFrame &frame,
                       uint64_t tx_deadline,
                       CanIOFlags flags)
{
    if (frame.isErrorFrame() || frame.dlc > 8U) {
        return -1;
    }

#ifdef __ZEPHYR__
    if (!_initialized) {
        return -1;
    }

    struct can_frame zframe;
    if (!to_zephyr_frame(frame, zframe)) {
        return -1;
    }

    // Use a non-blocking send; Zephyr will queue internally
    int ret = can_send(_dev, &zframe, K_NO_WAIT, nullptr, nullptr);
    if (ret == -EAGAIN || ret == -ENOMEM) {
        _tx_full++;
        return 0;  // No space right now
    }
    if (ret != 0) {
        _error_count++;
        return -1;
    }
    _tx_sent++;
#endif  // __ZEPHYR__

    // Call base for callback dispatch (forwarded frames etc.)
    return AP_HAL::CANIface::send(frame, tx_deadline, flags);
}

// ─── RX ──────────────────────────────────────────────────────────────────

int16_t CANIface::receive(AP_HAL::CANFrame &out_frame,
                          uint64_t &out_ts_monotonic,
                          CanIOFlags &out_flags)
{
#ifdef __ZEPHYR__
    if (!_initialized) {
        return -1;
    }

    k_spinlock_key_t key = k_spin_lock(&_rx_lock);
    if (_rx_head == _rx_tail) {
        k_spin_unlock(&_rx_lock, key);
        return 0;  // Nothing available
    }

    const RxEntry &entry = _rx_buf[_rx_head];
    out_frame         = entry.frame;
    out_ts_monotonic  = entry.timestamp_us;
    out_flags         = entry.flags;
    _rx_head = (_rx_head + 1U) % HAL_CAN_RX_QUEUE_SIZE;
    k_spin_unlock(&_rx_lock, key);

    // Decrement the semaphore count to match the consumed slot
    k_sem_take(&_rx_sem, K_NO_WAIT);

    // Call base for callback dispatch
    return AP_HAL::CANIface::receive(out_frame, out_ts_monotonic, out_flags);
#else
    (void)out_frame; (void)out_ts_monotonic; (void)out_flags;
    return -1;
#endif
}

// ─── select ──────────────────────────────────────────────────────────────

bool CANIface::select(bool &read_select, bool &write_select,
                      const AP_HAL::CANFrame *const pending_tx,
                      uint64_t blocking_deadline)
{
#ifdef __ZEPHYR__
    if (!_initialized) {
        return false;
    }

    bool rx_available = (_rx_head != _rx_tail);
    if (read_select && rx_available) {
        read_select  = true;
        write_select = true;
        return true;
    }

    if (!read_select && !write_select) {
        return false;
    }

    /* blocking_deadline is an ABSOLUTE micros64() timestamp, not a duration.
       ChibiOS names it that and waits blocking_deadline - micros64()
       (CANFDIface.cpp). Passing it straight to K_USEC() treated a timestamp
       as a delay, so a caller asking to wait 1 ms at 200 s uptime waited
       200 seconds - long enough that the CAN thread never came back on any
       normal timescale. */
    const uint64_t now_us = AP_HAL::micros64();
    k_timeout_t wait = K_NO_WAIT;
    if (blocking_deadline > now_us) {
        wait = K_USEC((int64_t)(blocking_deadline - now_us));
    }
    bool got = (k_sem_take(&_rx_sem, wait) == 0);
    if (got) {
        k_sem_give(&_rx_sem);  // restore count for receive()
    }

    read_select  = got || (_rx_head != _rx_tail);
    write_select = true;  // TX path never blocks in this driver
    return read_select || write_select;
#else
    (void)read_select; (void)write_select;
    (void)pending_tx; (void)blocking_deadline;
    return false;
#endif
}

bool CANIface::set_event_handle(AP_HAL::BinarySemaphore *handle)
{
#ifdef __ZEPHYR__
    _event_handle = handle;
#else
    (void)handle;
#endif
    return true;
}

// ─── Internal RX queue helper ─────────────────────────────────────────────

bool CANIface::add_to_rx_queue(const CanRxItem &rx_item)
{
#ifdef __ZEPHYR__
    /* Runs in ISR context (called from rx_callback, which Zephyr's CAN
       filter machinery dispatches from the FlexCAN IRQ handler). Every
       primitive used here must be ISR-safe: k_spin_lock and k_sem_give
       both are; the k_mutex this used to take was NOT - see the _rx_lock
       declaration in CANIface.h for the bench failure that found it. */
    k_spinlock_key_t lock_key = k_spin_lock(&_rx_lock);
    uint16_t next = (uint16_t)((_rx_tail + 1U) % HAL_CAN_RX_QUEUE_SIZE);
    if (next == _rx_head) {
        _error_count++;
        _rx_overflow++;
        k_spin_unlock(&_rx_lock, lock_key);
        return false;
    }
    _rx_buf[_rx_tail].frame        = rx_item.frame;
    _rx_buf[_rx_tail].timestamp_us = rx_item.timestamp_us;
    _rx_buf[_rx_tail].flags        = rx_item.flags;
    _rx_tail = next;
    _rx_received++;
    k_spin_unlock(&_rx_lock, lock_key);
    k_sem_give(&_rx_sem);

    if (_event_handle != nullptr) {
        _event_handle->signal();
    }
    return true;
#else
    (void)rx_item;
    return false;
#endif
}

// ─── stats (@SYS/canN_stats.txt) ─────────────────────────────────────────

void CANIface::get_stats(ExpandingString &str)
{
#ifdef __ZEPHYR__
    str.printf("iface%u initialized=%u bitrate=%lu\n"
               "rx_received %lu\nrx_overflow %lu\n"
               "tx_sent %lu\ntx_full %lu\nsw_error_count %lu\n",
               (unsigned)_index, (unsigned)_initialized, (unsigned long)bitrate_,
               (unsigned long)_rx_received, (unsigned long)_rx_overflow,
               (unsigned long)_tx_sent, (unsigned long)_tx_full,
               (unsigned long)_error_count);

    if (_dev != nullptr && _initialized) {
        enum can_state state;
        struct can_bus_err_cnt err_cnt;
        if (can_get_state(_dev, &state, &err_cnt) == 0) {
            const char *sname = "?";
            switch (state) {
            case CAN_STATE_ERROR_ACTIVE:  sname = "error-active";  break;
            case CAN_STATE_ERROR_WARNING: sname = "error-warning"; break;
            case CAN_STATE_ERROR_PASSIVE: sname = "error-passive"; break;
            case CAN_STATE_BUS_OFF:       sname = "bus-off";       break;
            case CAN_STATE_STOPPED:       sname = "stopped";       break;
            }
            str.printf("bus_state %s\ntx_err_cnt(TEC) %u\nrx_err_cnt(REC) %u\n",
                       sname, err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);
        }
    }
#endif
}

// ─── Zephyr RX ISR callback ───────────────────────────────────────────────

#ifdef __ZEPHYR__
void CANIface::rx_callback(const struct device * /*dev*/,
                            struct can_frame *zframe,
                            void *user_data)
{
    CANIface *self = static_cast<CANIface *>(user_data);

    CanRxItem rx_item;
    rx_item.timestamp_us = AP_HAL::micros64();
    rx_item.flags        = 0;

    if (!from_zephyr_frame(*zframe, rx_item.frame)) {
        return;
    }

    self->add_to_rx_queue(rx_item);
}

bool CANIface::to_zephyr_frame(const AP_HAL::CANFrame &in,
                                struct can_frame &out)
{
    memset(&out, 0, sizeof(out));

    if (in.isExtended()) {
        out.flags |= CAN_FRAME_IDE;
        out.id     = in.id & AP_HAL::CANFrame::MaskExtID;
    } else {
        out.id = in.id & AP_HAL::CANFrame::MaskStdID;
    }

    if (in.isRemoteTransmissionRequest()) {
        out.flags |= CAN_FRAME_RTR;
    }

    out.dlc = in.dlc <= 8U ? in.dlc : 8U;
    memcpy(out.data, in.data, out.dlc);
    return true;
}

bool CANIface::from_zephyr_frame(const struct can_frame &in,
                                  AP_HAL::CANFrame &out)
{
    if ((in.flags & CAN_FRAME_IDE) != 0) {
        out.id = (in.id & AP_HAL::CANFrame::MaskExtID) | AP_HAL::CANFrame::FlagEFF;
    } else {
        out.id = in.id & AP_HAL::CANFrame::MaskStdID;
    }

    if ((in.flags & CAN_FRAME_RTR) != 0) {
        out.id |= AP_HAL::CANFrame::FlagRTR;
    }

    uint8_t len = AP_HAL::CANFrame::dlcToDataLength(in.dlc);
    if (len > AP_HAL::CANFrame::MaxDataLen) {
        len = AP_HAL::CANFrame::MaxDataLen;
    }
    out.dlc   = in.dlc;
    out.canfd = false;
    memcpy(out.data, in.data, len);
    return true;
}
#endif  // __ZEPHYR__

#endif  // HAL_NUM_CAN_IFACES
#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
