/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <queue>
#include <vector>
#include <gtest/gtest.h>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/transport/frame.hpp>
#include <uavcan/driver/can.hpp>
#include <uavcan/driver/system_clock.hpp>
#include "../../clock.hpp"


class CanIfaceMock : public uavcan::ICanIface
{
public:
    struct FrameWithTime
    {
        uavcan::CanFrame frame;
        uavcan::MonotonicTime time;
        uavcan::UtcTime time_utc;
        uavcan::CanIOFlags flags;

        FrameWithTime(const uavcan::CanFrame& frame, uavcan::MonotonicTime time)
            : frame(frame)
            , time(time)
            , flags(0)
        { }

        FrameWithTime(const uavcan::CanFrame& frame, uavcan::MonotonicTime time, uavcan::UtcTime time_utc)
            : frame(frame)
            , time(time)
            , time_utc(time_utc)
            , flags(0)
        { }

        FrameWithTime(const uavcan::CanFrame& frame, uint64_t time_usec)
            : frame(frame)
            , time(uavcan::MonotonicTime::fromUSec(time_usec))
            , flags(0)
        { }
    };

    std::queue<FrameWithTime> tx;       ///< Queue of outgoing frames (bus <-- library)
    std::queue<FrameWithTime> rx;       ///< Queue of incoming frames (bus --> library)
    std::queue<FrameWithTime> loopback; ///< Loopback
    bool writeable;
    bool tx_failure;
    bool rx_failure;
    uint64_t num_errors;
    uavcan::ISystemClock& iclock;
    bool enable_utc_timestamping;
    uavcan::CanFrame pending_tx;

    CanIfaceMock(uavcan::ISystemClock& iclock)
        : writeable(true)
        , tx_failure(false)
        , rx_failure(false)
        , num_errors(0)
        , iclock(iclock)
        , enable_utc_timestamping(false)
    { }

    void pushRx(const uavcan::CanFrame& frame)
    {
        rx.push(FrameWithTime(frame, iclock.getMonotonic()));
    }

    void pushRx(const uavcan::RxFrame& frame)
    {
        uavcan::CanFrame can_frame;
        EXPECT_TRUE(frame.compile(can_frame));
        rx.push(FrameWithTime(can_frame, frame.getMonotonicTimestamp(), frame.getUtcTimestamp()));
    }

    bool matchAndPopTx(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline)
    {
        if (tx.empty())
        {
            std::cout << "Tx buffer is empty" << std::endl;
            return false;
        }
        const FrameWithTime frame_time = tx.front();
        tx.pop();
        return (frame_time.frame == frame) && (frame_time.time == tx_deadline);
    }

    bool matchPendingTx(const uavcan::CanFrame& frame) const
    {
        if (pending_tx != frame)
        {
            std::cout << "Pending TX mismatch: \n"
                      << "    Expected: " << frame.toString(uavcan::CanFrame::StrAligned) << "\n"
                      << "    Actual:   " << pending_tx.toString(uavcan::CanFrame::StrAligned) << std::endl;
        }
        return pending_tx == frame;
    }

    bool matchAndPopTx(const uavcan::CanFrame& frame, uint64_t tx_deadline_usec)
    {
        return matchAndPopTx(frame, uavcan::MonotonicTime::fromUSec(tx_deadline_usec));
    }

    uavcan::CanFrame popTxFrame()
    {
        if (tx.empty())
        {
            std::cout << "Tx buffer is empty" << std::endl;
            std::abort();
        }
        const FrameWithTime frame_time = tx.front();
        tx.pop();
        return frame_time.frame;
    }

    virtual uavcan::int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                                 uavcan::CanIOFlags flags)
    {
        assert(this);
        EXPECT_TRUE(writeable);        // Shall never be called when not writeable
        if (tx_failure)
        {
            return -1;
        }
        if (!writeable)
        {
            return 0;
        }
        tx.push(FrameWithTime(frame, tx_deadline));
        tx.back().flags = flags;
        if (flags & uavcan::CanIOFlagLoopback)
        {
            loopback.push(FrameWithTime(frame, iclock.getMonotonic()));
        }
        return 1;
    }

    virtual uavcan::int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                    uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
    {
        assert(this);
        if (loopback.empty())
        {
            EXPECT_TRUE(rx.size());        // Shall never be called when not readable
            if (rx_failure)
            {
                return -1;
            }
            if (rx.empty())
            {
                return 0;
            }
            const FrameWithTime frame = rx.front();
            rx.pop();
            out_frame = frame.frame;
            out_ts_monotonic = frame.time;
            out_ts_utc = frame.time_utc;
            out_flags = frame.flags;
        }
        else
        {
            out_flags |= uavcan::CanIOFlagLoopback;
            const FrameWithTime frame = loopback.front();
            loopback.pop();
            out_frame = frame.frame;
            out_ts_monotonic = frame.time;
            out_ts_utc = frame.time_utc;
        }

        // Let's just all pretend that this code is autogenerated, instead of being carefully designed by a human.
        if (out_ts_utc.isZero())
        {
            out_ts_utc = enable_utc_timestamping ? iclock.getUtc() : uavcan::UtcTime();
        }
        return 1;
    }

    // cppcheck-suppress unusedFunction
    // cppcheck-suppress functionConst
    virtual uavcan::int16_t configureFilters(const uavcan::CanFilterConfig*, uavcan::uint16_t) { return 0; }
    // cppcheck-suppress unusedFunction
    virtual uavcan::uint16_t getNumFilters() const { return 4; } // decrease number of HW_filters from 9 to 4
    virtual uavcan::uint64_t getErrorCount() const { return num_errors; }
};

class CanDriverMock : public uavcan::ICanDriver
{
public:
    std::vector<CanIfaceMock> ifaces;
    uavcan::ISystemClock& iclock;
    bool select_failure;

    CanDriverMock(unsigned num_ifaces, uavcan::ISystemClock& iclock)
        : ifaces(num_ifaces, CanIfaceMock(iclock))
        , iclock(iclock)
        , select_failure(false)
    { }

    void pushRxToAllIfaces(const uavcan::CanFrame& can_frame)
    {
        for (uint8_t i = 0; i < getNumIfaces(); i++)
        {
            ifaces.at(i).pushRx(can_frame);
        }
    }

    virtual uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks,
                                   const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces],
                                   uavcan::MonotonicTime deadline)
    {
        assert(this);
        //std::cout << "Write/read masks: " << inout_write_iface_mask << "/" << inout_read_iface_mask << std::endl;

        for (unsigned i = 0; i < ifaces.size(); i++)
        {
            ifaces.at(i).pending_tx = (pending_tx[i] == UAVCAN_NULLPTR) ? uavcan::CanFrame() : *pending_tx[i];
        }

        if (select_failure)
        {
            return -1;
        }

        const uavcan::uint8_t valid_iface_mask = uavcan::uint8_t((1 << getNumIfaces()) - 1);
        EXPECT_FALSE(inout_masks.write & ~valid_iface_mask);
        EXPECT_FALSE(inout_masks.read & ~valid_iface_mask);

        uavcan::uint8_t out_write_mask = 0;
        uavcan::uint8_t out_read_mask = 0;
        for (unsigned i = 0; i < getNumIfaces(); i++)
        {
            const uavcan::uint8_t mask = uavcan::uint8_t(1 << i);
            if ((inout_masks.write & mask) && ifaces.at(i).writeable)
            {
                out_write_mask |= mask;
            }
            if ((inout_masks.read & mask) && (ifaces.at(i).rx.size() || ifaces.at(i).loopback.size()))
            {
                out_read_mask |= mask;
            }
        }
        inout_masks.write = out_write_mask;
        inout_masks.read = out_read_mask;
        if ((out_write_mask | out_read_mask) == 0)
        {
            const uavcan::MonotonicTime ts = iclock.getMonotonic();
            const uavcan::MonotonicDuration diff = deadline - ts;
            SystemClockMock* const mock = dynamic_cast<SystemClockMock*>(&iclock);
            if (mock)
            {
                if (diff.isPositive())
                {
                    mock->advance(uint64_t(diff.toUSec()));   // Emulating timeout
                }
            }
            else
            {
                if (diff.isPositive())
                {
                    usleep(unsigned(diff.toUSec()));
                }
            }
            return 0;
        }
        return 1;  // This value is not being checked anyway, it just has to be greater than zero
    }

    virtual uavcan::ICanIface* getIface(uavcan::uint8_t iface_index) { return &ifaces.at(iface_index); }
    virtual const uavcan::ICanIface* getIface(uavcan::uint8_t iface_index) const { return &ifaces.at(iface_index); }
    virtual uavcan::uint8_t getNumIfaces() const { return uavcan::uint8_t(ifaces.size()); }
};

enum FrameType { STD, EXT };
inline uavcan::CanFrame makeCanFrame(uint32_t id, const std::string& str_data, FrameType type)
{
    id |= (type == EXT) ? uavcan::CanFrame::FlagEFF : 0;
    return uavcan::CanFrame(id, reinterpret_cast<const uint8_t*>(str_data.c_str()), uavcan::uint8_t(str_data.length()));
}
