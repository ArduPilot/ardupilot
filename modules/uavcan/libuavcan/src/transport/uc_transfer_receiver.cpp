/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/transfer_receiver.hpp>
#include <uavcan/transport/crc.hpp>
#include <uavcan/debug.hpp>
#include <cstdlib>
#include <cassert>

namespace uavcan
{

const uint16_t TransferReceiver::MinTransferIntervalMSec;
const uint16_t TransferReceiver::MaxTransferIntervalMSec;
const uint16_t TransferReceiver::DefaultTransferIntervalMSec;
const uint16_t TransferReceiver::DefaultTidTimeoutMSec;

MonotonicDuration TransferReceiver::getIfaceSwitchDelay() const
{
    return MonotonicDuration::fromMSec(transfer_interval_msec_);
}

MonotonicDuration TransferReceiver::getTidTimeout() const
{
    return MonotonicDuration::fromMSec(DefaultTidTimeoutMSec);
}

void TransferReceiver::registerError() const
{
    error_cnt_ = static_cast<uint8_t>(error_cnt_ + 1) & ErrorCntMask;
}

void TransferReceiver::updateTransferTimings()
{
    UAVCAN_ASSERT(!this_transfer_ts_.isZero());

    const MonotonicTime prev_prev_ts = prev_transfer_ts_;
    prev_transfer_ts_ = this_transfer_ts_;

    if ((!prev_prev_ts.isZero()) && (!prev_transfer_ts_.isZero()) && (prev_transfer_ts_ >= prev_prev_ts))
    {
        uint64_t interval_msec = uint64_t((prev_transfer_ts_ - prev_prev_ts).toMSec());
        interval_msec = min(interval_msec, uint64_t(MaxTransferIntervalMSec));
        interval_msec = max(interval_msec, uint64_t(MinTransferIntervalMSec));
        transfer_interval_msec_ = static_cast<uint16_t>((uint64_t(transfer_interval_msec_) * 7U + interval_msec) / 8U);
    }
}

void TransferReceiver::prepareForNextTransfer()
{
    tid_.increment();
    next_toggle_ = false;
    buffer_write_pos_ = 0;
}

bool TransferReceiver::validate(const RxFrame& frame) const
{
    if (iface_index_ != frame.getIfaceIndex())
    {
        return false;
    }
    if (frame.isStartOfTransfer() && !frame.isEndOfTransfer() && (frame.getPayloadLen() < TransferCRC::NumBytes))
    {
        UAVCAN_TRACE("TransferReceiver", "CRC expected, %s", frame.toString().c_str());
        registerError();
        return false;
    }
    if (frame.isStartOfTransfer() && frame.getToggle())
    {
        UAVCAN_TRACE("TransferReceiver", "Toggle bit is not cleared, %s", frame.toString().c_str());
        registerError();
        return false;
    }
    if (frame.isStartOfTransfer() && isMidTransfer())
    {
        UAVCAN_TRACE("TransferReceiver", "Unexpected start of transfer, %s", frame.toString().c_str());
        registerError();
    }
    if (frame.getToggle() != next_toggle_)
    {
        UAVCAN_TRACE("TransferReceiver", "Unexpected toggle bit (not %i), %s",
                     int(next_toggle_), frame.toString().c_str());
        registerError();
        return false;
    }
    if (frame.getTransferID() != tid_)
    {
        UAVCAN_TRACE("TransferReceiver", "Unexpected TID (current %i), %s", tid_.get(), frame.toString().c_str());
        registerError();
        return false;
    }
    return true;
}

bool TransferReceiver::writePayload(const RxFrame& frame, ITransferBuffer& buf)
{
    const uint8_t* const payload = frame.getPayloadPtr();
    const unsigned payload_len = frame.getPayloadLen();

    if (frame.isStartOfTransfer())     // First frame contains CRC, we need to extract it now
    {
        if (frame.getPayloadLen() < TransferCRC::NumBytes)
        {
            return false;    // Must have been validated earlier though. I think I'm paranoid.
        }
        this_transfer_crc_ = static_cast<uint16_t>(payload[0] & 0xFF);
        this_transfer_crc_ |= static_cast<uint16_t>(static_cast<uint16_t>(payload[1] & 0xFF) << 8);  // Little endian.

        const unsigned effective_payload_len = payload_len - TransferCRC::NumBytes;
        const int res = buf.write(buffer_write_pos_, payload + TransferCRC::NumBytes, effective_payload_len);
        const bool success = res == static_cast<int>(effective_payload_len);
        if (success)
        {
            buffer_write_pos_ = static_cast<uint16_t>(buffer_write_pos_ + effective_payload_len);
        }
        return success;
    }
    else
    {
        const int res = buf.write(buffer_write_pos_, payload, payload_len);
        const bool success = res == static_cast<int>(payload_len);
        if (success)
        {
            buffer_write_pos_ = static_cast<uint16_t>(buffer_write_pos_ + payload_len);
        }
        return success;
    }
}

TransferReceiver::ResultCode TransferReceiver::receive(const RxFrame& frame, TransferBufferAccessor& tba)
{
    // Transfer timestamps are derived from the first frame
    if (frame.isStartOfTransfer())
    {
        this_transfer_ts_ = frame.getMonotonicTimestamp();
        first_frame_ts_   = frame.getUtcTimestamp();
    }

    if (frame.isStartOfTransfer() && frame.isEndOfTransfer())
    {
        tba.remove();
        updateTransferTimings();
        prepareForNextTransfer();
        this_transfer_crc_ = 0;         // SFT has no CRC
        return ResultSingleFrame;
    }

    // Payload write
    ITransferBuffer* buf = tba.access();
    if (buf == UAVCAN_NULLPTR)
    {
        buf = tba.create();
    }
    if (buf == UAVCAN_NULLPTR)
    {
        UAVCAN_TRACE("TransferReceiver", "Failed to access the buffer, %s", frame.toString().c_str());
        prepareForNextTransfer();
        registerError();
        return ResultNotComplete;
    }
    if (!writePayload(frame, *buf))
    {
        UAVCAN_TRACE("TransferReceiver", "Payload write failed, %s", frame.toString().c_str());
        tba.remove();
        prepareForNextTransfer();
        registerError();
        return ResultNotComplete;
    }
    next_toggle_ = !next_toggle_;

    if (frame.isEndOfTransfer())
    {
        updateTransferTimings();
        prepareForNextTransfer();
        return ResultComplete;
    }
    return ResultNotComplete;
}

bool TransferReceiver::isTimedOut(MonotonicTime current_ts) const
{
    return (current_ts - this_transfer_ts_) > getTidTimeout();
}

TransferReceiver::ResultCode TransferReceiver::addFrame(const RxFrame& frame, TransferBufferAccessor& tba)
{
    if ((frame.getMonotonicTimestamp().isZero()) ||
        (frame.getMonotonicTimestamp() < prev_transfer_ts_) ||
        (frame.getMonotonicTimestamp() < this_transfer_ts_))
    {
        UAVCAN_TRACE("TransferReceiver", "Invalid frame, %s", frame.toString().c_str());
        return ResultNotComplete;
    }

    const bool not_initialized = !isInitialized();
    const bool tid_timed_out = isTimedOut(frame.getMonotonicTimestamp());
    const bool same_iface = frame.getIfaceIndex() == iface_index_;
    const bool first_frame = frame.isStartOfTransfer();
    const bool non_wrapped_tid = tid_.computeForwardDistance(frame.getTransferID()) < TransferID::Half;
    const bool not_previous_tid = frame.getTransferID().computeForwardDistance(tid_) > 1;
    const bool iface_switch_allowed = (frame.getMonotonicTimestamp() - this_transfer_ts_) > getIfaceSwitchDelay();

    // FSM, the hard way
    const bool need_restart =
        (not_initialized) ||
        (tid_timed_out) ||
        (same_iface && first_frame && not_previous_tid) ||
        (iface_switch_allowed && first_frame && non_wrapped_tid);

    if (need_restart)
    {
        if (!not_initialized && (tid_ != frame.getTransferID()))
        {
            registerError();
        }
        UAVCAN_TRACE("TransferReceiver", "Restart [ni=%d, isa=%d, tt=%d, si=%d, ff=%d, nwtid=%d, nptid=%d, tid=%d], %s",
                     int(not_initialized), int(iface_switch_allowed), int(tid_timed_out), int(same_iface),
                     int(first_frame), int(non_wrapped_tid), int(not_previous_tid), int(tid_.get()),
                     frame.toString().c_str());
        tba.remove();
        iface_index_ = frame.getIfaceIndex() & IfaceIndexMask;
        tid_ = frame.getTransferID();
        next_toggle_ = false;
        buffer_write_pos_ = 0;
        this_transfer_crc_ = 0;
        if (!first_frame)
        {
            tid_.increment();
            return ResultNotComplete;
        }
    }

    if (!validate(frame))
    {
        return ResultNotComplete;
    }
    return receive(frame, tba);
}

uint8_t TransferReceiver::yieldErrorCount()
{
    const uint8_t ret = error_cnt_;
    error_cnt_ = 0;
    return ret;
}

}
